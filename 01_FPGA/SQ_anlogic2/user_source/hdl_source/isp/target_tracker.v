// ===========================================================================
// target_tracker.v   （位置关联版 —— 升级"跟踪"的含金量）
//
// 扩-2 + 进-2 结合：对"颜色目标 / 车牌 / 交通灯"做跨帧关联，输出稳定 ID。
//
// ★ 本次升级：判"是不是同一个目标"时，除了比【类型】，还要比【位置】。
//    —— 上一版只用类型：把红牌拿走、换另一个红色物体进来，ID 不变（名不副实）
//    —— 本版用"类型 + 位置靠近"（最近邻数据关联）：
//         新框中心与上一帧中心距离 ≤ MAX_DX/MAX_DY → 判为同一目标，ID 保持
//         距离太远或类型变了            → 判为新目标，分配新 ID
//    这样换物体时 ID 会正确更新，才叫真正的"跟踪"。
//
// 三个候选源（优先级：交通灯 > 车牌 > 颜色目标）：
//   槽0 = 优先级最高的那个目标；槽1 = 次高（最多两个同时跟踪）
//   交通灯点亮时抑制"红色物体"候选，避免同一盏红灯被算成两个目标
//
// 类型编码 O_*_type：0红物 1蓝牌 2黄牌 3绿牌 4红灯 5黄灯 6绿灯
//
// 时钟域：axi4s_video_aclk
// ===========================================================================

`timescale 1ns/1ps

module target_tracker #(
    parameter MISS_MAX = 6'd40,
    // 位置关联阈值（像素）：新框中心与上一帧中心在这范围内就认为还是同一个目标。
    // 60fps 下手持目标每帧位移很小；160 已经足够宽松，不会误判成新目标。
    parameter [11:0] MAX_DX = 12'd160,
    parameter [11:0] MAX_DY = 12'd160
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tuser,

    input  wire        I_box_valid,
    input  wire [11:0] I_box_x0,
    input  wire [11:0] I_box_y0,
    input  wire [11:0] I_box_x1,
    input  wire [11:0] I_box_y1,

    input  wire        I_plate_valid,
    input  wire [11:0] I_plate_x0,
    input  wire [11:0] I_plate_y0,
    input  wire [11:0] I_plate_x1,
    input  wire [11:0] I_plate_y1,
    input  wire [1:0]  I_plate_type,

    input  wire        I_light_valid,
    input  wire [1:0]  I_light_color,
    input  wire [11:0] I_light_x0,
    input  wire [11:0] I_light_y0,
    input  wire [11:0] I_light_x1,
    input  wire [11:0] I_light_y1,

    output reg         O_t0_valid,
    output reg [1:0]   O_t0_id,
    output reg [2:0]   O_t0_type,
    output reg [11:0]  O_t0_x0,
    output reg [11:0]  O_t0_y0,
    output reg [11:0]  O_t0_x1,
    output reg [11:0]  O_t0_y1,

    output reg         O_t1_valid,
    output reg [1:0]   O_t1_id,
    output reg [2:0]   O_t1_type,
    output reg [11:0]  O_t1_x0,
    output reg [11:0]  O_t1_y0,
    output reg [11:0]  O_t1_x1,
    output reg [11:0]  O_t1_y1
);

    reg [1:0] vld;
    reg [1:0] id0, id1;
    reg [2:0] typ0, typ1;
    reg [5:0] miss0, miss1;
    reg [1:0] next_id;

    // 上一帧两个槽的中心位置（用于位置关联）
    reg [11:0] cx0_q, cy0_q;
    reg [11:0] cx1_q, cy1_q;

    // 颜色目标：交通灯点亮时抑制
    wire color_c_valid = I_box_valid && !I_light_valid;

    // ---- 槽0 候选（灯 > 牌 > 颜色）----
    wire       s0_valid = I_light_valid ? 1'b1 : (I_plate_valid ? 1'b1 : color_c_valid);
    wire [2:0] s0_type  = I_light_valid ? ({1'b0, I_light_color} + 3'd4) :
                           (I_plate_valid ? ({1'b0, I_plate_type} + 3'd1) : 3'd0);
    wire [11:0] s0_x0   = I_light_valid ? I_light_x0 : (I_plate_valid ? I_plate_x0 : I_box_x0);
    wire [11:0] s0_y0   = I_light_valid ? I_light_y0 : (I_plate_valid ? I_plate_y0 : I_box_y0);
    wire [11:0] s0_x1   = I_light_valid ? I_light_x1 : (I_plate_valid ? I_plate_x1 : I_box_x1);
    wire [11:0] s0_y1   = I_light_valid ? I_light_y1 : (I_plate_valid ? I_plate_y1 : I_box_y1);

    // ---- 槽1 候选 ----
    wire       s1_valid = (I_light_valid && I_plate_valid) || (I_plate_valid && color_c_valid);
    wire [2:0] s1_type  = (I_light_valid && I_plate_valid) ? ({1'b0, I_plate_type} + 3'd1) : 3'd0;
    wire [11:0] s1_x0   = (I_light_valid && I_plate_valid) ? I_plate_x0 : I_box_x0;
    wire [11:0] s1_y0   = (I_light_valid && I_plate_valid) ? I_plate_y0 : I_box_y0;
    wire [11:0] s1_x1   = (I_light_valid && I_plate_valid) ? I_plate_x1 : I_box_x1;
    wire [11:0] s1_y1   = (I_light_valid && I_plate_valid) ? I_plate_y1 : I_box_y1;

    // ---- 本帧两个候选的中心 ----
    wire [11:0] s0_cx = (s0_x0 + s0_x1) >> 1;
    wire [11:0] s0_cy = (s0_y0 + s0_y1) >> 1;
    wire [11:0] s1_cx = (s1_x0 + s1_x1) >> 1;
    wire [11:0] s1_cy = (s1_y0 + s1_y1) >> 1;

    // ---- 与上一帧中心的位置差（取绝对值）----
    wire [11:0] dx0 = (s0_cx > cx0_q) ? (s0_cx - cx0_q) : (cx0_q - s0_cx);
    wire [11:0] dy0 = (s0_cy > cy0_q) ? (s0_cy - cy0_q) : (cy0_q - s0_cy);
    wire [11:0] dx1 = (s1_cx > cx1_q) ? (s1_cx - cx1_q) : (cx1_q - s1_cx);
    wire [11:0] dy1 = (s1_cy > cy1_q) ? (s1_cy - cy1_q) : (cy1_q - s1_cy);

    wire near0 = (dx0 <= MAX_DX) && (dy0 <= MAX_DY);
    wire near1 = (dx1 <= MAX_DX) && (dy1 <= MAX_DY);

    // ---- 关联判定：类型相同 **且** 位置靠近 → 认为是同一个目标 ----
    wire s0_same = vld[0] && (typ0 == s0_type) && near0;
    wire s1_same = vld[1] && (typ1 == s1_type) && near1;

    wire s0_new = s0_valid && !s0_same;
    wire s1_new = s1_valid && !s1_same;

    wire [1:0] s0_new_id = next_id;
    wire [1:0] s1_new_id = next_id + {1'b0, s0_new};
    wire [1:0] id_inc    = {1'b0, s0_new} + {1'b0, s1_new};

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            vld      <= 2'd0;
            next_id  <= 2'd0;
            id0      <= 2'd0;
            id1      <= 2'd0;
            typ0     <= 3'd0;
            typ1     <= 3'd0;
            miss0    <= 6'd0;
            miss1    <= 6'd0;
            cx0_q    <= 12'd0; cy0_q <= 12'd0;
            cx1_q    <= 12'd0; cy1_q <= 12'd0;
            O_t0_valid <= 1'b0;
            O_t0_id    <= 2'd0;
            O_t0_type  <= 3'd0;
            O_t0_x0    <= 12'd0;
            O_t0_y0    <= 12'd0;
            O_t0_x1    <= 12'd0;
            O_t0_y1    <= 12'd0;
            O_t1_valid <= 1'b0;
            O_t1_id    <= 2'd0;
            O_t1_type  <= 3'd0;
            O_t1_x0    <= 12'd0;
            O_t1_y0    <= 12'd0;
            O_t1_x1    <= 12'd0;
            O_t1_y1    <= 12'd0;
        end
        else if (I_tuser) begin
            // ---- 槽0 ----
            if (s0_valid) begin
                if (s0_new) id0 <= s0_new_id;
                vld[0] <= 1'b1;
                typ0   <= s0_type;
                miss0  <= 6'd0;
                cx0_q  <= s0_cx;          // 记住这一帧的位置，供下一帧关联
                cy0_q  <= s0_cy;
                O_t0_valid <= 1'b1;
                O_t0_id    <= s0_new ? s0_new_id : id0;
                O_t0_type  <= s0_type;
                O_t0_x0    <= s0_x0;
                O_t0_y0    <= s0_y0;
                O_t0_x1    <= s0_x1;
                O_t0_y1    <= s0_y1;
            end
            else begin
                O_t0_valid <= 1'b0;
                if (vld[0]) begin
                    if (miss0 < MISS_MAX) miss0 <= miss0 + 6'd1;
                    else                  vld[0] <= 1'b0;
                end
            end

            // ---- 槽1 ----
            if (s1_valid) begin
                if (s1_new) id1 <= s1_new_id;
                vld[1] <= 1'b1;
                typ1   <= s1_type;
                miss1  <= 6'd0;
                cx1_q  <= s1_cx;
                cy1_q  <= s1_cy;
                O_t1_valid <= 1'b1;
                O_t1_id    <= s1_new ? s1_new_id : id1;
                O_t1_type  <= s1_type;
                O_t1_x0    <= s1_x0;
                O_t1_y0    <= s1_y0;
                O_t1_x1    <= s1_x1;
                O_t1_y1    <= s1_y1;
            end
            else begin
                O_t1_valid <= 1'b0;
                if (vld[1]) begin
                    if (miss1 < MISS_MAX) miss1 <= miss1 + 6'd1;
                    else                  vld[1] <= 1'b0;
                end
            end

            next_id <= next_id + id_inc;
        end
    end

endmodule
