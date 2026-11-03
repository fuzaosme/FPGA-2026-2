// ===========================================================================
// motion_diff.v  —— 进-4：帧差法运动目标检测（块级）
//
// 原理（大白话）：
//   把这一帧和上一帧逐"块"比。静止的东西块的平均亮度几乎不变，动过的地方
//   亮度差得大。所以它能回答"画面里哪儿动了"——不管动的东西是什么颜色，
//   正好和颜色检测互补。
//
// 为什么按"块"而不按"像素"：
//   逐像素帧差要存整帧（1280×720×8bit ≈ 7.4Mbit），ERAM 装不下；
//   按 16×16 的块，每块只存一个平均亮度（3600 块 × 8bit ≈ 29Kbit），
//   2 个 ERAM 就够。
//
// 尺寸选择：1280/16 = 80 列、720/16 = 45 行 —— 都整除；
//           一个块 256 像素是 2 的幂，求平均只要 >>8（不用除法器）。
//
// 数据流：与 isp 链路同一节拍（96bit = 4 像素/拍，axi4s_video_aclk）
//   每拍算 4 个像素的亮度和 lum4，累加到"当前块列"的累加器。
//   一行 320 拍 = 80 个块列（每块列 4 拍 = 16 像素）。
//   在"线 0 拍 0"发起收获：把上一块行的累加值 >>8 得到块平均，
//   与上一帧同位置比较 → 判定运动 → 累加器清零，开始累新块行。
//   ★ 收获必须连 tuser 那一拍也发起，否则最后一条块行的累加器永远不复位。
//
// 输出：运动区域包围盒（像素坐标）+ 运动块数，帧首（tuser）锁存整帧结果。
//
// 已知取舍：
//   · 块行 NROW-1（最下面 16 行）的收获发生在下一帧开头，所以 tuser 锁存
//     的那份结果少算最后一条块行 —— 影响不到 2% 的画面，演示无感。
//   · 上电第 1 帧 prev_mem 内容无效，用 warm_up 屏蔽掉。
// ===========================================================================

`timescale 1ns/1ps

module motion_diff #(
    parameter IMG_WIDTH  = 1280,
    parameter IMG_HEIGHT = 720,
    parameter [7:0]  DIFF_THR   = 8'd20,   // 块平均亮度差超过它 → 这块动了
    parameter [15:0] MIN_BLOCKS = 16'd3    // 运动块数下限，低于它视为噪声
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tvalid,
    input  wire        I_tuser,
    input  wire        I_tlast,
    input  wire [95:0] I_tdata,

    output reg         O_motion_valid,
    output reg  [11:0] O_motion_x0,
    output reg  [11:0] O_motion_y0,
    output reg  [11:0] O_motion_x1,
    output reg  [11:0] O_motion_y1,
    output reg  [15:0] O_motion_cnt,
    // ---- 块行运动活动向量（给区域引擎用）----
    output reg         O_row_v,
    output reg  [79:0] O_row_act,
    output reg         O_frame_last
);

    localparam NCOL = IMG_WIDTH  / 16;    // 80
    localparam NROW = IMG_HEIGHT / 16;    // 45
    localparam integer HISTORY_AW = (NCOL*NROW>1) ? $clog2(NCOL*NROW) : 1;

    // ---- 像素拆分（96bit = 4 × RGB888，p0 是最左列）----
    wire [23:0] p0 = I_tdata[95:72];
    wire [23:0] p1 = I_tdata[71:48];
    wire [23:0] p2 = I_tdata[47:24];
    wire [23:0] p3 = I_tdata[23: 0];

    // 先按通道求和再套亮度公式：Y=(R+2G+B)/4，只用加法
    wire [9:0]  rsum = p0[23:16] + p1[23:16] + p2[23:16] + p3[23:16];
    wire [9:0]  gsum = p0[15: 8] + p1[15: 8] + p2[15: 8] + p3[15: 8];
    wire [9:0]  bsum = p0[ 7: 0] + p1[ 7: 0] + p2[ 7: 0] + p3[ 7: 0];
    wire [11:0] lum4 = (rsum + {gsum, 1'b0} + bsum) >> 2;   // 0..1020
    wire [11:0] lum4_z = {2'd0, lum4[9:0]};

    // ---- 当前块行的累加器：80 × 16bit（综合成分布式 RAM）----
    reg [15:0] acc [0:NCOL-1];

    // ---- 上一帧的块平均：NCOL*NROW × 8bit（综合成 ERAM）----
    // Explicit ERAM below preserves the existing independent read/write ports.

    reg [1:0]  beat;      // 块内第几拍（0..3）
    reg [6:0]  col;       // 块列（0..79）
    reg [4:0]  line;      // 块行内第几行（0..15）
    reg [5:0]  brow;      // 块行号（0..44）

    // ---- 收获流水线 ----
    //   ★ prev_mem 是 ERAM，读有 1 拍延迟：第 T 拍发起读，第 T+1 拍地址才进内存，
    //     数据第 T+2 拍才出来。所以判定必须在"发起后第 2 拍"做，
    //     否则会拿到上一个地址的数据（曾因此误报一列）。
    reg [1:0]  pend;          // 01=刚发起, 10=可以判定了
    reg [7:0]  harv_avg;      // 上一块行的块平均
    reg [6:0]  harv_c;
    reg [5:0]  harv_r;
    reg [12:0] harv_addr;

    reg        prev_we;
    reg [12:0] prev_waddr;
    reg [7:0]  prev_wdata;
    reg [12:0] prev_raddr;
    wire [7:0] prev_rd;       // ERAM 读数据（比地址晚 1 拍）

    // 上电后前 2 帧屏蔽"运动判定"：
    //   第 1 帧 prev_mem 全是垃圾，写进去会产生一次错误基准；
    //   所以要从第 3 帧开始判才可靠。
    reg [1:0]  frame_cnt;
    wire       warm = (frame_cnt < 2'd2);

    reg [6:0]  mo_min_c, mo_max_c;
    reg        mov_bit;
    reg [79:0] motion_row_bits;
    reg        row_v_d;
    reg [5:0]  mo_min_r, mo_max_r;
    reg [15:0] mo_cnt;

    // 收获处理的是"上一个块行"
    wire [5:0]  brow_prev = (brow == 6'd0) ? (NROW-1) : (brow - 6'd1);
    wire [12:0] addr_now  = brow_prev * NCOL + col;
    wire        at_harv   = I_tvalid && (line == 5'd0) && (beat == 2'd0);

    integer i;

    // ---- ERAM：写同步、读带 1 拍延迟 ----
    // TD cannot infer this RAM through the asynchronously-reset read address.
    // NOREG is a single synchronous RAM read, identical to the former prev_rd.
    // Contents and output are not reset; existing two-frame warmup is retained.
    PH1P_LOGIC_ERAM #(
        .DATA_WIDTH_A(8),.DATA_WIDTH_B(8),
        .ADDR_WIDTH_A(HISTORY_AW),.ADDR_WIDTH_B(HISTORY_AW),
        .DATA_DEPTH_A(2**HISTORY_AW),.DATA_DEPTH_B(2**HISTORY_AW),
        .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
        .WRITEMODE_A("READBEFOREWRITE"),.WRITEMODE_B("READBEFOREWRITE"),
        .IMPLEMENT("20K(FAST)"),.CLKMODE("SYNC"),
        .RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC")
    ) u_history_eram (
        .dia(prev_wdata),.dib(8'd0),
        .addra(prev_waddr[HISTORY_AW-1:0]),.addrb(prev_raddr[HISTORY_AW-1:0]),
        .clka(I_clk),.clkb(I_clk),.cea(1'b1),.ceb(1'b1),
        .ocea(1'b0),.oceb(1'b0),.wea(prev_we),.web(1'b0),
        .bea(1'b0),.beb(1'b0),.rsta(1'b0),.rstb(1'b0),
        .doa(),.dob(prev_rd),.ecc_sbiterr(),.ecc_dbiterr(),
        .ecc_sbiterrinj(32'd0),.ecc_dbiterrinj(32'd0)
    );

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            beat      <= 2'd0;
            col       <= 7'd0;
            line      <= 5'd0;
            brow      <= 6'd0;
            pend      <= 2'd0;
            harv_avg  <= 8'd0;
            harv_c    <= 7'd0;
            harv_r    <= 6'd0;
            harv_addr <= 13'd0;
            prev_we   <= 1'b0;
            prev_waddr<= 13'd0;
            prev_wdata<= 8'd0;
            prev_raddr<= 13'd0;
            frame_cnt <= 2'd0;
            O_motion_valid <= 1'b0;
            O_motion_x0 <= 12'd0; O_motion_y0 <= 12'd0;
            O_motion_x1 <= 12'd0; O_motion_y1 <= 12'd0;
            O_motion_cnt<= 16'd0;
            mo_min_c <= 7'h7F; mo_max_c <= 7'd0;
            mo_min_r <= 6'h3F; mo_max_r <= 6'd0;
            mo_cnt   <= 16'd0;
            mov_bit  <= 1'b0;
            motion_row_bits <= 80'd0;
            row_v_d  <= 1'b0;
            O_row_v  <= 1'b0;
            O_row_act<= 80'd0;
            O_frame_last <= 1'b0;
            for (i = 0; i < NCOL; i = i + 1) acc[i] <= 16'd0;
        end
        else begin
            // ---------- (1) 收获判定：用"发起后第 2 拍"才有效的 prev 值 ----------
            prev_we <= 1'b0;
            if (pend[1]) begin
                mov_bit = (!warm) &&
                          ((harv_avg > prev_rd ? (harv_avg - prev_rd)
                                               : (prev_rd - harv_avg)) > DIFF_THR);
                motion_row_bits[harv_c] <= mov_bit;   // 逐块收集成本行向量
                if (mov_bit) begin
                    mo_cnt <= mo_cnt + 16'd1;
                    if (harv_c < mo_min_c) mo_min_c <= harv_c;
                    if (harv_c > mo_max_c) mo_max_c <= harv_c;
                    if (harv_r < mo_min_r) mo_min_r <= harv_r;
                    if (harv_r > mo_max_r) mo_max_r <= harv_r;
                end
                prev_we    <= 1'b1;            // 把本帧块平均存回去
                prev_waddr <= harv_addr;
                prev_wdata <= harv_avg;
            end

            // ---------- (2) 发起收获（★ tuser 拍也要发起）----------
            if (at_harv) begin
                harv_avg  <= acc[col][15:8];
                harv_c    <= col;
                harv_r    <= brow_prev;
                harv_addr <= addr_now;
                prev_raddr<= addr_now;
                acc[col]  <= {4'd0, lum4_z};   // 新块行从这一拍开始累
            end
            else if (I_tvalid && !I_tuser) begin
                acc[col] <= acc[col] + {4'd0, lum4_z};
            end

            // 收获流水线移位：发起那拍置 01，之后每拍左移
            pend <= at_harv ? 2'b01 : {pend[0], 1'b0};

            // ---- 本行向量输出：最后一块列的判决落地后再推一拍 ----
            row_v_d <= pend[1] && (harv_c == NCOL-1);
            O_row_v     <= row_v_d;
            O_row_act   <= motion_row_bits;
            O_frame_last<= row_v_d && (harv_r == NROW-1);

            // ---------- (3) 帧首锁存 / 计数器推进 ----------
            if (I_tuser) begin
                if (!warm && (mo_cnt >= MIN_BLOCKS)) begin
                    O_motion_valid <= 1'b1;
                    O_motion_x0 <= mo_min_c * 12'd16;
                    O_motion_y0 <= mo_min_r * 12'd16;
                    O_motion_x1 <= (mo_max_c + 7'd1) * 12'd16 - 12'd1;
                    O_motion_y1 <= (mo_max_r + 6'd1) * 12'd16 - 12'd1;
                end
                else begin
                    O_motion_valid <= 1'b0;
                end
                O_motion_cnt <= mo_cnt;
                mo_min_c <= 7'h7F; mo_max_c <= 7'd0;
                mo_min_r <= 6'h3F; mo_max_r <= 6'd0;
                mo_cnt   <= 16'd0;
                if (frame_cnt != 2'd3) frame_cnt <= frame_cnt + 2'd1;
                beat     <= 2'd1;              // tuser 这一拍被消耗，下一拍从 beat1 起
                col      <= 7'd0;
                line     <= 5'd0;
                brow     <= 6'd0;
            end
            else if (I_tvalid) begin
                if (beat == 2'd3) begin
                    beat <= 2'd0;
                    col  <= (col == NCOL-1) ? 7'd0 : (col + 7'd1);
                end
                else begin
                    beat <= beat + 2'd1;
                end

                if (I_tlast) begin
                    if (line == 5'd15) begin
                        line    <= 5'd0;
                        brow    <= (brow == NROW-1) ? 6'd0 : (brow + 6'd1);
                    end
                    else begin
                        line <= line + 5'd1;
                    end
                end
            end
        end
    end

endmodule


// ===========================================================================
// morph_open_3x3 —— 块级 3x3 开运算（先腐蚀后膨胀）
//
// 输入格式和 blk_act_gen 的输出一致：每个块行给一个 80 位活动向量。
// 输出行序不变，只是整体晚两个块行。
//
// 为什么用开运算：孤立的单块噪点会被腐蚀掉，而真实目标只是被磨掉一圈，
// 紧接着的膨胀又把它补回来 —— 目标尺寸基本不变，噪点被清掉。
// 只腐蚀会把目标越磨越小，只膨胀会把噪点连成片，都不合适。
//
// 省资源的做法：把 9 个邻居拆成"水平 3 邻域 + 垂直 3 行组合"。
//   水平 3 邻域是纯按位运算，一行 80 个查找表；
//   垂直方向只需要存"上面两行的水平结果"，不必存原始行。
//   整条链一拍一行，不用逐位扫描的状态机。
//
// 边界：水平两端按 0 补齐（画面外当作"没动"）。
// 帧末：最后两行的 3x3 窗口需要"下一行"，所以在 I_frame_last 之后自动
//       补两个全 0 的空行走完流水，这样第 45 行也有完整窗口。
//
// 节拍（以行事件序号 j 计，j 从 0 开始）：
//   输入行 k 到来时，腐蚀器输出的是第 k-1 行的腐蚀结果，
//   膨胀器输出的则是第 k-2 行的膨胀结果 —— 也就是本模块的输出。
//   所以输出行号 = j - 2，j 取 2..46 时依次吐出第 0..44 行。
// ===========================================================================
module morph_open_3x3 #(
    parameter integer NCOL = 80,
    parameter integer NROW = 45
)(
    input  wire               I_clk,
    input  wire               I_rst_n,
    input  wire               I_row_v,
    input  wire [NCOL-1:0]    I_row_act,
    input  wire               I_frame_last,
    output reg                O_row_v,
    output reg  [NCOL-1:0]    O_row_act,
    output reg                O_frame_last
);
    reg [6:0]      ev_cnt;     // 行事件序号 j
    reg [1:0]      nflush;     // 帧末还要补几个空行事件
    reg            ev_is_last; // 本次行事件是本帧最后一个（补空行的最后一拍）
    reg            ev;         // 本拍有行事件
    reg [NCOL-1:0] ev_x;       // 本事件的输入行

    reg [NCOL-1:0] eh1, eh2;   // 腐蚀器：上面两行的"水平 3 邻域与"
    reg [NCOL-1:0] dh1, dh2;   // 膨胀器：上面两行的"水平 3 邻域或"

    wire [NCOL-1:0] h_and   = ev_x & {ev_x[NCOL-2:0], 1'b0} & {1'b0, ev_x[NCOL-1:1]};
    wire [NCOL-1:0] eroded  = eh1 & eh2 & h_and;
    wire [NCOL-1:0] h_or    = eroded | {eroded[NCOL-2:0], 1'b0} | {1'b0, eroded[NCOL-1:1]};
    wire [NCOL-1:0] dilated = dh1 | dh2 | h_or;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            ev_cnt <= 7'd0;
            nflush <= 2'd0;
            ev_is_last <= 1'b0;
            ev     <= 1'b0;
            ev_x   <= {NCOL{1'b0}};
            eh1 <= {NCOL{1'b0}}; eh2 <= {NCOL{1'b0}};
            dh1 <= {NCOL{1'b0}}; dh2 <= {NCOL{1'b0}};
            O_row_v <= 1'b0;
            O_row_act <= {NCOL{1'b0}};
            O_frame_last <= 1'b0;
        end
        else begin
            O_row_v <= 1'b0;
            O_frame_last <= 1'b0;

            // ---------- 行事件：真实行优先，其次补空行 ----------
            if (I_row_v) begin
                ev   <= 1'b1;
                ev_x <= I_row_act;
                ev_is_last <= 1'b0;
                if (I_frame_last) nflush <= 2'd2;
            end
            else if (nflush != 2'd0) begin
                ev   <= 1'b1;
                ev_x <= {NCOL{1'b0}};
                nflush <= nflush - 2'd1;
                // 本次就是最后一个补空行（下一拍 ev_is_last 生效）
                ev_is_last <= (nflush == 2'd1);
            end
            else begin
                ev <= 1'b0;
                ev_is_last <= 1'b0;
            end

            // ---------- 两级流水 + 发射 ----------
            if (ev) begin
                eh2 <= eh1; eh1 <= h_and;
                dh2 <= dh1; dh1 <= h_or;
                if ((ev_cnt >= 7'd2) && (ev_cnt <= NROW + 1)) begin
                    O_row_v   <= 1'b1;
                    O_row_act <= dilated;
                    if (ev_cnt == NROW + 1) O_frame_last <= 1'b1;
                end
                // ★ 只在"最后一个补空行"那一拍清零。
                //   之前写的是判断 nflush==1，但 nflush 在"排程那一拍"就已经减过了，
                //   结果提前一拍清零，最后一行永远发不出来（被测试台抓到了）。
                ev_cnt <= ev_is_last ? 7'd0 : (ev_cnt + 7'd1);
            end
        end
    end
endmodule
