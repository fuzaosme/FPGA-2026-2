// ===========================================================================
// plate_detect.v
// 扩-2 车牌识别核心：
//   输入 96bit(4像素/拍) RGB 流，同时检测 蓝 / 黄 / 绿 三种颜色，
//   各自统计包围盒，再用"长宽比≈3.1"筛选车牌候选，取面积最大的一个输出。
//
// 输出：
//   O_plate_valid        是否检测到车牌
//   O_plate_x0/y0/x1/y1  车牌包围盒
//   O_plate_type         0=蓝牌 1=黄牌 2=绿牌
//
// 长宽比判定不采用除法：w*100 与 阈值*h 比较即可。
// ===========================================================================

`timescale 1ns/1ps

module plate_detect #(
    parameter IMG_WIDTH  = 1280,
    parameter IMG_HEIGHT = 720,
    parameter [19:0] MIN_PIX   = 20'd80,     // 车牌至少多少像素
    parameter [15:0] ASPECT_MIN = 16'd230,   // 2.30（放大 100 倍）
    parameter [15:0] ASPECT_MAX = 16'd400    // 4.00（放大 100 倍）
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tvalid,
    input  wire        I_tuser,
    input  wire        I_tlast,
    input  wire [95:0] I_tdata,
    output wire        O_plate_valid,
    output wire [11:0] O_plate_x0,
    output wire [11:0] O_plate_y0,
    output wire [11:0] O_plate_x1,
    output wire [11:0] O_plate_y1,
    output wire [1:0]  O_plate_type
);

    // ---- 三色掩码 ----
    wire [3:0] m_b, m_y, m_g;
    color_mask_96 u_cm_b (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_color_sel(3'd1),
        .I_tvalid(I_tvalid), .I_tdata(I_tdata), .O_mask(m_b));
    color_mask_96 u_cm_y (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_color_sel(3'd2),
        .I_tvalid(I_tvalid), .I_tdata(I_tdata), .O_mask(m_y));
    color_mask_96 u_cm_g (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_color_sel(3'd3),
        .I_tvalid(I_tvalid), .I_tdata(I_tdata), .O_mask(m_g));

    // ---- 三色包围盒 ----
    wire        v_b, v_y, v_g;
    wire [11:0] bx0, by0, bx1, by1;
    wire [11:0] yx0, yy0, yx1, yy1;
    wire [11:0] gx0, gy0, gx1, gy1;
    wire [19:0] b_cnt, y_cnt, g_cnt;

    blob_stat #(.IMG_WIDTH(IMG_WIDTH), .IMG_HEIGHT(IMG_HEIGHT), .MIN_PIX(MIN_PIX))
    u_bs_b (.I_clk(I_clk), .I_rst_n(I_rst_n), .I_tvalid(I_tvalid), .I_tuser(I_tuser), .I_tlast(I_tlast),
            .I_mask(m_b), .O_box_valid(v_b), .O_x0(bx0), .O_y0(by0), .O_x1(bx1), .O_y1(by1), .O_cnt(b_cnt));
    blob_stat #(.IMG_WIDTH(IMG_WIDTH), .IMG_HEIGHT(IMG_HEIGHT), .MIN_PIX(MIN_PIX))
    u_bs_y (.I_clk(I_clk), .I_rst_n(I_rst_n), .I_tvalid(I_tvalid), .I_tuser(I_tuser), .I_tlast(I_tlast),
            .I_mask(m_y), .O_box_valid(v_y), .O_x0(yx0), .O_y0(yy0), .O_x1(yx1), .O_y1(yy1), .O_cnt(y_cnt));
    blob_stat #(.IMG_WIDTH(IMG_WIDTH), .IMG_HEIGHT(IMG_HEIGHT), .MIN_PIX(MIN_PIX))
    u_bs_g (.I_clk(I_clk), .I_rst_n(I_rst_n), .I_tvalid(I_tvalid), .I_tuser(I_tuser), .I_tlast(I_tlast),
            .I_mask(m_g), .O_box_valid(v_g), .O_x0(gx0), .O_y0(gy0), .O_x1(gx1), .O_y1(gy1), .O_cnt(g_cnt));

    // ---- 宽、高、面积 ----
    wire [11:0] bw = v_b ? (bx1 - bx0 + 12'd1) : 12'd0;
    wire [11:0] bh = v_b ? (by1 - by0 + 12'd1) : 12'd0;
    wire [11:0] yw = v_y ? (yx1 - yx0 + 12'd1) : 12'd0;
    wire [11:0] yh = v_y ? (yy1 - yy0 + 12'd1) : 12'd0;
    wire [11:0] gw = v_g ? (gx1 - gx0 + 12'd1) : 12'd0;
    wire [11:0] gh = v_g ? (gy1 - gy0 + 12'd1) : 12'd0;

    wire [20:0] bw100 = bw * 21'd100;
    wire [20:0] yw100 = yw * 21'd100;
    wire [20:0] gw100 = gw * 21'd100;

    // 面积（供面积最大选择与填充率判定共用）
    wire [19:0] b_area = bw * bh;
    wire [19:0] y_area = yw * yh;
    wire [19:0] g_area = gw * gh;

    // 填充率：目标颜色像素数至少占包围盒面积一半。
    // 滤掉“同色多点/异形物体”造成的误检（真车牌是同色实心矩形，填充率高）。
    // cnt >= area/2
    wire fill_b = b_cnt >= {1'b0, b_area[19:1]};
    wire fill_y = y_cnt >= {1'b0, y_area[19:1]};
    wire fill_g = g_cnt >= {1'b0, g_area[19:1]};

    // 长宽比筛选：2.30 <= w/h <= 4.00（区间保持宽，用填充率补足精度）
    wire plate_b = v_b && (bh != 12'd0) && fill_b &&
                   (bw100 >= ASPECT_MIN * bh) && (bw100 <= ASPECT_MAX * bh);
    wire plate_y = v_y && (yh != 12'd0) && fill_y &&
                   (yw100 >= ASPECT_MIN * yh) && (yw100 <= ASPECT_MAX * yh);
    wire plate_g = v_g && (gh != 12'd0) && fill_g &&
                   (gw100 >= ASPECT_MIN * gh) && (gw100 <= ASPECT_MAX * gh);

    wire [19:0] area_b = plate_b ? b_area : 20'd0;
    wire [19:0] area_y = plate_y ? y_area : 20'd0;
    wire [19:0] area_g = plate_g ? g_area : 20'd0;

    // ---- 选面积最大的候选 ----
    reg        sel_valid;
    reg [1:0]  sel_type;
    reg [11:0] sel_x0, sel_y0, sel_x1, sel_y1;

    always @(*) begin
        sel_valid = 1'b0;
        sel_type  = 2'd0;
        sel_x0 = 12'd0; sel_y0 = 12'd0;
        sel_x1 = 12'd0; sel_y1 = 12'd0;

        if (plate_b) begin
            sel_valid = 1'b1; sel_type = 2'd0;
            sel_x0 = bx0; sel_y0 = by0; sel_x1 = bx1; sel_y1 = by1;
        end
        if (plate_y && ((!sel_valid) || (area_y > area_b))) begin
            sel_valid = 1'b1; sel_type = 2'd1;
            sel_x0 = yx0; sel_y0 = yy0; sel_x1 = yx1; sel_y1 = yy1;
        end
        if (plate_g && ((!sel_valid) || (area_g > area_b) || (area_g > area_y))) begin
            sel_valid = 1'b1; sel_type = 2'd2;
            sel_x0 = gx0; sel_y0 = gy0; sel_x1 = gx1; sel_y1 = gy1;
        end
    end

    assign O_plate_valid = sel_valid;
    assign O_plate_type  = sel_type;
    assign O_plate_x0    = sel_x0;
    assign O_plate_y0    = sel_y0;
    assign O_plate_x1    = sel_x1;
    assign O_plate_y1    = sel_y1;

endmodule
