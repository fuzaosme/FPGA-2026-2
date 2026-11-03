module hdmi_mixer #(
    parameter H_OFFSET = 128,
    parameter V_OFFSET = 60,
    parameter IMG_WIDTH = 1280,
    parameter IMG_HEIGHT = 720,
    parameter integer DEBUG_MODE = 0
)(
    input wire        I_clk,
    input wire        I_rst_n,

    input wire        I_video_vsync,
    input wire        I_video_hsync,
    input wire        I_video_de,
    input wire        I_video_user,
    input wire        I_video_last,

    input wire[3:0]   I_debug_status,

    output wire       O_video_rd_en,
    input wire[23:0]  I_video_rd_data,
    input wire        I_box_valid,
    input wire[11:0]  I_box_x0,
    input wire[11:0]  I_box_y0,
    input wire[11:0]  I_box_x1,
        input wire[11:0]  I_box_y1,
    input wire[2:0]   I_style,
    input wire[15:0]  I_latency,
    // ---- 多目标颜色块（ISP 域快照，每帧轮换一种颜色）----
    input wire[15:0]  I_mt_num,
    input wire[527:0] I_mt_tgt,
    input wire[3:0]   I_conf,     // 本帧最高置信度 0~9
    input wire[19:0]  I_e2e_us,   // 端到端延迟（微秒）
    // ---- 交通灯相位倒计时（2026-10-15 批 2，由顶层 phase_timer 产生）----
    input wire[3:0]   I_cd_t,     // 剩余秒 十位（BCD）
    input wire[3:0]   I_cd_u,     // 剩余秒 个位（BCD）
    input wire[1:0]   I_cd_phase, // 当前相位 0红 1黄 2绿
    input wire        I_cd_vld,   // 已经锁定过相位（否则数字留空）
    input wire        I_lp_sleep, // 批 3：检测支路是否在低功耗降频（空闲）
    input wire        I_ae_on,
    input wire        I_awb_on,
    input wire        I_t0_valid,
    input wire[1:0]   I_t0_id,
    input wire[2:0]   I_t0_type,
    input wire[11:0]  I_t0_x0,
    input wire[11:0]  I_t0_y0,
    input wire[11:0]  I_t0_x1,
    input wire[11:0]  I_t0_y1,
    input wire        I_t1_valid,
    input wire[1:0]   I_t1_id,
    input wire[2:0]   I_t1_type,
    input wire[11:0]  I_t1_x0,
    input wire[11:0]  I_t1_y0,
    input wire[11:0]  I_t1_x1,
    input wire[11:0]  I_t1_y1,

    output reg        O_hdmi_vsync,
    output reg        O_hdmi_hsync,
    output reg        O_hdmi_de,
    output reg[23:0]  O_hdmi_data,
    output wire[2:0] O_visible_lamps
);

    // These class bits are the shared lamp interface for rendering and UART.
    wire [2:0] lamp0_visible={I_t0_valid && I_t0_type==6,
                             I_t0_valid && I_t0_type==5,
                             I_t0_valid && I_t0_type==4};
    wire [2:0] lamp1_visible={I_t1_valid && I_t1_type==6,
                             I_t1_valid && I_t1_type==5,
                             I_t1_valid && I_t1_type==4};
    assign O_visible_lamps=lamp0_visible | lamp1_visible;
    wire draw_t0=(I_t0_type>=4 && I_t0_type<=6)?(|lamp0_visible):I_t0_valid;
    wire draw_t1=(I_t1_type>=4 && I_t1_type<=6)?(|lamp1_visible):I_t1_valid;

    reg        S_video_vsync_1d;
    reg        S_video_vsync_2d;
    reg        S_video_hsync_1d;
    reg        S_video_hsync_2d;
    reg        S_video_de_1d;
    reg        S_video_de_2d;
    reg [23:0] S_video_data_1d;
    reg [23:0] S_video_data_2d;
    reg [11:0] S_x;
    reg [11:0] S_y;
    reg [11:0] S_x_1d;
    reg [11:0] S_x_2d;
    reg [11:0] S_y_1d;
    reg [11:0] S_y_2d;

    reg        S_osd_hit;
    reg [23:0] S_osd_color;
    reg [11:0] S_logo_x;
    reg [11:0] S_logo_y;
    reg [3:0]  S_char_idx;
    reg [2:0]  S_char_row;
    reg [2:0]  S_char_col;
    reg [4:0]  S_char_bits;
    reg        S_char_on;
    reg        S_logo_blue_on;
    reg        S_logo_red_on;
    reg [11:0] S_logo_tmp_y;
    reg [3:0]  S_logo_red_band;
    reg [11:0] S_logo_red_x_start;
    reg [11:0] S_logo_red_x_end;
    reg [12:0] S_logo_addr;
    reg [12:0] S_logo_addr_r;
    reg        S_logo_in1, S_logo_in2;
    // 改动1：ROM 输出改为寄存读（原为组合读，是 75MHz 域最差路径的主因）
    wire[24:0] S_logo_pixel_c;
    wire [24:0] S_logo_pixel;
    // logo 寄存器块挪到 LOGO_X/Y/W/H 之后（避免前向引用）
    reg [31:0] S_frame_cnt;
    reg [11:0] S_dyn_x;
    reg [11:0] S_dyn_y;
    reg [3:0]  S_dyn_char;
    reg [2:0]  S_dyn_col;
    reg [2:0]  S_dyn_row;
    reg [4:0]  S_dyn_bits;
    reg        S_dyn_on;
    reg [3:0]  S_dyn_digit_idx;
    reg [11:0] S_dyn_digit_x;
    reg [6:0]  S_dyn_seg;
    reg [3:0]  S_cnt_d0;
    reg [3:0]  S_cnt_d1;
    reg [3:0]  S_cnt_d2;
    reg [3:0]  S_cnt_d3;
    reg [3:0]  S_cnt_d4;
    reg [3:0]  S_cnt_d5;
    reg [2:0]  S_lbl_row;
    reg [2:0]  S_lbl_col;
    reg [1:0]  S_lbl_idx;
    reg [4:0]  S_lbl_bits;
    reg [6:0]  S_lbl_local_x;
    reg [3:0]  S_dyn_dec_digit;
    reg [4:0]  S_cn_local_x;
    reg [4:0]  S_cn_local_y;
    reg [3:0]  S_cn_col;
    reg [3:0]  S_cn_row;
    reg [15:0] S_cn_row_bits;
    reg        S_cn_on;
    reg [9:0]  S_cn_scale_x;
    reg [9:0]  S_cn_scale_y;
    reg [7:0]  S_pf_char;
    reg [4:0]  S_pf_local_x;
    reg [4:0]  S_pf_local_y;
    reg [3:0]  S_pf_col;
    reg [3:0]  S_pf_row;
    wire [15:0] S_pf_row_bits;
    reg        S_pf_on;
    reg [7:0]  S_st_char;
    reg [3:0]  S_st_row;
    reg [3:0]  S_st_col;
    reg [7:0]  S_st_local_x;
    reg [5:0]  S_st_local_y;
    reg        S_st_active;
    wire[15:0] S_st_row_bits;
    reg [7:0]  S_an_char;
    reg [3:0]  S_an_row;
    wire[15:0] S_an_row_bits;
    reg  [3:0]  S_lat_h, S_lat_t, S_lat_u;
    integer     cc, tt;
    reg  [43:0] mt_tmp;
    // 显示只用一位数字：总数最多 18，超过 9 就按 9 显示（表示"9 个以上"）
    wire [5:0]  mt_sum = I_mt_num[3:0] + I_mt_num[7:4] + I_mt_num[11:8];
    wire [3:0]  mt_tot = (mt_sum > 6'd9) ? 4'd9 : mt_sum[3:0];
    wire[11:0] S_an_lx = S_x_2d - 12'd40;

    // ---- 圆形目标个数：12 个目标槽的形状位 = 每个 44bit 的 bit0 ----
    integer circ_i, circ_j;
    reg [4:0] S_circ_sum;
    always @(*) begin
        S_circ_sum = 5'd0;
        for (circ_i = 0; circ_i < 4; circ_i = circ_i + 1)
            for (circ_j = 0; circ_j < 3; circ_j = circ_j + 1)
                S_circ_sum = S_circ_sum +
                    {4'd0, (I_mt_tgt[circ_i*132 + circ_j*44 + 43] &
                            I_mt_tgt[circ_i*132 + circ_j*44 + 0])};
    end
    wire [3:0] S_circ_num = (S_circ_sum > 5'd9) ? 4'd9 : S_circ_sum[3:0];
    reg [9:0]  S_pf_scale_x;
    reg [9:0]  S_pf_scale_y;
    // ===================== 2026-10-10 时序补丁 timing3 =====================
    // 依据 phy_1/final_timing.rpt：HDMI_PIXEL_CLK 最差路径 slack 0.028ns，
    // 19 级逻辑 12.948ns（cell 5.423ns 41% / net 7.525ns 59%），
    // 起点 S_x_2d[1] fanout 44，中途 osd_char_lib/I_char[0] fanout 35。
    // 走向：坐标 → 减偏移 → 十进制位判断链 → 字符索引 → 字模 ROM → 命中链 → 输出寄存器。
    // 处理办法与本文件 logo 那处相同：把"算字符索引"和"查字模 ROM"拆成两拍。
    // 第 2 拍（本文下面第一个 always）只做几何判定 + 字符索引 + 候选使能；
    // 第 3 拍（本文后面）才查字模 ROM 并做并行命中优先链。
    reg        S_video_de_3d;
    reg [23:0] S_video_data_3d;
    reg        S_video_hsync_3d;
    reg        S_video_vsync_3d;

    reg        S_h_logo, S_h_box, S_h_t0, S_h_t1, S_h_mt;
    reg [23:0] S_c_logo, S_c_t0, S_c_t1, S_c_mt;
    reg        S_en_pfx, S_en_dgt, S_an_en;
    reg [23:0] S_an_c;

    reg [7:0]  S_pf_char_3;
    reg [3:0]  S_pf_row_3;
    reg [3:0]  S_pf_col_3;
    reg        S_pf_en_pfx_3, S_pf_en_dgt_3;
    reg [7:0]  S_st_char_3;
    reg [3:0]  S_st_row_3;
    reg [3:0]  S_st_col_3;
    reg        S_st_active_3;
    reg [7:0]  S_an_char_3;
    reg [3:0]  S_an_row_3;
    reg [3:0]  S_an_lx_3;
    reg        S_an_en_3;
    reg [23:0] S_an_c_3;
    reg        S_h_logo_3, S_h_box_3, S_h_t0_3, S_h_t1_3, S_h_mt_3;
    reg [23:0] S_c_logo_3, S_c_t0_3, S_c_t1_3, S_c_mt_3;
    reg        S_h_dyn, S_h_st, S_h_an;
    reg        S_sel1, S_sel2, S_sel3, S_sel4;
    reg        S_sel5, S_sel6, S_sel7, S_sel8;

    localparam LOGO_X = 12'd0;
    localparam LOGO_Y = 12'd0;
    localparam LOGO_W = 12'd160;
    localparam LOGO_H = 12'd160;
    // ★ logo 流水线：把"算地址"和"查 ROM"拆成两拍。
    //   实测原来这条是 75MHz 域最差路径（slack 只剩 0.005ns）：
    //   S_x_2d 寄存器 → 减 LOGO_X/Y → 乘160 加 x 算地址 → 2487 个 LUT 的 ROM
    //   → 输出寄存器，22 级逻辑共 13.085ns。插一拍后两段各自都有余量。
    //   命中标志同步延迟两拍，与 ROM 输出对齐（顺便修掉原来 1 像素的错位）。
    //   必须放在 LOGO_X/Y/W/H 之后，否则是前向引用，编译会报未定义。
    wire S_logo_in_box = S_video_de_2d &&
                         (S_x_2d >= LOGO_X) && (S_x_2d < (LOGO_X + LOGO_W)) &&
                         (S_y_2d >= LOGO_Y) && (S_y_2d < (LOGO_Y + LOGO_H));

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_logo_addr_r <= 13'd0;
            S_logo_in1    <= 1'b0;
            S_logo_in2    <= 1'b0;

        end
        else begin
            S_logo_addr_r <= S_logo_addr;
            S_logo_in1    <= S_logo_in_box;
            S_logo_in2    <= S_logo_in1;

        end
    end

    localparam COLOR_LOGO_BLUE   = 24'h1f315c;
    localparam COLOR_LOGO_RED    = 24'hec1c2d;
    localparam COLOR_TEXT        = 24'h00e6ff;
    localparam DYN_OSD_X         = 12'd375;
    localparam DYN_OSD_Y         = 12'd120;
    localparam DYN_OSD_W         = 12'd200;
    localparam DYN_OSD_H         = 12'd24;
    localparam CN_Y_OFFSET       = 12'd2;
    localparam COLOR_DYN_TEXT    = 24'hfff200;
    localparam CHAR_AN           = 8'h80;
    localparam CHAR_LU           = 8'h81;
    localparam CHAR_C            = 8'h43;
    localparam CHAR_N            = 8'h4e;
    localparam CHAR_T            = 8'h54;
    localparam CHAR_0            = 8'h30;
    localparam CHAR_COLON        = 8'h3a;

    function [4:0] F_logo_text_bits;
        input [3:0] I_char;
        input [2:0] I_row;
        begin
            case(I_char)
                4'd0: begin
                    case(I_row) // A
                        3'd0: F_logo_text_bits = 5'b01110;
                        3'd1: F_logo_text_bits = 5'b10001;
                        3'd2: F_logo_text_bits = 5'b10001;
                        3'd3: F_logo_text_bits = 5'b11111;
                        3'd4: F_logo_text_bits = 5'b10001;
                        3'd5: F_logo_text_bits = 5'b10001;
                        3'd6: F_logo_text_bits = 5'b10001;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                4'd1: begin
                    case(I_row) // N
                        3'd0: F_logo_text_bits = 5'b10001;
                        3'd1: F_logo_text_bits = 5'b11001;
                        3'd2: F_logo_text_bits = 5'b10101;
                        3'd3: F_logo_text_bits = 5'b10011;
                        3'd4: F_logo_text_bits = 5'b10001;
                        3'd5: F_logo_text_bits = 5'b10001;
                        3'd6: F_logo_text_bits = 5'b10001;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                4'd2: begin
                    case(I_row) // L
                        3'd0: F_logo_text_bits = 5'b10000;
                        3'd1: F_logo_text_bits = 5'b10000;
                        3'd2: F_logo_text_bits = 5'b10000;
                        3'd3: F_logo_text_bits = 5'b10000;
                        3'd4: F_logo_text_bits = 5'b10000;
                        3'd5: F_logo_text_bits = 5'b10000;
                        3'd6: F_logo_text_bits = 5'b11111;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                4'd3: begin
                    case(I_row) // O
                        3'd0: F_logo_text_bits = 5'b01110;
                        3'd1: F_logo_text_bits = 5'b10001;
                        3'd2: F_logo_text_bits = 5'b10001;
                        3'd3: F_logo_text_bits = 5'b10001;
                        3'd4: F_logo_text_bits = 5'b10001;
                        3'd5: F_logo_text_bits = 5'b10001;
                        3'd6: F_logo_text_bits = 5'b01110;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                4'd4: begin
                    case(I_row) // G
                        3'd0: F_logo_text_bits = 5'b01110;
                        3'd1: F_logo_text_bits = 5'b10001;
                        3'd2: F_logo_text_bits = 5'b10000;
                        3'd3: F_logo_text_bits = 5'b10111;
                        3'd4: F_logo_text_bits = 5'b10001;
                        3'd5: F_logo_text_bits = 5'b10001;
                        3'd6: F_logo_text_bits = 5'b01110;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                4'd5: begin
                    case(I_row) // I
                        3'd0: F_logo_text_bits = 5'b11111;
                        3'd1: F_logo_text_bits = 5'b00100;
                        3'd2: F_logo_text_bits = 5'b00100;
                        3'd3: F_logo_text_bits = 5'b00100;
                        3'd4: F_logo_text_bits = 5'b00100;
                        3'd5: F_logo_text_bits = 5'b00100;
                        3'd6: F_logo_text_bits = 5'b11111;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                4'd6: begin
                    case(I_row) // C
                        3'd0: F_logo_text_bits = 5'b01110;
                        3'd1: F_logo_text_bits = 5'b10001;
                        3'd2: F_logo_text_bits = 5'b10000;
                        3'd3: F_logo_text_bits = 5'b10000;
                        3'd4: F_logo_text_bits = 5'b10000;
                        3'd5: F_logo_text_bits = 5'b10001;
                        3'd6: F_logo_text_bits = 5'b01110;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                default: F_logo_text_bits = 5'b00000;
            endcase
        end
    endfunction

    // 7-seg bit mapping: {a,b,c,d,e,f,g}
    function [6:0] F_hex7seg;
        input [3:0] I_hex;
        begin
            case(I_hex)
                4'h0: F_hex7seg = 7'b1111110;
                4'h1: F_hex7seg = 7'b0110000;
                4'h2: F_hex7seg = 7'b1101101;
                4'h3: F_hex7seg = 7'b1111001;
                4'h4: F_hex7seg = 7'b0110011;
                4'h5: F_hex7seg = 7'b1011011;
                4'h6: F_hex7seg = 7'b1011111;
                4'h7: F_hex7seg = 7'b1110000;
                4'h8: F_hex7seg = 7'b1111111;
                4'h9: F_hex7seg = 7'b1111011;
                4'ha: F_hex7seg = 7'b1110111;
                4'hb: F_hex7seg = 7'b0011111;
                4'hc: F_hex7seg = 7'b1001110;
                4'hd: F_hex7seg = 7'b0111101;
                4'he: F_hex7seg = 7'b1001111;
                4'hf: F_hex7seg = 7'b1000111;
                default: F_hex7seg = 7'b0000000;
            endcase
        end
    endfunction

    function [4:0] F_cnt_label_bits;
        input [1:0] I_idx;
        input [2:0] I_row;
        begin
            case(I_idx)
                2'd0: begin // C
                    case(I_row)
                        3'd0: F_cnt_label_bits = 5'b01110;
                        3'd1: F_cnt_label_bits = 5'b10001;
                        3'd2: F_cnt_label_bits = 5'b10000;
                        3'd3: F_cnt_label_bits = 5'b10000;
                        3'd4: F_cnt_label_bits = 5'b10000;
                        3'd5: F_cnt_label_bits = 5'b10001;
                        3'd6: F_cnt_label_bits = 5'b01110;
                        default: F_cnt_label_bits = 5'b00000;
                    endcase
                end
                2'd1: begin // N
                    case(I_row)
                        3'd0: F_cnt_label_bits = 5'b10001;
                        3'd1: F_cnt_label_bits = 5'b11001;
                        3'd2: F_cnt_label_bits = 5'b10101;
                        3'd3: F_cnt_label_bits = 5'b10011;
                        3'd4: F_cnt_label_bits = 5'b10001;
                        3'd5: F_cnt_label_bits = 5'b10001;
                        3'd6: F_cnt_label_bits = 5'b10001;
                        default: F_cnt_label_bits = 5'b00000;
                    endcase
                end
                2'd2: begin // T
                    case(I_row)
                        3'd0: F_cnt_label_bits = 5'b11111;
                        3'd1: F_cnt_label_bits = 5'b00100;
                        3'd2: F_cnt_label_bits = 5'b00100;
                        3'd3: F_cnt_label_bits = 5'b00100;
                        3'd4: F_cnt_label_bits = 5'b00100;
                        3'd5: F_cnt_label_bits = 5'b00100;
                        3'd6: F_cnt_label_bits = 5'b00100;
                        default: F_cnt_label_bits = 5'b00000;
                    endcase
                end
                default: begin // ':'
                    case(I_row)
                        3'd2: F_cnt_label_bits = 5'b00100;
                        3'd5: F_cnt_label_bits = 5'b00100;
                        default: F_cnt_label_bits = 5'b00000;
                    endcase
                end
            endcase
        end
    endfunction

    function [2:0] F_char_idx3;
        input [7:0] I;
        begin
            if (I < 8'd48)      F_char_idx3 = 3'd0;
            else if (I < 8'd96) F_char_idx3 = 3'd1;
            else                F_char_idx3 = 3'd2;
        end
    endfunction

    function [5:0] F_char_local_x;
        input [7:0] I;
        reg [7:0] t;
        begin
            if (I < 8'd48)      t = I;
            else if (I < 8'd96) t = I - 8'd48;
            else                t = I - 8'd96;
            F_char_local_x = t[5:0];
        end
    endfunction

    function [3:0] F_scale3_col;
        input [5:0] I;
        begin
            case (I)
                6'd0, 6'd1, 6'd2:   F_scale3_col = 4'd0;
                6'd3, 6'd4, 6'd5:   F_scale3_col = 4'd1;
                6'd6, 6'd7, 6'd8:   F_scale3_col = 4'd2;
                6'd9, 6'd10, 6'd11: F_scale3_col = 4'd3;
                6'd12, 6'd13, 6'd14: F_scale3_col = 4'd4;
                6'd15, 6'd16, 6'd17: F_scale3_col = 4'd5;
                6'd18, 6'd19, 6'd20: F_scale3_col = 4'd6;
                6'd21, 6'd22, 6'd23: F_scale3_col = 4'd7;
                6'd24, 6'd25, 6'd26: F_scale3_col = 4'd8;
                6'd27, 6'd28, 6'd29: F_scale3_col = 4'd9;
                6'd30, 6'd31, 6'd32: F_scale3_col = 4'd10;
                6'd33, 6'd34, 6'd35: F_scale3_col = 4'd11;
                6'd36, 6'd37, 6'd38: F_scale3_col = 4'd12;
                6'd39, 6'd40, 6'd41: F_scale3_col = 4'd13;
                6'd42, 6'd43, 6'd44: F_scale3_col = 4'd14;
                6'd45, 6'd46, 6'd47: F_scale3_col = 4'd15;
                default: F_scale3_col = 4'd0;
            endcase
        end
    endfunction

    function [3:0] F_scale3_row;
        input [5:0] I;
        begin
            case (I)
                6'd0, 6'd1, 6'd2:  F_scale3_row = 4'd0;
                6'd3, 6'd4, 6'd5:  F_scale3_row = 4'd1;
                6'd6, 6'd7, 6'd8:  F_scale3_row = 4'd2;
                6'd9, 6'd10, 6'd11: F_scale3_row = 4'd3;
                6'd12, 6'd13, 6'd14: F_scale3_row = 4'd4;
                6'd15, 6'd16, 6'd17: F_scale3_row = 4'd5;
                6'd18, 6'd19, 6'd20: F_scale3_row = 4'd6;
                6'd21, 6'd22, 6'd23: F_scale3_row = 4'd7;
                6'd24, 6'd25, 6'd26: F_scale3_row = 4'd8;
                6'd27, 6'd28, 6'd29: F_scale3_row = 4'd9;
                6'd30, 6'd31, 6'd32: F_scale3_row = 4'd10;
                6'd33, 6'd34, 6'd35: F_scale3_row = 4'd11;
                6'd36, 6'd37, 6'd38: F_scale3_row = 4'd12;
                6'd39, 6'd40, 6'd41: F_scale3_row = 4'd13;
                6'd42, 6'd43, 6'd44: F_scale3_row = 4'd14;
                6'd45, 6'd46, 6'd47: F_scale3_row = 4'd15;
                default: F_scale3_row = 4'd0;
            endcase
        end
    endfunction

    function [7:0] F_mode_name_char;
        input [2:0] I_mode;
        input [2:0] I_idx;
        begin
            case (I_mode)
                3'd0: case (I_idx)
                    3'd0: F_mode_name_char = 8'hA0; // 原
                    3'd1: F_mode_name_char = 8'hA1; // 图
                    default: F_mode_name_char = 8'h20;
                endcase
                3'd1: case (I_idx)
                    3'd0: F_mode_name_char = 8'hA2; // 二
                    3'd1: F_mode_name_char = 8'hA3; // 值
                    3'd2: F_mode_name_char = 8'hA4; // 化
                    default: F_mode_name_char = 8'h20;
                endcase
                3'd2: case (I_idx)
                    3'd0: F_mode_name_char = 8'hA5; // 线
                    3'd1: F_mode_name_char = 8'hA6; // 稿
                    default: F_mode_name_char = 8'h20;
                endcase
                3'd3: case (I_idx)
                    3'd0: F_mode_name_char = 8'hA7; // 卡
                    3'd1: F_mode_name_char = 8'hA8; // 通
                    default: F_mode_name_char = 8'h20;
                endcase
                3'd4: case (I_idx)
                    3'd0: F_mode_name_char = 8'hA9; // 伪
                    3'd1: F_mode_name_char = 8'hAA; // 彩
                    default: F_mode_name_char = 8'h20;
                endcase
                default: case (I_idx)
                    3'd0: F_mode_name_char = 8'hAB; // 掩
                    3'd1: F_mode_name_char = 8'hAC; // 码
                    default: F_mode_name_char = 8'h20;
                endcase
            endcase
        end
    endfunction

    function [7:0] F_hex_ascii;
        input [3:0] I_hex;
        begin
            F_hex_ascii = (I_hex < 4'd10) ? (8'h30 + I_hex) : (8'h37 + I_hex);
        end
    endfunction

    // 多目标颜色块画框颜色：按类别红/绿/黄
    function [23:0] F_mt_color;
        input [1:0] c;
        begin
            case (c)
                2'd0: F_mt_color = 24'hFF4040;    // 红
                2'd1: F_mt_color = 24'h00FF60;    // 绿
                2'd2: F_mt_color = 24'hFFE000;    // 黄
                default: F_mt_color = 24'h808080; // 灰 = motion
            endcase
        end
    endfunction

    // 圆形目标的框色：同色系但更亮更淡，一眼能和"非圆形"区分
    function [23:0] F_mt_color_c;
        input [1:0] c;
        begin
            case (c)
                2'd0: F_mt_color_c = 24'hFFA8A8;    // 红-圆
                2'd1: F_mt_color_c = 24'hA8FFC8;    // 绿-圆
                2'd2: F_mt_color_c = 24'hFFF0A0;    // 黄-圆
                default: F_mt_color_c = 24'hD8D8D8; // 灰(motion)-圆
            endcase
        end
    endfunction

    function [7:0] F_cat0;
        input [2:0] I_type;
        begin
            case (I_type)
                3'd0: F_cat0 = 8'hAD;
                3'd1: F_cat0 = 8'h82;
                3'd2: F_cat0 = 8'h83;
                3'd3: F_cat0 = 8'h84;
                3'd4: F_cat0 = 8'hAD;
                3'd5: F_cat0 = 8'hB0;
                default: F_cat0 = 8'hB1;
            endcase
        end
    endfunction

    function [7:0] F_cat1;
        input [2:0] I_type;
        begin
            if (I_type == 3'd0) F_cat1 = 8'hAE;
            else if (I_type < 3'd4) F_cat1 = 8'h85;
            else F_cat1 = 8'hAF;
        end
    endfunction

    function [23:0] F_type_color;
        input [2:0] I_type;
        begin
            case (I_type)
                3'd0: F_type_color = 24'h808080;
                3'd1: F_type_color = 24'h00A0FF;
                3'd2: F_type_color = 24'hFFE000;
                3'd3: F_type_color = 24'h00E060;
                3'd4: F_type_color = 24'hff4040;
                3'd5: F_type_color = 24'hFFE000;
                3'd7: F_type_color = 24'hFF00FF;
                default: F_type_color = 24'h00FF60;
            endcase
        end
    endfunction

    // Minimal 16x16 Chinese bitmap rows for "an" and "lu".
    function [15:0] F_cn16_row;
        input I_char; // 0: an, 1: lu
        input [3:0] I_row;
        begin
            if(!I_char) begin
                case(I_row)
                    4'd0:  F_cn16_row = 16'h0200;
                    4'd1:  F_cn16_row = 16'h7ff0;
                    4'd2:  F_cn16_row = 16'h4410;
                    4'd3:  F_cn16_row = 16'h0400;
                    4'd4:  F_cn16_row = 16'h7ff0;
                    4'd5:  F_cn16_row = 16'h0840;
                    4'd6:  F_cn16_row = 16'h1040;
                    4'd7:  F_cn16_row = 16'h1c80;
                    4'd8:  F_cn16_row = 16'h0300;
                    4'd9:  F_cn16_row = 16'h0cc0;
                    4'd10: F_cn16_row = 16'h7020;
                    4'd11: F_cn16_row = 16'h0000;
                    4'd12: F_cn16_row = 16'h0000;
                    4'd13: F_cn16_row = 16'h0000;
                    4'd14: F_cn16_row = 16'h0000;
                    4'd15: F_cn16_row = 16'h0000;
                    default: F_cn16_row = 16'h0000;
                endcase
            end
            else begin
                case(I_row)
                    4'd0:  F_cn16_row = 16'h79e0;
                    4'd1:  F_cn16_row = 16'h4a20;
                    4'd2:  F_cn16_row = 16'h4d40;
                    4'd3:  F_cn16_row = 16'h7880;
                    4'd4:  F_cn16_row = 16'h1140;
                    4'd5:  F_cn16_row = 16'h1630;
                    4'd6:  F_cn16_row = 16'h5be0;
                    4'd7:  F_cn16_row = 16'h5220;
                    4'd8:  F_cn16_row = 16'h5220;
                    4'd9:  F_cn16_row = 16'h5be0;
                    4'd10: F_cn16_row = 16'h6220;
                    4'd11: F_cn16_row = 16'h0000;
                    4'd12: F_cn16_row = 16'h0000;
                    4'd13: F_cn16_row = 16'h0000;
                    4'd14: F_cn16_row = 16'h0000;
                    4'd15: F_cn16_row = 16'h0000;
                    default: F_cn16_row = 16'h0000;
                endcase
            end
        end
    endfunction

    function [4:0] F_dyn_char_bits;
        input [3:0] I_char;
        input [2:0] I_row;
        begin
            case(I_char)
                4'h0: begin case(I_row) 3'd0:F_dyn_char_bits=5'b01110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10011; 3'd3:F_dyn_char_bits=5'b10101; 3'd4:F_dyn_char_bits=5'b11001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b01110; default:F_dyn_char_bits=5'b00000; endcase end
                4'h1: begin case(I_row) 3'd0:F_dyn_char_bits=5'b00100; 3'd1:F_dyn_char_bits=5'b01100; 3'd2:F_dyn_char_bits=5'b00100; 3'd3:F_dyn_char_bits=5'b00100; 3'd4:F_dyn_char_bits=5'b00100; 3'd5:F_dyn_char_bits=5'b00100; 3'd6:F_dyn_char_bits=5'b01110; default:F_dyn_char_bits=5'b00000; endcase end
                4'h2: begin case(I_row) 3'd0:F_dyn_char_bits=5'b01110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b00001; 3'd3:F_dyn_char_bits=5'b00110; 3'd4:F_dyn_char_bits=5'b01000; 3'd5:F_dyn_char_bits=5'b10000; 3'd6:F_dyn_char_bits=5'b11111; default:F_dyn_char_bits=5'b00000; endcase end
                4'h3: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11110; 3'd1:F_dyn_char_bits=5'b00001; 3'd2:F_dyn_char_bits=5'b00001; 3'd3:F_dyn_char_bits=5'b01110; 3'd4:F_dyn_char_bits=5'b00001; 3'd5:F_dyn_char_bits=5'b00001; 3'd6:F_dyn_char_bits=5'b11110; default:F_dyn_char_bits=5'b00000; endcase end
                4'h4: begin case(I_row) 3'd0:F_dyn_char_bits=5'b00010; 3'd1:F_dyn_char_bits=5'b00110; 3'd2:F_dyn_char_bits=5'b01010; 3'd3:F_dyn_char_bits=5'b10010; 3'd4:F_dyn_char_bits=5'b11111; 3'd5:F_dyn_char_bits=5'b00010; 3'd6:F_dyn_char_bits=5'b00010; default:F_dyn_char_bits=5'b00000; endcase end
                4'h5: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11111; 3'd1:F_dyn_char_bits=5'b10000; 3'd2:F_dyn_char_bits=5'b11110; 3'd3:F_dyn_char_bits=5'b00001; 3'd4:F_dyn_char_bits=5'b00001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b01110; default:F_dyn_char_bits=5'b00000; endcase end
                4'h6: begin case(I_row) 3'd0:F_dyn_char_bits=5'b00110; 3'd1:F_dyn_char_bits=5'b01000; 3'd2:F_dyn_char_bits=5'b10000; 3'd3:F_dyn_char_bits=5'b11110; 3'd4:F_dyn_char_bits=5'b10001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b01110; default:F_dyn_char_bits=5'b00000; endcase end
                4'h7: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11111; 3'd1:F_dyn_char_bits=5'b00001; 3'd2:F_dyn_char_bits=5'b00010; 3'd3:F_dyn_char_bits=5'b00100; 3'd4:F_dyn_char_bits=5'b01000; 3'd5:F_dyn_char_bits=5'b01000; 3'd6:F_dyn_char_bits=5'b01000; default:F_dyn_char_bits=5'b00000; endcase end
                4'h8: begin case(I_row) 3'd0:F_dyn_char_bits=5'b01110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10001; 3'd3:F_dyn_char_bits=5'b01110; 3'd4:F_dyn_char_bits=5'b10001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b01110; default:F_dyn_char_bits=5'b00000; endcase end
                4'h9: begin case(I_row) 3'd0:F_dyn_char_bits=5'b01110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10001; 3'd3:F_dyn_char_bits=5'b01111; 3'd4:F_dyn_char_bits=5'b00001; 3'd5:F_dyn_char_bits=5'b00010; 3'd6:F_dyn_char_bits=5'b11100; default:F_dyn_char_bits=5'b00000; endcase end
                4'ha: begin case(I_row) 3'd0:F_dyn_char_bits=5'b01110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10001; 3'd3:F_dyn_char_bits=5'b11111; 3'd4:F_dyn_char_bits=5'b10001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b10001; default:F_dyn_char_bits=5'b00000; endcase end
                4'hb: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10001; 3'd3:F_dyn_char_bits=5'b11110; 3'd4:F_dyn_char_bits=5'b10001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b11110; default:F_dyn_char_bits=5'b00000; endcase end
                4'hc: begin case(I_row) 3'd0:F_dyn_char_bits=5'b01110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10000; 3'd3:F_dyn_char_bits=5'b10000; 3'd4:F_dyn_char_bits=5'b10000; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b01110; default:F_dyn_char_bits=5'b00000; endcase end
                4'hd: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10001; 3'd3:F_dyn_char_bits=5'b10001; 3'd4:F_dyn_char_bits=5'b10001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b11110; default:F_dyn_char_bits=5'b00000; endcase end
                4'he: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11111; 3'd1:F_dyn_char_bits=5'b10000; 3'd2:F_dyn_char_bits=5'b10000; 3'd3:F_dyn_char_bits=5'b11110; 3'd4:F_dyn_char_bits=5'b10000; 3'd5:F_dyn_char_bits=5'b10000; 3'd6:F_dyn_char_bits=5'b11111; default:F_dyn_char_bits=5'b00000; endcase end
                4'hf: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11111; 3'd1:F_dyn_char_bits=5'b10000; 3'd2:F_dyn_char_bits=5'b10000; 3'd3:F_dyn_char_bits=5'b11110; 3'd4:F_dyn_char_bits=5'b10000; 3'd5:F_dyn_char_bits=5'b10000; 3'd6:F_dyn_char_bits=5'b10000; default:F_dyn_char_bits=5'b00000; endcase end
                default: F_dyn_char_bits = 5'b00000;
            endcase
        end
    endfunction

    // Unified prefix character library (16x16 rows): an/lu/C/N/T/:
    function [15:0] F_osd_char16_row;
        input [7:0] I_char;
        input [3:0] I_row;
        begin
            case(I_char)
                CHAR_AN: begin
                    case(I_row)
                        4'd0:  F_osd_char16_row = 16'h0200;
                        4'd1:  F_osd_char16_row = 16'h7ff0;
                        4'd2:  F_osd_char16_row = 16'h4410;
                        4'd3:  F_osd_char16_row = 16'h0400;
                        4'd4:  F_osd_char16_row = 16'h7ff0;
                        4'd5:  F_osd_char16_row = 16'h0840;
                        4'd6:  F_osd_char16_row = 16'h1040;
                        4'd7:  F_osd_char16_row = 16'h1c80;
                        4'd8:  F_osd_char16_row = 16'h0300;
                        4'd9:  F_osd_char16_row = 16'h0cc0;
                        4'd10: F_osd_char16_row = 16'h7020;
                        default: F_osd_char16_row = 16'h0000;
                    endcase
                end
                CHAR_LU: begin
                    case(I_row)
                        4'd0:  F_osd_char16_row = 16'h79e0;
                        4'd1:  F_osd_char16_row = 16'h4a20;
                        4'd2:  F_osd_char16_row = 16'h4d40;
                        4'd3:  F_osd_char16_row = 16'h7880;
                        4'd4:  F_osd_char16_row = 16'h1140;
                        4'd5:  F_osd_char16_row = 16'h1630;
                        4'd6:  F_osd_char16_row = 16'h5be0;
                        4'd7:  F_osd_char16_row = 16'h5220;
                        4'd8:  F_osd_char16_row = 16'h5220;
                        4'd9:  F_osd_char16_row = 16'h5be0;
                        4'd10: F_osd_char16_row = 16'h6220;
                        default: F_osd_char16_row = 16'h0000;
                    endcase
                end
                CHAR_C: begin
                    case(I_row)
                        4'd2:  F_osd_char16_row = 16'h03f0;
                        4'd3:  F_osd_char16_row = 16'h0c0c;
                        4'd4:  F_osd_char16_row = 16'h1800;
                        4'd5:  F_osd_char16_row = 16'h3000;
                        4'd6:  F_osd_char16_row = 16'h3000;
                        4'd7:  F_osd_char16_row = 16'h3000;
                        4'd8:  F_osd_char16_row = 16'h3000;
                        4'd9:  F_osd_char16_row = 16'h3000;
                        4'd10: F_osd_char16_row = 16'h1800;
                        4'd11: F_osd_char16_row = 16'h0c0c;
                        4'd12: F_osd_char16_row = 16'h03f0;
                        default: F_osd_char16_row = 16'h0000;
                    endcase
                end
                CHAR_N: begin
                    case(I_row)
                        4'd2:  F_osd_char16_row = 16'h300c;
                        4'd3:  F_osd_char16_row = 16'h380c;
                        4'd4:  F_osd_char16_row = 16'h3c0c;
                        4'd5:  F_osd_char16_row = 16'h360c;
                        4'd6:  F_osd_char16_row = 16'h330c;
                        4'd7:  F_osd_char16_row = 16'h318c;
                        4'd8:  F_osd_char16_row = 16'h30cc;
                        4'd9:  F_osd_char16_row = 16'h306c;
                        4'd10: F_osd_char16_row = 16'h303c;
                        4'd11: F_osd_char16_row = 16'h301c;
                        4'd12: F_osd_char16_row = 16'h300c;
                        default: F_osd_char16_row = 16'h0000;
                    endcase
                end
                CHAR_T: begin
                    case(I_row)
                        4'd2:  F_osd_char16_row = 16'h3ffc;
                        4'd3:  F_osd_char16_row = 16'h03c0;
                        4'd4:  F_osd_char16_row = 16'h03c0;
                        4'd5:  F_osd_char16_row = 16'h03c0;
                        4'd6:  F_osd_char16_row = 16'h03c0;
                        4'd7:  F_osd_char16_row = 16'h03c0;
                        4'd8:  F_osd_char16_row = 16'h03c0;
                        4'd9:  F_osd_char16_row = 16'h03c0;
                        4'd10: F_osd_char16_row = 16'h03c0;
                        4'd11: F_osd_char16_row = 16'h03c0;
                        4'd12: F_osd_char16_row = 16'h03c0;
                        default: F_osd_char16_row = 16'h0000;
                    endcase
                end
                CHAR_COLON: begin
                    case(I_row)
                        4'd6:  F_osd_char16_row = 16'h0180;
                        4'd7:  F_osd_char16_row = 16'h0180;
                        4'd10: F_osd_char16_row = 16'h0180;
                        4'd11: F_osd_char16_row = 16'h0180;
                        default: F_osd_char16_row = 16'h0000;
                    endcase
                end
                default: F_osd_char16_row = 16'h0000;
            endcase
        end
    endfunction

    assign O_video_rd_en = I_video_de;

    anlogic_logo_rom u_anlogic_logo_rom(
        .I_clk(I_clk),.I_rst_n(I_rst_n),
        .I_addr(S_logo_addr),.O_pixel(S_logo_pixel)
    );

    osd_char_lib u_osd_char_lib(
        .I_clk(I_clk),
        .I_char     ( S_pf_char   ),
        .I_row      ( S_pf_row    ),
        .O_row_bits ( S_pf_row_bits )
    );


    osd_char_lib u_osd_char_lib_status(
        .I_clk(I_clk),
        .I_char     ( S_st_char ),
        .I_row      ( S_st_row  ),
        .O_row_bits ( S_st_row_bits )
    );

    osd_char_lib u_osd_char_lib_annot(
        .I_clk(I_clk),
        .I_char     ( S_an_char ),
        .I_row      ( S_an_row  ),
        .O_row_bits ( S_an_row_bits )
    );

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if(I_video_user) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if(I_video_de) begin
            if(I_video_last) begin
                S_x <= 12'd0;
                S_y <= S_y + 12'd1;
            end
            else begin
                S_x <= S_x + 12'd1;
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_frame_cnt <= 32'd0;
            S_cnt_d0 <= 4'd0;
            S_cnt_d1 <= 4'd0;
            S_cnt_d2 <= 4'd0;
            S_cnt_d3 <= 4'd0;
            S_cnt_d4 <= 4'd0;
            S_cnt_d5 <= 4'd0;
        end
        else if(I_video_user) begin
            S_frame_cnt <= S_frame_cnt + 32'd1;
            if(S_cnt_d0 == 4'd9) begin
                S_cnt_d0 <= 4'd0;
                if(S_cnt_d1 == 4'd9) begin
                    S_cnt_d1 <= 4'd0;
                    if(S_cnt_d2 == 4'd9) begin
                        S_cnt_d2 <= 4'd0;
                        if(S_cnt_d3 == 4'd9) begin
                            S_cnt_d3 <= 4'd0;
                            if(S_cnt_d4 == 4'd9) begin
                                S_cnt_d4 <= 4'd0;
                                if(S_cnt_d5 == 4'd9)
                                    S_cnt_d5 <= 4'd0;
                                else
                                    S_cnt_d5 <= S_cnt_d5 + 4'd1;
                            end
                            else
                                S_cnt_d4 <= S_cnt_d4 + 4'd1;
                        end
                        else
                            S_cnt_d3 <= S_cnt_d3 + 4'd1;
                    end
                    else
                        S_cnt_d2 <= S_cnt_d2 + 4'd1;
                end
                else
                    S_cnt_d1 <= S_cnt_d1 + 4'd1;
            end
            else
                S_cnt_d0 <= S_cnt_d0 + 4'd1;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x_1d <= 12'd0;
            S_x_2d <= 12'd0;
            S_y_1d <= 12'd0;
            S_y_2d <= 12'd0;
        end
        else begin
            S_x_1d <= S_x;
            S_x_2d <= S_x_1d;
            S_y_1d <= S_y;
            S_y_2d <= S_y_1d;
        end
    end


    // 阶段A：状态字符选择（独立组合块，避免与 S_osd_hit 形成组合环路）
    always @(*) begin
        S_st_char    = 8'h20;
        S_st_local_x = 8'd0;
        S_st_local_y = 6'd0;
        S_st_row     = 4'd0;
        S_st_col     = 4'd0;
        S_st_active  = 1'b0;

        if (S_video_de_2d &&
            (S_x_2d >= 12'd40) && (S_x_2d < 12'd184) &&
            (S_y_2d >= 12'd170) && (S_y_2d < 12'd218)) begin
            S_st_local_x = S_x_2d - 12'd40;
            S_st_local_y = S_y_2d - 12'd170;
            S_st_row = F_scale3_row(S_st_local_y[5:0]);
            S_st_col = F_scale3_col(F_char_local_x(S_st_local_x));
            S_st_char = F_mode_name_char(I_style, F_char_idx3(S_st_local_x));
            S_st_active = 1'b1;
        end
    end

    // =====================================================================
    // 第 2 拍：几何判定 + 字符索引 + 候选使能（原来这些和查 ROM 挤在同一拍）
    // =====================================================================
    always @(*) begin
        S_logo_addr = 13'd0;
        S_dyn_x     = 12'd0;
        S_dyn_y     = 12'd0;

        S_pf_char    = 8'h20;
        S_pf_local_x = 5'd0;
        S_pf_local_y = 5'd0;
        S_pf_col     = 4'd0;
        S_pf_row     = 4'd0;
        S_pf_scale_x = 10'd0;
        S_pf_scale_y = 10'd0;
        S_en_pfx     = 1'b0;
        S_en_dgt     = 1'b0;

        S_dyn_digit_idx = 4'd0;
        S_dyn_digit_x   = 12'd0;
        S_dyn_dec_digit = 4'd0;

        S_h_logo = 1'b0;  S_c_logo = 24'd0;
        S_h_box  = 1'b0;
        S_h_t0   = 1'b0;  S_c_t0   = 24'd0;
        S_h_t1   = 1'b0;  S_c_t1   = 24'd0;
        S_h_mt   = 1'b0;  S_c_mt   = 24'd0;

        if(S_video_de_2d) begin
            if((S_x_2d >= LOGO_X) && (S_x_2d < (LOGO_X + LOGO_W)) &&
               (S_y_2d >= LOGO_Y) && (S_y_2d < (LOGO_Y + LOGO_H))) begin
                S_logo_x = S_x_2d - LOGO_X;
                S_logo_y = S_y_2d - LOGO_Y;
                // logo 图降成 80x80：地址 = (y/2)*80 + (x/2)，80 = 64 + 16
                S_logo_addr = ({2'd0, S_logo_y[11:1]} << 6) + ({2'd0, S_logo_y[11:1]} << 4)
                              + {7'd0, S_logo_x[11:1]};   // 160=128+32: 移位加法代替 DSP
            end
            // 命中判断用延迟两拍的标志，和 ROM 输出（也是两拍后）对齐
            if(S_logo_in2 && S_logo_pixel[24]) begin
                S_h_logo = 1'b1;
                S_c_logo = S_logo_pixel[23:0];
            end

            if((S_x_2d >= DYN_OSD_X) && (S_x_2d < (DYN_OSD_X + DYN_OSD_W)) &&
               (S_y_2d >= DYN_OSD_Y) && (S_y_2d < (DYN_OSD_Y + DYN_OSD_H))) begin
                S_dyn_x = S_x_2d - DYN_OSD_X;
                S_dyn_y = S_y_2d - DYN_OSD_Y;

                // Unified prefix render using one character library.
                // an:[0..23], lu:[24..47], C:[52..67], N:[68..83], T:[84..99], :[100..107]
                if((S_dyn_x < 12'd108) && (S_dyn_y < 12'd24)) begin
                    if(S_dyn_x < 12'd48) begin
                        if(S_dyn_y >= CN_Y_OFFSET)
                            S_pf_local_y = S_dyn_y - CN_Y_OFFSET;
                        else
                            S_pf_local_y = 5'd31;

                        if(S_dyn_x < 12'd24) begin
                            S_pf_char    = CHAR_AN;
                            S_pf_local_x = S_dyn_x[4:0];
                        end
                        else begin
                            S_pf_char    = CHAR_LU;
                            S_pf_local_x = S_dyn_x - 12'd24;
                        end

                        if((S_pf_local_x < 5'd24) && (S_pf_local_y < 5'd24)) begin
                            S_pf_scale_x = S_pf_local_x * 10'd16;
                            S_pf_scale_y = S_pf_local_y * 10'd16;
                            S_pf_col     = S_pf_scale_x / 10'd24;
                            S_pf_row     = S_pf_scale_y / 10'd24;
                            S_en_pfx     = 1'b1;
                        end
                    end
                    else if((S_dyn_x >= 12'd52) && (S_dyn_x < 12'd100) && (S_dyn_y >= 12'd4) && (S_dyn_y < 12'd20)) begin
                        if(S_dyn_x < 12'd68) begin
                            S_pf_char    = CHAR_C;
                            S_pf_local_x = S_dyn_x - 12'd52;
                        end
                        else if(S_dyn_x < 12'd84) begin
                            S_pf_char    = CHAR_N;
                            S_pf_local_x = S_dyn_x - 12'd68;
                        end
                        else begin
                            S_pf_char    = CHAR_T;
                            S_pf_local_x = S_dyn_x - 12'd84;
                        end
                        S_pf_local_y = S_dyn_y - 12'd4;
                        S_pf_col     = S_pf_local_x[3:0];
                        S_pf_row     = S_pf_local_y[3:0];
                        S_en_pfx     = 1'b1;
                    end
                    else if((S_dyn_x >= 12'd100) && (S_dyn_x < 12'd108) && (S_dyn_y >= 12'd4) && (S_dyn_y < 12'd20)) begin
                        S_pf_char    = CHAR_COLON;
                        S_pf_local_x = S_dyn_x - 12'd100;
                        S_pf_local_y = S_dyn_y - 12'd4;
                        S_pf_scale_x = S_pf_local_x * 10'd16;
                        S_pf_col     = S_pf_scale_x / 10'd8;
                        S_pf_row     = S_pf_local_y[3:0];
                        S_en_pfx     = 1'b1;
                    end
                end

                // Decimal 6 digits starts at x=110. Each digit is 15x24.
                if((S_dyn_x >= 12'd110) && (S_dyn_x < 12'd200)) begin
                    if(S_dyn_x < 12'd125) begin
                        S_dyn_digit_idx = 4'd0;
                        S_dyn_digit_x   = S_dyn_x - 12'd110;
                        S_dyn_dec_digit = S_cnt_d5;
                    end
                    else if(S_dyn_x < 12'd140) begin
                        S_dyn_digit_idx = 4'd1;
                        S_dyn_digit_x   = S_dyn_x - 12'd125;
                        S_dyn_dec_digit = S_cnt_d4;
                    end
                    else if(S_dyn_x < 12'd155) begin
                        S_dyn_digit_idx = 4'd2;
                        S_dyn_digit_x   = S_dyn_x - 12'd140;
                        S_dyn_dec_digit = S_cnt_d3;
                    end
                    else if(S_dyn_x < 12'd170) begin
                        S_dyn_digit_idx = 4'd3;
                        S_dyn_digit_x   = S_dyn_x - 12'd155;
                        S_dyn_dec_digit = S_cnt_d2;
                    end
                    else if(S_dyn_x < 12'd185) begin
                        S_dyn_digit_idx = 4'd4;
                        S_dyn_digit_x   = S_dyn_x - 12'd170;
                        S_dyn_dec_digit = S_cnt_d1;
                    end
                    else begin
                        S_dyn_digit_idx = 4'd5;
                        S_dyn_digit_x   = S_dyn_x - 12'd185;
                        S_dyn_dec_digit = S_cnt_d0;
                    end

                    // Render decimal digits through unified 16x16 character library.
                    if((S_dyn_digit_x < 12'd15) && (S_dyn_y < 12'd24)) begin
                        S_pf_char    = CHAR_0 + {4'd0, S_dyn_dec_digit};
                        S_pf_local_x = S_dyn_digit_x[4:0];
                        S_pf_local_y = S_dyn_y[4:0];
                        S_pf_scale_x = S_pf_local_x * 10'd16;
                        S_pf_scale_y = S_pf_local_y * 10'd16;
                        S_pf_col     = S_pf_scale_x / 10'd15;
                        S_pf_row     = S_pf_scale_y / 10'd24;
                        S_en_dgt     = 1'b1;
                    end
                end
            end

            // ---- 检测框（只在框边上点亮） ----
            if(I_box_valid &&
               (S_x_2d >= I_box_x0) && (S_x_2d <= I_box_x1) &&
               (S_y_2d >= I_box_y0) && (S_y_2d <= I_box_y1) &&
               ((S_x_2d == I_box_x0) || (S_x_2d == I_box_x1) ||
                (S_y_2d == I_box_y0) || (S_y_2d == I_box_y1))) begin
                S_h_box = 1'b1;
            end

            // ---- 跟踪目标包围盒（按类型着色）----
            if(draw_t0 &&
               (S_x_2d >= I_t0_x0) && (S_x_2d <= I_t0_x1) &&
               (S_y_2d >= I_t0_y0) && (S_y_2d <= I_t0_y1) &&
               ((S_x_2d == I_t0_x0) || (S_x_2d == I_t0_x1) ||
                (S_y_2d == I_t0_y0) || (S_y_2d == I_t0_y1))) begin
                S_h_t0 = 1'b1;
                S_c_t0 = F_type_color(I_t0_type);
            end
            if(draw_t1 &&
               (S_x_2d >= I_t1_x0) && (S_x_2d <= I_t1_x1) &&
               (S_y_2d >= I_t1_y0) && (S_y_2d <= I_t1_y1) &&
               ((S_x_2d == I_t1_x0) || (S_x_2d == I_t1_x1) ||
                (S_y_2d == I_t1_y0) || (S_y_2d == I_t1_y1))) begin
                S_h_t1 = 1'b1;
                S_c_t1 = F_type_color(I_t1_type);
            end

            // ---- 多目标颜色块：3 色 x 3 目标 画框 ----
            // 目标打包成 44 位：{valid, x0[10:0], y0[9:0], x1[10:0], y1[9:0], 1'b0}
            for (cc = 0; cc < 4; cc = cc + 1) begin
                for (tt = 0; tt < 3; tt = tt + 1) begin
                    mt_tmp = I_mt_tgt[cc*132 + tt*44 +: 44];
                    if ((!S_h_mt) && mt_tmp[43] &&
                        (S_x_2d >= mt_tmp[42:32]) && (S_x_2d <= mt_tmp[21:11]) &&
                        (S_y_2d >= mt_tmp[31:22]) && (S_y_2d <= mt_tmp[10:1]) &&
                        ((S_x_2d == mt_tmp[42:32]) || (S_x_2d == mt_tmp[21:11]) ||
                         (S_y_2d == mt_tmp[31:22]) || (S_y_2d == mt_tmp[10:1]))) begin
                        S_h_mt = 1'b1;
                        // bit0 = 形状位：圆形用同色系高亮色
                        S_c_mt = mt_tmp[0] ? F_mt_color_c(cc[1:0])
                                           : F_mt_color  (cc[1:0]);
                    end
                end
            end
        end
    end

    // =====================================================================
    // 第 2 拍 → 第 3 拍：把字符索引和几何命中标志打一拍
    //   坐标计数器、视频数据、同步信号同步再延一拍，叠加位置保持不变。
    // =====================================================================
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_pf_char_3    <= 8'h20;  S_pf_row_3  <= 4'd0;  S_pf_col_3   <= 4'd0;
            S_pf_en_pfx_3  <= 1'b0;   S_pf_en_dgt_3 <= 1'b0;
            S_st_char_3    <= 8'h20;  S_st_row_3  <= 4'd0;  S_st_col_3   <= 4'd0;
            S_st_active_3  <= 1'b0;
            S_an_char_3    <= 8'h20;  S_an_row_3  <= 4'd0;
            S_an_lx_3      <= 4'd0;   S_an_en_3   <= 1'b0;  S_an_c_3     <= 24'h00FFA0;
            S_h_logo_3     <= 1'b0;   S_c_logo_3  <= 24'd0;
            S_h_box_3      <= 1'b0;
            S_h_t0_3       <= 1'b0;   S_c_t0_3    <= 24'd0;
            S_h_t1_3       <= 1'b0;   S_c_t1_3    <= 24'd0;
            S_h_mt_3       <= 1'b0;   S_c_mt_3    <= 24'd0;
        end
        else begin
            S_pf_char_3    <= S_pf_char;
            S_pf_row_3     <= S_pf_row;
            S_pf_col_3     <= S_pf_col;
            S_pf_en_pfx_3  <= S_en_pfx;
            S_pf_en_dgt_3  <= S_en_dgt;
            S_st_char_3    <= S_st_char;
            S_st_row_3     <= S_st_row;
            S_st_col_3     <= S_st_col;
            S_st_active_3  <= S_st_active;
            S_an_char_3    <= S_an_char;
            S_an_row_3     <= S_an_row;
            S_an_lx_3      <= S_an_lx[3:0];
            S_an_en_3      <= S_an_en;
            S_an_c_3       <= S_an_c;
            S_h_logo_3     <= S_h_logo;
            S_c_logo_3     <= S_c_logo;
            S_h_box_3      <= S_h_box;
            S_h_t0_3       <= S_h_t0;
            S_c_t0_3       <= S_c_t0;
            S_h_t1_3       <= S_h_t1;
            S_c_t1_3       <= S_c_t1;
            S_h_mt_3       <= S_h_mt;
            S_c_mt_3       <= S_c_mt;
        end
    end

    // =====================================================================
    // 第 3 拍：查字模 ROM + 并行命中优先链
    //   优先级与原来完全一致：
    //     logo > 动态前缀 > 状态文字 > 检测框 > 跟踪框t0 > 跟踪框t1 > 彩色块 > 标注
    // =====================================================================
    always @(*) begin
        S_h_dyn = (S_pf_en_pfx_3 || S_pf_en_dgt_3) && S_pf_row_bits[15 - S_pf_col_3];
        S_h_st  = S_st_active_3 && S_st_row_bits[15 - S_st_col_3];
        S_h_an  = S_an_en_3     && S_an_row_bits[15 - S_an_lx_3];

        S_sel1 = S_h_logo_3;
        S_sel2 = S_h_dyn   && !S_h_logo_3;
        S_sel3 = S_h_st    && !S_h_logo_3 && !S_h_dyn;
        S_sel4 = S_h_box_3 && !S_h_logo_3 && !S_h_dyn && !S_h_st;
        S_sel5 = S_h_t0_3  && !S_h_logo_3 && !S_h_dyn && !S_h_st && !S_h_box_3;
        S_sel6 = S_h_t1_3  && !S_h_logo_3 && !S_h_dyn && !S_h_st && !S_h_box_3 && !S_h_t0_3;
        S_sel7 = S_h_mt_3  && !S_h_logo_3 && !S_h_dyn && !S_h_st && !S_h_box_3
                           && !S_h_t0_3  && !S_h_t1_3;
        S_sel8 = S_h_an    && !S_h_logo_3 && !S_h_dyn && !S_h_st && !S_h_box_3
                           && !S_h_t0_3  && !S_h_t1_3 && !S_h_mt_3;

        S_osd_hit = S_sel1 | S_sel2 | S_sel3 | S_sel4 |
                    S_sel5 | S_sel6 | S_sel7 | S_sel8;

        S_osd_color = ({24{S_sel1}} & S_c_logo_3)
                    | ({24{S_sel2}} & COLOR_DYN_TEXT)
                    | ({24{S_sel3}} & 24'h00FFA0)
                    | ({24{S_sel4}} & 24'h808080)
                    | ({24{S_sel5}} & S_c_t0_3)
                    | ({24{S_sel6}} & S_c_t1_3)
                    | ({24{S_sel7}} & S_c_mt_3)
                    | ({24{S_sel8}} & S_an_c_3);
    end

    // 进-2：跟踪目标信息标注（ID + 类别 + X/Y，坐标十六进制）
    // ---- 端到端延迟 BCD：5 位十进制（单位微秒）----
    //   用减法状态机代替除法器：每帧按 10000/1000/100/10 逐级减，
    //   最多 6+10+10+10 次循环，一帧有 20 多万拍，完全不占时序。
    reg [19:0] e2e_work;
    reg [2:0]  e2e_state;
    reg [3:0]  S_e2e_d4, S_e2e_d3, S_e2e_d2, S_e2e_d1, S_e2e_d0;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            e2e_work  <= 20'd0;
            e2e_state <= 3'd0;
            S_e2e_d4 <= 4'd0; S_e2e_d3 <= 4'd0;
            S_e2e_d2 <= 4'd0; S_e2e_d1 <= 4'd0; S_e2e_d0 <= 4'd0;
        end
        else if (I_video_user) begin
            e2e_work  <= (I_e2e_us > 20'd65535) ? 20'd65535 : I_e2e_us;
            e2e_state <= 3'd1;
            S_e2e_d4 <= 4'd0; S_e2e_d3 <= 4'd0;
            S_e2e_d2 <= 4'd0; S_e2e_d1 <= 4'd0; S_e2e_d0 <= 4'd0;
        end
        else begin
            case (e2e_state)
                3'd1: if (e2e_work >= 20'd10000) begin
                          e2e_work <= e2e_work - 20'd10000;
                          S_e2e_d4 <= S_e2e_d4 + 4'd1;
                      end else e2e_state <= 3'd2;
                3'd2: if (e2e_work >= 20'd1000) begin
                          e2e_work <= e2e_work - 20'd1000;
                          S_e2e_d3 <= S_e2e_d3 + 4'd1;
                      end else e2e_state <= 3'd3;
                3'd3: if (e2e_work >= 20'd100) begin
                          e2e_work <= e2e_work - 20'd100;
                          S_e2e_d2 <= S_e2e_d2 + 4'd1;
                      end else e2e_state <= 3'd4;
                3'd4: if (e2e_work >= 20'd10) begin
                          e2e_work <= e2e_work - 20'd10;
                          S_e2e_d1 <= S_e2e_d1 + 4'd1;
                      end else e2e_state <= 3'd5;
                3'd5: begin
                          S_e2e_d0  <= e2e_work[3:0];
                          e2e_state <= 3'd0;
                      end
                default: e2e_state <= 3'd0;
            endcase
        end
    end

    // timing3：本块只算字符/行号和"是否落在标注行带内"，
    //          字模查表和命中判定挪到第 3 拍（见上面的并行优先链）。
    always @(*) begin
        S_an_char = 8'h20;
        S_an_row  = 4'd0;
        S_an_en   = 1'b0;
        S_an_c    = 24'h00FFA0;
        if (S_video_de_2d && (S_x_2d >= 12'd40) && (S_x_2d < 12'd296)) begin
            S_an_row = S_y_2d[3:0];
            S_an_en  = ((S_y_2d >= 12'd224) && (S_y_2d < 12'd240)) ||
                       ((S_y_2d >= 12'd256) && (S_y_2d < 12'd272)) ||
                       ((S_y_2d >= 12'd272) && (S_y_2d < 12'd288)) ||   // 批2：相位倒计时
                       ((S_y_2d >= 12'd288) && (S_y_2d < 12'd304)) ||
                       ((S_y_2d >= 12'd320) && (S_y_2d < 12'd336)) ||
                       ((S_y_2d >= 12'd336) && (S_y_2d < 12'd352));
            if      ((S_y_2d >= 12'd256) && (S_y_2d < 12'd272)) S_an_c = 24'hFFFFFF;
            else if ((S_y_2d >= 12'd320) && (S_y_2d < 12'd336)) S_an_c = 24'h00E0FF;
            else if ((S_y_2d >= 12'd336) && (S_y_2d < 12'd352)) S_an_c = 24'hFFA8FF;
            else if ((S_y_2d >= 12'd272) && (S_y_2d < 12'd288)) begin
                // 批2：倒计时那一行跟着相位变色 —— 扫一眼就知道现在是什么灯
                case (I_cd_phase)
                    2'd0:    S_an_c = 24'hFF5050;   // 红相位
                    2'd1:    S_an_c = 24'hFFE000;   // 黄相位
                    default: S_an_c = 24'h40FF60;   // 绿相位
                endcase
            end
            else                                               S_an_c = 24'h00FFA0;
            if ((S_y_2d >= 12'd224) && (S_y_2d < 12'd240) && I_t0_valid) begin
                case (S_an_lx[7:4])
                    4'd0:  S_an_char = 8'h49;
                    4'd1:  S_an_char = 8'h44;
                    4'd2:  S_an_char = 8'h30 + {6'd0, I_t0_id};
                    4'd3:  S_an_char = 8'h20;
                    4'd4:  S_an_char = F_cat0(I_t0_type);
                    4'd5:  S_an_char = F_cat1(I_t0_type);
                    4'd6:  S_an_char = 8'h20;
                    4'd7:  S_an_char = 8'h58;
                    4'd8:  S_an_char = F_hex_ascii(I_t0_x0[11:8]);
                    4'd9:  S_an_char = F_hex_ascii(I_t0_x0[7:4]);
                    4'd10: S_an_char = F_hex_ascii(I_t0_x0[3:0]);
                    4'd11: S_an_char = 8'h20;
                    4'd12: S_an_char = 8'h59;
                    4'd13: S_an_char = F_hex_ascii(I_t0_y0[11:8]);
                    4'd14: S_an_char = F_hex_ascii(I_t0_y0[7:4]);
                    4'd15: S_an_char = F_hex_ascii(I_t0_y0[3:0]);
                    default: S_an_char = 8'h20;
                endcase
            end
            else if ((S_y_2d >= 12'd256) && (S_y_2d < 12'd272)) begin
                // 多目标个数： R<n> G<n> Y<n> T<n>
                case (S_an_lx[7:4])
                    4'd0:  S_an_char = 8'h52;                            // R
                    4'd1:  S_an_char = 8'h30 + {4'd0, I_mt_num[3:0]};
                    4'd2:  S_an_char = 8'h20;
                    4'd3:  S_an_char = 8'h47;                            // G
                    4'd4:  S_an_char = 8'h30 + {4'd0, I_mt_num[7:4]};
                    4'd5:  S_an_char = 8'h20;
                    4'd6:  S_an_char = 8'h59;                            // Y
                    4'd7:  S_an_char = 8'h30 + {4'd0, I_mt_num[11:8]};
                    4'd8:  S_an_char = 8'h20;
                    4'd9:  S_an_char = 8'h54;                            // T
                    4'd10: S_an_char = 8'h30 + {4'd0, mt_tot};
                    4'd12: S_an_char = 8'h4D;   // M
                    4'd13: S_an_char = 8'h30 + {4'd0, I_mt_num[15:12]};
                    default: S_an_char = 8'h20;
                endcase
            end
            else if ((S_y_2d >= 12'd320) && (S_y_2d < 12'd336)) begin
                // 端到端延迟： E2E 33200US
                case (S_an_lx[7:4])
                    4'd0:  S_an_char = 8'h45;                            // E
                    4'd1:  S_an_char = 8'h32;                            // 2
                    4'd2:  S_an_char = 8'h45;                            // E
                    4'd3:  S_an_char = 8'h20;
                    4'd4:  S_an_char = 8'h30 + S_e2e_d4;
                    4'd5:  S_an_char = 8'h30 + S_e2e_d3;
                    4'd6:  S_an_char = 8'h30 + S_e2e_d2;
                    4'd7:  S_an_char = 8'h30 + S_e2e_d1;
                    4'd8:  S_an_char = 8'h30 + S_e2e_d0;
                    4'd9:  S_an_char = 8'h55;                            // U
                    4'd10: S_an_char = 8'h53;                            // S
                    default: S_an_char = 8'h20;
                endcase
            end
            else if ((S_y_2d >= 12'd288) && (S_y_2d < 12'd304)) begin
                // 进-3/扩-4：处理延迟（拍，1拍≈16.668ns）
                case (S_an_lx[7:4])
                    4'd0:  S_an_char = 8'h4C;              // L
                    4'd1:  S_an_char = 8'h41;              // A
                    4'd2:  S_an_char = 8'h54;              // T
                    4'd3:  S_an_char = 8'h20;
                    4'd4:  S_an_char = 8'h30 + S_lat_h;
                    4'd5:  S_an_char = 8'h30 + S_lat_t;
                    4'd6:  S_an_char = 8'h30 + S_lat_u;
                    // 单位：微秒
                    4'd7:  S_an_char = 8'h55;
                    4'd8:  S_an_char = 8'h53;
                    default: S_an_char = 8'h20;
                endcase
            end
            else if ((S_y_2d >= 12'd272) && (S_y_2d < 12'd288)) begin
                // 批2：交通灯相位倒计时   CD <相位字母> <两位秒>S
                //   CD = countdown；还没锁定过相位时数字留空（只显示 CD）
                case (S_an_lx[7:4])
                    4'd0:  S_an_char = 8'h43;   // C
                    4'd1:  S_an_char = 8'h44;   // D
                    4'd2:  S_an_char = 8'h20;
                    4'd3:  S_an_char = (I_cd_phase == 2'd0) ? 8'h52 :   // R
                                       (I_cd_phase == 2'd1) ? 8'h59 :   // Y
                                                              8'h47;    // G
                    4'd4:  S_an_char = 8'h20;
                    4'd5:  S_an_char = I_cd_vld ? (8'h30 + I_cd_t) : 8'h20;
                    4'd6:  S_an_char = I_cd_vld ? (8'h30 + I_cd_u) : 8'h20;
                    4'd7:  S_an_char = 8'h53;   // S（秒）
                    // 批 3：空闲降频时在这一行末尾点亮 LP（low power），
                    // 让"按需降频"在画面上也看得见，不用只靠串口数
                    4'd9:  S_an_char = I_lp_sleep ? 8'h4C : 8'h20;   // L
                    4'd10: S_an_char = I_lp_sleep ? 8'h50 : 8'h20;   // P
                    default: S_an_char = 8'h20;
                endcase
            end
            else if ((S_y_2d >= 12'd336) && (S_y_2d < 12'd352)) begin
                // 圆形目标个数： CIR <n>
                case (S_an_lx[7:4])
                    4'd0:  S_an_char = 8'h43;   // C
                    4'd1:  S_an_char = 8'h49;   // I
                    4'd2:  S_an_char = 8'h52;   // R
                    4'd3:  S_an_char = 8'h20;
                    4'd4:  S_an_char = 8'h30 + {4'd0, S_circ_num};
                    4'd5:  S_an_char = 8'h20;
                    4'd6:  S_an_char = 8'h51;   // Q
                    4'd7:  S_an_char = 8'h20;
                    4'd8:  S_an_char = 8'h30 + {4'd0, I_conf};
                    default: S_an_char = 8'h20;
                endcase
            end
        end
    end

    // 进-3/扩-4：处理延迟 BCD（单位：微秒）
    // 用减法状态机代替除法器，省 LUT，也缩短 HDMI 域路径。
    reg [15:0] lat_work;
    reg [1:0]  lat_state;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            lat_work <= 16'd0;
            lat_state <= 2'd0;
            S_lat_h <= 4'd0;
            S_lat_t <= 4'd0;
            S_lat_u <= 4'd0;
        end
        else if (I_video_user) begin
            lat_work <= (I_latency > 16'd999) ? 16'd999 : I_latency;
            lat_state <= 2'd1;
            S_lat_h <= 4'd0;
            S_lat_t <= 4'd0;
            S_lat_u <= 4'd0;
        end
        else begin
            case (lat_state)
                2'd1: begin
                    if (lat_work >= 16'd100) begin
                        lat_work <= lat_work - 16'd100;
                        S_lat_h  <= S_lat_h + 4'd1;
                    end
                    else
                        lat_state <= 2'd2;
                end
                2'd2: begin
                    if (lat_work >= 16'd10) begin
                        lat_work <= lat_work - 16'd10;
                        S_lat_t  <= S_lat_t + 4'd1;
                    end
                    else begin
                        S_lat_u  <= lat_work[3:0];
                        lat_state <= 2'd3;
                    end
                end
                default: lat_state <= 2'd0;
            endcase
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_video_vsync_1d <= 1'b0;
            S_video_vsync_2d <= 1'b0;
            S_video_vsync_3d <= 1'b0;
            S_video_hsync_1d <= 1'b0;
            S_video_hsync_2d <= 1'b0;
            S_video_hsync_3d <= 1'b0;
            S_video_de_1d    <= 1'b0;
            S_video_de_2d    <= 1'b0;
            S_video_de_3d    <= 1'b0;
            S_video_data_3d  <= 24'd0;
            O_hdmi_vsync     <= 1'b0;
            O_hdmi_hsync     <= 1'b0;
            O_hdmi_de        <= 1'b0;
            O_hdmi_data      <= 24'd0;
        end
        else begin
            S_video_vsync_1d <= I_video_vsync;
            S_video_vsync_2d <= S_video_vsync_1d;
            S_video_vsync_3d <= S_video_vsync_2d;
            O_hdmi_vsync     <= S_video_vsync_3d;

            S_video_hsync_1d <= I_video_hsync;
            S_video_hsync_2d <= S_video_hsync_1d;
            S_video_hsync_3d <= S_video_hsync_2d;
            O_hdmi_hsync     <= S_video_hsync_3d;

            S_video_de_1d    <= I_video_de;
            S_video_de_2d    <= S_video_de_1d;
            S_video_de_3d    <= S_video_de_2d;
            O_hdmi_de        <= S_video_de_3d;
            S_video_data_1d  <= I_video_rd_data;
            S_video_data_2d  <= S_video_data_1d;
            S_video_data_3d  <= S_video_data_2d;

            if(DEBUG_MODE == 1) begin
                if(S_video_de_2d) begin
                    if(S_x < 12'd128)
                        O_hdmi_data <= 24'hff0000;
                    else if(S_x < 12'd256)
                        O_hdmi_data <= 24'h00ff00;
                    else if(S_x < 12'd384)
                        O_hdmi_data <= 24'h0000ff;
                    else if(S_x < 12'd512)
                        O_hdmi_data <= 24'hffff00;
                    else if(S_x < 12'd640)
                        O_hdmi_data <= 24'h00ffff;
                    else if(S_x < 12'd768)
                        O_hdmi_data <= 24'hff00ff;
                    else if(S_x < 12'd896)
                        O_hdmi_data <= 24'hffffff;
                    else
                        O_hdmi_data <= 24'h202020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 2) begin
                if(S_video_de_2d) begin
                    case (I_debug_status)
                        4'b0000: O_hdmi_data <= 24'h000000;
                        4'b0001: O_hdmi_data <= 24'hff0000;
                        4'b0011: O_hdmi_data <= 24'hffff00;
                        4'b0111: O_hdmi_data <= 24'h00ff00;
                        4'b1111: O_hdmi_data <= I_video_rd_data;
                        default: O_hdmi_data <= 24'h0000ff;
                    endcase
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 3) begin
                if(S_video_de_2d) begin
                    if(S_x < 12'd256)
                        O_hdmi_data <= I_debug_status[0] ? 24'hff8000 : 24'h201000;
                    else if(S_x < 12'd512)
                        O_hdmi_data <= I_debug_status[1] ? 24'hffff00 : 24'h202000;
                    else if(S_x < 12'd768)
                        O_hdmi_data <= I_debug_status[2] ? 24'hff0000 : 24'h200000;
                    else
                        O_hdmi_data <= I_debug_status[3] ? 24'h00ffff : 24'h002020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else begin
                if(S_osd_hit)
                    O_hdmi_data <= S_osd_color;
                else
                    O_hdmi_data <= S_video_data_3d;
            end
        end
    end

endmodule
