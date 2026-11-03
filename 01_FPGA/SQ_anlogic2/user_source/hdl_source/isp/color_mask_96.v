// ===========================================================================
// color_mask_96.v
// M2：颜色掩码引擎
//
// 输入 96bit = 4 个 RGB888 像素（与 isp 链路同一节拍）：
//     p0 = I_tdata[95:72] = 本拍第 0 列
//     p1 = I_tdata[71:48] = 本拍第 1 列
//     p2 = I_tdata[47:24] = 本拍第 2 列
//     p3 = I_tdata[23: 0] = 本拍第 3 列
// 每个 24bit 像素 = {R[23:16], G[15:8], B[7:0]}
//
// 输出 4bit 掩码，bit3..0 依次对应第 0..3 列；1 表示命中所选颜色。
// I_color_sel：0红 1蓝 2黄 3绿 4白 5黑。
// 阈值用 parameter 给出；M5/M6 再把它们换成寄存器、由上位机下发。
// ===========================================================================

`timescale 1ns/1ps

module color_mask_96 #(
    parameter [7:0] RED_MIN  = 8'd80,   // 红：R 下限
    parameter [7:0] RED_GAP  = 8'd40,   // 红：R 比 G/B 高出多少才算红
    parameter [7:0] BLUE_MIN = 8'd90,   // 蓝：B 下限
    parameter [7:0] BLUE_GAP = 8'd35,   // 蓝：B 比 R/G 高出多少才算蓝
    parameter [7:0] GRN_MIN  = 8'd60,   // 绿：G 下限
    parameter [7:0] GRN_GAP  = 8'd12,   // 绿：G 比 R/B 高出多少才算绿
    parameter [7:0] YEL_MIN  = 8'd45,   // 黄：R、G 下限
    parameter [7:0] YEL_BMAX = 8'd200,  // 黄：B 上限（放宽，避免暗黄牌漏检）
    parameter [7:0] YEL_BDIFF = 8'd5,  // 黄：R/G 至少比 B 高这么多
    parameter [7:0] WHT_MIN  = 8'd180,  // 白：三通道都高于此值
    parameter [7:0] BLK_MAX  = 8'd70,   // 黑：三通道都低于此值
    // 阈值来源：0 = 用上面的 parameter 常量（交通灯那几路，编译期就折叠）
    //           1 = 用 I_thr 端口上的运行时值（颜色块那一路，串口可调）
    parameter integer RELATIVE_COLOR = 0,
    parameter integer THR_LIVE = 0
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire [2:0]  I_color_sel,
    input  wire        I_tvalid,
    input  wire [95:0] I_tdata,
    output reg  [3:0]  O_mask,
    // I_thr = {red_min, red_gap, grn_min, grn_gap, yel_min, yel_bdiff, yel_bmax}
    input  wire [55:0] I_thr
);

    // Relative-mode hue boundaries are exclusive: a boundary pixel belongs
    // to red/green, preventing a false yellow conflict on a single lamp.
    // I_thr_live=0 时下面这些都是常量，综合会直接折叠，逻辑与原来完全一致
    wire [7:0] red_min   = THR_LIVE ? I_thr[55:48] : RED_MIN;
    wire [7:0] red_gap   = THR_LIVE ? I_thr[47:40] : RED_GAP;
    wire [7:0] grn_min   = THR_LIVE ? I_thr[39:32] : GRN_MIN;
    wire [7:0] grn_gap   = THR_LIVE ? I_thr[31:24] : GRN_GAP;
    wire [7:0] yel_min   = THR_LIVE ? I_thr[23:16] : YEL_MIN;
    wire [7:0] yel_bdiff = THR_LIVE ? I_thr[15: 8] : YEL_BDIFF;
    wire [7:0] yel_bmax  = THR_LIVE ? I_thr[ 7: 0] : YEL_BMAX;

    wire [23:0] p0 = I_tdata[95:72];
    wire [23:0] p1 = I_tdata[71:48];
    wire [23:0] p2 = I_tdata[47:24];
    wire [23:0] p3 = I_tdata[23: 0];

    wire [7:0] r0 = p0[23:16], g0 = p0[15:8], b0 = p0[7:0];
    wire [7:0] r1 = p1[23:16], g1 = p1[15:8], b1 = p1[7:0];
    wire [7:0] r2 = p2[23:16], g2 = p2[15:8], b2 = p2[7:0];
    wire [7:0] r3 = p3[23:16], g3 = p3[15:8], b3 = p3[7:0];

    // Nine-bit channel differences retain the non-wrapping sum/compare rule:
    // a >= b + max(t,a>>3) iff d=a-b is nonnegative, d>=t, d>=a>>3.
    // Sharing d also removes the live-threshold max mux from each comparison.
    // Blue/white/black and the hue-ratio boundary tests stay unchanged.
    wire [8:0] drg0={1'b0,r0}-{1'b0,g0};
    wire [8:0] dgr0={1'b0,g0}-{1'b0,r0};
    wire [8:0] drb0={1'b0,r0}-{1'b0,b0};
    wire [8:0] dgb0={1'b0,g0}-{1'b0,b0};
    wire [7:0] ylevel0=(r0<g0)?r0:g0;
    wire red0 = (r0 >= red_min) &&
                (!drg0[8] && drg0[7:0]>=red_gap && (!RELATIVE_COLOR || drg0[7:0]>=(r0>>3))) &&
                (!drb0[8] && drb0[7:0]>=red_gap && (!RELATIVE_COLOR || drb0[7:0]>=(r0>>3))) &&
                // 比值约束：G <= 0.75R（红要"红得明显"，挡掉过曝红落进黄）
                ({2'd0, g0, 2'b00} <= {2'd0, r0} + {1'd0, r0, 1'b0});
    wire [8:0] drg1={1'b0,r1}-{1'b0,g1};
    wire [8:0] dgr1={1'b0,g1}-{1'b0,r1};
    wire [8:0] drb1={1'b0,r1}-{1'b0,b1};
    wire [8:0] dgb1={1'b0,g1}-{1'b0,b1};
    wire [7:0] ylevel1=(r1<g1)?r1:g1;
    wire red1 = (r1 >= red_min) &&
                (!drg1[8] && drg1[7:0]>=red_gap && (!RELATIVE_COLOR || drg1[7:0]>=(r1>>3))) &&
                (!drb1[8] && drb1[7:0]>=red_gap && (!RELATIVE_COLOR || drb1[7:0]>=(r1>>3))) &&
                // 比值约束：G <= 0.75R（红要"红得明显"，挡掉过曝红落进黄）
                ({2'd0, g1, 2'b00} <= {2'd0, r1} + {1'd0, r1, 1'b0});
    wire [8:0] drg2={1'b0,r2}-{1'b0,g2};
    wire [8:0] dgr2={1'b0,g2}-{1'b0,r2};
    wire [8:0] drb2={1'b0,r2}-{1'b0,b2};
    wire [8:0] dgb2={1'b0,g2}-{1'b0,b2};
    wire [7:0] ylevel2=(r2<g2)?r2:g2;
    wire red2 = (r2 >= red_min) &&
                (!drg2[8] && drg2[7:0]>=red_gap && (!RELATIVE_COLOR || drg2[7:0]>=(r2>>3))) &&
                (!drb2[8] && drb2[7:0]>=red_gap && (!RELATIVE_COLOR || drb2[7:0]>=(r2>>3))) &&
                // 比值约束：G <= 0.75R（红要"红得明显"，挡掉过曝红落进黄）
                ({2'd0, g2, 2'b00} <= {2'd0, r2} + {1'd0, r2, 1'b0});
    wire [8:0] drg3={1'b0,r3}-{1'b0,g3};
    wire [8:0] dgr3={1'b0,g3}-{1'b0,r3};
    wire [8:0] drb3={1'b0,r3}-{1'b0,b3};
    wire [8:0] dgb3={1'b0,g3}-{1'b0,b3};
    wire [7:0] ylevel3=(r3<g3)?r3:g3;
    wire red3 = (r3 >= red_min) &&
                (!drg3[8] && drg3[7:0]>=red_gap && (!RELATIVE_COLOR || drg3[7:0]>=(r3>>3))) &&
                (!drb3[8] && drb3[7:0]>=red_gap && (!RELATIVE_COLOR || drb3[7:0]>=(r3>>3))) &&
                // 比值约束：G <= 0.75R（红要"红得明显"，挡掉过曝红落进黄）
                ({2'd0, g3, 2'b00} <= {2'd0, r3} + {1'd0, r3, 1'b0});

    wire blu0 = (b0 >= BLUE_MIN) &&
                ({1'b0, b0} >= {1'b0, r0} + {1'b0, BLUE_GAP}) &&
                ({1'b0, b0} >= {1'b0, g0} + {1'b0, BLUE_GAP});
    wire blu1 = (b1 >= BLUE_MIN) &&
                ({1'b0, b1} >= {1'b0, r1} + {1'b0, BLUE_GAP}) &&
                ({1'b0, b1} >= {1'b0, g1} + {1'b0, BLUE_GAP});
    wire blu2 = (b2 >= BLUE_MIN) &&
                ({1'b0, b2} >= {1'b0, r2} + {1'b0, BLUE_GAP}) &&
                ({1'b0, b2} >= {1'b0, g2} + {1'b0, BLUE_GAP});
    wire blu3 = (b3 >= BLUE_MIN) &&
                ({1'b0, b3} >= {1'b0, r3} + {1'b0, BLUE_GAP}) &&
                ({1'b0, b3} >= {1'b0, g3} + {1'b0, BLUE_GAP});

    wire grn0 = (g0 >= grn_min) &&
                (!dgr0[8] && dgr0[7:0]>=grn_gap && (!RELATIVE_COLOR || dgr0[7:0]>=(g0>>3))) &&
                (!dgb0[8] && dgb0[7:0]>=grn_gap && (!RELATIVE_COLOR || dgb0[7:0]>=(g0>>3))) &&
                // 比值约束：R <= 0.75G（绿要"绿得明显"）
                ({2'd0, r0, 2'b00} <= {2'd0, g0} + {1'd0, g0, 1'b0});
    wire grn1 = (g1 >= grn_min) &&
                (!dgr1[8] && dgr1[7:0]>=grn_gap && (!RELATIVE_COLOR || dgr1[7:0]>=(g1>>3))) &&
                (!dgb1[8] && dgb1[7:0]>=grn_gap && (!RELATIVE_COLOR || dgb1[7:0]>=(g1>>3))) &&
                // 比值约束：R <= 0.75G（绿要"绿得明显"）
                ({2'd0, r1, 2'b00} <= {2'd0, g1} + {1'd0, g1, 1'b0});
    wire grn2 = (g2 >= grn_min) &&
                (!dgr2[8] && dgr2[7:0]>=grn_gap && (!RELATIVE_COLOR || dgr2[7:0]>=(g2>>3))) &&
                (!dgb2[8] && dgb2[7:0]>=grn_gap && (!RELATIVE_COLOR || dgb2[7:0]>=(g2>>3))) &&
                // 比值约束：R <= 0.75G（绿要"绿得明显"）
                ({2'd0, r2, 2'b00} <= {2'd0, g2} + {1'd0, g2, 1'b0});
    wire grn3 = (g3 >= grn_min) &&
                (!dgr3[8] && dgr3[7:0]>=grn_gap && (!RELATIVE_COLOR || dgr3[7:0]>=(g3>>3))) &&
                (!dgb3[8] && dgb3[7:0]>=grn_gap && (!RELATIVE_COLOR || dgb3[7:0]>=(g3>>3))) &&
                // 比值约束：R <= 0.75G（绿要"绿得明显"）
                ({2'd0, r3, 2'b00} <= {2'd0, g3} + {1'd0, g3, 1'b0});

    wire yel0 = (r0 >= yel_min) && (g0 >= yel_min) && (b0 <= yel_bmax) &&
                (!drb0[8] && drb0[7:0]>=yel_bdiff && (!RELATIVE_COLOR || drb0[7:0]>=(ylevel0>>3))) &&
                (!dgb0[8] && dgb0[7:0]>=yel_bdiff && (!RELATIVE_COLOR || dgb0[7:0]>=(ylevel0>>3))) &&
                // 比值约束：0.75 <= G/R <= 1.33（黄必须 R、G 相当，
                // 挡掉"过曝红"(G/R~0.7) 与"带红泄漏的绿"(G/R~1.5)）
                (RELATIVE_COLOR ? ({2'd0, g0, 2'b00} > {2'd0, r0} + {1'd0, r0, 1'b0}) : ({2'd0, g0, 2'b00} >= {2'd0, r0} + {1'd0, r0, 1'b0})) &&
                (RELATIVE_COLOR ? ({2'd0, g0} + {1'd0, g0, 1'b0} < {2'd0, r0, 2'b00}) : ({2'd0, g0} + {1'd0, g0, 1'b0} <= {2'd0, r0, 2'b00}));
    wire yel1 = (r1 >= yel_min) && (g1 >= yel_min) && (b1 <= yel_bmax) &&
                (!drb1[8] && drb1[7:0]>=yel_bdiff && (!RELATIVE_COLOR || drb1[7:0]>=(ylevel1>>3))) &&
                (!dgb1[8] && dgb1[7:0]>=yel_bdiff && (!RELATIVE_COLOR || dgb1[7:0]>=(ylevel1>>3))) &&
                // 比值约束：0.75 <= G/R <= 1.33（黄必须 R、G 相当，
                // 挡掉"过曝红"(G/R~0.7) 与"带红泄漏的绿"(G/R~1.5)）
                (RELATIVE_COLOR ? ({2'd0, g1, 2'b00} > {2'd0, r1} + {1'd0, r1, 1'b0}) : ({2'd0, g1, 2'b00} >= {2'd0, r1} + {1'd0, r1, 1'b0})) &&
                (RELATIVE_COLOR ? ({2'd0, g1} + {1'd0, g1, 1'b0} < {2'd0, r1, 2'b00}) : ({2'd0, g1} + {1'd0, g1, 1'b0} <= {2'd0, r1, 2'b00}));
    wire yel2 = (r2 >= yel_min) && (g2 >= yel_min) && (b2 <= yel_bmax) &&
                (!drb2[8] && drb2[7:0]>=yel_bdiff && (!RELATIVE_COLOR || drb2[7:0]>=(ylevel2>>3))) &&
                (!dgb2[8] && dgb2[7:0]>=yel_bdiff && (!RELATIVE_COLOR || dgb2[7:0]>=(ylevel2>>3))) &&
                // 比值约束：0.75 <= G/R <= 1.33（黄必须 R、G 相当，
                // 挡掉"过曝红"(G/R~0.7) 与"带红泄漏的绿"(G/R~1.5)）
                (RELATIVE_COLOR ? ({2'd0, g2, 2'b00} > {2'd0, r2} + {1'd0, r2, 1'b0}) : ({2'd0, g2, 2'b00} >= {2'd0, r2} + {1'd0, r2, 1'b0})) &&
                (RELATIVE_COLOR ? ({2'd0, g2} + {1'd0, g2, 1'b0} < {2'd0, r2, 2'b00}) : ({2'd0, g2} + {1'd0, g2, 1'b0} <= {2'd0, r2, 2'b00}));
    wire yel3 = (r3 >= yel_min) && (g3 >= yel_min) && (b3 <= yel_bmax) &&
                (!drb3[8] && drb3[7:0]>=yel_bdiff && (!RELATIVE_COLOR || drb3[7:0]>=(ylevel3>>3))) &&
                (!dgb3[8] && dgb3[7:0]>=yel_bdiff && (!RELATIVE_COLOR || dgb3[7:0]>=(ylevel3>>3))) &&
                // 比值约束：0.75 <= G/R <= 1.33（黄必须 R、G 相当，
                // 挡掉"过曝红"(G/R~0.7) 与"带红泄漏的绿"(G/R~1.5)）
                (RELATIVE_COLOR ? ({2'd0, g3, 2'b00} > {2'd0, r3} + {1'd0, r3, 1'b0}) : ({2'd0, g3, 2'b00} >= {2'd0, r3} + {1'd0, r3, 1'b0})) &&
                (RELATIVE_COLOR ? ({2'd0, g3} + {1'd0, g3, 1'b0} < {2'd0, r3, 2'b00}) : ({2'd0, g3} + {1'd0, g3, 1'b0} <= {2'd0, r3, 2'b00}));

    wire wht0 = (r0 >= WHT_MIN) && (g0 >= WHT_MIN) && (b0 >= WHT_MIN);
    wire wht1 = (r1 >= WHT_MIN) && (g1 >= WHT_MIN) && (b1 >= WHT_MIN);
    wire wht2 = (r2 >= WHT_MIN) && (g2 >= WHT_MIN) && (b2 >= WHT_MIN);
    wire wht3 = (r3 >= WHT_MIN) && (g3 >= WHT_MIN) && (b3 >= WHT_MIN);

    wire blk0 = (r0 <= BLK_MAX) && (g0 <= BLK_MAX) && (b0 <= BLK_MAX);
    wire blk1 = (r1 <= BLK_MAX) && (g1 <= BLK_MAX) && (b1 <= BLK_MAX);
    wire blk2 = (r2 <= BLK_MAX) && (g2 <= BLK_MAX) && (b2 <= BLK_MAX);
    wire blk3 = (r3 <= BLK_MAX) && (g3 <= BLK_MAX) && (b3 <= BLK_MAX);

    always @(*) begin
        if (!I_tvalid)
            O_mask = 4'd0;
        else begin
            case (I_color_sel)
                3'd0: O_mask = {red0, red1, red2, red3};
                3'd1: O_mask = {blu0, blu1, blu2, blu3};
                3'd2: O_mask = {yel0, yel1, yel2, yel3};
                3'd3: O_mask = {grn0, grn1, grn2, grn3};
                3'd4: O_mask = {wht0, wht1, wht2, wht3};
                3'd5: O_mask = {blk0, blk1, blk2, blk3};
                default: O_mask = 4'd0;
            endcase
        end
    end

endmodule
