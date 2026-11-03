// ===========================================================================
// uart_cmd.v
// 本文件里有三个模块（都跑在 HDMI 像素时钟域，和 uart_rx 同域）：
//   uart_cmd        —— "M" + 数字('0'..'5') → 显示模式；"L" + 数字('0'/'1') → 低功耗开关
//   thr_regs        —— 字母 + 三位十进制 → 颜色阈值寄存器（在线可调）
//   uart_esc_filter —— "C" 前缀转义：把蓝牙小车命令从阈值命令流里挑出来
//
// 为什么三个模块挤在一个文件里：新建的 .v 在 TD 改写工程时条目可能被丢掉
// （见交接简报踩坑清单第 6 条），所以新模块一律并进已在编译清单的文件。
// ===========================================================================
`timescale 1ns/1ps

module uart_cmd (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire [7:0]  I_byte,
    input  wire        I_byte_valid,
    output reg  [2:0]  O_mode,
    output reg         O_mode_valid,
    // 批 3 追加：低功耗按需降频总开关，"L0" 关 / "L1" 开（上电默认开）。
    // 真机上场景噪声多的时候 LP 会跳，演示前发个 L0 就能让它安静下来。
    output reg         O_lp_en
);
    reg expect_digit;
    reg expect_lp;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            expect_digit <= 1'b0;
            expect_lp    <= 1'b0;
            O_mode       <= 3'd0;
            O_mode_valid <= 1'b0;
            O_lp_en      <= 1'b1;
        end
        else begin
            O_mode_valid <= 1'b0;      // 默认清零，形成单拍脉冲
            if (I_byte_valid) begin
                if (I_byte == 8'h4D) begin                    // 'M'
                    expect_digit <= 1'b1;
                    expect_lp    <= 1'b0;
                end
                else if (I_byte == 8'h4C) begin               // 'L'
                    expect_lp    <= 1'b1;
                    expect_digit <= 1'b0;
                end
                else if (expect_digit &&
                         (I_byte >= 8'h30) && (I_byte <= 8'h36)) begin  // M0..M6 (6=RGB reference chart)
                    O_mode       <= I_byte[2:0];
                    O_mode_valid <= 1'b1;
                    expect_digit <= 1'b0;
                end
                else if (expect_lp &&
                         ((I_byte == 8'h30) || (I_byte == 8'h31))) begin  // '0'/'1'
                    O_lp_en   <= (I_byte == 8'h31);
                    expect_lp <= 1'b0;
                end
                else begin
                    expect_digit <= 1'b0;
                    expect_lp    <= 1'b0;
                end
            end
        end
    end

endmodule

// ===========================================================================
// thr_regs.v —— 颜色块检测阈值寄存器（串口在线可调）
//
// 命令格式：一个字母 + 三位十进制数字（多余的位数按"非数字"放弃）：
//   R###  红下限 RED_MIN        r###  红比 G/B 高的量 RED_GAP
//   G###  绿下限 GRN_MIN        g###  绿比 R/B 高的量 GRN_GAP
//   Y###  黄下限 YEL_MIN        y###  黄比蓝高的量   YEL_BDIFF
//   B###  黄里蓝的上限 YEL_BMAX
//   b##   一个块内至少多少目标像素才算有效（0..15）
//   m##   一个区域至少多少个块才算目标（0..15）
//
// 例：  R120  G90  y45  b40  m04
// 读回：**没有单独的读回命令**（原文写"发 N 读回"是错的，RTL 里从来没有 N 这条命令）。
//       当前生效的 9 个阈值一直随 uart_report 的 $T 行周期回发（各 2 位十六进制），
//       上位机直接解析就行，见 uart_report.v 的格式说明。
//
// 注意：这个模块跑在 HDMI 像素时钟域（和 uart_rx 同一个域），
//       阈值跨到 ISP 域由外面的 toggle 握手完成。
// ===========================================================================
module thr_regs (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire [7:0]  I_byte,
    input  wire        I_byte_valid,

    output reg  [7:0]  O_red_min,
    output reg  [7:0]  O_red_gap,
    output reg  [7:0]  O_grn_min,
    output reg  [7:0]  O_grn_gap,
    output reg  [7:0]  O_yel_min,
    output reg  [7:0]  O_yel_bdiff,
    output reg  [7:0]  O_yel_bmax,
    output reg  [3:0]  O_blk_pix,
    output reg  [3:0]  O_min_blk,
    output reg         O_tog          // 任一寄存器更新就翻转（跨域握手用）
);
    reg        busy;
    reg [3:0]  sel;
    reg [1:0]  nd;          // 已收到第几位数字
    reg [9:0]  acc;

    wire [3:0] d = I_byte - 8'h30;
    wire       is_dig = (I_byte >= 8'h30) && (I_byte <= 8'h39);

    task set_reg;
        input [3:0]  s;
        input [9:0]  v;
        begin
            case (s)
                4'd0: O_red_min   <= (v > 10'd255) ? 8'hFF : v[7:0];
                4'd1: O_red_gap   <= (v > 10'd255) ? 8'hFF : v[7:0];
                4'd2: O_grn_min   <= (v > 10'd255) ? 8'hFF : v[7:0];
                4'd3: O_grn_gap   <= (v > 10'd255) ? 8'hFF : v[7:0];
                4'd4: O_yel_min   <= (v > 10'd255) ? 8'hFF : v[7:0];
                4'd5: O_yel_bdiff <= (v > 10'd255) ? 8'hFF : v[7:0];
                4'd6: O_yel_bmax  <= (v > 10'd255) ? 8'hFF : v[7:0];
                4'd7: O_blk_pix   <= (v > 10'd15)  ? 4'hF  : v[3:0];
                default: O_min_blk <= (v > 10'd15) ? 4'hF  : v[3:0];
            endcase
        end
    endtask

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            busy      <= 1'b0;
            sel       <= 4'd0;
            nd        <= 2'd0;
            acc       <= 10'd0;
            O_tog     <= 1'b0;
            // 必须与 design_top_wrapper 里 ISP 域锁存的默认值一致，
            // 否则只发一条命令时，其余 8 个会被这里的旧值整组盖掉。
            O_red_min <= 8'd28;
            O_red_gap <= 8'd6;
            O_grn_min <= 8'd28;
            O_grn_gap <= 8'd6;
            O_yel_min <= 8'd28;
            O_yel_bdiff <= 8'd6;
            O_yel_bmax  <= 8'd255;
            O_blk_pix <= 4'd15;
            O_min_blk <= 4'd8;
        end
        else if (I_byte_valid) begin
            if (!busy) begin
                case (I_byte)
                    8'h52: begin busy <= 1'b1; sel <= 4'd0; nd <= 2'd0; acc <= 10'd0; end // R
                    8'h72: begin busy <= 1'b1; sel <= 4'd1; nd <= 2'd0; acc <= 10'd0; end // r
                    8'h47: begin busy <= 1'b1; sel <= 4'd2; nd <= 2'd0; acc <= 10'd0; end // G
                    8'h67: begin busy <= 1'b1; sel <= 4'd3; nd <= 2'd0; acc <= 10'd0; end // g
                    8'h59: begin busy <= 1'b1; sel <= 4'd4; nd <= 2'd0; acc <= 10'd0; end // Y
                    8'h79: begin busy <= 1'b1; sel <= 4'd5; nd <= 2'd0; acc <= 10'd0; end // y
                    8'h42: begin busy <= 1'b1; sel <= 4'd6; nd <= 2'd0; acc <= 10'd0; end // B
                    8'h62: begin busy <= 1'b1; sel <= 4'd7; nd <= 2'd0; acc <= 10'd0; end // b
                    8'h6D: begin busy <= 1'b1; sel <= 4'd8; nd <= 2'd0; acc <= 10'd0; end // m
                    default: ;
                endcase
            end
            else begin
                if (is_dig) begin
                    acc <= (nd == 2'd0) ? {6'd0, d} : ((acc << 3) + (acc << 1) + {6'd0, d});
                    if (nd == 2'd2) begin
                        set_reg(sel, (nd == 2'd0) ? {6'd0, d} : ((acc << 3) + (acc << 1) + {6'd0, d}));
                        busy  <= 1'b0;
                        O_tog <= ~O_tog;
                    end
                    else begin
                        nd <= nd + 2'd1;
                    end
                end
                else begin
                    busy <= 1'b0;   // 中间来了非数字，放弃这条命令
                end
            end
        end
    end
endmodule

// ===========================================================================
// uart_esc_filter.v —— "C 前缀"转义过滤器
//
// 【为什么需要】
//   上位机这一路串口上跑着两套命令，字母会撞车：
//     阈值命令：R 红下限 / r 红比 / G 绿下限 / g 绿比 / Y 黄下限 / y 黄比
//               B 黄里蓝上限 / b 块内像素 / m 区域块数
//     小车命令：F 前进 / B 后退 / L 左转 / R 右转 / S 停车 / X Y 掉头 ...
//   裸的 'R' 既可能是"红阈值"也可能是"右转"，'B' 既可能是"黄里蓝上限"
//   也可能是"后退"。所以小车命令统一加前缀：
//
//      上位机发  "C" + <1 字节>  →  这一字节只当小车命令
//
//   例： CF 前进  CS 停车  CB 后退  CL 左转  CR 右转  C+ 加速  C- 减速  C5 50%
//
// 【分流办法】
//   'C' 和它后面那一个字节**都不进**阈值/模式解析器（O_cmd_* 那一路），
//   后一个字节单独从 O_esc_* 送出去给蓝牙 TX。这样 "CB" 不会顺手把黄里蓝
//   上限寄存器 armed 起来，两套命令彻底互不干扰。
//   'C'(0x43) 原本没有任何命令占用，加前缀不会挤掉已有命令。
//
// 【两个边界情况】
//   1) 连着打两个 'C'：第二个 'C' 只当"重新开始的前缀"，不当作命令字节发出
//      去（'C' 不是合法的小车命令），这样"手滑多打一个 C"不会把后面的正常
//      命令吃掉。
//   2) 前缀之后 **ESC_TIMEOUT 拍内没等到命令字节就放弃**。上位机只发了半个
//      命令（比如只发了 'C' 就断了）时，没有超时的话这个"等第二个字节"的
//      状态会一直挂着，把后面第一条正常命令吃掉。
// ===========================================================================
module uart_esc_filter #(
    // 前缀后等命令字节的超时拍数：74.25MHz 下 1_500_000 拍 ≈ 20ms
    parameter [20:0] ESC_TIMEOUT = 21'd1500000
)(
    input  wire       I_clk,
    input  wire       I_rst_n,
    input  wire [7:0] I_byte,
    input  wire       I_byte_valid,
    // 给 uart_cmd / thr_regs
    output reg  [7:0] O_cmd_byte,
    output reg        O_cmd_byte_valid,
    // 给蓝牙 TX（小车手动命令）
    output reg  [7:0] O_esc_byte,
    output reg        O_esc_byte_valid
);
    localparam [7:0] CH_C = 8'h43;   // 'C'

    reg        expect_esc;
    reg [20:0] esc_cnt;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            expect_esc       <= 1'b0;
            esc_cnt          <= 21'd0;
            O_cmd_byte       <= 8'h00;
            O_cmd_byte_valid <= 1'b0;
            O_esc_byte       <= 8'h00;
            O_esc_byte_valid <= 1'b0;
        end
        else begin
            // 两个 valid 都是单拍脉冲；byte 和 valid 一起打一拍，
            // 所以下游看到的仍然是"对齐的、一拍宽"的字节事件。
            O_cmd_byte_valid <= 1'b0;
            O_esc_byte_valid <= 1'b0;
            if (I_byte_valid) begin
                if (expect_esc) begin
                    if (I_byte == CH_C) begin
                        esc_cnt <= 21'd0;        // 又一个 'C'：当作新前缀，重新计时
                    end
                    else begin
                        O_esc_byte       <= I_byte;
                        O_esc_byte_valid <= 1'b1;
                        expect_esc       <= 1'b0;
                        esc_cnt          <= 21'd0;
                    end
                end
                else if (I_byte == CH_C) begin
                    expect_esc <= 1'b1;          // 前缀本身不外发
                    esc_cnt    <= 21'd0;
                end
                else begin
                    O_cmd_byte       <= I_byte;
                    O_cmd_byte_valid <= 1'b1;
                end
            end
            else if (expect_esc) begin
                if (esc_cnt == ESC_TIMEOUT)
                    expect_esc <= 1'b0;          // 超时：放弃这次转义
                else
                    esc_cnt <= esc_cnt + 21'd1;
            end
        end
    end
endmodule

