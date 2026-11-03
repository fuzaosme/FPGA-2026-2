// ===========================================================================
// seg_scan.v   （2026-10-12 扩展：8 位全部用起来，作为场景闭环的"显示执行器"）
// 8 位共阳极数码管动态扫描驱动。
//   位选 SEL[7:0]：高电平有效（点亮对应那一位）
//   段选 SEG[7:0]：共阳，低电平点亮；位序 {dp,g,f,e,d,c,b,a}
// 显示内容（digit_idx 0 = 左起第 1 位）：
//   0: 当前显示模式 0~5
//   1: 检测到的交通灯 0无 1红 2黄 3绿
//   2: 颜色目标总数 R+G+Y（封顶 9）
//   3: 运动目标数 M（封顶 9）
//   4: 置信度 0~9
//   5: 报警 1/0（红灯相位 + 有运动目标）
//   6,7: 交通灯相位倒计时剩余秒数（两位，2026-10-15 批 2 启用；没有相位时熄灭）
// ===========================================================================
`timescale 1ns/1ps

module seg_scan (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire [2:0]  I_mode,          // 显示模式 0~5
    input  wire        I_light_valid,   // 是否检测到交通灯
    input  wire [1:0]  I_light_color,   // 0红 1黄 2绿
    input  wire [3:0]  I_tgt_num,       // 颜色目标总数（已封顶 9）
    input  wire [3:0]  I_mot_num,       // 运动目标数（已封顶 9）
    input  wire [3:0]  I_conf,          // 置信度 0~9
    input  wire        I_alarm,         // 报警
    input  wire [3:0]  I_cd_t,          // 相位倒计时 十位（BCD）
    input  wire [3:0]  I_cd_u,          // 相位倒计时 个位（BCD）
    input  wire        I_cd_vld,        // 已经锁定过相位（否则这两位熄灭）
    output reg  [7:0]  O_seg,           // 段控制（共阳，0=亮）
    output reg  [7:0]  O_sel            // 位选（高电平有效）
);
    reg [11:0] scan_cnt;
    reg [2:0]  digit_idx;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            scan_cnt <= 12'd0;
            digit_idx <= 3'd0;
        end
        else begin
            if (scan_cnt == 12'd1999) begin
                scan_cnt <= 12'd0;
                digit_idx <= (digit_idx == 3'd7) ? 3'd0 : (digit_idx + 3'd1);
            end
            else begin
                scan_cnt <= scan_cnt + 12'd1;
            end
        end
    end

    // 7 段译码（共阳：0=亮 1=灭），段序 {dp,g,f,e,d,c,b,a}
    function [7:0] F_seg;
        input [3:0] v;
        begin
            case (v)
                4'd0:  F_seg = 8'hC0;
                4'd1:  F_seg = 8'hF9;
                4'd2:  F_seg = 8'hA4;
                4'd3:  F_seg = 8'hB0;
                4'd4:  F_seg = 8'h99;
                4'd5:  F_seg = 8'h92;
                4'd6:  F_seg = 8'h82;
                4'd7:  F_seg = 8'hF8;
                4'd8:  F_seg = 8'h80;
                4'd9:  F_seg = 8'h90;
                default: F_seg = 8'hFF;   // 熄灭
            endcase
        end
    endfunction

    reg [3:0] value;
    always @(*) begin
        case (digit_idx)
            3'd0: value = {1'b0, I_mode};                                      // 模式 0~5
            3'd1: value = I_light_valid ? ({2'd0, I_light_color} + 4'd1) : 4'd0; // 0无 1红 2黄 3绿
            3'd2: value = I_tgt_num;                                           // 目标总数
            3'd3: value = I_mot_num;                                           // 运动目标数
            3'd4: value = I_conf;                                              // 置信度
            3'd5: value = I_alarm ? 4'd1 : 4'd0;                               // 报警
            3'd6: value = I_cd_vld ? I_cd_t : 4'd10;                           // 倒计时 十位
            3'd7: value = I_cd_vld ? I_cd_u : 4'd10;                           // 倒计时 个位
            default: value = 4'd10;                                            // 其余熄灭
        endcase
    end

    always @(*) begin
        O_sel = (8'b1 << digit_idx);
        O_seg = F_seg(value);
    end

endmodule
