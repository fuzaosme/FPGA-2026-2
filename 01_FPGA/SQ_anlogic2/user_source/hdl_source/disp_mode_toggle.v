// ===========================================================================
//  disp_mode_toggle.v —— 六档图像风格切换按键
//
//  KEY1(I_btn[0]) = 上一档，KEY2(I_btn[1]) = 下一档，循环切换：
//      0 原图 -> 1 二值化 -> 2 线稿 -> 3 卡通 -> 4 伪彩 -> 5 掩码高亮 -> 0
//  每个按键都做：两级同步 + 去抖 + 上升沿检测，按住不放不会连续跳档。
//  DB_CNT 默认 1_000_000：本模块用 100MHz 时钟，约 10ms。
// ===========================================================================
`timescale 1ns/1ps

module disp_mode_toggle #(
    parameter integer DB_CNT   = 1000000,   // 去抖阈值（连续多少拍才认）
    parameter integer MODE_MAX = 5
) (
    input  wire I_clk,
    input  wire I_rst_n,
    input  wire [1:0] I_btn,    // 低电平 = 按下；bit0=上一档，bit1=下一档
    output reg  [2:0] O_mode    // 0..5
);

    reg [1:0]  sync0, sync1;
    reg [31:0] cnt_low0, cnt_hi0;
    reg [31:0] cnt_low1, cnt_hi1;
    reg        db0, db1;
    reg        db0_1d, db1_1d;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            sync0    <= 2'b11;
            sync1    <= 2'b11;
            cnt_low0 <= 32'd0;
            cnt_hi0  <= 32'd0;
            cnt_low1 <= 32'd0;
            cnt_hi1  <= 32'd0;
            db0      <= 1'b0;
            db1      <= 1'b0;
            db0_1d   <= 1'b0;
            db1_1d   <= 1'b0;
            O_mode   <= 3'd0;
        end
        else begin
            sync0 <= {sync0[0], I_btn[0]};
            sync1 <= {sync1[0], I_btn[1]};

            // KEY1 去抖（低电平 = 按下）
            if (sync0[1] == 1'b0) begin
                if (cnt_low0 != DB_CNT) cnt_low0 <= cnt_low0 + 32'd1;
                cnt_hi0 <= 32'd0;
            end
            else begin
                if (cnt_hi0 != DB_CNT) cnt_hi0 <= cnt_hi0 + 32'd1;
                cnt_low0 <= 32'd0;
            end
            if ((sync0[1] == 1'b0) && (cnt_low0 >= DB_CNT)) db0 <= 1'b1;
            if ((sync0[1] == 1'b1) && (cnt_hi0  >= DB_CNT)) db0 <= 1'b0;

            // KEY2 去抖（低电平 = 按下）
            if (sync1[1] == 1'b0) begin
                if (cnt_low1 != DB_CNT) cnt_low1 <= cnt_low1 + 32'd1;
                cnt_hi1 <= 32'd0;
            end
            else begin
                if (cnt_hi1 != DB_CNT) cnt_hi1 <= cnt_hi1 + 32'd1;
                cnt_low1 <= 32'd0;
            end
            if ((sync1[1] == 1'b0) && (cnt_low1 >= DB_CNT)) db1 <= 1'b1;
            if ((sync1[1] == 1'b1) && (cnt_hi1  >= DB_CNT)) db1 <= 1'b0;

            db0_1d <= db0;
            db1_1d <= db1;

            // KEY2 = 下一档，KEY1 = 上一档；两个同时按下时优先 KEY2。
            if (db1 && !db1_1d) begin
                O_mode <= (O_mode == MODE_MAX[2:0]) ? 3'd0 : (O_mode + 3'd1);
            end
            else if (db0 && !db0_1d) begin
                O_mode <= (O_mode == 3'd0) ? MODE_MAX[2:0] : (O_mode - 3'd1);
            end
        end
    end

endmodule
