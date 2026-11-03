// ===========================================================================
// uart_tx.v
// 简单 UART 发送器：8N1，LSB 先发。给一个时钟周期的 I_start 脉冲，把 I_data 发出。
// ===========================================================================
`timescale 1ns/1ps

module uart_tx #(
    parameter CLK_FREQ = 50000000,   // 时钟频率
    parameter BAUD     = 115200      // 波特率
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_start,      // 1 个时钟周期启动脉冲
    input  wire [7:0]  I_data,
    output wire        O_busy,       // 正在发送
    output wire        O_tx
);
    localparam DIV = CLK_FREQ / BAUD;   // 50MHz/115200 = 434

    reg [15:0] cnt;
    reg [3:0]  bit_idx;   // 0=start, 1..8=data, 9=stop, 10=空闲
    reg [9:0]  frame;     // {stop, data[7:0], start}，LSB 先发

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            bit_idx <= 4'd10;
            cnt     <= 16'd0;
            frame   <= 10'd0;
        end
        else if (I_start && (bit_idx == 4'd10)) begin
            frame   <= {1'b1, I_data, 1'b0};
            bit_idx <= 4'd0;
            cnt     <= 16'd0;
        end
        else if (bit_idx < 4'd10) begin
            if (cnt == DIV - 1) begin
                cnt <= 16'd0;
                if (bit_idx == 4'd9)
                    bit_idx <= 4'd10;
                else
                    bit_idx <= bit_idx + 4'd1;
            end
            else begin
                cnt <= cnt + 16'd1;
            end
        end
    end

    assign O_busy = (bit_idx != 4'd10);
    assign O_tx   = (bit_idx < 4'd10) ? frame[bit_idx] : 1'b1;

endmodule
