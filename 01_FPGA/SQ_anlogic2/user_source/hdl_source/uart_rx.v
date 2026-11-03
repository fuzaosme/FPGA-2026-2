// ===========================================================================
// uart_rx.v
// 简单 UART 接收器：8N1，LSB 先收。收到一个字节时 O_valid 拉高一个时钟周期。
// ===========================================================================
`timescale 1ns/1ps

module uart_rx #(
    parameter CLK_FREQ = 75000000,
    parameter BAUD     = 115200
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_rx,
    output reg  [7:0]  O_data,
    output reg         O_valid
);
    localparam BIT_CNT = CLK_FREQ / BAUD;   // 651
    localparam HALF    = BIT_CNT / 2;       // 325

    reg rx_1, rx_2;
    always @(posedge I_clk) begin
        rx_1 <= I_rx;
        rx_2 <= rx_1;
    end

    localparam [1:0] IDLE = 2'd0, START = 2'd1, DATA = 2'd2, STOP = 2'd3;
    reg [1:0]  state;
    reg [15:0] cnt;
    reg [3:0]  bit_i;
    reg [7:0]  sh;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            state   <= IDLE;
            cnt     <= 16'd0;
            bit_i   <= 4'd0;
            sh      <= 8'd0;
            O_data  <= 8'd0;
            O_valid <= 1'b0;
        end
        else begin
            O_valid <= 1'b0;
            case (state)
                IDLE: begin
                    if (rx_2 && !rx_1) begin          // 起始位下降沿
                        state <= START;
                        cnt   <= HALF - 1;
                    end
                end
                START: begin
                    if (cnt == 0) begin
                        state <= DATA;
                        bit_i <= 4'd0;
                        cnt   <= BIT_CNT - 1;
                    end
                    else
                        cnt <= cnt - 16'd1;
                end
                DATA: begin
                    if (cnt == 0) begin
                        sh <= {rx_1, sh[7:1]};        // LSB 先收
                        if (bit_i == 4'd7) begin
                            state <= STOP;
                            cnt   <= BIT_CNT - 1;
                        end
                        else begin
                            bit_i <= bit_i + 4'd1;
                            cnt   <= BIT_CNT - 1;
                        end
                    end
                    else
                        cnt <= cnt - 16'd1;
                end
                STOP: begin
                    if (cnt == 0) begin
                        if (rx_1) begin               // 停止位为 1 才认为有效
                            O_data  <= sh;
                            O_valid <= 1'b1;
                        end
                        state <= IDLE;
                    end
                    else
                        cnt <= cnt - 16'd1;
                end
                default: state <= IDLE;
            endcase
        end
    end

endmodule
