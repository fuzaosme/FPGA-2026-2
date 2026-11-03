`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Create Date:    01:32:02 06/12/2023 
// Module Name:    SC500GainTbl 
//////////////////////////////////////////////////////////////////////////////////
module SC500GainTbl(
	input 			I_clk,
	input 			I_rstn,
	input 	[6:0] 	I,
	output reg	[15:0] 	O
);

    // SC500CS V1.8, table 2-4: coarse 03/07/0f/1f, fine 20..3f.
    // Complete the formerly truncated table; FPGA auto gain is capped at 8x.
    always @(posedge I_clk)begin
      case(I[6:5])
       0:O<={8'h03,3'b001,I[4:0]};
       1:O<={8'h07,3'b001,I[4:0]};
       2:O<={8'h0f,3'b001,I[4:0]};
       3:O<={8'h1f,3'b001,I[4:0]};
      endcase
    end
endmodule
