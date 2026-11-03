`timescale 1ns/1ps
// Serial Dice matcher: six templates, one foreground bit/cycle, no DSP/divider.
module sign_match32(input wire I_clk,I_rst_n,I_start,I_box_ok,
 input wire [31:0] I_frame, input wire [31:0] I_row,
 output reg [4:0] O_row_addr, output reg O_busy,O_done,
 output reg [1:0] O_dir, output reg [6:0] O_score,output reg [31:0] O_frame);
reg [2:0] kind;
reg [4:0] bitno;
reg [31:0] bits;
wire [31:0] tmpl;
sign_template_rom rom(.I_kind(kind),.I_row(O_row_addr),.O_row(tmpl));
reg [10:0] np,nq,ni;
reg [18:0] numerator;
reg [11:0] denominator;
reg [6:0] quotient;
reg [6:0] left_score,right_score,reject_score;
reg [3:0] state;
wire [6:0] best=(left_score>=right_score)?left_score:right_score;
wire [6:0] other=(left_score>=right_score)?right_score:left_score;
wire in_template_mask=((O_row_addr>=5'd2)&&(O_row_addr<=5'd29)&&(bitno>=5'd2)&&(bitno<=5'd29));
always @(posedge I_clk or negedge I_rst_n) begin
 if(!I_rst_n) begin
  O_busy<=0;O_done<=0;O_dir<=0;O_score<=0;O_frame<=0;O_row_addr<=0;
  kind<=0;bitno<=0;bits<=0;np<=0;nq<=0;ni<=0;state<=0;numerator<=0;denominator<=0;quotient<=0;
  left_score<=0;right_score<=0;reject_score<=0;
 end else begin
  O_done<=0;
  case(state)
  0: if(I_start) begin
   O_frame<=I_frame;O_busy<=1;kind<=0;O_row_addr<=0;np<=0;nq<=0;ni<=0;
   left_score<=0;right_score<=0;reject_score<=0;
   if(I_box_ok) state<=1;else begin O_dir<=0;O_score<=0;O_busy<=0;O_done<=1;end
  end
  1: state<=2;
  2: begin bits<=I_row;bitno<=0;state<=3;end
  3: begin
   if(in_template_mask) begin np<=np+bits[bitno];nq<=nq+tmpl[bitno];ni<=ni+(bits[bitno]&tmpl[bitno]);end
   if(bitno==31) begin
    if(O_row_addr==31) state<=4;
    else begin O_row_addr<=O_row_addr+1'b1;state<=1;end
   end else bitno<=bitno+1'b1;
  end
  4: begin
   numerator<=({8'd0,ni}<<7)+({8'd0,ni}<<6)+({8'd0,ni}<<3);
   denominator<={1'b0,np}+{1'b0,nq};quotient<=0;
   if(np<11'd40 || np>11'd600 || (np+nq)==0) begin numerator<=0;denominator<=12'd1;end
   state<=5;
  end
  5: if(numerator>=denominator && quotient<100) begin numerator<=numerator-denominator;quotient<=quotient+1'b1;end
     else begin
      if(kind==0) left_score<=quotient;
      else if(kind==1) right_score<=quotient;
      else if(quotient>reject_score) reject_score<=quotient;
      if(kind==5) state<=6;
      else begin kind<=kind+1'b1;O_row_addr<=0;np<=0;nq<=0;ni<=0;state<=1;end
     end
  6: begin
   O_score<=best;
   if(best>=65 && {1'b0,best}>={1'b0,other}+8'd6 && {1'b0,best}>={1'b0,reject_score}+8'd6)
    O_dir<=(left_score>=right_score)?2'd1:2'd2;
   else O_dir<=2'd3;
   O_busy<=0;O_done<=1;state<=0;
  end
  default: state<=0;
  endcase
 end
end
endmodule
