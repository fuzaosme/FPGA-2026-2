`timescale 1ns/1ps
// Lossless 96-to-128 video packing. An invalid beat pauses the byte stream;
// it does not discard a partially assembled word. SOF/EOL accompany output
// words, including SOF received separately from the first valid input beat.
module display_pack96(
 input wire clk,rst,v,u,l,input wire [95:0] d,
 output reg ov,ou,ol,output reg [127:0] od);
 reg [95:0] previous;
 reg [1:0] phase;
 reg start_pending;
 wire [1:0] cur=u?2'd0:phase;
 always @(posedge clk or negedge rst)begin
  if(!rst)begin previous<=0;phase<=0;start_pending<=0;ov<=0;ou<=0;ol<=0;od<=0;end
  else begin
   ov<=0;ou<=0;ol<=0;
   if(u)begin phase<=0;start_pending<=1;end
   if(v)begin
    previous<=d;phase<=cur+1'b1;
    case(cur)
     0:begin end
     1:begin ov<=1;od<={previous,d[95:64]};ou<=start_pending;start_pending<=0;end
     2:begin ov<=1;od<={previous[63:0],d[95:32]};end
     3:begin ov<=1;od<={previous[31:0],d};ol<=l;end
    endcase
   end
  end
 end
endmodule
