`timescale 1ns/1ps
// No added video latency. Badge at bottom right, sign bbox in cyan.
module arrow_overlay(input wire I_clk,I_rst_n,I_vs,I_de,input wire [23:0] I_rgb,
 input wire I_update,input wire [191:0] I_snapshot,output wire [23:0] O_rgb,
 output wire [1:0] O_visible_turn);
reg [11:0] x,y;reg de_d,vs_d;reg [191:0] obs;reg [23:0] age;
always @(posedge I_clk or negedge I_rst_n) begin
 if(!I_rst_n)begin x<=0;y<=0;de_d<=0;vs_d<=0;obs<=0;age<=24'hFFFFFF;end
 else begin
  de_d<=I_de;vs_d<=I_vs;
  if(I_vs!=vs_d)begin x<=0;y<=0;end
  else if(I_de) x<=x+1'b1;
  else if(de_d)begin x<=0;y<=y+1'b1;end
  if(age!=24'hFFFFFF)age<=age+1'b1;
  if(I_update)begin
   // Shared visible state drives both the box and its transmitted class.
   // Configuration/conflict/opposite-turn evidence
   // clears at once; a repeated source frame cannot renew this display lease.
   if(!I_snapshot[84] || I_snapshot[83] || I_snapshot[191:160]!=obs[191:160] ||
      (I_snapshot[93:92]!=0 && I_snapshot[93:92]!=obs[93:92]))begin
    obs<=I_snapshot;age<=(I_snapshot[82] && !I_snapshot[83])?24'd0:24'hffffff;
   end else if(I_snapshot[82] && I_snapshot[127:96]!=obs[127:96])begin obs<=I_snapshot;age<=0;end
  end
 end
end
wire show=obs[84] && !obs[83] && age<24'd15000000;
// One interface: these exact bits enable drawing and request transmission.
assign O_visible_turn={obs[82] && show && obs[93:92]==2,obs[82] && show && obs[93:92]==1};
wire valid_sign=|O_visible_turn;
wire [11:0] x0=obs[80:69],y0=obs[68:57],x1=obs[56:45],y1=obs[44:33];
wire border=valid_sign && x>=x0 && x<=x1 && y>=y0 && y<=y1 &&
 (x<=x0+1 || x+1>=x1 || y<=y0+1 || y+1>=y1);
wire badge=valid_sign && x>=1200 && x<1264 && y>=640 && y<704;
wire [4:0] ux=(x-12'd1200)>>1,uy=(y-12'd640)>>1;
wire [31:0] glyph;
wire [2:0] kind=obs[93:92]==1?3'd0:obs[93:92]==2?3'd1:3'd5;
sign_template_rom rom(.I_kind(kind),.I_row(uy),.O_row(glyph));
assign O_rgb=I_de && border?24'h00FFFF:
 I_de && badge?(glyph[ux]?(valid_sign?24'hFFFFFF:24'hFFBB00):24'h0640B0):I_rgb;
endmodule
