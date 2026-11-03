`timescale 1ns/1ps
// Stable-data request/ack mailbox. Source may replace only after acknowledgement.
module arrow_mailbox #(parameter W=128)(input wire S_clk,S_rst_n,S_valid,input wire [W-1:0] S_data,output wire S_busy,
 input wire D_clk,D_rst_n,output reg D_valid,output reg [W-1:0] D_data);
reg [W-1:0] held;reg req,ack;reg a1,a2,r1,r2,r3;
assign S_busy=req!=a2;
always @(posedge S_clk or negedge S_rst_n) begin
 if(!S_rst_n) begin held<=0;req<=0;a1<=0;a2<=0;end
 else begin a1<=ack;a2<=a1;if(S_valid && !S_busy) begin held<=S_data;req<=~req;end end
end
always @(posedge D_clk or negedge D_rst_n) begin
 if(!D_rst_n) begin ack<=0;r1<=0;r2<=0;r3<=0;D_valid<=0;D_data<=0;end
 else begin
  r1<=req;r2<=r1;r3<=r2;D_valid<=0;
  if(r3!=ack) begin D_data<=held;D_valid<=1;ack<=r3;end
 end
end
endmodule

// Consume every '$...\n' frame before the old C/M/L/threshold parsers.
// Only fixed-length, CRC-valid Q1 commits. A timeout discards, never replays.
module arrow_command #(parameter CLK_FREQ=75000000)(input wire I_clk,I_rst_n,input wire [7:0] I_byte,input wire I_valid,
 output reg [7:0] O_legacy_byte,output reg O_legacy_valid,output reg [127:0] O_cfg,output reg O_cfg_valid,
 output reg [34:0] O_detect_cfg,output reg O_detect_valid,
 output reg [2:0] O_camera_view,output reg O_camera_valid);
 reg active,valid_frame,detect_kind,camera_kind;reg [2:0] camera_view;reg [6:0] idx;reg [31:0] timer;
 reg [15:0] crc,received_crc;reg [127:0] cfg;reg [34:0] dcfg;
 function [15:0] crcbyte;input [15:0] c;input [7:0] b;reg [15:0] t;integer i;
  begin t=c^{b,8'd0};for(i=0;i<8;i=i+1)t=t[15]?(t<<1)^16'h1021:t<<1;crcbyte=t;end endfunction
 wire digit=(I_byte>=8'h30 && I_byte<=8'h39)||(I_byte>=8'h41 && I_byte<=8'h46);
 wire [3:0] nib=(I_byte<=8'h39)?I_byte-8'h30:I_byte-8'h37;
 wire ishex=(idx>=4 && idx<=11)||(idx>=13 && idx<=15)||(idx>=17 && idx<=19)||
  (idx>=21 && idx<=23)||(idx>=25 && idx<=27)||(idx>=29 && idx<=31)||
  (idx>=33 && idx<=35)||(idx>=37 && idx<=39)||(idx>=41 && idx<=43);
 wire comma=(idx==1 || idx==3 || idx==12 || idx==16 || idx==20 || idx==24 || idx==28 || idx==32 || idx==36 || idx==40);
 wire ranges=cfg[95:84]<cfg[71:60] && cfg[83:72]<cfg[59:48] && cfg[71:60]<1280 && cfg[59:48]<720 &&
  cfg[47:36]<cfg[23:12] && cfg[35:24]<cfg[11:0] && cfg[23:12]<1280 && cfg[11:0]<720;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin active<=0;valid_frame<=0;detect_kind<=0;camera_kind<=0;camera_view<=0;O_camera_view<=0;O_camera_valid<=0;idx<=0;timer<=0;crc<=16'hFFFF;received_crc<=0;cfg<=0;dcfg<=0;
   O_cfg<=0;O_cfg_valid<=0;O_detect_cfg<=0;O_detect_valid<=0;O_legacy_byte<=0;O_legacy_valid<=0;end
  else begin
   O_cfg_valid<=0;O_detect_valid<=0;O_legacy_valid<=0;O_camera_valid<=0;
   if(active)begin if(timer>=CLK_FREQ/5)begin valid_frame<=0;idx<=100;timer<=0;end else timer<=timer+1'b1;end
   if(I_valid)begin
    if(I_byte==8'h24)begin active<=1;idx<=0;timer<=0;crc<=16'hFFFF;received_crc<=0;cfg<=0;dcfg<=0;valid_frame<=1;detect_kind<=0;camera_kind<=0;end
    else if(!active)begin O_legacy_byte<=I_byte;O_legacy_valid<=1;end
    else begin
     timer<=0;
     if(idx==0)begin detect_kind<=I_byte=="D";camera_kind<=I_byte=="I";if(I_byte!="Q" && I_byte!="D" && I_byte!="I")valid_frame<=0;end
     if(idx<(camera_kind?5:(detect_kind?16:44)))crc<=crcbyte(crc,I_byte);
     if(idx==2 && I_byte!="1")valid_frame<=0;
     if(camera_kind)begin
      if((idx==1||idx==3)&&I_byte!=",")valid_frame<=0;
      if(idx==4)begin if(I_byte<"0"||I_byte>"5")valid_frame<=0;camera_view<=I_byte[2:0];end
      if(idx==5&&I_byte!="*")valid_frame<=0;
      if(idx>=6&&idx<=9)begin if(!digit)valid_frame<=0;received_crc<={received_crc[11:0],nib};end
      if(idx==10&&I_byte!=8'h0D)valid_frame<=0;
     end else if(detect_kind)begin
      if((idx==1||idx==3||idx==12||idx==14)&&I_byte!=",")valid_frame<=0;
      if(idx>=4&&idx<=11)begin if(!digit)valid_frame<=0;dcfg[34:3]<={dcfg[30:3],nib};end
      if(idx==13)begin if(I_byte!="0"&&I_byte!="1")valid_frame<=0;dcfg[2]<=I_byte[0];end
      if(idx==15)begin if(I_byte<"0"||I_byte>"2")valid_frame<=0;dcfg[1:0]<=I_byte[1:0];end
      if(idx==16&&I_byte!="*")valid_frame<=0;
      if(idx>=17&&idx<=20)begin if(!digit)valid_frame<=0;received_crc<={received_crc[11:0],nib};end
      if(idx==21&&I_byte!=8'h0D)valid_frame<=0;
     end else begin
      if(comma&&I_byte!=",")valid_frame<=0;
      if(ishex)begin if(!digit)valid_frame<=0;cfg<={cfg[123:0],nib};end
      if(idx==44&&I_byte!="*")valid_frame<=0;
      if(idx>=45&&idx<=48)begin if(!digit)valid_frame<=0;received_crc<={received_crc[11:0],nib};end
      if(idx==49&&I_byte!=8'h0D)valid_frame<=0;
     end
     if(I_byte==8'h0A)begin
      active<=0;
      if(valid_frame&&received_crc==crc)begin
       if(camera_kind&&idx==11)begin O_camera_view<=camera_view;O_camera_valid<=1;end
       else if(detect_kind&&idx==22)begin O_detect_cfg<=dcfg;O_detect_valid<=1;end
       else if(!camera_kind&&!detect_kind&&idx==50&&ranges)begin O_cfg<=cfg;O_cfg_valid<=1;end
      end
     end else if(idx>=100)begin valid_frame<=0;idx<=100;end
     else idx<=idx+1'b1;
    end
   end
  end
 end
endmodule
