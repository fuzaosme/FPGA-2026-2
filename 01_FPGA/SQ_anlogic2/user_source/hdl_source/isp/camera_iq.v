`timescale 1ns/1ps
// Sparse frame statistics. No pixel modification, no backpressure. The sampled
// grid is fixed, so averaging uses one shared constant multiplier per stage.
// Snapshot, MSB first: frame32, meanR8/G8/B8, minY8/maxY8, samples16,
// histogram[0..3]16 (Y=0..63,64..127,128..191,192..255), valid8.
module camera_meter96 #(parameter WIDTH=1280, HEIGHT=720)(
 input wire clk,rst,v,u,l,input wire [95:0] rgb,
 output reg [159:0] snapshot,output reg done);
 localparam NX=(WIDTH+15)/16,NY=(HEIGHT+15)/16,N=NX*NY;
 localparam [31:0] INV=((64'd1<<24)+N/2)/N;
 reg [11:0] x,y;reg pending,active;
 reg [31:0] frame_id,beats;
 wire sof=v&&(u||pending);
 wire [11:0] xx=sof?12'd0:x,yy=sof?12'd0:y;
 wire sample=v&&(xx[3:0]==0)&&(yy[3:0]==0);
 wire [7:0] r=rgb[95:88],g=rgb[87:80],b=rgb[79:72];
 wire [9:0] weighted={2'd0,r}+{1'd0,g,1'b0}+{2'd0,b}+10'd2;
 wire [7:0] lum=weighted[9:2];
 reg [23:0] sr,sg,sb;
 reg [15:0] count,h0,h1,h2,h3;
 reg [7:0] lo,hi;
 reg [23:0] ar,ag,ab;
 reg [15:0] ac,ah0,ah1,ah2,ah3;
 reg [7:0] alo,ahi;reg [31:0] aframe;reg good;
 reg [2:0] phase;
 reg [55:0] product;reg [7:0] mr,mg,mb;
 wire [23:0] nr=(sof?24'd0:sr)+(sample?r:8'd0);
 wire [23:0] ng=(sof?24'd0:sg)+(sample?g:8'd0);
 wire [23:0] nb=(sof?24'd0:sb)+(sample?b:8'd0);
 wire [15:0] nc=(sof?16'd0:count)+sample;
 wire [15:0] nh0=(sof?16'd0:h0)+(sample&&lum[7:6]==0);
 wire [15:0] nh1=(sof?16'd0:h1)+(sample&&lum[7:6]==1);
 wire [15:0] nh2=(sof?16'd0:h2)+(sample&&lum[7:6]==2);
 wire [15:0] nh3=(sof?16'd0:h3)+(sample&&lum[7:6]==3);
 wire [7:0] nlo=sample&&((sof)||(lum<lo))?lum:(sof?8'd255:lo);
 wire [7:0] nhi=sample&&((sof)||(lum>hi))?lum:(sof?8'd0:hi);
 always @(posedge clk or negedge rst)begin
  if(!rst)begin
   x<=0;y<=0;pending<=0;active<=0;frame_id<=0;beats<=0;
   sr<=0;sg<=0;sb<=0;count<=0;h0<=0;h1<=0;h2<=0;h3<=0;lo<=255;hi<=0;
   ar<=0;ag<=0;ab<=0;ac<=0;ah0<=0;ah1<=0;ah2<=0;ah3<=0;
   alo<=0;ahi<=0;aframe<=0;good<=0;phase<=0;product<=0;mr<=0;mg<=0;mb<=0;snapshot<=0;done<=0;
  end else begin
   done<=0;
   if(u&&!v)pending<=1;
   if(v)begin
    if(sof)begin pending<=0;active<=1;frame_id<=frame_id+1'b1;beats<=1;end
    else if(active)beats<=beats+1'b1;
    if(l)begin x<=0;y<=(yy==HEIGHT-1)?0:yy+1'b1;end
    else begin x<=xx+12'd4;y<=yy;end
    sr<=nr;sg<=ng;sb<=nb;count<=nc;h0<=nh0;h1<=nh1;h2<=nh2;h3<=nh3;lo<=nlo;hi<=nhi;
    if(l&&yy==HEIGHT-1&&(active||sof))begin
     active<=0;ar<=nr;ag<=ng;ab<=nb;ac<=nc;ah0<=nh0;ah1<=nh1;ah2<=nh2;ah3<=nh3;
     alo<=nlo;ahi<=nhi;aframe<=sof?frame_id+1'b1:frame_id;
     good<=nc==N && ((sof?32'd1:beats+1'b1)==(WIDTH/4)*HEIGHT);phase<=1;
    end
   end
   case(phase)
    1:begin product<=ar*INV;phase<=2;end
    2:begin mr<=(product+56'd8388608)>>24;product<=ag*INV;phase<=3;end
    3:begin mg<=(product+56'd8388608)>>24;product<=ab*INV;phase<=4;end
    4:begin mb<=(product+56'd8388608)>>24;phase<=5;end
    5:begin snapshot<={aframe,mr,mg,mb,alo,ahi,ac,ah0,ah1,ah2,ah3,7'd0,good};done<=1;phase<=0;end
   endcase
  end
 end
endmodule

// Automatic exposure only. Values are SC500CS half-line units and the existing
// analog-gain table indices (16=1x,48=2x,80=4x). A change is made only just after
// SOF and after verified I2C completion plus four sensor frames of settling.
module camera_auto_exposure #(
 parameter [15:0] EXP_MIN=16'd3,EXP_MAX=16'd1990,
 parameter [7:0] GAIN_MIN=8'd16,GAIN_MAX=8'd112,
 parameter [7:0] TARGET=8'd64,DEADBAND=8'd6)(
 input wire clk,rst,frame_tick,stats_valid,input wire [7:0] mean_y,
 input wire stats_good,cfg_ready,cfg_fault,
 input wire [15:0] applied_exp,input wire [7:0] applied_gain,
 output reg request,output reg [15:0] exposure,output reg [15:0] gain,
 output wire unstable,output reg at_limit);
 reg [7:0] measured;reg have_stats;reg [2:0] settle;
 reg waiting;reg saw_busy;
 reg [16:0] step,next_exp;reg [8:0] gain_step,next_gain;
 assign unstable=waiting||(settle!=0)||!cfg_ready;
 always @(posedge clk or negedge rst)begin
  if(!rst)begin measured<=0;have_stats<=0;settle<=4;waiting<=0;saw_busy<=0;
   request<=0;exposure<=EXP_MAX;gain<=16'd80;at_limit<=0;step<=0;next_exp<=0;gain_step<=0;next_gain<=0;end
  else begin
   request<=0;
   if(stats_valid)begin measured<=mean_y;have_stats<=stats_good;end
   if(waiting)begin
    if(!cfg_ready)saw_busy<=1;
    if(saw_busy&&cfg_ready)begin waiting<=0;saw_busy<=0;settle<=4;have_stats<=0;end
   end
   if(frame_tick&&!waiting&&cfg_ready&&!cfg_fault)begin
    if(settle!=0)begin settle<=settle-1'b1;have_stats<=0;end
    else if(have_stats)begin
     have_stats<=0;next_exp={1'b0,applied_exp};next_gain={1'b0,applied_gain};
     step=(measured+8<TARGET || measured>TARGET+8)?({1'b0,applied_exp}>>2):({1'b0,applied_exp}>>4);
     if(step==0)step=1;
     gain_step=(measured+8<TARGET || measured>TARGET+8)?9'd8:9'd2;
     at_limit<=0;
     if({1'b0,measured}+DEADBAND<TARGET)begin
      if(applied_exp<EXP_MAX)begin next_exp={1'b0,applied_exp}+step;if(next_exp>EXP_MAX)next_exp=EXP_MAX;end
      else if(applied_gain<GAIN_MAX)begin next_gain={1'b0,applied_gain}+gain_step;if(next_gain>GAIN_MAX)next_gain=GAIN_MAX;end
      else at_limit<=1;
     end else if({1'b0,measured}>{1'b0,TARGET}+DEADBAND)begin
      if(applied_gain>GAIN_MIN)begin next_gain=(applied_gain>GAIN_MIN+gain_step)?applied_gain-gain_step:{1'b0,GAIN_MIN};end
      else if(applied_exp>EXP_MIN)begin next_exp=(applied_exp>EXP_MIN+step)?applied_exp-step:{1'b0,EXP_MIN};end
      else at_limit<=1;
     end
     if(next_exp!=applied_exp || next_gain!=applied_gain)begin
      exposure<=next_exp[15:0];gain<={8'd0,next_gain[7:0]};request<=1;waiting<=1;saw_busy<=0;
     end
    end
   end
  end
 end
endmodule
