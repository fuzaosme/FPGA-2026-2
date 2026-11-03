`timescale 1ns/1ps
module arrow_vision #(parameter ENABLE_CONTROL=0, parameter [127:0] DEFAULT_CFG=128'd0)(input wire I_enable,I_external_flush,input wire I_clk,I_rst_n,I_tvalid,I_tuser,I_tlast,input wire [95:0] I_rgb,
 input wire I_cfg_valid,input wire [127:0] I_cfg,
 input wire I_lamp_done,input wire [2:0] I_lamp_colors,
 output reg [127:0] O_cfg,output reg O_flush,
 output reg [191:0] O_snapshot,output reg O_snapshot_valid,output wire [47:0] O_sign_box);
wire enabled=!ENABLE_CONTROL || I_enable;
wire external_flush=ENABLE_CONTROL && I_external_flush;
reg [127:0] pending;reg apply_pending;reg sdone_d;
reg [31:0] fid,lamp_frame,sign_frame;
reg [1:0] settling;
reg [1:0] lamp,dir;
reg [6:0] score;
reg lamp_valid,sign_valid,conflict;
reg [47:0] snapshot_box;
reg clear_prev;
reg lamp_ready,sign_waiting;
wire apply_now=I_tuser && apply_pending;
wire clear_result=!enabled || external_flush || apply_now || O_flush;
wire sdone,sstable;wire [1:0] sdir;wire [6:0] sscore;wire [31:0] sframe;
sign_detect sign(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_tvalid(I_tvalid),.I_tuser(I_tuser),.I_tlast(I_tlast),
 .I_flush(O_flush|!enabled|external_flush),.I_rgb(I_rgb),.I_roi(O_cfg[47:0]),.I_frame(fid),
 .O_done(sdone),.O_dir(sdir),.O_score(sscore),.O_frame(sframe),.O_stable(sstable),.O_box(O_sign_box));
// Snapshot: epoch32 lamp_frame32 sign_frame32 lamp2 dir2 score7 flags4 box48 reserved33.
always @(posedge I_clk or negedge I_rst_n) begin
 if(!I_rst_n) begin
  pending<=0;apply_pending<=0;sdone_d<=0;settling<=2;O_cfg<=DEFAULT_CFG;O_flush<=0;fid<=0;lamp_frame<=0;sign_frame<=0;
  lamp<=3;dir<=0;score<=0;lamp_valid<=0;sign_valid<=0;conflict<=0;snapshot_box<=0;clear_prev<=0;lamp_ready<=0;sign_waiting<=0;O_snapshot<=0;O_snapshot_valid<=0;
 end else begin
  O_flush<=0;clear_prev<=clear_result;sdone_d<=0;O_snapshot_valid<=sdone_d && !clear_result;
  if(I_cfg_valid) begin pending<=I_cfg;apply_pending<=1;end
  if(I_tuser) begin
   fid<=fid+1'b1;lamp_ready<=0;
   if(settling!=0 && !clear_result) settling<=settling-1'b1;
   if(apply_pending) begin
    O_cfg<=pending;apply_pending<=0;O_flush<=1;settling<=2;lamp_valid<=0;sign_valid<=0;lamp<=3;dir<=0;score<=0;
   end
  end
  if(I_lamp_done && !clear_result) begin
   lamp_ready<=1;
   if(sign_waiting)begin sdone_d<=1;sign_waiting<=0;end
   lamp_frame<=fid-1'b1;lamp_valid<=settling==0 && (I_lamp_colors==1 || I_lamp_colors==2 || I_lamp_colors==4);
   conflict<=(I_lamp_colors!=0 && I_lamp_colors!=1 && I_lamp_colors!=2 && I_lamp_colors!=4);
   case(I_lamp_colors) 1:lamp<=0;2:lamp<=1;4:lamp<=2;default:lamp<=3;endcase
  end
  if(sdone && !clear_result) begin
   if(lamp_ready || I_lamp_done)begin sdone_d<=1;sign_waiting<=0;end
   else sign_waiting<=1;
   sign_frame<=sframe;dir<=sdir;score<=sscore;sign_valid<=sstable;snapshot_box<=O_sign_box;
  end
  // Wait for both detectors: an empty sign can finish before the lamp scan.
  // This prevents a new empty-sign frame from publishing the previous lamp colour.
  // The following cycle sees all registered fields and emits an atomic payload.
  begin
   O_snapshot<={O_cfg[127:96],lamp_frame,sign_frame,lamp,dir,score,
                (enabled && O_cfg[127:96]!=0 && settling==0),
                (enabled&&conflict),(enabled&&sign_valid),(enabled&&lamp_valid),snapshot_box,33'd0};
  end
  // Invalidate all latched observations immediately on a configuration change,
  // disable or external flush. Do not publish a pending completion from before it.
  if(clear_result) begin
   settling<=2;lamp_frame<=0;sign_frame<=0;lamp<=3;dir<=0;score<=0;
   lamp_valid<=0;sign_valid<=0;conflict<=0;snapshot_box<=0;
   sdone_d<=0;lamp_ready<=0;sign_waiting<=0;O_snapshot_valid<=!clear_prev;
   O_snapshot<={apply_now?pending[127:96]:O_cfg[127:96],160'd0};
  end
 end
end
endmodule
