`timescale 1ns/1ps
// Test-prop lamp detector. Track several disconnected runs independently for
// each colour so a large unrelated object cannot capture the only seed.
module light_detect #(
 parameter ROI_ENABLE=0, IMG_WIDTH=1280, IMG_HEIGHT=720,
 parameter [19:0] MIN_PIX=20'd40,
 parameter [11:0] MIN_DIM=12'd10, MAX_DIM=12'd480,
 parameter integer THR_LIVE=1
)(input wire I_clk,I_rst_n,I_tvalid,I_tuser,I_tlast,input wire [95:0] I_tdata,
 output reg O_light_valid,output reg [1:0] O_light_color,
 output reg [11:0] O_light_x0,O_light_y0,O_light_x1,O_light_y1,
 input wire I_roi_active,input wire [47:0] I_roi,
 output wire O_eval,output reg [2:0] O_colors,input wire [55:0] I_thr,
 output reg [146:0] O_display_targets);
 // One representative colour sample per four-pixel beat bounds recognition
 // cost; ROI lane masks and all display pixels retain their original cadence.
 // Combine the prop colour floor and live thresholds before registering.
 // RGB, mask, ROI and stream tags therefore have exactly the same latency.
 reg [3:0] raw_r,raw_y,raw_g;
 // These are post-black, post-AWB recognition codes, not display Gamma codes.
 // Keep a noise floor, but admit dark coloured paper; structure validates it.
 wire [7:0] rmin=THR_LIVE && I_thr[55:48]>12?I_thr[55:48]:8'd12;
 wire [7:0] rgap=THR_LIVE && I_thr[47:40]>6?I_thr[47:40]:8'd6;
 wire [7:0] gmin=THR_LIVE && I_thr[39:32]>12?I_thr[39:32]:8'd12;
 wire [7:0] gbase=THR_LIVE && I_thr[31:24]>6?I_thr[31:24]:8'd6;
 wire [7:0] ymin=THR_LIVE && I_thr[23:16]>12?I_thr[23:16]:8'd12;
 wire [7:0] ygap=THR_LIVE?I_thr[15:8]:8'd5;
 wire [7:0] bmax=THR_LIVE?I_thr[7:0]:8'd200;
 genvar lane;
 generate for(lane=0;lane<4;lane=lane+1)begin: lamp_colour
  wire [7:0] r=I_tdata[71:64],g=I_tdata[63:56],b=I_tdata[55:48];
  wire [8:0] drg={1'b0,r}-{1'b0,g},drb={1'b0,r}-{1'b0,b};
  wire [8:0] dgr={1'b0,g}-{1'b0,r},dgb={1'b0,g}-{1'b0,b};
  wire [9:0] r3={1'b0,r,1'b0}+{2'b0,r},g3={1'b0,g,1'b0}+{2'b0,g},b3={1'b0,b,1'b0}+{2'b0,b};
  wire [7:0] ggap=(g>>4)>gbase?(g>>4):gbase;
  always @(posedge I_clk or negedge I_rst_n)begin
   if(!I_rst_n)begin raw_r[3-lane]<=0;raw_y[3-lane]<=0;raw_g[3-lane]<=0;end
   else begin
    raw_r[3-lane]<=I_tvalid && r>=rmin && !drg[8] && !drb[8] && drg[7:0]>=rgap && drb[7:0]>=rgap && {1'b0,r,3'b0}>={1'b0,g,3'b0}+{4'b0,g} && {1'b0,r,3'b0}>={1'b0,b,3'b0}+{4'b0,b};
    raw_g[3-lane]<=I_tvalid && g>=gmin && !dgr[8] && !dgb[8] && dgr[7:0]>=ggap && dgb[7:0]>=ggap && {1'b0,g,3'b0}>={1'b0,r,3'b0}+{4'b0,r};
    raw_y[3-lane]<=I_tvalid && r>=ymin && g>=ymin && b<=bmax && !drb[8] && !dgb[8] && drb[7:0]>=ygap && dgb[7:0]>=ygap &&
     {1'b0,r,3'b0}>={1'b0,b,3'b0}+{4'b0,b} && {1'b0,g,3'b0}>={1'b0,b,3'b0}+{4'b0,b} && {g,2'b0}>r3 && g3<{r,2'b0};
   end
  end
 end endgenerate
 // Masks are registered, so delay all stream and ROI coordinates one cycle.
 reg v,u,l;reg [11:0] x,y;reg [3:0] rm;
 wire [11:0] px=I_tuser?0:x,py=I_tuser?0:y;
 integer k;
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n)begin v<=0;u<=0;l<=0;x<=0;y<=0;rm<=0;end
  else begin
   v<=I_tvalid;u<=I_tuser;l<=I_tlast;
   for(k=0;k<4;k=k+1) rm[3-k]<=!ROI_ENABLE || !I_roi_active ||
    ((px+1)>=I_roi[47:36] && (px+1)<=I_roi[23:12] && py>=I_roi[35:24] && py<=I_roi[11:0]);
   if(I_tuser)begin x<=I_tvalid?4:0;y<=0;end
   else if(I_tvalid)begin if(I_tlast)begin x<=0;y<=y+1'b1;end else x<=x+4;end
  end
 end
 // The smallest printed lamp disk in the fixture set is 24x24 at 320x240;
 // a 22-pixel floor rejects narrow background colour streaks.
 localparam [11:0] PROP_MIN_DIM = (MIN_DIM < 22) ? 22 : MIN_DIM;
 wire [48:0] rbox,ybox,gbox;wire [19:0] rc,yc,gc;
 // A perfect circle fills only pi/4 of its bounding box. 75% left almost no
 // margin for a noisy edge. The striped-slot check remains mandatory.
 lamp_runs #(.MIN_PIX(MIN_PIX),.MIN_DIM(PROP_MIN_DIM),.MAX_DIM(MAX_DIM)) lr(I_clk,I_rst_n,v,u,l,raw_r&rm,rbox,rc);
 lamp_runs #(.MIN_PIX(MIN_PIX),.MIN_DIM(PROP_MIN_DIM),.MAX_DIM(MAX_DIM)) ly(I_clk,I_rst_n,v,u,l,raw_y&rm,ybox,yc);
 lamp_runs #(.MIN_PIX(MIN_PIX),.MIN_DIM(PROP_MIN_DIM),.MAX_DIM(MAX_DIM)) lg(I_clk,I_rst_n,v,u,l,raw_g&rm,gbox,gc);
 reg [5:0] phase;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)phase<=0; else if(u)phase<=1; else if(phase!=0)phase<=phase==24?0:phase+1'b1;
 end
 wire cr,cy,cg;
 lamp_context #(.SLOT(0)) pr(I_clk,I_rst_n,I_tvalid,I_tuser,I_tlast,I_tdata,phase==16,rbox,rc,cr);
 lamp_context #(.SLOT(1)) pyc(I_clk,I_rst_n,I_tvalid,I_tuser,I_tlast,I_tdata,phase==16,ybox,yc,cy);
 lamp_context #(.SLOT(2)) pg(I_clk,I_rst_n,I_tvalid,I_tuser,I_tlast,I_tdata,phase==16,gbox,gc,cg);
 wire rv=rbox[48]&&cr,yv=ybox[48]&&cy,gv=gbox[48]&&cg;
 assign O_eval=phase==23;

 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n)begin O_colors<=0;O_display_targets<=0;O_light_valid<=0;O_light_color<=0;O_light_x0<=0;O_light_y0<=0;O_light_x1<=0;O_light_y1<=0;end
  else if(phase==21) begin
   O_colors<={gv,yv,rv};O_display_targets<={{gv,gbox[47:0]},{yv,ybox[47:0]},{rv,rbox[47:0]}};
   if(rv && (!yv || rc>=yc) && (!gv || rc>=gc))begin O_light_valid<=1;O_light_color<=0;{O_light_x0,O_light_y0,O_light_x1,O_light_y1}<=rbox[47:0];end
   else if(yv && (!gv || yc>=gc))begin O_light_valid<=1;O_light_color<=1;{O_light_x0,O_light_y0,O_light_x1,O_light_y1}<=ybox[47:0];end
   else if(gv)begin O_light_valid<=1;O_light_color<=2;{O_light_x0,O_light_y0,O_light_x1,O_light_y1}<=gbox[47:0];end
   else begin O_light_valid<=0;O_light_color<=0;{O_light_x0,O_light_y0,O_light_x1,O_light_y1}<=0;end
  end
 end
endmodule

// Two independent live components; inspect both at SOF before reusing slots.
// The first image row is reserved for this two-cycle scan. A complete prop's
// disk and neighbouring empty slots supply many subsequent rows of evidence.
// MIN_RUN filters thin colour streaks that would otherwise occupy both live
// slots before the lamp disk is reached (high-brightness three-lamp photo).
module lamp_runs #(parameter [19:0] MIN_PIX=40,parameter [11:0] MIN_DIM=10,MAX_DIM=480,
 parameter FILL_3_4=0, parameter [10:0] MIN_RUN=11'd12)(
 input wire clk,rst_n,valid,sof,eol,input wire [3:0] mask,
 output reg [48:0] box,output reg [19:0] score);
 reg [10:0] x,run_x0,run_x1;reg [9:0] y;reg [10:0] run_count;reg running,scanning,epoch,scan_epoch;
 reg used[0:1];reg [10:0] x0[0:1],x1[0:1];reg [9:0] y0[0:1],y1[0:1];reg [19:0] cnt[0:1];
 reg [48:0] best;reg [19:0] best_n;
 wire [10:0] px=sof?0:x;wire [9:0] py=sof?0:y;
 wire [2:0] hits={2'd0,mask[3]}+{2'd0,mask[2]}+{2'd0,mask[1]}+{2'd0,mask[0]};
 wire [1:0] first=mask[3]?0:mask[2]?1:mask[1]?2:3;
 wire [1:0] last=mask[0]?3:mask[1]?2:mask[2]?1:0;
 wire [10:0] rx0=running?run_x0:px+first,rx1=hits!=0?px+last:run_x1;
 wire [11:0] rn=(running?{1'b0,run_count}:12'd0)+hits;
 wire finish=valid && !sof && (running||hits!=0) && (hits==0||eol);
 integer i;reg signed [1:0] match_idx,free_idx,stale_idx;reg [19:0] stale_min;reg idx,qi;
 localparam integer AB=MAX_DIM<=511?9:10;
 reg [11:0] qw;reg [10:0] qh;reg [2*AB-1:0] qa;reg qgood;
 reg [48:0] qbox;
 reg q_valid,q_epoch;reg [11:0] pw;reg [10:0] ph;reg [48:0] pbox;reg [19:0] pn;
 wire retire=valid && !sof && !scanning && py!=0 && finish && rn>=MIN_RUN &&
  {1'b0,rx1}-{1'b0,rx0}+12'd1<=MAX_DIM && match_idx<0 && free_idx<0 && stale_idx>=0;
 always @*begin
  match_idx=-1;free_idx=-1;stale_idx=-1;stale_min=20'hfffff;
  for(i=0;i<2;i=i+1)begin
   if(!used[i] && free_idx<0)free_idx=i;
   if(used[i] && {1'b0,py}>={1'b0,y1[i]} && {1'b0,py}<={1'b0,y1[i]}+11'd3 &&
      {1'b0,rx1}+12'd3>={1'b0,x0[i]} && {1'b0,x1[i]}+12'd3>={1'b0,rx0} && match_idx<0)match_idx=i;
   if(used[i] && {1'b0,py}>{1'b0,y1[i]}+11'd3 && cnt[i]<stale_min)begin stale_min=cnt[i];stale_idx=i;end
  end
  idx=0;if(match_idx>=0)idx=match_idx[0];else if(free_idx>=0)idx=free_idx[0];else if(stale_idx>=0)idx=stale_idx[0];
  qi=sof?1'b0:scanning?1'b1:idx;
  qw={1'b0,x1[qi]}-{1'b0,x0[qi]}+12'd1;qh={1'b0,y1[qi]}-{1'b0,y0[qi]}+11'd1;
  qa=pw[AB-1:0]*ph[AB-1:0];
  qgood=q_valid && pw>=MIN_DIM && pw<=MAX_DIM && ph>=MIN_DIM && ph<=MAX_DIM && pn>=MIN_PIX &&
   ({pw,1'b0}+{1'b0,pw})>={ph,1'b0} && ({ph,1'b0}+{1'b0,ph})>={pw,1'b0} &&
   (FILL_3_4 ? {pn,2'b0}>=({qa,1'b0}+qa) : {pn,3'b0}>=({qa,2'b0}+qa)) && {pn,5'b0}<=({qa,5'b0}-{5'b0,qa});
  qbox={1'b1,1'b0,x0[qi],2'b0,y0[qi],1'b0,x1[qi],2'b0,y1[qi]};
 end
 integer j;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   x<=0;y<=0;running<=0;scanning<=0;run_x0<=0;run_x1<=0;run_count<=0;epoch<=0;scan_epoch<=0;q_valid<=0;q_epoch<=0;box<=0;score<=0;best<=0;best_n<=0;
   for(j=0;j<2;j=j+1)begin used[j]<=0;x0[j]<=0;x1[j]<=0;y0[j]<=0;y1[j]<=0;cnt[j]<=0;end
  end else begin
   q_valid<=(sof || scanning || retire) && used[qi];
   q_epoch<=scanning?scan_epoch:epoch;pw<=qw;ph<=qh;pbox<=qbox;pn<=cnt[qi];
   if(sof)begin
    x<=valid?4:0;y<=0;running<=0;run_count<=0;scanning<=1;epoch<=~epoch;scan_epoch<=epoch;
    box<=best;score<=best_n;best<=0;best_n<=0;
    if(qgood && q_epoch==epoch && pn>best_n)begin box<=pbox;score<=pn;end
   end else begin
    if(valid)begin if(eol)begin x<=0;y<=y+1'b1;end else x<=x+4;end
    if(qgood)begin
     if(q_epoch!=epoch)begin
      if(pn>score)begin box<=pbox;score<=pn;end
     end else if(pn>best_n)begin best<=pbox;best_n<=pn;end
    end
    if(scanning)begin
     used[0]<=0;used[1]<=0;scanning<=0;
    end
    // Row zero is excluded from run accumulation, so frame-end inspection
    // cannot conflict with a new component allocation.
    if(valid && py!=0)begin
     if(hits!=0)begin if(!running)run_x0<=px+first;run_x1<=px+last;run_count<=rn[10:0];running<=!eol;end
     else if(running)begin running<=0;run_count<=0;end
     if(eol)run_count<=0;
     if(finish && rn>=MIN_RUN && {1'b0,rx1}-{1'b0,rx0}+12'd1<=MAX_DIM)begin
      if(match_idx>=0)begin
       if(rx0<x0[idx])x0[idx]<=rx0;if(rx1>x1[idx])x1[idx]<=rx1;y1[idx]<=py;cnt[idx]<=cnt[idx]+rn;
      end else if(free_idx>=0 || stale_idx>=0)begin
       used[idx]<=1;x0[idx]<=rx0;x1[idx]<=rx1;y0[idx]<=py;y1[idx]<=py;cnt[idx]<=rn;
      end
     end
    end
   end
  end
 end
endmodule

// The printed prop has white stripes on dark material in the other slots.
// Compare the previous frame's sampled context against the current disk box.
module lamp_context #(parameter SLOT=0)(input wire clk,rst_n,valid,sof,eol,
 input wire [95:0] rgb,input wire latch_box,input wire [48:0] b,input wire [19:0] n,output wire ok);
 reg [11:0] x,y,x0,y0,x1,y1,dx0,dy0;reg track,done_track;
 localparam [8:0] CONTRAST = SLOT==0 ? 9'd8 : 9'd12;
 reg [1:0] c1,c2,d1,d2;reg [7:0] last_sample;reg have_sample,have_direction,last_direction,last_window;
 wire [11:0] px=sof?0:x,py=sof?0:y;
 wire [12:0] w={1'b0,x1}-{1'b0,x0}+1,h={1'b0,y1}-{1'b0,y0}+1;
 // Integer-height bands cover the two neighbouring empty slots. Shift-only
 // boundaries replace fractional-height arithmetic in the pixel path.
 wire above=py<y0;
 wire [12:0] distance=above?{1'b0,y0}-{1'b0,py}:{1'b0,py}-{1'b0,y0};
 wire near_up=above && distance<h;
 wire far_up=above && distance>=h && distance<{h,2'b0};
 wire near_down=!above && distance>=h && distance<{h,1'b0};
 wire far_down=!above && distance>={h,1'b0} && distance<{h,2'b0};
 wire [14:0] h3={h,1'b0}+h;
 wire red_near=!above && distance>=({1'b0,h}+(h>>1)) && distance<={h,1'b0};
 wire red_far=!above && distance>=(h3-(h>>2)) && distance<=(h3+(h>>2));
 wire win1=SLOT==0?red_near:SLOT==1?near_up:far_up;
 wire win2=SLOT==0?red_far:SLOT==1?near_down:near_up;
 wire [12:0] sample_x={1'b0,x0}+(w>>1);
 wire sample_valid=track && (win1||win2) && sample_x>={1'b0,px} && sample_x<({1'b0,px}+4);
 reg [23:0] sample_rgb;
 always @* case(sample_x[1:0])
  0:sample_rgb=rgb[95:72];1:sample_rgb=rgb[71:48];2:sample_rgb=rgb[47:24];default:sample_rgb=rgb[23:0];
 endcase
 wire [7:0] sr=sample_rgb[23:16],sample=sample_rgb[15:8],sb=sample_rgb[7:0];
 wire [7:0] smax=sr>sample ? (sr>sb?sr:sb) : (sample>sb?sample:sb);
 wire [7:0] smin=sr<sample ? (sr<sb?sr:sb) : (sample<sb?sample:sb);
 wire [7:0] neutral_gap=(smax>>2)>16?(smax>>2):8'd16;
 // The empty slots contain WHITE ink. A coloured background edge is not
 // supporting evidence even when its green channel has adequate contrast.
 wire white_sample=({1'b0,smax}-{1'b0,smin})<={1'b0,neutral_gap};
 wire signed [12:0] dx=$signed({1'b0,b[47:36]})-$signed({1'b0,dx0});
 wire signed [12:0] dy=$signed({1'b0,b[35:24]})-$signed({1'b0,dy0});
 // The sampled pixels and colour box belong to the same completed frame.
 // Check position continuity; the new box independently passes size/fill tests.
 wire aligned=b[48] && done_track && dx>=-13'sd16 && dx<=13'sd16 && dy>=-13'sd16 && dy<=13'sd16;
 // Each of the two red/yellow empty slots must contain a complete stripe
 // (both directions). Hysteresis now finds soft edges, so a lone unrelated
 // edge in the second slot is no longer needed as a permissive substitute.
 assign ok=aligned && ((SLOT<2) ? (d1>=2 && d2>=2) : ({1'b0,d1}+{1'b0,d2}>=3));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin x<=0;y<=0;x0<=0;y0<=0;x1<=0;y1<=0;track<=0;dx0<=0;dy0<=0;done_track<=0;c1<=0;c2<=0;d1<=0;d2<=0;last_sample<=0;have_sample<=0;have_direction<=0;last_direction<=0;last_window<=0;end
  else begin
   if(sof)begin x<=valid?4:0;y<=0;dx0<=x0;dy0<=y0;done_track<=track;d1<=c1;d2<=c2;c1<=0;c2<=0;last_window<=0;have_sample<=0;have_direction<=0;end
   else if(valid)begin
    if(eol)begin x<=0;y<=y+1'b1;end else x<=x+4;
    if(sample_valid)begin
     // Hysteresis against the running extremum integrates a gradual edge.
     // Small changes do not move the anchor until their TOTAL contrast is
     // sufficient. During a slope, follow its extreme; count only reversals.
     // Reset at each slot window so a gap cannot manufacture a stripe.
     if(!have_sample || last_window!=win2)begin
      last_sample<=sample;have_direction<=0;
     end else if(white_sample && (!have_direction || !last_direction) &&
                 {1'b0,sample}>={1'b0,last_sample}+CONTRAST)begin
      if(win2)begin if(c2!=3)c2<=c2+1'b1;end else if(c1!=3)c1<=c1+1'b1;
      last_sample<=sample;last_direction<=1;have_direction<=1;
     end else if((!have_direction || last_direction) &&
                 {1'b0,last_sample}>={1'b0,sample}+CONTRAST)begin
      if(win2)begin if(c2!=3)c2<=c2+1'b1;end else if(c1!=3)c1<=c1+1'b1;
      last_sample<=sample;last_direction<=0;have_direction<=1;
     end else if(have_direction && ((last_direction && white_sample && sample>last_sample) ||
                                   (!last_direction && sample<last_sample)))last_sample<=sample;
     last_window<=win2;have_sample<=1;
    end
   end
   if(latch_box)begin track<=b[48];{x0,y0,x1,y1}<=b[47:0];end
  end
 end
endmodule
