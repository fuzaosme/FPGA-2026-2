`timescale 1ns/1ps
// Prop-specific bent-arrow recognition. The lower vertical white stem is on
// the right for LEFT and on the left for RIGHT; the upper white bar supplies
// independent foreground support. Local components isolate blue clutter.
module sign_detect(input wire I_clk,I_rst_n,I_tvalid,I_tuser,I_tlast,I_flush,
 input wire [95:0] I_rgb,input wire [47:0] I_roi,input wire [31:0] I_frame,
 output reg O_done,output reg [1:0] O_dir,output reg [6:0] O_score,
 output reg [31:0] O_frame,output reg O_stable,output reg [47:0] O_box);
reg [11:0] x,y;
wire [11:0] px=I_tuser?12'd0:x,py=I_tuser?12'd0:y;
wire [3:0] blue,white;
reg capture_ok,completed_ok;
reg [47:0] capture_box,completed_box;
// A supported highlight, not a single full-frame maximum, sets the split.
// Once a component exists, meter its box; otherwise bootstrap within the ROI.
// Sixteen saturating four-bit bins need no frame buffer or divider. Requiring
// eight samples ignores isolated hot pixels and tiny background highlights.
reg [7:0] peak,level,white_level;
reg [3:0] highlight_count[0:15];
reg color_mode,bright_mode; wire color_pixel; reg [2:0] misses;
wire [7:0] green=I_rgb[63:56];
wire [11:0] sample_x=px+12'd1;
wire in_reference=sample_x>=I_roi[47:36] && sample_x<=I_roi[23:12] && py>=I_roi[35:24] && py<=I_roi[11:0] &&
 (!capture_ok || (sample_x>=capture_box[47:36] && sample_x<=capture_box[23:12] && py>=capture_box[35:24] && py<=capture_box[11:0]));
integer hi;
always @* begin
 peak=0;
 for(hi=0;hi<16;hi=hi+1)if(highlight_count[hi]>=8)peak={hi[3:0],4'h0};
end
integer bin;
always @(posedge I_clk or negedge I_rst_n) begin
 if(!I_rst_n)begin level<=8'd32;white_level<=8'd32;for(bin=0;bin<16;bin=bin+1)highlight_count[bin]<=0;end
 else if(I_flush)begin level<=8'd32;white_level<=8'd32;for(bin=0;bin<16;bin=bin+1)highlight_count[bin]<=0;end
 else if(I_tuser)begin
  if(peak>=32)begin
   // Within the target, the supported highlight is the white ink reference;
   // half of it separates ink from the disk better than a global quarter.
   white_level<=capture_ok ? peak>>1 : peak>>2;
   // Local white metering must not change the disk threshold that produced
   // this box; that feedback can shrink a valid disk into an inner fragment.
   if(!capture_ok)level<=peak>>2;
  end
  for(bin=0;bin<16;bin=bin+1)highlight_count[bin]<=0;
 end else if(I_tvalid && in_reference && highlight_count[green[7:4]]!=15)
  highlight_count[green[7:4]]<=highlight_count[green[7:4]]+1'b1;
end
sign_mask_96 mask(.I_valid(I_tvalid),.I_rgb(I_rgb),.I_x(px),.I_y(py),.I_roi(I_roi),.I_level(level),.I_white_level(white_level),.I_color_mode(color_mode),.I_bright_mode(bright_mode),.O_blue(blue),.O_white(white),.O_color(color_pixel));
wire bv,bdone;wire [11:0] bx0,by0,bx1,by1,raw_y0;wire [19:0] bc;
prop_sign_components #(.MIN_PIX(20'd160),.MIN_DIM(12'd32),.MAX_DIM(12'd640)) bs(
 .I_clk(I_clk),.I_rst_n(I_rst_n),.I_tvalid(I_tvalid),.I_tuser(I_tuser),.I_tlast(I_tlast),.I_mask(blue),.I_roi(I_roi),
 .O_box_valid(bv),.O_x0(bx0),.O_y0(raw_y0),.O_x1(bx1),.O_y1(by1),.O_cnt(bc),.O_done(bdone));
// Short-run filtering can trim the narrow upper arrowhead. Sample a small
// margin above an ink-only component so the head is visible to the matcher.
wire [11:0] top_pad=(bx1-bx0+12'd1)>>3;
assign by0=bright_mode&&!color_mode ? (raw_y0>top_pad?raw_y0-top_pad:12'd0):raw_y0;
wire [12:0] bw={1'b0,bx1}-{1'b0,bx0}+1,bh={1'b0,by1}-{1'b0,by0}+1;
wire geom=bv && bw>=32 && bw<=640 && bh>=32 && bh<=640 &&
 {1'b0,bx0}>{1'b0,I_roi[47:36]}+13'd4 && {1'b0,by0}>{1'b0,I_roi[35:24]}+13'd4 &&
 {1'b0,bx1}+13'd4<{1'b0,I_roi[23:12]} && {1'b0,by1}+13'd4<{1'b0,I_roi[11:0]};
// Capture an exposure-normalised 8x8 foreground signature. A complete spatial
// pattern, including arrowhead evidence versus bent bars, replaces three
// aggregate pixel counts. One probe/beat suffices for the 32px size floor.
reg [31:0] completed_frame;
reg [15:0] gx,gy;
reg [11:0] cw,ch;
reg [2:0] sx,sy;
reg [6:0] samples,completed_samples;
reg [63:0] grid,completed_grid;
reg capture_ink,completed_ink;
reg [1:0] flush_frames;
wire [11:0] probe_x=gx[15:4],probe_y=gy[15:4];
wire sample_hit=capture_ok && I_tvalid && !I_tuser && samples<64 && py==probe_y && probe_x>=px && {1'b0,probe_x}<({1'b0,px}+13'd4);
wire [12:0] dx=(bx0>capture_box[47:36])?bx0-capture_box[47:36]:capture_box[47:36]-bx0;
wire [12:0] dy=(by0>capture_box[35:24])?by0-capture_box[35:24]:capture_box[35:24]-by0;
wire [12:0] dw=(bw>cw)?bw-cw:cw-bw,dh=(bh>ch)?bh-ch:ch-bh;
wire aligned=geom && dx<=(cw>>3)+12 && dy<=(ch>>3)+12 && dw<=(cw>>2)+16 && dh<=(ch>>2)+16;

wire mdone,mbusy;wire [1:0] direction;wire [6:0] shape_score;wire [31:0] shape_frame;
reg [9:0] history;
integer hv;reg [2:0] votes;
always @* begin
 votes=0;
 for(hv=0;hv<5;hv=hv+1)if(history[hv*2+:2]==direction && direction!=0)votes=votes+1'b1;
end
// The completed frame bank stays fixed until SOF. Abort an unfinished scan
// before replacing it, rather than copying its 64 bits into a third bank.
prop_arrow_match8 matcher(.clk(I_clk),.rst_n(I_rst_n),.flush(I_flush || (I_tuser && mbusy)),
 .start(bdone && !mbusy),.box_ok(completed_ok && aligned && completed_samples==64 && flush_frames==0),
 .ink(completed_ink),.pixels(completed_grid),.frame(completed_frame),
 .busy(mbusy),.done(mdone),.dir(direction),.score(shape_score),.oframe(shape_frame));
// Change segmentation only after completed shape rejections. The matcher
// reads the completed bank; the independent live capture cannot overwrite it.
always @(posedge I_clk or negedge I_rst_n)begin
 if(!I_rst_n)begin color_mode<=1;bright_mode<=0;misses<=0;end
 else if(I_flush)begin color_mode<=1;bright_mode<=0;misses<=0;end
 else if(mdone && flush_frames==0)begin
  if(direction!=0)misses<=0;
  else if(misses==3 || (misses==2 && !geom))begin
   if(color_mode)begin color_mode<=0;bright_mode<=0;end
   else if(!bright_mode)bright_mode<=1;
   else begin color_mode<=1;bright_mode<=0;end
   misses<=0;
  end else misses<=misses+1'b1;
 end
end
always @(posedge I_clk or negedge I_rst_n)begin
 if(!I_rst_n)begin
  x<=0;y<=0;capture_ok<=0;completed_ok<=0;capture_box<=0;completed_box<=0;completed_frame<=0;
  gx<=0;gy<=0;cw<=0;ch<=0;sx<=0;sy<=0;samples<=0;completed_samples<=0;
  grid<=0;completed_grid<=0;capture_ink<=0;completed_ink<=0;history<=0;flush_frames<=2;
  O_done<=0;O_dir<=0;O_score<=0;O_frame<=0;O_stable<=0;O_box<=0;
 end else begin
  O_done<=0;
  if(I_tuser)begin
   x<=I_tvalid?4:0;y<=0;
   completed_ok<=capture_ok;completed_box<=capture_box;completed_frame<=I_frame;
   completed_grid<=grid;completed_samples<=samples;completed_ink<=capture_ink;
   grid<=0;samples<=0;sx<=0;sy<=0;capture_ok<=0;
   if(flush_frames!=0)flush_frames<=flush_frames-1'b1;
  end else if(I_tvalid)begin
   if(I_tlast)begin x<=0;y<=y+1'b1;end else x<=x+4;
  end
  if(bdone)begin
   capture_ok<=geom && flush_frames==0;capture_box<={bx0,by0,bx1,by1};cw<=bw[11:0];ch<=bh[11:0];
   capture_ink<=bright_mode&&!color_mode;
   gx<={bx0,4'd0}+bw;gy<={by0,4'd0}+bh;
   if(geom && completed_ok && !aligned)history<=0;
  end
  if(sample_hit)begin
   grid<={white[2],grid[63:1]};samples<=samples+1'b1;
   if(sx==7)begin
    sx<=0;sy<=sy+1'b1;gx<={capture_box[47:36],4'd0}+cw;gy<=gy+{3'd0,ch,1'b0};
   end else begin sx<=sx+1'b1;gx<=gx+{3'd0,cw,1'b0};end
  end
  if(mdone)begin
   O_done<=1;O_frame<=shape_frame;O_dir<=direction;O_score<=shape_score;
   // Fresh observations only: the vote window tolerates missed frames, but
   // never labels an empty current frame as a fresh turn command.
   O_stable<=direction!=0 && votes>=2 && flush_frames==0;
   O_box<=direction!=0?completed_box:48'd0;
   history<={history[7:0],direction};
  end
  if(I_flush)begin
   capture_ok<=0;completed_ok<=0;grid<=0;completed_grid<=0;samples<=0;completed_samples<=0;
   history<=0;flush_frames<=2;O_done<=0;O_dir<=0;O_score<=0;O_stable<=0;O_box<=0;
  end
 end
end
endmodule

module prop_sign_components #(
 parameter IMG_WIDTH=1280,IMG_HEIGHT=720,SHAPE_ENABLE=0,
 parameter [19:0] MIN_PIX=160,parameter [11:0] MIN_DIM=32,MAX_DIM=640
)(input wire I_clk,I_rst_n,I_tvalid,I_tuser,I_tlast,input wire [3:0] I_mask,input wire [47:0] I_roi,
 output wire O_done,O_box_valid,output wire [11:0] O_x0,O_y0,O_x1,O_y1,
 output wire [19:0] O_cnt,output wire [19:0] O_envelope,output wire O_taper);
 wire [48:0] box;reg [2:0] frame_tag;
 prop_sign_runs #(.MIN_PIX(MIN_PIX),.MIN_DIM(MIN_DIM),.MAX_DIM(MAX_DIM)) runs(
  I_clk,I_rst_n,I_tvalid,I_tuser,I_tlast,I_mask,I_roi,box,O_cnt);
 assign {O_box_valid,O_x0,O_y0,O_x1,O_y1}=box;
 assign O_envelope=0;assign O_taper=1;assign O_done=frame_tag[2];
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)frame_tag<=0;else frame_tag<={frame_tag[1:0],I_tuser};
 end
endmodule
// Two independent live components; inspect both at SOF before reusing slots.
// The first image row is reserved for this two-cycle scan. A complete prop's
// disk and neighbouring empty slots supply many subsequent rows of evidence.
// MIN_RUN: a run shorter than this cannot seed or grow a candidate. Printed
// props have wide bright/dark runs; JPEG noise and thin scene edges do not.
// Without this floor, many 1-4 pixel runs occupy both slots and the real disk
// is never inspected (the right-arrow low-saturation regression).
module prop_sign_runs #(parameter [19:0] MIN_PIX=40,parameter [11:0] MIN_DIM=10,MAX_DIM=480,
 parameter [10:0] MIN_RUN=11'd10)(
 input wire clk,rst_n,valid,sof,eol,input wire [3:0] mask,input wire [47:0] roi,
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
  qgood=q_valid && {1'b0,pbox[47:36]}>{1'b0,roi[47:36]}+13'd4 && {1'b0,pbox[35:24]}>{1'b0,roi[35:24]}+13'd4 && {1'b0,pbox[23:12]}+13'd4<{1'b0,roi[23:12]} && {1'b0,pbox[11:0]}+13'd4<{1'b0,roi[11:0]} && pw>=MIN_DIM && pw<=MAX_DIM && ph>=MIN_DIM && ph<=MAX_DIM && pn>=MIN_PIX &&
   ({pw,3'b0})>=({ph,2'b0}+ph) && ({ph,3'b0})>=({pw,2'b0}+pw) &&
   {pn,3'b0}>=({qa,1'b0}+qa) && {pn,3'b0}<=({qa,3'b0}-{3'b0,qa});
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
      end else if((free_idx>=0 || stale_idx>=0) && {2'b0,rx0}>{1'b0,roi[47:36]}+13'd4 && {2'b0,rx1}+13'd4<{1'b0,roi[23:12]})begin
       used[idx]<=1;x0[idx]<=rx0;x1[idx]<=rx1;y0[idx]<=py;y1[idx]<=py;cnt[idx]<=rn;
      end
     end
    end
   end
  end
 end
endmodule

// Serial full-pattern Dice match with explicit non-arrow competitors.
// 16 candidates, 64 bits/candidate; bounded under 2800 clocks, no DSP/divider.
module prop_arrow_match8(input wire clk,rst_n,flush,start,box_ok,ink,
 input wire [63:0] pixels,input wire [31:0] frame,
 output reg busy,done,output reg [1:0] dir,output reg [6:0] score,output wire [31:0] oframe);
 // Caller holds the completed bank/frame fixed while busy, or asserts flush.
 wire [63:0] bits=pixels;
 assign oframe=frame;reg tight;reg [3:0] kind;reg [5:0] bitno;
 reg [6:0] np,nq,ni,lscore,rscore,reject_score,quotient;
 reg [13:0] numerator;reg [7:0] denominator;reg [2:0] state;
 reg [63:0] template;
 // Upright and +/-10 degree arrows and bent-bar competitors. Ink-only
 // arrows include a narrower head; ink-only L competitors include the same
 // top sampling margin as the candidate, so padding cannot create a head.
 always @* case({tight,kind})
 5'd0: template=64'h000030343e3e0400;
 5'd1: template=64'h000004043c7c2000;
 5'd2: template=64'h000030303c3c0000;
 5'd3: template=64'h000004043c3c0000;
 5'd4: template=64'h00001818183c1800;
 5'd5: template=64'h00081c1818181800;
 5'd6: template=64'h0000183c3c180000;
 5'd7: template=64'h0000183c3c1c0000;
 5'd8: template=64'h000020343e3e0000;
 5'd9: template=64'h00040c043c7c1000;
 5'd10: template=64'h001030343c3e0400;
 5'd11: template=64'h000004247c3c2000;
 5'd12: template=64'h000020303c3c0000;
 5'd13: template=64'h00040c041c3c0000;
 5'd14: template=64'h00103030383c0000;
 5'd15: template=64'h000004043c3c0000;
 5'd16: template=64'hc0c0c0c4ffff0604;
 5'd17: template=64'h03030323ffff6020;
 5'd18: template=64'he0e0e0e0e0ffff00;
 5'd19: template=64'h0707070707ffff00;
 5'd20: template=64'h18181818187e3c18;
 5'd21: template=64'h183c7e1818181818;
 5'd22: template=64'hc0c0c0c0c4fffe04;
 5'd23: template=64'h0303030323ff7f20;
 5'd24: template=64'hc0c0e4e6ff7e6604;
 5'd25: template=64'h0e0607277fff6010;
 5'd26: template=64'h7060e0e4feff0608;
 5'd27: template=64'h03032767ff7e6620;
 5'd28: template=64'he0e0e0e0ff7f7800;
 5'd29: template=64'h0e0e0e0707fff800;
 5'd30: template=64'h707070e0e0ff1f00;
 5'd31: template=64'h07070707fffe1e00;
 default:template=0;
 endcase
 wire [63:0] mask=tight?64'hffffffffffffffff:64'h007e7e7e7e7e7e00;
 // Ink-only candidates must contain the upper wing of the arrowhead.
 // A long bent bar can otherwise attain a high Dice score from its shaft.
 wire head_ok=!tight || (lscore>=rscore ? |(pixels[7:0]&8'h0e) : |(pixels[7:0]&8'h70));
 wire [6:0] best=lscore>=rscore?lscore:rscore,other=lscore>=rscore?rscore:lscore;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin busy<=0;done<=0;dir<=0;score<=0;tight<=0;kind<=0;bitno<=0;np<=0;nq<=0;ni<=0;lscore<=0;rscore<=0;reject_score<=0;quotient<=0;numerator<=0;denominator<=1;state<=0;end
  else if(flush)begin busy<=0;done<=0;dir<=0;score<=0;state<=0;end
  else begin
   done<=0;
   case(state)
    0:if(start)begin
     tight<=ink;kind<=0;bitno<=0;np<=0;nq<=0;ni<=0;lscore<=0;rscore<=0;reject_score<=0;
     if(box_ok)begin busy<=1;state<=1;end else begin dir<=0;score<=0;done<=1;end
    end
    1:begin
     if(mask[bitno])begin np<=np+bits[bitno];nq<=nq+template[bitno];ni<=ni+(bits[bitno]&template[bitno]);end
     if(bitno==63)state<=2;else bitno<=bitno+1'b1;
    end
    2:begin
     numerator<=({7'd0,ni}<<7)+({7'd0,ni}<<6)+({7'd0,ni}<<3);
     denominator<={1'b0,np}+{1'b0,nq};quotient<=0;
     if(np<4 || np>(tight?52:28) || np+nq==0)begin numerator<=0;denominator<=1;end
     state<=3;
    end
    3:if(numerator>=denominator && quotient<100)begin numerator<=numerator-denominator;quotient<=quotient+1'b1;end
     else begin
      if(kind==0 || kind==8 || kind==10 || (tight && kind==6))begin if(quotient>lscore)lscore<=quotient;end
      else if(kind==1 || kind==9 || kind==11 || (tight && kind==7))begin if(quotient>rscore)rscore<=quotient;end
      else if(quotient>reject_score)reject_score<=quotient;
      if(kind==15)state<=4;
      else begin kind<=kind+1'b1;bitno<=0;np<=0;nq<=0;ni<=0;state<=1;end
     end
    4:begin
     score<=best;
     if(head_ok && best>=70 && {1'b0,best}>={1'b0,other}+8'd8 && {1'b0,best}>={1'b0,reject_score}+8'd1)dir<=lscore>=rscore?1:2;
     else dir<=0;
     done<=1;busy<=0;state<=0;
    end
    default:state<=0;
   endcase
  end
 end
endmodule
