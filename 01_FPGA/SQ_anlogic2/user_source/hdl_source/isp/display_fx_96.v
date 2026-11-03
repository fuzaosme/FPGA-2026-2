`timescale 1ns/1ps
// Tablet-video presentation effects. The external M0..M6 interface is unchanged.
// All display transforms are isolated from detector/AWB/vehicle decisions.
// Intra-beat lane0 is the leftmost pixel (LSB here, reversed at module boundary).
module display_fx_96 #(parameter WIDTH=1280,HEIGHT=720,
 parameter DISPLAY_GAMMA_ENABLE=1,DISPLAY_COLOR_TONE_ENABLE=1,
 parameter EDGE_LOW=18,EDGE_HIGH=46,BIN_BIAS=6)(
 input wire I_clk,I_rst_n,I_tvalid,I_tuser,I_tlast,
 input wire [95:0] I_rgb,input wire [2:0] I_mode,
 output wire [95:0] O_rgb,output wire O_tvalid,O_tuser,O_tlast,
 output wire [11:0] O_x,O_y,
 input wire [2:0] I_debug_view,output reg [19:0] O_quant_black,O_edge_pixels,
 input wire [146:0] I_lamp_targets,input wire I_sign_valid,input wire [47:0] I_sign_box,
 input wire I_clear_targets,input wire [1:0] I_sign_dir,
 output wire [4:0] O_visible_signs);
 wire clk=I_clk,rst=I_rst_n;
 // Only valid raster coordinates are stored; public coordinates stay 12-bit.
 localparam XB=(WIDTH>1)?$clog2(WIDTH):1,YB=(HEIGHT>1)?$clog2(HEIGHT):1;
 reg [XB-1:0] x;reg [YB-1:0] y;reg pending_sof;reg [2:0] mode;
 wire sof=I_tvalid && (I_tuser || pending_sof);
 wire [11:0] px=sof?12'd0:x,py=sof?12'd0:y;
 wire [2:0] selected=sof?I_mode:mode;
 function [7:0] gray;
  input [23:0] p;reg [9:0] s;
  begin s={2'd0,p[23:16]}+{1'd0,p[15:8],1'b0}+{2'd0,p[7:0]}+10'd2;gray=s[9:2];end
 endfunction
 function [23:0] chart;
  input [11:0] xx,yy;integer band;reg [7:0] g;
  begin
   // Only bands 0..6 are distinguished; all higher quotients select the
   // existing default. This is exact for every 12-bit x, without division.
   if(xx<(WIDTH/8))band=0;
   else if(xx<2*(WIDTH/8))band=1;
   else if(xx<3*(WIDTH/8))band=2;
   else if(xx<4*(WIDTH/8))band=3;
   else if(xx<5*(WIDTH/8))band=4;
   else if(xx<6*(WIDTH/8))band=5;
   else if(xx<7*(WIDTH/8))band=6;
   else band=7;
   if(yy<HEIGHT/2) begin
    case(band)0:g=0;1:g=8;2:g=16;3:g=32;4:g=64;5:g=128;6:g=192;default:g=255;endcase
    chart={g,g,g};
   end else case(band)
    0:chart=24'hff0000;1:chart=24'h00ff00;2:chart=24'h0000ff;3:chart=24'hffff00;
    4:chart=24'h00ffff;5:chart=24'hff00ff;6:chart=24'hffffff;default:chart=0;
   endcase
  end
 endfunction
 function [23:0] heat;
  input [7:0] z;reg [7:0] a;
  begin
   a={z[5:0],z[5:4]};
   case(z[7:6])
    0:heat={8'd0,a,8'd255};
    1:heat={8'd0,8'd255,8'd255-a};
    2:heat={a,8'd255,8'd0};
    default:heat={8'd255,8'd255-a,8'd0};
   endcase
  end
 endfunction
 // M5 target membership is sampled with the corresponding original pixel.
 // Keep the 1B spatial/temporal pipeline unchanged for every other mode.
 // Presentation-only evidence: two hits within six video frames acquire;
 // eight missing frames expire. No held target feeds a control snapshot.
 wire clear_targets=(I_clear_targets===1'b1);
 wire [195:0] incoming_targets={I_sign_valid,I_sign_box,I_lamp_targets};
 reg [195:0] held_targets;
 wire [195:0] visible_targets;
 genvar target;
 generate for(target=0;target<4;target=target+1)begin: display_hold
  reg [4:0] evidence;reg [2:0] missed;
  wire [48:0] fresh=incoming_targets[target*49+:49];
  wire [48:0] held=held_targets[target*49+:49];
  wire next_valid=fresh[48] ? (evidence!=0 || held[48]) : (held[48] && missed!=7);
  // Coordinates need only one fresh/held mux. Invalid coordinates are not
  // visible; clearing their 192 bits added a redundant mux per pixel bound.
  assign visible_targets[target*49+:48]=(sof && fresh[48])?fresh[47:0]:held[47:0];
  assign visible_targets[target*49+48]=!clear_targets && (sof?next_valid:held[48]);
  always @(posedge clk or negedge rst)begin
   if(!rst)begin evidence<=0;missed<=0;held_targets[target*49+:49]<=0;end
   else if(clear_targets)begin evidence<=0;missed<=0;held_targets[target*49+48]<=0;end
   else if(sof)begin
    held_targets[target*49+48]<=next_valid;
    if(fresh[48])held_targets[target*49+:48]<=fresh[47:0];
    evidence<={evidence[3:0],fresh[48]};
    if(fresh[48])missed<=0;else if(missed!=7)missed<=missed+1'b1;
   end
  end
 end endgenerate
 wire [146:0] lamps=visible_targets[146:0];
 wire sign_ok=visible_targets[195];
 wire [47:0] sign_bounds=visible_targets[194:147];
 // Raster beats begin on a four-pixel boundary. Share each box's high
 // coordinate comparison; only its two boundary lanes need two-bit tests.
 function [3:0] target_lanes;
  input [11:0] base,yy;input [47:0] bounds;integer lane;
  begin
   for(lane=0;lane<4;lane=lane+1)target_lanes[lane]=
    yy>=bounds[35:24]&&yy<=bounds[11:0]&&
    base[11:2]>=bounds[47:38]&&base[11:2]<=bounds[23:14]&&
    (base[11:2]!=bounds[47:38]||lane>=bounds[37:36])&&
    (base[11:2]!=bounds[23:14]||lane<=bounds[13:12]);
  end
 endfunction
 // The exported five-bit interface itself enables the M5 pixels. UART uses
 // these same bits after the CDC mailbox, without another recognition gate.
 wire [3:0] target_bits=(target_lanes(px,py,sign_bounds)&{4{|O_visible_signs[4:3]}})|
  (target_lanes(px,py,lamps[47:0])&{4{O_visible_signs[0]}})|
  (target_lanes(px,py,lamps[96:49])&{4{O_visible_signs[1]}})|
  (target_lanes(px,py,lamps[145:98])&{4{O_visible_signs[2]}});
 reg [2:0] debug_view;
 reg [2:0] requested_debug;
 always @(*)begin
  case(I_debug_view) 1,2,3,4,5:requested_debug=I_debug_view;default:requested_debug=0;endcase
 end
 wire [2:0] selected_debug=sof?requested_debug:debug_view;
 // Observe M5's existing acquire/hold/clear state. Preserve its direction
 // alongside the held coordinates; a provisional opposite hit cannot relabel it.
 reg [1:0] held_sign_dir;
 always @(posedge clk or negedge rst)begin
  if(!rst)held_sign_dir<=0;
  else if(clear_targets)held_sign_dir<=0;
  else if(sof && I_sign_valid)held_sign_dir<=I_sign_dir;
 end
 wire [1:0] visible_dir=(sof && I_sign_valid)?I_sign_dir:held_sign_dir;
 assign O_visible_signs=(selected==5 && selected_debug==0)?
  {sign_ok && visible_dir==2,sign_ok && visible_dir==1,lamps[146],lamps[97],lamps[48]}:5'd0;
 always @(posedge clk or negedge rst)begin
  if(!rst)begin x<=0;y<=0;pending_sof<=0;mode<=0;debug_view<=0;end
  else begin
   if(I_tuser&&!I_tvalid)pending_sof<=1;
   if(I_tvalid)begin
    if(sof)begin pending_sof<=0;mode<=I_mode;debug_view<=requested_debug;end
    if(I_tlast)begin x<=0;y<=(py==HEIGHT-1)?0:py+1'b1;end
    else begin x<=px+12'd4;y<=py;end
   end
  end
 end
 // Preserve chroma at four bits per channel instead of independently jumping
 // by 85 codes. For n=c+8, n=16*a+b=17*a+(b-a), so the
 // quotient is a-(b<a). This removes twelve wide constant multipliers.
 function [3:0] color4;
  input [7:0] c;
  reg [8:0] rounded;
  begin rounded={1'b0,c}+9'd8;color4=rounded[7:4]-(rounded[3:0]<rounded[8:4]);end
 endfunction
 // Eight soft luminance bands: flat band centres, continuous transitions.
 // Shared RGB offset preserves colour differences instead of changing hue at
 // three independent thresholds. No frame memory or temporal image mixing.
 function [7:0] cartoon_luma;
  input [7:0] y;reg [3:0] next_band;reg [4:0] transition;
  begin
   next_band={1'b0,y[7:5]}+4'd1;
   transition=y[4:0]-5'd8;
   if(y[4:0]>=24)cartoon_luma=next_band[3]?8'd255:{next_band[2:0],5'd0};
   else if(y[4:0]>8)cartoon_luma={y[7:5],transition[3:0],1'b0};
   else cartoon_luma={y[7:5],5'd0};
  end
 endfunction
 function [7:0] clamp_color;
  input signed [10:0] c;
  begin if(c<0)clamp_color=0;else if(c>255)clamp_color=255;else clamp_color=c[7:0];end
 endfunction
 wire [95:0] tone;
 style_proc_96 #(.DISPLAY_GAMMA_ENABLE(DISPLAY_GAMMA_ENABLE),.DISPLAY_COLOR_TONE_ENABLE(DISPLAY_COLOR_TONE_ENABLE)) original(
  .I_clk(clk),.I_rst_n(rst),.I_style(3'd0),.I_rgb(I_rgb),.I_bin(96'd0),.I_mask(4'd0),.I_tvalid(I_tvalid),.O_tdata(tone));
 reg [95:0] raw1,raw2;
 reg [1:0] vp,up,lp;reg [11:0] x1,x2,y1,y2;reg [2:0] m1,m2,dbg1,dbg2;reg [3:0] mask1,mask2;
 integer z;
 always @(posedge clk or negedge rst)begin
  if(!rst)begin raw1<=0;raw2<=0;vp<=0;up<=0;lp<=0;x1<=0;x2<=0;y1<=0;y2<=0;m1<=0;m2<=0;dbg1<=0;dbg2<=0;mask1<=0;mask2<=0;end
  else begin
   vp<={vp[0],I_tvalid};up<={up[0],sof};lp<={lp[0],I_tlast};
   x1<=px;x2<=x1;y1<=py;y2<=y1;m1<=selected;m2<=m1;dbg1<=selected_debug;dbg2<=dbg1;mask2<=mask1;
   for(z=0;z<4;z=z+1)raw1[z*24+:24]<=I_rgb[95-z*24-:24];
   mask1<=target_bits;
   raw2<=raw1;
  end
 end
 // Tone pixels use a short ordered ERAM FIFO. Chart pixels are generated
 // from the final packet coordinates. This avoids carrying 192 bypass bits
 // through every spatial stage, saving registers and routing resources.
 wire [5:0] base={dbg2,m2};
 wire [95:0] tone_lanes={tone[23:0],tone[47:24],tone[71:48],tone[95:72]};
 wire raw_debug=(dbg2==1)||(dbg2==2)||(dbg2==3);
 wire [95:0] original_data=raw_debug?{raw2[23:0],raw2[47:24],raw2[71:48],raw2[95:72]}:tone;
 wire gv,gu,gl;wire [11:0] gx,gy;wire [95:0] gd;wire [5:0] gt;
 fx_gauss3 #(.T(6),.WIDTH(WIDTH)) gaussian(clk,rst,vp[1],up[1],lp[1],x2,y2,((m2==1||m2==2)?raw2:tone_lanes),base,gv,gu,gl,gx,gy,gd,gt);
 wire [31:0] gg;
 genvar k;
 generate for(k=0;k<4;k=k+1)begin: luminance
  assign gg[k*8+:8]=gray(gd[k*24+:24]);
 end endgenerate
 // Quantization is exactly the 1B color4 function, moved ahead of an
 // unchanged delay tag. Store its 48 result bits rather than 96 source bits.
 wire [47:0] compact_color;
 generate for(k=0;k<4;k=k+1)begin: compact_tag
  assign compact_color[k*12+:12]={color4(gd[k*24+16+:8]),color4(gd[k*24+8+:8]),color4(gd[k*24+:8])};
 end endgenerate
 wire [85:0] box_tag={gg,compact_color,gt};
 wire bv,bu,bl;wire [11:0] bx,by;wire [31:0] bd;wire [85:0] bt;
 fx_gray3 #(.T(86),.WIDTH(WIDTH)) gray_smoothing(clk,rst,gv,gu,gl,gx,gy,gg,box_tag,bv,bu,bl,bx,by,bd,bt);
 // M1/M2 work before display gamma. M1 uses a global frame threshold;
 // M2 blends the second gray Gaussian with its aligned centre for fine detail.
 // Other modes retain their original pixels.
 wire [7:0] bin_threshold;
 fx_region_threshold #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) region_threshold(
  clk,rst,bv,bu,bl,by,bt[61:54],bt[2:0]==1,bin_threshold);
 wire [87:0] sp;
 generate for(k=0;k<4;k=k+1)begin: binarize
  wire [7:0] pixel_y=(bt[2:0]==2)?bd[k*8+:8]:bt[54+k*8+:8];
  wire [1:0] bin_class=(pixel_y<=8||{1'b0,pixel_y}+9'd3<{1'b0,bin_threshold})?2'd2:
   ({1'b0,pixel_y}>={1'b0,bin_threshold}+9'd3)?2'd0:2'd1;
  assign sp[k*22+:22]={bin_class,bt[6+k*12+:12],pixel_y};
 end endgenerate
 wire sv,su,sl;wire [11:0] sx,sy;wire [791:0] sw;wire [5:0] st;
 fx_window3 #(.P(22),.T(6),.WIDTH(WIDTH)) sobel_window(clk,rst,bv,bu,bl,bx,by,sp,bt[5:0],sv,su,sl,sx,sy,sw,st);
 reg ev,eu,el;reg [11:0] ex,ey;reg [139:0] ed;reg [5:0] et;
 integer p,n;reg signed [10:0] dx,dy;reg [10:0] ax,ay;reg [9:0] sum_left,sum_right,sum_top,sum_bottom;reg [10:0] mag11;reg [1:0] direction;reg [21:0] pixel_cell;
 always @(posedge clk or negedge rst)begin
  if(!rst)begin ev<=0;eu<=0;el<=0;ex<=0;ey<=0;ed<=0;et<=0;end
  else begin
   ev<=sv;eu<=su;el<=sl;ex<=sx;ey<=sy;et<=st;
   if(sv)for(p=0;p<4;p=p+1)begin
    sum_left={2'd0,sw[(p*9+0)*22+:8]}+{1'd0,sw[(p*9+3)*22+:8],1'b0}+{2'd0,sw[(p*9+6)*22+:8]};
    sum_right={2'd0,sw[(p*9+2)*22+:8]}+{1'd0,sw[(p*9+5)*22+:8],1'b0}+{2'd0,sw[(p*9+8)*22+:8]};
    sum_top={2'd0,sw[(p*9+0)*22+:8]}+{1'd0,sw[(p*9+1)*22+:8],1'b0}+{2'd0,sw[(p*9+2)*22+:8]};
    sum_bottom={2'd0,sw[(p*9+6)*22+:8]}+{1'd0,sw[(p*9+7)*22+:8],1'b0}+{2'd0,sw[(p*9+8)*22+:8]};
    dx=$signed({1'b0,sum_right})-$signed({1'b0,sum_left});
    dy=$signed({1'b0,sum_bottom})-$signed({1'b0,sum_top});
    ax=dx[10]?-dx:dx;ay=dy[10]?-dy:dy;
    // Sobel components are bounded by 4*255=1020; ten absolute bits suffice.
    mag11={1'b0,ax[9:0]}+{1'b0,ay[9:0]};
    if((ay<<1)<=ax)direction=0;else if((ax<<1)<=ay)direction=1;else direction=(dx[10]==dy[10])?2:3;
    pixel_cell=sw[(p*9+4)*22+:22];
    // L1 Sobel magnitude is even: gx+gy has only even coefficients.
    // Store magnitude/2, recovering the exact magnitude after the line RAM.
    ed[p*35+:35]<={(direction==1)?dy[10]:dx[10],pixel_cell,direction,mag11[10:1]};
   end
  end
 end
 wire nv,nu,nl;wire [11:0] nx,ny;wire [1259:0] nw;wire [5:0] nt;
 fx_window3 #(.P(35),.T(6),.WIDTH(WIDTH)) nms_window(clk,rst,ev,eu,el,ex,ey,ed,et,nv,nu,nl,nx,ny,nw,nt);
 reg cv,cu,cl;reg [11:0] cx,cy;reg [95:0] cd;reg [5:0] ct;
 reg [34:0] centre,tangent_a,tangent_b;reg [9:0] before_mag,after_mag;reg [1:0] cls,mono_class;reg [7:0] edge_high,edge_low;reg [8:0] edge_strong;reg support_a,support_b,coherent,coherent_both,ridge;
 always @(posedge clk or negedge rst)begin
  if(!rst)begin cv<=0;cu<=0;cl<=0;cx<=0;cy<=0;cd<=0;ct<=0;end
  else begin
   cv<=nv;cu<=nu;cl<=nl;cx<=nx;cy<=ny;ct<=nt;
   if(nv)for(p=0;p<4;p=p+1)begin
    centre=nw[(p*9+4)*35+:35];
    case(centre[11:10])
     0:begin before_mag=nw[(p*9+3)*35+:10];after_mag=nw[(p*9+5)*35+:10];end
     1:begin before_mag=nw[(p*9+1)*35+:10];after_mag=nw[(p*9+7)*35+:10];end
     2:begin before_mag=nw[(p*9+0)*35+:10];after_mag=nw[(p*9+8)*35+:10];end
     default:begin before_mag=nw[(p*9+2)*35+:10];after_mag=nw[(p*9+6)*35+:10];end
    endcase
    cls=0;
    if(centre[9:0]>=before_mag && centre[9:0]>after_mag && {centre[9:0],1'b0}>=EDGE_LOW)
     cls=({centre[9:0],1'b0}>=EDGE_HIGH)?2:1;
    // Keep symmetric local maxima: equal peaks both survive, but the broad
    // Sobel skirts do not. This avoids one-sided tie switching at flat peaks.
    case(centre[11:10])
     0:begin tangent_a=nw[(p*9+1)*35+:35];tangent_b=nw[(p*9+7)*35+:35];end
     1:begin tangent_a=nw[(p*9+3)*35+:35];tangent_b=nw[(p*9+5)*35+:35];end
     2:begin tangent_a=nw[(p*9+2)*35+:35];tangent_b=nw[(p*9+6)*35+:35];end
     default:begin tangent_a=nw[(p*9+0)*35+:35];tangent_b=nw[(p*9+8)*35+:35];end
    endcase
    support_a=(tangent_a[11:10]==centre[11:10]&&tangent_a[34]==centre[34]);
    support_b=(tangent_b[11:10]==centre[11:10]&&tangent_b[34]==centre[34]);
    coherent=support_a||support_b;coherent_both=support_a&&support_b;
    ridge=centre[9:0]>=before_mag&&centre[9:0]>=after_mag;
    edge_high=8'd10+{3'd0,centre[19:15]};edge_low=8'd4+{4'd0,centre[19:16]};
    edge_strong={edge_high,1'b0}+{1'b0,edge_high};
    // Lower thresholds recover weak structure, while two tangent neighbours
    // must agree before a normal seed can grow temporal confidence.
    mono_class=!ridge?2'd0:({centre[9:0],1'b0}>=edge_strong&&coherent)?2'd3:
     ({centre[9:0],1'b0}>=edge_high&&coherent_both)?2'd2:({centre[9:0],1'b0}>=edge_low&&coherent)?2'd1:2'd0;
    if(nt[2:0]==1)mono_class=centre[33:32];
    cd[p*24+:24]<={centre[31:12],cls,mono_class};
   end
  end
 end
 wire hv,hu,hl;wire [11:0] hx,hy;wire [863:0] hw;wire [5:0] ht;
 fx_window3 #(.P(24),.T(6),.WIDTH(WIDTH)) connect_window(clk,rst,cv,cu,cl,cx,cy,cd,ct,hv,hu,hl,hx,hy,hw,ht);
 reg fv,fu,fl;reg [11:0] fx,fy;reg [91:0] fd;reg [5:0] ft;
 reg strong_neighbor,edge_bit;reg [23:0] connected;
 always @(posedge clk or negedge rst)begin
  if(!rst)begin fv<=0;fu<=0;fl<=0;fx<=0;fy<=0;fd<=0;ft<=0;end
  else begin
   fv<=hv;fu<=hu;fl<=hl;fx<=hx;fy<=hy;ft<=ht;
   if(hv)for(p=0;p<4;p=p+1)begin
    strong_neighbor=0;
    for(n=0;n<9;n=n+1)if(hw[(p*9+n)*24+2+:2]==2)strong_neighbor=1;
    connected=hw[(p*9+4)*24+:24];
    edge_bit=(connected[3:2]==2)||(connected[3:2]==1 && strong_neighbor);
    fd[p*23+:23]<={connected[23:4],connected[1:0],edge_bit};
   end
  end
 end
 wire qv,qu,ql;wire [11:0] qx,qy;wire [827:0] qw;wire [5:0] qt;
 fx_window3 #(.P(23),.T(6),.WIDTH(WIDTH)) cleanup_window(clk,rst,fv,fu,fl,fx,fy,fd,ft,qv,qu,ql,qx,qy,qw,qt);
 reg [5:0] tone_wptr,tone_rptr;
 wire [99:0] tone_delayed;
 wire [5:0] tone_next=qv?(tone_rptr+6'd1):tone_rptr;
 fx_ram #(.W(100),.AW(6)) original_fifo(clk,vp[1],tone_wptr,tone_next,{mask2,original_data},tone_delayed);
 always @(posedge clk or negedge rst)begin
  if(!rst)begin tone_wptr<=0;tone_rptr<=0;end
  else begin if(vp[1])tone_wptr<=tone_wptr+1'b1;if(qv)tone_rptr<=tone_rptr+1'b1;end
 end
 function [3:0] count9;
  input [8:0] bits;reg [1:0] a,b,c;
  begin a={1'b0,bits[0]}+bits[1]+bits[2];b={1'b0,bits[3]}+bits[4]+bits[5];
   c={1'b0,bits[6]}+bits[7]+bits[8];count9={2'd0,a}+{2'd0,b}+{2'd0,c};end
 endfunction
 // A majority vote must not erase a supported straight thin stroke.
 function binary_vote;
  input [8:0] bits;reg [8:0] same;
  begin
   same=bits^{9{!bits[4]}};
   if((same[0]&&same[8])||(same[1]&&same[7])||(same[2]&&same[6])||(same[3]&&same[5]))
    binary_vote=bits[4];
   else binary_vote=count9(bits)>=5;
  end
 endfunction
 // Reuse the last spatial window for colour smoothing. The extra registered
 // stage keeps Gaussian sums separate from luma quantisation/clipping timing.
 reg sample_v,sample_u,sample_l;reg [11:0] sample_x,sample_y;reg [5:0] sample_tag;
 reg [95:0] sample_original,sample_color;reg [31:0] sample_luma;
 reg [3:0] sample_bin,sample_mask,sample_line,sample_cartoon_line;reg [7:0] sample_class;
 reg [22:0] final_cell;reg [8:0] edge_bits,strong_bits;reg [1:0] final_class;
 // Adjacent 3x3 windows share six vertical columns. Sum [1,2,1]
 // vertically once per column, then horizontally. Bounds are 60 and 240;
 // no truncation/rounding occurs before the unchanged colour_product step.
 wire [5:0] color_vertical[0:17];
 genvar cc,vc;
 generate for(cc=0;cc<3;cc=cc+1)begin: vertical_channel
  for(vc=0;vc<6;vc=vc+1)begin: column
   localparam CELL=(vc<2)?vc:(vc-2)*9+2;
   assign color_vertical[cc*6+vc]={2'd0,qw[CELL*23+11+cc*4+:4]}+
    {1'd0,qw[(CELL+3)*23+11+cc*4+:4],1'b0}+{2'd0,qw[(CELL+6)*23+11+cc*4+:4]};
  end
 end endgenerate
 reg [7:0] color_sum;reg [12:0] color_product;integer ch;
 always @(posedge clk or negedge rst)begin
  if(!rst)begin
   sample_v<=0;sample_u<=0;sample_l<=0;sample_x<=0;sample_y<=0;sample_tag<=0;
   sample_original<=0;sample_color<=0;sample_luma<=0;
   sample_class<=0;sample_bin<=0;sample_mask<=0;sample_line<=0;sample_cartoon_line<=0;
  end else begin
   sample_v<=qv;sample_u<=qu;sample_l<=ql;sample_x<=qx;sample_y<=qy;sample_tag<=qt;
   if(qv)begin
    sample_original<=tone_delayed[95:0];sample_mask<=tone_delayed[99:96];
    for(p=0;p<4;p=p+1)begin
     for(n=0;n<9;n=n+1)begin
      edge_bits[n]=qw[(p*9+n)*23];
      strong_bits[n]=qw[(p*9+n)*23+1+:2]>=2;
     end
     final_cell=qw[(p*9+4)*23+:23];final_class=final_cell[2:1];
     if(qt[2:0]==2&&final_class>=2&&count9(strong_bits)<2)final_class=1;
     sample_class[p*2+:2]<=final_class;
     sample_bin[p]<=final_class==2;
     sample_line[p]<=final_class==2;
     sample_cartoon_line[p]<=final_cell[0]&&(count9(edge_bits)>=3);
     sample_luma[p*8+:8]<=final_cell[10:3];
     for(ch=0;ch<3;ch=ch+1)begin
      color_sum={2'd0,color_vertical[ch*6+p]}+{1'd0,color_vertical[ch*6+p+1],1'b0}+{2'd0,color_vertical[ch*6+p+2]};
      color_product={5'd0,color_sum}+({5'd0,color_sum}<<4)+13'd8;
      sample_color[p*24+ch*8+:8]<=color_product[11:4];
     end
    end
   end
  end
 end
 reg ov,ou,ol;reg [11:0] ox,oy;reg [95:0] result;
 reg [7:0] result_class;reg [2:0] result_mode;
 reg [23:0] rgb,quant_rgb,outpixel;reg [7:0] lum,color_y,quant_y;
 reg signed [10:0] color_delta;reg [19:0] qcount,ecount;reg [2:0] qb,eb;reg [8:0] blend;
 always @(posedge clk or negedge rst)begin
  if(!rst)begin ov<=0;ou<=0;ol<=0;ox<=0;oy<=0;result<=0;result_class<=0;result_mode<=0;qcount<=0;ecount<=0;O_quant_black<=0;O_edge_pixels<=0;end
  else begin
   ov<=sample_v;ou<=sample_u;ol<=sample_l;ox<=sample_x;oy<=sample_y;
   if(sample_v)begin
    qb=0;eb=0;result_class<=sample_class;result_mode<=(sample_tag[5:3]==0)?sample_tag[2:0]:3'd0;
    for(p=0;p<4;p=p+1)begin
     rgb=sample_color[p*24+:24];color_y=gray(rgb);quant_y=cartoon_luma(color_y);
     color_delta=$signed({3'd0,quant_y})-$signed({3'd0,color_y});
     for(ch=0;ch<3;ch=ch+1)quant_rgb[ch*8+:8]=clamp_color($signed({3'd0,rgb[ch*8+:8]})+color_delta);
     lum=sample_luma[p*8+:8];outpixel=0;
     case(sample_tag[2:0])
      0:outpixel=sample_original[95-p*24-:24];
      1:outpixel=sample_bin[p]?24'hffffff:24'h000000;
      2:outpixel=sample_line[p]?24'h000000:24'hffffff;
      3:outpixel=sample_cartoon_line[p]?24'h000000:quant_rgb;
      4:outpixel=heat(lum);
      5:outpixel=sample_mask[p]?sample_original[95-p*24-:24]:{lum,lum,lum};
      6:outpixel=chart(sample_x+p,sample_y);
      default:outpixel=sample_original[95-p*24-:24];
     endcase
     if(quant_rgb==0)qb=qb+1'b1;
     if(sample_cartoon_line[p])eb=eb+1'b1;
     if(sample_tag[2:0]!=6)case(sample_tag[5:3])
      1,2,3:outpixel=sample_original[95-p*24-:24];
      4:outpixel=quant_rgb;
      5:outpixel=sample_cartoon_line[p]?24'h000000:24'hffffff;
     endcase
     result[95-p*24-:24]<=outpixel;
    end
    qcount<=(sample_u?20'd0:qcount)+qb;ecount<=(sample_u?20'd0:ecount)+eb;
    if(sample_l&&sample_y==HEIGHT-1)begin O_quant_black<=(sample_u?20'd0:qcount)+qb;O_edge_pixels<=(sample_u?20'd0:ecount)+eb;end
   end
  end
 end
 fx_temporal_mono #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) temporal(
  clk,rst,ov,ou,ol,ox,oy,result,result_mode,result_class,
  O_tvalid,O_tuser,O_tlast,O_x,O_y,O_rgb);
endmodule
