`timescale 1ns/1ps
// Display-only RAM. Explicit primitives prevent asynchronously-reset address
// registers from converting video line storage to slice flip-flops. Contents
// are intentionally not reset; row/column replication masks unwritten data.
module fx_ram #(parameter W=32, AW=9)(
 input wire clk, input wire we, input wire [AW-1:0] wa,ra,
 input wire [W-1:0] wd, output wire [W-1:0] rd);
 localparam N=(W+19)/20;
 genvar k;
 generate for(k=0;k<N;k=k+1) begin: bank
  localparam B=((W-k*20)>=20)?20:W-k*20;
  PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(B),.DATA_WIDTH_B(B),
   .ADDR_WIDTH_A(AW),.ADDR_WIDTH_B(AW),.DATA_DEPTH_A(2**AW),.DATA_DEPTH_B(2**AW),
   .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
   .WRITEMODE_A("READBEFOREWRITE"),.WRITEMODE_B("READBEFOREWRITE"),
   .IMPLEMENT("20K(FAST)"),.CLKMODE("SYNC")) mem(
   .clka(clk),.clkb(clk),.cea(1'b1),.ceb(1'b1),.ocea(1'b0),.oceb(1'b0),
   .wea(we),.web(1'b0),.addra(wa),.addrb(ra),.dia(wd[k*20+:B]),.dib({B{1'b0}}),
   .bea(1'b0),.beb(1'b0),.rsta(1'b0),.rstb(1'b0),.doa(),.dob(rd[k*20+:B]),
   .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(32'd0),.ecc_dbiterrinj(32'd0));
 end endgenerate
endmodule

// Two prior lines share one dual-port ERAM. Port A overwrites the oldest
// line and returns its OLD value; port B reads the other line. Allocating
// each line to a power-of-two half uses only address concatenation, and cuts
// the number of ERAM primitives nearly in half without spatial decimation.
module fx_rows2 #(parameter W=32,T=1,WIDTH=1280)(
 input wire clk,rst,v,u,l,input wire [11:0] x,y,
 input wire [W-1:0] d,input wire [T-1:0] tag,
 output reg ov,ou,ol,output reg [11:0] ox,oy,
 output wire [W-1:0] top,mid,output reg [W-1:0] bot,output reg [T-1:0] ot);
 localparam CAW=$clog2(WIDTH/4),AW=CAW+1,N=(W+19)/20;
 wire [AW-1:0] wa={y[0],x[CAW+1:2]},ra={~y[0],x[CAW+1:2]};
 // ERAM's output registers replace the external top/mid register bank.
 // Both the two-cycle read latency and asynchronous output reset are retained.
 wire [W-1:0] old_top,old_mid;
 assign top=old_top;assign mid=old_mid;
 reg v1,u1,l1;reg [11:0] x1,y1;reg [W-1:0] d1;reg [T-1:0] t1;
 genvar k;
 generate for(k=0;k<N;k=k+1)begin: line_bank
  localparam B=((W-k*20)>=20)?20:W-k*20;
  PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(B),.DATA_WIDTH_B(B),
   .ADDR_WIDTH_A(AW),.ADDR_WIDTH_B(AW),.DATA_DEPTH_A(2**AW),.DATA_DEPTH_B(2**AW),
   .MODE("DP"),.REGMODE_A("OUTREG"),.REGMODE_B("OUTREG"),
   .WRITEMODE_A("READBEFOREWRITE"),.WRITEMODE_B("READBEFOREWRITE"),
   .IMPLEMENT("20K(FAST)"),.CLKMODE("SYNC")) mem(
   .clka(clk),.clkb(clk),.cea(1'b1),.ceb(1'b1),.ocea(1'b1),.oceb(1'b1),
   .wea(v),.web(1'b0),.addra(wa),.addrb(ra),.dia(d[k*20+:B]),.dib({B{1'b0}}),
   .bea(1'b0),.beb(1'b0),.rsta(!rst),.rstb(!rst),.doa(old_top[k*20+:B]),.dob(old_mid[k*20+:B]),
   .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(32'd0),.ecc_dbiterrinj(32'd0));
 end endgenerate
 always @(posedge clk or negedge rst)begin
  if(!rst)begin
   v1<=0;u1<=0;l1<=0;x1<=0;y1<=0;d1<=0;t1<=0;
   ov<=0;ou<=0;ol<=0;ox<=0;oy<=0;bot<=0;ot<=0;
  end else begin
   v1<=v;u1<=u;l1<=l;x1<=x;y1<=y;d1<=d;t1<=tag;
   ov<=v1;ou<=u1;ol<=l1;ox<=x1;oy<=y1;bot<=d1;ot<=t1;
  end
 end
endmodule

// Four full-resolution 3x3 windows per beat. Windows are causal raster
// neighbourhoods, with replicated top/left boundaries, never cross-row or
// cross-frame neighbours. The centre payload can travel with its feature.
module fx_window3 #(parameter P=8,T=1,WIDTH=1280)(
 input wire clk,rst,v,u,l,input wire [11:0] x,y,
 input wire [4*P-1:0] d,input wire [T-1:0] tag,
 output wire ov,ou,ol,output wire [11:0] ox,oy,
 output wire [36*P-1:0] win,output wire [T-1:0] ot);
 wire [4*P-1:0] a,b,c;
 wire [4*P-1:0] row[0:2];
 reg [P-1:0] h0[0:2],h1[0:2];
 fx_rows2 #(.W(4*P),.T(T),.WIDTH(WIDTH)) rows(
  clk,rst,v,u,l,x,y,d,tag,ov,ou,ol,ox,oy,a,b,c,ot);
 assign row[2]=c;
 assign row[1]=(oy==0)?c:b;
 assign row[0]=(oy<2)?row[1]:a;
 genvar p,r,j;
 generate for(p=0;p<4;p=p+1) begin: lane
  for(r=0;r<3;r=r+1) begin: rr
   for(j=0;j<3;j=j+1) begin: cc
    if(p+j>=2) assign win[(p*9+r*3+j)*P+:P]=row[r][(p+j-2)*P+:P];
    else if(p+j==1) assign win[(p*9+r*3+j)*P+:P]=(ox==0)?row[r][0+:P]:h1[r];
    else assign win[(p*9+r*3+j)*P+:P]=(ox==0)?row[r][0+:P]:h0[r];
   end
  end
 end endgenerate
 integer i;
 always @(posedge clk or negedge rst) begin
  if(!rst) for(i=0;i<3;i=i+1) begin h0[i]<=0;h1[i]<=0;end
  else if(ov) for(i=0;i<3;i=i+1) begin h0[i]<=row[i][2*P+:P];h1[i]<=row[i][3*P+:P];end
 end
endmodule

// Separable [1 2 1]x[1 2 1] Gaussian. No DSPs, no horizontal decimation.
module fx_gauss3 #(parameter T=1,WIDTH=1280)(
 input wire clk,rst,v,u,l,input wire [11:0] x,y,
 input wire [95:0] d,input wire [T-1:0] tag,
 output reg ov,ou,ol,output reg [11:0] ox,oy,
 output reg [95:0] od,output reg [T-1:0] ot);
 reg [23:0] h0,h1;
 reg hv,hu,hl;reg [11:0] hx,hy;reg [119:0] hd;reg [T-1:0] ht;
 wire rv,ru,rl;wire [11:0] rx,ry;wire [119:0] ra,rb,rc;wire [T-1:0] rt;
 fx_rows2 #(.W(120),.T(T),.WIDTH(WIDTH)) rows(
  clk,rst,hv,hu,hl,hx,hy,hd,ht,rv,ru,rl,rx,ry,ra,rb,rc,rt);
 wire [119:0] rm=(ry==0)?rc:rb;
 wire [119:0] rr=(ry<2)?rm:ra;
 integer p,ch;reg [7:0] a,b,c;reg [11:0] s;
 always @(posedge clk or negedge rst) begin
  if(!rst) begin
   h0<=0;h1<=0;hv<=0;hu<=0;hl<=0;hx<=0;hy<=0;hd<=0;ht<=0;
   ov<=0;ou<=0;ol<=0;ox<=0;oy<=0;od<=0;ot<=0;
  end else begin
   hv<=v;hu<=u;hl<=l;hx<=x;hy<=y;ht<=tag;
   if(v) begin
    for(p=0;p<4;p=p+1) for(ch=0;ch<3;ch=ch+1) begin
     a=(p>=2)?d[(p-2)*24+ch*8+:8]:((x==0)?d[ch*8+:8]:((p==0)?h0[ch*8+:8]:h1[ch*8+:8]));
     b=(p>=1)?d[(p-1)*24+ch*8+:8]:((x==0)?d[ch*8+:8]:h1[ch*8+:8]);
     c=d[p*24+ch*8+:8];
     hd[p*30+ch*10+:10]<={2'd0,a}+{1'd0,b,1'b0}+{2'd0,c};
    end
    h0<=d[48+:24];h1<=d[72+:24];
   end
   ov<=rv;ou<=ru;ol<=rl;ox<=rx;oy<=ry;ot<=rt;
   if(rv) for(p=0;p<4;p=p+1) for(ch=0;ch<3;ch=ch+1) begin
    s={2'd0,rr[p*30+ch*10+:10]}+{1'd0,rm[p*30+ch*10+:10],1'b0}+{2'd0,rc[p*30+ch*10+:10]}+12'd8;
    od[p*24+ch*8+:8]<=s[11:4];
   end
  end
 end
endmodule

// Exact 15x15 support: vertical running sum in ERAM, then horizontal running
// sum. Top/left samples are replicated. Rounded reciprocal is within one
// luminance code of division by 225, exhaustively checked by the test runner.
module fx_mean15 #(parameter T=1,WIDTH=1280)(
 input wire clk,rst,v,u,l,input wire [11:0] x,y,
 input wire [31:0] d,input wire [T-1:0] tag,
 output reg ov,ou,ol,output reg [11:0] ox,oy,
 output reg [31:0] od,output reg [T-1:0] ot);
 localparam B=WIDTH/4,CAW=$clog2(B),AW=$clog2(15*B);
 reg [3:0] page;
 wire [3:0] pg=u?4'd0:page;
 wire [AW-1:0] wa=pg*B+x[CAW+1:2];
 wire [AW-1:0] ra=(y<15)?x[CAW+1:2]:wa;
 wire [31:0] old;
 wire [47:0] sums;
 reg v1,u1,l1;reg [11:0] x1,y1;reg [31:0] d1;reg [T-1:0] t1;
 reg [47:0] ns;
 fx_ram #(.W(32),.AW(AW)) history(clk,v,wa,ra,d,old);
 fx_ram #(.W(48),.AW(CAW)) columns(clk,v1,x1[CAW+1:2],x[CAW+1:2],ns,sums);
 integer j;reg [12:0] vs;
 always @* begin
  ns=0;
  for(j=0;j<4;j=j+1) begin
   if(y1==0) vs=({5'd0,d1[j*8+:8]}<<4)-d1[j*8+:8];
   else vs={1'b0,sums[j*12+:12]}+d1[j*8+:8]-old[j*8+:8];
   ns[j*12+:12]=vs[11:0];
  end
 end
 reg v2,u2,l2;reg [11:0] x2,y2;reg [47:0] d2;reg [T-1:0] t2;
 reg [11:0] hist[0:14];reg [15:0] hsum;
 reg [15:0] total[0:3];reg [31:0] product;integer p,i;
 always @* begin
  if(x2==0) begin
   total[0]=({4'd0,d2[0+:12]}<<4)-d2[0+:12];
   for(p=1;p<4;p=p+1) total[p]=total[p-1]-d2[0+:12]+d2[p*12+:12];
  end else begin
   total[0]=hsum-hist[0]+d2[0+:12];
   for(p=1;p<4;p=p+1) total[p]=total[p-1]-hist[p]+d2[p*12+:12];
  end
 end
 always @(posedge clk or negedge rst) begin
  if(!rst) begin
   page<=0;v1<=0;u1<=0;l1<=0;x1<=0;y1<=0;d1<=0;t1<=0;
   v2<=0;u2<=0;l2<=0;x2<=0;y2<=0;d2<=0;t2<=0;
   ov<=0;ou<=0;ol<=0;ox<=0;oy<=0;od<=0;ot<=0;hsum<=0;
   for(i=0;i<15;i=i+1) hist[i]<=0;
  end else begin
   if(v && l)page<=(pg==14)?0:pg+1'b1;
   else if(u)page<=0;
   v1<=v;u1<=u;l1<=l;x1<=x;y1<=y;d1<=d;t1<=tag;
   v2<=v1;u2<=u1;l2<=l1;x2<=x1;y2<=y1;d2<=ns;t2<=t1;
   ov<=v2;ou<=u2;ol<=l2;ox<=x2;oy<=y2;ot<=t2;
   if(v2) begin
    for(p=0;p<4;p=p+1) begin
     product=(total[p]+16'd112)*16'd291;
     od[p*8+:8]<=product[23:16];
    end
    hsum<=total[3];
    for(i=0;i<11;i=i+1)hist[i]<=(x2==0)?d2[0+:12]:hist[i+4];
    for(i=0;i<4;i=i+1)hist[i+11]<=d2[i*12+:12];
   end
  end
 end
endmodule

// M2 detail-preserving smoothing: half aligned centre, half Gaussian.
// Retain the existing three-cycle tag delay; M1 does not consume this output.
module fx_gray3 #(parameter T=1,WIDTH=1280)(
 input wire clk,rst,v,u,l,input wire [11:0] x,y,input wire [31:0] d,input wire [T-1:0] tag,
 output reg ov,ou,ol,output reg [11:0] ox,oy,output reg [31:0] od,output reg [T-1:0] ot);
 wire wv,wu,wl;wire [11:0] wx,wy;wire [287:0] win;wire [T-1:0] wt;
 fx_window3 #(.P(8),.T(T),.WIDTH(WIDTH)) window(clk,rst,v,u,l,x,y,d,tag,wv,wu,wl,wx,wy,win,wt);
 integer p;reg [9:0] htop,hmid,hbot;reg [11:0] gauss_sum;reg [12:0] sum;
 always @(posedge clk or negedge rst)begin
  if(!rst)begin ov<=0;ou<=0;ol<=0;ox<=0;oy<=0;od<=0;ot<=0;end
  else begin
   ov<=wv;ou<=wu;ol<=wl;ox<=wx;oy<=wy;ot<=wt;
   if(wv)for(p=0;p<4;p=p+1)begin
    htop={2'd0,win[(p*9+0)*8+:8]}+{1'b0,win[(p*9+1)*8+:8],1'b0}+{2'd0,win[(p*9+2)*8+:8]};
    hmid={2'd0,win[(p*9+3)*8+:8]}+{1'b0,win[(p*9+4)*8+:8],1'b0}+{2'd0,win[(p*9+5)*8+:8]};
    hbot={2'd0,win[(p*9+6)*8+:8]}+{1'b0,win[(p*9+7)*8+:8],1'b0}+{2'd0,win[(p*9+8)*8+:8]};
    gauss_sum={2'd0,htop}+{1'b0,hmid,1'b0}+{2'd0,hbot};
    sum={1'b0,gauss_sum}+{1'b0,win[(p*9+4)*8+:8],4'b0}+13'd16;
    od[p*8+:8]<=sum[12:5];
   end
  end
 end
endmodule

// One global threshold per frame. Sample 1/16 pixels, estimate two class
// means using a shared 24-cycle restoring divider, and slew by one code/frame.
// The final sampled row leaves three whole rows for the calculation at 720p.
module fx_region_threshold #(parameter WIDTH=1280,HEIGHT=720)(
 input wire clk,rst,v,u,l,input wire [11:0] y,input wire [7:0] gray,input wire enable,
 output wire [7:0] threshold);
 reg [7:0] active,pending;
 reg [23:0] dark_sum,light_sum;
 reg [15:0] dark_count,light_count;
 wire sample=v&&enable&&y[1:0]==0;
 assign threshold=u?pending:active;
 wire dark=gray<threshold;
 wire [23:0] ds=(u?24'd0:dark_sum)+((sample&&dark)?{16'd0,gray}:24'd0);
 wire [23:0] ls=(u?24'd0:light_sum)+((sample&&!dark)?{16'd0,gray}:24'd0);
 wire [15:0] dc=(u?16'd0:dark_count)+((sample&&dark)?16'd1:16'd0);
 wire [15:0] lc=(u?16'd0:light_count)+((sample&&!dark)?16'd1:16'd0);
 reg busy,second,have_dark,have_light;reg [4:0] step;
 reg [23:0] work,saved_light;reg [15:0] denominator,saved_count;
 reg [16:0] remainder,trial;reg [23:0] next_work;reg [7:0] mean_dark,mean_light;
 reg signed [9:0] candidate;
 always @(posedge clk or negedge rst)begin
  if(!rst)begin
   active<=64;pending<=64;dark_sum<=0;light_sum<=0;dark_count<=0;light_count<=0;
   busy<=0;second<=0;have_dark<=0;have_light<=0;step<=0;work<=0;saved_light<=0;denominator<=1;saved_count<=0;remainder<=0;mean_dark<=0;
  end else begin
   if(v)begin
    if(u)active<=pending;
    dark_sum<=ds;light_sum<=ls;dark_count<=dc;light_count<=lc;
    if(enable&&l&&y==((HEIGHT-1)/4)*4&&!busy)begin
     busy<=1;second<=0;step<=0;remainder<=0;work<=ds;denominator<=dc?dc:16'd1;
     saved_light<=ls;saved_count<=lc;have_dark<=dc!=0;have_light<=lc!=0;
    end
   end
   if(busy)begin
    trial={remainder[15:0],work[23]};next_work={work[22:0],1'b0};
    if(trial>={1'b0,denominator})begin remainder<=trial-{1'b0,denominator};next_work[0]=1;end
    else remainder<=trial;
    work<=next_work;
    if(step==23)begin
     if(!second)begin
      mean_dark<=next_work[7:0];work<=saved_light;denominator<=saved_count?saved_count:16'd1;
      remainder<=0;step<=0;second<=1;
     end else begin
      mean_light=next_work[7:0];busy<=0;
      if(have_dark&&have_light&&({1'b0,mean_light}>={1'b0,mean_dark}+9'd8))
       candidate=({2'd0,mean_dark}+{2'd0,mean_light})>>>1;
      else if(!have_dark)candidate=$signed({2'd0,mean_light})-10'sd8;
      else if(!have_light)candidate=$signed({2'd0,mean_dark})-10'sd8;
      else candidate=$signed(({2'd0,mean_dark}+{2'd0,mean_light})>>1)-10'sd8;
      if(candidate<8)candidate=8;else if(candidate>224)candidate=224;
      if(candidate>$signed({2'd0,pending})+10'sd1)pending<=pending+1'b1;
      else if(candidate+10'sd1<$signed({2'd0,pending}))pending<=pending-1'b1;
     end
    end else step<=step+1'b1;
   end
  end
 end
endmodule

// M1 stores two 2x2 decisions per beat. M2 uses the same two bits as a
// saturating confidence counter for a 4x2 region. All displayed pixels remain
// full resolution. Very strong current contours bypass confirmation; a flat
// current pixel clears immediately, even with a high historical confidence.
module fx_temporal_mono #(parameter WIDTH=1280,HEIGHT=720)(
 input wire clk,rst,v,u,l,input wire [11:0] x,y,input wire [95:0] rgb,
 input wire [2:0] mode,input wire [7:0] cls,
 output reg ov,ou,ol,output reg [11:0] ox,oy,output reg [95:0] od);
 localparam B=WIDTH/4,CAW=$clog2(B),N=B*((HEIGHT+1)/2),AW=$clog2(N);
 reg [AW-1:0] base;wire [AW-1:0] address=(u?{AW{1'b0}}:base)+(x>>2);
 reg have_frame,history_frame;reg [2:0] last_mode;
 wire mono=mode==1||mode==2;
 wire use_history=u?(have_frame&&last_mode==mode&&mono):history_frame;
 reg sv,su,sl,sh;reg [11:0] sx,sy;reg [95:0] sd;reg [2:0] sm;reg [7:0] sc;reg [AW-1:0] sa;
 wire [1:0] previous;wire [1:0] confidence=sh?previous:2'd0;
 wire [3:0] top_count;reg [3:0] black,counts;reg [2:0] total0,total1;reg new0,new1;reg [1:0] next_history;
 wire pair_write=sv&&(sm==1||sm==2)&&sy[0];
 fx_ram #(.W(2),.AW(AW)) history(clk,pair_write,sa,address,next_history,previous);
 fx_ram #(.W(4),.AW(CAW)) first_row(clk,sv&&(sm==1||sm==2)&&!sy[0],sx[CAW+1:2],x[CAW+1:2],counts,top_count);
 integer p,i;
 always @*begin
  counts=0;
  for(p=0;p<4;p=p+1)begin
   if(sm==2)begin
    black[p]=(sc[p*2+:2]==3)||((sc[p*2+:2]!=0)&&confidence[1]);
    if(sc[p*2+:2]>=2)counts[0]=1;
   end else black[p]=(sc[p*2+:2]==2)||((sc[p*2+:2]==1)&&sh&&previous[p/2]);
  end
  total0=0;total1=0;new0=0;new1=0;next_history=0;
  if(sm==2)begin
   if(counts[0]||top_count[0])next_history=(confidence==3)?2'd3:confidence+1'b1;
   else next_history=(confidence==0)?2'd0:confidence-1'b1;
  end else begin
   counts[1:0]={1'b0,black[0]}+black[1];counts[3:2]={1'b0,black[2]}+black[3];
   total0={1'b0,top_count[1:0]}+{1'b0,counts[1:0]};total1={1'b0,top_count[3:2]}+{1'b0,counts[3:2]};
   new0=(total0>2)||((total0==2)&&sh&&previous[0]);new1=(total1>2)||((total1==2)&&sh&&previous[1]);
   next_history={new1,new0};
  end
 end
 always @(posedge clk or negedge rst)begin
  if(!rst)begin
   base<=0;have_frame<=0;history_frame<=0;last_mode<=0;sv<=0;su<=0;sl<=0;sh<=0;sx<=0;sy<=0;sd<=0;sm<=0;sc<=0;sa<=0;
   ov<=0;ou<=0;ol<=0;ox<=0;oy<=0;od<=0;
  end else begin
   sv<=v;su<=u;sl<=l;sh<=use_history;sx<=x;sy<=y;sd<=rgb;sm<=mode;sc<=cls;sa<=address;
   if(v)begin
    if(u)begin base<=0;history_frame<=use_history;end
    if(l&&y[0])base<=((u?{AW{1'b0}}:base)+B);
    if(l&&y==HEIGHT-1)begin have_frame<=mono;last_mode<=mode;end
   end
   ov<=sv;ou<=su;ol<=sl;ox<=sx;oy<=sy;
   if(sv)begin
    od<=sd;
    if(sm==1||sm==2)for(i=0;i<4;i=i+1)od[95-i*24-:24]<=black[i]?24'h000000:24'hffffff;
   end
  end
 end
endmodule
