`timescale 1ns/1ps
// Four RGB pixels per clock. Histories contain two complete 8x8-block means.
// A block is harvested on its LAST pixel, including the bottom image row.
module motion_blocks_v5 #(
 parameter IMG_WIDTH=1280, IMG_HEIGHT=720
)(
 input wire I_clk,I_rst_n,I_flush,I_tvalid,I_tuser,I_tlast,
 input wire [95:0] I_rgb,input wire [1:0] I_profile,
 output reg O_row_v,output reg [IMG_WIDTH/16-1:0] O_row_act,
 output reg O_frame_last,output wire O_ready
);
 localparam NX=IMG_WIDTH/8, BX=IMG_WIDTH/16;
 localparam AW=$clog2((IMG_WIDTH/8)*(IMG_HEIGHT/8));
 reg [9:0] first_lum;
 wire [15:0] acc_rd;reg acc_we;reg [7:0] acc_wa;reg [15:0] acc_wd;
 reg [9:0] xb;reg [10:0] yy;reg [1:0] frames;
 wire [9:0] x=I_tuser?10'd0:xb;
 wire [10:0] y=I_tuser?11'd0:yy;
 wire [7:0] sc=x>>1;
 wire [9:0] sr=y>>3;
 wire [23:0] p0=I_rgb[95:72],p1=I_rgb[71:48],p2=I_rgb[47:24],p3=I_rgb[23:0];
 wire [9:0] rs={2'd0,p0[23:16]}+p1[23:16]+p2[23:16]+p3[23:16];
 wire [9:0] gs={2'd0,p0[15:8]}+p1[15:8]+p2[15:8]+p3[15:8];
 wire [9:0] bs={2'd0,p0[7:0]}+p1[7:0]+p2[7:0]+p3[7:0];
 wire [11:0] weighted={2'd0,rs}+{1'd0,gs,1'b0}+{2'd0,bs};
 wire [9:0] lum=weighted[11:2];
 wire [10:0] pair_lum={1'b0,first_lum}+{1'b0,lum};
 wire [14:0] sum=(y[2:0]==0)?{4'd0,pair_lum}:({1'b0,acc_rd[13:0]}+{4'd0,pair_lum});
 PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(16),.DATA_WIDTH_B(16),.ADDR_WIDTH_A(8),.ADDR_WIDTH_B(8),
  .DATA_DEPTH_A(256),.DATA_DEPTH_B(256),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("READBEFOREWRITE"),.WRITEMODE_B("READBEFOREWRITE"),.IMPLEMENT("20K(FAST)"),.CLKMODE("SYNC")) accumulator(
  .dia(acc_wd),.dib(16'd0),.addra(acc_wa),.addrb(sc),.clka(I_clk),.clkb(I_clk),
  .cea(1'b1),.ceb(1'b1),.ocea(1'b0),.oceb(1'b0),.wea(acc_we),.web(1'b0),
  .bea(1'b0),.beb(1'b0),.rsta(1'b0),.rstb(1'b0),.doa(),.dob(acc_rd),
  .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(32'd0),.ecc_dbiterrinj(32'd0));
 wire harvest=I_tvalid && y[2:0]==3'd7 && x[0];
 wire [AW-1:0] rdaddr=sr*NX+sc;
 wire [15:0] history;
 reg we;reg [AW-1:0] wa;reg [15:0] wd;
 PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(16),.DATA_WIDTH_B(16),
  .ADDR_WIDTH_A(AW),.ADDR_WIDTH_B(AW),.DATA_DEPTH_A(2**AW),.DATA_DEPTH_B(2**AW),
  .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("READBEFOREWRITE"),.WRITEMODE_B("READBEFOREWRITE"),
  .IMPLEMENT("20K(FAST)"),.CLKMODE("SYNC")) hist(
  .dia(wd),.dib(16'd0),.addra(wa),.addrb(rdaddr),.clka(I_clk),.clkb(I_clk),
  .cea(1'b1),.ceb(1'b1),.ocea(1'b0),.oceb(1'b0),.wea(we),.web(1'b0),
  .bea(1'b0),.beb(1'b0),.rsta(1'b0),.rstb(1'b0),.doa(),.dob(history),
  .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(32'd0),.ecc_dbiterrinj(32'd0));
 reg pv,pwarm;reg [7:0] avg,pc;reg [9:0] pr;reg [AW-1:0] pa;
 reg left_bit;reg [BX-1:0] upper,coarse;
 reg emit,emit_last;
 wire [7:0] d1=(avg>history[7:0])?(avg-history[7:0]):(history[7:0]-avg);
 wire [7:0] d2=(avg>history[15:8])?(avg-history[15:8]):(history[15:8]-avg);
 wire [7:0] t1=(I_profile==1)?8'd6:(I_profile==2)?8'd12:8'd8;
 wire [7:0] t2=(I_profile==1)?8'd10:(I_profile==2)?8'd18:8'd12;
 wire active=!pwarm && ((d1>=t1)||(d2>=t2));
 assign O_ready=frames>=2;
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n)begin
   first_lum<=0;acc_we<=0;acc_wa<=0;acc_wd<=0;xb<=0;yy<=0;frames<=0;pv<=0;pwarm<=1;avg<=0;pc<=0;pr<=0;pa<=0;
   we<=0;wa<=0;wd<=0;left_bit<=0;upper<=0;coarse<=0;emit<=0;emit_last<=0;
   O_row_v<=0;O_row_act<=0;O_frame_last<=0;
  end else begin
   acc_we<=0;we<=0;pv<=harvest;emit<=0;O_row_v<=emit;O_frame_last<=emit&&emit_last;
   if(emit)O_row_act<=coarse;
   if(I_tuser)begin xb<=I_tvalid?10'd1:10'd0;yy<=0;end
   else if(I_tvalid)begin
    if(I_tlast)begin xb<=0;yy<=(yy==IMG_HEIGHT-1)?11'd0:yy+1'b1;end
    else xb<=xb+1'b1;
   end
   if(I_tvalid)begin
    if(!x[0])first_lum<=lum;
    else begin acc_we<=1;acc_wa<=sc;acc_wd<={2'd0,sum[13:0]};end
    if(y==IMG_HEIGHT-1 && I_tlast && frames!=3)frames<=frames+1'b1;
   end
   if(harvest)begin avg<=sum[13:6];pc<=sc;pr<=sr;pa<=rdaddr;pwarm<=frames<2;end
   if(pv)begin
    we<=1;wa<=pa;wd<={history[7:0],avg};
    if(!pc[0])left_bit<=active;
    else if(!pr[0])upper[pc>>1]<=left_bit|active;
    else begin
     coarse[pc>>1]<=upper[pc>>1]|left_bit|active;
     if(pc==NX-1)begin emit<=1;emit_last<=pr==(IMG_HEIGHT/8-1);end
    end
   end
   if(I_flush)begin
    frames<=0;pv<=0;we<=0;emit<=0;O_row_v<=0;O_frame_last<=0;
    upper<=0;coarse<=0;left_bit<=0;
   end
  end
 end
endmodule

// Remove isolated blocks, then vote at each location over three frames.
// Temporal history is ERAM, not a resettable 7200-bit register array.
module motion_filter_v5 #(parameter BX=80,BY=45)(
 input wire I_clk,I_rst_n,I_flush,I_row_v,I_frame_last,I_region_ready,
 input wire [BX-1:0] I_row_act,
 output reg O_row_v,output reg [BX-1:0] O_row_act,output reg O_frame_last
);
 localparam AW=$clog2(BX*BY);
 reg [BX-1:0] above,center,work,result;
 reg have,flush_last,work_last;
 reg [6:0] cy,wy;reg [1:0] frames,state;
 reg [7:0] ri,di;reg dv,db;reg [1:0] accept_delay;
 wire [AW-1:0] ra=wy*BX+ri;
 wire [1:0] histdata;
 reg we;reg [AW-1:0] wa;reg [1:0] wd;
 PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(2),.DATA_WIDTH_B(2),.ADDR_WIDTH_A(AW),.ADDR_WIDTH_B(AW),
  .DATA_DEPTH_A(2**AW),.DATA_DEPTH_B(2**AW),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("READBEFOREWRITE"),.WRITEMODE_B("READBEFOREWRITE"),
  .IMPLEMENT("20K(FAST)"),.CLKMODE("SYNC")) hist(
  .dia(wd),.dib(2'd0),.addra(wa),.addrb(ra),.clka(I_clk),.clkb(I_clk),
  .cea(1'b1),.ceb(1'b1),.ocea(1'b0),.oceb(1'b0),.wea(we),.web(1'b0),
  .bea(1'b0),.beb(1'b0),.rsta(1'b0),.rstb(1'b0),.doa(),.dob(histdata),
  .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(32'd0),.ecc_dbiterrinj(32'd0));
 function [BX-1:0] clean;
  input [BX-1:0] a,b,c;reg [BX-1:0] allrows,neighbours;
  begin allrows=a|b|c;neighbours=a|c|(allrows<<1)|(allrows>>1);clean=b&neighbours;end
 endfunction
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   above<=0;center<=0;work<=0;result<=0;have<=0;flush_last<=0;work_last<=0;
   accept_delay<=0;cy<=0;wy<=0;frames<=0;state<=0;ri<=0;di<=0;dv<=0;db<=0;we<=0;wa<=0;wd<=0;
   O_row_v<=0;O_row_act<=0;O_frame_last<=0;
  end else begin
   O_row_v<=0;O_frame_last<=0;we<=0;dv<=0;
   if(I_row_v)begin
    if(!have)begin center<=I_row_act;above<=0;cy<=0;have<=1;end
    else begin
     work<=clean(above,center,I_row_act);wy<=cy;work_last<=0;
     above<=center;center<=I_row_act;cy<=cy+1'b1;
     state<=1;ri<=0;dv<=0;
    end
    if(I_frame_last)flush_last<=1;
   end
   if(state==1)begin
    if(ri<BX)begin dv<=1;db<=work[ri];di<=ri;ri<=ri+1'b1;end
    if(dv)begin
     we<=1;wa<=wy*BX+di;wd<={histdata[0],db};
     result[di]<=frames>=2 && ((db&histdata[0])|(db&histdata[1])|(histdata[0]&histdata[1]));
     if(di==BX-1)begin state<=2;dv<=0;end
    end
   end else if(state==2 && I_region_ready)begin
    O_row_v<=1;O_row_act<=result;O_frame_last<=work_last;
    if(work_last)begin have<=0;above<=0;state<=0;if(frames!=3)frames<=frames+1'b1;end
    else if(flush_last)begin state<=3;accept_delay<=2;end
    else state<=0;
   end else if(state==3)begin
    if(accept_delay!=0)accept_delay<=accept_delay-1'b1;
    else if(I_region_ready)begin
     work<=clean(above,center,{BX{1'b0}});wy<=cy;work_last<=1;
     flush_last<=0;ri<=0;dv<=0;state<=1;
    end
   end
   if(I_flush)begin
    above<=0;center<=0;work<=0;result<=0;have<=0;flush_last<=0;work_last<=0;
    cy<=0;wy<=0;frames<=0;state<=0;ri<=0;dv<=0;we<=0;O_row_v<=0;O_frame_last<=0;O_row_act<=0;
   end
  end
 end
endmodule

module motion_v5 #(parameter IMG_WIDTH=1280,IMG_HEIGHT=720)(
 input wire I_clk,I_rst_n,I_flush,I_tvalid,I_tuser,I_tlast,
 input wire [95:0] I_rgb,input wire [1:0] I_profile,
 output wire O_ready,output reg O_toggle,output reg [3:0] O_num,output reg [191:0] O_targets
);
 wire rv,fl,fv,ff,region_ready;wire [IMG_WIDTH/16-1:0] rows,filtered;
 motion_blocks_v5 #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) blocks(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_flush(I_flush),.I_tvalid(I_tvalid),.I_tuser(I_tuser),.I_tlast(I_tlast),
  .I_rgb(I_rgb),.I_profile(I_profile),.O_row_v(rv),.O_row_act(rows),.O_frame_last(fl),.O_ready(O_ready));
 motion_filter_v5 #(.BX(IMG_WIDTH/16),.BY(IMG_HEIGHT/16)) filter(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_flush(I_flush),.I_region_ready(region_ready),.I_row_v(rv),.I_row_act(rows),.I_frame_last(fl),
  .O_row_v(fv),.O_row_act(filtered),.O_frame_last(ff));
 wire done,valid;wire [3:0] count,conf;wire [191:0] targets;wire [2:0] shape;
 wire [3:0] minblocks=I_profile==2?4'd3:4'd2;
 multi_region #(.BX(IMG_WIDTH/16),.BY(IMG_HEIGHT/16),.NREG(6),.NOUT(3),.BLK_SHF(4),
  .FILL_DEN(8),.NOUT_W(4),.RUNTIME_MIN(1),.SHAPE_ENABLE(0)) regions(
  .I_clk(I_clk),.I_rst_n(I_rst_n && !I_flush),.I_row_v(fv),.I_row_act(filtered),.I_min_blk(minblocks),
  .I_frame_last(ff),.O_valid(valid),.O_done(done),.O_num(count),.O_tgt(targets),.O_shape(shape),.O_conf(conf),.O_row_ready(region_ready));
 reg [2:0] hold;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin O_num<=0;O_targets<=0;O_toggle<=0;hold<=0;end
  else if(I_flush)begin O_num<=0;O_targets<=0;O_toggle<=~O_toggle;hold<=0;end
  else if(done)begin
   O_toggle<=~O_toggle;
   if(count!=0)begin O_num<=count;O_targets<=targets;hold<=6;end
   else if(hold>1)hold<=hold-1'b1;
   else begin O_num<=0;O_targets<=0;hold<=0;end
  end
 end
endmodule

// Frame-level lamp confirmation; a missing/ambiguous lamp invalidates immediately.
module light_confirm_v5(input wire I_clk,I_rst_n,I_enable,I_flush,I_eval,
 input wire I_valid,input wire [1:0] I_color,input wire [2:0] I_colors,
 input wire [11:0] I_x0,I_y0,I_x1,I_y1,
 output reg O_valid,output reg [1:0] O_color,output reg [2:0] O_colors,
 output reg [11:0] O_x0,O_y0,O_x1,O_y1,output reg O_eval);
 // Display/phase confirmation only. Raw fresh lamp observations remain on
 // the separate arrow/car snapshot path. Two hits within six evaluations
 // acquire the same lamp; at most eight missed frames retain its display.
 reg [4:0] evidence;reg [2:0] missed;reg [1:0] last_color;reg [11:0] cx,cy;
 wire [11:0] nx=(I_x0+I_x1)>>1,ny=(I_y0+I_y1)>>1;
 wire [11:0] dx=nx>cx?nx-cx:cx-nx,dy=ny>cy?ny-cy:cy-ny;
 wire single=(I_colors==1)||(I_colors==2)||(I_colors==4);
 wire same=(I_color==last_color)&&(dx<=64)&&(dy<=64)&&((evidence!=0)||O_valid);
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin evidence<=0;missed<=0;last_color<=0;cx<=0;cy<=0;O_valid<=0;O_color<=0;O_colors<=0;
   O_x0<=0;O_y0<=0;O_x1<=0;O_y1<=0;O_eval<=0;end
  else begin
   O_eval<=I_eval;
   if(I_eval)begin
    evidence<={evidence[3:0],1'b0};
    if(I_valid&&single)begin
     last_color<=I_color;cx<=nx;cy<=ny;missed<=0;
     evidence<=same?{evidence[3:0],1'b1}:5'b00001;
     O_valid<=same && (O_valid || evidence!=0);
     O_colors<=(same && (O_valid || evidence!=0))?I_colors:3'd0;O_color<=I_color;
     O_x0<=I_x0;O_y0<=I_y0;O_x1<=I_x1;O_y1<=I_y1;
    end else if(!single && I_colors!=0)begin
     // Conflicts invalidate immediately; never bridge a colour transition.
     evidence<=0;missed<=0;O_valid<=0;O_colors<=I_colors;
    end else begin
     if(missed!=7)missed<=missed+1'b1;
     else begin O_valid<=0;O_colors<=0;evidence<=0;end
     if(!O_valid)O_colors<=0;
    end
   end
   if(!I_enable||I_flush)begin evidence<=0;missed<=0;O_valid<=0;O_colors<=0;O_color<=0;O_x0<=0;O_y0<=0;O_x1<=0;O_y1<=0;end
  end
 end
endmodule
