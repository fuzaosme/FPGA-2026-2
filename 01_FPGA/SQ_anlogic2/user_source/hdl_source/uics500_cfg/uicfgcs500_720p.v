`timescale 1ns/1ps
// SC500CS 720p60 setup and verified automatic exposure updates.
// Group 0 packs the five exposure/gain registers. A request is latched in full;
// later requests cannot tear a transaction. Readback and ACK are both required.
module uicfgcs500_720p #(
 parameter CLK_DIV=239,parameter CLOCK_HZ=24000000
)(
 input wire I_clk,I_rst_n,I_ae_req,input wire [15:0] I_ae,I_ag,
 output wire O_cam_scl,inout wire IO_cam_sda,
 output reg O_cfg_done,O_ae_cfg_done,
 output reg O_fault,output reg [7:0] O_error,
 output reg [15:0] O_applied_ae,output reg [7:0] O_applied_ag);
 localparam IDLE=0,PREPARE=1,START=2,WAIT_BUSY=3,WAIT_DONE=4,FINISH=5,FAULT=6;
 reg [2:0] state;
 reg [8:0] index;
 reg updating,reading;
 reg [17:0] post_delay;
 reg req;reg [31:0] wr;reg [7:0] wc,rc;
 wire busy,error;wire [7:0] rd;
 reg [1:0] bs,es;reg transaction_error;
 reg [23:0] timeout_count;
 reg [1:0] retries;
 reg [15:0] exposure;reg [7:0] gain_index;
 wire [15:0] gain_value;
 wire [31:0] init_word;wire [8:0] init_size;
 reg [15:0] address;reg [7:0] value;
 wire [7:0] expected=(index==0)?{4'd0,exposure[15:12]}:
  (index==1)?exposure[11:4]:(index==2)?{exposure[3:0],4'd0}:
  (index==3)?gain_value[15:8]:gain_value[7:0];
 uics500reg_720p60 init_table(index,init_word,init_size);
 SC500GainTbl gain_table(I_clk,I_rst_n,gain_index[6:0]-7'd16,gain_value);
 uii2c #(.WMEN_LEN(4),.RMEN_LEN(1),.CLK_DIV(CLK_DIV)) bus(
  .I_clk(I_clk),.I_rstn(I_rst_n),.O_iic_scl(O_cam_scl),.IO_iic_sda(IO_cam_sda),
  .I_wr_data(wr),.I_wr_cnt(wc),.O_rd_data(rd),.I_rd_cnt(rc),
  .I_iic_mode(1'b1),.I_iic_req(req),.O_iic_busy(busy),.O_iic_bus_error(error));
 always @(*)begin
  address=0;value=0;
  if(reading)begin
   case(index)0:address=16'h3e00;1:address=16'h3e01;2:address=16'h3e02;
    3:address=16'h3e08;4:address=16'h3e09;endcase
   value=expected;
  end else if(updating)begin
   case(index)
    0:begin address=16'h3800;value=8'h00;end
    1:begin address=16'h3e00;value={4'd0,exposure[15:12]};end
    2:begin address=16'h3e01;value=exposure[11:4];end
    3:begin address=16'h3e02;value={exposure[3:0],4'd0};end
    4:begin address=16'h3e08;value=gain_value[15:8];end
    5:begin address=16'h3e09;value=gain_value[7:0];end
    6:begin address=16'h3800;value=8'h10;end
    7:begin address=16'h3800;value=8'h60;end
   endcase
  end else begin address=init_word[23:8];value=init_word[7:0];end
 end
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   state<=PREPARE;index<=0;updating<=0;reading<=0;post_delay<=0;
   req<=0;wr<=0;wc<=4;rc<=0;bs<=0;es<=0;transaction_error<=0;
   timeout_count<=0;retries<=0;exposure<=1990;gain_index<=80;
   O_cfg_done<=0;O_ae_cfg_done<=0;O_fault<=0;O_error<=0;O_applied_ae<=1990;O_applied_ag<=80;
  end else begin
   bs<={bs[0],busy};es<={es[0],error};
   if(es[1]&&(state==WAIT_BUSY||state==WAIT_DONE))transaction_error<=1;
   if(state==WAIT_BUSY||state==WAIT_DONE)begin
    timeout_count<=timeout_count+1'b1;
   end else timeout_count<=0;
   case(state)
    IDLE:begin
     req<=0;
     if(I_ae_req)begin
      if(I_ae<3||I_ae>1990||I_ag<16||I_ag>112)begin O_error[3]<=1;end
      else begin exposure<=I_ae;gain_index<=I_ag[7:0];updating<=1;reading<=0;
       O_ae_cfg_done<=0;index<=0;retries<=0;state<=PREPARE;end
     end
    end
    PREPARE:begin
     // The vendor table requires 10 ms between stream-on and its last write.
     if(!updating&&!reading&&index==115&&post_delay<CLOCK_HZ/100)post_delay<=post_delay+1'b1;
     else if(!updating&&!reading&&index==init_size)begin reading<=1;index<=0;state<=PREPARE;end
     else begin
      wr<={value,address[7:0],address[15:8],8'h6c};wc<=reading?8'd3:8'd4;rc<=reading?8'd1:8'd0;
      transaction_error<=0;state<=START;
     end
    end
    START:if(!bs[1])begin req<=1;state<=WAIT_BUSY;end
    WAIT_BUSY:if(bs[1])begin req<=0;state<=WAIT_DONE;end
    WAIT_DONE:if(!bs[1])begin state<=FINISH;end
    FINISH:begin
     if(transaction_error||(reading&&rd!=expected))begin
      if(transaction_error)O_error[0]<=1;else O_error[1]<=1;
      if(retries==2)begin O_fault<=1;state<=FAULT;end
      else begin retries<=retries+1'b1;state<=PREPARE;end
     end else begin
      retries<=0;
      if(reading&&index==4)begin
       O_cfg_done<=1;O_ae_cfg_done<=1;O_applied_ae<=exposure;O_applied_ag<=gain_index;
       reading<=0;state<=IDLE;
      end else if(updating&&!reading&&index==7)begin reading<=1;index<=0;state<=PREPARE;end
      else begin index<=index+1'b1;state<=PREPARE;end
     end
    end
    FAULT:begin req<=0;O_ae_cfg_done<=0;end
   endcase
   // Timeout wins over a coincident bus transition.
   if((state==WAIT_BUSY||state==WAIT_DONE)&&timeout_count>=CLOCK_HZ/40)begin
    state<=FAULT;O_fault<=1;O_ae_cfg_done<=0;O_error[2]<=1;req<=0;
   end
  end
 end
endmodule
