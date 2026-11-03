`timescale 1ns/1ps
// EVENT transmitter for the HC-05 car link.
//
// One 0->1 appearance of a class is one command event and is queued once
// under normal event cadence (the queue-full case is flagged by O_overflow).
// A class that stays high produces no further bytes: the car owns the
// "new command vs. still the same box" decision, so re-sending the current
// level would look like a brand new command and is forbidden here. The car
// side keeps its 2 s / 10 s event cooldown; this module never guesses it.
//
// The 100 ms H heartbeat is a link keepalive only. It carries no action state,
// no cooldown and no fault clearing information.
module car_command_tx #(
 parameter integer CLK_FREQ=75000000,
 parameter integer BAUD=9600,
 parameter integer HEARTBEAT_MS=100,
 // STM32 Motor_Apply blocks its polled RX for 2ms during a direction change.
 // Leave a 3ms idle gap between bytes so a multi-class event group cannot
 // overrun that blocking window. This is byte pacing, not recognition debounce.
 parameter integer INTER_BYTE_MS=3
)(
 input wire I_clk,I_rst_n,
 // [4:0] = right,left,green,yellow,red. One-shot rising-edge event classes;
 // several bits may be set in the same cycle for a simultaneous appearance.
 input wire [4:0] I_event_classes,
 input wire I_event_valid,
 input wire I_manual_valid,input wire [7:0] I_manual_byte,
 output reg [7:0] O_last_cmd,output wire O_tx,
 // Sticky: an automatic/manual byte wanted the queue while all 16 entries were
 // still busy. The staged class is delivered once the queue drains, but two
 // events of the SAME class arriving while the queue is full coalesce into one
 // staged bit, so this flag means "delayed/merged", not "every event got its
 // own byte". Normal 60 fps event traffic never reaches this state.
 output reg O_overflow
);
 localparam [7:0] STOP="S",FORWARD="F",LEFT="L",RIGHT="R",HOLD="H";
 localparam integer MS_DIV=CLK_FREQ/1000;
 localparam integer MS_BITS=(MS_DIV<2)?1:$clog2(MS_DIV);
 localparam integer HB_BITS=$clog2(HEARTBEAT_MS+1);
 localparam integer GAP_CYCLES=(CLK_FREQ/1000)*INTER_BYTE_MS;
 localparam integer GAP_BITS=(GAP_CYCLES<2)?1:$clog2(GAP_CYCLES);
 // Byte queue: only command bytes are ever stored, so an event cannot be
 // replayed or merged into a later level. Depth 16 was already proven above
 // the 60 fps event rate (a byte leaves every ~4.04 ms).
 reg [7:0] queue[0:15];
 reg [3:0] read_ptr,write_ptr;
 reg [4:0] count;
 // Event group staging: a simultaneous group is normalized to one S byte, or
 // to turn bytes (L/R in fixed order) or one F byte. Remaining classes stay
 // here across as many cycles as the busy UART needs; a same-class repeat can
 // merge into the same bit while staged (see O_overflow).
 reg [4:0] ev_pending;
 reg manual_pending,boot_pending,active;
 reg [MS_BITS-1:0] ms_count;
 reg [HB_BITS-1:0] heartbeat_age;
 reg [GAP_BITS-1:0] gap_count;
 reg [1:0] tx_state;
 reg tx_start;
 reg [7:0] tx_data;
 wire tx_busy;
 wire ms_tick=ms_count==MS_DIV-1;
 // Red/yellow are the same STOP action: a group that contains either sends one
 // S and drops the rest. Otherwise a direction outranks green (a turn in the
 // same frame must not be overwritten by a later F), and a pure green sends F.
 wire [4:0] ev_norm=(I_event_classes[0]||I_event_classes[1])?5'b00001:
                     ((I_event_classes[3]||I_event_classes[4])?
                      {I_event_classes[4],I_event_classes[3],3'b000}:
                      {2'b00,I_event_classes[2],2'b00});
 // Fixed dequeue order for a staged group: S, then L, then R, then F (F never
 // shares a group with S/L/R after normalization, so the last case is a pair
 // that can only be reached by a test forcing I_event_classes directly).
 reg [4:0] ev_take;
 reg [7:0] ev_byte;
 always @* begin
  ev_take=0;ev_byte=HOLD;
  if(ev_pending[0]||ev_pending[1])begin ev_take=ev_pending&5'b00011;ev_byte=STOP;end
  else if(ev_pending[3])begin ev_take=5'b01000;ev_byte=LEFT;end
  else if(ev_pending[4])begin ev_take=5'b10000;ev_byte=RIGHT;end
  else if(ev_pending[2])begin ev_take=5'b00100;ev_byte=FORWARD;end
 end
 reg [7:0] manual_byte_sel;
 always @* begin
  case(I_manual_byte)
   FORWARD:manual_byte_sel=FORWARD;
   LEFT:manual_byte_sel=LEFT;
   RIGHT:manual_byte_sel=RIGHT;
   default:manual_byte_sel=STOP;
  endcase
 end
 wire boot_ready=boot_pending;
 wire manual_ready=manual_pending;
 wire event_ready=|ev_pending;
 // A PC manual byte is one queued one-shot byte and never vetoes an event.
 wire push=count<16&&(boot_ready||manual_ready||event_ready||
   (heartbeat_age>=HEARTBEAT_MS&&count==0&&!active&&tx_state==0&&
    !event_ready&&!manual_pending));
 wire [7:0] push_byte=boot_ready?STOP:
   manual_ready?manual_byte_sel:
   event_ready?ev_byte:HOLD;
 wire push_blocked=(boot_ready||manual_ready||event_ready)&&count>=16;
 wire pop=count!=0&&!active&&tx_state==0;
 wire [7:0] head=queue[read_ptr];
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   read_ptr<=0;write_ptr<=0;count<=0;ev_pending<=0;manual_pending<=0;
   boot_pending<=1;active<=0;ms_count<=0;heartbeat_age<=0;gap_count<=0;
   tx_state<=0;tx_start<=0;tx_data<=HOLD;O_last_cmd<=HOLD;O_overflow<=0;
  end else begin
   tx_start<=0;
   if(ms_tick)begin
    ms_count<=0;
    if(heartbeat_age<HEARTBEAT_MS)heartbeat_age<=heartbeat_age+1'b1;
   end else ms_count<=ms_count+1'b1;
   // Rising-edge events are captured here; a held class never re-enters.
   if(I_event_valid)ev_pending<=ev_pending|ev_norm;
   if(I_manual_valid)manual_pending<=1;
   if(push_blocked)O_overflow<=1;
   if(push)begin
    queue[write_ptr]<=push_byte;write_ptr<=write_ptr+1'b1;heartbeat_age<=0;
    if(boot_ready)boot_pending<=0;
    else if(manual_ready)manual_pending<=0;
    else if(event_ready)ev_pending<=((I_event_valid?ev_pending|ev_norm:ev_pending)&~ev_take);
   end
   if(pop)begin
    tx_data<=head;read_ptr<=read_ptr+1'b1;active<=1;
   end
   case({push,pop})
    2'b10:count<=count+1'b1;
    2'b01:count<=count-1'b1;
    default:begin end
   endcase
   case(tx_state)
    0:if(active)begin
     O_last_cmd<=tx_data;tx_start<=1;tx_state<=1;active<=0;
    end
    1:if(tx_busy)tx_state<=2;
    2:if(!tx_busy)begin
     if(GAP_CYCLES==0)tx_state<=0;
     else begin gap_count<=GAP_CYCLES-1;tx_state<=3;end
    end
    3:if(gap_count==0)tx_state<=0;else gap_count<=gap_count-1'b1;
   endcase
  end
 end
 uart_tx #(.CLK_FREQ(CLK_FREQ),.BAUD(BAUD)) uart(
 .I_clk(I_clk),.I_rst_n(I_rst_n),.I_start(tx_start),.I_data(tx_data),.O_busy(tx_busy),.O_tx(O_tx));
endmodule
