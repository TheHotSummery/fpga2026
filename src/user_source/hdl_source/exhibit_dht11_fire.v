// DHT11, open-drain DATA. A bad/partial transaction never publishes a sample.
module exhibit_dht11_reader #(
 parameter integer CLK_HZ=50_000_000,
 parameter integer BOOT_US=2_000_000,
 parameter integer POLL_US=2_500_000,
 parameter integer START_US=20_000
)(input wire clk,rst,inout wire data,
 output reg sample_valid,sample_error,output reg [7:0] temperature_c);
localparam integer US_DIV=(CLK_HZ/1_000_000<1)?1:CLK_HZ/1_000_000;
localparam integer DIV_BITS=(US_DIV<2)?1:$clog2(US_DIV);
reg [DIV_BITS-1:0] divider;
wire us_tick=(divider==US_DIV-1);
always @(posedge clk or posedge rst)begin
 if(rst)divider<=0;else if(us_tick)divider<=0;else divider<=divider+1'b1;
end
reg [2:0] pin_sync;
always @(posedge clk or posedge rst)begin
 if(rst)pin_sync<=3'b111;else pin_sync<={pin_sync[1:0],data};
end
wire pin_level=pin_sync[2];
localparam [3:0] BOOT=0,START=1,WAIT_RESPONSE=2,RESPONSE_LOW=3,
 RESPONSE_HIGH=4,BIT_LOW=5,BIT_HIGH=6,CHECK=7,REST=8,RELEASE_WAIT=9;
reg [3:0] state;
reg [21:0] timer_us;
reg [5:0] bit_count;
reg [39:0] payload;
wire drive_low=state==START;
assign data=drive_low?1'b0:1'bz;
wire [7:0] checksum=payload[39:32]+payload[31:24]+payload[23:16]+payload[15:8];
always @(posedge clk or posedge rst)begin
 if(rst)begin state<=BOOT;timer_us<=0;bit_count<=0;payload<=0;sample_valid<=0;sample_error<=0;temperature_c<=0;end
 else begin
  sample_valid<=0;sample_error<=0;
  if(us_tick)begin
   if(timer_us!=22'h3fffff)timer_us<=timer_us+1'b1;
   case(state)
    BOOT:if(timer_us>=BOOT_US-1)begin state<=START;timer_us<=0;end
    REST:if(timer_us>=POLL_US-1)begin state<=START;timer_us<=0;end
    START:if(timer_us>=START_US-1)begin state<=RELEASE_WAIT;timer_us<=0;payload<=0;bit_count<=0;end
    RELEASE_WAIT:if(timer_us>=19)begin state<=WAIT_RESPONSE;timer_us<=0;end
    WAIT_RESPONSE:if(!pin_level)begin state<=RESPONSE_LOW;timer_us<=0;end
      else if(timer_us>=150)begin state<=REST;timer_us<=0;sample_error<=1;end
    RESPONSE_LOW:if(pin_level)begin state<=RESPONSE_HIGH;timer_us<=0;end
      else if(timer_us>=150)begin state<=REST;timer_us<=0;sample_error<=1;end
    RESPONSE_HIGH:if(!pin_level)begin state<=BIT_LOW;timer_us<=0;end
      else if(timer_us>=150)begin state<=REST;timer_us<=0;sample_error<=1;end
    BIT_LOW:if(pin_level)begin state<=BIT_HIGH;timer_us<=0;end
      else if(timer_us>=120)begin state<=REST;timer_us<=0;sample_error<=1;end
    BIT_HIGH:if(!pin_level)begin
       payload<={payload[38:0],timer_us>=48};timer_us<=0;
       if(bit_count==39)state<=CHECK;else begin bit_count<=bit_count+1'b1;state<=BIT_LOW;end
      end else if(timer_us>=120)begin state<=REST;timer_us<=0;sample_error<=1;end
    CHECK:begin
      if(checksum==payload[7:0] && payload[39:32]<=100 && payload[23:16]<=80 && payload[15:8]<=9)begin
       temperature_c<=payload[23:16];sample_valid<=1;
      end else sample_error<=1;
      timer_us<=0;state<=REST;
    end
    default:begin state<=BOOT;timer_us<=0;end
   endcase
  end
 end
end
endmodule

// Demonstration threshold: freeze a stable ambient baseline, confirm two rises,
// and re-arm only after two cool readings. Manual page return cannot retrigger it.
module exhibit_temperature_alarm #(
 parameter integer RISE_C=2,BASELINE_SAMPLES=3,CONFIRM_SAMPLES=2,COOL_SAMPLES=2
)(input wire clk,rst,sample_valid,sample_error,input wire [7:0] temperature_c,
 output reg alarm_level,output reg baseline_ready,output reg [7:0] baseline_c);
reg [3:0] calibration_count,hot_count,cool_count;
wire hot={1'b0,temperature_c}>={1'b0,baseline_c}+RISE_C;
wire cool={1'b0,temperature_c}<={1'b0,baseline_c}+((RISE_C>0)?RISE_C-1:0);
always @(posedge clk or posedge rst)begin
 if(rst)begin alarm_level<=0;baseline_ready<=0;baseline_c<=0;calibration_count<=0;hot_count<=0;cool_count<=0;end
 else if(sample_error)begin hot_count<=0;cool_count<=0;end
 else if(sample_valid)begin
  if(!baseline_ready)begin
   if(calibration_count==0 || {1'b0,temperature_c}>{1'b0,baseline_c}+1 || {1'b0,temperature_c}+1<{1'b0,baseline_c})begin
    baseline_c<=temperature_c;calibration_count<=1;
   end else if(calibration_count>=BASELINE_SAMPLES-1)begin baseline_ready<=1;hot_count<=0;end
   else calibration_count<=calibration_count+1'b1;
  end else if(!alarm_level)begin
   cool_count<=0;
   if(hot)begin
    if(hot_count>=CONFIRM_SAMPLES-1)begin alarm_level<=1;hot_count<=0;end
    else hot_count<=hot_count+1'b1;
   end else hot_count<=0;
  end else begin
   hot_count<=0;
   if(cool)begin
    if(cool_count>=COOL_SAMPLES-1)begin alarm_level<=0;cool_count<=0;end
    else cool_count<=cool_count+1'b1;
   end else cool_count<=0;
  end
 end
end
endmodule

// Temperature telemetry only: does not change sensor polling or fire decisions.
// Multi-bit payload stays constant until the destination acknowledges receipt.
module exhibit_temperature_display_bridge #(
 parameter integer CLK_HZ=50_000_000,STALE_SECONDS=10
)(input wire sensor_clk,video_clk,rst,sample_valid,
 input wire [7:0] temperature_c,input wire alarm_level,
 output reg display_valid,display_alarm,display_update,
 output reg [3:0] display_tens,display_ones);
localparam integer AGE_BITS=$clog2(CLK_HZ*STALE_SECONDS+1);
reg [AGE_BITS-1:0] age;
reg seen,sample_toggle;
reg [3:0] tens,ones;
wire fresh=seen && age<CLK_HZ*STALE_SECONDS;
wire [10:0] desired={sample_toggle,fresh,alarm_level,tens,ones};
reg [10:0] held;
reg request,acknowledge;
(* ASYNC_REG="TRUE" *) reg [2:0] ack_sync,request_sync;
always @(posedge sensor_clk or posedge rst)begin
 if(rst)begin age<=0;seen<=0;sample_toggle<=0;tens<=0;ones<=0;held<=0;request<=0;ack_sync<=0;end
 else begin
  ack_sync<={ack_sync[1:0],acknowledge};
  if(sample_valid)begin seen<=1;sample_toggle<=~sample_toggle;age<=0;tens<=temperature_c/10;ones<=temperature_c%10;end
  else if(age<CLK_HZ*STALE_SECONDS)age<=age+1'b1;
  if(request==ack_sync[2] && held!=desired)begin held<=desired;request<=~request;end
 end
end
always @(posedge video_clk or posedge rst)begin
 if(rst)begin request_sync<=0;acknowledge<=0;display_valid<=0;display_alarm<=0;display_update<=0;display_tens<=0;display_ones<=0;end
 else begin
  request_sync<={request_sync[1:0],request};
  if(request_sync[2]!=acknowledge)begin
   {display_update,display_valid,display_alarm,display_tens,display_ones}<=held;
   acknowledge<=request_sync[2];
  end
 end
end
endmodule
