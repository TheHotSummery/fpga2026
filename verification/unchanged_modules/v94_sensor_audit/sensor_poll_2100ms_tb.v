`timescale 1ns/1ps
// Production microsecond divider and synchronized DATA at the actual 50 MHz.
// BOOT only is accelerated. The first2.1s REST runs in full at50MHz.
// Later REST middles are fast-forwarded, while all deadline cycles and all wire
// transactions remain actual50MHz. Poll parameter is production proposal2.1s.
module sensor_poll_2100ms_tb;
reg clk=0,rst=1,sensor_low=0;
always #10 clk=~clk;
tri1 data;assign data=sensor_low?1'b0:1'bz;
wire valid,error,alarm,ready;wire [7:0] temp,baseline;
exhibit_dht11_reader #(.CLK_HZ(50_000_000),.BOOT_US(50),.POLL_US(2_100_000)) dut(
 .clk(clk),.rst(rst),.data(data),.sample_valid(valid),.sample_error(error),.temperature_c(temp));
exhibit_temperature_alarm a(.clk(clk),.rst(rst),.sample_valid(valid),.sample_error(error),
 .temperature_c(temp),.alarm_level(alarm),.baseline_ready(ready),.baseline_c(baseline));
integer checks=0,failures=0,valids=0,errors=0;time low_start,previous_release,full_interval;reg expect_alarm=0,fast_idle=0;
always @(posedge clk)begin #1;if(valid)valids=valids+1;if(error)errors=errors+1;
 if(alarm && !expect_alarm)begin $display("FAIL spurious early fire");$fatal(1);end
end
task check(input c,input [767:0] why);begin checks=checks+1;if(c!==1'b1)begin failures=failures+1;$display("FAIL %0s",why);end end endtask
task send(input [7:0] t,input integer delay_ns,input integer zero_ns,input integer one_ns,input integer bad_crc);
integer b;reg [39:0] bits;reg [7:0] crc;begin
 crc=8'd55+t+bad_crc;bits={8'd55,8'd0,t,8'd0,crc};
 if(fast_idle)begin
  wait(dut.state==8);@(negedge clk);dut.timer_us=2_100_000-10;
  // Fast-forward only the middle of later REST states. The deadline and
  // the real divider at the deadline remain production values.
  repeat(8)begin @(posedge dut.us_tick);#1;check(!dut.drive_low,"REST deadline not early after idle fast-forward");end
 end
 wait(dut.drive_low);low_start=$time;wait(!dut.drive_low);
 check($time-low_start==20_000_000,"actual 50MHz divider creates 20ms host low");
 #(delay_ns);sensor_low=1;#80000;sensor_low=0;#80000;
 for(b=39;b>=0;b=b-1)begin sensor_low=1;#50000;sensor_low=0;if(bits[b])#(one_ns);else #(zero_ns);end
 sensor_low=1;#50000;sensor_low=0;#10000;
end endtask
initial begin
 #125;rst=0;
 // No module response cannot create a fake sample, fire, or baseline.
 wait(dut.drive_low);wait(!dut.drive_low);wait(errors==1);#100;previous_release=$time;
 check(valids==0&&!alarm&&!ready,"absent sensor cannot fire on boot");
 send(25,20000,26000,68000,0);check(valids==1&&temp==25&&!ready,"20us response, 26/68us bits decode at 50MHz");
 full_interval=$time-previous_release;
 check(full_interval>=2_123_000_000 && full_interval<=2_125_000_000,"full unaccelerated REST2100ms plus real start/frame interval is2.124s");
 $display("FULL_INTERVAL_NS=%0d",full_interval);fast_idle=1;
 send(25,40000,30000,75000,0);check(valids==2&&temp==25&&!ready,"40us response, 30/75us bits decode at 50MHz");
 send(25,30000,28000,70000,0);check(valids==3&&ready&&baseline==25&&!alarm,"three physical frames establish baseline");
 send(27,30000,28000,70000,1);check(valids==3&&errors==2&&!alarm,"bad CRC high sample cannot trigger");
 send(27,30000,28000,70000,0);check(valids==4&&!alarm,"one real hot sample cannot trigger");
 expect_alarm=1;send(27,30000,28000,70000,0);check(valids==5&&alarm,"two real hot samples trigger at 50MHz");
 $display("SENSOR_POLL2100_RESULT checks=%0d failures=%0d valid_frames=%0d errors=%0d",checks,failures,valids,errors);
 if(failures)$fatal(1);$finish;
end
initial begin #3000000000;$fatal(1,"timeout");end
endmodule
