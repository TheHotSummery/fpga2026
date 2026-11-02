`timescale 1ns/1ps
module bridge_v94_tb;
reg sc=0,vc=0,rst=1,valid=0,alarm=0,run_video=1;reg [7:0] temp=0;
always #5 sc=~sc;
always #7 if(run_video)vc=~vc;
wire fresh,hot,update;wire [3:0] tens,ones;
exhibit_temperature_display_bridge #(.CLK_HZ(100),.STALE_SECONDS(2)) dut(
 .sensor_clk(sc),.video_clk(vc),.rst(rst),.sample_valid(valid),
 .temperature_c(temp),.alarm_level(alarm),.display_valid(fresh),.display_alarm(hot),
 .display_update(update),.display_tens(tens),.display_ones(ones));
integer checks=0,failures=0,i,atomic_observations=0;reg [10:0] held_before;
reg heartbeat_before;reg [10:0] visible_before;reg ack_before;
task check(input c,input [767:0] why);begin checks=checks+1;if(c!==1'b1)begin failures=failures+1;$display("FAIL %0s",why);end end endtask
task ticks(input integer n);begin repeat(n)begin @(posedge sc);#1;end end endtask
task sample(input [7:0] t);begin @(negedge sc);temp=t;valid=1;@(negedge sc);valid=0;end endtask
// Every observed display update must be an entire accepted reading. A split
// tens/ones update can create e.g.20/35/49, none of which is in this sequence.
always @(posedge vc)begin
 visible_before={update,fresh,hot,tens,ones};ack_before=dut.acknowledge;
 #1;
 if(!rst && {update,fresh,hot,tens,ones}!==visible_before)begin
  atomic_observations=atomic_observations+1;
  if(fresh && !((tens==2&&ones==5)||(tens==2&&ones==9)||(tens==3&&ones==0)||
    (tens==4&&ones==7)||(tens==3&&ones==9)||(tens==0&&ones==0)||
    (tens==8&&ones==0)||(tens==2&&ones==6)))begin
   failures=failures+1;$display("FAIL torn or never sampled temperature %0d%0d",tens,ones);
  end
  if(dut.acknowledge===ack_before)begin failures=failures+1;$display("FAIL display changed outside atomic transfer");end
 end
end
initial begin
 ticks(4);rst=0;ticks(30);check(!fresh&&!update,"no sample displays unavailable without heartbeat");
 sample(25);ticks(30);check(fresh && tens==2 && ones==5,"first valid temperature reaches video atomically");
 check(update,"first accepted sample toggles heartbeat");
 heartbeat_before=update;sample(25);ticks(30);
 check(fresh&&tens==2&&ones==5&&update!=heartbeat_before,"same25C sample changes heartbeat while digits remain25");
 heartbeat_before=update;sample(25);ticks(30);
 check(fresh&&tens==2&&ones==5&&update!=heartbeat_before,"third same25C sample toggles again");
 heartbeat_before=update;ticks(25);check(update==heartbeat_before,"no sample means no fake heartbeat");
 temp=49;ticks(15);check(update==heartbeat_before&&tens==2&&ones==5,"unvalidated sensor bus changes cannot alter value or heartbeat");
 sample(29);ticks(30);check(fresh && tens==2 && ones==9,"normal update");
 @(negedge vc);run_video=0;sample(30);ticks(5);held_before=dut.held;
 check(held_before[7:0]==8'h30,"first pending mailbox contains complete30C");
 sample(47);ticks(5);check(dut.held==held_before,"source mailbox held while destination stopped");
 sample(39);ticks(5);check(dut.held==held_before,"newer sample cannot corrupt pending payload");
 visible_before={update,fresh,hot,tens,ones};ticks(15);
 check({update,fresh,hot,tens,ones}==visible_before,"paused video cannot receive partial or asynchronous changes");
 run_video=1;ticks(40);check(fresh && tens==3 && ones==9,"latest queued reading arrives after clock resumes");
 check(update==dut.sample_toggle,"heartbeat catches up to latest accepted sample after video resumes");
 heartbeat_before=update;alarm=1;ticks(30);
 check(hot && fresh && tens==3 && ones==9,"alarm color state transfers without changing value");
 check(update==heartbeat_before,"alarm-only transfer cannot invent a new temperature sample");
 ticks(210);check(!fresh,"production10-second timeout represented by accelerated counter");
 check(update==heartbeat_before,"stale timeout preserves last successful sample heartbeat");
 sample(0);ticks(30);check(fresh && tens==0 && ones==0,"real zero reading distinguishable from missing sensor");
 check(update!=heartbeat_before,"sensor replug and valid zero create genuine heartbeat");
 sample(80);ticks(30);check(fresh && tens==8 && ones==0,"highest supported sensor temperature");
 heartbeat_before=update;sample(26);ticks(30);
 check(fresh&&tens==2&&ones==6&&update!=heartbeat_before,"multiple-digit transition and heartbeat arrive together");
 rst=1;alarm=0;ticks(4);rst=0;ticks(20);
 check(!fresh && !hot&&!update,"reset clears cached reading, alarm and heartbeat");
 check(atomic_observations==11,"all11 expected asynchronous mailbox transfers were atomic");
 $display("V94_BRIDGE_RESULT checks=%0d failures=%0d atomic_observations=%0d",checks,failures,atomic_observations);
 if(failures)$fatal(1,"V94 bridge regression");$finish;
end
initial begin #100000;$fatal(1,"timeout");end
endmodule
