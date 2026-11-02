`timescale 1ns/1ps
module rail_train_flow_tb;
 reg clk=0; always #5 clk=~clk;
 reg rst=1,ir_sensor=1,system_ready=1,emergency=0,video_available=1,frame_tick=1;
 reg video_done=0,media_commit=0;
 wire audio_available=0,audio_done=0;
 reg [2:0] committed_media=0;
 reg [3:0] found_count=8;
 wire [2:0] photo_count=found_count>=4 ? 3'd4 : found_count[2:0];
 wire start_pulse,request_pulse,busy,finish_pulse;
 wire [2:0] request_media,speech_event;
 wire [2:0] journey_stage;
 wire [3:0] phase;
 integer failures=0,checks=0;
 rail_train_flow #(.CLK_FREQ_HZ(100),.ANNOUNCE_FRAMES(12)) dut(.*);
 task tick; begin @(posedge clk); #1; end endtask
 task cycles(input integer n); integer k; begin for(k=0;k<n;k=k+1) tick; end endtask
 task check(input condition,input [511:0] label);
 begin checks=checks+1; if(condition) $display("PASS %0s",label);
 else begin failures=failures+1;$display("FAIL %0s",label);end end endtask
 task reset_flow;
 begin rst=1;ir_sensor=1;emergency=0;video_done=0;media_commit=0;video_available=1;
 found_count=8;frame_tick=1;system_ready=1;cycles(3);rst=0;cycles(8);end endtask
 task wait_request(input [2:0] expected);
 integer n; begin n=0;while(!request_pulse && n<1000)begin tick;n=n+1;end
 check(request_pulse && request_media==expected,"requested expected media");end endtask
 task commit(input [2:0] media,input [2:0] event_code);
 begin cycles(2);committed_media=media;media_commit=1;tick;media_commit=0;
 check(speech_event==event_code,"speech event starts only after display commit");
 check(journey_stage==event_code,"journey stage changes atomically with media commit");end endtask
 task start_train;
 begin ir_sensor=0;wait_request(4);commit(4,1);end endtask
 task finish_train;
 integer n;begin n=0;while(!finish_pulse && n<1000)begin tick;n=n+1;end
 check(finish_pulse && !busy,"flow completes once");
 check(journey_stage==5,"next-station journey stage persists after flow finishes");end endtask
 task interrupt_wait(input [2:0] expected,input [3:0] expected_phase);
 reg [2:0] before_stage;
 begin before_stage=journey_stage;emergency=1;cycles(5);check(phase==expected_phase,"emergency preserves waiting stage");
 check(journey_stage==before_stage,"emergency preserves committed journey stage");
 emergency=0;wait_request(expected);end endtask
 initial begin
 reset_flow;
 check(journey_stage==0,"reset starts at normal railway stage");
 ir_sensor=0;tick;ir_sensor=1;cycles(10);
 check(!busy,"one-cycle infrared glitch ignored");
 system_ready=0;ir_sensor=0;cycles(10);check(!busy,"unready system ignores obstruction");
 system_ready=1;wait_request(4);commit(4,1);
 cycles(8);check(phase==2,"video waits for last frame commit");
 video_done=1;tick;video_done=0;wait_request(0);commit(0,2);
 frame_tick=0;cycles(30);check(phase==4,"announcement hold advances only on frame tick");frame_tick=1;
 wait_request(1);commit(1,3);wait_request(2);commit(2,4);
 cycles(20);check(phase==9 && !request_pulse,"departure waits for infrared clear");
 ir_sensor=1;wait_request(3);commit(3,5);finish_train;
 cycles(20);check(!busy,"clear sensor does not repeat flow");
 check(journey_stage==5,"idle retains final route and subtitle stage");
 ir_sensor=0;wait_request(4);check(start_pulse,"new obstruction rearms flow");
 check(journey_stage==0,"new train cycle resets initial route before video commit");
 check(found_count==8 && photo_count==4,"full eight-BMP catalog clamps railway count");

 // Emergency aborts normal media loading. Each interrupted wait must retry.
 reset_flow;ir_sensor=0;wait_request(4);interrupt_wait(4,1);commit(4,1);
 emergency=1;cycles(10);check(phase==2,"emergency pauses video phase");emergency=0;
 cycles(2);check(phase==2 && !request_pulse,"playing video resumes without restart request");
 video_done=1;tick;video_done=0;wait_request(0);interrupt_wait(0,3);commit(0,2);
 wait_request(1);interrupt_wait(1,5);commit(1,3);
 wait_request(2);interrupt_wait(2,7);commit(2,4);
 ir_sensor=1;wait_request(3);interrupt_wait(3,10);commit(3,5);finish_train;

 // Early sensor clear is consumed only after the preceding phases finish.
 reset_flow;start_train;ir_sensor=1;cycles(10);
 video_done=1;tick;video_done=0;wait_request(0);commit(0,2);
 wait_request(1);commit(1,3);wait_request(2);commit(2,4);
 wait_request(3);commit(3,5);finish_train;

 // Missing video falls back to pictures without waiting for a last frame.
 reset_flow;video_available=0;found_count=1;ir_sensor=0;
 wait_request(0);commit(0,2);wait_request(0);commit(0,3);
 wait_request(0);commit(0,4);ir_sensor=1;wait_request(0);commit(0,5);finish_train;
 $display("RESULT checks=%0d failures=%0d",checks,failures);$finish;
 end
endmodule
