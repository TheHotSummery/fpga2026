`timescale 1ns/1ps
module rail_train_manager_tb;
 reg clk=0,rst=1,ir_sensor=1,ctrl_next=0,ctrl_prev=0,ctrl_auto=0;
 always #5 clk=~clk;
 wire [2:0] train_speech_event;
 wire [2:0] train_journey_stage;
 wire [3:0] train_phase;
 integer checks=0,failures=0;
 sd_card_bmp #(.CLK_FREQ_HZ(1000)) d(.clk(clk),.rst(rst),.fire_trigger(1'b0),.ir_sensor(ir_sensor),
 .pip_read_req(1'b0),.pip_read_sector(32'd0),.ctrl_next(ctrl_next),.ctrl_prev(ctrl_prev),.ctrl_auto(ctrl_auto),.ctrl_emerg(1'b0),.ctrl_period(4'd10),
 .key_next(1'b1),.key_enc_sw(1'b1),.key_auto(1'b1),.key_period(1'b1),.emerg_n(1'b1),.vsync(1'b0),.fade_swap_ok(1'b0),
 .bmp_width(16'd640),.bmp_height(16'd480),.write_finish_toggle(1'b0),.wav_spi_hold(1'b0),.write_ready(1'b1),
 .flash_cap_rdy(1'b1),.flash_idle(1'b1),.flash_slot_ok(4'd15),.write_req_ack(1'b0),.SD_MISO(1'b1),
 .train_audio_available(1'b0),.train_audio_done(1'b0),.train_speech_event(train_speech_event),.train_phase(train_phase),.train_journey_stage(train_journey_stage));
 task tick;begin @(posedge clk);#1;@(negedge clk);end endtask
 task cycles(input integer n);integer k;begin for(k=0;k<n;k=k+1)tick;end endtask
 task check(input condition,input [511:0] label);
 begin checks=checks+1;if(condition)$display("PASS %0s",label);
 else begin failures=failures+1;$display("FAIL %0s",label);end end endtask
 task launch(input [2:0] media);
 integer n;begin n=0;while((!d.load_start_pulse || d.load_idx!=media) && n<1000)begin tick;n=n+1;end
 check(d.load_start_pulse && d.load_idx==media,"manager launches expected train media");end endtask
 task commit(input [2:0] media,input [2:0] event_code);
 begin d.disp_commit_pending=1;d.video_continuation=0;tick;
 check(d.img_idx_r==media && train_speech_event==event_code,"manager commit updates visible media and event together");
 check(train_journey_stage==event_code,"manager exports journey stage on visible media commit");end endtask
 initial begin
 cycles(6);rst=0;cycles(20);
 // Inject a completed catalog and quiescent memory buses. Production manager
 // scheduling and train FSM execute; TF/SPI, pixel, SDRAM IP are not modeled.
 force d.sd_init_done=1;force d.bmp_ready=1;force d.scan_done=1;force d.wav_catalog_complete=1;force d.scan_kicked=1;
 force d.img_found_count=8;force d.pip_catalog_valid=1;force d.pip_sector0=500;
 force d.img_sector0=100;force d.img_sector1=200;force d.img_sector2=300;force d.img_sector3=400;
 force d.first_image_committed=1;force d.display_valid=1;
 force d.emerg_active=0;force d.emerg_pending=0;force d.emerg_req_pulse=0;
 force d.sd_frozen=0;force d.fill_active=0;force d.wav_load_busy=0;force d.loading_buf_c=0;
 force d.want_c_bmp=0;force d.want_c_scrub=0;force d.c_post_hold=0;force d.heal_after_emerg=0;
 force d.load_busy=0;force d.fade_out_req=0;force d.vs_rise=1;
 force d.wav_phase2_kicked=1;force d.wav_phase2_busy=0;force d.wav_found_count=0;
 force d.scan_found_valid=0;force d.wav_found_valid=0;force d.wav_load_done=0;force d.wav_scan_done_w=0;
 force d.load_fail=0;force d.video_last=0;force d.g_loading=0;
 d.img_idx_r=0;d.disp_commit_pending=0;d.auto_play_en=1;d.auto_cnt=0;
 ir_sensor=0;launch(4);check(!d.auto_play_en && d.train_saved_auto,"previous autoplay saved and paused");
 check(train_journey_stage==0,"new cycle starts with initial route state");
 check(d.train_flow.photo_count==4,"production manager clamps full catalog to four railway photos");
 // Trigger the actual source timeout threshold, then let the manager retry.
 tick;force d.load_busy=1;d.source_done_seen=0;d.sd_watchdog=1999;tick;
 check(d.next_req_pending && d.load_abort_pulse,"actual load watchdog queues a retry");
 force d.load_busy=0;launch(4);
 check(d.train_busy && !d.next_req_pending,"watchdog retry accepted while train busy");
 // A parser rejection must retry the same target without displaying partial data.
 tick;force d.load_busy=1;force d.load_fail=1;tick;
 check(d.next_req_pending && d.requested_media==4,"parser failure queues original train target");
 check(d.img_idx_r==0 && !d.disp_commit_pending,"parser failure does not commit partial media");
 force d.load_fail=0;force d.load_busy=0;launch(4);
 commit(4,1);cycles(5);force d.video_last=1;
 d.disp_commit_pending=1;d.video_continuation=1;d.video_age=5;tick;
 release d.video_last;force d.video_last=0;
 launch(0);commit(0,2);
 ctrl_next=1;ctrl_prev=1;ctrl_auto=1;tick;ctrl_next=0;ctrl_prev=0;ctrl_auto=0;
 check(!d.auto_play_en && !d.next_req_pending,"manual navigation and autoplay suppressed during train sequence");
 launch(1);commit(1,3);launch(2);commit(2,4);
 cycles(200);check(train_phase==9,"manager remains on departure photo until sensor clear");
 ir_sensor=1;launch(3);commit(3,5);cycles(185);
 check(train_phase==0 && d.auto_play_en,"autoplay restored after complete train sequence");
 check(train_journey_stage==5,"manager retains next-station stage after restoring autoplay");
 $display("RESULT checks=%0d failures=%0d",checks,failures);$finish;
 end
 initial begin #2000000;$fatal(1,"manager test timeout");end
endmodule
