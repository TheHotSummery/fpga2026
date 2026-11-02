`timescale 1ns/1ps
// Targeted manager error/priority tests. Catalog time advances at the scaled
// production clock. Timeout boundary/state injection avoids a 64-second WAV
// simulation. Card/parser/SDRAM interfaces are fixtures, not physical models.
module recovery_edges_tb;
reg clk=0,rst=1;always #5 clk=~clk;
sd_card_bmp #(.CLK_FREQ_HZ(1000),.FRAME_PIXELS(8)) d(
 .clk(clk),.rst(rst),.fire_trigger(1'b0),.ir_sensor(1'b1),
 .pip_read_req(1'b0),.pip_read_sector(0),.ctrl_next(1'b0),.ctrl_prev(1'b0),.ctrl_auto(1'b0),.ctrl_emerg(1'b0),.ctrl_period(4'd10),
 .key_next(1'b1),.key_enc_sw(1'b1),.key_auto(1'b1),.key_period(1'b1),.emerg_n(1'b1),.vsync(1'b0),.fade_swap_ok(1'b0),
 .bmp_width(16'd640),.bmp_height(16'd480),.write_finish_toggle(1'b0),.wav_spi_hold(1'b0),.write_ready(1'b1),
 .flash_cap_rdy(1'b1),.flash_idle(1'b1),.flash_slot_ok(4'd0),.write_req_ack(1'b0),.SD_MISO(1'b1),
 .train_audio_available(1'b0),.train_audio_done(1'b0));
integer checks=0,failures=0,k,resets=0;reg last_reset=0,kill_before;
always @(posedge clk)begin
 if(rst)begin resets=0;last_reset=0;end
 else begin if(d.sd_bus_rst&&!last_reset)resets=resets+1;last_reset=d.sd_bus_rst;end
end
task tick;begin @(posedge clk);#1;@(negedge clk);end endtask
task clocks(input integer n);integer j;begin for(j=0;j<n;j=j+1)tick;end endtask
task check(input condition,input [799:0] why);begin checks=checks+1;if(!condition)begin failures=failures+1;$display("FAIL %0s",why);end else $display("PASS %0s",why);end endtask
task clean;begin
 rst=1;clocks(4);rst=0;clocks(4);
 d.wav_load_busy=0;d.wav_ready=0;d.wav_ph=0;d.wav_await_wrfin=0;d.wav_drain=0;
 d.wav_watch=0;d.wav_saw_wrfin=0;d.load_busy=0;d.loading_buf_c=0;d.buf_c_ready=0;
 d.load_source_started=0;d.source_done_seen=0;d.g_loading=0;d.g_commit=0;
 d.write_buf_idx=0;d.next_req_pending=0;d.disp_commit_pending=0;d.auto_play_en=0;
end endtask
initial begin
 force d.sd_init_done=1;force d.bmp_ready_raw=0;force d.state_code=6;
 force d.scan_done=1;force d.scan_kicked=1;force d.scan_found_valid=0;force d.wav_found_valid=0;
 force d.first_image_committed=1;force d.display_valid=1;force d.img_found_count=8;
 force d.emerg_active=0;force d.emerg_pending=0;force d.emerg_req_pulse=0;
 force d.sd_frozen=0;force d.fill_active=0;force d.want_c_bmp=0;force d.want_c_scrub=0;
 force d.heal_after_emerg=0;force d.fade_out_req=0;force d.c_post_hold=0;
 force d.wav_phase2_kicked=1;force d.wav_phase2_busy=0;force d.wav_found_count=0;
 force d.wav_load_done=0;force d.wav_load_ok=0;force d.wav_scan_done_w=0;
 force d.wav_catalog_complete=1;
 force d.load_fail=0;force d.bmp_write_req=0;force d.bmp_data_wr_en=0;
 force d.sd_sec_read_data_valid=0;force d.sd_sec_read_end=0;force d.pip_catalog_valid=0;
 force d.write_finish_pulse=0;
 clean;
 force d.wav_phase2_busy=1;clocks(6200);
 check(resets==1,"stalled second audio directory scan performs one card recovery");
 check(d.display_valid,"audio catalog recovery retains committed front frame");
 force d.wav_phase2_busy=0;clean;
 force d.wav_phase2_busy=1;
 for(k=0;k<7100;k=k+1)begin
  if(k%100==0)force d.sd_sec_read_data_valid=1;else force d.sd_sec_read_data_valid=0;
  tick;
 end
 check(resets==0,"audio catalog with continuing bytes is not reset by total duration");
 force d.wav_phase2_busy=0;force d.sd_sec_read_data_valid=0;clean;
 force d.state_code=5;
 d.wav_load_busy=1;d.wav_ph=1;d.wav_watch=d.WAV_TIMEOUT_CYCLES-1;d.write_buf_idx=3;kill_before=d.wr_kill_tgl;
 tick;
 check(!d.wav_load_busy && d.wav_ph==0,"WAV request timeout retires the session atomically");
 check(!d.wav_wr_req && !d.wav_load_start_pulse,"expired WAV request cannot restart later in the same clock");
 check(d.wr_kill_tgl!=kill_before,"WAV timeout cancels the partial SDRAM writer");
 check(d.load_abort_pulse && d.sd_bus_rst,"WAV timeout cancels parser and resets the stalled card");
 check(d.write_buf_idx==0 && !d.wav_ready,"WAV timeout releases audio write ownership without marking PCM valid");
 clean;force d.state_code=5;
 d.wav_load_busy=1;d.wav_ph=2;d.wav_watch=d.WAV_TIMEOUT_CYCLES-1;d.write_buf_idx=3;kill_before=d.wr_kill_tgl;
 force d.wav_load_done=1;force d.wav_load_ok=1;force d.write_finish_pulse=1;
 tick;
 check(!d.wav_ready && !d.wav_load_busy && d.wav_ph==0,"late success at expired WAV deadline cannot promote a cancelled session");
 check(d.wr_kill_tgl!=kill_before && d.load_abort_pulse,"WAV run timeout emits one shared writer/parser cancellation");
 kill_before=d.wr_kill_tgl;force d.wav_load_done=0;force d.wav_load_ok=0;force d.write_finish_pulse=0;
 force d.load_fail=1;tick;force d.load_fail=0;tick;
 check(d.wr_kill_tgl==kill_before,"delayed parser abort acknowledgement does not cancel twice");
 clean;force d.state_code=1;force d.bmp_ready_raw=1;
 d.load_busy=1;d.loading_buf_c=1;d.write_buf_idx=2;kill_before=d.wr_kill_tgl;
 force d.load_fail=1;tick;force d.load_fail=0;
 check(d.wr_kill_tgl!=kill_before && d.load_abort_pulse,"failed background emergency poster cancels its partial writer");
 check(!d.loading_buf_c && !d.load_busy && d.write_buf_idx==0,"failed emergency poster releases write page ownership");
 check(!d.buf_c_ready && !d.disp_commit_pending && d.display_valid,"partial emergency poster is not promoted and front remains valid");
 kill_before=d.wr_kill_tgl;tick;
 check(d.wr_kill_tgl==kill_before,"emergency-poster error cleanup is a single cancellation");
 force d.bmp_ready_raw=1;force d.state_code=1;force d.wav_phase2_kicked=1;force d.wav_found_count=0;
 clean;force d.wav_phase2_kicked=0;d.next_req_pending=1;d.requested_media=1;tick;
 check(!d.load_start_pulse && !d.load_busy && d.next_req_pending,"queued picture waits while resumed audio discovery takes ownership");
 force d.wav_phase2_kicked=1;force d.wav_found_count=0;clean;
 force d.wav_found_count=1;d.next_req_pending=1;d.requested_media=1;tick;
 check(d.wav_load_busy && !d.load_start_pulse && !d.load_busy,"initial PCM and queued picture cannot start in the same clock");
 force d.wav_found_count=0;clean;d.next_req_pending=1;d.requested_media=1;tick;
 check(d.load_start_pulse && d.load_busy,"confirmed no-audio catalog still allows normal picture switching");
 clean;force d.first_image_committed=0;force d.wav_phase2_kicked=0;tick;
 check(d.load_start_pulse && d.load_busy && d.load_idx==0,"cold first image remains allowed before audio initialization");
 force d.first_image_committed=1;force d.wav_phase2_kicked=1;clean;
 d.wav_load_busy=1;d.wav_ph=2;d.wav_watch=100;d.write_buf_idx=3;
 force d.wav_load_done=1;force d.wav_load_ok=1;force d.write_finish_pulse=1;tick;
 check(d.wav_ready && !d.wav_load_busy && !d.wav_load_start_pulse,"successful WAV done and IDLE handshake cannot launch a duplicate read");
 $display("RECOVERY_EDGES_RESULT checks=%0d failures=%0d",checks,failures);$finish;
end
initial begin #1000000;$fatal(1,"edge test timeout");end
endmodule
