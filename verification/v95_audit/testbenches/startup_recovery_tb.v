`timescale 1ns/1ps
// Real production manager and train sequencer. The parser/card and memory
// completion are interface fixtures; no manager state is forced. This tests
// orchestration, not the physical SD protocol, SDRAM PHY, or audio DAC.
module startup_recovery_tb;
reg clk=0,rst=1,ir=1,audio_done=0,vs=0;
always #5 clk=~clk;
integer vclock=0;
always @(negedge clk)begin vclock=vclock+1;vs=(vclock%20)<5;end
wire wav_ready;
reg card_initialized=0,manual_card_loss=0,stall_catalog=1;
reg slow_catalog=0;
integer init_delay=1500,init_elapsed=0;
reg parser_ready=1,cat_done=0,found=0,wav_found=0,parser_pixel=0;
reg parser_req=0,pcm_we=0,wav_done=0,wav_ok=0,sector_byte=0,write_finished=0;
reg [31:0] found_sector=0;
reg [3:0] parser_state=1;
integer mode=0,pos=0,delay=0,finish_delay=0,pixels=0;
sd_card_bmp #(.CLK_FREQ_HZ(1000),.FRAME_PIXELS(8)) d(
 .clk(clk),.rst(rst),.fire_trigger(1'b0),.ir_sensor(ir),
 .pip_read_req(1'b0),.pip_read_sector(0),
 .ctrl_next(1'b0),.ctrl_prev(1'b0),.ctrl_auto(1'b0),.ctrl_emerg(1'b0),.ctrl_period(4'd3),
 .key_next(1'b1),.key_enc_sw(1'b1),.key_auto(1'b1),.key_period(1'b1),.emerg_n(1'b1),
 .vsync(vs),.fade_swap_ok(1'b1),.bmp_width(16'd640),.bmp_height(16'd480),
 .write_finish_toggle(write_finished),.wav_spi_hold(1'b0),.write_ready(1'b1),
 .flash_cap_rdy(1'b1),.flash_idle(1'b1),.flash_slot_ok(4'd0),.write_req_ack(1'b1),
 .SD_MISO(1'b1),.train_audio_available(wav_ready),.train_audio_done(audio_done),.wav_ready(wav_ready));
integer checks=0,failures=0,resets=0,scans=0,rises=0,events2=0,events3=0,n;
integer disarmed_warm_scan_cycles=0;
reg old_reset=0;
always @(posedge clk)if(!rst)begin
 if(d.sd_bus_rst && !old_reset)resets=resets+1;
 old_reset=d.sd_bus_rst;
 if(d.scan_start_pulse)scans=scans+1;
 if(d.sd_init_rise)rises=rises+1;
 if(d.train_speech_event==2)events2=events2+1;
 if(d.train_speech_event==3)events3=events3+1;
 if(scans>=3 && mode==1 && !cat_done && !d.scan_kicked)
  disarmed_warm_scan_cycles=disarmed_warm_scan_cycles+1;
end
// The card loses initialization when reset. Its second initialization is fast;
// its first response deliberately takes 1.5 s, within the new 8 s allowance.
always @(negedge clk)begin
 if(rst || d.sd_bus_rst || manual_card_loss)begin
  card_initialized=0;init_elapsed=0;
  if(d.sd_bus_rst)begin init_delay=20;stall_catalog=0;end
 end else if(!card_initialized)begin
  if(init_elapsed>=init_delay)card_initialized=1;
  else init_elapsed=init_elapsed+1;
 end
end
// Complete eight image pixels and a physical write-finish toggle only after
// the manager starts/acknowledges a session. The fixture never modifies
// load_busy, first_image_committed, display_valid, img_found_count or wav_ready.
always @(negedge clk)begin
 found=0;wav_found=0;wav_done=0;wav_ok=0;sector_byte=0;
 if(rst || !card_initialized)begin
  parser_ready=1;cat_done=0;parser_state=1;parser_req=0;
  parser_pixel=0;pcm_we=0;mode=0;pos=0;finish_delay=0;
 end else begin
  if(finish_delay>0)begin
   finish_delay=finish_delay-1;
   if(finish_delay==0)write_finished=~write_finished;
  end
  case(mode)
   0:begin
    if(d.scan_start_pulse)begin mode=1;pos=0;cat_done=0;parser_ready=0;parser_state=7;end
    else if(d.load_start_pulse)begin mode=2;delay=4;parser_ready=0;parser_state=2;end
    else if(d.wav_load_start_pulse)begin mode=5;delay=8;parser_ready=0;parser_state=5;pcm_we=1;end
   end
   1:if(!stall_catalog && slow_catalog)begin
    // A long scan that continues returning sector bytes is healthy.
    if(vclock%100==0)sector_byte=1;
   end else if(!stall_catalog)begin
    sector_byte=1;
    if(pos<8)begin found=1;found_sector=(pos+1)*100;pos=pos+1;end
    else if(pos==8)begin wav_found=1;pos=pos+1;end
    else begin cat_done=1;parser_ready=1;parser_state=1;mode=0;end
   end
   2:begin
    delay=delay-1;
    if(delay==0)begin parser_req=1;parser_state=3;mode=3;end
   end
   3:if(d.write_req && !d.wr_kill_tgl)begin
    parser_req=0;mode=4;pixels=0;parser_pixel=1;parser_state=4;
   end else if(d.write_req)begin
    parser_req=0;mode=4;pixels=0;parser_pixel=1;parser_state=4;
   end
   4:begin
    pixels=pixels+1;
    if(pixels==8)begin parser_pixel=0;parser_ready=1;parser_state=1;mode=0;finish_delay=4;end
   end
   5:begin
    delay=delay-1;
    if(delay==0)begin pcm_we=0;wav_done=1;wav_ok=1;parser_ready=1;parser_state=1;mode=0;finish_delay=4;end
   end
  endcase
 end
end
task check(input condition,input [799:0] why);begin
 checks=checks+1;if(!condition)begin failures=failures+1;$display("FAIL %0s",why);end
end endtask
task clocks(input integer count);integer k;begin for(k=0;k<count;k=k+1)begin @(posedge clk);#1;end end endtask
initial begin
 force d.sd_init_done=card_initialized;
 force d.bmp_ready_raw=parser_ready;
 force d.state_code=parser_state;
 force d.scan_done=cat_done;
 force d.scan_found_valid=found;
 force d.scan_found_sector=found_sector;
 force d.wav_found_valid=wav_found;
 force d.wav_found_sector=32'd900;
 force d.wav_catalog_complete=1;
 force d.wav_scan_done_w=0;
 force d.wav_load_done=wav_done;
 force d.wav_load_ok=wav_ok;
 force d.wav_pcm_we_w=pcm_we;
 force d.rail_audio_present=1;
 force d.pip_catalog_valid=0;
 force d.load_fail=0;
 force d.bmp_write_req=parser_req;
 force d.bmp_data_wr_en=parser_pixel;
 force d.sd_sec_read_data_valid=sector_byte;
 force d.sd_sec_read_end=0;
 clocks(5);@(negedge clk);rst=0;
 clocks(1400);check(resets==0,"slow initialization is not reset after one second");
 clocks(300);check(card_initialized && scans==1,"slow card initializes and initial catalog starts");
 check(!d.display_valid&&!wav_ready,"stalled initial catalog has not fabricated media readiness");
 n=0;while(!(d.display_valid && wav_ready) && n<9000)begin clocks(1);n=n+1;end
 check(d.display_valid && d.first_image_committed,"catalog timeout recovers and first image actually commits");
 check(wav_ready && d.wav_found_count==1,"normal PCM load resumes after the first image");
 check(resets==1 && scans==2,"one catalog failure performs one reinit and one rescan");
 check(rises==2,"initial and recovery SD initialized edges are real driven pulses");
 check(d.img_found_count==8 && d.img_idx_r==0,"recovered catalog is complete without duplicate counts");
 ir=0;n=0;while(events2==0 && n<400)begin clocks(1);n=n+1;end
 check(events2==1 && d.train_phase==4,"IR flow emits arrival speech after its photo commit");
 clocks(4000);check(d.train_phase==4,"existing audio-driven stage waits for real audio completion");
 @(negedge clk);audio_done=1;clocks(1);@(negedge clk);audio_done=0;
 n=0;while(events3==0 && n<400)begin clocks(1);n=n+1;end
 check(events3==1 && d.img_idx_r==1,"audio completion advances to the next photo and speech");
 // Warm loss must retain the committed front page while reinitialization and
 // a fresh catalog occur. This is the formerly undriven sd_init_rise path.
 @(negedge clk);slow_catalog=1;manual_card_loss=1;clocks(4);@(negedge clk);manual_card_loss=0;
 clocks(7100);
 check(resets==1 && scans==3 && !cat_done,"continuous scan progress beyond six seconds is never reset");
 check(disarmed_warm_scan_cycles==0 && d.scan_kicked,"late initialized-edge synchronization cannot disarm an already running warm catalog");
 // Now stop the formerly healthy warm scan completely. The watchdog must
 // still be armed after the synchronized initialization edge has passed.
 @(negedge clk);slow_catalog=0;stall_catalog=1;
 n=0;while(!(resets>=2 && scans>=4 && cat_done && d.img_found_count==8 && !d.sd_frozen) && n<6500)begin clocks(1);n=n+1;end
 check(resets==2 && scans==4,"a stalled immediate-start warm catalog is recovered by the still-armed watchdog");
 check(scans==4 && rises==4 && d.img_found_count==8 && !d.sd_frozen,"warm card return resynchronizes catalog and leaves freeze");
 check(d.display_valid && d.img_idx_r==1 && wav_ready,"warm recovery retains front image and already loaded PCM");
 check(d.wav_found_count==1,"fresh catalog does not accumulate duplicate WAV entries after retry");
 $display("STARTUP_RECOVERY_RESULT checks=%0d failures=%0d resets=%0d scans=%0d initialized_edges=%0d",checks,failures,resets,scans,rises);
 if(failures)$fatal(1,"startup recovery regression");$finish;
end
initial begin #400000;$fatal(1,"startup recovery timeout");end
endmodule
