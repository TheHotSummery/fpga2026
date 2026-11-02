`timescale 1ns/1ps
// Real production manager and train sequencer. The parser/card and memory
// completion are interface fixtures; no manager state is forced. This tests
// orchestration, not the physical SD protocol, SDRAM PHY, or audio DAC.
module audio_recovery_integration_tb;
reg clk=0,rst=1,ir=1,audio_done=0,vs=0;
always #5 clk=~clk;
integer vclock=0;
always @(negedge clk)begin vclock=vclock+1;vs=(vclock%20)<5;end
wire wav_ready;
reg card_initialized=0,manual_card_loss=0,stall_catalog=0;
reg slow_catalog=0;
reg stall_phase2=1,stall_wav=1,wav_catalog_complete_fixture=0,wav_scan_done_fixture=0;
integer phase2_starts=0,wav_attempts=0,invalid_front_cycles=0;reg monitor_front=0;
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
  if(d.sd_bus_rst)begin
   init_delay=20;stall_catalog=0;
   if(phase2_starts>0)begin stall_phase2=0;end
   if(wav_attempts>0)stall_wav=0;
  end
 end else if(!card_initialized)begin
  if(init_elapsed>=init_delay)card_initialized=1;
  else init_elapsed=init_elapsed+1;
 end
end
// Complete eight image pixels and a physical write-finish toggle only after
// the manager starts/acknowledges a session. The fixture never modifies
// load_busy, first_image_committed, display_valid, img_found_count or wav_ready.
always @(negedge clk)begin
 found=0;wav_found=0;wav_done=0;wav_ok=0;sector_byte=0;wav_scan_done_fixture=0;
 if(rst || !card_initialized)begin
  wav_catalog_complete_fixture=0;
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
    else if(d.wav_scan_start_pulse)begin mode=6;phase2_starts=phase2_starts+1;parser_ready=0;parser_state=6;end
    else if(d.wav_load_start_pulse)begin mode=5;wav_attempts=wav_attempts+1;delay=8;parser_ready=0;parser_state=5;pcm_we=1;end
   end
   1:if(!stall_catalog && slow_catalog)begin
    // A long scan that continues returning sector bytes is healthy.
    if(vclock%100==0)sector_byte=1;
   end else if(!stall_catalog)begin
    sector_byte=1;
    if(pos<8)begin found=1;found_sector=(pos+1)*100;pos=pos+1;end
    else if(pos==8)begin pos=pos+1;end
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
   5:if(stall_wav)begin pcm_we=0;end else begin
    delay=delay-1;
    if(delay==0)begin pcm_we=0;wav_done=1;wav_ok=1;parser_ready=1;parser_state=1;mode=0;finish_delay=4;end
   end
   6:if(!stall_phase2)begin
    wav_found=1;wav_scan_done_fixture=1;wav_catalog_complete_fixture=1;parser_ready=1;parser_state=1;mode=0;
   end
  endcase
 end
end
always @(posedge clk)if(monitor_front && !d.display_valid)invalid_front_cycles=invalid_front_cycles+1;
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
 force d.wav_catalog_complete=wav_catalog_complete_fixture;
 force d.wav_scan_done_w=wav_scan_done_fixture;
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
 n=0;while(!d.display_valid && n<2500)begin clocks(1);n=n+1;end
 check(d.display_valid && !wav_ready,"first frame displays before the stalled second audio scan");monitor_front=1;
 n=0;while(resets<1 && n<6500)begin clocks(1);n=n+1;end
 check(resets==1 && phase2_starts==1 && d.display_valid,"second audio scan recovers without blanking committed image");
 n=0;while(wav_attempts<1 && n<1000)begin clocks(1);n=n+1;end
 check(wav_attempts==1 && !wav_ready,"rescanned catalog starts first PCM load");
 n=0;while(resets<2 && n<40500)begin clocks(1);n=n+1;end
 check(resets==2 && !wav_ready,"stalled PCM session reaches its 40-second configured recovery");
 n=0;while(!wav_ready && n<2000)begin clocks(1);n=n+1;end
 $display("RECOVERY_EVIDENCE ready=%0d attempts=%0d resets=%0d scans=%0d mode=%0d ph=%0d start=%0d",wav_ready,wav_attempts,resets,scans,mode,d.wav_ph,d.wav_load_start_pulse);
 check(wav_ready && wav_attempts==2,"fresh PCM write completes after card and writer cancellation");
 check(scans==3 && d.img_found_count==8 && d.wav_found_count==1,"repeated recovery retains a clean image and audio catalog");
 check(invalid_front_cycles==0 && !d.sd_frozen,"committed front image remains valid throughout both audio recoveries");
 ir=0;n=0;while(events2==0 && n<500)begin clocks(1);n=n+1;end
 check(events2==1 && d.train_phase==4,"recovered media and PCM resume the original IR arrival announcement");
 $display("AUDIO_RECOVERY_INTEGRATION_RESULT checks=%0d failures=%0d",checks,failures);
 if(failures)$fatal(1,"audio recovery integration failed");$finish;
end
initial begin #1500000;$fatal(1,"startup recovery timeout");end
endmodule
