`timescale 1ns/1ps
// Production manager + fire sequencer; catalog/parser and memory completion are
// interface fixtures. Actual BMP decoding and SDRAM PHY are outside this test.
module manager_tb;
reg clk=0,rst=1,fire_trigger=0,ctrl_emerg=0,ctrl_next=0,ctrl_auto=0,vs=0;
reg reader_ready=1,pixel=0,write_finished=0;
always #5 clk=~clk;
sd_card_bmp #(.CLK_FREQ_HZ(1000),.FRAME_PIXELS(8)) d(.clk(clk),.rst(rst),.fire_trigger(fire_trigger),.ir_sensor(1'b1),
 .pip_read_req(1'b0),.pip_read_sector(0),.ctrl_next(ctrl_next),.ctrl_prev(1'b0),.ctrl_auto(ctrl_auto),.ctrl_emerg(ctrl_emerg),.ctrl_period(4'd1),
 .key_next(1'b1),.key_enc_sw(1'b1),.key_auto(1'b1),.key_period(1'b1),.emerg_n(1'b1),.vsync(vs),.fade_swap_ok(1'b1),
 .bmp_width(16'd640),.bmp_height(16'd480),.write_finish_toggle(write_finished),.wav_spi_hold(1'b0),.write_ready(1'b1),
 .flash_cap_rdy(1'b1),.flash_idle(1'b1),.flash_slot_ok(4'd15),.write_req_ack(1'b1),.SD_MISO(1'b1),.train_audio_available(1'b0),.train_audio_done(1'b0));
integer checks=0,failures=0,k,n,launch_count=0,launch_before;
always @(posedge clk)begin #1;if(d.load_start_pulse)launch_count=launch_count+1;end
task tick;begin @(posedge clk);#1;@(negedge clk);end endtask
task cycles(input integer count);integer j;begin for(j=0;j<count;j=j+1)tick;end endtask
task check(input c,input [767:0] why);begin checks=checks+1;if(!c)begin failures=failures+1;$display("FAIL %0s",why);end end endtask
task frame;begin vs=1;cycles(5);vs=0;cycles(5);end endtask
task launch(input [2:0] index,input [31:0] sector);
begin n=0;while(!d.g_loading&&n<500)begin tick;n=n+1;end
 check(d.g_loading && d.g_load_idx==index && d.load_sector==sector,"forced gallery request uses expected photo and sector");
 check(d.write_buf_idx!=d.saved_disp_buf && d.write_buf_idx!=3,"fire poster never writes normal front or audio buffer");
end endtask
task complete;
begin
 reader_ready=0;cycles(4);pixel=1;cycles(8);pixel=0;reader_ready=1;cycles(3);write_finished=~write_finished;
 n=0;while(!d.g_commit&&n<100)begin tick;n=n+1;end
 check(d.g_commit&&!d.g_loading&&!d.load_busy,"exact eight interface pixels plus memory completion prepare a commit");
 frame;
end endtask
initial begin
 cycles(5);rst=0;cycles(5);
 force d.sd_init_done=1;force d.bmp_ready_raw=reader_ready;force d.scan_done=1;force d.wav_catalog_complete=1;force d.scan_kicked=1;
 force d.img_found_count=8;force d.pip_catalog_valid=1;force d.pip_sector0=900;
 force d.img_sector4=500;force d.img_sector5=600;force d.img_sector6=700;force d.img_sector7=800;
 force d.first_image_committed=1;force d.display_valid=1;force d.sd_frozen=0;force d.fill_active=0;
 force d.wav_load_busy=0;force d.loading_buf_c=0;force d.want_c_bmp=0;force d.want_c_scrub=0;force d.heal_after_emerg=0;
 force d.wav_phase2_kicked=1;force d.wav_phase2_busy=0;force d.wav_found_count=0;
 force d.scan_found_valid=0;force d.wav_found_valid=0;force d.wav_load_done=0;force d.wav_scan_done_w=0;
 force d.load_fail=0;force d.bmp_write_req=0;force d.bmp_data_wr_en=pixel;
 d.disp_buf_idx=0;d.img_idx_r=2;d.g_c_valid=1;d.g_c_idx=0;d.g_last_idx=0;d.buf_c_ready=1;d.auto_play_en=0;
 fire_trigger=1;tick;fire_trigger=0;cycles(3);frame;
 check(d.emerg_active && d.saved_img_idx==2,"fire enters emergency and saves railway state");
 check(!d.buf_c_ready,"cached A/B map is hidden until red poster is ready");launch(1,600);complete;
 check(d.g_idx==1&&d.buf_c_ready,"red poster commits before route map");
 for(k=0;k<170;k=k+1)frame;
 check(d.g_idx==1&&!d.g_loading,"red poster stays for the requested displayed duration");
 ctrl_next=1;ctrl_auto=1;tick;ctrl_next=0;ctrl_auto=0;
 check(d.g_idx==1&&!d.g_request,"short navigation cannot override fire sequence");
 for(k=0;k<12;k=k+1)frame;
 launch(0,500);complete;check(d.g_idx==0&&d.buf_c_ready,"route A/B map commits next");
 for(k=0;k<10;k=k+1)frame;check(!d.g_loading&&d.g_idx==0,"fire mode holds route map instead of autoplay/video");
 ctrl_emerg=1;tick;ctrl_emerg=0;frame;cycles(3);
 check(!d.emerg_active && d.img_idx_r==2 && d.disp_buf_idx==0,"manual return restores protected railway frame");
 check(!d.fire_force,"manual return releases forced page selection");
 // Exercise the registered carousel deadline through the actual scheduler.
 d.auto_play_en=1;d.auto_cnt=0;d.auto_tick_q=0;launch_before=launch_count;
 n=0;while(!d.load_start_pulse && n<1005)begin tick;n=n+1;end
 check(n>=1000&&n<=1003,"registered carousel deadline preserves one-second interval");
 check(d.load_start_pulse&&d.load_idx==3,"normal carousel resumes with the correct next photo");
 cycles(20);check(launch_count==launch_before+1,"deadline emits no duplicate load after counter reset");
 $display("MANAGER_RESULT checks=%0d failures=%0d",checks,failures);if(failures)$fatal(1,"fire manager");$finish;
end
initial begin #5000000;$fatal(1,"manager timeout");end
endmodule
