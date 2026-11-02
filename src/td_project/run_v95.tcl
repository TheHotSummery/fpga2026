import_device eagle_s20.db -package EG4S20BG256
if {![file exists ../user_source/hdl_source/pip_scale_coefficients.hex]} {error "Missing source-local scale ROM"}
read_verilog -file {../user_source/hdl_source/exhibit_dht11_fire.v ../user_source/hdl_source/exhibit_fire_sequence.v ../user_source/hdl_source/rail_announcement_player.v ../user_source/hdl_source/I2S_receiver.v ../user_source/hdl_source/audio_arc_calculate.v ../user_source/hdl_source/hdmi_phy_warpper.v ../user_source/hdl_source/lane_lvds_10_1.v ../user_source/hdl_source/hdmi1.4b_transmitter_core/hdmi_1_4b_transmitter_core_wrapper.enc.v ../user_source/hdl_source/top_tf_hdmi_audio.v ../user_source/hdl_source/exhibit_sd_startup_guard.v ../user_source/hdl_source/exhibit_media_bilinear.v ../user_source/hdl_source/exhibit_q565_decoder.v ../user_source/hdl_source/SD/bmp_read.v ../user_source/hdl_source/SD/color_bar.v ../user_source/hdl_source/SD/frame_fifo_read.v ../user_source/hdl_source/SD/frame_fifo_write.v ../user_source/hdl_source/SD/frame_read_write.v ../user_source/hdl_source/SD/sd_card_bmp.v ../user_source/hdl_source/SD/seg_decoder.v ../user_source/hdl_source/SD/seg_scan.v ../user_source/hdl_source/SD/video_delay.v ../user_source/hdl_source/SD/video_timing_data.v ../user_source/hdl_source/hdmi_audio_tone_i2s_64fs.v ../user_source/hdl_source/startup_pulse.v ../user_source/hdl_source/video_rgb_to_axis_640x480.v ../user_source/hdl_source/SD/sd_card_cmd.v ../user_source/hdl_source/SD/sd_card_sec_read_write.v ../user_source/hdl_source/SD/sd_card_top.v ../user_source/hdl_source/SD/spi_master.v ../user_source/hdl_source/SD/sdram.v ../user_source/hdl_source/include/sdr_as_ram.enc.v ../user_source/hdl_source/include/sdr_init_ref.enc.v ../user_source/hdl_source/include/sdr_wrrd.enc.v ./al_ip/PLL_HDMI_AUDIO.v ../user_source/hdl_source/ip/afifo_16_32_256.v ../user_source/hdl_source/ip/afifo_32_16_256.v ../user_source/hdl_source/ip/sys_pll.v ../user_source/hdl_source/ip/video_pll.v} -top top
read_adc ../user_source/constraints_source/pin.adc
read_sdc ../user_source/constraints_source/timing.sdc
read_sdc -ip rfifo_32_32_512 ../user_source/hdl_source/ip/afifo_32_16_256.tcl
read_sdc -ip wfifo_32_32_512 ../user_source/hdl_source/ip/afifo_16_32_256.tcl
set_param gate opt_area high
set_param gate pack_effort high
optimize_rtl
optimize_gate
report_area -file v95_synth_area.rpt
puts "FULL_WAVE_SYNTHESIS_COMPLETE"



legalize_phy_inst
read_sdc ../user_source/constraints_source/scaler_phase.sdc
export_db v95_gate.db
set_param route fix_hold on
set_param route effort routability
set_param route timing on
set_param place effort high
set_param flow enable_seed on
set_param place seed 77
place
report_timing_summary -file v95_placed_timing_summary.rpt
export_db v95_place.db
set_param route effort routability
commit_param -step route
route
fix_hold
update_timing -mode final
# Extra physical repair is needed only when final routed setup/hold fails.
set prev_cs -999.0
for {set closure_try 0} {$closure_try < 2} {incr closure_try} {
    report_timing_summary -file v95_closure_check.rpt
    set cf [open v95_closure_check.rpt r]
    set closure_text [read $cf]
    close $cf
    if {![regexp {SWNS:\s*([-0-9.]+)ns} $closure_text -> cs] || ![regexp {HWNS:\s*([-0-9.]+)ns} $closure_text -> ch]} {error "Missing physical timing report"}
    if {$cs >= 0 && $ch >= 0} {break}
    if {$cs == $prev_cs} {break}
    set prev_cs $cs
    puts "V95_EXTRA_TIMING_REPAIR $closure_try setup=$cs hold=$ch"
    multi_try_route
    fix_hold
    update_timing -mode final
}
# Target current negative paths, so rebuilt mapped instance names need not
# match a previous release. Keep timing constraints and the final gate strict.
set prev_as -999.0
for {set repair_round 0} {$repair_round < 2} {incr repair_round} {
 report_timing_summary -file v95_targeted_check.rpt
 set af [open v95_targeted_check.rpt r];set at [read $af];close $af
 if {![regexp {SWNS:\s*([-0-9.]+)ns} $at -> as]} {error "Missing targeted timing status"}
 if {$as >= 0 || $as == $prev_as} {break}
 set prev_as $as
 set negative 0;set sinks {};set data_sinks {}
 foreach line [split $at "\n"] {
  if {[regexp {^Slack\s*:\s*([-0-9.]+)ns} $line -> slack]} {set negative [expr {$slack < 0}]}
  if {$negative && [regexp {^End Point\s*:\s*(\S+)} $line -> pin] && [lsearch -exact $sinks $pin] < 0} {lappend sinks $pin}
  if {$negative && [regexp {pin:\s*(\S+)} $line -> pin] && [regexp {\.(a|b|c|d|addrb)\[[0-9]+\]$|\.ce$} $pin] && [lsearch -exact $data_sinks $pin] < 0} {lappend data_sinks $pin}
 }
 set sinks [concat $sinks $data_sinks]
 if {[llength $sinks] == 0} {break}
 foreach sink [lrange $sinks 0 31] {
  puts "V95_TARGETED_REPAIR $repair_round $sink"
  multi_try_route -sink $sink
 }
 fix_hold
 update_timing -mode final
}
report_timing_summary -file v95_timing_summary.rpt
report_area -file v95_routed_area.rpt
export_db v95_route.db
puts "V95_ROUTING_COMPLETE"

set tf [open v95_timing_summary.rpt r]
set timing_text [read $tf]
close $tf
if {![regexp {SWNS:\s*([-0-9.]+)ns} $timing_text -> setup_slack]} {error "Cannot read setup timing"}
if {![regexp {HWNS:\s*([-0-9.]+)ns} $timing_text -> hold_slack]} {error "Cannot read hold timing"}
if {$setup_slack < 0 || $hold_slack < 0} {error "Timing failed: setup=$setup_slack hold=$hold_slack. Do not download an old bit file."}
bitgen -bit ../../release_v95/Metro_V95.bit -bin ../../release_v95/Metro_V95.bin -version 95
puts "V95_BITGEN_COMPLETE"

