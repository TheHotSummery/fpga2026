# p00 is captured only in phase 1; h0 only in phase 4.
# The state machine advances 1->2->3->4 without a shortcut. Therefore this
# data input is stable for >=3 clocks before capture. Use only 2 clocks here.
# Phase/enable/fraction paths retain their ordinary single-clock checks.
set scaler_mcp_from [get_regs {sd_card_bmp_m0/u_video_reconstruct/p00*}]
set scaler_mcp_to [get_regs {sd_card_bmp_m0/u_video_reconstruct/h0*}]
if {[llength $scaler_mcp_from] != 16 || [llength $scaler_mcp_to] != 24} {
 error "Scaler register mapping changed: revalidate its clock-enable proof."
}
foreach n $scaler_mcp_from {
 if {![regexp {^sd_card_bmp_m0/u_video_reconstruct/p00\[[0-9]+\]$} $n]} {error "Unexpected MCP source: $n"}
}
foreach n $scaler_mcp_to {
 if {![regexp {^sd_card_bmp_m0/u_video_reconstruct/h0\[[0-9]+\]$} $n]} {error "Unexpected MCP target: $n"}
}
set_multicycle_path 2 -setup -end -from [get_regs {sd_card_bmp_m0/u_video_reconstruct/p00*}] -to [get_regs {sd_card_bmp_m0/u_video_reconstruct/h0*}]
set_multicycle_path 1 -hold -end -from [get_regs {sd_card_bmp_m0/u_video_reconstruct/p00*}] -to [get_regs {sd_card_bmp_m0/u_video_reconstruct/h0*}]
# wav_found_count / wav_ready are written only during initial boot and stay constant in STATE_IDLE;
# source_width_q is latched at media load start before pixel streaming begins.
set_multicycle_path 2 -setup -end -from [get_regs {sd_card_bmp_m0/wav_found_count* sd_card_bmp_m0/wav_ready* sd_card_bmp_m0/source_width_q*}]
set_multicycle_path 1 -hold -end -from [get_regs {sd_card_bmp_m0/wav_found_count* sd_card_bmp_m0/wav_ready* sd_card_bmp_m0/source_width_q*}]
puts "V95_SCALER_PHASE_CONSTRAINT_BOUND sources=16 targets=24 setup=2 hold=1"
