import_device eagle_s20.db -package EG4S20BG256
import_db verified_base_route.db
read_sdc ../user_source/constraints_source/scaler_phase.sdc
update_timing -mode final
report_timing_summary -file v95_timing_summary.rpt
report_timing_exception -file v95_phase_exception.rpt -ep_num 30
report_area -file v95_routed_area.rpt
export_db v95_route.db
puts V95_ROUTING_COMPLETE
set f [open v95_timing_summary.rpt r];set t [read $f];close $f
if {![regexp {SWNS:\s*([-0-9.]+)ns} $t -> ss] || ![regexp {HWNS:\s*([-0-9.]+)ns} $t -> hs]} {error "Missing timing"}
if {$ss<0 || $hs<0} {error "Timing failed. No bit generated."}
set f [open v95_phase_exception.rpt r];set t [read $f];close $f
if {![regexp {20124\s+20124\s+0\s+0\s+set_multicycle_path 2 -setup} $t]} {error "Scaler timing constraint did not bind"}
bitgen -bit ../../release_v95/Metro_V95.bit -bin ../../release_v95/Metro_V95.bin -version 95
puts V95_BITGEN_COMPLETE
