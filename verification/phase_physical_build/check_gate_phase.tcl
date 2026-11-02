import_device eagle_s20.db -package EG4S20BG256
import_db gate_for_binding_check.db
read_sdc ../user_source/constraints_source/scaler_phase.sdc
update_timing -mode basic
puts V95_GATE_STAGE_PHASE_CONSTRAINT_PASS
