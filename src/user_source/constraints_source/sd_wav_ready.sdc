set_multicycle_path 2 -setup -end -from [get_regs {sd_card_bmp_m0/wav_ready*}]
set_multicycle_path 1 -hold -end -from [get_regs {sd_card_bmp_m0/wav_ready*}]
puts "SD_WAV_READY_MCP_LOADED"
