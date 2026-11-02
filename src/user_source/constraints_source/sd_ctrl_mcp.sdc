set_multicycle_path 2 -setup -end -from [get_regs {sd_card_bmp_m0/wav_found_count* sd_card_bmp_m0/wav_ready* sd_card_bmp_m0/source_width_q*}]
set_multicycle_path 1 -hold -end -from [get_regs {sd_card_bmp_m0/wav_found_count* sd_card_bmp_m0/wav_ready* sd_card_bmp_m0/source_width_q*}]
puts "SD_CTRL_MCP_LOADED"
