create_clock -name sys_clk -period 20.000 [get_ports {clk_50m}]
create_generated_clock -name adc_clk -source [get_ports {clk_50m}] -divide_by 10 [get_ports {adc_clk}]
// Conservative board assumptions, not guaranteed ADC datasheet maxima.
// 3PA1030 Figure22: data changes after FALLING CLK, tOD=25ns typical.
// Capture at rise+80ns = previous fall+180ns; next fall is 20ns later.
// 45ns includes an assumed ADC/board budget, not a guaranteed tOD maximum.
set_input_delay -clock adc_clk -clock_fall -max 45.000 [get_ports {adc_data_in[*] adc_otr}]
set_input_delay -clock adc_clk -clock_fall -min 0.000 [get_ports {adc_data_in[*] adc_otr}]
set_multicycle_path -setup 9 -end -from [get_ports {adc_data_in[*] adc_otr}] -to [get_cells {u_adc/captured* u_adc/over_range*}]
// Hold pairs capture rise+80ns with the following launch fall=rise+100ns.
set_multicycle_path -hold 9 -end -from [get_ports {adc_data_in[*] adc_otr}] -to [get_cells {u_adc/captured* u_adc/over_range*}]
// Async external inputs are used only through their synchronizer front stages.
set_false_path -from [get_ports {rst_n key_sw[*] uart_rx enc1_a enc1_b}]
set_output_delay -clock sys_clk -max 5.000 [get_ports {uart_tx AN1 AN2 PWMA BN1 BN2 PWMB adc_oe_n}]
set_output_delay -clock sys_clk -min 0.000 [get_ports {uart_tx AN1 AN2 PWMA BN1 BN2 PWMB adc_oe_n}]
