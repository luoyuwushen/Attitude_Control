create_clock -name sys_clk -period 20.000 [get_ports {clk_50m}]
create_generated_clock -name adc_clk -source [get_ports {clk_50m}] -divide_by 10 [get_ports {adc_clk}]
// Conservative board assumptions, not guaranteed ADC datasheet maxima.
// Capture register clock enable samples 80ns after the ADC rising edge.
set_input_delay -clock adc_clk -max 45.000 [get_ports {adc_data_in[*] adc_otr}]
set_input_delay -clock adc_clk -min 0.000 [get_ports {adc_data_in[*] adc_otr}]
set_multicycle_path -setup 4 -end -from [get_ports {adc_data_in[*] adc_otr}] -to [get_cells {u_adc/captured* u_adc/over_range*}]
// At the enabled capture (80ns), next ADC launch is 200ns: hold gap is 120ns.
set_multicycle_path -hold 9 -end -from [get_ports {adc_data_in[*] adc_otr}] -to [get_cells {u_adc/captured* u_adc/over_range*}]
// Async external inputs are used only through their synchronizer front stages.
set_false_path -from [get_ports {rst_n key_sw[*] uart_rx enc1_a enc1_b}]
set_output_delay -clock sys_clk -max 5.000 [get_ports {uart_tx AN1 AN2 PWMA BN1 BN2 PWMB adc_oe_n}]
set_output_delay -clock sys_clk -min 0.000 [get_ports {uart_tx AN1 AN2 PWMA BN1 BN2 PWMB adc_oe_n}]
