
# SYSCLK 200MHz
set_property IOSTANDARD LVDS [get_ports SYSCLK_P]
set_property PACKAGE_PIN E19 [get_ports SYSCLK_P]
set_property PACKAGE_PIN E18 [get_ports SYSCLK_N]
set_property IOSTANDARD LVDS [get_ports SYSCLK_N]

# Pushbuttons
set_property PACKAGE_PIN AV40 [get_ports RESET_BUTTON]
set_property IOSTANDARD LVCMOS18 [get_ports RESET_BUTTON]
set_property PACKAGE_PIN AW40 [get_ports next_addr]
set_property IOSTANDARD LVCMOS18 [get_ports next_addr]
set_property PACKAGE_PIN AU38 [get_ports prev_addr]
set_property IOSTANDARD LVCMOS18 [get_ports prev_addr]

#GPIO
set_property PACKAGE_PIN AR39 [get_ports TCK]
set_property PACKAGE_PIN AT42 [get_ports TDO]
set_property PACKAGE_PIN AN40 [get_ports TDI]
set_property PACKAGE_PIN AR42 [get_ports TMS]
set_property IOSTANDARD LVCMOS18 [get_ports TCK]
set_property IOSTANDARD LVCMOS18 [get_ports TDI]
set_property IOSTANDARD LVCMOS18 [get_ports TDO]
set_property IOSTANDARD LVCMOS18 [get_ports TMS]

# LEDs
set_property PACKAGE_PIN AM39 [get_ports {LED[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {LED[0]}]
set_property PACKAGE_PIN AN39 [get_ports {LED[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {LED[1]}]
set_property PACKAGE_PIN AR37 [get_ports {LED[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {LED[2]}]
set_property PACKAGE_PIN AT37 [get_ports {LED[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {LED[3]}]
set_property PACKAGE_PIN AR35 [get_ports {LED[4]}]
set_property IOSTANDARD LVCMOS18 [get_ports {LED[4]}]
set_property PACKAGE_PIN AP41 [get_ports {LED[5]}]
set_property IOSTANDARD LVCMOS18 [get_ports {LED[5]}]
set_property PACKAGE_PIN AP42 [get_ports {LED[6]}]
set_property IOSTANDARD LVCMOS18 [get_ports {LED[6]}]
set_property PACKAGE_PIN AU39 [get_ports {LED[7]}]
set_property IOSTANDARD LVCMOS18 [get_ports {LED[7]}]

#DIP Switches
set_property PACKAGE_PIN AV30 [get_ports DEBUG_OVERWRITE]
set_property IOSTANDARD LVCMOS18 [get_ports DEBUG_OVERWRITE]

set_property PACKAGE_PIN AY33 [get_ports ext_mode]
set_property IOSTANDARD LVCMOS18 [get_ports ext_mode]

set_property PACKAGE_PIN BA31 [get_ports debug_display]
set_property IOSTANDARD LVCMOS18 [get_ports debug_display]



create_clock -period 500.000 -name TCK -waveform {0.000 250.000} [get_ports TCK]
set_false_path -from [get_clocks clk_out1_clk_wiz_0] -to [get_clocks TCK]
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets TCK_IBUF]


set_false_path -from [get_pins Clock_module/inst/mmcm_adv_inst/CLKOUT1] -to [get_pins CLINT/rtc_synchronizer/meta_reg/D]

create_clock -period 50.000 -name VIRTUAL_clk_out1_clk_wiz_0 -waveform {0.000 25.000}
set_input_delay -clock [get_clocks VIRTUAL_clk_out1_clk_wiz_0] -min -add_delay 4.000 [get_ports RESET_BUTTON]
set_input_delay -clock [get_clocks VIRTUAL_clk_out1_clk_wiz_0] -max -add_delay 4.000 [get_ports RESET_BUTTON]




