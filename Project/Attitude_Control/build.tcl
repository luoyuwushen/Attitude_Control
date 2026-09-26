set project_dir [file dirname [file normalize [info script]]]
open_project "$project_dir/Attitude_Control.gprj"
set_option -top_module top
set_option -use_sspi_as_gpio 1
run all
exit
