create_project planAhead . -part xc6slx150fgg484-2
close [ open ./Pano-generated.ucf w ]
add_files -fileset constrs_1 -norecurse ../Pano.ucf ./Pano-generated.ucf
add_files -norecurse ../Pano.v
set_property target_constrs_file ./Pano-generated.ucf [current_fileset -constrset]

set_property strategy MapTiming [get_runs impl_1]

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1
