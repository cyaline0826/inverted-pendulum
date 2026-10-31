create_clock -name sys_clk -period 20.000 [get_ports {sys_clk}]

# daolibai locks PID input snapshots, then commits once after 32 clocks.
# Internal PID history changes only on that commit; all other paths stay single-cycle.
set_multicycle_path -from [get_regs {pid_*_snapshot*}] -to [get_regs {u_angle_loop/*}] -setup -end 32
set_multicycle_path -from [get_regs {pid_*_snapshot*}] -to [get_regs {u_angle_loop/*}] -hold -start 31
set_multicycle_path -from [get_regs {u_angle_loop/*}] -to [get_regs {u_angle_loop/*}] -setup -end 32
set_multicycle_path -from [get_regs {u_angle_loop/*}] -to [get_regs {u_angle_loop/*}] -hold -start 31
