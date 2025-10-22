onerror {quit -f}
vlib work
vlog +define+SIM+SIM_MODE+EFX_SIM -sv ./tb_divider.v
vlog +define+SIM+SIM_MODE+EFX_SIM -sv ./demo_divider.v
vlog +define+SIM+SIM_MODE+EFX_SIM -sv ./div_u39_u31.v
vsim -t ns work.tb_divider
run -all
