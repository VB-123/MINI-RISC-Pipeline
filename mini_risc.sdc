# Primary input clock: 50 MHz
create_clock -name ref_clk -period 20.000 [get_ports {CLOCK_50}]

derive_clock_uncertainty