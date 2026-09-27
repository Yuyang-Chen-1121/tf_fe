# Engine clock target: 100 MHz. Add this file only if the project does not
# already constrain clk. This is a clock constraint, not a complete interface
# timing specification. Input/output delays belong to the integrating system.
create_clock -name clk -period 10.000 [get_ports {clk}]
