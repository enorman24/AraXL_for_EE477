# constraints.tcl
#
# Block-level SDC constraints for ara_soc on sky130. Single clock domain
# (clk_i), async rst_ni, and a DFT scan chain that is unused in functional
# mode. No CDC primitives or multi-clock crossings are instantiated below
# ara_soc, so no clock groups or multicycle exceptions are needed.

#------------------------------------------------------------------------
# Clock
#------------------------------------------------------------------------
# Starting at 10 ns (100 MHz) for bring-up. CVA6+Ara on sky130 will almost
# certainly miss timing here -- relax CORE_CLOCK_PERIOD (archive uses 50 ns)
# once you see the initial WNS.
set CORE_CLOCK_PERIOD     80
set CORE_CLOCK_UNCERT      0.5
set CORE_CLOCK_TRANSITION  0.1

create_clock -name core_clk -period $CORE_CLOCK_PERIOD [get_ports clk_i]

set_clock_uncertainty $CORE_CLOCK_UNCERT [get_clocks core_clk]
set_clock_transition -rise $CORE_CLOCK_TRANSITION [get_clocks core_clk]
set_clock_transition -fall $CORE_CLOCK_TRANSITION [get_clocks core_clk]

#------------------------------------------------------------------------
# IO slew assumptions
#------------------------------------------------------------------------
set_input_transition -max 0.5  [all_inputs]
set_input_transition -min 0.1  [all_inputs]
# Clock pin gets a tighter slew than generic data
set_input_transition -max 0.2  [get_ports clk_i]
set_input_transition -min 0.05 [get_ports clk_i]

#------------------------------------------------------------------------
# Async / DFT exceptions
#------------------------------------------------------------------------
# rst_ni is an asynchronous active-low reset (FF macro uses negedge rst_ni).
set_false_path -from [get_ports rst_ni]

# Scan chain: i_system in ara_soc ties scan_enable/scan_data to '0 and leaves
# scan_data_o unconnected, so the DFT path is dead in functional mode. Force
# functional mode for STA and false-path the scan I/O for safety.
set_case_analysis 0 [get_ports scan_enable_i]
set_false_path -from [get_ports scan_data_i]
set_false_path -to   [get_ports scan_data_o]

#------------------------------------------------------------------------
# Synchronous input/output delays
#------------------------------------------------------------------------
# Budget 40% of the period each for setup, with a small min floor.
set INPUT_DELAY_MAX  [expr {0.40 * $CORE_CLOCK_PERIOD}]
set INPUT_DELAY_MIN  0.5
set OUTPUT_DELAY_MAX [expr {0.40 * $CORE_CLOCK_PERIOD}]
set OUTPUT_DELAY_MIN 0.5

# Exclude clock + async/DFT ports from synchronous IO timing.
set async_inputs [get_ports {rst_ni scan_enable_i scan_data_i}]
set sync_inputs  [remove_from_collection [all_inputs] [get_ports clk_i]]
set sync_inputs  [remove_from_collection $sync_inputs $async_inputs]

if {[sizeof_collection $sync_inputs] > 0} {
  set_input_delay -max $INPUT_DELAY_MAX -clock [get_clocks core_clk] $sync_inputs
  set_input_delay -min $INPUT_DELAY_MIN -clock [get_clocks core_clk] $sync_inputs
}

set async_outputs [get_ports {scan_data_o}]
set sync_outputs  [remove_from_collection [all_outputs] $async_outputs]

if {[sizeof_collection $sync_outputs] > 0} {
  set_output_delay -max $OUTPUT_DELAY_MAX -clock [get_clocks core_clk] $sync_outputs
  set_output_delay -min $OUTPUT_DELAY_MIN -clock [get_clocks core_clk] $sync_outputs
}

#------------------------------------------------------------------------
# Output load
#------------------------------------------------------------------------
# sky130 lib units are pF; 0.010 pF (10 fF) models a few fanout cells.
set_load 0.010 [all_outputs]
