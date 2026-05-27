// ara_soc_syn.sv
//
// Hammer/Genus synthesis top for AraXL. Hammer only supports
// synthesis.inputs.top_module (no parameter overrides on elaborate), so this
// wrapper fixes the RVV geometry for the chip flow:
//   NrClusters = 2  (matches Makefile default nr_clusters=2)
//   NrLanes    = 4  (matches config/4_lanes.mk)
//
// Yields AxiDataWidth = 32 * 4 * 2 = 256, matching fakeram_d1024_w256 on L2.
// Port list matches ara_soc so constraints.tcl stays valid.

module ara_soc_syn (
  input  logic        clk_i,
  input  logic        rst_ni,
  output logic [63:0] exit_o,
  output logic [63:0] hw_cnt_en_o,
  input  logic        scan_enable_i,
  input  logic        scan_data_i,
  output logic        scan_data_o,
  output logic        uart_penable_o,
  output logic        uart_pwrite_o,
  output logic [31:0] uart_paddr_o,
  output logic        uart_psel_o,
  output logic [31:0] uart_pwdata_o,
  input  logic [31:0] uart_prdata_i,
  input  logic        uart_pready_i,
  input  logic        uart_pslverr_i
);

  localparam int unsigned NrLanes     = 4;
  localparam int unsigned NrClusters  = 2;

  ara_soc #(
    .NrLanes    (NrLanes    ),
    .NrClusters (NrClusters )
  ) i_ara_soc (
    .*
  );

endmodule
