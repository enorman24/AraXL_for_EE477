// PCIe PHY stub.
//
// Terminates the AXI-Stream PHY boundary of `pcie_datalink_layer`. Two modes:
//   LOOPBACK_MODE=1: m_phy_axis is wrapped back into s_phy_axis. The DLL talks
//                    to itself; useful for self-stim and ASIC bring-up where no
//                    real PCIe partner exists.
//   LOOPBACK_MODE=0: PHY pads pass through to chip-level ports (future real-PHY
//                    integration point).
//
// In either mode, phy_link_up_o is asserted after a small reset-release counter
// so the DLL transitions out of DL_INACTIVE and initializes flow control.

`timescale 1ns / 1ps

module pcie_phy_stub #(
    parameter int DATA_WIDTH    = 32,
    parameter int KEEP_WIDTH    = DATA_WIDTH / 8,
    parameter int USER_WIDTH    = 3,
    parameter bit LOOPBACK_MODE = 1'b1,
    parameter int LINKUP_DELAY  = 16
) (
    input  logic                  clk_i,
    input  logic                  rst_ni,

    // DLL-facing: drive these into pcie_datalink_layer.s_phy_axis_*
    output logic [DATA_WIDTH-1:0] dll_phy_rx_tdata_o,
    output logic [KEEP_WIDTH-1:0] dll_phy_rx_tkeep_o,
    output logic                  dll_phy_rx_tvalid_o,
    output logic                  dll_phy_rx_tlast_o,
    output logic [USER_WIDTH-1:0] dll_phy_rx_tuser_o,
    input  logic                  dll_phy_rx_tready_i,

    // DLL-facing: take these from pcie_datalink_layer.m_phy_axis_*
    input  logic [DATA_WIDTH-1:0] dll_phy_tx_tdata_i,
    input  logic [KEEP_WIDTH-1:0] dll_phy_tx_tkeep_i,
    input  logic                  dll_phy_tx_tvalid_i,
    input  logic                  dll_phy_tx_tlast_i,
    input  logic [USER_WIDTH-1:0] dll_phy_tx_tuser_i,
    output logic                  dll_phy_tx_tready_o,

    // External PHY pads (unused in loopback mode; future real PHY hookup)
    output logic [DATA_WIDTH-1:0] pad_tx_tdata_o,
    output logic [KEEP_WIDTH-1:0] pad_tx_tkeep_o,
    output logic                  pad_tx_tvalid_o,
    output logic                  pad_tx_tlast_o,
    output logic [USER_WIDTH-1:0] pad_tx_tuser_o,
    input  logic                  pad_tx_tready_i,
    input  logic [DATA_WIDTH-1:0] pad_rx_tdata_i,
    input  logic [KEEP_WIDTH-1:0] pad_rx_tkeep_i,
    input  logic                  pad_rx_tvalid_i,
    input  logic                  pad_rx_tlast_i,
    input  logic [USER_WIDTH-1:0] pad_rx_tuser_i,
    output logic                  pad_rx_tready_o,

    output logic phy_link_up_o,
    output logic idle_valid_o
);

  // Link-up after LINKUP_DELAY cycles out of reset.
  logic [$clog2(LINKUP_DELAY+1)-1:0] linkup_cnt_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      linkup_cnt_q <= '0;
    end else if (linkup_cnt_q != LINKUP_DELAY[$clog2(LINKUP_DELAY+1)-1:0]) begin
      linkup_cnt_q <= linkup_cnt_q + 1'b1;
    end
  end

  assign phy_link_up_o = (linkup_cnt_q == LINKUP_DELAY[$clog2(LINKUP_DELAY+1)-1:0]);
  assign idle_valid_o  = phy_link_up_o;

  generate
    if (LOOPBACK_MODE) begin : g_loopback
      // Wrap TX -> RX internally. Ignore external pads.
      assign dll_phy_rx_tdata_o  = dll_phy_tx_tdata_i;
      assign dll_phy_rx_tkeep_o  = dll_phy_tx_tkeep_i;
      assign dll_phy_rx_tvalid_o = dll_phy_tx_tvalid_i & phy_link_up_o;
      assign dll_phy_rx_tlast_o  = dll_phy_tx_tlast_i;
      assign dll_phy_rx_tuser_o  = dll_phy_tx_tuser_i;
      assign dll_phy_tx_tready_o = dll_phy_rx_tready_i | ~phy_link_up_o;

      assign pad_tx_tdata_o      = '0;
      assign pad_tx_tkeep_o      = '0;
      assign pad_tx_tvalid_o     = 1'b0;
      assign pad_tx_tlast_o      = 1'b0;
      assign pad_tx_tuser_o      = '0;
      assign pad_rx_tready_o     = 1'b0;
    end else begin : g_passthrough
      assign pad_tx_tdata_o      = dll_phy_tx_tdata_i;
      assign pad_tx_tkeep_o      = dll_phy_tx_tkeep_i;
      assign pad_tx_tvalid_o     = dll_phy_tx_tvalid_i;
      assign pad_tx_tlast_o      = dll_phy_tx_tlast_i;
      assign pad_tx_tuser_o      = dll_phy_tx_tuser_i;
      assign dll_phy_tx_tready_o = pad_tx_tready_i;

      assign dll_phy_rx_tdata_o  = pad_rx_tdata_i;
      assign dll_phy_rx_tkeep_o  = pad_rx_tkeep_i;
      assign dll_phy_rx_tvalid_o = pad_rx_tvalid_i & phy_link_up_o;
      assign dll_phy_rx_tlast_o  = pad_rx_tlast_i;
      assign dll_phy_rx_tuser_o  = pad_rx_tuser_i;
      assign pad_rx_tready_o     = dll_phy_rx_tready_i;
    end
  endgenerate

endmodule
