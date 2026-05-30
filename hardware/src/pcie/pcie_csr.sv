// PCIe control / status registers (AXI-Lite slave, 32-bit data).
// Modeled on ctrl_registers.sv -> wraps axi_lite_regs.
//
// Register map (all offsets are 32-bit words):
//   0x00  ID/VER         RO   {16'hPCIE, 16'h0100}
//   0x04  CONTROL        RW   [0] bridge_enable
//                             [1] loopback_mode (informational; set by SW)
//                             [8] irq_force (RW1S/W1C-by-status path)
//   0x08  LINK_STATUS    RO   [0] phy_link_up
//                             [1] fc_initialized
//   0x0C  CFG_IDS        RO   {bus[7:0], device[4:0], function[2:0], 16'h0}
//   0x10  BAR0_BASE_LO   RW   incoming PCIe BAR base, [31:0]
//   0x14  BAR0_BASE_HI   RW   incoming PCIe BAR base, [63:32]
//   0x18  BAR0_MASK_LO   RW   BAR mask (size = ~mask+1 within page)
//   0x1C  BAR0_MASK_HI   RW
//   0x20  AXI_TGT_LO     RW   AXI-side target base (default DRAMBase)
//   0x24  AXI_TGT_HI     RW
//   0x28  PHY_PARAMS     RO   {max_payload[2:0], max_read[2:0], rcb_128b, 25'h0}
//   0x2C  UR_COUNT       RO   unsupported-request completions emitted
//   0x30  CA_COUNT       RO   completer-abort completions emitted
//   0x34  TLP_ERR_COUNT  RO   malformed-TLP drops
//   0x38  OUTSTANDING    RO   reads currently in flight
//   0x3C  RESERVED       RO
//   0x40  IRQ_STATUS     RW1C latched IRQ bits ([0]=error, [1]=irq_force)
//   0x44  IRQ_MASK       RW   one-hot mask (1 = enabled)
//   0x48  BOOT_HOLD      RW   [0] hold CVA6 at boot until host clears it

`timescale 1ns / 1ps

module pcie_csr #(
    parameter int           AddrWidth = 64,
    parameter logic [31:0]  IdVersion = 32'h_50C1_0100,  // 'P','C',ver=01.00
    parameter logic [63:0]  AxiTargetBaseRst = 64'h8000_0000,
    parameter type          axi_lite_req_t  = logic,
    parameter type          axi_lite_resp_t = logic
) (
    input  logic                 clk_i,
    input  logic                 rst_ni,

    // AXI-Lite slave
    input  axi_lite_req_t        axi_lite_req_i,
    output axi_lite_resp_t       axi_lite_resp_o,

    // Live status inputs (from DLL / bridge)
    input  logic                 phy_link_up_i,
    input  logic                 fc_initialized_i,
    input  logic [7:0]           cfg_bus_number_i,
    input  logic [4:0]           cfg_device_number_i,
    input  logic [2:0]           cfg_function_number_i,
    input  logic [2:0]           max_payload_size_i,
    input  logic [2:0]           max_read_request_size_i,
    input  logic                 rcb_128b_i,

    // Counters driven by bridge
    input  logic [31:0]          ur_count_i,
    input  logic [31:0]          ca_count_i,
    input  logic [31:0]          tlp_err_count_i,
    input  logic [31:0]          outstanding_count_i,
    input  logic                 bridge_error_event_i,

    // Software-programmed controls out to the bridge
    output logic                 bridge_enable_o,
    output logic [63:0]          bar_base_o,
    output logic [63:0]          bar_mask_o,
    output logic [63:0]          axi_target_base_o,
    output logic                 boot_hold_o,

    // Chip-level IRQ
    output logic                 pcie_irq_o
);

  `include "common_cells/registers.svh"

  localparam int unsigned NumRegs          = 19;
  localparam int unsigned DataWidth        = 32;
  localparam int unsigned DataWidthInBytes = DataWidth / 8;
  localparam int unsigned RegNumBytes      = NumRegs * DataWidthInBytes;

  localparam logic [DataWidthInBytes-1:0] RO = {DataWidthInBytes{1'b1}};
  localparam logic [DataWidthInBytes-1:0] RW = {DataWidthInBytes{1'b0}};

  // Reset values (index order matches reg_q_o unpacking below).
  localparam logic [NumRegs-1:0][DataWidth-1:0] RegRstVal = '{
    /* 18: 0x48 BOOT_HOLD     */ 32'h0000_0001,
    /* 17: 0x44 IRQ_MASK      */ 32'h0000_0000,
    /* 16: 0x40 IRQ_STATUS    */ 32'h0000_0000,
    /* 15: 0x3C RESERVED      */ 32'h0000_0000,
    /* 14: 0x38 OUTSTANDING   */ 32'h0000_0000,
    /* 13: 0x34 TLP_ERR_COUNT */ 32'h0000_0000,
    /* 12: 0x30 CA_COUNT      */ 32'h0000_0000,
    /* 11: 0x2C UR_COUNT      */ 32'h0000_0000,
    /* 10: 0x28 PHY_PARAMS    */ 32'h0000_0000,
    /*  9: 0x24 AXI_TGT_HI    */ AxiTargetBaseRst[63:32],
    /*  8: 0x20 AXI_TGT_LO    */ AxiTargetBaseRst[31:0],
    /*  7: 0x1C BAR0_MASK_HI  */ 32'h0000_0000,
    /*  6: 0x18 BAR0_MASK_LO  */ 32'hFFFF_FFFF,
    /*  5: 0x14 BAR0_BASE_HI  */ 32'h0000_0000,
    /*  4: 0x10 BAR0_BASE_LO  */ 32'h0000_0000,
    /*  3: 0x0C CFG_IDS       */ 32'h0000_0000,
    /*  2: 0x08 LINK_STATUS   */ 32'h0000_0000,
    /*  1: 0x04 CONTROL       */ 32'h0000_0001,
    /*  0: 0x00 ID/VER        */ IdVersion
  };

  // Per-register byte-strobe RO/RW mask. axi_lite_regs treats a '1' byte as RO.
  localparam logic [NumRegs-1:0][DataWidthInBytes-1:0] AxiReadOnly = '{
    /* BOOT_HOLD     */ RW,
    /* IRQ_MASK      */ RW,
    /* IRQ_STATUS    */ RW,   // SW writes 1 to clear; we model that below
    /* RESERVED      */ RO,
    /* OUTSTANDING   */ RO,
    /* TLP_ERR_COUNT */ RO,
    /* CA_COUNT      */ RO,
    /* UR_COUNT      */ RO,
    /* PHY_PARAMS    */ RO,
    /* AXI_TGT_HI    */ RW,
    /* AXI_TGT_LO    */ RW,
    /* BAR0_MASK_HI  */ RW,
    /* BAR0_MASK_LO  */ RW,
    /* BAR0_BASE_HI  */ RW,
    /* BAR0_BASE_LO  */ RW,
    /* CFG_IDS       */ RO,
    /* LINK_STATUS   */ RO,
    /* CONTROL       */ RW,
    /* ID_VER        */ RO
  };

  // Latched IRQ bits, fed back into the register file via reg_load_i.
  logic [DataWidth-1:0] irq_status_q;
  logic                 irq_force_q;
  logic                 ctrl_irq_force_pulse;

  // reg_q_o exposed by axi_lite_regs (matches RegRstVal index order, LSB first).
  logic [DataWidth-1:0] id_ver_q;
  logic [DataWidth-1:0] control_q;
  logic [DataWidth-1:0] link_status_q;
  logic [DataWidth-1:0] cfg_ids_q;
  logic [DataWidth-1:0] bar_base_lo_q,  bar_base_hi_q;
  logic [DataWidth-1:0] bar_mask_lo_q,  bar_mask_hi_q;
  logic [DataWidth-1:0] axi_tgt_lo_q,   axi_tgt_hi_q;
  logic [DataWidth-1:0] phy_params_q;
  logic [DataWidth-1:0] ur_count_q, ca_count_q, tlp_err_count_q, outstanding_q;
  logic [DataWidth-1:0] reserved_q;
  logic [DataWidth-1:0] irq_status_reg_q;
  logic [DataWidth-1:0] irq_mask_q;
  logic [DataWidth-1:0] boot_hold_q;

  // Drive the RO and self-clearing registers via reg_load_i.
  logic [NumRegs-1:0][DataWidth-1:0] reg_d;
  logic [NumRegs-1:0]                reg_load;

  // Per-register write-active strobe from axi_lite_regs
  logic [RegNumBytes-1:0] wr_active_d, wr_active_q;
  `FF(wr_active_q, wr_active_d, '0);

  // Convenience: write strobes per 32b register
  logic [NumRegs-1:0] reg_wr_active;
  always_comb begin
    for (int i = 0; i < NumRegs; i++) begin
      reg_wr_active[i] = |wr_active_q[i*DataWidthInBytes +: DataWidthInBytes];
    end
  end

  // Live-driven registers (RO) and IRQ_STATUS update path.
  always_comb begin
    reg_d    = '0;
    reg_load = '0;

    // 0x08 LINK_STATUS
    reg_d   [2] = {30'h0, fc_initialized_i, phy_link_up_i};
    reg_load[2] = 1'b1;

    // 0x0C CFG_IDS = {bus[7:0], dev[4:0], func[2:0], 16'h0}
    reg_d   [3] = {cfg_bus_number_i, cfg_device_number_i, cfg_function_number_i, 16'h0};
    reg_load[3] = 1'b1;

    // 0x28 PHY_PARAMS
    reg_d   [10] = {max_payload_size_i, max_read_request_size_i, rcb_128b_i, 25'h0};
    reg_load[10] = 1'b1;

    // RO counters
    reg_d   [11] = ur_count_i;        reg_load[11] = 1'b1;
    reg_d   [12] = ca_count_i;        reg_load[12] = 1'b1;
    reg_d   [13] = tlp_err_count_i;   reg_load[13] = 1'b1;
    reg_d   [14] = outstanding_count_i; reg_load[14] = 1'b1;

    // 0x40 IRQ_STATUS - drive from internal latch
    reg_d   [16] = irq_status_q;
    reg_load[16] = 1'b1;
  end

  axi_lite_regs #(
    .RegNumBytes (RegNumBytes    ),
    .AxiAddrWidth(AddrWidth      ),
    .AxiDataWidth(DataWidth      ),
    .AxiReadOnly (AxiReadOnly    ),
    .RegRstVal   (RegRstVal      ),
    .req_lite_t  (axi_lite_req_t ),
    .resp_lite_t (axi_lite_resp_t)
  ) i_axi_lite_regs (
    .clk_i      (clk_i              ),
    .rst_ni     (rst_ni             ),
    .axi_req_i  (axi_lite_req_i     ),
    .axi_resp_o (axi_lite_resp_o    ),
    .wr_active_o(wr_active_d        ),
    .rd_active_o(/* unused */       ),
    .reg_d_i    (reg_d              ),
    .reg_load_i (reg_load           ),
    .reg_q_o    ({boot_hold_q,
                  irq_mask_q,
                  irq_status_reg_q,
                  reserved_q,
                  outstanding_q,
                  tlp_err_count_q,
                  ca_count_q,
                  ur_count_q,
                  phy_params_q,
                  axi_tgt_hi_q,
                  axi_tgt_lo_q,
                  bar_mask_hi_q,
                  bar_mask_lo_q,
                  bar_base_hi_q,
                  bar_base_lo_q,
                  cfg_ids_q,
                  link_status_q,
                  control_q,
                  id_ver_q})
  );

  // Software irq_force pulse: rising edge of control_q[8] write generates one
  // sticky IRQ event. (Simple model: forward as a level-set into irq_status_q.)
  logic control_irq_force_q;
  `FF(control_irq_force_q, control_q[8], 1'b0);
  assign ctrl_irq_force_pulse = control_q[8] & ~control_irq_force_q;

  // IRQ_STATUS latch:
  //   bit[0] = error event (any of UR/CA/tlp_err counter increments via bridge_error_event_i)
  //   bit[1] = sw force
  // SW clears by writing 1 to the corresponding bit of IRQ_STATUS (reg 16).
  // axi_lite_regs stored irq_status_reg_q reflects whatever SW most recently wrote;
  // we use the W1C semantics: any bit SW writes as 1 clears the matching latch bit
  // *next cycle*.
  logic sw_irq_clr_bit0, sw_irq_clr_bit1;
  assign sw_irq_clr_bit0 = reg_wr_active[16] & irq_status_reg_q[0];
  assign sw_irq_clr_bit1 = reg_wr_active[16] & irq_status_reg_q[1];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      irq_status_q <= '0;
    end else begin
      // bit0: error sticky
      if (sw_irq_clr_bit0) irq_status_q[0] <= 1'b0;
      else if (bridge_error_event_i) irq_status_q[0] <= 1'b1;

      // bit1: SW-forced
      if (sw_irq_clr_bit1) irq_status_q[1] <= 1'b0;
      else if (ctrl_irq_force_pulse) irq_status_q[1] <= 1'b1;

      // Upper bits reserved
      irq_status_q[31:2] <= '0;
    end
  end

  // Outputs to the bridge
  assign bridge_enable_o   = control_q[0];
  assign bar_base_o        = {bar_base_hi_q, bar_base_lo_q};
  assign bar_mask_o        = {bar_mask_hi_q, bar_mask_lo_q};
  assign axi_target_base_o = {axi_tgt_hi_q,  axi_tgt_lo_q};
  assign boot_hold_o       = boot_hold_q[0];

  // Chip-level IRQ pin
  assign pcie_irq_o = |(irq_status_q & irq_mask_q);

  // Avoid 'unused' warnings on signals we deliberately don't act on.
  logic _unused_ok;
  assign _unused_ok = |{id_ver_q, link_status_q, cfg_ids_q, phy_params_q,
	                        ur_count_q, ca_count_q, tlp_err_count_q, outstanding_q,
	                        reserved_q, boot_hold_q[31:1]};

endmodule
