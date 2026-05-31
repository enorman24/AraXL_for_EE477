
module ara_chip_tb_harness #(
  parameter int unsigned NrLanes                = 0,
  parameter int unsigned NrClusters             = 0,
  parameter int unsigned AxiDataWidth           = 32 * NrLanes * NrClusters,
  parameter int unsigned ClusterAxiDataWidth    = 32 * NrLanes,
  parameter int unsigned AxiAddrWidth           = 64,
  parameter int unsigned AxiUserWidth           = 1,
  parameter int unsigned AxiIdWidth             = 6,
  parameter int unsigned L2NumWords             = 1024,
  parameter int unsigned AxiRespDelay           = 200,
  parameter int unsigned PcieBridgeAxiDataWidth = 64
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  // Phase control (loading → execution hand-off)
  input  logic        loading_done_i,
  output logic [63:0] exit_o,
  output logic        pcie_irq_o,
  // TLP injection: TB drives during loading, host-partner DLL drives during execution
  input  logic [31:0] s_tlp_tdata_i,
  input  logic [3:0]  s_tlp_tkeep_i,
  input  logic        s_tlp_tvalid_i,
  input  logic        s_tlp_tlast_i,
  input  logic [2:0]  s_tlp_tuser_i,
  output logic        s_tlp_tready_o,
  input  logic        m_tlp_tready_i,
  // Observability outputs consumed by the testbench
  output logic        pcie_load_ready_o,
  output logic        boot_hold_o,
  output int unsigned beh_write_count_o
);

  // -----------------------------------------------------------------------
  // PHY pad interconnect wires (host partner ↔ ara_chip pad boundary)
  // -----------------------------------------------------------------------
  // RX direction: host partner → ara_chip pad_rx inputs
  logic [31:0] pad_rx_tdata;
  logic [3:0]  pad_rx_tkeep;
  logic        pad_rx_tvalid;
  logic        pad_rx_tlast;
  logic [2:0]  pad_rx_tuser;
  logic        pad_rx_tready;   // ara_chip → host partner

  // TX direction: ara_chip pad_tx outputs → host partner
  logic [31:0] pad_tx_tdata;
  logic [3:0]  pad_tx_tkeep;
  logic        pad_tx_tvalid;
  logic        pad_tx_tlast;
  logic [2:0]  pad_tx_tuser;
  logic        pad_tx_tready;   // host partner → ara_chip

  // -----------------------------------------------------------------------
  // Host partner model (host-side DLL + PHY stub + feature-exchange bootstrap)
  // -----------------------------------------------------------------------
  logic        host_fc_initialized;
  logic        host_link_up;
  int unsigned host_rx_tlp_count;
  int unsigned host_phy_packet_count;

  pcie_host_partner_model #(
    .DATA_WIDTH       (32 ),
    .USER_WIDTH       (3  ),
    .S_COUNT          (2  ),
    .RX_FIFO_SIZE     (3  ),
    .RETRY_TLP_SIZE   (3  ),
    .MAX_PAYLOAD_SIZE (256),
    .LINKUP_DELAY     (16 )
  ) i_pcie_host_partner (
    .clk_i                  (clk_i                                      ),
    .rst_ni                 (rst_ni                                     ),
    // TB drives TLPs only during the loading phase.
    .s_tlp_tdata_i          (s_tlp_tdata_i                              ),
    .s_tlp_tkeep_i          (s_tlp_tkeep_i                              ),
    .s_tlp_tvalid_i         (loading_done_i ? 1'b0 : s_tlp_tvalid_i    ),
    .s_tlp_tlast_i          (s_tlp_tlast_i                              ),
    .s_tlp_tready_o         (s_tlp_tready_o                             ),
    // Host → device (pad_rx): host partner drives the chip's PHY RX inputs
    .device_phy_rx_tdata_o  (pad_rx_tdata                               ),
    .device_phy_rx_tkeep_o  (pad_rx_tkeep                               ),
    .device_phy_rx_tvalid_o (pad_rx_tvalid                              ),
    .device_phy_rx_tlast_o  (pad_rx_tlast                               ),
    .device_phy_rx_tuser_o  (pad_rx_tuser                               ),
    .device_phy_rx_tready_i (pad_rx_tready                              ),
    // Device → host (pad_tx): host partner receives the chip's PHY TX outputs
    .device_phy_tx_tdata_i  (pad_tx_tdata                               ),
    .device_phy_tx_tkeep_i  (pad_tx_tkeep                               ),
    .device_phy_tx_tvalid_i (pad_tx_tvalid                              ),
    .device_phy_tx_tlast_i  (pad_tx_tlast                               ),
    .device_phy_tx_tuser_i  (pad_tx_tuser                               ),
    .device_phy_tx_tready_o (pad_tx_tready                              ),
    .link_up_o              (host_link_up                               ),
    .host_fc_initialized_o  (host_fc_initialized                        ),
    .host_rx_tlp_count_o    (host_rx_tlp_count                          ),
    .host_phy_packet_count_o(host_phy_packet_count                      )
  );

  // -----------------------------------------------------------------------
  // ara_chip — chip-level DUT
  // Internal pcie_phy_stub is in LOOPBACK_MODE=0 (passthrough) so the
  // host partner's PHY packets flow through to the DLL unchanged.
  // -----------------------------------------------------------------------
  logic        uart_penable, uart_pwrite, uart_psel;
  logic [31:0] uart_paddr, uart_pwdata, uart_prdata;
  logic        uart_pready, uart_pslverr;
  logic [63:0] hw_cnt_en;

  ara_chip #(
    .NrLanes             (NrLanes            ),
    .NrClusters          (NrClusters         ),
    .AxiDataWidth        (AxiDataWidth       ),
    .ClusterAxiDataWidth (ClusterAxiDataWidth),
    .AxiAddrWidth        (AxiAddrWidth       ),
    .AxiUserWidth        (AxiUserWidth       ),
    .AxiIdWidth          (AxiIdWidth         ),
    .AxiRespDelay        (AxiRespDelay       ),
    .L2NumWords          (L2NumWords         )
  ) i_ara_chip (
    .clk_i                   (clk_i         ),
    .rst_ni                  (rst_ni        ),
    .scan_enable_i           (1'b0          ),
    .scan_data_i             (1'b0          ),
    .scan_data_o             (/* unused */  ),
    .uart_penable_o          (uart_penable  ),
    .uart_pwrite_o           (uart_pwrite   ),
    .uart_paddr_o            (uart_paddr    ),
    .uart_psel_o             (uart_psel     ),
    .uart_pwdata_o           (uart_pwdata   ),
    .uart_prdata_i           (uart_prdata   ),
    .uart_pready_i           (uart_pready   ),
    .uart_pslverr_i          (uart_pslverr  ),
    // PHY pad ports wired to/from host partner via intermediate nets
    .pcie_phy_pad_tx_tdata_o (pad_tx_tdata  ),
    .pcie_phy_pad_tx_tkeep_o (pad_tx_tkeep  ),
    .pcie_phy_pad_tx_tvalid_o(pad_tx_tvalid ),
    .pcie_phy_pad_tx_tlast_o (pad_tx_tlast  ),
    .pcie_phy_pad_tx_tuser_o (pad_tx_tuser  ),
    .pcie_phy_pad_tx_tready_i(pad_tx_tready ),
    .pcie_phy_pad_rx_tdata_i (pad_rx_tdata  ),
    .pcie_phy_pad_rx_tkeep_i (pad_rx_tkeep  ),
    .pcie_phy_pad_rx_tvalid_i(pad_rx_tvalid ),
    .pcie_phy_pad_rx_tlast_i (pad_rx_tlast  ),
    .pcie_phy_pad_rx_tuser_i (pad_rx_tuser  ),
    .pcie_phy_pad_rx_tready_o(pad_rx_tready ),
    .pcie_irq_o              (pcie_irq_o    ),
    .exit_o                  (exit_o        ),
    .hw_cnt_en_o             (hw_cnt_en     )
  );

  // -----------------------------------------------------------------------
  // mock_uart
  // -----------------------------------------------------------------------
  mock_uart i_mock_uart (
    .clk_i    (clk_i       ),
    .rst_ni   (rst_ni      ),
    .penable_i(uart_penable),
    .pwrite_i (uart_pwrite ),
    .paddr_i  (uart_paddr  ),
    .psel_i   (uart_psel   ),
    .pwdata_i (uart_pwdata ),
    .prdata_o (uart_prdata ),
    .pready_o (uart_pready ),
    .pslverr_o(uart_pslverr)
  );

  // -----------------------------------------------------------------------
  // Observability
  // -----------------------------------------------------------------------

  // PCIe load-ready: both host and device DLLs must complete FC init.
  assign pcie_load_ready_o = host_fc_initialized & i_ara_chip.fc_initialized;

  // Boot hold: pcie_boot_hold is the local wire in ara_chip, driven by
  // ara_soc.pcie_boot_hold_o (itself a pcie_csr register).
  // The TB clears it by sending an MWr TLP to BootHoldTlpAddr.
  assign boot_hold_o = i_ara_chip.pcie_boot_hold;

  // PCIe write counter: count completed bursts on the wide AXI side of the DWC.
  int unsigned pcie_write_cnt;
  assign beh_write_count_o = pcie_write_cnt;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      pcie_write_cnt <= 0;
    end else if (i_ara_chip.pcie_wide_req.w_valid  &&
                 i_ara_chip.pcie_wide_resp.w_ready &&
                 i_ara_chip.pcie_wide_req.w.last) begin
      pcie_write_cnt <= pcie_write_cnt + 1;
    end
  end

  // -----------------------------------------------------------------------
  // Vector runtime counter (mirrors ara_pcie_full_testharness)
  // -----------------------------------------------------------------------
  logic [63:0] runtime_cnt_d, runtime_cnt_q;
  logic [63:0] runtime_buf_d, runtime_buf_q;
  logic        runtime_cnt_en_d, runtime_cnt_en_q;
  logic        runtime_to_be_updated_d, runtime_to_be_updated_q;

  always_comb begin
    runtime_cnt_en_d = runtime_cnt_en_q;
    if (!runtime_cnt_en_q)
      runtime_cnt_en_d = i_ara_chip.i_ara_soc.i_system.i_ara_cluster.acc_req_i.req_valid;
    if (runtime_cnt_en_q)
      runtime_cnt_en_d = ~i_ara_chip.i_ara_soc.i_system.i_ara_cluster.p_cluster[0].i_ara_macro.i_ara.ara_idle;
  end

  always_comb begin
    runtime_cnt_d           = runtime_cnt_q;
    runtime_to_be_updated_d = runtime_to_be_updated_q;
    runtime_buf_d           = runtime_buf_q;
    if (runtime_cnt_en_q) runtime_cnt_d = runtime_cnt_q + 1;
    if (!runtime_to_be_updated_q &&
        i_ara_chip.i_ara_soc.i_system.i_ara_cluster.acc_req_i.req_valid)
      runtime_to_be_updated_d = 1'b1;
    if (runtime_to_be_updated_q &&
        i_ara_chip.i_ara_soc.i_system.i_ara_cluster.p_cluster[0].i_ara_macro.i_ara.ara_idle &&
        !i_ara_chip.i_ara_soc.i_system.i_ara_cluster.acc_req_i.req_valid) begin
      runtime_buf_d           = runtime_cnt_q;
      runtime_to_be_updated_d = 1'b0;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      runtime_cnt_en_q        <= 1'b0;
      runtime_cnt_q           <= '0;
      runtime_to_be_updated_q <= '0;
      runtime_buf_q           <= '0;
    end else begin
      runtime_cnt_en_q        <= runtime_cnt_en_d;
      runtime_cnt_q           <= runtime_cnt_d;
      runtime_to_be_updated_q <= runtime_to_be_updated_d;
      runtime_buf_q           <= runtime_buf_d;
    end
  end

  // -----------------------------------------------------------------------
  // CVA6 performance counters
  // -----------------------------------------------------------------------
  logic [63:0] dcache_stall_cnt_d, dcache_stall_cnt_q;
  logic [63:0] icache_stall_cnt_d, icache_stall_cnt_q;
  logic [63:0] sb_full_cnt_d,      sb_full_cnt_q;
  logic [63:0] dcache_stall_buf_q, icache_stall_buf_q, sb_full_buf_q;

  always_comb begin
    dcache_stall_cnt_d = dcache_stall_cnt_q;
    icache_stall_cnt_d = icache_stall_cnt_q;
    sb_full_cnt_d      = sb_full_cnt_q;
    dcache_stall_buf_q = dcache_stall_cnt_q;
    icache_stall_buf_q = icache_stall_cnt_q;
    sb_full_buf_q      = sb_full_cnt_q;
    if (runtime_cnt_en_q &&
        i_ara_chip.i_ara_soc.i_system.i_ariane.gen_perf_counter.perf_counters_i.l1_dcache_miss_i)
      dcache_stall_cnt_d += 1;
    if (runtime_cnt_en_q &&
        i_ara_chip.i_ara_soc.i_system.i_ariane.gen_perf_counter.perf_counters_i.l1_icache_miss_i)
      icache_stall_cnt_d += 1;
    if (runtime_cnt_en_q &&
        i_ara_chip.i_ara_soc.i_system.i_ariane.gen_perf_counter.perf_counters_i.sb_full_i)
      sb_full_cnt_d += 1;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dcache_stall_cnt_q <= '0;
      icache_stall_cnt_q <= '0;
      sb_full_cnt_q      <= '0;
    end else begin
      dcache_stall_cnt_q <= dcache_stall_cnt_d;
      icache_stall_cnt_q <= icache_stall_cnt_d;
      sb_full_cnt_q      <= sb_full_cnt_d;
    end
  end

  logic _unused;
  assign _unused = |{hw_cnt_en, host_link_up, host_rx_tlp_count, host_phy_packet_count,
                     s_tlp_tuser_i, m_tlp_tready_i};

endmodule : ara_chip_tb_harness
