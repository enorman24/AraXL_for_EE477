`timescale 1ns / 1ps

module ara_chip
    import axi_pkg::*;
    import ara_pkg::*;
#(
    parameter int  unsigned NrLanes             = 2,
    parameter int  unsigned NrClusters          = 1,
    parameter fpu_support_e   FPUSupport        = FPUSupportHalfSingleDouble,
    parameter fpext_support_e FPExtSupport      = FPExtSupportEnable,
    parameter fixpt_support_e FixPtSupport      = FixedPointEnable,
    parameter int  unsigned AxiDataWidth        = 128,
    parameter int  unsigned ClusterAxiDataWidth = 32 * NrLanes,
    parameter int  unsigned AxiAddrWidth        = 64,
    parameter int  unsigned AxiUserWidth        = 1,
    parameter int  unsigned AxiIdWidth          = 6,
    parameter int  unsigned AxiRespDelay        = 200,
    parameter int  unsigned L2NumWords          = 1024,
    parameter int  unsigned PcieBridgeAxiDataWidth = 64
) (
    input  logic        clk_i,
    input  logic        rst_ni,
    // Scan chain
    input  logic        scan_enable_i,
    input  logic        scan_data_i,
    output logic        scan_data_o,
    // UART APB pass-through
    output logic        uart_penable_o,
    output logic        uart_pwrite_o,
    output logic [31:0] uart_paddr_o,
    output logic        uart_psel_o,
    output logic [31:0] uart_pwdata_o,
    input  logic [31:0] uart_prdata_i,
    input  logic        uart_pready_i,
    input  logic        uart_pslverr_i,
    // PCIe PHY pads (loopback by default; pcie_phy_stub configurable)
    output logic [31:0] pcie_phy_pad_tx_tdata_o,
    output logic [3:0]  pcie_phy_pad_tx_tkeep_o,
    output logic        pcie_phy_pad_tx_tvalid_o,
    output logic        pcie_phy_pad_tx_tlast_o,
    output logic [2:0]  pcie_phy_pad_tx_tuser_o,
    input  logic        pcie_phy_pad_tx_tready_i,
    input  logic [31:0] pcie_phy_pad_rx_tdata_i,
    input  logic [3:0]  pcie_phy_pad_rx_tkeep_i,
    input  logic        pcie_phy_pad_rx_tvalid_i,
    input  logic        pcie_phy_pad_rx_tlast_i,
    input  logic [2:0]  pcie_phy_pad_rx_tuser_i,
    output logic        pcie_phy_pad_rx_tready_o,
    // Chip-level PCIe interrupt
    output logic        pcie_irq_o,
    // SoC status
    output logic [63:0] exit_o,
    output logic [63:0] hw_cnt_en_o
);

  `include "axi/assign.svh"
  `include "axi/typedef.svh"

  /////////////////////////
  //   AXI typedefs       //
  /////////////////////////

  // System-wide types (wide). Match ara_soc's internal `system_*` family.
  localparam int unsigned AxiSocIdWidth = AxiIdWidth - $clog2(2);

  typedef logic [AxiAddrWidth-1:0]      axi_addr_t;
  typedef logic [AxiDataWidth-1:0]      axi_data_t;
  typedef logic [AxiDataWidth/8-1:0]    axi_strb_t;
  typedef logic [AxiIdWidth-1:0]        axi_id_t;
  typedef logic [AxiSocIdWidth-1:0]     axi_soc_id_t;
  typedef logic [AxiUserWidth-1:0]      axi_user_t;

  // SoC slave-port AXI ("system" - wide bus, AxiSocIdWidth ID). PCIe master
  // emits requests of this type after the dwc.
  `AXI_TYPEDEF_ALL(system, axi_addr_t, axi_soc_id_t, axi_data_t, axi_strb_t, axi_user_t)

  // PCIe bridge narrow side: 64-bit data, AxiSocIdWidth ID, then widened by
  // axi_dw_converter to AxiDataWidth before entering the SoC.
  typedef logic [PcieBridgeAxiDataWidth-1:0]   pcie_n_data_t;
  typedef logic [PcieBridgeAxiDataWidth/8-1:0] pcie_n_strb_t;
  `AXI_TYPEDEF_ALL(pcie_narrow, axi_addr_t, axi_soc_id_t, pcie_n_data_t, pcie_n_strb_t, axi_user_t)

  /////////////////////////
  //  Internal signals   //
  /////////////////////////

  // PHY-stub <-> DLL
  logic [31:0] dll_phy_rx_tdata,  dll_phy_tx_tdata;
  logic [3:0]  dll_phy_rx_tkeep,  dll_phy_tx_tkeep;
  logic        dll_phy_rx_tvalid, dll_phy_tx_tvalid;
  logic        dll_phy_rx_tlast,  dll_phy_tx_tlast;
  logic [2:0]  dll_phy_rx_tuser,  dll_phy_tx_tuser;
  logic        dll_phy_rx_tready, dll_phy_tx_tready;
  logic        phy_link_up, phy_idle_valid;

  // DLL <-> bridge TLP stream
  logic [31:0] dll_tlp_rx_tdata,  dll_tlp_tx_tdata;
  logic [3:0]  dll_tlp_rx_tkeep,  dll_tlp_tx_tkeep;
  logic        dll_tlp_rx_tvalid, dll_tlp_tx_tvalid;
  logic        dll_tlp_rx_tlast,  dll_tlp_tx_tlast;
  logic [2:0]  dll_tlp_rx_tuser,  dll_tlp_tx_tuser;
  logic        dll_tlp_rx_tready, dll_tlp_tx_tready;

  // DLL captured config
  logic        fc_initialized;
  logic [7:0]  cfg_bus_number;
  logic [4:0]  cfg_device_number;
  logic [2:0]  cfg_function_number;
  logic        ext_tag_enable;
  logic        rcb_128b;
  logic [2:0]  max_read_request_size;
  logic [2:0]  max_payload_size;
  logic        msix_enable, msix_mask;

  // Bridge AXI (narrow)
  pcie_narrow_req_t  pcie_narrow_req;
  pcie_narrow_resp_t pcie_narrow_resp;

  // After dw_converter (wide system AXI)
  system_req_t  pcie_wide_req;
  system_resp_t pcie_wide_resp;

  // Bridge <-> CSR
  logic        bridge_enable;
  logic [63:0] bar_base, bar_mask, axi_target_base;
  logic [31:0] ur_count, ca_count, tlp_err_count, outstanding_count;
  logic        bridge_error_event;
  logic        pcie_boot_hold;

  /////////////////////////
  //  PHY stub           //
  /////////////////////////

  pcie_phy_stub #(
    .DATA_WIDTH    (32),
    .USER_WIDTH    (3),
    .LOOPBACK_MODE (1'b0),
    .LINKUP_DELAY  (16)
  ) i_pcie_phy_stub (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .dll_phy_rx_tdata_o (dll_phy_rx_tdata),
    .dll_phy_rx_tkeep_o (dll_phy_rx_tkeep),
    .dll_phy_rx_tvalid_o(dll_phy_rx_tvalid),
    .dll_phy_rx_tlast_o (dll_phy_rx_tlast),
    .dll_phy_rx_tuser_o (dll_phy_rx_tuser),
    .dll_phy_rx_tready_i(dll_phy_rx_tready),
    .dll_phy_tx_tdata_i (dll_phy_tx_tdata),
    .dll_phy_tx_tkeep_i (dll_phy_tx_tkeep),
    .dll_phy_tx_tvalid_i(dll_phy_tx_tvalid),
    .dll_phy_tx_tlast_i (dll_phy_tx_tlast),
    .dll_phy_tx_tuser_i (dll_phy_tx_tuser),
    .dll_phy_tx_tready_o(dll_phy_tx_tready),
    .pad_tx_tdata_o     (pcie_phy_pad_tx_tdata_o),
    .pad_tx_tkeep_o     (pcie_phy_pad_tx_tkeep_o),
    .pad_tx_tvalid_o    (pcie_phy_pad_tx_tvalid_o),
    .pad_tx_tlast_o     (pcie_phy_pad_tx_tlast_o),
    .pad_tx_tuser_o     (pcie_phy_pad_tx_tuser_o),
    .pad_tx_tready_i    (pcie_phy_pad_tx_tready_i),
    .pad_rx_tdata_i     (pcie_phy_pad_rx_tdata_i),
    .pad_rx_tkeep_i     (pcie_phy_pad_rx_tkeep_i),
    .pad_rx_tvalid_i    (pcie_phy_pad_rx_tvalid_i),
    .pad_rx_tlast_i     (pcie_phy_pad_rx_tlast_i),
    .pad_rx_tuser_i     (pcie_phy_pad_rx_tuser_i),
    .pad_rx_tready_o    (pcie_phy_pad_rx_tready_o),
    .phy_link_up_o      (phy_link_up),
    .idle_valid_o       (phy_idle_valid)
  );

  /////////////////////////
  //  PCIe DLL           //
  /////////////////////////

  pcie_datalink_layer #(
    .DATA_WIDTH       (32),
    .USER_WIDTH       (3),
    .S_COUNT          (2),
    .RX_FIFO_SIZE     (3),
    .RETRY_TLP_SIZE   (3),
    .MAX_PAYLOAD_SIZE (256)
  ) i_pcie_dll (
    .clk_i                  (clk_i),
    .rst_i                  (~rst_ni),
    // TLP stream from bridge into DLL (DLL slave port, going to link)
    .s_tlp_axis_tdata       (dll_tlp_tx_tdata),
    .s_tlp_axis_tkeep       (dll_tlp_tx_tkeep),
    .s_tlp_axis_tvalid      (dll_tlp_tx_tvalid),
    .s_tlp_axis_tlast       (dll_tlp_tx_tlast),
    .s_tlp_axis_tuser       (dll_tlp_tx_tuser),
    .s_tlp_axis_tready      (dll_tlp_tx_tready),
    // TLP stream from DLL out to bridge (received from link)
    .m_tlp_axis_tdata       (dll_tlp_rx_tdata),
    .m_tlp_axis_tkeep       (dll_tlp_rx_tkeep),
    .m_tlp_axis_tvalid      (dll_tlp_rx_tvalid),
    .m_tlp_axis_tlast       (dll_tlp_rx_tlast),
    .m_tlp_axis_tuser       (dll_tlp_rx_tuser),
    .m_tlp_axis_tready      (dll_tlp_rx_tready),
    // PHY stream from stub into DLL
    .s_phy_axis_tdata       (dll_phy_rx_tdata),
    .s_phy_axis_tkeep       (dll_phy_rx_tkeep),
    .s_phy_axis_tvalid      (dll_phy_rx_tvalid),
    .s_phy_axis_tlast       (dll_phy_rx_tlast),
    .s_phy_axis_tuser       (dll_phy_rx_tuser),
    .s_phy_axis_tready      (dll_phy_rx_tready),
    // PHY stream from DLL out to stub
    .m_phy_axis_tdata       (dll_phy_tx_tdata),
    .m_phy_axis_tkeep       (dll_phy_tx_tkeep),
    .m_phy_axis_tvalid      (dll_phy_tx_tvalid),
    .m_phy_axis_tlast       (dll_phy_tx_tlast),
    .m_phy_axis_tuser       (dll_phy_tx_tuser),
    .m_phy_axis_tready      (dll_phy_tx_tready),
    .phy_link_up_i          (phy_link_up),
    .fc_initialized_o       (fc_initialized),
    .idle_valid_i           (phy_idle_valid),
    .cfg_bus_number_o       (cfg_bus_number),
    .cfg_device_number_o    (cfg_device_number),
    .cfg_function_number_o  (cfg_function_number),
    .ext_tag_enable_o       (ext_tag_enable),
    .rcb_128b_o             (rcb_128b),
    .max_read_request_size_o(max_read_request_size),
    .max_payload_size_o     (max_payload_size),
    .msix_enable_o          (msix_enable),
    .msix_mask_o            (msix_mask),
    .status_error_cor_i     (1'b0),
    .status_error_uncor_i   (1'b0),
    .rx_cpl_stall_i         (1'b0)
  );

  /////////////////////////
  //  TLP -> AXI bridge  //
  /////////////////////////

  pcie_tlp_axi_bridge #(
    .TLP_DATA_WIDTH (32),
    .TLP_USER_WIDTH (3),
    .AxiAddrWidth   (AxiAddrWidth),
    .AxiDataWidth   (PcieBridgeAxiDataWidth),
    .AxiIdWidth     (AxiSocIdWidth),
    .axi_req_t      (pcie_narrow_req_t),
    .axi_resp_t     (pcie_narrow_resp_t)
  ) i_pcie_bridge (
    .clk_i               (clk_i),
    .rst_ni              (rst_ni),
    .s_tlp_tdata_i       (dll_tlp_rx_tdata),
    .s_tlp_tkeep_i       (dll_tlp_rx_tkeep),
    .s_tlp_tvalid_i      (dll_tlp_rx_tvalid),
    .s_tlp_tlast_i       (dll_tlp_rx_tlast),
    .s_tlp_tuser_i       (dll_tlp_rx_tuser),
    .s_tlp_tready_o      (dll_tlp_rx_tready),
    .m_tlp_tdata_o       (dll_tlp_tx_tdata),
    .m_tlp_tkeep_o       (dll_tlp_tx_tkeep),
    .m_tlp_tvalid_o      (dll_tlp_tx_tvalid),
    .m_tlp_tlast_o       (dll_tlp_tx_tlast),
    .m_tlp_tuser_o       (dll_tlp_tx_tuser),
    .m_tlp_tready_i      (dll_tlp_tx_tready),
    .m_axi_req_o         (pcie_narrow_req),
    .m_axi_resp_i        (pcie_narrow_resp),
    .cfg_bus_number_i    (cfg_bus_number),
    .cfg_device_number_i (cfg_device_number),
    .cfg_function_number_i(cfg_function_number),
    .bridge_enable_i     (bridge_enable),
    .bar_base_i          (bar_base),
    .bar_mask_i          (bar_mask),
    .axi_target_base_i   (axi_target_base),
    .ur_count_o          (ur_count),
    .ca_count_o          (ca_count),
    .tlp_err_count_o     (tlp_err_count),
    .outstanding_count_o (outstanding_count),
    .error_event_o       (bridge_error_event)
  );

  /////////////////////////////////////
  //  AXI dw_converter (narrow→wide) //
  /////////////////////////////////////

  axi_dw_converter #(
    .AxiSlvPortDataWidth(PcieBridgeAxiDataWidth),
    .AxiMstPortDataWidth(AxiDataWidth         ),
    .AxiAddrWidth       (AxiAddrWidth         ),
    .AxiIdWidth         (AxiSocIdWidth        ),
    .AxiMaxReads        (2                    ),
    .ar_chan_t          (pcie_narrow_ar_chan_t),
    .mst_r_chan_t       (system_r_chan_t      ),
    .slv_r_chan_t       (pcie_narrow_r_chan_t ),
    .aw_chan_t          (pcie_narrow_aw_chan_t),
    .b_chan_t           (pcie_narrow_b_chan_t ),
    .mst_w_chan_t       (system_w_chan_t      ),
    .slv_w_chan_t       (pcie_narrow_w_chan_t ),
    .axi_mst_req_t      (system_req_t         ),
    .axi_mst_resp_t     (system_resp_t        ),
    .axi_slv_req_t      (pcie_narrow_req_t    ),
    .axi_slv_resp_t     (pcie_narrow_resp_t   )
  ) i_pcie_axi_dwc (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .slv_req_i (pcie_narrow_req),
    .slv_resp_o(pcie_narrow_resp),
    .mst_req_o (pcie_wide_req),
    .mst_resp_i(pcie_wide_resp)
  );

  /////////////////////////
  //  ara_soc            //
  /////////////////////////

  ara_soc #(
    .NrLanes             (NrLanes             ),
    .NrClusters          (NrClusters          ),
    .FPUSupport          (FPUSupport          ),
    .FPExtSupport        (FPExtSupport        ),
    .FixPtSupport        (FixPtSupport        ),
    .AxiDataWidth        (AxiDataWidth        ),
    .ClusterAxiDataWidth (ClusterAxiDataWidth ),
    .AxiAddrWidth        (AxiAddrWidth        ),
    .AxiUserWidth        (AxiUserWidth        ),
    .AxiIdWidth          (AxiIdWidth          ),
    .AxiRespDelay        (AxiRespDelay        ),
    .L2NumWords          (L2NumWords          ),
    .pcie_axi_req_t      (system_req_t        ),
    .pcie_axi_resp_t     (system_resp_t       )
  ) i_ara_soc (
    .clk_i                          (clk_i),
    .rst_ni                         (rst_ni),
    .exit_o                         (exit_o),
    .hw_cnt_en_o                    (hw_cnt_en_o),
    .scan_enable_i                  (scan_enable_i),
    .scan_data_i                    (scan_data_i),
    .scan_data_o                    (scan_data_o),
    .uart_penable_o                 (uart_penable_o),
    .uart_pwrite_o                  (uart_pwrite_o),
    .uart_paddr_o                   (uart_paddr_o),
    .uart_psel_o                    (uart_psel_o),
    .uart_pwdata_o                  (uart_pwdata_o),
    .uart_prdata_i                  (uart_prdata_i),
    .uart_pready_i                  (uart_pready_i),
    .uart_pslverr_i                 (uart_pslverr_i),
    .pcie_axi_req_i                 (pcie_wide_req),
    .pcie_axi_resp_o                (pcie_wide_resp),
    .pcie_phy_link_up_i             (phy_link_up),
    .pcie_fc_initialized_i          (fc_initialized),
    .pcie_cfg_bus_number_i          (cfg_bus_number),
    .pcie_cfg_device_number_i       (cfg_device_number),
    .pcie_cfg_function_number_i     (cfg_function_number),
    .pcie_max_payload_size_i        (max_payload_size),
    .pcie_max_read_request_size_i   (max_read_request_size),
    .pcie_rcb_128b_i                (rcb_128b),
    .pcie_ur_count_i                (ur_count),
    .pcie_ca_count_i                (ca_count),
    .pcie_tlp_err_count_i           (tlp_err_count),
    .pcie_outstanding_count_i       (outstanding_count),
    .pcie_bridge_error_event_i      (bridge_error_event),
	    .pcie_bridge_enable_o           (bridge_enable),
	    .pcie_bar_base_o                (bar_base),
	    .pcie_bar_mask_o                (bar_mask),
	    .pcie_axi_target_base_o         (axi_target_base),
	    .pcie_boot_hold_o               (pcie_boot_hold),
	    .pcie_irq_o                     (pcie_irq_o)
	  );

  // Tie off unused DLL outputs so lint stays quiet.
  logic _unused_ok;
  assign _unused_ok = |{ext_tag_enable, msix_enable, msix_mask, phy_idle_valid};

endmodule
