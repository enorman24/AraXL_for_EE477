// Simple host-side PCIe partner for the AraXL real-DLL testbench.
//
// The TB provides plain 32-bit TLP AXI-Stream words on s_tlp_*.  This module
// wraps those TLPs with a second pcie_datalink_layer instance, then connects
// the host DLL PHY stream to the device DLL PHY stream.  That keeps the host
// model compact while using the same RTL to generate flow-control DLLPs,
// sequence numbers, LCRC, ACK/NAK, and retry-visible PHY packets.

`timescale 1ns / 1ps

module pcie_host_partner_model
  import pcie_datalink_pkg::*;
#(
  parameter int DATA_WIDTH       = 32,
  parameter int KEEP_WIDTH       = DATA_WIDTH / 8,
  parameter int USER_WIDTH       = 3,
  parameter int S_COUNT          = 2,
  parameter int RX_FIFO_SIZE     = 3,
  parameter int RETRY_TLP_SIZE   = 3,
  parameter int MAX_PAYLOAD_SIZE = 256,
  parameter int LINKUP_DELAY     = 16
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,

  // Plain host-originated TLP stream from the top-level TB.
  input  logic [DATA_WIDTH-1:0] s_tlp_tdata_i,
  input  logic [KEEP_WIDTH-1:0] s_tlp_tkeep_i,
  input  logic                  s_tlp_tvalid_i,
  input  logic                  s_tlp_tlast_i,
  output logic                  s_tlp_tready_o,

  // Host TX -> device RX PHY stream.
  output logic [DATA_WIDTH-1:0] device_phy_rx_tdata_o,
  output logic [KEEP_WIDTH-1:0] device_phy_rx_tkeep_o,
  output logic                  device_phy_rx_tvalid_o,
  output logic                  device_phy_rx_tlast_o,
  output logic [USER_WIDTH-1:0] device_phy_rx_tuser_o,
  input  logic                  device_phy_rx_tready_i,

  // Device TX -> host RX PHY stream.
  input  logic [DATA_WIDTH-1:0] device_phy_tx_tdata_i,
  input  logic [KEEP_WIDTH-1:0] device_phy_tx_tkeep_i,
  input  logic                  device_phy_tx_tvalid_i,
  input  logic                  device_phy_tx_tlast_i,
  input  logic [USER_WIDTH-1:0] device_phy_tx_tuser_i,
  output logic                  device_phy_tx_tready_o,

  output logic                  link_up_o,
  output logic                  host_fc_initialized_o,
  output int unsigned           host_rx_tlp_count_o,
  output int unsigned           host_phy_packet_count_o
);

  // -----------------------------------------------------------------------
  // Link-up model
  // -----------------------------------------------------------------------
  logic [$clog2(LINKUP_DELAY+1)-1:0] linkup_cnt_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      linkup_cnt_q <= '0;
    end else if (linkup_cnt_q != LINKUP_DELAY[$clog2(LINKUP_DELAY+1)-1:0]) begin
      linkup_cnt_q <= linkup_cnt_q + 1'b1;
    end
  end

  assign link_up_o = (linkup_cnt_q == LINKUP_DELAY[$clog2(LINKUP_DELAY+1)-1:0]);

  // -----------------------------------------------------------------------
  // Host DLL
  // -----------------------------------------------------------------------
  logic [DATA_WIDTH-1:0] host_s_phy_tdata;
  logic [KEEP_WIDTH-1:0] host_s_phy_tkeep;
  logic                  host_s_phy_tvalid;
  logic                  host_s_phy_tlast;
  logic [USER_WIDTH-1:0] host_s_phy_tuser;
  logic                  host_s_phy_tready;

  logic [DATA_WIDTH-1:0] host_m_phy_tdata;
  logic [KEEP_WIDTH-1:0] host_m_phy_tkeep;
  logic                  host_m_phy_tvalid;
  logic                  host_m_phy_tlast;
  logic [USER_WIDTH-1:0] host_m_phy_tuser;
  logic                  host_m_phy_tready;

  logic [DATA_WIDTH-1:0] host_rx_tlp_tdata;
  logic [KEEP_WIDTH-1:0] host_rx_tlp_tkeep;
  logic                  host_rx_tlp_tvalid;
  logic                  host_rx_tlp_tlast;
  logic [USER_WIDTH-1:0] host_rx_tlp_tuser;

  logic [7:0] cfg_bus_number;
  logic [4:0] cfg_device_number;
  logic [2:0] cfg_function_number;
  logic       ext_tag_enable;
  logic       rcb_128b;
  logic [2:0] max_read_request_size;
  logic [2:0] max_payload_size;
  logic       msix_enable;
  logic       msix_mask;

  // Both real DLLs need one received feature-exchange DLLP to start their FC
  // init sequence.  This bootstrap source emits that packet once, then the
  // host DLL owns the PHY streams for normal FC/TLP traffic.
  localparam logic [USER_WIDTH-1:0] DLLP_USER = {{(USER_WIDTH-1){1'b0}}, 1'b1};

  typedef enum logic [1:0] {
    BOOT_IDLE,
    BOOT_DATA,
    BOOT_CRC,
    BOOT_DONE
  } boot_state_e;

  boot_state_e boot_state_q, boot_state_d;
  logic        boot_dev_sent_q,  boot_dev_sent_d;
  logic        boot_host_sent_q, boot_host_sent_d;

  logic [DATA_WIDTH-1:0] feature_exchange_word;
  logic [15:0]           feature_exchange_crc;
  logic [DATA_WIDTH-1:0] feature_exchange_crc_word;

  assign feature_exchange_word     = {{(DATA_WIDTH-8){1'b0}}, Feature_Exchange};
  assign feature_exchange_crc_word = {{(DATA_WIDTH-16){1'b0}}, ~feature_exchange_crc};

  pcie_datalink_crc i_feature_exchange_crc (
    .crcIn (16'hFFFF             ),
    .data  (feature_exchange_word),
    .crcOut(feature_exchange_crc )
  );

  always_comb begin
    boot_state_d     = boot_state_q;
    boot_dev_sent_d  = boot_dev_sent_q;
    boot_host_sent_d = boot_host_sent_q;

    unique case (boot_state_q)
      BOOT_IDLE: begin
        boot_dev_sent_d  = 1'b0;
        boot_host_sent_d = 1'b0;
        if (link_up_o)
          boot_state_d = BOOT_DATA;
      end
      BOOT_DATA: begin
        if (!boot_dev_sent_q && device_phy_rx_tready_i)
          boot_dev_sent_d = 1'b1;
        if (!boot_host_sent_q && host_s_phy_tready)
          boot_host_sent_d = 1'b1;
        if (boot_dev_sent_d && boot_host_sent_d) begin
          boot_dev_sent_d  = 1'b0;
          boot_host_sent_d = 1'b0;
          boot_state_d     = BOOT_CRC;
        end
      end
      BOOT_CRC: begin
        if (!boot_dev_sent_q && device_phy_rx_tready_i)
          boot_dev_sent_d = 1'b1;
        if (!boot_host_sent_q && host_s_phy_tready)
          boot_host_sent_d = 1'b1;
        if (boot_dev_sent_d && boot_host_sent_d)
          boot_state_d = BOOT_DONE;
      end
      BOOT_DONE: begin
        boot_dev_sent_d  = 1'b1;
        boot_host_sent_d = 1'b1;
      end
      default: begin
        boot_state_d = BOOT_IDLE;
      end
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      boot_state_q     <= BOOT_IDLE;
      boot_dev_sent_q  <= 1'b0;
      boot_host_sent_q <= 1'b0;
    end else begin
      boot_state_q     <= boot_state_d;
      boot_dev_sent_q  <= boot_dev_sent_d;
      boot_host_sent_q <= boot_host_sent_d;
    end
  end

  always_comb begin
    device_phy_rx_tdata_o  = host_m_phy_tdata;
    device_phy_rx_tkeep_o  = host_m_phy_tkeep;
    device_phy_rx_tvalid_o = host_m_phy_tvalid;
    device_phy_rx_tlast_o  = host_m_phy_tlast;
    device_phy_rx_tuser_o  = host_m_phy_tuser;
    host_m_phy_tready      = device_phy_rx_tready_i;

    host_s_phy_tdata       = device_phy_tx_tdata_i;
    host_s_phy_tkeep       = device_phy_tx_tkeep_i;
    host_s_phy_tvalid      = device_phy_tx_tvalid_i & link_up_o;
    host_s_phy_tlast       = device_phy_tx_tlast_i;
    host_s_phy_tuser       = device_phy_tx_tuser_i;
    device_phy_tx_tready_o = host_s_phy_tready;

    if (boot_state_q == BOOT_DATA || boot_state_q == BOOT_CRC) begin
      if (!boot_dev_sent_q) begin
        device_phy_rx_tdata_o  = (boot_state_q == BOOT_DATA) ? feature_exchange_word : feature_exchange_crc_word;
        device_phy_rx_tkeep_o  = (boot_state_q == BOOT_DATA) ? '1 : {{(KEEP_WIDTH-2){1'b0}}, 2'b11};
        device_phy_rx_tvalid_o = 1'b1;
        device_phy_rx_tlast_o  = (boot_state_q == BOOT_CRC);
        device_phy_rx_tuser_o  = DLLP_USER;
        host_m_phy_tready      = 1'b0;
      end

      if (!boot_host_sent_q) begin
        host_s_phy_tdata       = (boot_state_q == BOOT_DATA) ? feature_exchange_word : feature_exchange_crc_word;
        host_s_phy_tkeep       = (boot_state_q == BOOT_DATA) ? '1 : {{(KEEP_WIDTH-2){1'b0}}, 2'b11};
        host_s_phy_tvalid      = 1'b1;
        host_s_phy_tlast       = (boot_state_q == BOOT_CRC);
        host_s_phy_tuser       = DLLP_USER;
        device_phy_tx_tready_o = 1'b0;
      end
    end
  end

  pcie_datalink_layer #(
    .DATA_WIDTH       (DATA_WIDTH      ),
    .USER_WIDTH       (USER_WIDTH      ),
    .S_COUNT          (S_COUNT         ),
    .RX_FIFO_SIZE     (RX_FIFO_SIZE    ),
    .RETRY_TLP_SIZE   (RETRY_TLP_SIZE  ),
    .MAX_PAYLOAD_SIZE (MAX_PAYLOAD_SIZE)
  ) i_host_dll (
    .clk_i                  (clk_i                    ),
    .rst_i                  (~rst_ni                  ),
    .s_tlp_axis_tdata       (s_tlp_tdata_i            ),
    .s_tlp_axis_tkeep       (s_tlp_tkeep_i            ),
    .s_tlp_axis_tvalid      (s_tlp_tvalid_i           ),
    .s_tlp_axis_tlast       (s_tlp_tlast_i            ),
    .s_tlp_axis_tuser       (3'b010                   ),
    .s_tlp_axis_tready      (s_tlp_tready_o           ),
    .m_tlp_axis_tdata       (host_rx_tlp_tdata        ),
    .m_tlp_axis_tkeep       (host_rx_tlp_tkeep        ),
    .m_tlp_axis_tvalid      (host_rx_tlp_tvalid       ),
    .m_tlp_axis_tlast       (host_rx_tlp_tlast        ),
    .m_tlp_axis_tuser       (host_rx_tlp_tuser        ),
    .m_tlp_axis_tready      (1'b1                     ),
    .s_phy_axis_tdata       (host_s_phy_tdata         ),
    .s_phy_axis_tkeep       (host_s_phy_tkeep         ),
    .s_phy_axis_tvalid      (host_s_phy_tvalid        ),
    .s_phy_axis_tlast       (host_s_phy_tlast         ),
    .s_phy_axis_tuser       (host_s_phy_tuser         ),
    .s_phy_axis_tready      (host_s_phy_tready        ),
    .m_phy_axis_tdata       (host_m_phy_tdata         ),
    .m_phy_axis_tkeep       (host_m_phy_tkeep         ),
    .m_phy_axis_tvalid      (host_m_phy_tvalid        ),
    .m_phy_axis_tlast       (host_m_phy_tlast         ),
    .m_phy_axis_tuser       (host_m_phy_tuser         ),
    .m_phy_axis_tready      (host_m_phy_tready        ),
    .phy_link_up_i          (link_up_o                ),
    .fc_initialized_o       (host_fc_initialized_o    ),
    .idle_valid_i           (link_up_o                ),
    .cfg_bus_number_o       (cfg_bus_number           ),
    .cfg_device_number_o    (cfg_device_number        ),
    .cfg_function_number_o  (cfg_function_number      ),
    .ext_tag_enable_o       (ext_tag_enable           ),
    .rcb_128b_o             (rcb_128b                 ),
    .max_read_request_size_o(max_read_request_size    ),
    .max_payload_size_o     (max_payload_size         ),
    .msix_enable_o          (msix_enable              ),
    .msix_mask_o            (msix_mask                ),
    .status_error_cor_i     (1'b0                     ),
    .status_error_uncor_i   (1'b0                     ),
    .rx_cpl_stall_i         (1'b0                     )
  );

  // -----------------------------------------------------------------------
  // Lightweight observability counters
  // -----------------------------------------------------------------------
  int unsigned host_rx_tlp_count_q;
  int unsigned host_phy_packet_count_q;

  assign host_rx_tlp_count_o     = host_rx_tlp_count_q;
  assign host_phy_packet_count_o = host_phy_packet_count_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      host_rx_tlp_count_q     <= 0;
      host_phy_packet_count_q <= 0;
    end else begin
      if (host_rx_tlp_tvalid && host_rx_tlp_tlast)
        host_rx_tlp_count_q <= host_rx_tlp_count_q + 1;
      if (device_phy_rx_tvalid_o && device_phy_rx_tready_i && device_phy_rx_tlast_o)
        host_phy_packet_count_q <= host_phy_packet_count_q + 1;
    end
  end

  logic _unused_ok;
  assign _unused_ok = |{host_rx_tlp_tdata, host_rx_tlp_tkeep, host_rx_tlp_tuser,
                        cfg_bus_number, cfg_device_number, cfg_function_number,
                        ext_tag_enable, rcb_128b, max_read_request_size,
                        max_payload_size, msix_enable, msix_mask};

endmodule : pcie_host_partner_model
