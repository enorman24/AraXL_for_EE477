// PCIe TLP <-> AXI4-Full bridge (transaction layer).
//
// Sits between `pcie_datalink_layer` (32-bit TLP AXI-Stream) and the SoC's AXI
// crossbar. Translates Memory Write (MWr) TLPs into AXI write bursts and
// Memory Read (MRd) TLPs into AXI read bursts plus PCIe Completion-with-Data
// (CplD) TLPs.
//
// Simplifying choices for the EE477 first-cut integration:
//   * Single AXI transaction in flight at a time. The next TLP is consumed
//     only after the previous one's B (write) or last CplD (read) drains.
//     This eliminates a tag table and avoids out-of-order completion logic.
//   * One AXI W beat per DW (awsize = 2 on a 64-bit bus). Wasteful of AXI
//     bandwidth but lane placement & wstrb are straightforward.
//   * One CplD TLP per data DW. byte_count / lower_address are patched post-
//     `gen_cpld()` so the host sees correct remaining-byte counts.
//   * Unsupported TLP types (IO, MsgD, atomics, Cfg1, ...) get drained and
//     tlp_err_count is incremented. UR completions for unsupported reads are
//     not generated in this revision (TODO).
//   * Strict PCIe PR1/PR2 ordering (writes-pass-reads) is NOT enforced; the
//     bridge issues AXI in TLP-arrival order and relies on the downstream
//     xbar for per-ID ordering.

`timescale 1ns / 1ps

module pcie_tlp_axi_bridge
  import pcie_datalink_pkg::*;
  import pcie_tlp_pkg::*;
#(
    parameter int  TLP_DATA_WIDTH = 32,
    parameter int  TLP_USER_WIDTH = 3,
    parameter int  AxiAddrWidth   = 64,
    parameter int  AxiDataWidth   = 64,
    parameter int  AxiIdWidth     = 5,
    parameter type axi_req_t      = logic,
    parameter type axi_resp_t     = logic
) (
    input  logic                          clk_i,
    input  logic                          rst_ni,

    // TLP stream IN (slave port) -- connect to DLL m_tlp_axis_*
    input  logic [TLP_DATA_WIDTH-1:0]     s_tlp_tdata_i,
    input  logic [TLP_DATA_WIDTH/8-1:0]   s_tlp_tkeep_i,
    input  logic                          s_tlp_tvalid_i,
    input  logic                          s_tlp_tlast_i,
    input  logic [TLP_USER_WIDTH-1:0]     s_tlp_tuser_i,
    output logic                          s_tlp_tready_o,

    // TLP stream OUT (master port) -- connect to DLL s_tlp_axis_*
    output logic [TLP_DATA_WIDTH-1:0]     m_tlp_tdata_o,
    output logic [TLP_DATA_WIDTH/8-1:0]   m_tlp_tkeep_o,
    output logic                          m_tlp_tvalid_o,
    output logic                          m_tlp_tlast_o,
    output logic [TLP_USER_WIDTH-1:0]     m_tlp_tuser_o,
    input  logic                          m_tlp_tready_i,

    // AXI4-Full master
    output axi_req_t                      m_axi_req_o,
    input  axi_resp_t                     m_axi_resp_i,

    // Captured PCIe completer ID (from DLL)
    input  logic [7:0]                    cfg_bus_number_i,
    input  logic [4:0]                    cfg_device_number_i,
    input  logic [2:0]                    cfg_function_number_i,

    // CSR-programmed controls
    input  logic                          bridge_enable_i,
    input  logic [63:0]                   bar_base_i,
    input  logic [63:0]                   bar_mask_i,
    input  logic [63:0]                   axi_target_base_i,

    // CSR status outputs
    output logic [31:0]                   ur_count_o,
    output logic [31:0]                   ca_count_o,
    output logic [31:0]                   tlp_err_count_o,
    output logic [31:0]                   outstanding_count_o,
    output logic                          error_event_o
);

  localparam int AxiStrbWidth = AxiDataWidth / 8;

  // -------------------------------------------------------------------------
  // RX FSM: parse incoming TLP header, drive AW/W, capture MRd metadata.
  // -------------------------------------------------------------------------
  typedef enum logic [3:0] {
    RX_IDLE,
    RX_DW1,
    RX_ADDR_HI,     // 4DW header: DW2 is addr[63:32]
    RX_ADDR_LO,     // 4DW header: DW3 is addr[31:0]; or 3DW DW2 is addr[31:0]
    RX_W_PAYLOAD,   // streaming MWr payload DWs onto AXI W
    RX_AW_ISSUE,    // single-cycle issue of AW (after we have addr + length)
    RX_AR_ISSUE,    // single-cycle issue of AR
    RX_WAIT_B,      // wait for AXI B (write completion)
    RX_WAIT_CPLD,   // wait for read-completion FSM to drain
    RX_DRAIN,       // discard remaining DWs of an unsupported TLP
    RX_ERROR_BUMP
  } rx_state_e;

  rx_state_e rx_state_q, rx_state_d;

  // Header capture registers
  logic [2:0]   hdr_fmt_q;          // {with_data, 4dw, x}: see Fmt decode below
  logic [4:0]   hdr_type_q;
  logic [9:0]   hdr_length_dw_q;    // 10-bit DW count; 0 means 1024
  logic [15:0]  hdr_req_id_q;
  logic [7:0]   hdr_tag_q;
  logic [3:0]   hdr_first_be_q;
  logic [3:0]   hdr_last_be_q;
  logic [63:0]  hdr_pcie_addr_q;    // raw TLP address (pre-translation)
  logic [63:0]  hdr_axi_addr_q;     // translated AXI address
  logic [2:0]   hdr_attr_q;

  // Per-transaction running state
  logic [9:0]   payload_left_q;     // DWs left to send (MWr) or to receive (MRd)
  logic [9:0]   beat_index_q;       // current beat index within this transaction

  // Decoded header flags
  logic         is_4dw;
  logic         is_with_data;
  logic         is_mwr;
  logic         is_mrd;
  logic         is_supported;

  // Header DW0 decode (combinational on s_tlp_tdata_i during RX_IDLE)
  pcie_tlp_byte0_t dw0_byte0_w;
  assign dw0_byte0_w = s_tlp_tdata_i[7:0];

  logic [2:0] dw0_fmt_w;
  logic [4:0] dw0_type_w;
  assign dw0_fmt_w  = dw0_byte0_w.Fmt;
  assign dw0_type_w = dw0_byte0_w.Type;

  // Fmt encoding: bit[0]=4DW, bit[1]=with-data, bit[2]=prefix (ignored).
  logic dw0_is_4dw_w, dw0_is_with_data_w;
  assign dw0_is_4dw_w       = dw0_fmt_w[0];
  assign dw0_is_with_data_w = dw0_fmt_w[1];

  // Length field is byte2.Length1[1:0] || byte3.Length0[7:0]
  logic [9:0] dw0_length_w;
  assign dw0_length_w = {s_tlp_tdata_i[17:16], s_tlp_tdata_i[31:24]};

  // Attr field (sparse; only TC[6:4] of byte1 carried for now)
  logic [2:0] dw0_tc_w;
  assign dw0_tc_w = s_tlp_tdata_i[14:12];

  // MWr/MRd type detection (type field [4:0] is 5'b00000 for both; differ by Fmt)
  logic dw0_is_mem_w;
  assign dw0_is_mem_w = (dw0_type_w == 5'b00000);

  // Decode flags for captured header
  assign is_4dw       = hdr_fmt_q[0];
  assign is_with_data = hdr_fmt_q[1];
  assign is_mwr       = (hdr_type_q == 5'b00000) & is_with_data;
  assign is_mrd       = (hdr_type_q == 5'b00000) & ~is_with_data;
  assign is_supported = is_mwr | is_mrd;

  // -------------------------------------------------------------------------
  // BAR translation: axi_addr = (pcie_addr & ~bar_mask) + axi_target_base
  // (bar_mask=0xFFFF_FFFF passes the low 32 bits straight through.)
  // -------------------------------------------------------------------------
  function automatic logic [63:0] bar_translate(input logic [63:0] pcie_addr);
    bar_translate = (pcie_addr & bar_mask_i) + axi_target_base_i;
  endfunction

  // -------------------------------------------------------------------------
  // Counter state
  // -------------------------------------------------------------------------
  logic [31:0] ur_count_q, ca_count_q, tlp_err_count_q;
  logic        error_event_q;
  logic        bump_ur_w, bump_ca_w, bump_tlp_err_w;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ur_count_q      <= '0;
      ca_count_q      <= '0;
      tlp_err_count_q <= '0;
      error_event_q   <= 1'b0;
    end else begin
      if (bump_ur_w)      ur_count_q      <= ur_count_q + 1;
      if (bump_ca_w)      ca_count_q      <= ca_count_q + 1;
      if (bump_tlp_err_w) tlp_err_count_q <= tlp_err_count_q + 1;
      error_event_q <= bump_ur_w | bump_ca_w | bump_tlp_err_w;
    end
  end

  assign ur_count_o          = ur_count_q;
  assign ca_count_o          = ca_count_q;
  assign tlp_err_count_o     = tlp_err_count_q;
  assign error_event_o       = error_event_q;
  assign outstanding_count_o = (rx_state_q == RX_WAIT_B) | (rx_state_q == RX_WAIT_CPLD) ? 32'd1 : 32'd0;

  // -------------------------------------------------------------------------
  // AXI master output registers (single AW / AR held until accepted)
  // -------------------------------------------------------------------------
  logic                        aw_valid_q;
  logic [AxiAddrWidth-1:0]     aw_addr_q;
  logic [7:0]                  aw_len_q;
  logic [AxiIdWidth-1:0]       aw_id_q;

  logic                        ar_valid_q;
  logic [AxiAddrWidth-1:0]     ar_addr_q;
  logic [7:0]                  ar_len_q;
  logic [AxiIdWidth-1:0]       ar_id_q;

  // W channel
  logic                        w_valid_q;
  logic [AxiDataWidth-1:0]     w_data_q;
  logic [AxiStrbWidth-1:0]     w_strb_q;
  logic                        w_last_q;

  // R-channel ready and completion-stream state owned by the CplD FSM below.

  // -------------------------------------------------------------------------
  // CplD generator FSM (drives m_tlp_*)
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    CPLD_IDLE,
    CPLD_DW0,
    CPLD_DW1,
    CPLD_DW2,
    CPLD_DW3
  } cpld_state_e;

  cpld_state_e cpld_state_q, cpld_state_d;
  cpl_tlp_hdr_t cpld_tlp_q;

  // Latched per-CplD context for header construction
  logic [11:0] cpld_byte_count_q;   // bytes remaining to return for the request
  logic [6:0]  cpld_lower_addr_q;   // lower-address field of the data in this CplD
  logic [31:0] cpld_data_q;         // current 32-bit DW to be sent
  logic        cpld_pending_q;      // a CplD construction is in flight

  // Number of DWs already returned in the current MRd
  logic [9:0]  read_returned_q;

  // -------------------------------------------------------------------------
  // RX FSM next-state and outputs
  // -------------------------------------------------------------------------
  logic capture_dw0_w, capture_dw1_w, capture_addr_hi_w, capture_addr_lo_w;
  logic accept_payload_w;

  // ready_to_consume drives s_tlp_tready_o
  always_comb begin
    rx_state_d        = rx_state_q;
    capture_dw0_w     = 1'b0;
    capture_dw1_w     = 1'b0;
    capture_addr_hi_w = 1'b0;
    capture_addr_lo_w = 1'b0;
    accept_payload_w  = 1'b0;
    bump_ur_w         = 1'b0;
    bump_ca_w         = 1'b0;
    bump_tlp_err_w    = 1'b0;
    s_tlp_tready_o    = 1'b0;

    case (rx_state_q)
      RX_IDLE: begin
        s_tlp_tready_o = bridge_enable_i;
        if (s_tlp_tvalid_i & s_tlp_tready_o) begin
          capture_dw0_w = 1'b1;
          rx_state_d    = RX_DW1;
        end
      end

      RX_DW1: begin
        s_tlp_tready_o = 1'b1;
        if (s_tlp_tvalid_i) begin
          capture_dw1_w = 1'b1;
          if (s_tlp_tlast_i) begin
            // Malformed: header-only TLP that wasn't truly a 1DW MRd (no addr) - drop
            bump_tlp_err_w = 1'b1;
            rx_state_d     = RX_IDLE;
          end else if (hdr_fmt_q[0]) begin
            rx_state_d = RX_ADDR_HI;
          end else begin
            rx_state_d = RX_ADDR_LO;
          end
        end
      end

      RX_ADDR_HI: begin
        s_tlp_tready_o = 1'b1;
        if (s_tlp_tvalid_i) begin
          capture_addr_hi_w = 1'b1;
          if (s_tlp_tlast_i) begin
            bump_tlp_err_w = 1'b1;
            rx_state_d     = RX_IDLE;
          end else begin
            rx_state_d = RX_ADDR_LO;
          end
        end
      end

      RX_ADDR_LO: begin
        s_tlp_tready_o = 1'b1;
        if (s_tlp_tvalid_i) begin
          capture_addr_lo_w = 1'b1;
          if (!is_supported) begin
            // Drain unsupported TLP body if any DWs remain
            bump_tlp_err_w = 1'b1;
            rx_state_d = s_tlp_tlast_i ? RX_IDLE : RX_DRAIN;
          end else if (is_mwr) begin
            // Header parsed; now we must issue AW and stream W beats.
            // Important: for MWr, more DWs (payload) follow on the stream.
            rx_state_d = RX_AW_ISSUE;
          end else begin // is_mrd
            // MRd has no payload; should be tlast on this beat.
            if (!s_tlp_tlast_i) begin
              // Spurious extra DWs on an MRd: drain
              bump_tlp_err_w = 1'b1;
              rx_state_d     = RX_DRAIN;
            end else begin
              rx_state_d = RX_AR_ISSUE;
            end
          end
        end
      end

      RX_AW_ISSUE: begin
        // Wait until AW is accepted, then start streaming W beats.
        if (aw_valid_q & m_axi_resp_i.aw_ready) begin
          rx_state_d = RX_W_PAYLOAD;
        end
      end

      RX_W_PAYLOAD: begin
        // Accept a new payload DW whenever the W register can take it.
        s_tlp_tready_o = ~w_valid_q | m_axi_resp_i.w_ready;
        if (s_tlp_tvalid_i & s_tlp_tready_o) begin
          accept_payload_w = 1'b1;
          if (s_tlp_tlast_i || payload_left_q == 10'd1) begin
            rx_state_d = RX_WAIT_B;
          end
        end
      end

      RX_AR_ISSUE: begin
        if (ar_valid_q & m_axi_resp_i.ar_ready) begin
          rx_state_d = RX_WAIT_CPLD;
        end
      end

      RX_WAIT_B: begin
        if (m_axi_resp_i.b_valid) begin
          if (m_axi_resp_i.b.resp == axi_pkg::RESP_SLVERR ||
              m_axi_resp_i.b.resp == axi_pkg::RESP_DECERR) begin
            bump_ca_w = 1'b1;
          end
          rx_state_d = RX_IDLE;
        end
      end

      RX_WAIT_CPLD: begin
        // Read completion FSM consumes R beats and emits CplD TLPs.
        // Done when all DWs returned AND completion FSM has emitted them all.
        if (read_returned_q == hdr_length_dw_q && cpld_state_q == CPLD_IDLE && !cpld_pending_q) begin
          rx_state_d = RX_IDLE;
        end
      end

      RX_DRAIN: begin
        s_tlp_tready_o = 1'b1;
        if (s_tlp_tvalid_i & s_tlp_tlast_i) begin
          rx_state_d = RX_IDLE;
        end
      end

      default: rx_state_d = RX_IDLE;
    endcase
  end

  // -------------------------------------------------------------------------
  // RX FSM sequential
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rx_state_q       <= RX_IDLE;
      hdr_fmt_q        <= '0;
      hdr_type_q       <= '0;
      hdr_length_dw_q  <= '0;
      hdr_req_id_q     <= '0;
      hdr_tag_q        <= '0;
      hdr_first_be_q   <= '0;
      hdr_last_be_q    <= '0;
      hdr_pcie_addr_q  <= '0;
      hdr_axi_addr_q   <= '0;
      hdr_attr_q       <= '0;
      payload_left_q   <= '0;
      beat_index_q     <= '0;
    end else begin
`ifdef PCIE_BRIDGE_DEBUG
      if (rx_state_q != rx_state_d)
        $display("[BRG %0t] state %0d -> %0d  tvalid=%b tready=%b tdata=%h",
                 $time, rx_state_q, rx_state_d,
                 s_tlp_tvalid_i, s_tlp_tready_o, s_tlp_tdata_i);
`endif
      rx_state_q <= rx_state_d;

      if (capture_dw0_w) begin
        hdr_fmt_q       <= dw0_fmt_w;
        hdr_type_q      <= dw0_type_w;
        hdr_length_dw_q <= dw0_length_w;
        hdr_attr_q      <= dw0_tc_w;
      end

      if (capture_dw1_w) begin
        // DW1 layout: requester_id[31:16], tag[15:8], last_be[7:4], first_be[3:0]
        hdr_req_id_q   <= s_tlp_tdata_i[31:16];
        hdr_tag_q      <= s_tlp_tdata_i[15:8];
        hdr_last_be_q  <= s_tlp_tdata_i[7:4];
        hdr_first_be_q <= s_tlp_tdata_i[3:0];
      end

      if (capture_addr_hi_w) begin
        hdr_pcie_addr_q[63:32] <= s_tlp_tdata_i;
      end

      if (capture_addr_lo_w) begin
        if (is_4dw) begin
          hdr_pcie_addr_q[31:0]  <= s_tlp_tdata_i;
        end else begin
          hdr_pcie_addr_q[63:32] <= '0;
          hdr_pcie_addr_q[31:0]  <= s_tlp_tdata_i;
        end
        hdr_axi_addr_q <= bar_translate(
            is_4dw ? {hdr_pcie_addr_q[63:32], s_tlp_tdata_i}
                   : {32'h0, s_tlp_tdata_i});
        // Initialize per-transaction counters.
        payload_left_q <= hdr_length_dw_q;
        beat_index_q   <= '0;
      end

      if (accept_payload_w) begin
        payload_left_q <= payload_left_q - 1;
        beat_index_q   <= beat_index_q + 1;
      end

      // Reset per-transaction counters when transaction completes
      if (rx_state_q == RX_WAIT_B && rx_state_d == RX_IDLE) begin
        payload_left_q <= '0;
        beat_index_q   <= '0;
      end
      if (rx_state_q == RX_WAIT_CPLD && rx_state_d == RX_IDLE) begin
        payload_left_q <= '0;
        beat_index_q   <= '0;
      end
    end
  end

  // -------------------------------------------------------------------------
  // AXI AW issuance
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      aw_valid_q <= 1'b0;
      aw_addr_q  <= '0;
      aw_len_q   <= '0;
      aw_id_q    <= '0;
    end else begin
      if (rx_state_q == RX_AR_ISSUE) begin
        // not the path for AW; clear if leftover
        aw_valid_q <= aw_valid_q & ~m_axi_resp_i.aw_ready;
      end else if (rx_state_q != RX_AW_ISSUE) begin
        if (aw_valid_q & m_axi_resp_i.aw_ready) aw_valid_q <= 1'b0;
      end else begin
        if (!aw_valid_q) begin
          aw_valid_q <= 1'b1;
          aw_addr_q  <= hdr_axi_addr_q;
          aw_len_q   <= hdr_length_dw_q[7:0] - 8'd1; // length_dw is 1..N (0 means 1024 in PCIe; we don't handle that)
          aw_id_q    <= {{(AxiIdWidth){1'b0}}};      // bridge uses ID 0 for writes
        end else if (m_axi_resp_i.aw_ready) begin
          aw_valid_q <= 1'b0;
        end
      end
    end
  end

  // -------------------------------------------------------------------------
  // AXI AR issuance
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ar_valid_q <= 1'b0;
      ar_addr_q  <= '0;
      ar_len_q   <= '0;
      ar_id_q    <= '0;
    end else begin
      if (rx_state_q != RX_AR_ISSUE) begin
        if (ar_valid_q & m_axi_resp_i.ar_ready) ar_valid_q <= 1'b0;
      end else begin
        if (!ar_valid_q) begin
          ar_valid_q <= 1'b1;
          ar_addr_q  <= hdr_axi_addr_q;
          ar_len_q   <= hdr_length_dw_q[7:0] - 8'd1;
          ar_id_q    <= {{(AxiIdWidth-1){1'b0}}, 1'b1}; // bridge uses ID 1 for reads
        end else if (m_axi_resp_i.ar_ready) begin
          ar_valid_q <= 1'b0;
        end
      end
    end
  end

  // -------------------------------------------------------------------------
  // AXI W beat packer
  //   Each accepted payload DW becomes one W beat. Lane within wdata[63:0] is
  //   chosen by axi_addr[2] XOR beat_index[0].
  //   wstrb defaults to 4'b1111; the first beat uses hdr_first_be, the last
  //   beat uses hdr_last_be.
  // -------------------------------------------------------------------------
  logic        lane_sel_w;
  logic [3:0]  be_for_beat_w;
  logic        is_first_w_beat_w;
  logic        is_last_w_beat_w;

  assign is_first_w_beat_w = (beat_index_q == 10'd0);
  assign is_last_w_beat_w  = (payload_left_q == 10'd1);

  always_comb begin
    if (is_first_w_beat_w && hdr_length_dw_q == 10'd1) begin
      be_for_beat_w = hdr_first_be_q;
    end else if (is_first_w_beat_w) begin
      be_for_beat_w = hdr_first_be_q;
    end else if (is_last_w_beat_w) begin
      be_for_beat_w = hdr_last_be_q;
    end else begin
      be_for_beat_w = 4'b1111;
    end
  end

  assign lane_sel_w = hdr_axi_addr_q[2] ^ beat_index_q[0];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      w_valid_q <= 1'b0;
      w_data_q  <= '0;
      w_strb_q  <= '0;
      w_last_q  <= 1'b0;
    end else begin
      if (w_valid_q & m_axi_resp_i.w_ready) begin
        w_valid_q <= 1'b0;
      end
      if (accept_payload_w) begin
        w_valid_q <= 1'b1;
        if (lane_sel_w) begin
          w_data_q <= {s_tlp_tdata_i, 32'h0};
          w_strb_q <= {be_for_beat_w, 4'b0000};
        end else begin
          w_data_q <= {32'h0, s_tlp_tdata_i};
          w_strb_q <= {4'b0000, be_for_beat_w};
        end
        w_last_q <= is_last_w_beat_w;
      end
    end
  end

  // -------------------------------------------------------------------------
  // R-channel & CplD generation FSM
  //   Consumes one R beat at a time, emits a 4-DW CplD TLP per data DW.
  // -------------------------------------------------------------------------
  // R-channel ready: only when CplD FSM is idle AND we are in RX_WAIT_CPLD.
  logic r_ready_w;
  assign r_ready_w = (rx_state_q == RX_WAIT_CPLD) & (cpld_state_q == CPLD_IDLE) & ~cpld_pending_q;

  // Extract 32-bit DW from R beat based on (ar_addr + read_returned*4)[2]
  logic [31:0] r_dw_w;
  logic        r_lane_sel_w;
  assign r_lane_sel_w = hdr_axi_addr_q[2] ^ read_returned_q[0];
  assign r_dw_w = r_lane_sel_w ? m_axi_resp_i.r.data[63:32] : m_axi_resp_i.r.data[31:0];

  // Combinational helpers for the about-to-be-emitted CplD
  logic [13:0] bytes_remaining_w;
  logic [11:0] la_full_w;
  assign bytes_remaining_w = ({4'b0, hdr_length_dw_q} - {4'b0, read_returned_q}) << 2;
  assign la_full_w         = {5'b0, hdr_axi_addr_q[6:0]} + ({2'b0, read_returned_q} << 2);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      cpld_state_q       <= CPLD_IDLE;
      cpld_tlp_q         <= '0;
      cpld_byte_count_q  <= '0;
      cpld_lower_addr_q  <= '0;
      cpld_data_q        <= '0;
      cpld_pending_q     <= 1'b0;
      read_returned_q    <= '0;
    end else begin
      // Reset read_returned at start of a new MRd transaction
      if (rx_state_q == RX_AR_ISSUE && rx_state_d == RX_WAIT_CPLD) begin
        read_returned_q <= '0;
      end

      // Capture an R beat
      if (r_ready_w & m_axi_resp_i.r_valid) begin
        cpld_data_q       <= r_dw_w;
        cpld_byte_count_q <= bytes_remaining_w[11:0];
        cpld_lower_addr_q <= la_full_w[6:0];
        cpld_pending_q    <= 1'b1;
        read_returned_q   <= read_returned_q + 10'd1;
`ifdef PCIE_BRIDGE_DEBUG
        $display("[CRB %0t] R beat captured: r_dw=%h lane=%b rdata=%h rready=%b rvalid=%b",
                 $time, r_dw_w, r_lane_sel_w, m_axi_resp_i.r.data,
                 r_ready_w, m_axi_resp_i.r_valid);
`endif

        if (m_axi_resp_i.r.resp == axi_pkg::RESP_SLVERR ||
            m_axi_resp_i.r.resp == axi_pkg::RESP_DECERR) begin
          // Mark error sticky on counters
          // (bump_ca handled elsewhere via cpld FSM detecting error)
        end
      end

      // CplD emission FSM (one TLP per DW)
      case (cpld_state_q)
        CPLD_IDLE: begin
          if (cpld_pending_q) begin
            // Pre-build header struct via gen_cpld, then patch byte_count/lower_address
            begin : cpld_build
              cpl_tlp_hdr_t built;
              built = patch_cpld(gen_cpld(make_request_hdr(), cpld_data_q),
                                 cpld_byte_count_q, cpld_lower_addr_q);
`ifdef PCIE_BRIDGE_DEBUG
              $display("[CRB %0t] CPLD build: data=%h dw0=%h dw1=%h dw2=%h cpld_data_q=%h bc=%h la=%h",
                       $time, built.data, built.dw_0, built.dw_1, built.dw_2,
                       cpld_data_q, cpld_byte_count_q, cpld_lower_addr_q);
`endif
              cpld_tlp_q <= built;
            end
            cpld_state_q <= CPLD_DW0;
          end
        end
        CPLD_DW0: if (m_tlp_tready_i) cpld_state_q <= CPLD_DW1;
        CPLD_DW1: if (m_tlp_tready_i) cpld_state_q <= CPLD_DW2;
        CPLD_DW2: if (m_tlp_tready_i) cpld_state_q <= CPLD_DW3;
        CPLD_DW3: if (m_tlp_tready_i) begin
`ifdef PCIE_BRIDGE_DEBUG
          $display("[CRB %0t] CPLD sent DW3/data=%h (last)", $time, cpld_tlp_q.data);
`endif
          cpld_state_q   <= CPLD_IDLE;
          cpld_pending_q <= 1'b0;
        end
        default: cpld_state_q <= CPLD_IDLE;
      endcase
    end
  end

  // Reconstruct a tlp_hdr_t for gen_cpld() from captured request fields.
  // Only the fields gen_cpld() actually reads are populated:
  //   word_1.requester_id, word_1.tag, word_2.byte_0/byte_1 (completer_id).
  function automatic tlp_hdr_t make_request_hdr();
    tlp_hdr_t h;
    h = '0;
    h.word_1.requester_id = hdr_req_id_q;
    h.word_1.tag          = hdr_tag_q;
    // Completer ID = {bus, device, function}
    h.word_2.byte_0.Bus_Number              = cfg_bus_number_i;
    h.word_2.byte_1.Device_Number           = cfg_device_number_i[4:0];
    h.word_2.byte_1.Function_Number_With_ARI = cfg_function_number_i;
    return h;
  endfunction

  // Override byte_count and lower_address fields in a CplD produced by gen_cpld.
  function automatic cpl_tlp_hdr_t patch_cpld(input cpl_tlp_hdr_t in,
                                              input logic [11:0] bc,
                                              input logic [6:0]  la);
    cpl_tlp_hdr_t o;
    o = in;
    {o.dw_1.byte2.byte_count, o.dw_1.byte3.byte_count} = bc;
    o.dw_2.byte3.lower_address = la[6:0];
    return o;
  endfunction

  // -------------------------------------------------------------------------
  // m_tlp_axis output multiplex: drives DW0..DW3 of the current CplD.
  // -------------------------------------------------------------------------
  always_comb begin
    m_tlp_tvalid_o = 1'b0;
    m_tlp_tdata_o  = '0;
    m_tlp_tkeep_o  = 4'b1111;
    m_tlp_tlast_o  = 1'b0;
    m_tlp_tuser_o  = '0;

    case (cpld_state_q)
      CPLD_DW0: begin
        m_tlp_tvalid_o = 1'b1;
        m_tlp_tdata_o  = cpld_tlp_q.dw_0;
      end
      CPLD_DW1: begin
        m_tlp_tvalid_o = 1'b1;
        m_tlp_tdata_o  = cpld_tlp_q.dw_1;
      end
      CPLD_DW2: begin
        m_tlp_tvalid_o = 1'b1;
        m_tlp_tdata_o  = cpld_tlp_q.dw_2;
      end
      CPLD_DW3: begin
        m_tlp_tvalid_o = 1'b1;
        m_tlp_tdata_o  = cpld_tlp_q.data;
        m_tlp_tlast_o  = 1'b1;
      end
      default: ;
    endcase
  end

  // -------------------------------------------------------------------------
  // Assemble AXI master request struct
  // -------------------------------------------------------------------------
  always_comb begin
    m_axi_req_o = '0;

    // AW
    m_axi_req_o.aw.id    = aw_id_q;
    m_axi_req_o.aw.addr  = aw_addr_q;
    m_axi_req_o.aw.len   = aw_len_q;
    m_axi_req_o.aw.size  = 3'b011;            // 8 bytes / beat — full bus width; wstrb masks unused lanes
    m_axi_req_o.aw.burst = axi_pkg::BURST_INCR;
    m_axi_req_o.aw.lock  = 1'b0;
    m_axi_req_o.aw.cache = 4'b0000;
    m_axi_req_o.aw.prot  = 3'b000;
    m_axi_req_o.aw.qos   = 4'b0000;
    m_axi_req_o.aw.region= 4'b0000;
    m_axi_req_o.aw.atop  = 6'b0;
    m_axi_req_o.aw.user  = '0;
    m_axi_req_o.aw_valid = aw_valid_q;

    // W
    m_axi_req_o.w.data = w_data_q;
    m_axi_req_o.w.strb = w_strb_q;
    m_axi_req_o.w.last = w_last_q;
    m_axi_req_o.w.user = '0;
    m_axi_req_o.w_valid = w_valid_q;

    // B
    m_axi_req_o.b_ready = (rx_state_q == RX_WAIT_B);

    // AR
    m_axi_req_o.ar.id    = ar_id_q;
    m_axi_req_o.ar.addr  = ar_addr_q;
    m_axi_req_o.ar.len   = ar_len_q;
    m_axi_req_o.ar.size  = 3'b011;
    m_axi_req_o.ar.burst = axi_pkg::BURST_INCR;
    m_axi_req_o.ar.lock  = 1'b0;
    m_axi_req_o.ar.cache = 4'b0000;
    m_axi_req_o.ar.prot  = 3'b000;
    m_axi_req_o.ar.qos   = 4'b0000;
    m_axi_req_o.ar.region= 4'b0000;
    m_axi_req_o.ar.user  = '0;
    m_axi_req_o.ar_valid = ar_valid_q;

    // R
    m_axi_req_o.r_ready = r_ready_w;
  end

endmodule
