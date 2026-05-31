import "DPI-C" function void read_elf (input string filename);
import "DPI-C" function byte get_section (output longint address, output longint len);
import "DPI-C" context function byte read_section(input longint address, inout byte buffer[]);

module ara_pcie_tb;

  timeunit      1ns;
  timeprecision 1ps;

  `ifdef NR_LANES
  localparam NrLanes = `NR_LANES;
  `else
  localparam NrLanes = 0;
  `endif

  `ifdef NR_CLUSTERS
  localparam NrClusters = `NR_CLUSTERS;
  `else
  localparam NrClusters = 0;
  `endif

  localparam ClockPeriod        = 1ns;
  localparam int unsigned AxiRespDelay = 200;

  localparam AxiAddrWidth        = 64;
  localparam AxiWideDataWidth    = 32 * NrLanes * NrClusters;
  localparam ClusterAxiDataWidth = 32 * NrLanes;
  localparam AxiWideBeWidth      = AxiWideDataWidth / 8;
  localparam AxiWideByteOffset   = $clog2(AxiWideBeWidth);

  localparam DRAMAddrBase = 64'h8000_0000;
  localparam DRAMLength   = 64'h4000_0000;
  localparam logic [31:0] BootHoldTlpAddr = 32'h4000_1048;

  localparam logic [15:0] TLP_REQ_ID = 16'h0100;
  localparam logic [3:0]  TLP_ALL_BE = 4'b1111;


  `ifdef VCS
    initial begin
      $fsdbDumpfile("waveform.fsdb");
      $fsdbDumpvars(0, "+struct");
    end
  `endif

  // -----------------------------------------------------------------------
  // Clock / reset
  // -----------------------------------------------------------------------
  logic clk;
  logic rst_n;
  logic loading_done;

  initial begin
    clk          = 1'b0;
    rst_n        = 1'b0;
    loading_done = 1'b0;
    repeat (10) #(ClockPeriod/2) clk = ~clk;
    clk = 1'b0;
    repeat (5) #(ClockPeriod);
    rst_n = 1'b1;
    repeat (5) #(ClockPeriod);
    forever #(ClockPeriod/2) clk = ~clk;
  end




  // -----------------------------------------------------------------------
  // TLP / copy signals
  // -----------------------------------------------------------------------
  logic [31:0] s_tlp_tdata;
  logic [3:0]  s_tlp_tkeep;
  logic        s_tlp_tvalid;
  logic        s_tlp_tlast;
  logic [2:0]  s_tlp_tuser;
  logic        s_tlp_tready;

  logic        pcie_load_ready;
  logic        boot_hold;
  int unsigned beh_write_count;

  // -----------------------------------------------------------------------
  // DUT
  // -----------------------------------------------------------------------
  logic [63:0] exit;
  logic        pcie_irq;

  ara_chip_tb_harness #(
    .NrLanes             (NrLanes            ),
    .NrClusters          (NrClusters         ),
    .AxiDataWidth        (AxiWideDataWidth   ),
    .ClusterAxiDataWidth (ClusterAxiDataWidth),
    .AxiRespDelay        (AxiRespDelay       )
  ) dut (
    .clk_i               (clk               ),
    .rst_ni              (rst_n             ),
    .loading_done_i      (loading_done      ),
    .exit_o              (exit              ),
    .pcie_irq_o          (pcie_irq          ),
    .s_tlp_tdata_i       (s_tlp_tdata       ),
    .s_tlp_tkeep_i       (s_tlp_tkeep       ),
    .s_tlp_tvalid_i      (s_tlp_tvalid      ),
    .s_tlp_tlast_i       (s_tlp_tlast       ),
    .s_tlp_tuser_i       (s_tlp_tuser       ),
    .s_tlp_tready_o      (s_tlp_tready      ),
    .m_tlp_tready_i      (1'b1              ),
    .pcie_load_ready_o   (pcie_load_ready   ),
    .boot_hold_o         (boot_hold         ),
    .beh_write_count_o   (beh_write_count   )
  );

  // -----------------------------------------------------------------------
  // TLP helpers (identical to TB2)
  // -----------------------------------------------------------------------
  int unsigned tready_stall_cycles;

  task automatic drive_tlp_dw(input logic [31:0] data, input logic is_last);
    s_tlp_tdata  = data;
    s_tlp_tkeep  = 4'b1111;
    s_tlp_tvalid = 1'b1;
    s_tlp_tlast  = is_last;
    s_tlp_tuser  = 3'b010;
    if (!s_tlp_tready)
      $display("[TB3] STALL: s_tlp_tready=0 at t=%0t, data=%08h last=%0b", $time, data, is_last);
    tready_stall_cycles = 0;
    while (!s_tlp_tready) begin
      @(posedge clk); #1;
      tready_stall_cycles++;
      if (tready_stall_cycles % 500 == 0)
        $display("[TB3] STALL: still waiting tready, %0d stall cycles, t=%0t", tready_stall_cycles, $time);
    end
    if (tready_stall_cycles > 0)
      $display("[TB3] STALL resolved after %0d cycles, t=%0t", tready_stall_cycles, $time);
    @(posedge clk); #1;
    s_tlp_tvalid = 1'b0;
    s_tlp_tlast  = 1'b0;
  endtask

  task automatic send_mwr_3dw(
    input logic [31:0] tlp_addr,
    input logic [31:0] data,
    input logic [7:0]  tag,
    input logic [3:0]  first_be
  );
    logic [31:0] dw0, dw1;
    dw0 = {8'h01, 8'h00, 8'h00, 8'h40};
    dw1 = {TLP_REQ_ID, tag, 4'b0000, first_be};
    drive_tlp_dw(dw0, 1'b0);
    drive_tlp_dw(dw1, 1'b0);
    drive_tlp_dw(tlp_addr, 1'b0);
    drive_tlp_dw(data,     1'b1);
  endtask

  // -----------------------------------------------------------------------
  // DRAM initialization via TLP (identical flow to TB2)
  // -----------------------------------------------------------------------
  typedef logic [AxiAddrWidth-1:0]     addr_t;
  typedef logic [AxiWideDataWidth-1:0] data_t;

  initial begin : dram_tlp_load
    automatic data_t mem_row;
    byte     buffer [];
    addr_t   address;
    addr_t   length;
    string   binary;
    logic [7:0] tag_ctr;
    int unsigned write_cnt, drain_cycles;

    s_tlp_tdata       = '0;
    s_tlp_tkeep       = 4'b1111;
    s_tlp_tvalid      = 1'b0;
    s_tlp_tlast       = 1'b0;
    s_tlp_tuser       = '0;
    tag_ctr           = 8'h00;
    write_cnt         = 0;

    $display("[TB3] STEP: waiting for posedge rst_n, t=%0t", $time);
    @(posedge rst_n);
    $display("[TB3] STEP: rst_n asserted, waiting 10 clocks, t=%0t", $time);
    repeat (10) @(posedge clk);
    $display("[TB3] STEP: waiting for PCIe load path readiness (host_fc + device_fc init), t=%0t", $time);
    wait(pcie_load_ready);
    $display("[TB3] STEP: PCIe load path ready, t=%0t", $time);

    void'($value$plusargs("PRELOAD=%s", binary));
    if (binary == "") begin
      $error("[TB3] No firmware supplied via +PRELOAD=. Aborting.");
      $finish;
    end

    $display("[TB3] STEP: reading ELF: %s, t=%0t", binary, $time);
    read_elf(binary);
    $display("[TB3] STEP: ELF parsed, starting TLP section load, t=%0t", $time);

    while (get_section(address, length)) begin
      automatic int nwords = (length + AxiWideBeWidth - 1) / AxiWideBeWidth;
      $display("[TB3] STEP: section addr=%h length=%h (%0d wide-words), t=%0t",
               address, length, nwords, $time);
      buffer = new[nwords * AxiWideBeWidth];
      void'(read_section(address, buffer));
      $display("[TB3] STEP: section read into buffer, sending TLPs..., t=%0t", $time);

      for (int w = 0; w < nwords; w++) begin
        mem_row = '0;
        for (int b = 0; b < AxiWideBeWidth; b++)
          mem_row[8*b +: 8] = buffer[w * AxiWideBeWidth + b];

        if (address >= DRAMAddrBase && address < DRAMAddrBase + DRAMLength) begin
          automatic addr_t base_byte = address + (w << AxiWideByteOffset);
          for (int dw = 0; dw < AxiWideBeWidth/4; dw++) begin
            automatic addr_t  byte_addr  = base_byte + (dw * 4);
            automatic addr_t  tlp_offset = byte_addr - DRAMAddrBase;
            automatic logic [31:0] tlp_addr32 = tlp_offset[31:0];
            automatic logic [31:0] dw_data    = mem_row[dw*32 +: 32];
            send_mwr_3dw(tlp_addr32, dw_data, tag_ctr, TLP_ALL_BE);
            tag_ctr   = tag_ctr + 1;
            write_cnt = write_cnt + 1;
            if (write_cnt % 200 == 0)
              $display("[TB3] Load progress: %0d TLPs sent / %0d AXI writes committed, t=%0t",
                       write_cnt, beh_write_count, $time);
          end
        end else begin
          $display("[TB3] Section word at %h outside DRAM window, skipping.", address);
        end
      end
      $display("[TB3] STEP: section done, total writes so far: %0d, t=%0t", write_cnt, $time);
    end

    $display("[TB3] STEP: TLP loading complete: %0d 32-bit writes sent, t=%0t", write_cnt, $time);

    drain_cycles = 100;
    $display("[TB3] STEP: draining %0d cycles..., t=%0t", drain_cycles, $time);
    repeat (drain_cycles) @(posedge clk);
    $display("[TB3] STEP: drain complete, t=%0t", $time);

    $display("[TB3] STEP: releasing BOOT_HOLD through PCIe CSR MWr addr=%08h, t=%0t",
             BootHoldTlpAddr, $time);
    send_mwr_3dw(BootHoldTlpAddr, 32'h0000_0000, tag_ctr, TLP_ALL_BE);
    tag_ctr = tag_ctr + 1;
    $display("[TB3] STEP: waiting for BOOT_HOLD to clear, t=%0t", $time);
    wait(!boot_hold);
    @(posedge clk);
    loading_done = 1'b1;
    $display("[TB3] STEP: BOOT_HOLD=0, execution starting, t=%0t", $time);
  end : dram_tlp_load

  // -----------------------------------------------------------------------
  // Result dump
  // -----------------------------------------------------------------------
  localparam OutResultFile = "../gold_results_pcie_full_host.txt";

  int fd;
  data_t                     ara_w;
  logic [AxiWideBeWidth-1:0] ara_w_strb;
  logic                      ara_w_valid;
  logic                      ara_w_ready;
  logic                      dump_en_mask;

  initial begin
    fd = $fopen(OutResultFile, "w");
    $display("[TB3] Dumping results to %s", OutResultFile);
  end

  assign ara_w       = dut.i_ara_chip.i_ara_soc.i_system.i_ara_cluster.axi_req_o.w.data;
  assign ara_w_strb  = dut.i_ara_chip.i_ara_soc.i_system.i_ara_cluster.axi_req_o.w.strb;
  assign ara_w_valid = dut.i_ara_chip.i_ara_soc.i_system.i_ara_cluster.axi_req_o.w_valid;
  assign ara_w_ready = dut.i_ara_chip.i_ara_soc.i_system.i_ara_cluster.axi_resp_i.w_ready;

  assign dump_en_mask = dut.i_ara_chip.hw_cnt_en_o[0];

  always_ff @(posedge clk)
    if (dump_en_mask)
      if (ara_w_valid && ara_w_ready)
        for (int b = 0; b < AxiWideBeWidth; b++)
          if (ara_w_strb[b])
            $fdisplay(fd, "%0x", ara_w[b*8 +: 8]);

  // -----------------------------------------------------------------------
  // Heartbeat
  // -----------------------------------------------------------------------
  longint unsigned cycle_ctr;
  always @(posedge clk) begin
    if (!boot_hold) begin
      cycle_ctr <= cycle_ctr + 1;
      if ((cycle_ctr % 10000) == 0 && cycle_ctr != 0)
        $display("[TB3] heartbeat: %0d cycles post-release, exit=%0h, t=%0t",
                 cycle_ctr, exit, $time);
    end else begin
      cycle_ctr <= '0;
    end
  end

  // -----------------------------------------------------------------------
  // End-of-simulation
  // -----------------------------------------------------------------------
  always @(posedge clk) begin
    if (exit[0]) begin
      if (exit >> 1) begin
        $warning("[TB3] Core Test *** FAILED *** (tohost = %0d)", (exit >> 1));
      end else begin
        $display("[hw-cycles]: %d",      int'(dut.runtime_buf_q));
        $display("[cva6-d$-stalls]: %d", int'(dut.dcache_stall_buf_q));
        $display("[cva6-i$-stalls]: %d", int'(dut.icache_stall_buf_q));
        $display("[cva6-sb-full]: %d",   int'(dut.sb_full_buf_q));
        $info("[TB3] Core Test *** SUCCESS *** (tohost = %0d)", (exit >> 1));
      end
      $fclose(fd);
      $finish(exit >> 1);
    end
  end

endmodule : ara_pcie_tb
