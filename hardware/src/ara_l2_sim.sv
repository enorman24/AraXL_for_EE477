// Copyright 2024 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Behavioral L2 memory for RTL simulation.  Interface matches
// bsg_mem_1rw_sync_mask_write_byte.  Exposes a flat mem[] array so the
// testbench can backdoor-load ELF sections without traversing generate-for
// scopes (VCS Q-2020 cannot index into them with a runtime variable).

module ara_l2_sim #(
  parameter int unsigned els_p               = 0,
  parameter int unsigned data_width_p        = 0,
  parameter int unsigned addr_width_lp       = $clog2(els_p),
  parameter int unsigned write_mask_width_lp = data_width_p >> 3
) (
  input  logic                             clk_i,
  input  logic                             reset_i,
  input  logic                             v_i,
  input  logic                             w_i,
  input  logic [addr_width_lp-1:0]         addr_i,
  input  logic [data_width_p-1:0]          data_i,
  input  logic [write_mask_width_lp-1:0]   write_mask_i,
  output logic [data_width_p-1:0]          data_o
);

  logic [data_width_p-1:0] mem [0:els_p-1];

  // Use always (not always_ff) so the TB can backdoor-write mem[] directly.
  always @(posedge clk_i) begin
    if (v_i) begin
      if (w_i) begin
        for (int b = 0; b < write_mask_width_lp; b++)
          if (write_mask_i[b])
            mem[addr_i][8*b +: 8] <= data_i[8*b +: 8];
      end
      data_o <= mem[addr_i];
    end
  end

endmodule
