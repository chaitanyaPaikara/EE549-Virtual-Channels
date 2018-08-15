/**
 *  bsg_repl.v
 */

module bsg_repl
  #(parameter lg_sets_lp="inv"
    ,parameter sets_p="inv")
(
  input clock_i
  ,input reset_i
  
  ,input [lg_sets_lp-1:0] index_v_i

  ,input [lg_sets_lp-1:0] index_tl_i
  ,input miss_minus_recover_v_i
  ,input tagged_access_v_i

  ,input ld_st_set_v_i  // used for load or store
  ,input wipe_set_v_i   // for flush or invalidate

  ,input ld_op_v_i
  ,input st_op_v_i
  ,input wipe_v_i       // for flush or invalidate
  
  ,input status_mem_re_i

  // outputs
  ,output logic dirty0_o
  ,output logic dirty1_o
  ,output logic mru_o
  ,output logic write_over_read_v_o  
);

  logic replacement_we;
  logic [2:0] replacement_data_in;
  logic [2:0] replacement_mask;
  
  logic [lg_sets_lp-1:0] line_final;
  logic read_dirty_r; 

  assign replacement_data_in = wipe_v_i
    ? {2'b00, ~wipe_set_v_i}
    : {2'b11, ld_st_set_v_i};

  assign replacement_we = wipe_v_i | st_op_v_i | ld_op_v_i;

  assign line_final = (replacement_we | miss_minus_recover_v_i)
    ? index_v_i
    : index_tl_i;
  
  always_ff @ (posedge clock_i) begin
    read_dirty_r <= ~(replacement_we | miss_minus_recover_v_i);
  end

  assign write_over_read_v_o = st_op_v_i & tagged_access_v_i
    & (~read_dirty_r | ((~ld_st_set_v_i & ~dirty0_o) | (ld_st_set_v_i & ~dirty1_o)));

  assign replacement_mask = {
    (wipe_v_i ? ~wipe_set_v_i : (st_op_v_i ? ~ld_st_set_v_i : 1'b0)),
    (wipe_v_i ? wipe_set_v_i : (st_op_v_i ? ld_st_set_v_i : 1'b0)),
    (wipe_v_i | st_op_v_i | ld_op_v_i)
  };

  bsg_mem_1rw_sync_mask_write_bit #(.width_p(3), .els_p(sets_p)) status_mem (
    .clk_i(clock_i)
    ,.reset_i(reset_i)
    ,.v_i(~reset_i & (status_mem_re_i | replacement_we))
    ,.w_i(~reset_i & replacement_we)
    ,.w_mask_i(replacement_mask)
    ,.addr_i(line_final)
    ,.data_i(replacement_data_in)
    ,.data_o({dirty0_o, dirty1_o, mru_o})
  );

endmodule