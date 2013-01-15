`timescale 1ns/1ns

module config_net_tb;

  reg clk_i, enable;
  reg bit_i;
  wire clk_o0;
  wire [3:0] config_data;

  config_node     #(.id_width_p(4),
                    .info_width_p(5),
                    .id_p(1),
                    .config_bits_p(8),
                    .default_p(1) )
    config_node_dut(.clk_i(clk_i),
                    .enable(enable),
                    .bit_i(bit_i),
                    .clk_o(clk_o0),
                    .config_o(config_data) );
  initial begin
    clk_i = 1;
    enable = 0;
    bit_i = 1;
    #10 enable = 1;
  end

  always #5
    clk_i = !clk_i; // flip clock every 5 ns, cycle 10 ns

  initial begin
    $dumpfile( "config_net_tb.vcd" );
    $dumpvars;
  end

  initial
  #300 $finish; // ends at 100 ns

endmodule
