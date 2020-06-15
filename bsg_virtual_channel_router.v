`include "bsg_defines.v"
`include "bsg_noc_links.vh"
`include "bsg_wormhole_router.vh"

module bsg_virtual_channel_router
	import bsg_wormhole_router_pkg::StrictXY;
	import bsg_wormhole_router_pkg::StrictX;
	#(parameter flit_width_p        = "inv"
 	,parameter dims_p              = 2 // 1
 	,parameter dirs_lp         = dims_p*2+1

	 // this list determines the range of bit values used to determine each dimension in the N-D router
	 // cord_dims_p is normally the same as dims_p.  However, the override allows users to pass
	 // a larger cord array than necessary, useful for parameterizing between 1d/nd networks
	,parameter cord_dims_p = dims_p
	,parameter int cord_markers_pos_p[cord_dims_p:0] =   '{ 5, 4, 0 }  // '{5,0} //
	,parameter bit [1:0][dirs_lp-1:0][dirs_lp-1:0] routing_matrix_p =  (dims_p == 2) ? StrictXY : StrictX
	,parameter reverse_order_p       = 0
	,parameter len_width_p           = "inv"
	,parameter debug_lp              = 0
	,parameter vc_num                = 2 
	)
	   
	(input clk_i
	 ,input reset_i

	// Traffics
	,input  [dirs_lp-1:0][`bsg_ready_and_link_vc_sif_width(flit_width_p, vc_num)-1:0] link_i
	,output [dirs_lp-1:0][`bsg_ready_and_link_vc_sif_width(flit_width_p, vc_num)-1:0] link_o

	// My coordinate data
	,input [cord_markers_pos_p[dims_p]-1:0] my_cord_i
	);
	
	genvar j, k;//, m, n;

	`declare_bsg_ready_and_link_vc_sif_s(flit_width_p,bsg_ready_and_link_vc_sif_s, vc_num);
	bsg_ready_and_link_vc_sif_s [dirs_lp-1:0] link_i_cast, link_o_cast;

	assign link_i_cast = link_i;
	assign link_o = link_o_cast;

	`declare_bsg_ready_and_link_sif_s(flit_width_p,bsg_ready_and_link_sif_s);
	bsg_ready_and_link_sif_s [vc_num-1:0][dirs_lp-1:0] wormhole_in_li;
	bsg_ready_and_link_sif_s [vc_num-1:0][dirs_lp-1:0] wormhole_out_lo;

	logic [dirs_lp-1:0][vc_num-1:0] yumi_from_rr;

	for (j = 0; j < vc_num; j++) begin: each_vc
		// instantiates [dirs_lp*vc_num] fifos at input
		for (k = 0; k < dirs_lp; k++) begin: each_direction
			bsg_two_fifo #(.width_p(flit_width_p)) input_fifo
			    (.clk_i  (clk_i)
			    ,.reset_i(reset_i)
			    ,.ready_o(link_o_cast[k].ready_and_rev[j])
			    ,.data_i (link_i_cast[k].data)
			    ,.v_i    (link_i_cast[k].v[j])
			    ,.v_o    (wormhole_in_li[j][k].v)
			    ,.data_o (wormhole_in_li[j][k].data)
			    ,.yumi_i (wormhole_in_li[j][k].v & wormhole_out_lo[j][k].ready_and_rev)// (wormhole_in_li[j][k].ready_and_rev) // CP added -
			    );
			assign wormhole_in_li[j][k].ready_and_rev = yumi_from_rr[k][j];
		end

		// instantiates [vc_num] wormhole routers after fifo
		bsg_wormhole_router #(.flit_width_p     (flit_width_p)
							,.dims_p            (dims_p)
							,.cord_markers_pos_p(cord_markers_pos_p)
							,.routing_matrix_p  (routing_matrix_p)
							,.len_width_p       (len_width_p)
							) wormhole_router
		(.clk_i    (clk_i)
		,.reset_i  (reset_i)
		,.my_cord_i(my_cord_i)
		,.link_i   (wormhole_in_li[j])
		,.link_o   (wormhole_out_lo[j])
		);
	end

	//transpose whormhole's output
	// bsg_ready_and_link_sif_s [dirs_lp-1:0][vc_num-1:0] wormhole_tranpose_out_lo;
	// bsg_transpose #(.width_p  	(dirs_lp)
	// 				,.els_p		(vc_num))
	// 	(.i(wormhole_out_lo)
	// 	,.o(wormhole_tranpose_out_lo));

	// logic [dirs_lp-1:0][vc_num-1:0] wormhole_to_rr_v, wormhole_to_rr_ready_and_rev;
	// logic [vc_num-1:0][flit_width_p-1:0] wormhole_to_rr_data;
	// for (n = 0; n < vc_num; n++) begin
	// 		assign wormhole_to_rr_v[n] = wormhole_tranpose_out_lo[m][n].v;
	// 		assign wormhole_to_rr_ready_and_rev[n] = wormhole_tranpose_out_lo[m][n].ready_and_rev;
	// 		assign wormhole_to_rr_data[n] = wormhole_tranpose_out_lo[m][n].data;
	// 	end

	integer m, n;
	logic [dirs_lp-1:0][vc_num-1:0] wormhole_to_rr_v, link_i_to_rr_ready_and_rev;
	logic [dirs_lp-1:0][vc_num-1:0][flit_width_p-1:0] wormhole_to_rr_data;
	always @(*) begin
		for (m = 0; m < dirs_lp; m++) begin
			for (n = 0; n < vc_num; n++) begin
				wormhole_to_rr_v[m][n] = wormhole_out_lo[n][m].v;
				link_i_to_rr_ready_and_rev[m][n] = link_i_cast[m].ready_and_rev[n];
				wormhole_to_rr_data[m][n] = wormhole_out_lo[n][m].data;
			end
		end
	end

	// instantiates [dirs_lp] round_robin arbiters
	localparam tag_width = `BSG_SAFE_CLOG2(vc_num);
	logic [dirs_lp-1:0][tag_width-1:0] tag;
	logic [dirs_lp-1:0] rr_valid_o;

	genvar i;
	for (i = 0; i < dirs_lp; i++)
	begin: rr
		bsg_round_robin_n_to_1 #(.width_p  (flit_width_p)
                            	,.num_in_p (vc_num)
                            	,.strict_p (0)
                            	) rr
		(.clk_i		(clk_i)
		,.reset_i	(reset_i)
		,.data_i 	(wormhole_to_rr_data[i])
		,.v_i 		(wormhole_to_rr_v[i])
		,.yumi_o	(yumi_from_rr[i])      ////////////////////////////need to connect to wormhole routers' ready_and_rev
		,.v_o 		(rr_valid_o[i])
		,.data_o  	(link_o_cast[i].data)
		,.tag_o 	(tag[i])
		,.yumi_i 	(link_i_to_rr_ready_and_rev[i]));

		bsg_decode_with_v #(.num_out_p(vc_num)) valid_decode
		(.i(tag[i])
		,.v_i(rr_valid_o[i])
		,.o(link_o_cast[i].v));

	end
endmodule