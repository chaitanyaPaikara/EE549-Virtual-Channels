
# Workspace directory
dir = ../../../..

export BSG_CADENV_DIR = $(dir)/bsg_cadenv
export BASEJUMP_STL_DIR = $(dir)/basejump_stl

# Environment
# include $(BSG_CADENV_DIR)/cadenv.mk

run:
	$(VCS_BIN)/vcs -full64 -sverilog -timescale=1ps/1ps -f ./filelist -debug_pp -R -top bsg_wormhole_network_tester +vcs+vcdpluson
    
view:
	-dve -full64 -vpd vcdplus.vpd &

junk = csrc DVEfiles simv.daidir *.old *.vpd simv *.key

clean:
	rm -rf $(junk)

# vcs:
# 	-vcs -f ./filelist +v2k -R +lint=all -sverilog -full64 \
# 	-debug_pp -timescale=1ns/10ps -l trial.log -top bsg_wormhole_network_tester +vcs+vcdpluson

vcs:
	-vcs -full64 -sverilog -timescale=1ps/1ps -f ./filelist -debug_pp -R -top bsg_wormhole_network_tester +vcs+vcdpluson

# verilator.build:
# 	verilator --top-module testbench --Wno-fatal --Wno-lint --Wno-style --Wno-widthconcat --Wno-unoptflat --sc -f flist.verilator -CFLAGS -std=c++11
# 	(cd obj_dir; make -j -f Vtestbench.mk Vtestbench__ALL.a)
# 	(cd obj_dir; make -j -f Vtestbench.mk ../test_bp.o verilated.o)
# 	(cd obj_dir; g++ -L$(SYSTEMC_LIBDIR) ../test_bp.o Vtestbench__ALL*.o verilated.o -o Vtestbench -lsystemc)

# verilator.run:
# 	./obj_dir/Vtestbench