#
# This file is part of LiteX.
#
# Copyright (c) 2021 Hensoldt Cyber GmbH <www.hensoldt-cyber.com>
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import re
import subprocess

from migen import *

from litex import get_data_mod
from litex.soc.interconnect import axi
from litex.soc.interconnect import wishbone
from litex.soc.cores.cpu import CPU, CPU_GCC_TRIPLE_RISCV64

class Open(Signal): pass

# Variants -----------------------------------------------------------------------------------------

CPU_VARIANTS = ["standard", "full"]

# GCC Flags ----------------------------------------------------------------------------------------

GCC_FLAGS = {
    #                       /-------- Base ISA
    #                       |/------- Hardware Multiply + Divide
    #                       ||/----- Atomics
    #                       |||/---- Compressed ISA
    #                       ||||/--- Single-Precision Floating-Point
    #                       |||||/-- Double-Precision Floating-Point
    #                       imacfd
    "standard": "-march=rv64imac -mabi=lp64 ",
    "full":     "-march=rv64gc   -mabi=lp64 ",
}

# Helpers ------------------------------------------------------------------------------------------

def add_manifest_sources(platform, manifest):
    basedir = get_data_mod("cpu", "scariv").data_location
    with open(os.path.join(basedir, manifest), 'r') as f:
        for l in f:
            res = re.search('(.+)', l)
            if res and not re.match('//', l):
                if re.match('\+incdir\+', l):
                    platform.add_verilog_include_path(os.path.join(basedir, res.group(1)))
                else:
                    platform.add_source(os.path.join(basedir, "src", res.group(1)))

# SCARIV ---------------------------------------------------------------------------------------------

class ScariV(CPU):
    category             = "softcore"
    family               = "riscv"
    name                 = "scariv"
    human_name           = "SCARIV"
    variants             = CPU_VARIANTS
    data_width           = 64
    endianness           = "little"
    gcc_triple           = CPU_GCC_TRIPLE_RISCV64
    linker_output_format = "elf64-littleriscv"
    nop                  = "nop"
    io_regions           = {0x8000_0000: 0x8000_0000} # Origin, Length.

    # GCC Flags.
    @property
    def gcc_flags(self):
        flags = GCC_FLAGS[self.variant]
        flags += "-D__scariv__ "
        #flags += f" -DUART_POLLING"
        return flags

    # Memory Mapping.
    @property
    def mem_map(self):
        return {
            "rom"       : 0x1000_0000,
            "sram"      : 0x2000_0000,
            "main_sram" : 0x4000_0000,
            "csr"       : 0xf000_0000,
        }

    def __init__(self, platform, variant="standard"):
        self.platform     = platform
        self.variant      = variant
        self.reset        = Signal()
        self.interrupt    = Signal(32)
        # Peripheral bus (Connected to main SoC's bus).
        axi_data_if = axi.AXIInterface(data_width=128, address_width=32,       id_width=5)
        axi_fetch_if = axi.AXIInterface(data_width=128, address_width=32, id_width=5)
        self.periph_buses = [axi_data_if, axi_fetch_if]
        # Memory buses (Connected directly to LiteDRAM).
        self.memory_buses = []

        # # #

        # CPU Instance.
        self.cpu_params = dict(
            # Clk / Rst.
            i_i_clk       = ClockSignal("sys"),
            i_i_reset_n       = ~ResetSignal("sys") | self.reset,

            # # Interrupts
            # i_irq_sources = self.interrupt,

            # AXI interface.
	    o_o_l1d_req_valid   = axi_data_if.ar.valid,
	    o_o_l1d_req_addr    = axi_data_if.ar.addr,
	    o_o_l1d_req_tag     = axi_data_if.ar.id,
	    i_i_l1d_req_ready   = axi_data_if.ar.ready,

	    i_i_l1d_resp_valid = axi_data_if.r.valid,
	    i_i_l1d_resp_tag   = axi_data_if.r.id,
	    i_i_l1d_resp_data  = axi_data_if.r.data,
	    o_o_l1d_resp_ready = axi_data_if.r.ready,

            o_o_ic_req_valid = axi_fetch_if.ar.valid,
            i_i_ic_req_ready = axi_fetch_if.ar.ready,
            o_o_ic_req_tag   = axi_fetch_if.ar.id,
            o_o_ic_req_addr  = axi_fetch_if.ar.addr,

	    i_i_ic_resp_valid = axi_fetch_if.r.valid,
	    i_i_ic_resp_tag   = axi_fetch_if.r.id,
	    i_i_ic_resp_data  = axi_fetch_if.r.data,
	    o_o_ic_resp_ready = axi_fetch_if.r.ready,
        )

        self.comb += axi_fetch_if.ar.len.eq(0)
        self.comb += axi_fetch_if.ar.size.eq(4)  # 128
        self.comb += axi_fetch_if.ar.burst.eq(0)

        # Add Verilog sources.
        # TODO: use Flist.cv64a6_imafdc_sv39 and Flist.cv32a6_imac_sv0 instead
        basedir = get_data_mod("cpu", "scariv").data_location
        platform.add_source(os.path.join(basedir, "src", "litex_defines.svh"))
        platform.add_source(os.path.join(basedir, "src", "riscv_common_pkg.sv"))
        platform.add_source(os.path.join(basedir, "src", "riscv_fpu_imafdc_pkg.sv"))
        platform.add_source(os.path.join(basedir, "src", "riscv64_pkg.sv"))
        platform.add_source(os.path.join(basedir, "src", "scariv_tiny_conf_pkg.sv"))
        add_manifest_sources(platform, "src/fpnew.vf")
        add_manifest_sources(platform, "src/filelist.vf")
        platform.add_verilog_include_path(os.path.join(basedir, "src", "fpnew", "src", "common_cells", "include"))
        platform.add_source(os.path.join(basedir, "src", "decoder_inst_cat.sv"))
        platform.add_source(os.path.join(basedir, "src", "decoder_alu_ctrl.sv"))
        platform.add_source(os.path.join(basedir, "src", "decoder_lsu_ctrl.sv"))
        platform.add_source(os.path.join(basedir, "src", "decoder_bru_ctrl.sv"))
        platform.add_source(os.path.join(basedir, "src", "decoder_csu_ctrl.sv"))
        platform.add_source(os.path.join(basedir, "src", "decoder_fpu_ctrl.sv"))
        platform.add_source(os.path.join(basedir, "src", "decoder_reg.sv"))
        platform.add_source(os.path.join(basedir, "src", "pma_map.sv"))

    def add_jtag(self, pads):
        from migen.fhdl.specials import Tristate
        self.jtag_tck  = Signal()
        self.jtag_tms  = Signal()
        self.jtag_trst = Signal()
        self.jtag_tdi  = Signal()
        self.jtag_tdo  = Signal()

        tdo_o  = Signal()
        tdo_oe = Signal()
        self.specials += Tristate(self.jtag_tdo, tdo_o, tdo_oe)

        self.cpu_params.update(
            i_trst_n = self.jtag_trst,
            i_tck    = self.jtag_tck,
            i_tms    = self.jtag_tms,
            i_tdi    = self.jtag_tdi,
            o_tdo    = tdo_o,
            o_tdo_oe = tdo_oe,
        )

    def set_reset_address(self, reset_address):
        self.reset_address = reset_address
        assert reset_address == 0x1000_0000, "cpu_reset_addr hardcoded in during elaboration!"

    def do_finalize(self):
        assert hasattr(self, "reset_address")
        basedir = get_data_mod("cpu", "scariv").data_location
        subprocess.check_call("make -C {basedir}/verilator_sim .config_design_xlen64_flen64".format(
            basedir = basedir),
                              shell=True)
        self.specials += Instance("scariv_tile_wrapper", **self.cpu_params)
