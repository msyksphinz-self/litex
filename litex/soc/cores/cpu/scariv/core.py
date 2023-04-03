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
            "rom"       : 0x0000_0000,
            "sram"      : 0x2000_0000,
            "main_sram" : 0x4000_0000,
            "csr"       : 0xf000_0000,
            "clint"     : 0x0200_0000,
            "plic"      : 0x0c00_0000,
        }

    def __init__(self, platform, variant="standard"):
        self.platform     = platform
        self.variant      = variant
        self.reset        = Signal()
        self.interrupt    = Signal(32)
        # Peripheral bus (Connected to main SoC's bus).
        axi_if  = axi.AXIInterface(data_width=128, address_width=32, id_width=9)
        wb_if  = wishbone.Interface(data_width=128, adr_width=32-log2_int(128//8))

        self.periph_buses = [wb_if]
        # Memory buses (Connected directly to LiteDRAM).
        self.memory_buses = []

        # CPU Instance.
        self.cpu_params = dict(
            # Clk / Rst.
            i_i_clk       = ClockSignal("sys"),
            i_i_reset_n   = ~ResetSignal("sys") | self.reset,

            # Interrupts
            i_i_interrupts = self.interrupt,

            # AXI interface.
            o_axi_if_aw_valid   = axi_if.aw.valid     ,
            i_axi_if_aw_ready   = axi_if.aw.ready     ,
            o_axi_if_aw_last    = axi_if.aw.last      ,
            o_axi_if_aw_addr    = axi_if.aw.addr      ,
            o_axi_if_aw_burst   = axi_if.aw.burst     ,
            o_axi_if_aw_len     = axi_if.aw.len       ,
            o_axi_if_aw_size    = axi_if.aw.size      ,
            o_axi_if_aw_lock    = axi_if.aw.lock      ,
            o_axi_if_aw_prot    = axi_if.aw.prot      ,
            o_axi_if_aw_cache   = axi_if.aw.cache     ,
            o_axi_if_aw_qos     = axi_if.aw.qos       ,
            o_axi_if_aw_region  = axi_if.aw.region    ,
            o_axi_if_aw_id      = axi_if.aw.id        ,

            o_axi_if_w_valid    = axi_if.w.valid      ,
            i_axi_if_w_ready    = axi_if.w.ready      ,
            o_axi_if_w_last     = axi_if.w.last       ,
            o_axi_if_w_data     = axi_if.w.data       ,
            o_axi_if_w_strb     = axi_if.w.strb       ,

            i_axi_if_b_valid    = axi_if.b.valid      ,
            o_axi_if_b_ready    = axi_if.b.ready      ,
            i_axi_if_b_resp     = axi_if.b.resp       ,
            i_axi_if_b_id       = axi_if.b.id         ,

            o_axi_if_ar_valid   = axi_if.ar.valid     ,
            i_axi_if_ar_ready   = axi_if.ar.ready     ,
            o_axi_if_ar_last    = axi_if.ar.last      ,
            o_axi_if_ar_addr    = axi_if.ar.addr      ,
            o_axi_if_ar_burst   = axi_if.ar.burst     ,
            o_axi_if_ar_len     = axi_if.ar.len       ,
            o_axi_if_ar_size    = axi_if.ar.size      ,
            o_axi_if_ar_lock    = axi_if.ar.lock      ,
            o_axi_if_ar_prot    = axi_if.ar.prot      ,
            o_axi_if_ar_cache   = axi_if.ar.cache     ,
            o_axi_if_ar_qos     = axi_if.ar.qos       ,
            o_axi_if_ar_region  = axi_if.ar.region    ,
            o_axi_if_ar_id      = axi_if.ar.id        ,

            i_axi_if_r_valid    = axi_if.r.valid      ,
            o_axi_if_r_ready    = axi_if.r.ready      ,
            i_axi_if_r_last     = axi_if.r.last       ,
            i_axi_if_r_resp     = axi_if.r.resp       ,
            i_axi_if_r_data     = axi_if.r.data       ,
            i_axi_if_r_id       = axi_if.r.id         ,
        )

        # Adapt AXI interfaces to Wishbone.
        a2w = axi.AXI2Wishbone(axi_if, wb_if, base_address=0)
        self.submodules += a2w

        # Add Verilog sources.
        # TODO: use Flist.cv64a6_imafdc_sv39 and Flist.cv32a6_imac_sv0 instead
        basedir = get_data_mod("cpu", "scariv").data_location
        platform.add_source(os.path.join(basedir, "tb",  "sim_pkg.sv"))
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


    def add_soc_components(self, soc, soc_region_cls):
        # Define ISA.
        soc.add_constant("LITEX_SIMULATION", 1)


    def set_reset_address(self, reset_address):
        self.reset_address = reset_address
        # assert reset_address == 0x1000_0000, "cpu_reset_addr hardcoded in during elaboration!"

    def do_finalize(self):
        assert hasattr(self, "reset_address")
        basedir = get_data_mod("cpu", "scariv").data_location
        subprocess.check_call("make -C {basedir}/verilator_sim .config_design_xlen64_flen64 RV_FLEN=64 ISA=imac".format(
            basedir = basedir),
                              shell=True)
        self.specials += Instance("scariv_subsystem_axi_wrapper", **self.cpu_params)
