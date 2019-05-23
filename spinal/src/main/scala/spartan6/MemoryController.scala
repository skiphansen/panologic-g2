
package spartan6

import spinal.core._

import spinal.lib._
import spinal.lib.io._
import spinal.lib.bus.amba4.axi._

import spartan6._


class MemoryController() extends Component {

    val io = new Bundle {
	val clk = in(Bool)
	val ddr2a = master(DDRIntfc())
	val ddr2b = master(DDRIntfc())
	val axi1 = slave(Axi4(Axi4Config(addressWidth = 32, dataWidth = 32, idWidth = 4)))
	val axi2 = slave(Axi4(Axi4Config(addressWidth = 32, dataWidth = 32, idWidth = 4)))
    }


    val mig = new MIG()
    io.ddr2a.d      <> mig.io.mcb1_dram_dq
    io.ddr2a.a      <> mig.io.mcb1_dram_a
    io.ddr2a.ba     <> mig.io.mcb1_dram_ba
    io.ddr2a.ras_l  <> mig.io.mcb1_dram_ras_n
    io.ddr2a.cas_l  <> mig.io.mcb1_dram_cas_n
    io.ddr2a.we_l   <> mig.io.mcb1_dram_we_n
    io.ddr2a.odt    <> mig.io.mcb1_dram_odt
    io.ddr2a.cke    <> mig.io.mcb1_dram_cke
    io.ddr2a.ldm    <> mig.io.mcb1_dram_dm
    io.ddr2a.udm    <> mig.io.mcb1_dram_udm
    io.ddr2a.rzq    <> mig.io.mcb1_rzq
    //io.ddr2a.zio    <> mig.io.mcb1_zio
    io.ddr2a.ldqs_p <> mig.io.mcb1_dram_dqs
    io.ddr2a.ldqs_n <> mig.io.mcb1_dram_dqs_n
    io.ddr2a.udqs_p <> mig.io.mcb1_dram_udqs
    io.ddr2a.udqs_n <> mig.io.mcb1_dram_udqs_n
    io.ddr2a.ck_p   <> mig.io.mcb1_dram_ck
    io.ddr2a.ck_n   <> mig.io.mcb1_dram_ck_n

    io.ddr2b.d      <> mig.io.mcb3_dram_dq
    io.ddr2b.a      <> mig.io.mcb3_dram_a
    io.ddr2b.ba     <> mig.io.mcb3_dram_ba
    io.ddr2b.ras_l  <> mig.io.mcb3_dram_ras_n
    io.ddr2b.cas_l  <> mig.io.mcb3_dram_cas_n
    io.ddr2b.we_l   <> mig.io.mcb3_dram_we_n
    io.ddr2b.odt    <> mig.io.mcb3_dram_odt
    io.ddr2b.cke    <> mig.io.mcb3_dram_cke
    io.ddr2b.ldm    <> mig.io.mcb3_dram_dm
    io.ddr2b.udm    <> mig.io.mcb3_dram_udm
    io.ddr2b.rzq    <> mig.io.mcb3_rzq
    //io.ddr2b.zio    <> mig.io.mcb3_zio
    io.ddr2b.ldqs_p <> mig.io.mcb3_dram_dqs
    io.ddr2b.ldqs_n <> mig.io.mcb3_dram_dqs_n
    io.ddr2b.udqs_p <> mig.io.mcb3_dram_udqs
    io.ddr2b.udqs_n <> mig.io.mcb3_dram_udqs_n
    io.ddr2b.ck_p   <> mig.io.mcb3_dram_ck
    io.ddr2b.ck_n   <> mig.io.mcb3_dram_ck_n

    mig.io.c1_sys_clk := io.clk
    mig.io.c1_sys_rst_i := True
    mig.io.c3_sys_clk := io.clk
    mig.io.c3_sys_rst_i := True

    // AXI 1
    mig.io.c1_s0_axi_aclk := io.clk
    mig.io.c1_s0_axi_aresetn := True

    mig.io.c1_s0_axi_awid := io.axi1.aw.id.asBits
    mig.io.c1_s0_axi_awaddr := io.axi1.aw.addr.asBits
    mig.io.c1_s0_axi_awlen := io.axi1.aw.len.asBits
    mig.io.c1_s0_axi_awsize := io.axi1.aw.size.asBits
    mig.io.c1_s0_axi_awburst := io.axi1.aw.burst
    mig.io.c1_s0_axi_awlock := io.axi1.aw.lock
    mig.io.c1_s0_axi_awcache := io.axi1.aw.cache
    mig.io.c1_s0_axi_awprot := io.axi1.aw.prot
    mig.io.c1_s0_axi_awqos := io.axi1.aw.qos
    mig.io.c1_s0_axi_awvalid := io.axi1.aw.valid
    io.axi1.aw.ready := mig.io.c1_s0_axi_awready

    mig.io.c1_s0_axi_wdata := io.axi1.w.data.resize(128)
    mig.io.c1_s0_axi_wstrb := io.axi1.w.strb.resize(16)
    mig.io.c1_s0_axi_wlast := io.axi1.w.last
    mig.io.c1_s0_axi_wvalid := io.axi1.w.valid
    io.axi1.w.ready := mig.io.c1_s0_axi_wready
    
    // ? io.axi1.w.id := mig.io.c1_s0_axi_wid

    io.axi1.b.id := mig.io.c1_s0_axi_bid.asUInt
    io.axi1.b.resp := mig.io.c1_s0_axi_bresp
    io.axi1.b.valid := mig.io.c1_s0_axi_bvalid
    mig.io.c1_s0_axi_bready := io.axi1.b.ready

    mig.io.c1_s0_axi_arid := io.axi1.ar.id.asBits
    mig.io.c1_s0_axi_araddr := io.axi1.ar.addr.asBits
    mig.io.c1_s0_axi_arlen := io.axi1.ar.len.asBits
    mig.io.c1_s0_axi_arsize := io.axi1.ar.size.asBits
    mig.io.c1_s0_axi_arburst := io.axi1.ar.burst
    mig.io.c1_s0_axi_arlock := io.axi1.ar.lock
    mig.io.c1_s0_axi_arcache := io.axi1.ar.cache
    mig.io.c1_s0_axi_arprot := io.axi1.ar.prot
    mig.io.c1_s0_axi_arqos := io.axi1.ar.qos
    mig.io.c1_s0_axi_arvalid := io.axi1.ar.valid
    io.axi1.ar.ready := mig.io.c1_s0_axi_arready

    io.axi1.r.id := mig.io.c1_s0_axi_rid.asUInt
    io.axi1.r.data := mig.io.c1_s0_axi_rdata.resize(32)
    io.axi1.r.resp := mig.io.c1_s0_axi_rresp
    io.axi1.r.last := mig.io.c1_s0_axi_rlast
    io.axi1.r.valid := mig.io.c1_s0_axi_rvalid
    mig.io.c1_s0_axi_rready := io.axi1.r.ready

    // AXI 2
    mig.io.c3_s0_axi_aclk := io.clk
    mig.io.c3_s0_axi_aresetn := True

    mig.io.c3_s0_axi_awid := io.axi2.aw.id.asBits
    mig.io.c3_s0_axi_awaddr := io.axi2.aw.addr.asBits
    mig.io.c3_s0_axi_awlen := io.axi2.aw.len.asBits
    mig.io.c3_s0_axi_awsize := io.axi2.aw.size.asBits
    mig.io.c3_s0_axi_awburst := io.axi2.aw.burst
    mig.io.c3_s0_axi_awlock := io.axi2.aw.lock
    mig.io.c3_s0_axi_awcache := io.axi2.aw.cache
    mig.io.c3_s0_axi_awprot := io.axi2.aw.prot
    mig.io.c3_s0_axi_awqos := io.axi2.aw.qos
    mig.io.c3_s0_axi_awvalid := io.axi2.aw.valid
    io.axi2.aw.ready := mig.io.c3_s0_axi_awready

    mig.io.c3_s0_axi_wdata := io.axi2.w.data.resize(128)
    mig.io.c3_s0_axi_wstrb := io.axi2.w.strb.resize(16)
    mig.io.c3_s0_axi_wlast := io.axi2.w.last
    mig.io.c3_s0_axi_wvalid := io.axi2.w.valid
    io.axi2.w.ready := mig.io.c3_s0_axi_wready
    
    // ? io.axi2.w.id := mig.io.c3_s0_axi_wid

    io.axi2.b.id := mig.io.c3_s0_axi_bid.asUInt
    io.axi2.b.resp := mig.io.c3_s0_axi_bresp
    io.axi2.b.valid := mig.io.c3_s0_axi_bvalid
    mig.io.c3_s0_axi_bready := io.axi2.b.ready

    mig.io.c3_s0_axi_arid := io.axi2.ar.id.asBits
    mig.io.c3_s0_axi_araddr := io.axi2.ar.addr.asBits
    mig.io.c3_s0_axi_arlen := io.axi2.ar.len.asBits
    mig.io.c3_s0_axi_arsize := io.axi2.ar.size.asBits
    mig.io.c3_s0_axi_arburst := io.axi2.ar.burst
    mig.io.c3_s0_axi_arlock := io.axi2.ar.lock
    mig.io.c3_s0_axi_arcache := io.axi2.ar.cache
    mig.io.c3_s0_axi_arprot := io.axi2.ar.prot
    mig.io.c3_s0_axi_arqos := io.axi2.ar.qos
    mig.io.c3_s0_axi_arvalid := io.axi2.ar.valid
    io.axi2.ar.ready := mig.io.c3_s0_axi_arready

    io.axi2.r.id := mig.io.c3_s0_axi_rid.asUInt
    io.axi2.r.data := mig.io.c3_s0_axi_rdata.resize(32)
    io.axi2.r.resp := mig.io.c3_s0_axi_rresp
    io.axi2.r.last := mig.io.c3_s0_axi_rlast
    io.axi2.r.valid := mig.io.c3_s0_axi_rvalid
    mig.io.c3_s0_axi_rready := io.axi2.r.ready

}
