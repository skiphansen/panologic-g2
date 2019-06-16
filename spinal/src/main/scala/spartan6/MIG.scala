
package spartan6

import spinal.core._
import spinal.lib._
import spinal.lib.io._

case class DDRIntfc() extends Bundle with IMasterSlave
{
    val d = Analog(Bits(16 bits))
    val a  = Bits(13 bits)
    val ba = Bits(2 bits)
    val ras_l = Bool
    val cas_l = Bool
    val we_l = Bool
    val odt = Bool
    val cke = Bool
    val ldm = Bool
    val udm = Bool
    val rzq = Analog(Bool)
    //val zio = Analog(Bool)
    val ldqs_p = Analog(Bool)
    val ldqs_n = Analog(Bool)
    val udqs_p = Analog(Bool)
    val udqs_n = Analog(Bool)
    val ck_p = Bool
    val ck_n = Bool

    override def asMaster: Unit = {
        inout(d)
        out(a)
        out(ba)
        out(ras_l)
        out(cas_l)
        out(we_l)
        out(odt)
        out(cke)
        out(ldm)
        out(udm)
        inout(rzq)
        //inout(zio)
        inout(ldqs_p)
        inout(ldqs_n)
        inout(udqs_p)
        inout(udqs_n)
        out(ck_p)
        out(ck_n)
    }
}

class MIG(
  AXI_ID_WIDTH : Int = 4,
  AXI_ADDR_WIDTH : Int = 32,
  AXI_DATA_WIDTH : Int = 32
) extends BlackBox {

    val generic = new Generic {
        val C1_S0_AXI_ID_WIDTH = AXI_ID_WIDTH
	val C1_S0_AXI_ADDR_WIDTH = AXI_ADDR_WIDTH
	val C1_S0_AXI_DATA_WIDTH = AXI_DATA_WIDTH
        val C3_S0_AXI_ID_WIDTH = AXI_ID_WIDTH
	val C3_S0_AXI_ADDR_WIDTH = AXI_ADDR_WIDTH
	val C3_S0_AXI_DATA_WIDTH = AXI_DATA_WIDTH
    }

    val io = new Bundle {
        val mcb1_dram_dq = inout(Analog(Bits(16 bits)))
        val mcb1_dram_a  = out(Bits(13 bits))
        val mcb1_dram_ba = out(Bits(2 bits))
        val mcb1_dram_ras_n = out(Bool)
        val mcb1_dram_cas_n = out(Bool)
        val mcb1_dram_we_n = out(Bool)
        val mcb1_dram_odt = out(Bool)
        val mcb1_dram_cke = out(Bool)
        val mcb1_dram_dm  = out(Bool)
        val mcb1_dram_udm  = out(Bool)
        val mcb1_rzq = inout(Analog(Bool))
        //val mcb1_zio = inout(Analog(Bool))
        val c1_sys_clk = in(Bool)
        val c1_sys_rst_i = in(Bool)
        val c1_calib_done = out(Bool)
        val c1_clk0 = out(Bool)
        val c1_rst0 = out(Bool)
        val mcb1_dram_dqs = inout(Analog(Bool))
        val mcb1_dram_dqs_n = inout(Analog(Bool))
        val mcb1_dram_udqs = inout(Analog(Bool))
        val mcb1_dram_udqs_n = inout(Analog(Bool))
        val mcb1_dram_ck = out(Bool)
        val mcb1_dram_ck_n = out(Bool)

        val mcb3_dram_dq = inout(Analog(Bits(16 bits)))
        val mcb3_dram_a  = out(Bits(13 bits))
        val mcb3_dram_ba = out(Bits(2 bits))
        val mcb3_dram_ras_n = out(Bool)
        val mcb3_dram_cas_n = out(Bool)
        val mcb3_dram_we_n = out(Bool)
        val mcb3_dram_odt = out(Bool)
        val mcb3_dram_cke = out(Bool)
        val mcb3_dram_dm  = out(Bool)
        val mcb3_dram_udm  = out(Bool)
        val mcb3_rzq = inout(Analog(Bool))
        //val mcb3_zio = inout(Analog(Bool))
        val c3_sys_clk = in(Bool)
        val c3_sys_rst_i = in(Bool)
        val c3_calib_done = out(Bool)
        val c3_clk0 = out(Bool)
        val c3_rst0 = out(Bool)
        val mcb3_dram_dqs = inout(Analog(Bool))
        val mcb3_dram_dqs_n = inout(Analog(Bool))
        val mcb3_dram_udqs = inout(Analog(Bool))
        val mcb3_dram_udqs_n = inout(Analog(Bool))
        val mcb3_dram_ck = out(Bool)
        val mcb3_dram_ck_n = out(Bool)

	// AXI bus for controller 1
        val c1_s0_axi_aclk = in(Bool)
        val c1_s0_axi_aresetn = in(Bool)
        val c1_s0_axi_awid = in(Bits(AXI_ID_WIDTH bits))
        val c1_s0_axi_awaddr = in(Bits(AXI_ADDR_WIDTH bits))
        val c1_s0_axi_awlen = in(Bits(8 bits))
        val c1_s0_axi_awsize = in(Bits(3 bits))
        val c1_s0_axi_awburst = in(Bits(2 bits))
        val c1_s0_axi_awlock = in(Bits(1 bits))
        val c1_s0_axi_awcache = in(Bits(4 bits))
        val c1_s0_axi_awprot = in(Bits(3 bits))
        val c1_s0_axi_awqos = in(Bits(4 bits))
        val c1_s0_axi_awvalid = in(Bool)
        val c1_s0_axi_awready = out(Bool)
        val c1_s0_axi_wdata = in(Bits(AXI_DATA_WIDTH bits))
        val c1_s0_axi_wstrb = in(Bits(AXI_DATA_WIDTH/8 bits))
        val c1_s0_axi_wlast = in(Bool)
        val c1_s0_axi_wvalid = in(Bool)
        val c1_s0_axi_wready = out(Bool)
        val c1_s0_axi_bid = out(Bits(AXI_ID_WIDTH bits))
        val c1_s0_axi_wid = out(Bits(AXI_ID_WIDTH bits))
        val c1_s0_axi_bresp = out(Bits(2 bits))
        val c1_s0_axi_bvalid = out(Bool)
        val c1_s0_axi_bready = in(Bool)
        val c1_s0_axi_arid = in(Bits(AXI_ID_WIDTH bits))
        val c1_s0_axi_araddr = in(Bits(AXI_ADDR_WIDTH bits))
        val c1_s0_axi_arlen = in(Bits(8 bits))
        val c1_s0_axi_arsize = in(Bits(3 bits))
        val c1_s0_axi_arburst = in(Bits(2 bits))
        val c1_s0_axi_arlock = in(Bits(1 bits))
        val c1_s0_axi_arcache = in(Bits(4 bits))
        val c1_s0_axi_arprot = in(Bits(3 bits))
        val c1_s0_axi_arqos = in(Bits(4 bits))
        val c1_s0_axi_arvalid = in(Bool)
        val c1_s0_axi_arready = out(Bool)
        val c1_s0_axi_rid = out(Bits(AXI_ID_WIDTH bits))
        val c1_s0_axi_rdata = out(Bits(AXI_DATA_WIDTH bits))
        val c1_s0_axi_rresp = out(Bits(2 bits))
        val c1_s0_axi_rlast = out(Bool)
        val c1_s0_axi_rvalid = out(Bool)
        val c1_s0_axi_rready = in(Bool)

	// AXI bus for controller 3
        val c3_s0_axi_aclk = in(Bool)
        val c3_s0_axi_aresetn = in(Bool)
        val c3_s0_axi_awid = in(Bits(AXI_ID_WIDTH bits))
        val c3_s0_axi_awaddr = in(Bits(AXI_ADDR_WIDTH bits))
        val c3_s0_axi_awlen = in(Bits(8 bits))
        val c3_s0_axi_awsize = in(Bits(3 bits))
        val c3_s0_axi_awburst = in(Bits(2 bits))
        val c3_s0_axi_awlock = in(Bits(1 bits))
        val c3_s0_axi_awcache = in(Bits(4 bits))
        val c3_s0_axi_awprot = in(Bits(3 bits))
        val c3_s0_axi_awqos = in(Bits(4 bits))
        val c3_s0_axi_awvalid = in(Bool)
        val c3_s0_axi_awready = out(Bool)
        val c3_s0_axi_wdata = in(Bits(AXI_DATA_WIDTH bits))
        val c3_s0_axi_wstrb = in(Bits(AXI_DATA_WIDTH/8 bits))
        val c3_s0_axi_wlast = in(Bool)
        val c3_s0_axi_wvalid = in(Bool)
        val c3_s0_axi_wready = out(Bool)
        val c3_s0_axi_bid = out(Bits(AXI_ID_WIDTH bits))
        val c3_s0_axi_wid = out(Bits(AXI_ID_WIDTH bits))
        val c3_s0_axi_bresp = out(Bits(2 bits))
        val c3_s0_axi_bvalid = out(Bool)
        val c3_s0_axi_bready = in(Bool)
        val c3_s0_axi_arid = in(Bits(AXI_ID_WIDTH bits))
        val c3_s0_axi_araddr = in(Bits(AXI_ADDR_WIDTH bits))
        val c3_s0_axi_arlen = in(Bits(8 bits))
        val c3_s0_axi_arsize = in(Bits(3 bits))
        val c3_s0_axi_arburst = in(Bits(2 bits))
        val c3_s0_axi_arlock = in(Bits(1 bits))
        val c3_s0_axi_arcache = in(Bits(4 bits))
        val c3_s0_axi_arprot = in(Bits(3 bits))
        val c3_s0_axi_arqos = in(Bits(4 bits))
        val c3_s0_axi_arvalid = in(Bool)
        val c3_s0_axi_arready = out(Bool)
        val c3_s0_axi_rid = out(Bits(AXI_ID_WIDTH bits))
        val c3_s0_axi_rdata = out(Bits(AXI_DATA_WIDTH bits))
        val c3_s0_axi_rresp = out(Bits(2 bits))
        val c3_s0_axi_rlast = out(Bool)
        val c3_s0_axi_rvalid = out(Bool)
        val c3_s0_axi_rready = in(Bool)
    }

    noIoPrefix()
}
