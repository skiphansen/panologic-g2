
package pano

import spinal.core._
import spinal.lib._
import spinal.lib.io._
import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.amba4.axi._
import spinal.lib.bus.misc.SizeMapping

import spartan6._

import vexriscv.demo.MuraxApb3Timer

import scala.collection.mutable.ArrayBuffer
import spinal.lib.com.uart._

import cc._
import gmii._
import ulpi._

case class CpuTop(panoConfig: PanoConfig) extends Component {

    val io = new Bundle {

        val led_ctrl_apb        = master(Apb3(Apb3Gpio.getApb3Config()))
        val uart_ctrl_apb       = if (panoConfig.includeUart)   master(Apb3(Apb3UartCtrl.getApb3Config))  else null
        val dvi_ctrl_apb        = if (panoConfig.includeDviI2C) master(Apb3(CCGpio.getApb3Config()))      else null
        val gmii_ctrl_apb       = if (panoConfig.includeGmii)   master(Apb3(GmiiCtrl.getApb3Config()))    else null
        val test_patt_apb       = master(Apb3(VideoTestPattern.getApb3Config()))
        val txt_gen_apb         = master(Apb3(VideoTxtGen.getApb3Config()))
        val ulpi_apb            = if (panoConfig.includeUlpi)   master(Apb3(UlpiCtrl.getApb3Config()))    else null
        val usb_host_apb        = if (panoConfig.includeUlpi)   master(Apb3(UsbHost.getApb3Config()))     else null

        val switch_             = in(Bool)

	val axi1 = master(Axi4(Axi4Config(addressWidth = 32, dataWidth = 32, idWidth = 4)))
	val axi2 = master(Axi4(Axi4Config(addressWidth = 32, dataWidth = 32, idWidth = 4)))
    }

//    val u_cpu = CpuComplex(CpuComplexConfig.default.copy(onChipRamSize = 8 kB, onChipRamHexFile = "sw/progmem.hex"))
    val u_cpu = CpuComplex(CpuComplexConfig.default)
    u_cpu.io.externalInterrupt <> False

    val jtagUart = new Apb3JtagUart()

    val apbMapping = ArrayBuffer[(Apb3, SizeMapping)]()

    apbMapping += io.led_ctrl_apb       -> (0x00000, 256 Byte)

    if (panoConfig.includeDviI2C){
        apbMapping += io.dvi_ctrl_apb       -> (0x00100, 256 Byte)
    }
    apbMapping += io.test_patt_apb      -> (0x00200, 256 Byte)
    if (panoConfig.includeUlpi){
        apbMapping += io.ulpi_apb           -> (0x00300, 256 Byte)
        apbMapping += io.usb_host_apb       -> (0x00400, 256 Byte)
    }
    if (panoConfig.includeUart){
        apbMapping += io.uart_ctrl_apb      -> (0x00500, 256 Byte)
    }
    apbMapping += jtagUart.io.apb           -> (0x00600, 16 Byte)

    if (panoConfig.includeGmii){
        apbMapping += io.gmii_ctrl_apb      -> (0x10000, 4 kB)
    }

    apbMapping += io.txt_gen_apb        -> (0x20000, 64 kB)

    //============================================================
    // Timer
    //============================================================

    val u_timer = new MuraxApb3Timer()
    u_timer.io.interrupt        <> u_cpu.io.timerInterrupt
    apbMapping += u_timer.io.apb -> (0x30000, 4 kB)

    io.axi1 << u_cpu.io.axiMem1.toAxi4
    io.axi2 << u_cpu.io.axiMem2.toAxi4

    //============================================================
    // Local APB decoder
    //============================================================
    val apbDecoder = Apb3Decoder(
      master = u_cpu.io.apb,
      slaves = apbMapping
    )

    val jtag = new BSCAN_SPARTAN6()
    u_cpu.io.jtag.tdi <> jtag.io.TDI
    u_cpu.io.jtag.tms <> jtag.io.TMS
    u_cpu.io.jtag.tck <> jtag.io.TCK
    u_cpu.io.jtag.tdo <> jtag.io.TDO

}
