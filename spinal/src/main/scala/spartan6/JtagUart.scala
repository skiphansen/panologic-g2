
package spartan6

import spinal.core._
import spinal.lib._
import spinal.lib.io._
import spinal.lib.bus.misc.BusSlaveFactory
import spinal.lib.bus.amba3.apb.{Apb3, Apb3Config, Apb3SlaveFactory}

import spartan6._

object JtagUart {
    // 4 bits -> 4 32-bit registers
    def getApb3Config() = Apb3Config(addressWidth = 4, dataWidth=32)
}

class JtagUart() extends Component {
    val io = new Bundle {
        val tx = slave(Stream(Bits(8 bits)))
	val rx = master(Stream(Bits(8 bits)))
    }

    val jtag = new BSCAN_SPARTAN6(jtagChain = 3)

    val jtagClock = new ClockDomain(clock = jtag.io.TCK, reset = jtag.io.RESET)

    val txStreamCC = new StreamFifoCC(dataType = Bits(8 bits), depth = 256, pushClock = ClockDomain.current, popClock = jtagClock)
    txStreamCC.io.push << io.tx

    val rxStreamCC = new StreamFifoCC(dataType = Bits(8 bits), depth = 32, pushClock = jtagClock, popClock = ClockDomain.current)
    io.rx << rxStreamCC.io.pop


    val jtagArea = new ClockingArea(jtagClock) {

        val dr_shift     = Reg(Bits(9 bits)) init(0)
	val push_valid   = Reg(Bool)         init(False)
	val push_payload = Reg(Bits(8 bits)) init(0)

	rxStreamCC.io.push.valid := push_valid
	rxStreamCC.io.push.payload := push_payload

	when (rxStreamCC.io.push.ready) {
	    rxStreamCC.io.push.valid := False
	    push_valid := False
	}

	txStreamCC.io.pop.ready := False;

        when (jtag.io.SEL) {
            when (jtag.io.CAPTURE) {
                dr_shift(8)             := txStreamCC.io.pop.valid
		dr_shift(7 downto 0)    := txStreamCC.io.pop.payload
		txStreamCC.io.pop.ready := True
            }
            when (jtag.io.SHIFT) {
                dr_shift := (B(jtag.io.TDI, 9 bits) |<< 8) | (dr_shift |>> 1) 
            }
            when (jtag.io.UPDATE) {
		when (dr_shift(8)) {
		    push_valid   := True
		    push_payload := dr_shift(7 downto 0)
		}
            }
        }

        jtag.io.TDO := dr_shift(0)
    }

    def driveFrom(busCtrl: BusSlaveFactory, baseAddress: BigInt) = new Area {
	
	busCtrl.createAndDriveFlow(Bits(8 bits), address = 8).toStream >> io.tx
	busCtrl.readStreamNonBlocking(io.rx, address = 12, validBitOffset = 31, payloadBitOffset = 0)
    }
}

case class Apb3JtagUart() extends Component
{
    val io = new Bundle {
        val apb = slave(Apb3(JtagUart.getApb3Config()))
    }

    val jtagUart = new JtagUart()

    val busCtrl = Apb3SlaveFactory(io.apb)
    val apb_regs = jtagUart.driveFrom(busCtrl, 0x0)
}


