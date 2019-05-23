package spartan6

import spinal.core._

class BSCAN_SPARTAN6 (
        jtagChain : Int = 1 // Chain number.
    ) extends BlackBox {

    val generic = new Generic {
        val JTAG_CHAIN = jtagChain
    }

    val io = new Bundle {
        val CAPTURE          = out(Bool) // Scan Data Register Capture instruction
        val DRCK             = out(Bool) // Scan Clock instruction. DRCK is a gated version of TCK, it toggles during the CAPTUREDR and SHIFTDR states
        val RESET            = out(Bool) // Scan register reset instruction
        val RUNTEST          = out(Bool) // Asserted when the TAP controller is in the Run Test Idle state
        val SEL              = out(Bool) // Scan mode Select instruction
        val SHIFT            = out(Bool) // Scan Chain Shift instruction
        val TCK              = out(Bool) // Scan Clock
        val TDI              = out(Bool) // Scan Chain Output. Mirror of the TDI input to the FPGA
        val TMS              = out(Bool) // Test Mode Select
        val UPDATE           = out(Bool) // Scan Register Update instruction
        val TDO              = in(Bool)  // Scan Chaout INput
    }

    noIoPrefix()
}
