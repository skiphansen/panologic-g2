// Generator : SpinalHDL v1.3.2    git head : 41815ceafff4e72c2e3a3e1ff7e9ada5202a0d26
// Date      : 05/06/2019, 19:28:18
// Component : Pano


`define HostXferType_staticEncoding_type [3:0]
`define HostXferType_staticEncoding_SETUP 4'b0001
`define HostXferType_staticEncoding_BULK_IN 4'b0000
`define HostXferType_staticEncoding_BULK_OUT 4'b0010
`define HostXferType_staticEncoding_HS_IN 4'b1000
`define HostXferType_staticEncoding_HS_OUT 4'b1010
`define HostXferType_staticEncoding_ISO_IN 4'b0100
`define HostXferType_staticEncoding_ISO_OUT 4'b0110

`define HostXferResult_defaultEncoding_type [3:0]
`define HostXferResult_defaultEncoding_SUCCESS 4'b0000
`define HostXferResult_defaultEncoding_BUSY 4'b0001
`define HostXferResult_defaultEncoding_BADREQ 4'b0010
`define HostXferResult_defaultEncoding_UNDEF 4'b0011
`define HostXferResult_defaultEncoding_NAK 4'b0100
`define HostXferResult_defaultEncoding_STALL 4'b0101
`define HostXferResult_defaultEncoding_TOGERR 4'b0110
`define HostXferResult_defaultEncoding_WRONGPID 4'b0111
`define HostXferResult_defaultEncoding_BADBC 4'b1000
`define HostXferResult_defaultEncoding_PIDERR 4'b1001
`define HostXferResult_defaultEncoding_PKTERR 4'b1010
`define HostXferResult_defaultEncoding_CRCERR 4'b1011
`define HostXferResult_defaultEncoding_KERR 4'b1100
`define HostXferResult_defaultEncoding_JERR 4'b1101
`define HostXferResult_defaultEncoding_TIMEOUT 4'b1110
`define HostXferResult_defaultEncoding_BABBLE 4'b1111

`define EnvCtrlEnum_defaultEncoding_type [0:0]
`define EnvCtrlEnum_defaultEncoding_NONE 1'b0
`define EnvCtrlEnum_defaultEncoding_XRET 1'b1

`define BranchCtrlEnum_defaultEncoding_type [1:0]
`define BranchCtrlEnum_defaultEncoding_INC 2'b00
`define BranchCtrlEnum_defaultEncoding_B 2'b01
`define BranchCtrlEnum_defaultEncoding_JAL 2'b10
`define BranchCtrlEnum_defaultEncoding_JALR 2'b11

`define ShiftCtrlEnum_defaultEncoding_type [1:0]
`define ShiftCtrlEnum_defaultEncoding_DISABLE_1 2'b00
`define ShiftCtrlEnum_defaultEncoding_SLL_1 2'b01
`define ShiftCtrlEnum_defaultEncoding_SRL_1 2'b10
`define ShiftCtrlEnum_defaultEncoding_SRA_1 2'b11

`define AluBitwiseCtrlEnum_defaultEncoding_type [1:0]
`define AluBitwiseCtrlEnum_defaultEncoding_XOR_1 2'b00
`define AluBitwiseCtrlEnum_defaultEncoding_OR_1 2'b01
`define AluBitwiseCtrlEnum_defaultEncoding_AND_1 2'b10
`define AluBitwiseCtrlEnum_defaultEncoding_SRC1 2'b11

`define AluCtrlEnum_defaultEncoding_type [1:0]
`define AluCtrlEnum_defaultEncoding_ADD_SUB 2'b00
`define AluCtrlEnum_defaultEncoding_SLT_SLTU 2'b01
`define AluCtrlEnum_defaultEncoding_BITWISE 2'b10

`define Src2CtrlEnum_defaultEncoding_type [1:0]
`define Src2CtrlEnum_defaultEncoding_RS 2'b00
`define Src2CtrlEnum_defaultEncoding_IMI 2'b01
`define Src2CtrlEnum_defaultEncoding_IMS 2'b10
`define Src2CtrlEnum_defaultEncoding_PC 2'b11

`define Src1CtrlEnum_defaultEncoding_type [1:0]
`define Src1CtrlEnum_defaultEncoding_RS 2'b00
`define Src1CtrlEnum_defaultEncoding_IMU 2'b01
`define Src1CtrlEnum_defaultEncoding_PC_INCREMENT 2'b10
`define Src1CtrlEnum_defaultEncoding_URS1 2'b11

`define JtagState_defaultEncoding_type [3:0]
`define JtagState_defaultEncoding_RESET 4'b0000
`define JtagState_defaultEncoding_IDLE 4'b0001
`define JtagState_defaultEncoding_IR_SELECT 4'b0010
`define JtagState_defaultEncoding_IR_CAPTURE 4'b0011
`define JtagState_defaultEncoding_IR_SHIFT 4'b0100
`define JtagState_defaultEncoding_IR_EXIT1 4'b0101
`define JtagState_defaultEncoding_IR_PAUSE 4'b0110
`define JtagState_defaultEncoding_IR_EXIT2 4'b0111
`define JtagState_defaultEncoding_IR_UPDATE 4'b1000
`define JtagState_defaultEncoding_DR_SELECT 4'b1001
`define JtagState_defaultEncoding_DR_CAPTURE 4'b1010
`define JtagState_defaultEncoding_DR_SHIFT 4'b1011
`define JtagState_defaultEncoding_DR_EXIT1 4'b1100
`define JtagState_defaultEncoding_DR_PAUSE 4'b1101
`define JtagState_defaultEncoding_DR_EXIT2 4'b1110
`define JtagState_defaultEncoding_DR_UPDATE 4'b1111

`define Axi4ToApb3BridgePhase_defaultEncoding_type [1:0]
`define Axi4ToApb3BridgePhase_defaultEncoding_SETUP 2'b00
`define Axi4ToApb3BridgePhase_defaultEncoding_ACCESS_1 2'b01
`define Axi4ToApb3BridgePhase_defaultEncoding_RESPONSE 2'b10

`define UlpiState_defaultEncoding_type [3:0]
`define UlpiState_defaultEncoding_WaitIdle 4'b0000
`define UlpiState_defaultEncoding_Idle 4'b0001
`define UlpiState_defaultEncoding_Tx 4'b0010
`define UlpiState_defaultEncoding_Rx 4'b0011
`define UlpiState_defaultEncoding_RegWrAddr 4'b0100
`define UlpiState_defaultEncoding_RegWrData 4'b0101
`define UlpiState_defaultEncoding_RegWrStp 4'b0110
`define UlpiState_defaultEncoding_RegRdAddr 4'b0111
`define UlpiState_defaultEncoding_RegRdTurn 4'b1000
`define UlpiState_defaultEncoding_RegRdData 4'b1001

`define PidType_staticEncoding_type [3:0]
`define PidType_staticEncoding_NULL_1 4'b0000
`define PidType_staticEncoding_OUT_1 4'b0001
`define PidType_staticEncoding_IN_1 4'b1001
`define PidType_staticEncoding_SOF 4'b0101
`define PidType_staticEncoding_SETUP 4'b1101
`define PidType_staticEncoding_DATA0 4'b0011
`define PidType_staticEncoding_DATA1 4'b1011
`define PidType_staticEncoding_DATA2 4'b0111
`define PidType_staticEncoding_MDATA 4'b1111
`define PidType_staticEncoding_ACK 4'b0010
`define PidType_staticEncoding_NAK 4'b1010
`define PidType_staticEncoding_STALL 4'b1110
`define PidType_staticEncoding_NYET 4'b0110
`define PidType_staticEncoding_PRE_ERR 4'b1100
`define PidType_staticEncoding_SPLIT 4'b1000
`define PidType_staticEncoding_PING 4'b0100

`define TxState_defaultEncoding_type [3:0]
`define TxState_defaultEncoding_Idle 4'b0000
`define TxState_defaultEncoding_TokenPid 4'b0001
`define TxState_defaultEncoding_TokenAddr 4'b0010
`define TxState_defaultEncoding_TokenEndpoint 4'b0011
`define TxState_defaultEncoding_DataPid 4'b0100
`define TxState_defaultEncoding_DataData 4'b0101
`define TxState_defaultEncoding_DataCRC0 4'b0110
`define TxState_defaultEncoding_DataCRC1 4'b0111
`define TxState_defaultEncoding_HandshakePid 4'b1000
`define TxState_defaultEncoding_SpecialPid 4'b1001

`define TopState_defaultEncoding_type [1:0]
`define TopState_defaultEncoding_Idle 2'b00
`define TopState_defaultEncoding_SetupSendToken 2'b01
`define TopState_defaultEncoding_SetupSendData0 2'b10
`define TopState_defaultEncoding_SetupWaitHandshake 2'b11

`define SpiMasterCtrlCmdMode_defaultEncoding_type [0:0]
`define SpiMasterCtrlCmdMode_defaultEncoding_DATA 1'b0
`define SpiMasterCtrlCmdMode_defaultEncoding_SS 1'b1

module BufferCC (
      input   io_dataIn,
      output  io_dataOut,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg  buffers_0;
  reg  buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge toplevel_main_clk) begin
    buffers_0 <= io_dataIn;
    buffers_1 <= buffers_0;
  end

endmodule

module BufferCC_1_ (
      input  [8:0] io_initial,
      input  [8:0] io_dataIn,
      output [8:0] io_dataOut,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg [8:0] buffers_0;
  reg [8:0] buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      buffers_0 <= io_initial;
      buffers_1 <= io_initial;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end

endmodule

module BufferCC_2_ (
      input  [8:0] io_initial,
      input  [8:0] io_dataIn,
      output [8:0] io_dataOut,
      input   jtag_TCK,
      input   jtag_RESET);
  reg [8:0] buffers_0;
  reg [8:0] buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge jtag_TCK or posedge jtag_RESET) begin
    if (jtag_RESET) begin
      buffers_0 <= io_initial;
      buffers_1 <= io_initial;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end

endmodule

module BufferCC_3_ (
      input  [5:0] io_initial,
      input  [5:0] io_dataIn,
      output [5:0] io_dataOut,
      input   jtag_TCK,
      input   jtag_RESET);
  reg [5:0] buffers_0;
  reg [5:0] buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge jtag_TCK or posedge jtag_RESET) begin
    if (jtag_RESET) begin
      buffers_0 <= io_initial;
      buffers_1 <= io_initial;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end

endmodule

module BufferCC_4_ (
      input  [5:0] io_initial,
      input  [5:0] io_dataIn,
      output [5:0] io_dataOut,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg [5:0] buffers_0;
  reg [5:0] buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      buffers_0 <= io_initial;
      buffers_1 <= io_initial;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end

endmodule

module StreamFifoLowLatency (
      input   io_push_valid,
      output  io_push_ready,
      input   io_push_payload_error,
      input  [31:0] io_push_payload_inst,
      output reg  io_pop_valid,
      input   io_pop_ready,
      output reg  io_pop_payload_error,
      output reg [31:0] io_pop_payload_inst,
      input   io_flush,
      output [0:0] io_occupancy,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire [0:0] _zz_StreamFifoLowLatency_5_;
  reg  _zz_StreamFifoLowLatency_1_;
  reg  pushPtr_willIncrement;
  reg  pushPtr_willClear;
  wire  pushPtr_willOverflowIfInc;
  wire  pushPtr_willOverflow;
  reg  popPtr_willIncrement;
  reg  popPtr_willClear;
  wire  popPtr_willOverflowIfInc;
  wire  popPtr_willOverflow;
  wire  ptrMatch;
  reg  risingOccupancy;
  wire  empty;
  wire  full;
  wire  pushing;
  wire  popping;
  wire [32:0] _zz_StreamFifoLowLatency_2_;
  wire [32:0] _zz_StreamFifoLowLatency_3_;
  reg [32:0] _zz_StreamFifoLowLatency_4_;
  assign _zz_StreamFifoLowLatency_5_ = _zz_StreamFifoLowLatency_2_[0 : 0];
  always @ (*) begin
    _zz_StreamFifoLowLatency_1_ = 1'b0;
    pushPtr_willIncrement = 1'b0;
    if(pushing)begin
      _zz_StreamFifoLowLatency_1_ = 1'b1;
      pushPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    pushPtr_willClear = 1'b0;
    popPtr_willClear = 1'b0;
    if(io_flush)begin
      pushPtr_willClear = 1'b1;
      popPtr_willClear = 1'b1;
    end
  end

  assign pushPtr_willOverflowIfInc = 1'b1;
  assign pushPtr_willOverflow = (pushPtr_willOverflowIfInc && pushPtr_willIncrement);
  always @ (*) begin
    popPtr_willIncrement = 1'b0;
    if(popping)begin
      popPtr_willIncrement = 1'b1;
    end
  end

  assign popPtr_willOverflowIfInc = 1'b1;
  assign popPtr_willOverflow = (popPtr_willOverflowIfInc && popPtr_willIncrement);
  assign ptrMatch = 1'b1;
  assign empty = (ptrMatch && (! risingOccupancy));
  assign full = (ptrMatch && risingOccupancy);
  assign pushing = (io_push_valid && io_push_ready);
  assign popping = (io_pop_valid && io_pop_ready);
  assign io_push_ready = (! full);
  always @ (*) begin
    if((! empty))begin
      io_pop_valid = 1'b1;
      io_pop_payload_error = _zz_StreamFifoLowLatency_5_[0];
      io_pop_payload_inst = _zz_StreamFifoLowLatency_2_[32 : 1];
    end else begin
      io_pop_valid = io_push_valid;
      io_pop_payload_error = io_push_payload_error;
      io_pop_payload_inst = io_push_payload_inst;
    end
  end

  assign _zz_StreamFifoLowLatency_2_ = _zz_StreamFifoLowLatency_3_;
  assign io_occupancy = (risingOccupancy && ptrMatch);
  assign _zz_StreamFifoLowLatency_3_ = _zz_StreamFifoLowLatency_4_;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      risingOccupancy <= 1'b0;
    end else begin
      if((pushing != popping))begin
        risingOccupancy <= pushing;
      end
      if(io_flush)begin
        risingOccupancy <= 1'b0;
      end
    end
  end

  always @ (posedge toplevel_main_clk) begin
    if(_zz_StreamFifoLowLatency_1_)begin
      _zz_StreamFifoLowLatency_4_ <= {io_push_payload_inst,io_push_payload_error};
    end
  end

endmodule

module FlowCCByToggle (
      input   io_input_valid,
      input   io_input_payload_last,
      input  [0:0] io_input_payload_fragment,
      output  io_output_valid,
      output  io_output_payload_last,
      output [0:0] io_output_payload_fragment,
      input   _zz_FlowCCByToggle_1_,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire  bufferCC_19__io_dataOut;
  wire  outHitSignal;
  reg  inputArea_target = 0;
  reg  inputArea_data_last;
  reg [0:0] inputArea_data_fragment;
  wire  outputArea_target;
  reg  outputArea_hit;
  wire  outputArea_flow_valid;
  wire  outputArea_flow_payload_last;
  wire [0:0] outputArea_flow_payload_fragment;
  reg  outputArea_flow_m2sPipe_valid;
  reg  outputArea_flow_m2sPipe_payload_last;
  reg [0:0] outputArea_flow_m2sPipe_payload_fragment;
  BufferCC bufferCC_19_ ( 
    .io_dataIn(inputArea_target),
    .io_dataOut(bufferCC_19__io_dataOut),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  assign outputArea_target = bufferCC_19__io_dataOut;
  assign outputArea_flow_valid = (outputArea_target != outputArea_hit);
  assign outputArea_flow_payload_last = inputArea_data_last;
  assign outputArea_flow_payload_fragment = inputArea_data_fragment;
  assign io_output_valid = outputArea_flow_m2sPipe_valid;
  assign io_output_payload_last = outputArea_flow_m2sPipe_payload_last;
  assign io_output_payload_fragment = outputArea_flow_m2sPipe_payload_fragment;
  always @ (posedge _zz_FlowCCByToggle_1_) begin
    if(io_input_valid)begin
      inputArea_target <= (! inputArea_target);
      inputArea_data_last <= io_input_payload_last;
      inputArea_data_fragment <= io_input_payload_fragment;
    end
  end

  always @ (posedge toplevel_main_clk) begin
    outputArea_hit <= outputArea_target;
    if(outputArea_flow_valid)begin
      outputArea_flow_m2sPipe_payload_last <= outputArea_flow_payload_last;
      outputArea_flow_m2sPipe_payload_fragment <= outputArea_flow_payload_fragment;
    end
  end

  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      outputArea_flow_m2sPipe_valid <= 1'b0;
    end else begin
      outputArea_flow_m2sPipe_valid <= outputArea_flow_valid;
    end
  end

endmodule

module Axi4ReadOnlyErrorSlave (
      input   io_axi_ar_valid,
      output  io_axi_ar_ready,
      input  [31:0] io_axi_ar_payload_addr,
      input  [3:0] io_axi_ar_payload_cache,
      input  [2:0] io_axi_ar_payload_prot,
      output  io_axi_r_valid,
      input   io_axi_r_ready,
      output [31:0] io_axi_r_payload_data,
      output [1:0] io_axi_r_payload_resp,
      output  io_axi_r_payload_last,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire  _zz_Axi4ReadOnlyErrorSlave_1_;
  reg  sendRsp;
  reg [7:0] remaining;
  wire  remainingZero;
  assign _zz_Axi4ReadOnlyErrorSlave_1_ = (io_axi_ar_valid && io_axi_ar_ready);
  assign remainingZero = (remaining == (8'b00000000));
  assign io_axi_ar_ready = (! sendRsp);
  assign io_axi_r_valid = sendRsp;
  assign io_axi_r_payload_resp = (2'b11);
  assign io_axi_r_payload_last = remainingZero;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      sendRsp <= 1'b0;
    end else begin
      if(_zz_Axi4ReadOnlyErrorSlave_1_)begin
        sendRsp <= 1'b1;
      end
      if(sendRsp)begin
        if(io_axi_r_ready)begin
          if(remainingZero)begin
            sendRsp <= 1'b0;
          end
        end
      end
    end
  end

  always @ (posedge toplevel_main_clk) begin
    if(_zz_Axi4ReadOnlyErrorSlave_1_)begin
      remaining <= (8'b00000000);
    end
    if(sendRsp)begin
      if(io_axi_r_ready)begin
        remaining <= (remaining - (8'b00000001));
      end
    end
  end

endmodule

module Axi4SharedErrorSlave (
      input   io_axi_arw_valid,
      output  io_axi_arw_ready,
      input  [31:0] io_axi_arw_payload_addr,
      input  [2:0] io_axi_arw_payload_size,
      input  [3:0] io_axi_arw_payload_cache,
      input  [2:0] io_axi_arw_payload_prot,
      input   io_axi_arw_payload_write,
      input   io_axi_w_valid,
      output  io_axi_w_ready,
      input  [31:0] io_axi_w_payload_data,
      input  [3:0] io_axi_w_payload_strb,
      input   io_axi_w_payload_last,
      output  io_axi_b_valid,
      input   io_axi_b_ready,
      output [1:0] io_axi_b_payload_resp,
      output  io_axi_r_valid,
      input   io_axi_r_ready,
      output [31:0] io_axi_r_payload_data,
      output [1:0] io_axi_r_payload_resp,
      output  io_axi_r_payload_last,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire  _zz_Axi4SharedErrorSlave_1_;
  reg  consumeData;
  reg  sendReadRsp;
  reg  sendWriteRsp;
  reg [7:0] remaining;
  wire  remainingZero;
  assign _zz_Axi4SharedErrorSlave_1_ = (io_axi_arw_valid && io_axi_arw_ready);
  assign remainingZero = (remaining == (8'b00000000));
  assign io_axi_arw_ready = (! ((consumeData || sendWriteRsp) || sendReadRsp));
  assign io_axi_w_ready = consumeData;
  assign io_axi_b_valid = sendWriteRsp;
  assign io_axi_b_payload_resp = (2'b11);
  assign io_axi_r_valid = sendReadRsp;
  assign io_axi_r_payload_resp = (2'b11);
  assign io_axi_r_payload_last = remainingZero;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      consumeData <= 1'b0;
      sendReadRsp <= 1'b0;
      sendWriteRsp <= 1'b0;
    end else begin
      if(_zz_Axi4SharedErrorSlave_1_)begin
        consumeData <= io_axi_arw_payload_write;
        sendReadRsp <= (! io_axi_arw_payload_write);
      end
      if(((io_axi_w_valid && io_axi_w_ready) && io_axi_w_payload_last))begin
        consumeData <= 1'b0;
        sendWriteRsp <= 1'b1;
      end
      if((io_axi_b_valid && io_axi_b_ready))begin
        sendWriteRsp <= 1'b0;
      end
      if(sendReadRsp)begin
        if(io_axi_r_ready)begin
          if(remainingZero)begin
            sendReadRsp <= 1'b0;
          end
        end
      end
    end
  end

  always @ (posedge toplevel_main_clk) begin
    if(_zz_Axi4SharedErrorSlave_1_)begin
      remaining <= (8'b00000000);
    end
    if(sendReadRsp)begin
      if(io_axi_r_ready)begin
        remaining <= (remaining - (8'b00000001));
      end
    end
  end

endmodule

module StreamArbiter (
      input   io_inputs_0_valid,
      output  io_inputs_0_ready,
      input  [31:0] io_inputs_0_payload_addr,
      input  [2:0] io_inputs_0_payload_id,
      input  [3:0] io_inputs_0_payload_region,
      input  [7:0] io_inputs_0_payload_len,
      input  [2:0] io_inputs_0_payload_size,
      input  [1:0] io_inputs_0_payload_burst,
      input  [0:0] io_inputs_0_payload_lock,
      input  [3:0] io_inputs_0_payload_cache,
      input  [3:0] io_inputs_0_payload_qos,
      input  [2:0] io_inputs_0_payload_prot,
      input   io_inputs_0_payload_write,
      input   io_inputs_1_valid,
      output  io_inputs_1_ready,
      input  [31:0] io_inputs_1_payload_addr,
      input  [2:0] io_inputs_1_payload_id,
      input  [3:0] io_inputs_1_payload_region,
      input  [7:0] io_inputs_1_payload_len,
      input  [2:0] io_inputs_1_payload_size,
      input  [1:0] io_inputs_1_payload_burst,
      input  [0:0] io_inputs_1_payload_lock,
      input  [3:0] io_inputs_1_payload_cache,
      input  [3:0] io_inputs_1_payload_qos,
      input  [2:0] io_inputs_1_payload_prot,
      input   io_inputs_1_payload_write,
      output  io_output_valid,
      input   io_output_ready,
      output [31:0] io_output_payload_addr,
      output [2:0] io_output_payload_id,
      output [3:0] io_output_payload_region,
      output [7:0] io_output_payload_len,
      output [2:0] io_output_payload_size,
      output [1:0] io_output_payload_burst,
      output [0:0] io_output_payload_lock,
      output [3:0] io_output_payload_cache,
      output [3:0] io_output_payload_qos,
      output [2:0] io_output_payload_prot,
      output  io_output_payload_write,
      output [0:0] io_chosen,
      output [1:0] io_chosenOH,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire [3:0] _zz_StreamArbiter_6_;
  wire [1:0] _zz_StreamArbiter_7_;
  wire [3:0] _zz_StreamArbiter_8_;
  wire [0:0] _zz_StreamArbiter_9_;
  wire [0:0] _zz_StreamArbiter_10_;
  reg  locked;
  wire  maskProposal_0;
  wire  maskProposal_1;
  reg  maskLocked_0;
  reg  maskLocked_1;
  wire  maskRouted_0;
  wire  maskRouted_1;
  wire [1:0] _zz_StreamArbiter_1_;
  wire [3:0] _zz_StreamArbiter_2_;
  wire [3:0] _zz_StreamArbiter_3_;
  wire [1:0] _zz_StreamArbiter_4_;
  wire  _zz_StreamArbiter_5_;
  assign _zz_StreamArbiter_6_ = (_zz_StreamArbiter_2_ - _zz_StreamArbiter_8_);
  assign _zz_StreamArbiter_7_ = {maskLocked_0,maskLocked_1};
  assign _zz_StreamArbiter_8_ = {2'd0, _zz_StreamArbiter_7_};
  assign _zz_StreamArbiter_9_ = _zz_StreamArbiter_4_[0 : 0];
  assign _zz_StreamArbiter_10_ = _zz_StreamArbiter_4_[1 : 1];
  assign maskRouted_0 = (locked ? maskLocked_0 : maskProposal_0);
  assign maskRouted_1 = (locked ? maskLocked_1 : maskProposal_1);
  assign _zz_StreamArbiter_1_ = {io_inputs_1_valid,io_inputs_0_valid};
  assign _zz_StreamArbiter_2_ = {_zz_StreamArbiter_1_,_zz_StreamArbiter_1_};
  assign _zz_StreamArbiter_3_ = (_zz_StreamArbiter_2_ & (~ _zz_StreamArbiter_6_));
  assign _zz_StreamArbiter_4_ = (_zz_StreamArbiter_3_[3 : 2] | _zz_StreamArbiter_3_[1 : 0]);
  assign maskProposal_0 = _zz_StreamArbiter_9_[0];
  assign maskProposal_1 = _zz_StreamArbiter_10_[0];
  assign io_output_valid = ((io_inputs_0_valid && maskRouted_0) || (io_inputs_1_valid && maskRouted_1));
  assign io_output_payload_addr = (maskRouted_0 ? io_inputs_0_payload_addr : io_inputs_1_payload_addr);
  assign io_output_payload_id = (maskRouted_0 ? io_inputs_0_payload_id : io_inputs_1_payload_id);
  assign io_output_payload_region = (maskRouted_0 ? io_inputs_0_payload_region : io_inputs_1_payload_region);
  assign io_output_payload_len = (maskRouted_0 ? io_inputs_0_payload_len : io_inputs_1_payload_len);
  assign io_output_payload_size = (maskRouted_0 ? io_inputs_0_payload_size : io_inputs_1_payload_size);
  assign io_output_payload_burst = (maskRouted_0 ? io_inputs_0_payload_burst : io_inputs_1_payload_burst);
  assign io_output_payload_lock = (maskRouted_0 ? io_inputs_0_payload_lock : io_inputs_1_payload_lock);
  assign io_output_payload_cache = (maskRouted_0 ? io_inputs_0_payload_cache : io_inputs_1_payload_cache);
  assign io_output_payload_qos = (maskRouted_0 ? io_inputs_0_payload_qos : io_inputs_1_payload_qos);
  assign io_output_payload_prot = (maskRouted_0 ? io_inputs_0_payload_prot : io_inputs_1_payload_prot);
  assign io_output_payload_write = (maskRouted_0 ? io_inputs_0_payload_write : io_inputs_1_payload_write);
  assign io_inputs_0_ready = (maskRouted_0 && io_output_ready);
  assign io_inputs_1_ready = (maskRouted_1 && io_output_ready);
  assign io_chosenOH = {maskRouted_1,maskRouted_0};
  assign _zz_StreamArbiter_5_ = io_chosenOH[1];
  assign io_chosen = _zz_StreamArbiter_5_;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      locked <= 1'b0;
      maskLocked_0 <= 1'b0;
      maskLocked_1 <= 1'b1;
    end else begin
      if(io_output_valid)begin
        maskLocked_0 <= maskRouted_0;
        maskLocked_1 <= maskRouted_1;
      end
      if(io_output_valid)begin
        locked <= 1'b1;
      end
      if((io_output_valid && io_output_ready))begin
        locked <= 1'b0;
      end
    end
  end

endmodule

module StreamFork (
      input   io_input_valid,
      output reg  io_input_ready,
      input  [31:0] io_input_payload_addr,
      input  [2:0] io_input_payload_id,
      input  [3:0] io_input_payload_region,
      input  [7:0] io_input_payload_len,
      input  [2:0] io_input_payload_size,
      input  [1:0] io_input_payload_burst,
      input  [0:0] io_input_payload_lock,
      input  [3:0] io_input_payload_cache,
      input  [3:0] io_input_payload_qos,
      input  [2:0] io_input_payload_prot,
      input   io_input_payload_write,
      output  io_outputs_0_valid,
      input   io_outputs_0_ready,
      output [31:0] io_outputs_0_payload_addr,
      output [2:0] io_outputs_0_payload_id,
      output [3:0] io_outputs_0_payload_region,
      output [7:0] io_outputs_0_payload_len,
      output [2:0] io_outputs_0_payload_size,
      output [1:0] io_outputs_0_payload_burst,
      output [0:0] io_outputs_0_payload_lock,
      output [3:0] io_outputs_0_payload_cache,
      output [3:0] io_outputs_0_payload_qos,
      output [2:0] io_outputs_0_payload_prot,
      output  io_outputs_0_payload_write,
      output  io_outputs_1_valid,
      input   io_outputs_1_ready,
      output [31:0] io_outputs_1_payload_addr,
      output [2:0] io_outputs_1_payload_id,
      output [3:0] io_outputs_1_payload_region,
      output [7:0] io_outputs_1_payload_len,
      output [2:0] io_outputs_1_payload_size,
      output [1:0] io_outputs_1_payload_burst,
      output [0:0] io_outputs_1_payload_lock,
      output [3:0] io_outputs_1_payload_cache,
      output [3:0] io_outputs_1_payload_qos,
      output [2:0] io_outputs_1_payload_prot,
      output  io_outputs_1_payload_write,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg  linkEnable_0;
  reg  linkEnable_1;
  always @ (*) begin
    io_input_ready = 1'b1;
    if(((! io_outputs_0_ready) && linkEnable_0))begin
      io_input_ready = 1'b0;
    end
    if(((! io_outputs_1_ready) && linkEnable_1))begin
      io_input_ready = 1'b0;
    end
  end

  assign io_outputs_0_valid = (io_input_valid && linkEnable_0);
  assign io_outputs_0_payload_addr = io_input_payload_addr;
  assign io_outputs_0_payload_id = io_input_payload_id;
  assign io_outputs_0_payload_region = io_input_payload_region;
  assign io_outputs_0_payload_len = io_input_payload_len;
  assign io_outputs_0_payload_size = io_input_payload_size;
  assign io_outputs_0_payload_burst = io_input_payload_burst;
  assign io_outputs_0_payload_lock = io_input_payload_lock;
  assign io_outputs_0_payload_cache = io_input_payload_cache;
  assign io_outputs_0_payload_qos = io_input_payload_qos;
  assign io_outputs_0_payload_prot = io_input_payload_prot;
  assign io_outputs_0_payload_write = io_input_payload_write;
  assign io_outputs_1_valid = (io_input_valid && linkEnable_1);
  assign io_outputs_1_payload_addr = io_input_payload_addr;
  assign io_outputs_1_payload_id = io_input_payload_id;
  assign io_outputs_1_payload_region = io_input_payload_region;
  assign io_outputs_1_payload_len = io_input_payload_len;
  assign io_outputs_1_payload_size = io_input_payload_size;
  assign io_outputs_1_payload_burst = io_input_payload_burst;
  assign io_outputs_1_payload_lock = io_input_payload_lock;
  assign io_outputs_1_payload_cache = io_input_payload_cache;
  assign io_outputs_1_payload_qos = io_input_payload_qos;
  assign io_outputs_1_payload_prot = io_input_payload_prot;
  assign io_outputs_1_payload_write = io_input_payload_write;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      linkEnable_0 <= 1'b1;
      linkEnable_1 <= 1'b1;
    end else begin
      if((io_outputs_0_valid && io_outputs_0_ready))begin
        linkEnable_0 <= 1'b0;
      end
      if((io_outputs_1_valid && io_outputs_1_ready))begin
        linkEnable_1 <= 1'b0;
      end
      if(io_input_ready)begin
        linkEnable_0 <= 1'b1;
        linkEnable_1 <= 1'b1;
      end
    end
  end

endmodule

module StreamFifoLowLatency_1_ (
      input   io_push_valid,
      output  io_push_ready,
      output reg  io_pop_valid,
      input   io_pop_ready,
      input   io_flush,
      output [2:0] io_occupancy,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire [0:0] _zz_StreamFifoLowLatency_1__1_;
  wire [1:0] _zz_StreamFifoLowLatency_1__2_;
  wire [0:0] _zz_StreamFifoLowLatency_1__3_;
  wire [1:0] _zz_StreamFifoLowLatency_1__4_;
  reg  pushPtr_willIncrement;
  reg  pushPtr_willClear;
  reg [1:0] pushPtr_valueNext;
  reg [1:0] pushPtr_value;
  wire  pushPtr_willOverflowIfInc;
  wire  pushPtr_willOverflow;
  reg  popPtr_willIncrement;
  reg  popPtr_willClear;
  reg [1:0] popPtr_valueNext;
  reg [1:0] popPtr_value;
  wire  popPtr_willOverflowIfInc;
  wire  popPtr_willOverflow;
  wire  ptrMatch;
  reg  risingOccupancy;
  wire  empty;
  wire  full;
  wire  pushing;
  wire  popping;
  wire [1:0] ptrDif;
  assign _zz_StreamFifoLowLatency_1__1_ = pushPtr_willIncrement;
  assign _zz_StreamFifoLowLatency_1__2_ = {1'd0, _zz_StreamFifoLowLatency_1__1_};
  assign _zz_StreamFifoLowLatency_1__3_ = popPtr_willIncrement;
  assign _zz_StreamFifoLowLatency_1__4_ = {1'd0, _zz_StreamFifoLowLatency_1__3_};
  always @ (*) begin
    pushPtr_willIncrement = 1'b0;
    if(pushing)begin
      pushPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    pushPtr_willClear = 1'b0;
    popPtr_willClear = 1'b0;
    if(io_flush)begin
      pushPtr_willClear = 1'b1;
      popPtr_willClear = 1'b1;
    end
  end

  assign pushPtr_willOverflowIfInc = (pushPtr_value == (2'b11));
  assign pushPtr_willOverflow = (pushPtr_willOverflowIfInc && pushPtr_willIncrement);
  always @ (*) begin
    pushPtr_valueNext = (pushPtr_value + _zz_StreamFifoLowLatency_1__2_);
    if(pushPtr_willClear)begin
      pushPtr_valueNext = (2'b00);
    end
  end

  always @ (*) begin
    popPtr_willIncrement = 1'b0;
    if(popping)begin
      popPtr_willIncrement = 1'b1;
    end
  end

  assign popPtr_willOverflowIfInc = (popPtr_value == (2'b11));
  assign popPtr_willOverflow = (popPtr_willOverflowIfInc && popPtr_willIncrement);
  always @ (*) begin
    popPtr_valueNext = (popPtr_value + _zz_StreamFifoLowLatency_1__4_);
    if(popPtr_willClear)begin
      popPtr_valueNext = (2'b00);
    end
  end

  assign ptrMatch = (pushPtr_value == popPtr_value);
  assign empty = (ptrMatch && (! risingOccupancy));
  assign full = (ptrMatch && risingOccupancy);
  assign pushing = (io_push_valid && io_push_ready);
  assign popping = (io_pop_valid && io_pop_ready);
  assign io_push_ready = (! full);
  always @ (*) begin
    if((! empty))begin
      io_pop_valid = 1'b1;
    end else begin
      io_pop_valid = io_push_valid;
    end
  end

  assign ptrDif = (pushPtr_value - popPtr_value);
  assign io_occupancy = {(risingOccupancy && ptrMatch),ptrDif};
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      pushPtr_value <= (2'b00);
      popPtr_value <= (2'b00);
      risingOccupancy <= 1'b0;
    end else begin
      pushPtr_value <= pushPtr_valueNext;
      popPtr_value <= popPtr_valueNext;
      if((pushing != popping))begin
        risingOccupancy <= pushing;
      end
      if(io_flush)begin
        risingOccupancy <= 1'b0;
      end
    end
  end

endmodule

module StreamArbiter_1_ (
      input   io_inputs_0_valid,
      output  io_inputs_0_ready,
      input  [31:0] io_inputs_0_payload_addr,
      input  [2:0] io_inputs_0_payload_id,
      input  [3:0] io_inputs_0_payload_region,
      input  [7:0] io_inputs_0_payload_len,
      input  [2:0] io_inputs_0_payload_size,
      input  [1:0] io_inputs_0_payload_burst,
      input  [0:0] io_inputs_0_payload_lock,
      input  [3:0] io_inputs_0_payload_cache,
      input  [3:0] io_inputs_0_payload_qos,
      input  [2:0] io_inputs_0_payload_prot,
      input   io_inputs_0_payload_write,
      input   io_inputs_1_valid,
      output  io_inputs_1_ready,
      input  [31:0] io_inputs_1_payload_addr,
      input  [2:0] io_inputs_1_payload_id,
      input  [3:0] io_inputs_1_payload_region,
      input  [7:0] io_inputs_1_payload_len,
      input  [2:0] io_inputs_1_payload_size,
      input  [1:0] io_inputs_1_payload_burst,
      input  [0:0] io_inputs_1_payload_lock,
      input  [3:0] io_inputs_1_payload_cache,
      input  [3:0] io_inputs_1_payload_qos,
      input  [2:0] io_inputs_1_payload_prot,
      input   io_inputs_1_payload_write,
      output  io_output_valid,
      input   io_output_ready,
      output [31:0] io_output_payload_addr,
      output [2:0] io_output_payload_id,
      output [3:0] io_output_payload_region,
      output [7:0] io_output_payload_len,
      output [2:0] io_output_payload_size,
      output [1:0] io_output_payload_burst,
      output [0:0] io_output_payload_lock,
      output [3:0] io_output_payload_cache,
      output [3:0] io_output_payload_qos,
      output [2:0] io_output_payload_prot,
      output  io_output_payload_write,
      output [0:0] io_chosen,
      output [1:0] io_chosenOH,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire [3:0] _zz_StreamArbiter_1__6_;
  wire [1:0] _zz_StreamArbiter_1__7_;
  wire [3:0] _zz_StreamArbiter_1__8_;
  wire [0:0] _zz_StreamArbiter_1__9_;
  wire [0:0] _zz_StreamArbiter_1__10_;
  reg  locked;
  wire  maskProposal_0;
  wire  maskProposal_1;
  reg  maskLocked_0;
  reg  maskLocked_1;
  wire  maskRouted_0;
  wire  maskRouted_1;
  wire [1:0] _zz_StreamArbiter_1__1_;
  wire [3:0] _zz_StreamArbiter_1__2_;
  wire [3:0] _zz_StreamArbiter_1__3_;
  wire [1:0] _zz_StreamArbiter_1__4_;
  wire  _zz_StreamArbiter_1__5_;
  assign _zz_StreamArbiter_1__6_ = (_zz_StreamArbiter_1__2_ - _zz_StreamArbiter_1__8_);
  assign _zz_StreamArbiter_1__7_ = {maskLocked_0,maskLocked_1};
  assign _zz_StreamArbiter_1__8_ = {2'd0, _zz_StreamArbiter_1__7_};
  assign _zz_StreamArbiter_1__9_ = _zz_StreamArbiter_1__4_[0 : 0];
  assign _zz_StreamArbiter_1__10_ = _zz_StreamArbiter_1__4_[1 : 1];
  assign maskRouted_0 = (locked ? maskLocked_0 : maskProposal_0);
  assign maskRouted_1 = (locked ? maskLocked_1 : maskProposal_1);
  assign _zz_StreamArbiter_1__1_ = {io_inputs_1_valid,io_inputs_0_valid};
  assign _zz_StreamArbiter_1__2_ = {_zz_StreamArbiter_1__1_,_zz_StreamArbiter_1__1_};
  assign _zz_StreamArbiter_1__3_ = (_zz_StreamArbiter_1__2_ & (~ _zz_StreamArbiter_1__6_));
  assign _zz_StreamArbiter_1__4_ = (_zz_StreamArbiter_1__3_[3 : 2] | _zz_StreamArbiter_1__3_[1 : 0]);
  assign maskProposal_0 = _zz_StreamArbiter_1__9_[0];
  assign maskProposal_1 = _zz_StreamArbiter_1__10_[0];
  assign io_output_valid = ((io_inputs_0_valid && maskRouted_0) || (io_inputs_1_valid && maskRouted_1));
  assign io_output_payload_addr = (maskRouted_0 ? io_inputs_0_payload_addr : io_inputs_1_payload_addr);
  assign io_output_payload_id = (maskRouted_0 ? io_inputs_0_payload_id : io_inputs_1_payload_id);
  assign io_output_payload_region = (maskRouted_0 ? io_inputs_0_payload_region : io_inputs_1_payload_region);
  assign io_output_payload_len = (maskRouted_0 ? io_inputs_0_payload_len : io_inputs_1_payload_len);
  assign io_output_payload_size = (maskRouted_0 ? io_inputs_0_payload_size : io_inputs_1_payload_size);
  assign io_output_payload_burst = (maskRouted_0 ? io_inputs_0_payload_burst : io_inputs_1_payload_burst);
  assign io_output_payload_lock = (maskRouted_0 ? io_inputs_0_payload_lock : io_inputs_1_payload_lock);
  assign io_output_payload_cache = (maskRouted_0 ? io_inputs_0_payload_cache : io_inputs_1_payload_cache);
  assign io_output_payload_qos = (maskRouted_0 ? io_inputs_0_payload_qos : io_inputs_1_payload_qos);
  assign io_output_payload_prot = (maskRouted_0 ? io_inputs_0_payload_prot : io_inputs_1_payload_prot);
  assign io_output_payload_write = (maskRouted_0 ? io_inputs_0_payload_write : io_inputs_1_payload_write);
  assign io_inputs_0_ready = (maskRouted_0 && io_output_ready);
  assign io_inputs_1_ready = (maskRouted_1 && io_output_ready);
  assign io_chosenOH = {maskRouted_1,maskRouted_0};
  assign _zz_StreamArbiter_1__5_ = io_chosenOH[1];
  assign io_chosen = _zz_StreamArbiter_1__5_;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      locked <= 1'b0;
      maskLocked_0 <= 1'b0;
      maskLocked_1 <= 1'b1;
    end else begin
      if(io_output_valid)begin
        maskLocked_0 <= maskRouted_0;
        maskLocked_1 <= maskRouted_1;
      end
      if(io_output_valid)begin
        locked <= 1'b1;
      end
      if((io_output_valid && io_output_ready))begin
        locked <= 1'b0;
      end
    end
  end

endmodule


//StreamFork_1_ remplaced by StreamFork

module StreamFifoLowLatency_2_ (
      input   io_push_valid,
      output  io_push_ready,
      output reg  io_pop_valid,
      input   io_pop_ready,
      input   io_flush,
      output [2:0] io_occupancy,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire [0:0] _zz_StreamFifoLowLatency_2__1_;
  wire [1:0] _zz_StreamFifoLowLatency_2__2_;
  wire [0:0] _zz_StreamFifoLowLatency_2__3_;
  wire [1:0] _zz_StreamFifoLowLatency_2__4_;
  reg  pushPtr_willIncrement;
  reg  pushPtr_willClear;
  reg [1:0] pushPtr_valueNext;
  reg [1:0] pushPtr_value;
  wire  pushPtr_willOverflowIfInc;
  wire  pushPtr_willOverflow;
  reg  popPtr_willIncrement;
  reg  popPtr_willClear;
  reg [1:0] popPtr_valueNext;
  reg [1:0] popPtr_value;
  wire  popPtr_willOverflowIfInc;
  wire  popPtr_willOverflow;
  wire  ptrMatch;
  reg  risingOccupancy;
  wire  empty;
  wire  full;
  wire  pushing;
  wire  popping;
  wire [1:0] ptrDif;
  assign _zz_StreamFifoLowLatency_2__1_ = pushPtr_willIncrement;
  assign _zz_StreamFifoLowLatency_2__2_ = {1'd0, _zz_StreamFifoLowLatency_2__1_};
  assign _zz_StreamFifoLowLatency_2__3_ = popPtr_willIncrement;
  assign _zz_StreamFifoLowLatency_2__4_ = {1'd0, _zz_StreamFifoLowLatency_2__3_};
  always @ (*) begin
    pushPtr_willIncrement = 1'b0;
    if(pushing)begin
      pushPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    pushPtr_willClear = 1'b0;
    popPtr_willClear = 1'b0;
    if(io_flush)begin
      pushPtr_willClear = 1'b1;
      popPtr_willClear = 1'b1;
    end
  end

  assign pushPtr_willOverflowIfInc = (pushPtr_value == (2'b11));
  assign pushPtr_willOverflow = (pushPtr_willOverflowIfInc && pushPtr_willIncrement);
  always @ (*) begin
    pushPtr_valueNext = (pushPtr_value + _zz_StreamFifoLowLatency_2__2_);
    if(pushPtr_willClear)begin
      pushPtr_valueNext = (2'b00);
    end
  end

  always @ (*) begin
    popPtr_willIncrement = 1'b0;
    if(popping)begin
      popPtr_willIncrement = 1'b1;
    end
  end

  assign popPtr_willOverflowIfInc = (popPtr_value == (2'b11));
  assign popPtr_willOverflow = (popPtr_willOverflowIfInc && popPtr_willIncrement);
  always @ (*) begin
    popPtr_valueNext = (popPtr_value + _zz_StreamFifoLowLatency_2__4_);
    if(popPtr_willClear)begin
      popPtr_valueNext = (2'b00);
    end
  end

  assign ptrMatch = (pushPtr_value == popPtr_value);
  assign empty = (ptrMatch && (! risingOccupancy));
  assign full = (ptrMatch && risingOccupancy);
  assign pushing = (io_push_valid && io_push_ready);
  assign popping = (io_pop_valid && io_pop_ready);
  assign io_push_ready = (! full);
  always @ (*) begin
    if((! empty))begin
      io_pop_valid = 1'b1;
    end else begin
      io_pop_valid = io_push_valid;
    end
  end

  assign ptrDif = (pushPtr_value - popPtr_value);
  assign io_occupancy = {(risingOccupancy && ptrMatch),ptrDif};
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      pushPtr_value <= (2'b00);
      popPtr_value <= (2'b00);
      risingOccupancy <= 1'b0;
    end else begin
      pushPtr_value <= pushPtr_valueNext;
      popPtr_value <= popPtr_valueNext;
      if((pushing != popping))begin
        risingOccupancy <= pushing;
      end
      if(io_flush)begin
        risingOccupancy <= 1'b0;
      end
    end
  end

endmodule

module StreamArbiter_2_ (
      input   io_inputs_0_valid,
      output  io_inputs_0_ready,
      input  [12:0] io_inputs_0_payload_addr,
      input  [2:0] io_inputs_0_payload_id,
      input  [7:0] io_inputs_0_payload_len,
      input  [2:0] io_inputs_0_payload_size,
      input  [1:0] io_inputs_0_payload_burst,
      input   io_inputs_0_payload_write,
      input   io_inputs_1_valid,
      output  io_inputs_1_ready,
      input  [12:0] io_inputs_1_payload_addr,
      input  [2:0] io_inputs_1_payload_id,
      input  [7:0] io_inputs_1_payload_len,
      input  [2:0] io_inputs_1_payload_size,
      input  [1:0] io_inputs_1_payload_burst,
      input   io_inputs_1_payload_write,
      output  io_output_valid,
      input   io_output_ready,
      output [12:0] io_output_payload_addr,
      output [2:0] io_output_payload_id,
      output [7:0] io_output_payload_len,
      output [2:0] io_output_payload_size,
      output [1:0] io_output_payload_burst,
      output  io_output_payload_write,
      output [0:0] io_chosen,
      output [1:0] io_chosenOH,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire [3:0] _zz_StreamArbiter_2__6_;
  wire [1:0] _zz_StreamArbiter_2__7_;
  wire [3:0] _zz_StreamArbiter_2__8_;
  wire [0:0] _zz_StreamArbiter_2__9_;
  wire [0:0] _zz_StreamArbiter_2__10_;
  reg  locked;
  wire  maskProposal_0;
  wire  maskProposal_1;
  reg  maskLocked_0;
  reg  maskLocked_1;
  wire  maskRouted_0;
  wire  maskRouted_1;
  wire [1:0] _zz_StreamArbiter_2__1_;
  wire [3:0] _zz_StreamArbiter_2__2_;
  wire [3:0] _zz_StreamArbiter_2__3_;
  wire [1:0] _zz_StreamArbiter_2__4_;
  wire  _zz_StreamArbiter_2__5_;
  assign _zz_StreamArbiter_2__6_ = (_zz_StreamArbiter_2__2_ - _zz_StreamArbiter_2__8_);
  assign _zz_StreamArbiter_2__7_ = {maskLocked_0,maskLocked_1};
  assign _zz_StreamArbiter_2__8_ = {2'd0, _zz_StreamArbiter_2__7_};
  assign _zz_StreamArbiter_2__9_ = _zz_StreamArbiter_2__4_[0 : 0];
  assign _zz_StreamArbiter_2__10_ = _zz_StreamArbiter_2__4_[1 : 1];
  assign maskRouted_0 = (locked ? maskLocked_0 : maskProposal_0);
  assign maskRouted_1 = (locked ? maskLocked_1 : maskProposal_1);
  assign _zz_StreamArbiter_2__1_ = {io_inputs_1_valid,io_inputs_0_valid};
  assign _zz_StreamArbiter_2__2_ = {_zz_StreamArbiter_2__1_,_zz_StreamArbiter_2__1_};
  assign _zz_StreamArbiter_2__3_ = (_zz_StreamArbiter_2__2_ & (~ _zz_StreamArbiter_2__6_));
  assign _zz_StreamArbiter_2__4_ = (_zz_StreamArbiter_2__3_[3 : 2] | _zz_StreamArbiter_2__3_[1 : 0]);
  assign maskProposal_0 = _zz_StreamArbiter_2__9_[0];
  assign maskProposal_1 = _zz_StreamArbiter_2__10_[0];
  assign io_output_valid = ((io_inputs_0_valid && maskRouted_0) || (io_inputs_1_valid && maskRouted_1));
  assign io_output_payload_addr = (maskRouted_0 ? io_inputs_0_payload_addr : io_inputs_1_payload_addr);
  assign io_output_payload_id = (maskRouted_0 ? io_inputs_0_payload_id : io_inputs_1_payload_id);
  assign io_output_payload_len = (maskRouted_0 ? io_inputs_0_payload_len : io_inputs_1_payload_len);
  assign io_output_payload_size = (maskRouted_0 ? io_inputs_0_payload_size : io_inputs_1_payload_size);
  assign io_output_payload_burst = (maskRouted_0 ? io_inputs_0_payload_burst : io_inputs_1_payload_burst);
  assign io_output_payload_write = (maskRouted_0 ? io_inputs_0_payload_write : io_inputs_1_payload_write);
  assign io_inputs_0_ready = (maskRouted_0 && io_output_ready);
  assign io_inputs_1_ready = (maskRouted_1 && io_output_ready);
  assign io_chosenOH = {maskRouted_1,maskRouted_0};
  assign _zz_StreamArbiter_2__5_ = io_chosenOH[1];
  assign io_chosen = _zz_StreamArbiter_2__5_;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      locked <= 1'b0;
      maskLocked_0 <= 1'b0;
      maskLocked_1 <= 1'b1;
    end else begin
      if(io_output_valid)begin
        maskLocked_0 <= maskRouted_0;
        maskLocked_1 <= maskRouted_1;
      end
      if(io_output_valid)begin
        locked <= 1'b1;
      end
      if((io_output_valid && io_output_ready))begin
        locked <= 1'b0;
      end
    end
  end

endmodule

module StreamFork_2_ (
      input   io_input_valid,
      output reg  io_input_ready,
      input  [12:0] io_input_payload_addr,
      input  [2:0] io_input_payload_id,
      input  [7:0] io_input_payload_len,
      input  [2:0] io_input_payload_size,
      input  [1:0] io_input_payload_burst,
      input   io_input_payload_write,
      output  io_outputs_0_valid,
      input   io_outputs_0_ready,
      output [12:0] io_outputs_0_payload_addr,
      output [2:0] io_outputs_0_payload_id,
      output [7:0] io_outputs_0_payload_len,
      output [2:0] io_outputs_0_payload_size,
      output [1:0] io_outputs_0_payload_burst,
      output  io_outputs_0_payload_write,
      output  io_outputs_1_valid,
      input   io_outputs_1_ready,
      output [12:0] io_outputs_1_payload_addr,
      output [2:0] io_outputs_1_payload_id,
      output [7:0] io_outputs_1_payload_len,
      output [2:0] io_outputs_1_payload_size,
      output [1:0] io_outputs_1_payload_burst,
      output  io_outputs_1_payload_write,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg  linkEnable_0;
  reg  linkEnable_1;
  always @ (*) begin
    io_input_ready = 1'b1;
    if(((! io_outputs_0_ready) && linkEnable_0))begin
      io_input_ready = 1'b0;
    end
    if(((! io_outputs_1_ready) && linkEnable_1))begin
      io_input_ready = 1'b0;
    end
  end

  assign io_outputs_0_valid = (io_input_valid && linkEnable_0);
  assign io_outputs_0_payload_addr = io_input_payload_addr;
  assign io_outputs_0_payload_id = io_input_payload_id;
  assign io_outputs_0_payload_len = io_input_payload_len;
  assign io_outputs_0_payload_size = io_input_payload_size;
  assign io_outputs_0_payload_burst = io_input_payload_burst;
  assign io_outputs_0_payload_write = io_input_payload_write;
  assign io_outputs_1_valid = (io_input_valid && linkEnable_1);
  assign io_outputs_1_payload_addr = io_input_payload_addr;
  assign io_outputs_1_payload_id = io_input_payload_id;
  assign io_outputs_1_payload_len = io_input_payload_len;
  assign io_outputs_1_payload_size = io_input_payload_size;
  assign io_outputs_1_payload_burst = io_input_payload_burst;
  assign io_outputs_1_payload_write = io_input_payload_write;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      linkEnable_0 <= 1'b1;
      linkEnable_1 <= 1'b1;
    end else begin
      if((io_outputs_0_valid && io_outputs_0_ready))begin
        linkEnable_0 <= 1'b0;
      end
      if((io_outputs_1_valid && io_outputs_1_ready))begin
        linkEnable_1 <= 1'b0;
      end
      if(io_input_ready)begin
        linkEnable_0 <= 1'b1;
        linkEnable_1 <= 1'b1;
      end
    end
  end

endmodule

module StreamFifoLowLatency_3_ (
      input   io_push_valid,
      output  io_push_ready,
      output reg  io_pop_valid,
      input   io_pop_ready,
      input   io_flush,
      output [2:0] io_occupancy,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire [0:0] _zz_StreamFifoLowLatency_3__1_;
  wire [1:0] _zz_StreamFifoLowLatency_3__2_;
  wire [0:0] _zz_StreamFifoLowLatency_3__3_;
  wire [1:0] _zz_StreamFifoLowLatency_3__4_;
  reg  pushPtr_willIncrement;
  reg  pushPtr_willClear;
  reg [1:0] pushPtr_valueNext;
  reg [1:0] pushPtr_value;
  wire  pushPtr_willOverflowIfInc;
  wire  pushPtr_willOverflow;
  reg  popPtr_willIncrement;
  reg  popPtr_willClear;
  reg [1:0] popPtr_valueNext;
  reg [1:0] popPtr_value;
  wire  popPtr_willOverflowIfInc;
  wire  popPtr_willOverflow;
  wire  ptrMatch;
  reg  risingOccupancy;
  wire  empty;
  wire  full;
  wire  pushing;
  wire  popping;
  wire [1:0] ptrDif;
  assign _zz_StreamFifoLowLatency_3__1_ = pushPtr_willIncrement;
  assign _zz_StreamFifoLowLatency_3__2_ = {1'd0, _zz_StreamFifoLowLatency_3__1_};
  assign _zz_StreamFifoLowLatency_3__3_ = popPtr_willIncrement;
  assign _zz_StreamFifoLowLatency_3__4_ = {1'd0, _zz_StreamFifoLowLatency_3__3_};
  always @ (*) begin
    pushPtr_willIncrement = 1'b0;
    if(pushing)begin
      pushPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    pushPtr_willClear = 1'b0;
    popPtr_willClear = 1'b0;
    if(io_flush)begin
      pushPtr_willClear = 1'b1;
      popPtr_willClear = 1'b1;
    end
  end

  assign pushPtr_willOverflowIfInc = (pushPtr_value == (2'b11));
  assign pushPtr_willOverflow = (pushPtr_willOverflowIfInc && pushPtr_willIncrement);
  always @ (*) begin
    pushPtr_valueNext = (pushPtr_value + _zz_StreamFifoLowLatency_3__2_);
    if(pushPtr_willClear)begin
      pushPtr_valueNext = (2'b00);
    end
  end

  always @ (*) begin
    popPtr_willIncrement = 1'b0;
    if(popping)begin
      popPtr_willIncrement = 1'b1;
    end
  end

  assign popPtr_willOverflowIfInc = (popPtr_value == (2'b11));
  assign popPtr_willOverflow = (popPtr_willOverflowIfInc && popPtr_willIncrement);
  always @ (*) begin
    popPtr_valueNext = (popPtr_value + _zz_StreamFifoLowLatency_3__4_);
    if(popPtr_willClear)begin
      popPtr_valueNext = (2'b00);
    end
  end

  assign ptrMatch = (pushPtr_value == popPtr_value);
  assign empty = (ptrMatch && (! risingOccupancy));
  assign full = (ptrMatch && risingOccupancy);
  assign pushing = (io_push_valid && io_push_ready);
  assign popping = (io_pop_valid && io_pop_ready);
  assign io_push_ready = (! full);
  always @ (*) begin
    if((! empty))begin
      io_pop_valid = 1'b1;
    end else begin
      io_pop_valid = io_push_valid;
    end
  end

  assign ptrDif = (pushPtr_value - popPtr_value);
  assign io_occupancy = {(risingOccupancy && ptrMatch),ptrDif};
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      pushPtr_value <= (2'b00);
      popPtr_value <= (2'b00);
      risingOccupancy <= 1'b0;
    end else begin
      pushPtr_value <= pushPtr_valueNext;
      popPtr_value <= popPtr_valueNext;
      if((pushing != popping))begin
        risingOccupancy <= pushing;
      end
      if(io_flush)begin
        risingOccupancy <= 1'b0;
      end
    end
  end

endmodule

module StreamArbiter_3_ (
      input   io_inputs_0_valid,
      output  io_inputs_0_ready,
      input  [19:0] io_inputs_0_payload_addr,
      input  [3:0] io_inputs_0_payload_id,
      input  [7:0] io_inputs_0_payload_len,
      input  [2:0] io_inputs_0_payload_size,
      input  [1:0] io_inputs_0_payload_burst,
      input   io_inputs_0_payload_write,
      output  io_output_valid,
      input   io_output_ready,
      output [19:0] io_output_payload_addr,
      output [3:0] io_output_payload_id,
      output [7:0] io_output_payload_len,
      output [2:0] io_output_payload_size,
      output [1:0] io_output_payload_burst,
      output  io_output_payload_write,
      output [0:0] io_chosenOH,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire [1:0] _zz_StreamArbiter_3__4_;
  wire [0:0] _zz_StreamArbiter_3__5_;
  wire [1:0] _zz_StreamArbiter_3__6_;
  wire [0:0] _zz_StreamArbiter_3__7_;
  wire [0:0] _zz_StreamArbiter_3__8_;
  reg  locked;
  wire  maskProposal_0;
  reg  maskLocked_0;
  wire  maskRouted_0;
  wire [0:0] _zz_StreamArbiter_3__1_;
  wire [1:0] _zz_StreamArbiter_3__2_;
  wire [1:0] _zz_StreamArbiter_3__3_;
  assign _zz_StreamArbiter_3__4_ = (_zz_StreamArbiter_3__2_ - _zz_StreamArbiter_3__6_);
  assign _zz_StreamArbiter_3__5_ = maskLocked_0;
  assign _zz_StreamArbiter_3__6_ = {1'd0, _zz_StreamArbiter_3__5_};
  assign _zz_StreamArbiter_3__7_ = _zz_StreamArbiter_3__8_[0 : 0];
  assign _zz_StreamArbiter_3__8_ = (_zz_StreamArbiter_3__3_[1 : 1] | _zz_StreamArbiter_3__3_[0 : 0]);
  assign maskRouted_0 = (locked ? maskLocked_0 : maskProposal_0);
  assign _zz_StreamArbiter_3__1_ = io_inputs_0_valid;
  assign _zz_StreamArbiter_3__2_ = {_zz_StreamArbiter_3__1_,_zz_StreamArbiter_3__1_};
  assign _zz_StreamArbiter_3__3_ = (_zz_StreamArbiter_3__2_ & (~ _zz_StreamArbiter_3__4_));
  assign maskProposal_0 = _zz_StreamArbiter_3__7_[0];
  assign io_output_valid = (io_inputs_0_valid && maskRouted_0);
  assign io_output_payload_addr = io_inputs_0_payload_addr;
  assign io_output_payload_id = io_inputs_0_payload_id;
  assign io_output_payload_len = io_inputs_0_payload_len;
  assign io_output_payload_size = io_inputs_0_payload_size;
  assign io_output_payload_burst = io_inputs_0_payload_burst;
  assign io_output_payload_write = io_inputs_0_payload_write;
  assign io_inputs_0_ready = (maskRouted_0 && io_output_ready);
  assign io_chosenOH = maskRouted_0;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      locked <= 1'b0;
      maskLocked_0 <= 1'b1;
    end else begin
      if(io_output_valid)begin
        maskLocked_0 <= maskRouted_0;
      end
      if(io_output_valid)begin
        locked <= 1'b1;
      end
      if((io_output_valid && io_output_ready))begin
        locked <= 1'b0;
      end
    end
  end

endmodule

module StreamFork_3_ (
      input   io_input_valid,
      output reg  io_input_ready,
      input  [19:0] io_input_payload_addr,
      input  [3:0] io_input_payload_id,
      input  [7:0] io_input_payload_len,
      input  [2:0] io_input_payload_size,
      input  [1:0] io_input_payload_burst,
      input   io_input_payload_write,
      output  io_outputs_0_valid,
      input   io_outputs_0_ready,
      output [19:0] io_outputs_0_payload_addr,
      output [3:0] io_outputs_0_payload_id,
      output [7:0] io_outputs_0_payload_len,
      output [2:0] io_outputs_0_payload_size,
      output [1:0] io_outputs_0_payload_burst,
      output  io_outputs_0_payload_write,
      output  io_outputs_1_valid,
      input   io_outputs_1_ready,
      output [19:0] io_outputs_1_payload_addr,
      output [3:0] io_outputs_1_payload_id,
      output [7:0] io_outputs_1_payload_len,
      output [2:0] io_outputs_1_payload_size,
      output [1:0] io_outputs_1_payload_burst,
      output  io_outputs_1_payload_write,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg  linkEnable_0;
  reg  linkEnable_1;
  always @ (*) begin
    io_input_ready = 1'b1;
    if(((! io_outputs_0_ready) && linkEnable_0))begin
      io_input_ready = 1'b0;
    end
    if(((! io_outputs_1_ready) && linkEnable_1))begin
      io_input_ready = 1'b0;
    end
  end

  assign io_outputs_0_valid = (io_input_valid && linkEnable_0);
  assign io_outputs_0_payload_addr = io_input_payload_addr;
  assign io_outputs_0_payload_id = io_input_payload_id;
  assign io_outputs_0_payload_len = io_input_payload_len;
  assign io_outputs_0_payload_size = io_input_payload_size;
  assign io_outputs_0_payload_burst = io_input_payload_burst;
  assign io_outputs_0_payload_write = io_input_payload_write;
  assign io_outputs_1_valid = (io_input_valid && linkEnable_1);
  assign io_outputs_1_payload_addr = io_input_payload_addr;
  assign io_outputs_1_payload_id = io_input_payload_id;
  assign io_outputs_1_payload_len = io_input_payload_len;
  assign io_outputs_1_payload_size = io_input_payload_size;
  assign io_outputs_1_payload_burst = io_input_payload_burst;
  assign io_outputs_1_payload_write = io_input_payload_write;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      linkEnable_0 <= 1'b1;
      linkEnable_1 <= 1'b1;
    end else begin
      if((io_outputs_0_valid && io_outputs_0_ready))begin
        linkEnable_0 <= 1'b0;
      end
      if((io_outputs_1_valid && io_outputs_1_ready))begin
        linkEnable_1 <= 1'b0;
      end
      if(io_input_ready)begin
        linkEnable_0 <= 1'b1;
        linkEnable_1 <= 1'b1;
      end
    end
  end

endmodule

module StreamFifoLowLatency_4_ (
      input   io_push_valid,
      output  io_push_ready,
      output reg  io_pop_valid,
      input   io_pop_ready,
      input   io_flush,
      output [2:0] io_occupancy,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire [0:0] _zz_StreamFifoLowLatency_4__1_;
  wire [1:0] _zz_StreamFifoLowLatency_4__2_;
  wire [0:0] _zz_StreamFifoLowLatency_4__3_;
  wire [1:0] _zz_StreamFifoLowLatency_4__4_;
  reg  pushPtr_willIncrement;
  reg  pushPtr_willClear;
  reg [1:0] pushPtr_valueNext;
  reg [1:0] pushPtr_value;
  wire  pushPtr_willOverflowIfInc;
  wire  pushPtr_willOverflow;
  reg  popPtr_willIncrement;
  reg  popPtr_willClear;
  reg [1:0] popPtr_valueNext;
  reg [1:0] popPtr_value;
  wire  popPtr_willOverflowIfInc;
  wire  popPtr_willOverflow;
  wire  ptrMatch;
  reg  risingOccupancy;
  wire  empty;
  wire  full;
  wire  pushing;
  wire  popping;
  wire [1:0] ptrDif;
  assign _zz_StreamFifoLowLatency_4__1_ = pushPtr_willIncrement;
  assign _zz_StreamFifoLowLatency_4__2_ = {1'd0, _zz_StreamFifoLowLatency_4__1_};
  assign _zz_StreamFifoLowLatency_4__3_ = popPtr_willIncrement;
  assign _zz_StreamFifoLowLatency_4__4_ = {1'd0, _zz_StreamFifoLowLatency_4__3_};
  always @ (*) begin
    pushPtr_willIncrement = 1'b0;
    if(pushing)begin
      pushPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    pushPtr_willClear = 1'b0;
    popPtr_willClear = 1'b0;
    if(io_flush)begin
      pushPtr_willClear = 1'b1;
      popPtr_willClear = 1'b1;
    end
  end

  assign pushPtr_willOverflowIfInc = (pushPtr_value == (2'b11));
  assign pushPtr_willOverflow = (pushPtr_willOverflowIfInc && pushPtr_willIncrement);
  always @ (*) begin
    pushPtr_valueNext = (pushPtr_value + _zz_StreamFifoLowLatency_4__2_);
    if(pushPtr_willClear)begin
      pushPtr_valueNext = (2'b00);
    end
  end

  always @ (*) begin
    popPtr_willIncrement = 1'b0;
    if(popping)begin
      popPtr_willIncrement = 1'b1;
    end
  end

  assign popPtr_willOverflowIfInc = (popPtr_value == (2'b11));
  assign popPtr_willOverflow = (popPtr_willOverflowIfInc && popPtr_willIncrement);
  always @ (*) begin
    popPtr_valueNext = (popPtr_value + _zz_StreamFifoLowLatency_4__4_);
    if(popPtr_willClear)begin
      popPtr_valueNext = (2'b00);
    end
  end

  assign ptrMatch = (pushPtr_value == popPtr_value);
  assign empty = (ptrMatch && (! risingOccupancy));
  assign full = (ptrMatch && risingOccupancy);
  assign pushing = (io_push_valid && io_push_ready);
  assign popping = (io_pop_valid && io_pop_ready);
  assign io_push_ready = (! full);
  always @ (*) begin
    if((! empty))begin
      io_pop_valid = 1'b1;
    end else begin
      io_pop_valid = io_push_valid;
    end
  end

  assign ptrDif = (pushPtr_value - popPtr_value);
  assign io_occupancy = {(risingOccupancy && ptrMatch),ptrDif};
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      pushPtr_value <= (2'b00);
      popPtr_value <= (2'b00);
      risingOccupancy <= 1'b0;
    end else begin
      pushPtr_value <= pushPtr_valueNext;
      popPtr_value <= popPtr_valueNext;
      if((pushing != popping))begin
        risingOccupancy <= pushing;
      end
      if(io_flush)begin
        risingOccupancy <= 1'b0;
      end
    end
  end

endmodule

module StreamFifoCC (
      input   io_push_valid,
      output  io_push_ready,
      input  [7:0] io_push_payload,
      output  io_pop_valid,
      input   io_pop_ready,
      output [7:0] io_pop_payload,
      output [8:0] io_pushOccupancy,
      output [8:0] io_popOccupancy,
      input   toplevel_main_clk,
      input   toplevel_main_reset_,
      input   jtag_TCK,
      input   jtag_RESET);
  wire [8:0] _zz_StreamFifoCC_19_;
  wire [8:0] _zz_StreamFifoCC_20_;
  reg [7:0] _zz_StreamFifoCC_21_;
  wire [8:0] bufferCC_19__io_dataOut;
  wire [8:0] bufferCC_20__io_dataOut;
  wire [0:0] _zz_StreamFifoCC_22_;
  wire [8:0] _zz_StreamFifoCC_23_;
  wire [8:0] _zz_StreamFifoCC_24_;
  wire [7:0] _zz_StreamFifoCC_25_;
  wire [0:0] _zz_StreamFifoCC_26_;
  wire [8:0] _zz_StreamFifoCC_27_;
  wire [8:0] _zz_StreamFifoCC_28_;
  wire [7:0] _zz_StreamFifoCC_29_;
  wire  _zz_StreamFifoCC_30_;
  reg  _zz_StreamFifoCC_1_;
  wire [8:0] popToPushGray;
  wire [8:0] pushToPopGray;
  reg  pushCC_pushPtr_willIncrement;
  wire  pushCC_pushPtr_willClear;
  reg [8:0] pushCC_pushPtr_valueNext;
  reg [8:0] pushCC_pushPtr_value;
  wire  pushCC_pushPtr_willOverflowIfInc;
  wire  pushCC_pushPtr_willOverflow;
  reg [8:0] pushCC_pushPtrGray;
  wire [8:0] pushCC_popPtrGray;
  wire  pushCC_full;
  wire  _zz_StreamFifoCC_2_;
  wire  _zz_StreamFifoCC_3_;
  wire  _zz_StreamFifoCC_4_;
  wire  _zz_StreamFifoCC_5_;
  wire  _zz_StreamFifoCC_6_;
  wire  _zz_StreamFifoCC_7_;
  wire  _zz_StreamFifoCC_8_;
  wire  _zz_StreamFifoCC_9_;
  reg  popCC_popPtr_willIncrement;
  wire  popCC_popPtr_willClear;
  reg [8:0] popCC_popPtr_valueNext;
  reg [8:0] popCC_popPtr_value;
  wire  popCC_popPtr_willOverflowIfInc;
  wire  popCC_popPtr_willOverflow;
  reg [8:0] popCC_popPtrGray;
  wire [8:0] popCC_pushPtrGray;
  wire  popCC_empty;
  wire [8:0] _zz_StreamFifoCC_10_;
  wire  _zz_StreamFifoCC_11_;
  wire  _zz_StreamFifoCC_12_;
  wire  _zz_StreamFifoCC_13_;
  wire  _zz_StreamFifoCC_14_;
  wire  _zz_StreamFifoCC_15_;
  wire  _zz_StreamFifoCC_16_;
  wire  _zz_StreamFifoCC_17_;
  wire  _zz_StreamFifoCC_18_;
  reg [7:0] ram [0:255];
  assign _zz_StreamFifoCC_22_ = pushCC_pushPtr_willIncrement;
  assign _zz_StreamFifoCC_23_ = {8'd0, _zz_StreamFifoCC_22_};
  assign _zz_StreamFifoCC_24_ = (pushCC_pushPtr_valueNext >>> (1'b1));
  assign _zz_StreamFifoCC_25_ = pushCC_pushPtr_value[7:0];
  assign _zz_StreamFifoCC_26_ = popCC_popPtr_willIncrement;
  assign _zz_StreamFifoCC_27_ = {8'd0, _zz_StreamFifoCC_26_};
  assign _zz_StreamFifoCC_28_ = (popCC_popPtr_valueNext >>> (1'b1));
  assign _zz_StreamFifoCC_29_ = _zz_StreamFifoCC_10_[7:0];
  assign _zz_StreamFifoCC_30_ = 1'b1;
  always @ (posedge toplevel_main_clk) begin
    if(_zz_StreamFifoCC_1_) begin
      ram[_zz_StreamFifoCC_25_] <= io_push_payload;
    end
  end

  always @ (posedge jtag_TCK) begin
  end

  always @ (posedge jtag_TCK) begin
    if(_zz_StreamFifoCC_30_) begin
      _zz_StreamFifoCC_21_ <= ram[_zz_StreamFifoCC_29_];
    end
  end

  BufferCC_1_ bufferCC_19_ ( 
    .io_initial(_zz_StreamFifoCC_19_),
    .io_dataIn(popToPushGray),
    .io_dataOut(bufferCC_19__io_dataOut),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  BufferCC_2_ bufferCC_20_ ( 
    .io_initial(_zz_StreamFifoCC_20_),
    .io_dataIn(pushToPopGray),
    .io_dataOut(bufferCC_20__io_dataOut),
    .jtag_TCK(jtag_TCK),
    .jtag_RESET(jtag_RESET) 
  );
  always @ (*) begin
    _zz_StreamFifoCC_1_ = 1'b0;
    pushCC_pushPtr_willIncrement = 1'b0;
    if((io_push_valid && io_push_ready))begin
      _zz_StreamFifoCC_1_ = 1'b1;
      pushCC_pushPtr_willIncrement = 1'b1;
    end
  end

  assign pushCC_pushPtr_willClear = 1'b0;
  assign pushCC_pushPtr_willOverflowIfInc = (pushCC_pushPtr_value == (9'b111111111));
  assign pushCC_pushPtr_willOverflow = (pushCC_pushPtr_willOverflowIfInc && pushCC_pushPtr_willIncrement);
  always @ (*) begin
    pushCC_pushPtr_valueNext = (pushCC_pushPtr_value + _zz_StreamFifoCC_23_);
    if(pushCC_pushPtr_willClear)begin
      pushCC_pushPtr_valueNext = (9'b000000000);
    end
  end

  assign _zz_StreamFifoCC_19_ = (9'b000000000);
  assign pushCC_popPtrGray = bufferCC_19__io_dataOut;
  assign pushCC_full = ((pushCC_pushPtrGray[8 : 7] == (~ pushCC_popPtrGray[8 : 7])) && (pushCC_pushPtrGray[6 : 0] == pushCC_popPtrGray[6 : 0]));
  assign io_push_ready = (! pushCC_full);
  assign _zz_StreamFifoCC_2_ = (pushCC_popPtrGray[1] ^ _zz_StreamFifoCC_3_);
  assign _zz_StreamFifoCC_3_ = (pushCC_popPtrGray[2] ^ _zz_StreamFifoCC_4_);
  assign _zz_StreamFifoCC_4_ = (pushCC_popPtrGray[3] ^ _zz_StreamFifoCC_5_);
  assign _zz_StreamFifoCC_5_ = (pushCC_popPtrGray[4] ^ _zz_StreamFifoCC_6_);
  assign _zz_StreamFifoCC_6_ = (pushCC_popPtrGray[5] ^ _zz_StreamFifoCC_7_);
  assign _zz_StreamFifoCC_7_ = (pushCC_popPtrGray[6] ^ _zz_StreamFifoCC_8_);
  assign _zz_StreamFifoCC_8_ = (pushCC_popPtrGray[7] ^ _zz_StreamFifoCC_9_);
  assign _zz_StreamFifoCC_9_ = pushCC_popPtrGray[8];
  assign io_pushOccupancy = (pushCC_pushPtr_value - {_zz_StreamFifoCC_9_,{_zz_StreamFifoCC_8_,{_zz_StreamFifoCC_7_,{_zz_StreamFifoCC_6_,{_zz_StreamFifoCC_5_,{_zz_StreamFifoCC_4_,{_zz_StreamFifoCC_3_,{_zz_StreamFifoCC_2_,(pushCC_popPtrGray[0] ^ _zz_StreamFifoCC_2_)}}}}}}}});
  always @ (*) begin
    popCC_popPtr_willIncrement = 1'b0;
    if((io_pop_valid && io_pop_ready))begin
      popCC_popPtr_willIncrement = 1'b1;
    end
  end

  assign popCC_popPtr_willClear = 1'b0;
  assign popCC_popPtr_willOverflowIfInc = (popCC_popPtr_value == (9'b111111111));
  assign popCC_popPtr_willOverflow = (popCC_popPtr_willOverflowIfInc && popCC_popPtr_willIncrement);
  always @ (*) begin
    popCC_popPtr_valueNext = (popCC_popPtr_value + _zz_StreamFifoCC_27_);
    if(popCC_popPtr_willClear)begin
      popCC_popPtr_valueNext = (9'b000000000);
    end
  end

  assign _zz_StreamFifoCC_20_ = (9'b000000000);
  assign popCC_pushPtrGray = bufferCC_20__io_dataOut;
  assign popCC_empty = (popCC_popPtrGray == popCC_pushPtrGray);
  assign io_pop_valid = (! popCC_empty);
  assign _zz_StreamFifoCC_10_ = popCC_popPtr_valueNext;
  assign io_pop_payload = _zz_StreamFifoCC_21_;
  assign _zz_StreamFifoCC_11_ = (popCC_pushPtrGray[1] ^ _zz_StreamFifoCC_12_);
  assign _zz_StreamFifoCC_12_ = (popCC_pushPtrGray[2] ^ _zz_StreamFifoCC_13_);
  assign _zz_StreamFifoCC_13_ = (popCC_pushPtrGray[3] ^ _zz_StreamFifoCC_14_);
  assign _zz_StreamFifoCC_14_ = (popCC_pushPtrGray[4] ^ _zz_StreamFifoCC_15_);
  assign _zz_StreamFifoCC_15_ = (popCC_pushPtrGray[5] ^ _zz_StreamFifoCC_16_);
  assign _zz_StreamFifoCC_16_ = (popCC_pushPtrGray[6] ^ _zz_StreamFifoCC_17_);
  assign _zz_StreamFifoCC_17_ = (popCC_pushPtrGray[7] ^ _zz_StreamFifoCC_18_);
  assign _zz_StreamFifoCC_18_ = popCC_pushPtrGray[8];
  assign io_popOccupancy = ({_zz_StreamFifoCC_18_,{_zz_StreamFifoCC_17_,{_zz_StreamFifoCC_16_,{_zz_StreamFifoCC_15_,{_zz_StreamFifoCC_14_,{_zz_StreamFifoCC_13_,{_zz_StreamFifoCC_12_,{_zz_StreamFifoCC_11_,(popCC_pushPtrGray[0] ^ _zz_StreamFifoCC_11_)}}}}}}}} - popCC_popPtr_value);
  assign pushToPopGray = pushCC_pushPtrGray;
  assign popToPushGray = popCC_popPtrGray;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      pushCC_pushPtr_value <= (9'b000000000);
      pushCC_pushPtrGray <= (9'b000000000);
    end else begin
      pushCC_pushPtr_value <= pushCC_pushPtr_valueNext;
      pushCC_pushPtrGray <= (_zz_StreamFifoCC_24_ ^ pushCC_pushPtr_valueNext);
    end
  end

  always @ (posedge jtag_TCK or posedge jtag_RESET) begin
    if (jtag_RESET) begin
      popCC_popPtr_value <= (9'b000000000);
      popCC_popPtrGray <= (9'b000000000);
    end else begin
      popCC_popPtr_value <= popCC_popPtr_valueNext;
      popCC_popPtrGray <= (_zz_StreamFifoCC_28_ ^ popCC_popPtr_valueNext);
    end
  end

endmodule

module StreamFifoCC_1_ (
      input   io_push_valid,
      output  io_push_ready,
      input  [7:0] io_push_payload,
      output  io_pop_valid,
      input   io_pop_ready,
      output [7:0] io_pop_payload,
      output [5:0] io_pushOccupancy,
      output [5:0] io_popOccupancy,
      input   jtag_TCK,
      input   jtag_RESET,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire [5:0] _zz_StreamFifoCC_1__13_;
  wire [5:0] _zz_StreamFifoCC_1__14_;
  reg [7:0] _zz_StreamFifoCC_1__15_;
  wire [5:0] bufferCC_19__io_dataOut;
  wire [5:0] bufferCC_20__io_dataOut;
  wire [0:0] _zz_StreamFifoCC_1__16_;
  wire [5:0] _zz_StreamFifoCC_1__17_;
  wire [5:0] _zz_StreamFifoCC_1__18_;
  wire [4:0] _zz_StreamFifoCC_1__19_;
  wire [0:0] _zz_StreamFifoCC_1__20_;
  wire [5:0] _zz_StreamFifoCC_1__21_;
  wire [5:0] _zz_StreamFifoCC_1__22_;
  wire [4:0] _zz_StreamFifoCC_1__23_;
  wire  _zz_StreamFifoCC_1__24_;
  reg  _zz_StreamFifoCC_1__1_;
  wire [5:0] popToPushGray;
  wire [5:0] pushToPopGray;
  reg  pushCC_pushPtr_willIncrement;
  wire  pushCC_pushPtr_willClear;
  reg [5:0] pushCC_pushPtr_valueNext;
  reg [5:0] pushCC_pushPtr_value;
  wire  pushCC_pushPtr_willOverflowIfInc;
  wire  pushCC_pushPtr_willOverflow;
  reg [5:0] pushCC_pushPtrGray;
  wire [5:0] pushCC_popPtrGray;
  wire  pushCC_full;
  wire  _zz_StreamFifoCC_1__2_;
  wire  _zz_StreamFifoCC_1__3_;
  wire  _zz_StreamFifoCC_1__4_;
  wire  _zz_StreamFifoCC_1__5_;
  wire  _zz_StreamFifoCC_1__6_;
  reg  popCC_popPtr_willIncrement;
  wire  popCC_popPtr_willClear;
  reg [5:0] popCC_popPtr_valueNext;
  reg [5:0] popCC_popPtr_value;
  wire  popCC_popPtr_willOverflowIfInc;
  wire  popCC_popPtr_willOverflow;
  reg [5:0] popCC_popPtrGray;
  wire [5:0] popCC_pushPtrGray;
  wire  popCC_empty;
  wire [5:0] _zz_StreamFifoCC_1__7_;
  wire  _zz_StreamFifoCC_1__8_;
  wire  _zz_StreamFifoCC_1__9_;
  wire  _zz_StreamFifoCC_1__10_;
  wire  _zz_StreamFifoCC_1__11_;
  wire  _zz_StreamFifoCC_1__12_;
  reg [7:0] ram [0:31];
  assign _zz_StreamFifoCC_1__16_ = pushCC_pushPtr_willIncrement;
  assign _zz_StreamFifoCC_1__17_ = {5'd0, _zz_StreamFifoCC_1__16_};
  assign _zz_StreamFifoCC_1__18_ = (pushCC_pushPtr_valueNext >>> (1'b1));
  assign _zz_StreamFifoCC_1__19_ = pushCC_pushPtr_value[4:0];
  assign _zz_StreamFifoCC_1__20_ = popCC_popPtr_willIncrement;
  assign _zz_StreamFifoCC_1__21_ = {5'd0, _zz_StreamFifoCC_1__20_};
  assign _zz_StreamFifoCC_1__22_ = (popCC_popPtr_valueNext >>> (1'b1));
  assign _zz_StreamFifoCC_1__23_ = _zz_StreamFifoCC_1__7_[4:0];
  assign _zz_StreamFifoCC_1__24_ = 1'b1;
  always @ (posedge jtag_TCK) begin
    if(_zz_StreamFifoCC_1__1_) begin
      ram[_zz_StreamFifoCC_1__19_] <= io_push_payload;
    end
  end

  always @ (posedge toplevel_main_clk) begin
  end

  always @ (posedge toplevel_main_clk) begin
    if(_zz_StreamFifoCC_1__24_) begin
      _zz_StreamFifoCC_1__15_ <= ram[_zz_StreamFifoCC_1__23_];
    end
  end

  BufferCC_3_ bufferCC_19_ ( 
    .io_initial(_zz_StreamFifoCC_1__13_),
    .io_dataIn(popToPushGray),
    .io_dataOut(bufferCC_19__io_dataOut),
    .jtag_TCK(jtag_TCK),
    .jtag_RESET(jtag_RESET) 
  );
  BufferCC_4_ bufferCC_20_ ( 
    .io_initial(_zz_StreamFifoCC_1__14_),
    .io_dataIn(pushToPopGray),
    .io_dataOut(bufferCC_20__io_dataOut),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  always @ (*) begin
    _zz_StreamFifoCC_1__1_ = 1'b0;
    pushCC_pushPtr_willIncrement = 1'b0;
    if((io_push_valid && io_push_ready))begin
      _zz_StreamFifoCC_1__1_ = 1'b1;
      pushCC_pushPtr_willIncrement = 1'b1;
    end
  end

  assign pushCC_pushPtr_willClear = 1'b0;
  assign pushCC_pushPtr_willOverflowIfInc = (pushCC_pushPtr_value == (6'b111111));
  assign pushCC_pushPtr_willOverflow = (pushCC_pushPtr_willOverflowIfInc && pushCC_pushPtr_willIncrement);
  always @ (*) begin
    pushCC_pushPtr_valueNext = (pushCC_pushPtr_value + _zz_StreamFifoCC_1__17_);
    if(pushCC_pushPtr_willClear)begin
      pushCC_pushPtr_valueNext = (6'b000000);
    end
  end

  assign _zz_StreamFifoCC_1__13_ = (6'b000000);
  assign pushCC_popPtrGray = bufferCC_19__io_dataOut;
  assign pushCC_full = ((pushCC_pushPtrGray[5 : 4] == (~ pushCC_popPtrGray[5 : 4])) && (pushCC_pushPtrGray[3 : 0] == pushCC_popPtrGray[3 : 0]));
  assign io_push_ready = (! pushCC_full);
  assign _zz_StreamFifoCC_1__2_ = (pushCC_popPtrGray[1] ^ _zz_StreamFifoCC_1__3_);
  assign _zz_StreamFifoCC_1__3_ = (pushCC_popPtrGray[2] ^ _zz_StreamFifoCC_1__4_);
  assign _zz_StreamFifoCC_1__4_ = (pushCC_popPtrGray[3] ^ _zz_StreamFifoCC_1__5_);
  assign _zz_StreamFifoCC_1__5_ = (pushCC_popPtrGray[4] ^ _zz_StreamFifoCC_1__6_);
  assign _zz_StreamFifoCC_1__6_ = pushCC_popPtrGray[5];
  assign io_pushOccupancy = (pushCC_pushPtr_value - {_zz_StreamFifoCC_1__6_,{_zz_StreamFifoCC_1__5_,{_zz_StreamFifoCC_1__4_,{_zz_StreamFifoCC_1__3_,{_zz_StreamFifoCC_1__2_,(pushCC_popPtrGray[0] ^ _zz_StreamFifoCC_1__2_)}}}}});
  always @ (*) begin
    popCC_popPtr_willIncrement = 1'b0;
    if((io_pop_valid && io_pop_ready))begin
      popCC_popPtr_willIncrement = 1'b1;
    end
  end

  assign popCC_popPtr_willClear = 1'b0;
  assign popCC_popPtr_willOverflowIfInc = (popCC_popPtr_value == (6'b111111));
  assign popCC_popPtr_willOverflow = (popCC_popPtr_willOverflowIfInc && popCC_popPtr_willIncrement);
  always @ (*) begin
    popCC_popPtr_valueNext = (popCC_popPtr_value + _zz_StreamFifoCC_1__21_);
    if(popCC_popPtr_willClear)begin
      popCC_popPtr_valueNext = (6'b000000);
    end
  end

  assign _zz_StreamFifoCC_1__14_ = (6'b000000);
  assign popCC_pushPtrGray = bufferCC_20__io_dataOut;
  assign popCC_empty = (popCC_popPtrGray == popCC_pushPtrGray);
  assign io_pop_valid = (! popCC_empty);
  assign _zz_StreamFifoCC_1__7_ = popCC_popPtr_valueNext;
  assign io_pop_payload = _zz_StreamFifoCC_1__15_;
  assign _zz_StreamFifoCC_1__8_ = (popCC_pushPtrGray[1] ^ _zz_StreamFifoCC_1__9_);
  assign _zz_StreamFifoCC_1__9_ = (popCC_pushPtrGray[2] ^ _zz_StreamFifoCC_1__10_);
  assign _zz_StreamFifoCC_1__10_ = (popCC_pushPtrGray[3] ^ _zz_StreamFifoCC_1__11_);
  assign _zz_StreamFifoCC_1__11_ = (popCC_pushPtrGray[4] ^ _zz_StreamFifoCC_1__12_);
  assign _zz_StreamFifoCC_1__12_ = popCC_pushPtrGray[5];
  assign io_popOccupancy = ({_zz_StreamFifoCC_1__12_,{_zz_StreamFifoCC_1__11_,{_zz_StreamFifoCC_1__10_,{_zz_StreamFifoCC_1__9_,{_zz_StreamFifoCC_1__8_,(popCC_pushPtrGray[0] ^ _zz_StreamFifoCC_1__8_)}}}}} - popCC_popPtr_value);
  assign pushToPopGray = pushCC_pushPtrGray;
  assign popToPushGray = popCC_popPtrGray;
  always @ (posedge jtag_TCK or posedge jtag_RESET) begin
    if (jtag_RESET) begin
      pushCC_pushPtr_value <= (6'b000000);
      pushCC_pushPtrGray <= (6'b000000);
    end else begin
      pushCC_pushPtr_value <= pushCC_pushPtr_valueNext;
      pushCC_pushPtrGray <= (_zz_StreamFifoCC_1__18_ ^ pushCC_pushPtr_valueNext);
    end
  end

  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      popCC_popPtr_value <= (6'b000000);
      popCC_popPtrGray <= (6'b000000);
    end else begin
      popCC_popPtr_value <= popCC_popPtr_valueNext;
      popCC_popPtrGray <= (_zz_StreamFifoCC_1__22_ ^ popCC_popPtr_valueNext);
    end
  end

endmodule

module BufferCC_5_ (
      input  [11:0] io_initial,
      input  [11:0] io_dataIn,
      output [11:0] io_dataOut,
      input   u_gmii_rx_io_rx_clk);
  reg [11:0] buffers_0 = (12'b000000000000);
  reg [11:0] buffers_1 = (12'b000000000000);
  assign io_dataOut = buffers_1;
  always @ (posedge u_gmii_rx_io_rx_clk) begin
    buffers_0 <= io_dataIn;
    buffers_1 <= buffers_0;
  end

endmodule

module BufferCC_6_ (
      input  [11:0] io_initial,
      input  [11:0] io_dataIn,
      output [11:0] io_dataOut,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg [11:0] buffers_0;
  reg [11:0] buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      buffers_0 <= io_initial;
      buffers_1 <= io_initial;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end

endmodule

module VexRiscv (
      output  iBus_cmd_valid,
      input   iBus_cmd_ready,
      output [31:0] iBus_cmd_payload_pc,
      input   iBus_rsp_valid,
      input   iBus_rsp_payload_error,
      input  [31:0] iBus_rsp_payload_inst,
      input   timerInterrupt,
      input   externalInterrupt,
      input   debug_bus_cmd_valid,
      output reg  debug_bus_cmd_ready,
      input   debug_bus_cmd_payload_wr,
      input  [7:0] debug_bus_cmd_payload_address,
      input  [31:0] debug_bus_cmd_payload_data,
      output reg [31:0] debug_bus_rsp_data,
      output  debug_resetOut,
      output  dBus_cmd_valid,
      input   dBus_cmd_ready,
      output  dBus_cmd_payload_wr,
      output [31:0] dBus_cmd_payload_address,
      output [31:0] dBus_cmd_payload_data,
      output [1:0] dBus_cmd_payload_size,
      input   dBus_rsp_ready,
      input   dBus_rsp_error,
      input  [31:0] dBus_rsp_data,
      input   toplevel_main_clk,
      input   toplevel_main_reset_,
      input   _zz_VexRiscv_151_);
  wire  _zz_VexRiscv_152_;
  reg [31:0] _zz_VexRiscv_153_;
  reg [31:0] _zz_VexRiscv_154_;
  wire  IBusSimplePlugin_rspJoin_rspBuffer_c_io_push_ready;
  wire  IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid;
  wire  IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error;
  wire [31:0] IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst;
  wire [0:0] IBusSimplePlugin_rspJoin_rspBuffer_c_io_occupancy;
  wire  _zz_VexRiscv_155_;
  wire  _zz_VexRiscv_156_;
  wire  _zz_VexRiscv_157_;
  wire  _zz_VexRiscv_158_;
  wire  _zz_VexRiscv_159_;
  wire  _zz_VexRiscv_160_;
  wire  _zz_VexRiscv_161_;
  wire  _zz_VexRiscv_162_;
  wire [5:0] _zz_VexRiscv_163_;
  wire [1:0] _zz_VexRiscv_164_;
  wire [1:0] _zz_VexRiscv_165_;
  wire  _zz_VexRiscv_166_;
  wire [1:0] _zz_VexRiscv_167_;
  wire [1:0] _zz_VexRiscv_168_;
  wire [2:0] _zz_VexRiscv_169_;
  wire [31:0] _zz_VexRiscv_170_;
  wire [2:0] _zz_VexRiscv_171_;
  wire [0:0] _zz_VexRiscv_172_;
  wire [2:0] _zz_VexRiscv_173_;
  wire [0:0] _zz_VexRiscv_174_;
  wire [2:0] _zz_VexRiscv_175_;
  wire [0:0] _zz_VexRiscv_176_;
  wire [2:0] _zz_VexRiscv_177_;
  wire [0:0] _zz_VexRiscv_178_;
  wire [0:0] _zz_VexRiscv_179_;
  wire [0:0] _zz_VexRiscv_180_;
  wire [0:0] _zz_VexRiscv_181_;
  wire [0:0] _zz_VexRiscv_182_;
  wire [0:0] _zz_VexRiscv_183_;
  wire [0:0] _zz_VexRiscv_184_;
  wire [0:0] _zz_VexRiscv_185_;
  wire [0:0] _zz_VexRiscv_186_;
  wire [0:0] _zz_VexRiscv_187_;
  wire [0:0] _zz_VexRiscv_188_;
  wire [2:0] _zz_VexRiscv_189_;
  wire [4:0] _zz_VexRiscv_190_;
  wire [11:0] _zz_VexRiscv_191_;
  wire [11:0] _zz_VexRiscv_192_;
  wire [31:0] _zz_VexRiscv_193_;
  wire [31:0] _zz_VexRiscv_194_;
  wire [31:0] _zz_VexRiscv_195_;
  wire [31:0] _zz_VexRiscv_196_;
  wire [1:0] _zz_VexRiscv_197_;
  wire [31:0] _zz_VexRiscv_198_;
  wire [1:0] _zz_VexRiscv_199_;
  wire [1:0] _zz_VexRiscv_200_;
  wire [31:0] _zz_VexRiscv_201_;
  wire [32:0] _zz_VexRiscv_202_;
  wire [19:0] _zz_VexRiscv_203_;
  wire [11:0] _zz_VexRiscv_204_;
  wire [11:0] _zz_VexRiscv_205_;
  wire [0:0] _zz_VexRiscv_206_;
  wire [0:0] _zz_VexRiscv_207_;
  wire [0:0] _zz_VexRiscv_208_;
  wire [0:0] _zz_VexRiscv_209_;
  wire [0:0] _zz_VexRiscv_210_;
  wire [0:0] _zz_VexRiscv_211_;
  wire  _zz_VexRiscv_212_;
  wire  _zz_VexRiscv_213_;
  wire [31:0] _zz_VexRiscv_214_;
  wire  _zz_VexRiscv_215_;
  wire  _zz_VexRiscv_216_;
  wire [0:0] _zz_VexRiscv_217_;
  wire [0:0] _zz_VexRiscv_218_;
  wire [0:0] _zz_VexRiscv_219_;
  wire [0:0] _zz_VexRiscv_220_;
  wire  _zz_VexRiscv_221_;
  wire [0:0] _zz_VexRiscv_222_;
  wire [17:0] _zz_VexRiscv_223_;
  wire [31:0] _zz_VexRiscv_224_;
  wire [31:0] _zz_VexRiscv_225_;
  wire [31:0] _zz_VexRiscv_226_;
  wire [31:0] _zz_VexRiscv_227_;
  wire [0:0] _zz_VexRiscv_228_;
  wire [1:0] _zz_VexRiscv_229_;
  wire [0:0] _zz_VexRiscv_230_;
  wire [0:0] _zz_VexRiscv_231_;
  wire  _zz_VexRiscv_232_;
  wire [0:0] _zz_VexRiscv_233_;
  wire [14:0] _zz_VexRiscv_234_;
  wire [31:0] _zz_VexRiscv_235_;
  wire [31:0] _zz_VexRiscv_236_;
  wire [31:0] _zz_VexRiscv_237_;
  wire [31:0] _zz_VexRiscv_238_;
  wire [31:0] _zz_VexRiscv_239_;
  wire [0:0] _zz_VexRiscv_240_;
  wire [2:0] _zz_VexRiscv_241_;
  wire  _zz_VexRiscv_242_;
  wire [1:0] _zz_VexRiscv_243_;
  wire [1:0] _zz_VexRiscv_244_;
  wire  _zz_VexRiscv_245_;
  wire [0:0] _zz_VexRiscv_246_;
  wire [11:0] _zz_VexRiscv_247_;
  wire [31:0] _zz_VexRiscv_248_;
  wire [31:0] _zz_VexRiscv_249_;
  wire  _zz_VexRiscv_250_;
  wire [31:0] _zz_VexRiscv_251_;
  wire [31:0] _zz_VexRiscv_252_;
  wire [31:0] _zz_VexRiscv_253_;
  wire [31:0] _zz_VexRiscv_254_;
  wire  _zz_VexRiscv_255_;
  wire [1:0] _zz_VexRiscv_256_;
  wire [1:0] _zz_VexRiscv_257_;
  wire  _zz_VexRiscv_258_;
  wire [0:0] _zz_VexRiscv_259_;
  wire [8:0] _zz_VexRiscv_260_;
  wire [31:0] _zz_VexRiscv_261_;
  wire [31:0] _zz_VexRiscv_262_;
  wire [0:0] _zz_VexRiscv_263_;
  wire [0:0] _zz_VexRiscv_264_;
  wire [0:0] _zz_VexRiscv_265_;
  wire [0:0] _zz_VexRiscv_266_;
  wire [1:0] _zz_VexRiscv_267_;
  wire [1:0] _zz_VexRiscv_268_;
  wire  _zz_VexRiscv_269_;
  wire [0:0] _zz_VexRiscv_270_;
  wire [5:0] _zz_VexRiscv_271_;
  wire [31:0] _zz_VexRiscv_272_;
  wire [31:0] _zz_VexRiscv_273_;
  wire [31:0] _zz_VexRiscv_274_;
  wire  _zz_VexRiscv_275_;
  wire  _zz_VexRiscv_276_;
  wire [0:0] _zz_VexRiscv_277_;
  wire [0:0] _zz_VexRiscv_278_;
  wire [1:0] _zz_VexRiscv_279_;
  wire [1:0] _zz_VexRiscv_280_;
  wire  _zz_VexRiscv_281_;
  wire [0:0] _zz_VexRiscv_282_;
  wire [2:0] _zz_VexRiscv_283_;
  wire [31:0] _zz_VexRiscv_284_;
  wire [31:0] _zz_VexRiscv_285_;
  wire  _zz_VexRiscv_286_;
  wire [0:0] _zz_VexRiscv_287_;
  wire [1:0] _zz_VexRiscv_288_;
  wire  _zz_VexRiscv_289_;
  wire [2:0] _zz_VexRiscv_290_;
  wire [2:0] _zz_VexRiscv_291_;
  wire  _zz_VexRiscv_292_;
  wire  _zz_VexRiscv_293_;
  wire [31:0] _zz_VexRiscv_294_;
  wire [31:0] _zz_VexRiscv_295_;
  wire [31:0] _zz_VexRiscv_296_;
  wire [31:0] _zz_VexRiscv_297_;
  wire [31:0] _zz_VexRiscv_298_;
  wire  _zz_VexRiscv_299_;
  wire  _zz_VexRiscv_300_;
  wire  _zz_VexRiscv_301_;
  wire  _zz_VexRiscv_302_;
  wire [31:0] _zz_VexRiscv_303_;
  wire [31:0] _zz_VexRiscv_304_;
  wire  decode_CSR_READ_OPCODE;
  wire  decode_CSR_WRITE_OPCODE;
  wire  decode_IS_CSR;
  wire [31:0] writeBack_FORMAL_PC_NEXT;
  wire [31:0] memory_FORMAL_PC_NEXT;
  wire [31:0] execute_FORMAL_PC_NEXT;
  wire [31:0] decode_FORMAL_PC_NEXT;
  wire  decode_BYPASSABLE_EXECUTE_STAGE;
  wire [1:0] memory_MEMORY_ADDRESS_LOW;
  wire [1:0] execute_MEMORY_ADDRESS_LOW;
  wire [31:0] memory_PC;
  wire [31:0] writeBack_REGFILE_WRITE_DATA;
  wire [31:0] execute_REGFILE_WRITE_DATA;
  wire [31:0] decode_RS2;
  wire  decode_DO_EBREAK;
  wire [31:0] decode_SRC1;
  wire  execute_BYPASSABLE_MEMORY_STAGE;
  wire  decode_BYPASSABLE_MEMORY_STAGE;
  wire [31:0] decode_RS1;
  wire [31:0] execute_BRANCH_CALC;
  wire [31:0] memory_MEMORY_READ_DATA;
  wire  execute_BRANCH_DO;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_VexRiscv_1_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_VexRiscv_2_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_VexRiscv_3_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_VexRiscv_4_;
  wire `EnvCtrlEnum_defaultEncoding_type decode_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_VexRiscv_5_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_VexRiscv_6_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_VexRiscv_7_;
  wire  decode_SRC_LESS_UNSIGNED;
  wire `BranchCtrlEnum_defaultEncoding_type decode_BRANCH_CTRL;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_VexRiscv_8_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_VexRiscv_9_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_VexRiscv_10_;
  wire [31:0] decode_SRC2;
  wire `ShiftCtrlEnum_defaultEncoding_type decode_SHIFT_CTRL;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_VexRiscv_11_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_VexRiscv_12_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_VexRiscv_13_;
  wire  decode_MEMORY_ENABLE;
  wire  decode_SRC_USE_SUB_LESS;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type decode_ALU_BITWISE_CTRL;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_VexRiscv_14_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_VexRiscv_15_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_VexRiscv_16_;
  wire `AluCtrlEnum_defaultEncoding_type decode_ALU_CTRL;
  wire `AluCtrlEnum_defaultEncoding_type _zz_VexRiscv_17_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_VexRiscv_18_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_VexRiscv_19_;
  wire  execute_DO_EBREAK;
  wire  decode_IS_EBREAK;
  wire  _zz_VexRiscv_20_;
  wire [31:0] memory_BRANCH_CALC;
  wire  memory_BRANCH_DO;
  wire [31:0] _zz_VexRiscv_21_;
  wire [31:0] execute_PC;
  wire [31:0] execute_RS1;
  wire `BranchCtrlEnum_defaultEncoding_type execute_BRANCH_CTRL;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_VexRiscv_22_;
  wire  _zz_VexRiscv_23_;
  wire  decode_RS2_USE;
  wire  decode_RS1_USE;
  wire  execute_REGFILE_WRITE_VALID;
  wire  execute_BYPASSABLE_EXECUTE_STAGE;
  wire  memory_REGFILE_WRITE_VALID;
  wire  memory_BYPASSABLE_MEMORY_STAGE;
  wire  writeBack_REGFILE_WRITE_VALID;
  wire [31:0] memory_REGFILE_WRITE_DATA;
  wire `ShiftCtrlEnum_defaultEncoding_type execute_SHIFT_CTRL;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_VexRiscv_24_;
  wire  _zz_VexRiscv_25_;
  wire [31:0] _zz_VexRiscv_26_;
  wire [31:0] _zz_VexRiscv_27_;
  wire  execute_SRC_LESS_UNSIGNED;
  wire  execute_SRC_USE_SUB_LESS;
  wire [31:0] _zz_VexRiscv_28_;
  wire [31:0] _zz_VexRiscv_29_;
  wire `Src2CtrlEnum_defaultEncoding_type decode_SRC2_CTRL;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_VexRiscv_30_;
  wire [31:0] _zz_VexRiscv_31_;
  wire [31:0] _zz_VexRiscv_32_;
  wire `Src1CtrlEnum_defaultEncoding_type decode_SRC1_CTRL;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_VexRiscv_33_;
  wire [31:0] _zz_VexRiscv_34_;
  wire [31:0] execute_SRC_ADD_SUB;
  wire  execute_SRC_LESS;
  wire `AluCtrlEnum_defaultEncoding_type execute_ALU_CTRL;
  wire `AluCtrlEnum_defaultEncoding_type _zz_VexRiscv_35_;
  wire [31:0] _zz_VexRiscv_36_;
  wire [31:0] execute_SRC2;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type execute_ALU_BITWISE_CTRL;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_VexRiscv_37_;
  wire [31:0] _zz_VexRiscv_38_;
  wire  _zz_VexRiscv_39_;
  reg  _zz_VexRiscv_40_;
  wire [31:0] _zz_VexRiscv_41_;
  wire [31:0] _zz_VexRiscv_42_;
  wire [31:0] decode_INSTRUCTION_ANTICIPATED;
  reg  decode_REGFILE_WRITE_VALID;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_VexRiscv_43_;
  wire  _zz_VexRiscv_44_;
  wire  _zz_VexRiscv_45_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_VexRiscv_46_;
  wire  _zz_VexRiscv_47_;
  wire  _zz_VexRiscv_48_;
  wire  _zz_VexRiscv_49_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_VexRiscv_50_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_VexRiscv_51_;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_VexRiscv_52_;
  wire  _zz_VexRiscv_53_;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_VexRiscv_54_;
  wire  _zz_VexRiscv_55_;
  wire  _zz_VexRiscv_56_;
  wire  _zz_VexRiscv_57_;
  wire  _zz_VexRiscv_58_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_VexRiscv_59_;
  reg [31:0] _zz_VexRiscv_60_;
  wire [31:0] execute_SRC1;
  wire  execute_CSR_READ_OPCODE;
  wire  execute_CSR_WRITE_OPCODE;
  wire  execute_IS_CSR;
  wire `EnvCtrlEnum_defaultEncoding_type memory_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_VexRiscv_61_;
  wire `EnvCtrlEnum_defaultEncoding_type execute_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_VexRiscv_62_;
  wire  _zz_VexRiscv_63_;
  wire  _zz_VexRiscv_64_;
  wire `EnvCtrlEnum_defaultEncoding_type writeBack_ENV_CTRL;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_VexRiscv_65_;
  reg [31:0] _zz_VexRiscv_66_;
  wire  writeBack_MEMORY_ENABLE;
  wire [1:0] writeBack_MEMORY_ADDRESS_LOW;
  wire [31:0] writeBack_MEMORY_READ_DATA;
  wire [31:0] memory_INSTRUCTION;
  wire  memory_MEMORY_ENABLE;
  wire [31:0] _zz_VexRiscv_67_;
  wire [1:0] _zz_VexRiscv_68_;
  wire [31:0] execute_RS2;
  wire [31:0] execute_SRC_ADD;
  wire [31:0] execute_INSTRUCTION;
  wire  execute_ALIGNEMENT_FAULT;
  wire  execute_MEMORY_ENABLE;
  reg [31:0] _zz_VexRiscv_69_;
  wire [31:0] _zz_VexRiscv_70_;
  wire [31:0] _zz_VexRiscv_71_;
  wire [31:0] _zz_VexRiscv_72_;
  wire [31:0] _zz_VexRiscv_73_;
  wire [31:0] writeBack_PC /* verilator public */ ;
  wire [31:0] writeBack_INSTRUCTION /* verilator public */ ;
  wire [31:0] decode_PC /* verilator public */ ;
  wire [31:0] decode_INSTRUCTION /* verilator public */ ;
  reg  decode_arbitration_haltItself /* verilator public */ ;
  reg  decode_arbitration_haltByOther;
  reg  decode_arbitration_removeIt;
  wire  decode_arbitration_flushAll /* verilator public */ ;
  wire  decode_arbitration_redoIt;
  reg  decode_arbitration_isValid /* verilator public */ ;
  wire  decode_arbitration_isStuck;
  wire  decode_arbitration_isStuckByOthers;
  wire  decode_arbitration_isFlushed;
  wire  decode_arbitration_isMoving;
  wire  decode_arbitration_isFiring;
  reg  execute_arbitration_haltItself;
  reg  execute_arbitration_haltByOther;
  reg  execute_arbitration_removeIt;
  reg  execute_arbitration_flushAll;
  wire  execute_arbitration_redoIt;
  reg  execute_arbitration_isValid;
  wire  execute_arbitration_isStuck;
  wire  execute_arbitration_isStuckByOthers;
  wire  execute_arbitration_isFlushed;
  wire  execute_arbitration_isMoving;
  wire  execute_arbitration_isFiring;
  reg  memory_arbitration_haltItself;
  wire  memory_arbitration_haltByOther;
  reg  memory_arbitration_removeIt;
  reg  memory_arbitration_flushAll;
  wire  memory_arbitration_redoIt;
  reg  memory_arbitration_isValid;
  wire  memory_arbitration_isStuck;
  wire  memory_arbitration_isStuckByOthers;
  wire  memory_arbitration_isFlushed;
  wire  memory_arbitration_isMoving;
  wire  memory_arbitration_isFiring;
  wire  writeBack_arbitration_haltItself;
  wire  writeBack_arbitration_haltByOther;
  reg  writeBack_arbitration_removeIt;
  wire  writeBack_arbitration_flushAll;
  wire  writeBack_arbitration_redoIt;
  reg  writeBack_arbitration_isValid /* verilator public */ ;
  wire  writeBack_arbitration_isStuck;
  wire  writeBack_arbitration_isStuckByOthers;
  wire  writeBack_arbitration_isFlushed;
  wire  writeBack_arbitration_isMoving;
  wire  writeBack_arbitration_isFiring /* verilator public */ ;
  reg  _zz_VexRiscv_74_;
  reg  _zz_VexRiscv_75_;
  reg  _zz_VexRiscv_76_;
  reg  _zz_VexRiscv_77_;
  reg [31:0] _zz_VexRiscv_78_;
  wire  contextSwitching;
  reg [1:0] CsrPlugin_privilege;
  reg  _zz_VexRiscv_79_;
  wire  _zz_VexRiscv_80_;
  wire [31:0] _zz_VexRiscv_81_;
  reg  _zz_VexRiscv_82_;
  reg  _zz_VexRiscv_83_;
  wire  IBusSimplePlugin_jump_pcLoad_valid;
  wire [31:0] IBusSimplePlugin_jump_pcLoad_payload;
  wire [1:0] _zz_VexRiscv_84_;
  wire  IBusSimplePlugin_fetchPc_preOutput_valid;
  wire  IBusSimplePlugin_fetchPc_preOutput_ready;
  wire [31:0] IBusSimplePlugin_fetchPc_preOutput_payload;
  wire  _zz_VexRiscv_85_;
  wire  IBusSimplePlugin_fetchPc_output_valid;
  wire  IBusSimplePlugin_fetchPc_output_ready;
  wire [31:0] IBusSimplePlugin_fetchPc_output_payload;
  reg [31:0] IBusSimplePlugin_fetchPc_pcReg /* verilator public */ ;
  reg  IBusSimplePlugin_fetchPc_inc;
  reg  IBusSimplePlugin_fetchPc_propagatePc;
  reg [31:0] IBusSimplePlugin_fetchPc_pc;
  reg  IBusSimplePlugin_fetchPc_samplePcNext;
  reg  _zz_VexRiscv_86_;
  wire  IBusSimplePlugin_iBusRsp_stages_0_input_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_0_input_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_0_input_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_0_output_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_0_output_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_0_output_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_0_halt;
  wire  IBusSimplePlugin_iBusRsp_stages_0_inputSample;
  wire  IBusSimplePlugin_iBusRsp_stages_1_input_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_1_input_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_1_input_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_1_output_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_1_output_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_1_output_payload;
  reg  IBusSimplePlugin_iBusRsp_stages_1_halt;
  wire  IBusSimplePlugin_iBusRsp_stages_1_inputSample;
  wire  IBusSimplePlugin_iBusRsp_stages_2_input_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_2_input_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_2_input_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_2_output_valid;
  wire  IBusSimplePlugin_iBusRsp_stages_2_output_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_stages_2_output_payload;
  wire  IBusSimplePlugin_iBusRsp_stages_2_halt;
  wire  IBusSimplePlugin_iBusRsp_stages_2_inputSample;
  wire  _zz_VexRiscv_87_;
  wire  _zz_VexRiscv_88_;
  wire  _zz_VexRiscv_89_;
  wire  _zz_VexRiscv_90_;
  wire  _zz_VexRiscv_91_;
  reg  _zz_VexRiscv_92_;
  wire  _zz_VexRiscv_93_;
  reg  _zz_VexRiscv_94_;
  reg [31:0] _zz_VexRiscv_95_;
  reg  IBusSimplePlugin_iBusRsp_readyForError;
  wire  IBusSimplePlugin_iBusRsp_inputBeforeStage_valid;
  wire  IBusSimplePlugin_iBusRsp_inputBeforeStage_ready;
  wire [31:0] IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_pc;
  wire  IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_error;
  wire [31:0] IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_inst;
  wire  IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_isRvc;
  wire  IBusSimplePlugin_injector_decodeInput_valid;
  wire  IBusSimplePlugin_injector_decodeInput_ready;
  wire [31:0] IBusSimplePlugin_injector_decodeInput_payload_pc;
  wire  IBusSimplePlugin_injector_decodeInput_payload_rsp_error;
  wire [31:0] IBusSimplePlugin_injector_decodeInput_payload_rsp_inst;
  wire  IBusSimplePlugin_injector_decodeInput_payload_isRvc;
  reg  _zz_VexRiscv_96_;
  reg [31:0] _zz_VexRiscv_97_;
  reg  _zz_VexRiscv_98_;
  reg [31:0] _zz_VexRiscv_99_;
  reg  _zz_VexRiscv_100_;
  reg  IBusSimplePlugin_injector_nextPcCalc_valids_0;
  reg  IBusSimplePlugin_injector_nextPcCalc_valids_1;
  reg  IBusSimplePlugin_injector_nextPcCalc_valids_2;
  reg  IBusSimplePlugin_injector_nextPcCalc_valids_3;
  reg  IBusSimplePlugin_injector_nextPcCalc_valids_4;
  reg  IBusSimplePlugin_injector_nextPcCalc_valids_5;
  reg  IBusSimplePlugin_injector_decodeRemoved;
  reg [31:0] IBusSimplePlugin_injector_formal_rawInDecode;
  wire  IBusSimplePlugin_cmd_valid;
  wire  IBusSimplePlugin_cmd_ready;
  wire [31:0] IBusSimplePlugin_cmd_payload_pc;
  reg [2:0] IBusSimplePlugin_pendingCmd;
  wire [2:0] IBusSimplePlugin_pendingCmdNext;
  reg [2:0] IBusSimplePlugin_rspJoin_discardCounter;
  wire  IBusSimplePlugin_rspJoin_rspBufferOutput_valid;
  wire  IBusSimplePlugin_rspJoin_rspBufferOutput_ready;
  wire  IBusSimplePlugin_rspJoin_rspBufferOutput_payload_error;
  wire [31:0] IBusSimplePlugin_rspJoin_rspBufferOutput_payload_inst;
  wire  iBus_rsp_takeWhen_valid;
  wire  iBus_rsp_takeWhen_payload_error;
  wire [31:0] iBus_rsp_takeWhen_payload_inst;
  wire [31:0] IBusSimplePlugin_rspJoin_fetchRsp_pc;
  reg  IBusSimplePlugin_rspJoin_fetchRsp_rsp_error;
  wire [31:0] IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst;
  wire  IBusSimplePlugin_rspJoin_fetchRsp_isRvc;
  wire  IBusSimplePlugin_rspJoin_issueDetected;
  wire  IBusSimplePlugin_rspJoin_join_valid;
  wire  IBusSimplePlugin_rspJoin_join_ready;
  wire [31:0] IBusSimplePlugin_rspJoin_join_payload_pc;
  wire  IBusSimplePlugin_rspJoin_join_payload_rsp_error;
  wire [31:0] IBusSimplePlugin_rspJoin_join_payload_rsp_inst;
  wire  IBusSimplePlugin_rspJoin_join_payload_isRvc;
  wire  _zz_VexRiscv_101_;
  wire  execute_DBusSimplePlugin_cmdSent;
  reg [31:0] _zz_VexRiscv_102_;
  reg [3:0] _zz_VexRiscv_103_;
  wire [3:0] execute_DBusSimplePlugin_formalMask;
  reg [31:0] writeBack_DBusSimplePlugin_rspShifted;
  wire  _zz_VexRiscv_104_;
  reg [31:0] _zz_VexRiscv_105_;
  wire  _zz_VexRiscv_106_;
  reg [31:0] _zz_VexRiscv_107_;
  reg [31:0] writeBack_DBusSimplePlugin_rspFormated;
  wire [1:0] CsrPlugin_misa_base;
  wire [25:0] CsrPlugin_misa_extensions;
  wire [1:0] CsrPlugin_mtvec_mode;
  wire [29:0] CsrPlugin_mtvec_base;
  reg [31:0] CsrPlugin_mepc;
  reg  CsrPlugin_mstatus_MIE;
  reg  CsrPlugin_mstatus_MPIE;
  reg [1:0] CsrPlugin_mstatus_MPP;
  reg  CsrPlugin_mip_MEIP;
  reg  CsrPlugin_mip_MTIP;
  reg  CsrPlugin_mip_MSIP;
  reg  CsrPlugin_mie_MEIE;
  reg  CsrPlugin_mie_MTIE;
  reg  CsrPlugin_mie_MSIE;
  reg  CsrPlugin_mcause_interrupt;
  reg [3:0] CsrPlugin_mcause_exceptionCode;
  reg [31:0] CsrPlugin_mtval;
  reg [63:0] CsrPlugin_mcycle = 64'b0000000000000000000000000000000000000000000000000000000000000000;
  reg [63:0] CsrPlugin_minstret = 64'b0000000000000000000000000000000000000000000000000000000000000000;
  wire [31:0] CsrPlugin_medeleg;
  wire [31:0] CsrPlugin_mideleg;
  wire  _zz_VexRiscv_108_;
  wire  _zz_VexRiscv_109_;
  wire  _zz_VexRiscv_110_;
  reg  CsrPlugin_interrupt;
  reg [3:0] CsrPlugin_interruptCode /* verilator public */ ;
  wire [1:0] CsrPlugin_interruptTargetPrivilege;
  wire  CsrPlugin_exception;
  wire  CsrPlugin_lastStageWasWfi;
  reg  CsrPlugin_pipelineLiberator_done;
  wire  CsrPlugin_interruptJump /* verilator public */ ;
  reg  CsrPlugin_hadException;
  wire [1:0] CsrPlugin_targetPrivilege;
  wire [3:0] CsrPlugin_trapCause;
  wire  execute_CsrPlugin_blockedBySideEffects;
  reg  execute_CsrPlugin_illegalAccess;
  reg  execute_CsrPlugin_illegalInstruction;
  reg [31:0] execute_CsrPlugin_readData;
  wire  execute_CsrPlugin_writeInstruction;
  wire  execute_CsrPlugin_readInstruction;
  wire  execute_CsrPlugin_writeEnable;
  wire  execute_CsrPlugin_readEnable;
  reg [31:0] execute_CsrPlugin_writeData;
  wire [11:0] execute_CsrPlugin_csrAddress;
  wire [23:0] _zz_VexRiscv_111_;
  wire  _zz_VexRiscv_112_;
  wire  _zz_VexRiscv_113_;
  wire  _zz_VexRiscv_114_;
  wire  _zz_VexRiscv_115_;
  wire  _zz_VexRiscv_116_;
  wire  _zz_VexRiscv_117_;
  wire `EnvCtrlEnum_defaultEncoding_type _zz_VexRiscv_118_;
  wire `Src1CtrlEnum_defaultEncoding_type _zz_VexRiscv_119_;
  wire `Src2CtrlEnum_defaultEncoding_type _zz_VexRiscv_120_;
  wire `AluBitwiseCtrlEnum_defaultEncoding_type _zz_VexRiscv_121_;
  wire `BranchCtrlEnum_defaultEncoding_type _zz_VexRiscv_122_;
  wire `AluCtrlEnum_defaultEncoding_type _zz_VexRiscv_123_;
  wire `ShiftCtrlEnum_defaultEncoding_type _zz_VexRiscv_124_;
  wire [4:0] decode_RegFilePlugin_regFileReadAddress1;
  wire [4:0] decode_RegFilePlugin_regFileReadAddress2;
  wire [31:0] decode_RegFilePlugin_rs1Data;
  wire [31:0] decode_RegFilePlugin_rs2Data;
  reg  writeBack_RegFilePlugin_regFileWrite_valid /* verilator public */ ;
  wire [4:0] writeBack_RegFilePlugin_regFileWrite_payload_address /* verilator public */ ;
  wire [31:0] writeBack_RegFilePlugin_regFileWrite_payload_data /* verilator public */ ;
  reg  _zz_VexRiscv_125_;
  reg [31:0] execute_IntAluPlugin_bitwise;
  reg [31:0] _zz_VexRiscv_126_;
  reg [31:0] _zz_VexRiscv_127_;
  wire  _zz_VexRiscv_128_;
  reg [19:0] _zz_VexRiscv_129_;
  wire  _zz_VexRiscv_130_;
  reg [19:0] _zz_VexRiscv_131_;
  reg [31:0] _zz_VexRiscv_132_;
  wire [31:0] execute_SrcPlugin_addSub;
  wire  execute_SrcPlugin_less;
  reg  execute_LightShifterPlugin_isActive;
  wire  execute_LightShifterPlugin_isShift;
  reg [4:0] execute_LightShifterPlugin_amplitudeReg;
  wire [4:0] execute_LightShifterPlugin_amplitude;
  wire [31:0] execute_LightShifterPlugin_shiftInput;
  wire  execute_LightShifterPlugin_done;
  reg [31:0] _zz_VexRiscv_133_;
  reg  _zz_VexRiscv_134_;
  reg  _zz_VexRiscv_135_;
  wire  _zz_VexRiscv_136_;
  reg  _zz_VexRiscv_137_;
  reg [4:0] _zz_VexRiscv_138_;
  wire  execute_BranchPlugin_eq;
  wire [2:0] _zz_VexRiscv_139_;
  reg  _zz_VexRiscv_140_;
  reg  _zz_VexRiscv_141_;
  wire [31:0] execute_BranchPlugin_branch_src1;
  wire  _zz_VexRiscv_142_;
  reg [10:0] _zz_VexRiscv_143_;
  wire  _zz_VexRiscv_144_;
  reg [19:0] _zz_VexRiscv_145_;
  wire  _zz_VexRiscv_146_;
  reg [18:0] _zz_VexRiscv_147_;
  reg [31:0] _zz_VexRiscv_148_;
  wire [31:0] execute_BranchPlugin_branch_src2;
  wire [31:0] execute_BranchPlugin_branchAdder;
  reg  DebugPlugin_firstCycle;
  reg  DebugPlugin_secondCycle;
  reg  DebugPlugin_resetIt;
  reg  DebugPlugin_haltIt;
  reg  DebugPlugin_stepIt;
  reg  DebugPlugin_isPipActive;
  reg  DebugPlugin_isPipActive_regNext;
  wire  DebugPlugin_isPipBusy;
  reg  DebugPlugin_haltedByBreak;
  reg [31:0] DebugPlugin_busReadDataReg;
  reg  _zz_VexRiscv_149_;
  reg  DebugPlugin_resetIt_regNext;
  reg `AluCtrlEnum_defaultEncoding_type decode_to_execute_ALU_CTRL;
  reg [31:0] decode_to_execute_INSTRUCTION;
  reg [31:0] execute_to_memory_INSTRUCTION;
  reg [31:0] memory_to_writeBack_INSTRUCTION;
  reg `AluBitwiseCtrlEnum_defaultEncoding_type decode_to_execute_ALU_BITWISE_CTRL;
  reg  decode_to_execute_SRC_USE_SUB_LESS;
  reg  decode_to_execute_MEMORY_ENABLE;
  reg  execute_to_memory_MEMORY_ENABLE;
  reg  memory_to_writeBack_MEMORY_ENABLE;
  reg `ShiftCtrlEnum_defaultEncoding_type decode_to_execute_SHIFT_CTRL;
  reg [31:0] decode_to_execute_SRC2;
  reg `BranchCtrlEnum_defaultEncoding_type decode_to_execute_BRANCH_CTRL;
  reg  decode_to_execute_SRC_LESS_UNSIGNED;
  reg `EnvCtrlEnum_defaultEncoding_type decode_to_execute_ENV_CTRL;
  reg `EnvCtrlEnum_defaultEncoding_type execute_to_memory_ENV_CTRL;
  reg `EnvCtrlEnum_defaultEncoding_type memory_to_writeBack_ENV_CTRL;
  reg  execute_to_memory_BRANCH_DO;
  reg [31:0] memory_to_writeBack_MEMORY_READ_DATA;
  reg [31:0] execute_to_memory_BRANCH_CALC;
  reg [31:0] decode_to_execute_RS1;
  reg  decode_to_execute_BYPASSABLE_MEMORY_STAGE;
  reg  execute_to_memory_BYPASSABLE_MEMORY_STAGE;
  reg [31:0] decode_to_execute_SRC1;
  reg  decode_to_execute_DO_EBREAK;
  reg [31:0] decode_to_execute_RS2;
  reg [31:0] execute_to_memory_REGFILE_WRITE_DATA;
  reg [31:0] memory_to_writeBack_REGFILE_WRITE_DATA;
  reg [31:0] decode_to_execute_PC;
  reg [31:0] execute_to_memory_PC;
  reg [31:0] memory_to_writeBack_PC;
  reg  decode_to_execute_REGFILE_WRITE_VALID;
  reg  execute_to_memory_REGFILE_WRITE_VALID;
  reg  memory_to_writeBack_REGFILE_WRITE_VALID;
  reg [1:0] execute_to_memory_MEMORY_ADDRESS_LOW;
  reg [1:0] memory_to_writeBack_MEMORY_ADDRESS_LOW;
  reg  decode_to_execute_BYPASSABLE_EXECUTE_STAGE;
  reg [31:0] decode_to_execute_FORMAL_PC_NEXT;
  reg [31:0] execute_to_memory_FORMAL_PC_NEXT;
  reg [31:0] memory_to_writeBack_FORMAL_PC_NEXT;
  reg  decode_to_execute_IS_CSR;
  reg  decode_to_execute_CSR_WRITE_OPCODE;
  reg  decode_to_execute_CSR_READ_OPCODE;
  reg [2:0] _zz_VexRiscv_150_;
  `ifndef SYNTHESIS
  reg [31:0] _zz_VexRiscv_1__string;
  reg [31:0] _zz_VexRiscv_2__string;
  reg [31:0] _zz_VexRiscv_3__string;
  reg [31:0] _zz_VexRiscv_4__string;
  reg [31:0] decode_ENV_CTRL_string;
  reg [31:0] _zz_VexRiscv_5__string;
  reg [31:0] _zz_VexRiscv_6__string;
  reg [31:0] _zz_VexRiscv_7__string;
  reg [31:0] decode_BRANCH_CTRL_string;
  reg [31:0] _zz_VexRiscv_8__string;
  reg [31:0] _zz_VexRiscv_9__string;
  reg [31:0] _zz_VexRiscv_10__string;
  reg [71:0] decode_SHIFT_CTRL_string;
  reg [71:0] _zz_VexRiscv_11__string;
  reg [71:0] _zz_VexRiscv_12__string;
  reg [71:0] _zz_VexRiscv_13__string;
  reg [39:0] decode_ALU_BITWISE_CTRL_string;
  reg [39:0] _zz_VexRiscv_14__string;
  reg [39:0] _zz_VexRiscv_15__string;
  reg [39:0] _zz_VexRiscv_16__string;
  reg [63:0] decode_ALU_CTRL_string;
  reg [63:0] _zz_VexRiscv_17__string;
  reg [63:0] _zz_VexRiscv_18__string;
  reg [63:0] _zz_VexRiscv_19__string;
  reg [31:0] execute_BRANCH_CTRL_string;
  reg [31:0] _zz_VexRiscv_22__string;
  reg [71:0] execute_SHIFT_CTRL_string;
  reg [71:0] _zz_VexRiscv_24__string;
  reg [23:0] decode_SRC2_CTRL_string;
  reg [23:0] _zz_VexRiscv_30__string;
  reg [95:0] decode_SRC1_CTRL_string;
  reg [95:0] _zz_VexRiscv_33__string;
  reg [63:0] execute_ALU_CTRL_string;
  reg [63:0] _zz_VexRiscv_35__string;
  reg [39:0] execute_ALU_BITWISE_CTRL_string;
  reg [39:0] _zz_VexRiscv_37__string;
  reg [71:0] _zz_VexRiscv_43__string;
  reg [63:0] _zz_VexRiscv_46__string;
  reg [31:0] _zz_VexRiscv_50__string;
  reg [39:0] _zz_VexRiscv_51__string;
  reg [23:0] _zz_VexRiscv_52__string;
  reg [95:0] _zz_VexRiscv_54__string;
  reg [31:0] _zz_VexRiscv_59__string;
  reg [31:0] memory_ENV_CTRL_string;
  reg [31:0] _zz_VexRiscv_61__string;
  reg [31:0] execute_ENV_CTRL_string;
  reg [31:0] _zz_VexRiscv_62__string;
  reg [31:0] writeBack_ENV_CTRL_string;
  reg [31:0] _zz_VexRiscv_65__string;
  reg [31:0] _zz_VexRiscv_118__string;
  reg [95:0] _zz_VexRiscv_119__string;
  reg [23:0] _zz_VexRiscv_120__string;
  reg [39:0] _zz_VexRiscv_121__string;
  reg [31:0] _zz_VexRiscv_122__string;
  reg [63:0] _zz_VexRiscv_123__string;
  reg [71:0] _zz_VexRiscv_124__string;
  reg [63:0] decode_to_execute_ALU_CTRL_string;
  reg [39:0] decode_to_execute_ALU_BITWISE_CTRL_string;
  reg [71:0] decode_to_execute_SHIFT_CTRL_string;
  reg [31:0] decode_to_execute_BRANCH_CTRL_string;
  reg [31:0] decode_to_execute_ENV_CTRL_string;
  reg [31:0] execute_to_memory_ENV_CTRL_string;
  reg [31:0] memory_to_writeBack_ENV_CTRL_string;
  `endif

  reg [31:0] RegFilePlugin_regFile [0:31] /* verilator public */ ;
  assign _zz_VexRiscv_155_ = ((execute_arbitration_isValid && execute_LightShifterPlugin_isShift) && (execute_SRC2[4 : 0] != (5'b00000)));
  assign _zz_VexRiscv_156_ = (! execute_arbitration_isStuckByOthers);
  assign _zz_VexRiscv_157_ = (execute_arbitration_isValid && execute_DO_EBREAK);
  assign _zz_VexRiscv_158_ = (({writeBack_arbitration_isValid,memory_arbitration_isValid} != (2'b00)) == 1'b0);
  assign _zz_VexRiscv_159_ = (DebugPlugin_stepIt && _zz_VexRiscv_76_);
  assign _zz_VexRiscv_160_ = (CsrPlugin_hadException || CsrPlugin_interruptJump);
  assign _zz_VexRiscv_161_ = (writeBack_arbitration_isValid && (writeBack_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET));
  assign _zz_VexRiscv_162_ = (IBusSimplePlugin_fetchPc_preOutput_valid && IBusSimplePlugin_fetchPc_preOutput_ready);
  assign _zz_VexRiscv_163_ = debug_bus_cmd_payload_address[7 : 2];
  assign _zz_VexRiscv_164_ = writeBack_INSTRUCTION[13 : 12];
  assign _zz_VexRiscv_165_ = writeBack_INSTRUCTION[29 : 28];
  assign _zz_VexRiscv_166_ = execute_INSTRUCTION[13];
  assign _zz_VexRiscv_167_ = (_zz_VexRiscv_84_ & (~ _zz_VexRiscv_168_));
  assign _zz_VexRiscv_168_ = (_zz_VexRiscv_84_ - (2'b01));
  assign _zz_VexRiscv_169_ = {IBusSimplePlugin_fetchPc_inc,(2'b00)};
  assign _zz_VexRiscv_170_ = {29'd0, _zz_VexRiscv_169_};
  assign _zz_VexRiscv_171_ = (IBusSimplePlugin_pendingCmd + _zz_VexRiscv_173_);
  assign _zz_VexRiscv_172_ = (IBusSimplePlugin_cmd_valid && IBusSimplePlugin_cmd_ready);
  assign _zz_VexRiscv_173_ = {2'd0, _zz_VexRiscv_172_};
  assign _zz_VexRiscv_174_ = iBus_rsp_valid;
  assign _zz_VexRiscv_175_ = {2'd0, _zz_VexRiscv_174_};
  assign _zz_VexRiscv_176_ = (iBus_rsp_valid && (IBusSimplePlugin_rspJoin_discardCounter != (3'b000)));
  assign _zz_VexRiscv_177_ = {2'd0, _zz_VexRiscv_176_};
  assign _zz_VexRiscv_178_ = _zz_VexRiscv_111_[1 : 1];
  assign _zz_VexRiscv_179_ = _zz_VexRiscv_111_[2 : 2];
  assign _zz_VexRiscv_180_ = _zz_VexRiscv_111_[3 : 3];
  assign _zz_VexRiscv_181_ = _zz_VexRiscv_111_[4 : 4];
  assign _zz_VexRiscv_182_ = _zz_VexRiscv_111_[7 : 7];
  assign _zz_VexRiscv_183_ = _zz_VexRiscv_111_[14 : 14];
  assign _zz_VexRiscv_184_ = _zz_VexRiscv_111_[15 : 15];
  assign _zz_VexRiscv_185_ = _zz_VexRiscv_111_[16 : 16];
  assign _zz_VexRiscv_186_ = _zz_VexRiscv_111_[19 : 19];
  assign _zz_VexRiscv_187_ = _zz_VexRiscv_111_[20 : 20];
  assign _zz_VexRiscv_188_ = execute_SRC_LESS;
  assign _zz_VexRiscv_189_ = (3'b100);
  assign _zz_VexRiscv_190_ = decode_INSTRUCTION[19 : 15];
  assign _zz_VexRiscv_191_ = decode_INSTRUCTION[31 : 20];
  assign _zz_VexRiscv_192_ = {decode_INSTRUCTION[31 : 25],decode_INSTRUCTION[11 : 7]};
  assign _zz_VexRiscv_193_ = ($signed(_zz_VexRiscv_194_) + $signed(_zz_VexRiscv_198_));
  assign _zz_VexRiscv_194_ = ($signed(_zz_VexRiscv_195_) + $signed(_zz_VexRiscv_196_));
  assign _zz_VexRiscv_195_ = execute_SRC1;
  assign _zz_VexRiscv_196_ = (execute_SRC_USE_SUB_LESS ? (~ execute_SRC2) : execute_SRC2);
  assign _zz_VexRiscv_197_ = (execute_SRC_USE_SUB_LESS ? _zz_VexRiscv_199_ : _zz_VexRiscv_200_);
  assign _zz_VexRiscv_198_ = {{30{_zz_VexRiscv_197_[1]}}, _zz_VexRiscv_197_};
  assign _zz_VexRiscv_199_ = (2'b01);
  assign _zz_VexRiscv_200_ = (2'b00);
  assign _zz_VexRiscv_201_ = (_zz_VexRiscv_202_ >>> 1);
  assign _zz_VexRiscv_202_ = {((execute_SHIFT_CTRL == `ShiftCtrlEnum_defaultEncoding_SRA_1) && execute_LightShifterPlugin_shiftInput[31]),execute_LightShifterPlugin_shiftInput};
  assign _zz_VexRiscv_203_ = {{{execute_INSTRUCTION[31],execute_INSTRUCTION[19 : 12]},execute_INSTRUCTION[20]},execute_INSTRUCTION[30 : 21]};
  assign _zz_VexRiscv_204_ = execute_INSTRUCTION[31 : 20];
  assign _zz_VexRiscv_205_ = {{{execute_INSTRUCTION[31],execute_INSTRUCTION[7]},execute_INSTRUCTION[30 : 25]},execute_INSTRUCTION[11 : 8]};
  assign _zz_VexRiscv_206_ = execute_CsrPlugin_writeData[7 : 7];
  assign _zz_VexRiscv_207_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_VexRiscv_208_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_VexRiscv_209_ = execute_CsrPlugin_writeData[11 : 11];
  assign _zz_VexRiscv_210_ = execute_CsrPlugin_writeData[7 : 7];
  assign _zz_VexRiscv_211_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_VexRiscv_212_ = 1'b1;
  assign _zz_VexRiscv_213_ = 1'b1;
  assign _zz_VexRiscv_214_ = (32'b00000000000000000111000001010100);
  assign _zz_VexRiscv_215_ = ((decode_INSTRUCTION & (32'b01000000000000000011000001010100)) == (32'b01000000000000000001000000010000));
  assign _zz_VexRiscv_216_ = ((decode_INSTRUCTION & (32'b00000000000000000111000001010100)) == (32'b00000000000000000001000000010000));
  assign _zz_VexRiscv_217_ = ((decode_INSTRUCTION & _zz_VexRiscv_224_) == (32'b00000000000000000000000001000000));
  assign _zz_VexRiscv_218_ = ((decode_INSTRUCTION & _zz_VexRiscv_225_) == (32'b00000000000000000000000001000000));
  assign _zz_VexRiscv_219_ = _zz_VexRiscv_117_;
  assign _zz_VexRiscv_220_ = (1'b0);
  assign _zz_VexRiscv_221_ = ((_zz_VexRiscv_226_ == _zz_VexRiscv_227_) != (1'b0));
  assign _zz_VexRiscv_222_ = ({_zz_VexRiscv_228_,_zz_VexRiscv_229_} != (3'b000));
  assign _zz_VexRiscv_223_ = {(_zz_VexRiscv_230_ != _zz_VexRiscv_231_),{_zz_VexRiscv_232_,{_zz_VexRiscv_233_,_zz_VexRiscv_234_}}};
  assign _zz_VexRiscv_224_ = (32'b00000000000000000000000001010000);
  assign _zz_VexRiscv_225_ = (32'b00000000000100000011000001000000);
  assign _zz_VexRiscv_226_ = (decode_INSTRUCTION & (32'b00000000000000000000000001010000));
  assign _zz_VexRiscv_227_ = (32'b00000000000000000000000000000000);
  assign _zz_VexRiscv_228_ = ((decode_INSTRUCTION & _zz_VexRiscv_235_) == (32'b00000000000000000100000000000000));
  assign _zz_VexRiscv_229_ = {(_zz_VexRiscv_236_ == _zz_VexRiscv_237_),(_zz_VexRiscv_238_ == _zz_VexRiscv_239_)};
  assign _zz_VexRiscv_230_ = _zz_VexRiscv_112_;
  assign _zz_VexRiscv_231_ = (1'b0);
  assign _zz_VexRiscv_232_ = ({_zz_VexRiscv_116_,{_zz_VexRiscv_240_,_zz_VexRiscv_241_}} != (5'b00000));
  assign _zz_VexRiscv_233_ = (_zz_VexRiscv_242_ != (1'b0));
  assign _zz_VexRiscv_234_ = {(_zz_VexRiscv_243_ != _zz_VexRiscv_244_),{_zz_VexRiscv_245_,{_zz_VexRiscv_246_,_zz_VexRiscv_247_}}};
  assign _zz_VexRiscv_235_ = (32'b00000000000000000100000000000100);
  assign _zz_VexRiscv_236_ = (decode_INSTRUCTION & (32'b00000000000000000000000001100100));
  assign _zz_VexRiscv_237_ = (32'b00000000000000000000000000100100);
  assign _zz_VexRiscv_238_ = (decode_INSTRUCTION & (32'b00000000000000000011000000000100));
  assign _zz_VexRiscv_239_ = (32'b00000000000000000001000000000000);
  assign _zz_VexRiscv_240_ = _zz_VexRiscv_115_;
  assign _zz_VexRiscv_241_ = {(_zz_VexRiscv_248_ == _zz_VexRiscv_249_),{_zz_VexRiscv_250_,_zz_VexRiscv_117_}};
  assign _zz_VexRiscv_242_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000010000)) == (32'b00000000000000000000000000010000));
  assign _zz_VexRiscv_243_ = {(_zz_VexRiscv_251_ == _zz_VexRiscv_252_),(_zz_VexRiscv_253_ == _zz_VexRiscv_254_)};
  assign _zz_VexRiscv_244_ = (2'b00);
  assign _zz_VexRiscv_245_ = (_zz_VexRiscv_114_ != (1'b0));
  assign _zz_VexRiscv_246_ = (_zz_VexRiscv_255_ != (1'b0));
  assign _zz_VexRiscv_247_ = {(_zz_VexRiscv_256_ != _zz_VexRiscv_257_),{_zz_VexRiscv_258_,{_zz_VexRiscv_259_,_zz_VexRiscv_260_}}};
  assign _zz_VexRiscv_248_ = (decode_INSTRUCTION & (32'b00000000000000000001000000010000));
  assign _zz_VexRiscv_249_ = (32'b00000000000000000001000000010000);
  assign _zz_VexRiscv_250_ = ((decode_INSTRUCTION & (32'b00000000000000000010000000010000)) == (32'b00000000000000000010000000010000));
  assign _zz_VexRiscv_251_ = (decode_INSTRUCTION & (32'b00000000000000000001000001010000));
  assign _zz_VexRiscv_252_ = (32'b00000000000000000001000001010000);
  assign _zz_VexRiscv_253_ = (decode_INSTRUCTION & (32'b00000000000000000010000001010000));
  assign _zz_VexRiscv_254_ = (32'b00000000000000000010000001010000);
  assign _zz_VexRiscv_255_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001011000)) == (32'b00000000000000000000000001000000));
  assign _zz_VexRiscv_256_ = {(_zz_VexRiscv_261_ == _zz_VexRiscv_262_),_zz_VexRiscv_116_};
  assign _zz_VexRiscv_257_ = (2'b00);
  assign _zz_VexRiscv_258_ = ({_zz_VexRiscv_116_,{_zz_VexRiscv_263_,_zz_VexRiscv_264_}} != (3'b000));
  assign _zz_VexRiscv_259_ = ({_zz_VexRiscv_265_,_zz_VexRiscv_266_} != (2'b00));
  assign _zz_VexRiscv_260_ = {(_zz_VexRiscv_267_ != _zz_VexRiscv_268_),{_zz_VexRiscv_269_,{_zz_VexRiscv_270_,_zz_VexRiscv_271_}}};
  assign _zz_VexRiscv_261_ = (decode_INSTRUCTION & (32'b00000000000000000001000000000000));
  assign _zz_VexRiscv_262_ = (32'b00000000000000000001000000000000);
  assign _zz_VexRiscv_263_ = ((decode_INSTRUCTION & _zz_VexRiscv_272_) == (32'b00000000000000000001000000000000));
  assign _zz_VexRiscv_264_ = ((decode_INSTRUCTION & _zz_VexRiscv_273_) == (32'b00000000000000000010000000000000));
  assign _zz_VexRiscv_265_ = _zz_VexRiscv_116_;
  assign _zz_VexRiscv_266_ = ((decode_INSTRUCTION & _zz_VexRiscv_274_) == (32'b00000000000000000000000000100000));
  assign _zz_VexRiscv_267_ = {_zz_VexRiscv_116_,_zz_VexRiscv_115_};
  assign _zz_VexRiscv_268_ = (2'b00);
  assign _zz_VexRiscv_269_ = ({_zz_VexRiscv_275_,_zz_VexRiscv_276_} != (2'b00));
  assign _zz_VexRiscv_270_ = ({_zz_VexRiscv_277_,_zz_VexRiscv_278_} != (2'b00));
  assign _zz_VexRiscv_271_ = {(_zz_VexRiscv_279_ != _zz_VexRiscv_280_),{_zz_VexRiscv_281_,{_zz_VexRiscv_282_,_zz_VexRiscv_283_}}};
  assign _zz_VexRiscv_272_ = (32'b00000000000000000011000000000000);
  assign _zz_VexRiscv_273_ = (32'b00000000000000000011000000000000);
  assign _zz_VexRiscv_274_ = (32'b00000000000000000000000001110000);
  assign _zz_VexRiscv_275_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000110100)) == (32'b00000000000000000000000000100000));
  assign _zz_VexRiscv_276_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001100100)) == (32'b00000000000000000000000000100000));
  assign _zz_VexRiscv_277_ = _zz_VexRiscv_114_;
  assign _zz_VexRiscv_278_ = _zz_VexRiscv_113_;
  assign _zz_VexRiscv_279_ = {(_zz_VexRiscv_284_ == _zz_VexRiscv_285_),_zz_VexRiscv_113_};
  assign _zz_VexRiscv_280_ = (2'b00);
  assign _zz_VexRiscv_281_ = ({_zz_VexRiscv_286_,{_zz_VexRiscv_287_,_zz_VexRiscv_288_}} != (4'b0000));
  assign _zz_VexRiscv_282_ = (_zz_VexRiscv_289_ != (1'b0));
  assign _zz_VexRiscv_283_ = {(_zz_VexRiscv_290_ != _zz_VexRiscv_291_),{_zz_VexRiscv_292_,_zz_VexRiscv_293_}};
  assign _zz_VexRiscv_284_ = (decode_INSTRUCTION & (32'b00000000000000000000000001000100));
  assign _zz_VexRiscv_285_ = (32'b00000000000000000000000000000100);
  assign _zz_VexRiscv_286_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001000100)) == (32'b00000000000000000000000000000000));
  assign _zz_VexRiscv_287_ = ((decode_INSTRUCTION & _zz_VexRiscv_294_) == (32'b00000000000000000000000000000000));
  assign _zz_VexRiscv_288_ = {_zz_VexRiscv_112_,(_zz_VexRiscv_295_ == _zz_VexRiscv_296_)};
  assign _zz_VexRiscv_289_ = ((decode_INSTRUCTION & (32'b00010000000000000011000001010000)) == (32'b00000000000000000000000001010000));
  assign _zz_VexRiscv_290_ = {(_zz_VexRiscv_297_ == _zz_VexRiscv_298_),{_zz_VexRiscv_299_,_zz_VexRiscv_300_}};
  assign _zz_VexRiscv_291_ = (3'b000);
  assign _zz_VexRiscv_292_ = ({_zz_VexRiscv_301_,_zz_VexRiscv_302_} != (2'b00));
  assign _zz_VexRiscv_293_ = ((_zz_VexRiscv_303_ == _zz_VexRiscv_304_) != (1'b0));
  assign _zz_VexRiscv_294_ = (32'b00000000000000000000000000011000);
  assign _zz_VexRiscv_295_ = (decode_INSTRUCTION & (32'b00000000000000000101000000000100));
  assign _zz_VexRiscv_296_ = (32'b00000000000000000001000000000000);
  assign _zz_VexRiscv_297_ = (decode_INSTRUCTION & (32'b00000000000000000000000001000100));
  assign _zz_VexRiscv_298_ = (32'b00000000000000000000000001000000);
  assign _zz_VexRiscv_299_ = ((decode_INSTRUCTION & (32'b01000000000000000000000000110000)) == (32'b01000000000000000000000000110000));
  assign _zz_VexRiscv_300_ = ((decode_INSTRUCTION & (32'b00000000000000000010000000010100)) == (32'b00000000000000000010000000010000));
  assign _zz_VexRiscv_301_ = ((decode_INSTRUCTION & (32'b00000000000000000010000000010000)) == (32'b00000000000000000010000000000000));
  assign _zz_VexRiscv_302_ = ((decode_INSTRUCTION & (32'b00000000000000000101000000000000)) == (32'b00000000000000000001000000000000));
  assign _zz_VexRiscv_303_ = (decode_INSTRUCTION & (32'b00000000000100000011000001010000));
  assign _zz_VexRiscv_304_ = (32'b00000000000000000000000001010000);
  always @ (posedge toplevel_main_clk) begin
    if(_zz_VexRiscv_40_) begin
      RegFilePlugin_regFile[writeBack_RegFilePlugin_regFileWrite_payload_address] <= writeBack_RegFilePlugin_regFileWrite_payload_data;
    end
  end

  always @ (posedge toplevel_main_clk) begin
    if(_zz_VexRiscv_212_) begin
      _zz_VexRiscv_153_ <= RegFilePlugin_regFile[decode_RegFilePlugin_regFileReadAddress1];
    end
  end

  always @ (posedge toplevel_main_clk) begin
    if(_zz_VexRiscv_213_) begin
      _zz_VexRiscv_154_ <= RegFilePlugin_regFile[decode_RegFilePlugin_regFileReadAddress2];
    end
  end

  StreamFifoLowLatency IBusSimplePlugin_rspJoin_rspBuffer_c ( 
    .io_push_valid(iBus_rsp_takeWhen_valid),
    .io_push_ready(IBusSimplePlugin_rspJoin_rspBuffer_c_io_push_ready),
    .io_push_payload_error(iBus_rsp_takeWhen_payload_error),
    .io_push_payload_inst(iBus_rsp_takeWhen_payload_inst),
    .io_pop_valid(IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid),
    .io_pop_ready(IBusSimplePlugin_rspJoin_rspBufferOutput_ready),
    .io_pop_payload_error(IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error),
    .io_pop_payload_inst(IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst),
    .io_flush(_zz_VexRiscv_152_),
    .io_occupancy(IBusSimplePlugin_rspJoin_rspBuffer_c_io_occupancy),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(_zz_VexRiscv_1_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_VexRiscv_1__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_VexRiscv_1__string = "XRET";
      default : _zz_VexRiscv_1__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_2_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_VexRiscv_2__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_VexRiscv_2__string = "XRET";
      default : _zz_VexRiscv_2__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_3_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_VexRiscv_3__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_VexRiscv_3__string = "XRET";
      default : _zz_VexRiscv_3__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_4_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_VexRiscv_4__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_VexRiscv_4__string = "XRET";
      default : _zz_VexRiscv_4__string = "????";
    endcase
  end
  always @(*) begin
    case(decode_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : decode_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : decode_ENV_CTRL_string = "XRET";
      default : decode_ENV_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_5_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_VexRiscv_5__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_VexRiscv_5__string = "XRET";
      default : _zz_VexRiscv_5__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_6_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_VexRiscv_6__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_VexRiscv_6__string = "XRET";
      default : _zz_VexRiscv_6__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_7_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_VexRiscv_7__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_VexRiscv_7__string = "XRET";
      default : _zz_VexRiscv_7__string = "????";
    endcase
  end
  always @(*) begin
    case(decode_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : decode_BRANCH_CTRL_string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : decode_BRANCH_CTRL_string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : decode_BRANCH_CTRL_string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : decode_BRANCH_CTRL_string = "JALR";
      default : decode_BRANCH_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_8_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_VexRiscv_8__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_VexRiscv_8__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_VexRiscv_8__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_VexRiscv_8__string = "JALR";
      default : _zz_VexRiscv_8__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_9_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_VexRiscv_9__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_VexRiscv_9__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_VexRiscv_9__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_VexRiscv_9__string = "JALR";
      default : _zz_VexRiscv_9__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_10_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_VexRiscv_10__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_VexRiscv_10__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_VexRiscv_10__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_VexRiscv_10__string = "JALR";
      default : _zz_VexRiscv_10__string = "????";
    endcase
  end
  always @(*) begin
    case(decode_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : decode_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : decode_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : decode_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : decode_SHIFT_CTRL_string = "SRA_1    ";
      default : decode_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_11_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_VexRiscv_11__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_VexRiscv_11__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_VexRiscv_11__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_VexRiscv_11__string = "SRA_1    ";
      default : _zz_VexRiscv_11__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_12_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_VexRiscv_12__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_VexRiscv_12__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_VexRiscv_12__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_VexRiscv_12__string = "SRA_1    ";
      default : _zz_VexRiscv_12__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_13_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_VexRiscv_13__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_VexRiscv_13__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_VexRiscv_13__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_VexRiscv_13__string = "SRA_1    ";
      default : _zz_VexRiscv_13__string = "?????????";
    endcase
  end
  always @(*) begin
    case(decode_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : decode_ALU_BITWISE_CTRL_string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : decode_ALU_BITWISE_CTRL_string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : decode_ALU_BITWISE_CTRL_string = "AND_1";
      `AluBitwiseCtrlEnum_defaultEncoding_SRC1 : decode_ALU_BITWISE_CTRL_string = "SRC1 ";
      default : decode_ALU_BITWISE_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_14_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_VexRiscv_14__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_VexRiscv_14__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_VexRiscv_14__string = "AND_1";
      `AluBitwiseCtrlEnum_defaultEncoding_SRC1 : _zz_VexRiscv_14__string = "SRC1 ";
      default : _zz_VexRiscv_14__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_15_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_VexRiscv_15__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_VexRiscv_15__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_VexRiscv_15__string = "AND_1";
      `AluBitwiseCtrlEnum_defaultEncoding_SRC1 : _zz_VexRiscv_15__string = "SRC1 ";
      default : _zz_VexRiscv_15__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_16_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_VexRiscv_16__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_VexRiscv_16__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_VexRiscv_16__string = "AND_1";
      `AluBitwiseCtrlEnum_defaultEncoding_SRC1 : _zz_VexRiscv_16__string = "SRC1 ";
      default : _zz_VexRiscv_16__string = "?????";
    endcase
  end
  always @(*) begin
    case(decode_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : decode_ALU_CTRL_string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : decode_ALU_CTRL_string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : decode_ALU_CTRL_string = "BITWISE ";
      default : decode_ALU_CTRL_string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_17_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_VexRiscv_17__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_VexRiscv_17__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_VexRiscv_17__string = "BITWISE ";
      default : _zz_VexRiscv_17__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_18_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_VexRiscv_18__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_VexRiscv_18__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_VexRiscv_18__string = "BITWISE ";
      default : _zz_VexRiscv_18__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_19_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_VexRiscv_19__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_VexRiscv_19__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_VexRiscv_19__string = "BITWISE ";
      default : _zz_VexRiscv_19__string = "????????";
    endcase
  end
  always @(*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : execute_BRANCH_CTRL_string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : execute_BRANCH_CTRL_string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : execute_BRANCH_CTRL_string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : execute_BRANCH_CTRL_string = "JALR";
      default : execute_BRANCH_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_22_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_VexRiscv_22__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_VexRiscv_22__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_VexRiscv_22__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_VexRiscv_22__string = "JALR";
      default : _zz_VexRiscv_22__string = "????";
    endcase
  end
  always @(*) begin
    case(execute_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : execute_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : execute_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : execute_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : execute_SHIFT_CTRL_string = "SRA_1    ";
      default : execute_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_24_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_VexRiscv_24__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_VexRiscv_24__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_VexRiscv_24__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_VexRiscv_24__string = "SRA_1    ";
      default : _zz_VexRiscv_24__string = "?????????";
    endcase
  end
  always @(*) begin
    case(decode_SRC2_CTRL)
      `Src2CtrlEnum_defaultEncoding_RS : decode_SRC2_CTRL_string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : decode_SRC2_CTRL_string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : decode_SRC2_CTRL_string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : decode_SRC2_CTRL_string = "PC ";
      default : decode_SRC2_CTRL_string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_30_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_VexRiscv_30__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_VexRiscv_30__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_VexRiscv_30__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_VexRiscv_30__string = "PC ";
      default : _zz_VexRiscv_30__string = "???";
    endcase
  end
  always @(*) begin
    case(decode_SRC1_CTRL)
      `Src1CtrlEnum_defaultEncoding_RS : decode_SRC1_CTRL_string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : decode_SRC1_CTRL_string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : decode_SRC1_CTRL_string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : decode_SRC1_CTRL_string = "URS1        ";
      default : decode_SRC1_CTRL_string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_33_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_VexRiscv_33__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_VexRiscv_33__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_VexRiscv_33__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_VexRiscv_33__string = "URS1        ";
      default : _zz_VexRiscv_33__string = "????????????";
    endcase
  end
  always @(*) begin
    case(execute_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : execute_ALU_CTRL_string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : execute_ALU_CTRL_string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : execute_ALU_CTRL_string = "BITWISE ";
      default : execute_ALU_CTRL_string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_35_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_VexRiscv_35__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_VexRiscv_35__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_VexRiscv_35__string = "BITWISE ";
      default : _zz_VexRiscv_35__string = "????????";
    endcase
  end
  always @(*) begin
    case(execute_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : execute_ALU_BITWISE_CTRL_string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : execute_ALU_BITWISE_CTRL_string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : execute_ALU_BITWISE_CTRL_string = "AND_1";
      `AluBitwiseCtrlEnum_defaultEncoding_SRC1 : execute_ALU_BITWISE_CTRL_string = "SRC1 ";
      default : execute_ALU_BITWISE_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_37_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_VexRiscv_37__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_VexRiscv_37__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_VexRiscv_37__string = "AND_1";
      `AluBitwiseCtrlEnum_defaultEncoding_SRC1 : _zz_VexRiscv_37__string = "SRC1 ";
      default : _zz_VexRiscv_37__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_43_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_VexRiscv_43__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_VexRiscv_43__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_VexRiscv_43__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_VexRiscv_43__string = "SRA_1    ";
      default : _zz_VexRiscv_43__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_46_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_VexRiscv_46__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_VexRiscv_46__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_VexRiscv_46__string = "BITWISE ";
      default : _zz_VexRiscv_46__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_50_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_VexRiscv_50__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_VexRiscv_50__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_VexRiscv_50__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_VexRiscv_50__string = "JALR";
      default : _zz_VexRiscv_50__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_51_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_VexRiscv_51__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_VexRiscv_51__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_VexRiscv_51__string = "AND_1";
      `AluBitwiseCtrlEnum_defaultEncoding_SRC1 : _zz_VexRiscv_51__string = "SRC1 ";
      default : _zz_VexRiscv_51__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_52_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_VexRiscv_52__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_VexRiscv_52__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_VexRiscv_52__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_VexRiscv_52__string = "PC ";
      default : _zz_VexRiscv_52__string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_54_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_VexRiscv_54__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_VexRiscv_54__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_VexRiscv_54__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_VexRiscv_54__string = "URS1        ";
      default : _zz_VexRiscv_54__string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_59_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_VexRiscv_59__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_VexRiscv_59__string = "XRET";
      default : _zz_VexRiscv_59__string = "????";
    endcase
  end
  always @(*) begin
    case(memory_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : memory_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : memory_ENV_CTRL_string = "XRET";
      default : memory_ENV_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_61_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_VexRiscv_61__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_VexRiscv_61__string = "XRET";
      default : _zz_VexRiscv_61__string = "????";
    endcase
  end
  always @(*) begin
    case(execute_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : execute_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : execute_ENV_CTRL_string = "XRET";
      default : execute_ENV_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_62_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_VexRiscv_62__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_VexRiscv_62__string = "XRET";
      default : _zz_VexRiscv_62__string = "????";
    endcase
  end
  always @(*) begin
    case(writeBack_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : writeBack_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : writeBack_ENV_CTRL_string = "XRET";
      default : writeBack_ENV_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_65_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_VexRiscv_65__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_VexRiscv_65__string = "XRET";
      default : _zz_VexRiscv_65__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_118_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_VexRiscv_118__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_VexRiscv_118__string = "XRET";
      default : _zz_VexRiscv_118__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_119_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_VexRiscv_119__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_VexRiscv_119__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_VexRiscv_119__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_VexRiscv_119__string = "URS1        ";
      default : _zz_VexRiscv_119__string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_120_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_VexRiscv_120__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_VexRiscv_120__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_VexRiscv_120__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_VexRiscv_120__string = "PC ";
      default : _zz_VexRiscv_120__string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_121_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_VexRiscv_121__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_VexRiscv_121__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_VexRiscv_121__string = "AND_1";
      `AluBitwiseCtrlEnum_defaultEncoding_SRC1 : _zz_VexRiscv_121__string = "SRC1 ";
      default : _zz_VexRiscv_121__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_122_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_VexRiscv_122__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_VexRiscv_122__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_VexRiscv_122__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_VexRiscv_122__string = "JALR";
      default : _zz_VexRiscv_122__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_123_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_VexRiscv_123__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_VexRiscv_123__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_VexRiscv_123__string = "BITWISE ";
      default : _zz_VexRiscv_123__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_VexRiscv_124_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_VexRiscv_124__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_VexRiscv_124__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_VexRiscv_124__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_VexRiscv_124__string = "SRA_1    ";
      default : _zz_VexRiscv_124__string = "?????????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : decode_to_execute_ALU_CTRL_string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : decode_to_execute_ALU_CTRL_string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : decode_to_execute_ALU_CTRL_string = "BITWISE ";
      default : decode_to_execute_ALU_CTRL_string = "????????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : decode_to_execute_ALU_BITWISE_CTRL_string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : decode_to_execute_ALU_BITWISE_CTRL_string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : decode_to_execute_ALU_BITWISE_CTRL_string = "AND_1";
      `AluBitwiseCtrlEnum_defaultEncoding_SRC1 : decode_to_execute_ALU_BITWISE_CTRL_string = "SRC1 ";
      default : decode_to_execute_ALU_BITWISE_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : decode_to_execute_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : decode_to_execute_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : decode_to_execute_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : decode_to_execute_SHIFT_CTRL_string = "SRA_1    ";
      default : decode_to_execute_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : decode_to_execute_BRANCH_CTRL_string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : decode_to_execute_BRANCH_CTRL_string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : decode_to_execute_BRANCH_CTRL_string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : decode_to_execute_BRANCH_CTRL_string = "JALR";
      default : decode_to_execute_BRANCH_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : decode_to_execute_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : decode_to_execute_ENV_CTRL_string = "XRET";
      default : decode_to_execute_ENV_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(execute_to_memory_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : execute_to_memory_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : execute_to_memory_ENV_CTRL_string = "XRET";
      default : execute_to_memory_ENV_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(memory_to_writeBack_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : memory_to_writeBack_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : memory_to_writeBack_ENV_CTRL_string = "XRET";
      default : memory_to_writeBack_ENV_CTRL_string = "????";
    endcase
  end
  `endif

  assign decode_CSR_READ_OPCODE = _zz_VexRiscv_63_;
  assign decode_CSR_WRITE_OPCODE = _zz_VexRiscv_64_;
  assign decode_IS_CSR = _zz_VexRiscv_49_;
  assign writeBack_FORMAL_PC_NEXT = memory_to_writeBack_FORMAL_PC_NEXT;
  assign memory_FORMAL_PC_NEXT = execute_to_memory_FORMAL_PC_NEXT;
  assign execute_FORMAL_PC_NEXT = decode_to_execute_FORMAL_PC_NEXT;
  assign decode_FORMAL_PC_NEXT = _zz_VexRiscv_70_;
  assign decode_BYPASSABLE_EXECUTE_STAGE = _zz_VexRiscv_44_;
  assign memory_MEMORY_ADDRESS_LOW = execute_to_memory_MEMORY_ADDRESS_LOW;
  assign execute_MEMORY_ADDRESS_LOW = _zz_VexRiscv_68_;
  assign memory_PC = execute_to_memory_PC;
  assign writeBack_REGFILE_WRITE_DATA = memory_to_writeBack_REGFILE_WRITE_DATA;
  assign execute_REGFILE_WRITE_DATA = _zz_VexRiscv_36_;
  assign decode_RS2 = _zz_VexRiscv_41_;
  assign decode_DO_EBREAK = _zz_VexRiscv_20_;
  assign decode_SRC1 = _zz_VexRiscv_34_;
  assign execute_BYPASSABLE_MEMORY_STAGE = decode_to_execute_BYPASSABLE_MEMORY_STAGE;
  assign decode_BYPASSABLE_MEMORY_STAGE = _zz_VexRiscv_48_;
  assign decode_RS1 = _zz_VexRiscv_42_;
  assign execute_BRANCH_CALC = _zz_VexRiscv_21_;
  assign memory_MEMORY_READ_DATA = _zz_VexRiscv_67_;
  assign execute_BRANCH_DO = _zz_VexRiscv_23_;
  assign _zz_VexRiscv_1_ = _zz_VexRiscv_2_;
  assign _zz_VexRiscv_3_ = _zz_VexRiscv_4_;
  assign decode_ENV_CTRL = _zz_VexRiscv_5_;
  assign _zz_VexRiscv_6_ = _zz_VexRiscv_7_;
  assign decode_SRC_LESS_UNSIGNED = _zz_VexRiscv_58_;
  assign decode_BRANCH_CTRL = _zz_VexRiscv_8_;
  assign _zz_VexRiscv_9_ = _zz_VexRiscv_10_;
  assign decode_SRC2 = _zz_VexRiscv_31_;
  assign decode_SHIFT_CTRL = _zz_VexRiscv_11_;
  assign _zz_VexRiscv_12_ = _zz_VexRiscv_13_;
  assign decode_MEMORY_ENABLE = _zz_VexRiscv_45_;
  assign decode_SRC_USE_SUB_LESS = _zz_VexRiscv_57_;
  assign decode_ALU_BITWISE_CTRL = _zz_VexRiscv_14_;
  assign _zz_VexRiscv_15_ = _zz_VexRiscv_16_;
  assign decode_ALU_CTRL = _zz_VexRiscv_17_;
  assign _zz_VexRiscv_18_ = _zz_VexRiscv_19_;
  assign execute_DO_EBREAK = decode_to_execute_DO_EBREAK;
  assign decode_IS_EBREAK = _zz_VexRiscv_56_;
  assign memory_BRANCH_CALC = execute_to_memory_BRANCH_CALC;
  assign memory_BRANCH_DO = execute_to_memory_BRANCH_DO;
  assign execute_PC = decode_to_execute_PC;
  assign execute_RS1 = decode_to_execute_RS1;
  assign execute_BRANCH_CTRL = _zz_VexRiscv_22_;
  assign decode_RS2_USE = _zz_VexRiscv_53_;
  assign decode_RS1_USE = _zz_VexRiscv_55_;
  assign execute_REGFILE_WRITE_VALID = decode_to_execute_REGFILE_WRITE_VALID;
  assign execute_BYPASSABLE_EXECUTE_STAGE = decode_to_execute_BYPASSABLE_EXECUTE_STAGE;
  assign memory_REGFILE_WRITE_VALID = execute_to_memory_REGFILE_WRITE_VALID;
  assign memory_BYPASSABLE_MEMORY_STAGE = execute_to_memory_BYPASSABLE_MEMORY_STAGE;
  assign writeBack_REGFILE_WRITE_VALID = memory_to_writeBack_REGFILE_WRITE_VALID;
  assign memory_REGFILE_WRITE_DATA = execute_to_memory_REGFILE_WRITE_DATA;
  assign execute_SHIFT_CTRL = _zz_VexRiscv_24_;
  assign execute_SRC_LESS_UNSIGNED = decode_to_execute_SRC_LESS_UNSIGNED;
  assign execute_SRC_USE_SUB_LESS = decode_to_execute_SRC_USE_SUB_LESS;
  assign _zz_VexRiscv_28_ = decode_PC;
  assign _zz_VexRiscv_29_ = decode_RS2;
  assign decode_SRC2_CTRL = _zz_VexRiscv_30_;
  assign _zz_VexRiscv_32_ = decode_RS1;
  assign decode_SRC1_CTRL = _zz_VexRiscv_33_;
  assign execute_SRC_ADD_SUB = _zz_VexRiscv_27_;
  assign execute_SRC_LESS = _zz_VexRiscv_25_;
  assign execute_ALU_CTRL = _zz_VexRiscv_35_;
  assign execute_SRC2 = decode_to_execute_SRC2;
  assign execute_ALU_BITWISE_CTRL = _zz_VexRiscv_37_;
  assign _zz_VexRiscv_38_ = writeBack_INSTRUCTION;
  assign _zz_VexRiscv_39_ = writeBack_REGFILE_WRITE_VALID;
  always @ (*) begin
    _zz_VexRiscv_40_ = 1'b0;
    if(writeBack_RegFilePlugin_regFileWrite_valid)begin
      _zz_VexRiscv_40_ = 1'b1;
    end
  end

  assign decode_INSTRUCTION_ANTICIPATED = _zz_VexRiscv_73_;
  always @ (*) begin
    decode_REGFILE_WRITE_VALID = _zz_VexRiscv_47_;
    if((decode_INSTRUCTION[11 : 7] == (5'b00000)))begin
      decode_REGFILE_WRITE_VALID = 1'b0;
    end
  end

  always @ (*) begin
    _zz_VexRiscv_60_ = execute_REGFILE_WRITE_DATA;
    execute_arbitration_haltItself = 1'b0;
    if(((((execute_arbitration_isValid && execute_MEMORY_ENABLE) && (! dBus_cmd_ready)) && (! execute_ALIGNEMENT_FAULT)) && (! execute_DBusSimplePlugin_cmdSent)))begin
      execute_arbitration_haltItself = 1'b1;
    end
    if((execute_arbitration_isValid && execute_IS_CSR))begin
      _zz_VexRiscv_60_ = execute_CsrPlugin_readData;
      if(execute_CsrPlugin_blockedBySideEffects)begin
        execute_arbitration_haltItself = 1'b1;
      end
    end
    if(_zz_VexRiscv_155_)begin
      _zz_VexRiscv_60_ = _zz_VexRiscv_133_;
      if(_zz_VexRiscv_156_)begin
        if(! execute_LightShifterPlugin_done) begin
          execute_arbitration_haltItself = 1'b1;
        end
      end
    end
  end

  assign execute_SRC1 = decode_to_execute_SRC1;
  assign execute_CSR_READ_OPCODE = decode_to_execute_CSR_READ_OPCODE;
  assign execute_CSR_WRITE_OPCODE = decode_to_execute_CSR_WRITE_OPCODE;
  assign execute_IS_CSR = decode_to_execute_IS_CSR;
  assign memory_ENV_CTRL = _zz_VexRiscv_61_;
  assign execute_ENV_CTRL = _zz_VexRiscv_62_;
  assign writeBack_ENV_CTRL = _zz_VexRiscv_65_;
  always @ (*) begin
    _zz_VexRiscv_66_ = writeBack_REGFILE_WRITE_DATA;
    if((writeBack_arbitration_isValid && writeBack_MEMORY_ENABLE))begin
      _zz_VexRiscv_66_ = writeBack_DBusSimplePlugin_rspFormated;
    end
  end

  assign writeBack_MEMORY_ENABLE = memory_to_writeBack_MEMORY_ENABLE;
  assign writeBack_MEMORY_ADDRESS_LOW = memory_to_writeBack_MEMORY_ADDRESS_LOW;
  assign writeBack_MEMORY_READ_DATA = memory_to_writeBack_MEMORY_READ_DATA;
  assign memory_INSTRUCTION = execute_to_memory_INSTRUCTION;
  assign memory_MEMORY_ENABLE = execute_to_memory_MEMORY_ENABLE;
  assign execute_RS2 = decode_to_execute_RS2;
  assign execute_SRC_ADD = _zz_VexRiscv_26_;
  assign execute_INSTRUCTION = decode_to_execute_INSTRUCTION;
  assign execute_ALIGNEMENT_FAULT = 1'b0;
  assign execute_MEMORY_ENABLE = decode_to_execute_MEMORY_ENABLE;
  always @ (*) begin
    _zz_VexRiscv_69_ = memory_FORMAL_PC_NEXT;
    if(_zz_VexRiscv_80_)begin
      _zz_VexRiscv_69_ = _zz_VexRiscv_81_;
    end
  end

  assign writeBack_PC = memory_to_writeBack_PC;
  assign writeBack_INSTRUCTION = memory_to_writeBack_INSTRUCTION;
  assign decode_PC = _zz_VexRiscv_72_;
  assign decode_INSTRUCTION = _zz_VexRiscv_71_;
  always @ (*) begin
    decode_arbitration_haltItself = 1'b0;
    decode_arbitration_isValid = (IBusSimplePlugin_injector_decodeInput_valid && (! IBusSimplePlugin_injector_decodeRemoved));
    _zz_VexRiscv_83_ = 1'b0;
    case(_zz_VexRiscv_150_)
      3'b000 : begin
      end
      3'b001 : begin
      end
      3'b010 : begin
        decode_arbitration_isValid = 1'b1;
        decode_arbitration_haltItself = 1'b1;
      end
      3'b011 : begin
        decode_arbitration_isValid = 1'b1;
      end
      3'b100 : begin
        _zz_VexRiscv_83_ = 1'b1;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    decode_arbitration_haltByOther = 1'b0;
    if((CsrPlugin_interrupt && decode_arbitration_isValid))begin
      decode_arbitration_haltByOther = 1'b1;
    end
    if(({(memory_arbitration_isValid && (memory_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET)),(execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET))} != (2'b00)))begin
      decode_arbitration_haltByOther = 1'b1;
    end
    if((decode_arbitration_isValid && (_zz_VexRiscv_134_ || _zz_VexRiscv_135_)))begin
      decode_arbitration_haltByOther = 1'b1;
    end
  end

  always @ (*) begin
    decode_arbitration_removeIt = 1'b0;
    if(decode_arbitration_isFlushed)begin
      decode_arbitration_removeIt = 1'b1;
    end
  end

  assign decode_arbitration_flushAll = 1'b0;
  assign decode_arbitration_redoIt = 1'b0;
  always @ (*) begin
    execute_arbitration_haltByOther = 1'b0;
    _zz_VexRiscv_74_ = 1'b0;
    _zz_VexRiscv_75_ = 1'b0;
    if(_zz_VexRiscv_157_)begin
      execute_arbitration_haltByOther = 1'b1;
      if(_zz_VexRiscv_158_)begin
        _zz_VexRiscv_75_ = 1'b1;
        _zz_VexRiscv_74_ = 1'b1;
      end
    end
    if(DebugPlugin_haltIt)begin
      _zz_VexRiscv_74_ = 1'b1;
    end
    if(_zz_VexRiscv_159_)begin
      _zz_VexRiscv_74_ = 1'b1;
    end
  end

  always @ (*) begin
    execute_arbitration_removeIt = 1'b0;
    if(execute_arbitration_isFlushed)begin
      execute_arbitration_removeIt = 1'b1;
    end
  end

  always @ (*) begin
    execute_arbitration_flushAll = 1'b0;
    if(_zz_VexRiscv_80_)begin
      execute_arbitration_flushAll = 1'b1;
    end
    if(_zz_VexRiscv_157_)begin
      if(_zz_VexRiscv_158_)begin
        execute_arbitration_flushAll = 1'b1;
      end
    end
  end

  assign execute_arbitration_redoIt = 1'b0;
  always @ (*) begin
    memory_arbitration_haltItself = 1'b0;
    if((((memory_arbitration_isValid && memory_MEMORY_ENABLE) && (! memory_INSTRUCTION[5])) && (! dBus_rsp_ready)))begin
      memory_arbitration_haltItself = 1'b1;
    end
  end

  assign memory_arbitration_haltByOther = 1'b0;
  always @ (*) begin
    memory_arbitration_removeIt = 1'b0;
    if(memory_arbitration_isFlushed)begin
      memory_arbitration_removeIt = 1'b1;
    end
  end

  always @ (*) begin
    memory_arbitration_flushAll = 1'b0;
    _zz_VexRiscv_77_ = 1'b0;
    _zz_VexRiscv_78_ = (32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx);
    if(_zz_VexRiscv_160_)begin
      _zz_VexRiscv_77_ = 1'b1;
      _zz_VexRiscv_78_ = {CsrPlugin_mtvec_base,(2'b00)};
      memory_arbitration_flushAll = 1'b1;
    end
    if(_zz_VexRiscv_161_)begin
      _zz_VexRiscv_78_ = CsrPlugin_mepc;
      _zz_VexRiscv_77_ = 1'b1;
      memory_arbitration_flushAll = 1'b1;
    end
  end

  assign memory_arbitration_redoIt = 1'b0;
  assign writeBack_arbitration_haltItself = 1'b0;
  assign writeBack_arbitration_haltByOther = 1'b0;
  always @ (*) begin
    writeBack_arbitration_removeIt = 1'b0;
    if(writeBack_arbitration_isFlushed)begin
      writeBack_arbitration_removeIt = 1'b1;
    end
  end

  assign writeBack_arbitration_flushAll = 1'b0;
  assign writeBack_arbitration_redoIt = 1'b0;
  always @ (*) begin
    _zz_VexRiscv_76_ = 1'b0;
    if((IBusSimplePlugin_iBusRsp_stages_1_input_valid || IBusSimplePlugin_iBusRsp_stages_2_input_valid))begin
      _zz_VexRiscv_76_ = 1'b1;
    end
    if(IBusSimplePlugin_injector_decodeInput_valid)begin
      _zz_VexRiscv_76_ = 1'b1;
    end
  end

  always @ (*) begin
    _zz_VexRiscv_79_ = 1'b1;
    if((DebugPlugin_haltIt || DebugPlugin_stepIt))begin
      _zz_VexRiscv_79_ = 1'b0;
    end
  end

  assign IBusSimplePlugin_jump_pcLoad_valid = ({_zz_VexRiscv_80_,_zz_VexRiscv_77_} != (2'b00));
  assign _zz_VexRiscv_84_ = {_zz_VexRiscv_80_,_zz_VexRiscv_77_};
  assign IBusSimplePlugin_jump_pcLoad_payload = (_zz_VexRiscv_167_[0] ? _zz_VexRiscv_78_ : _zz_VexRiscv_81_);
  assign _zz_VexRiscv_85_ = (! _zz_VexRiscv_74_);
  assign IBusSimplePlugin_fetchPc_output_valid = (IBusSimplePlugin_fetchPc_preOutput_valid && _zz_VexRiscv_85_);
  assign IBusSimplePlugin_fetchPc_preOutput_ready = (IBusSimplePlugin_fetchPc_output_ready && _zz_VexRiscv_85_);
  assign IBusSimplePlugin_fetchPc_output_payload = IBusSimplePlugin_fetchPc_preOutput_payload;
  always @ (*) begin
    IBusSimplePlugin_fetchPc_propagatePc = 1'b0;
    if((IBusSimplePlugin_iBusRsp_stages_1_input_valid && IBusSimplePlugin_iBusRsp_stages_1_input_ready))begin
      IBusSimplePlugin_fetchPc_propagatePc = 1'b1;
    end
  end

  always @ (*) begin
    IBusSimplePlugin_fetchPc_pc = (IBusSimplePlugin_fetchPc_pcReg + _zz_VexRiscv_170_);
    IBusSimplePlugin_fetchPc_samplePcNext = 1'b0;
    if(IBusSimplePlugin_fetchPc_propagatePc)begin
      IBusSimplePlugin_fetchPc_samplePcNext = 1'b1;
    end
    if(IBusSimplePlugin_jump_pcLoad_valid)begin
      IBusSimplePlugin_fetchPc_samplePcNext = 1'b1;
      IBusSimplePlugin_fetchPc_pc = IBusSimplePlugin_jump_pcLoad_payload;
    end
    if(_zz_VexRiscv_162_)begin
      IBusSimplePlugin_fetchPc_samplePcNext = 1'b1;
    end
    IBusSimplePlugin_fetchPc_pc[0] = 1'b0;
    IBusSimplePlugin_fetchPc_pc[1] = 1'b0;
  end

  assign IBusSimplePlugin_fetchPc_preOutput_valid = _zz_VexRiscv_86_;
  assign IBusSimplePlugin_fetchPc_preOutput_payload = IBusSimplePlugin_fetchPc_pc;
  assign IBusSimplePlugin_iBusRsp_stages_0_input_valid = IBusSimplePlugin_fetchPc_output_valid;
  assign IBusSimplePlugin_fetchPc_output_ready = IBusSimplePlugin_iBusRsp_stages_0_input_ready;
  assign IBusSimplePlugin_iBusRsp_stages_0_input_payload = IBusSimplePlugin_fetchPc_output_payload;
  assign IBusSimplePlugin_iBusRsp_stages_0_inputSample = 1'b1;
  assign IBusSimplePlugin_iBusRsp_stages_0_halt = 1'b0;
  assign _zz_VexRiscv_87_ = (! IBusSimplePlugin_iBusRsp_stages_0_halt);
  assign IBusSimplePlugin_iBusRsp_stages_0_input_ready = (IBusSimplePlugin_iBusRsp_stages_0_output_ready && _zz_VexRiscv_87_);
  assign IBusSimplePlugin_iBusRsp_stages_0_output_valid = (IBusSimplePlugin_iBusRsp_stages_0_input_valid && _zz_VexRiscv_87_);
  assign IBusSimplePlugin_iBusRsp_stages_0_output_payload = IBusSimplePlugin_iBusRsp_stages_0_input_payload;
  always @ (*) begin
    IBusSimplePlugin_iBusRsp_stages_1_halt = 1'b0;
    if((IBusSimplePlugin_iBusRsp_stages_1_input_valid && ((! IBusSimplePlugin_cmd_valid) || (! IBusSimplePlugin_cmd_ready))))begin
      IBusSimplePlugin_iBusRsp_stages_1_halt = 1'b1;
    end
  end

  assign _zz_VexRiscv_88_ = (! IBusSimplePlugin_iBusRsp_stages_1_halt);
  assign IBusSimplePlugin_iBusRsp_stages_1_input_ready = (IBusSimplePlugin_iBusRsp_stages_1_output_ready && _zz_VexRiscv_88_);
  assign IBusSimplePlugin_iBusRsp_stages_1_output_valid = (IBusSimplePlugin_iBusRsp_stages_1_input_valid && _zz_VexRiscv_88_);
  assign IBusSimplePlugin_iBusRsp_stages_1_output_payload = IBusSimplePlugin_iBusRsp_stages_1_input_payload;
  assign IBusSimplePlugin_iBusRsp_stages_2_halt = 1'b0;
  assign _zz_VexRiscv_89_ = (! IBusSimplePlugin_iBusRsp_stages_2_halt);
  assign IBusSimplePlugin_iBusRsp_stages_2_input_ready = (IBusSimplePlugin_iBusRsp_stages_2_output_ready && _zz_VexRiscv_89_);
  assign IBusSimplePlugin_iBusRsp_stages_2_output_valid = (IBusSimplePlugin_iBusRsp_stages_2_input_valid && _zz_VexRiscv_89_);
  assign IBusSimplePlugin_iBusRsp_stages_2_output_payload = IBusSimplePlugin_iBusRsp_stages_2_input_payload;
  assign IBusSimplePlugin_iBusRsp_stages_0_output_ready = _zz_VexRiscv_90_;
  assign _zz_VexRiscv_90_ = ((1'b0 && (! _zz_VexRiscv_91_)) || IBusSimplePlugin_iBusRsp_stages_1_input_ready);
  assign _zz_VexRiscv_91_ = _zz_VexRiscv_92_;
  assign IBusSimplePlugin_iBusRsp_stages_1_input_valid = _zz_VexRiscv_91_;
  assign IBusSimplePlugin_iBusRsp_stages_1_input_payload = IBusSimplePlugin_fetchPc_pcReg;
  assign IBusSimplePlugin_iBusRsp_stages_1_output_ready = ((1'b0 && (! _zz_VexRiscv_93_)) || IBusSimplePlugin_iBusRsp_stages_2_input_ready);
  assign _zz_VexRiscv_93_ = _zz_VexRiscv_94_;
  assign IBusSimplePlugin_iBusRsp_stages_2_input_valid = _zz_VexRiscv_93_;
  assign IBusSimplePlugin_iBusRsp_stages_2_input_payload = _zz_VexRiscv_95_;
  always @ (*) begin
    IBusSimplePlugin_iBusRsp_readyForError = 1'b1;
    if(IBusSimplePlugin_injector_decodeInput_valid)begin
      IBusSimplePlugin_iBusRsp_readyForError = 1'b0;
    end
  end

  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_ready = ((1'b0 && (! IBusSimplePlugin_injector_decodeInput_valid)) || IBusSimplePlugin_injector_decodeInput_ready);
  assign IBusSimplePlugin_injector_decodeInput_valid = _zz_VexRiscv_96_;
  assign IBusSimplePlugin_injector_decodeInput_payload_pc = _zz_VexRiscv_97_;
  assign IBusSimplePlugin_injector_decodeInput_payload_rsp_error = _zz_VexRiscv_98_;
  assign IBusSimplePlugin_injector_decodeInput_payload_rsp_inst = _zz_VexRiscv_99_;
  assign IBusSimplePlugin_injector_decodeInput_payload_isRvc = _zz_VexRiscv_100_;
  assign _zz_VexRiscv_73_ = (decode_arbitration_isStuck ? decode_INSTRUCTION : IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_inst);
  assign IBusSimplePlugin_injector_decodeInput_ready = (! decode_arbitration_isStuck);
  assign _zz_VexRiscv_72_ = IBusSimplePlugin_injector_decodeInput_payload_pc;
  assign _zz_VexRiscv_71_ = IBusSimplePlugin_injector_decodeInput_payload_rsp_inst;
  assign _zz_VexRiscv_70_ = (decode_PC + (32'b00000000000000000000000000000100));
  assign iBus_cmd_valid = IBusSimplePlugin_cmd_valid;
  assign IBusSimplePlugin_cmd_ready = iBus_cmd_ready;
  assign iBus_cmd_payload_pc = IBusSimplePlugin_cmd_payload_pc;
  assign IBusSimplePlugin_pendingCmdNext = (_zz_VexRiscv_171_ - _zz_VexRiscv_175_);
  assign IBusSimplePlugin_cmd_valid = ((IBusSimplePlugin_iBusRsp_stages_1_input_valid && IBusSimplePlugin_iBusRsp_stages_1_output_ready) && (IBusSimplePlugin_pendingCmd != (3'b111)));
  assign IBusSimplePlugin_cmd_payload_pc = {IBusSimplePlugin_iBusRsp_stages_1_input_payload[31 : 2],(2'b00)};
  assign iBus_rsp_takeWhen_valid = (iBus_rsp_valid && (! (IBusSimplePlugin_rspJoin_discardCounter != (3'b000))));
  assign iBus_rsp_takeWhen_payload_error = iBus_rsp_payload_error;
  assign iBus_rsp_takeWhen_payload_inst = iBus_rsp_payload_inst;
  assign _zz_VexRiscv_152_ = (IBusSimplePlugin_jump_pcLoad_valid || _zz_VexRiscv_75_);
  assign IBusSimplePlugin_rspJoin_rspBufferOutput_valid = IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid;
  assign IBusSimplePlugin_rspJoin_rspBufferOutput_payload_error = IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error;
  assign IBusSimplePlugin_rspJoin_rspBufferOutput_payload_inst = IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst;
  assign IBusSimplePlugin_rspJoin_fetchRsp_pc = IBusSimplePlugin_iBusRsp_stages_2_output_payload;
  always @ (*) begin
    IBusSimplePlugin_rspJoin_fetchRsp_rsp_error = IBusSimplePlugin_rspJoin_rspBufferOutput_payload_error;
    if((! IBusSimplePlugin_rspJoin_rspBufferOutput_valid))begin
      IBusSimplePlugin_rspJoin_fetchRsp_rsp_error = 1'b0;
    end
  end

  assign IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst = IBusSimplePlugin_rspJoin_rspBufferOutput_payload_inst;
  assign IBusSimplePlugin_rspJoin_issueDetected = 1'b0;
  assign IBusSimplePlugin_rspJoin_join_valid = (IBusSimplePlugin_iBusRsp_stages_2_output_valid && IBusSimplePlugin_rspJoin_rspBufferOutput_valid);
  assign IBusSimplePlugin_rspJoin_join_payload_pc = IBusSimplePlugin_rspJoin_fetchRsp_pc;
  assign IBusSimplePlugin_rspJoin_join_payload_rsp_error = IBusSimplePlugin_rspJoin_fetchRsp_rsp_error;
  assign IBusSimplePlugin_rspJoin_join_payload_rsp_inst = IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst;
  assign IBusSimplePlugin_rspJoin_join_payload_isRvc = IBusSimplePlugin_rspJoin_fetchRsp_isRvc;
  assign IBusSimplePlugin_iBusRsp_stages_2_output_ready = (IBusSimplePlugin_iBusRsp_stages_2_output_valid ? (IBusSimplePlugin_rspJoin_join_valid && IBusSimplePlugin_rspJoin_join_ready) : IBusSimplePlugin_rspJoin_join_ready);
  assign IBusSimplePlugin_rspJoin_rspBufferOutput_ready = (IBusSimplePlugin_rspJoin_join_valid && IBusSimplePlugin_rspJoin_join_ready);
  assign _zz_VexRiscv_101_ = (! IBusSimplePlugin_rspJoin_issueDetected);
  assign IBusSimplePlugin_rspJoin_join_ready = (IBusSimplePlugin_iBusRsp_inputBeforeStage_ready && _zz_VexRiscv_101_);
  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_valid = (IBusSimplePlugin_rspJoin_join_valid && _zz_VexRiscv_101_);
  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_pc = IBusSimplePlugin_rspJoin_join_payload_pc;
  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_error = IBusSimplePlugin_rspJoin_join_payload_rsp_error;
  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_inst = IBusSimplePlugin_rspJoin_join_payload_rsp_inst;
  assign IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_isRvc = IBusSimplePlugin_rspJoin_join_payload_isRvc;
  assign execute_DBusSimplePlugin_cmdSent = 1'b0;
  assign dBus_cmd_valid = (((((execute_arbitration_isValid && execute_MEMORY_ENABLE) && (! execute_arbitration_isStuckByOthers)) && (! execute_arbitration_isFlushed)) && (! execute_ALIGNEMENT_FAULT)) && (! execute_DBusSimplePlugin_cmdSent));
  assign dBus_cmd_payload_wr = execute_INSTRUCTION[5];
  assign dBus_cmd_payload_address = execute_SRC_ADD;
  assign dBus_cmd_payload_size = execute_INSTRUCTION[13 : 12];
  always @ (*) begin
    case(dBus_cmd_payload_size)
      2'b00 : begin
        _zz_VexRiscv_102_ = {{{execute_RS2[7 : 0],execute_RS2[7 : 0]},execute_RS2[7 : 0]},execute_RS2[7 : 0]};
      end
      2'b01 : begin
        _zz_VexRiscv_102_ = {execute_RS2[15 : 0],execute_RS2[15 : 0]};
      end
      default : begin
        _zz_VexRiscv_102_ = execute_RS2[31 : 0];
      end
    endcase
  end

  assign dBus_cmd_payload_data = _zz_VexRiscv_102_;
  assign _zz_VexRiscv_68_ = dBus_cmd_payload_address[1 : 0];
  always @ (*) begin
    case(dBus_cmd_payload_size)
      2'b00 : begin
        _zz_VexRiscv_103_ = (4'b0001);
      end
      2'b01 : begin
        _zz_VexRiscv_103_ = (4'b0011);
      end
      default : begin
        _zz_VexRiscv_103_ = (4'b1111);
      end
    endcase
  end

  assign execute_DBusSimplePlugin_formalMask = (_zz_VexRiscv_103_ <<< dBus_cmd_payload_address[1 : 0]);
  assign _zz_VexRiscv_67_ = dBus_rsp_data;
  always @ (*) begin
    writeBack_DBusSimplePlugin_rspShifted = writeBack_MEMORY_READ_DATA;
    case(writeBack_MEMORY_ADDRESS_LOW)
      2'b01 : begin
        writeBack_DBusSimplePlugin_rspShifted[7 : 0] = writeBack_MEMORY_READ_DATA[15 : 8];
      end
      2'b10 : begin
        writeBack_DBusSimplePlugin_rspShifted[15 : 0] = writeBack_MEMORY_READ_DATA[31 : 16];
      end
      2'b11 : begin
        writeBack_DBusSimplePlugin_rspShifted[7 : 0] = writeBack_MEMORY_READ_DATA[31 : 24];
      end
      default : begin
      end
    endcase
  end

  assign _zz_VexRiscv_104_ = (writeBack_DBusSimplePlugin_rspShifted[7] && (! writeBack_INSTRUCTION[14]));
  always @ (*) begin
    _zz_VexRiscv_105_[31] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[30] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[29] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[28] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[27] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[26] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[25] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[24] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[23] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[22] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[21] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[20] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[19] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[18] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[17] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[16] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[15] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[14] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[13] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[12] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[11] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[10] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[9] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[8] = _zz_VexRiscv_104_;
    _zz_VexRiscv_105_[7 : 0] = writeBack_DBusSimplePlugin_rspShifted[7 : 0];
  end

  assign _zz_VexRiscv_106_ = (writeBack_DBusSimplePlugin_rspShifted[15] && (! writeBack_INSTRUCTION[14]));
  always @ (*) begin
    _zz_VexRiscv_107_[31] = _zz_VexRiscv_106_;
    _zz_VexRiscv_107_[30] = _zz_VexRiscv_106_;
    _zz_VexRiscv_107_[29] = _zz_VexRiscv_106_;
    _zz_VexRiscv_107_[28] = _zz_VexRiscv_106_;
    _zz_VexRiscv_107_[27] = _zz_VexRiscv_106_;
    _zz_VexRiscv_107_[26] = _zz_VexRiscv_106_;
    _zz_VexRiscv_107_[25] = _zz_VexRiscv_106_;
    _zz_VexRiscv_107_[24] = _zz_VexRiscv_106_;
    _zz_VexRiscv_107_[23] = _zz_VexRiscv_106_;
    _zz_VexRiscv_107_[22] = _zz_VexRiscv_106_;
    _zz_VexRiscv_107_[21] = _zz_VexRiscv_106_;
    _zz_VexRiscv_107_[20] = _zz_VexRiscv_106_;
    _zz_VexRiscv_107_[19] = _zz_VexRiscv_106_;
    _zz_VexRiscv_107_[18] = _zz_VexRiscv_106_;
    _zz_VexRiscv_107_[17] = _zz_VexRiscv_106_;
    _zz_VexRiscv_107_[16] = _zz_VexRiscv_106_;
    _zz_VexRiscv_107_[15 : 0] = writeBack_DBusSimplePlugin_rspShifted[15 : 0];
  end

  always @ (*) begin
    case(_zz_VexRiscv_164_)
      2'b00 : begin
        writeBack_DBusSimplePlugin_rspFormated = _zz_VexRiscv_105_;
      end
      2'b01 : begin
        writeBack_DBusSimplePlugin_rspFormated = _zz_VexRiscv_107_;
      end
      default : begin
        writeBack_DBusSimplePlugin_rspFormated = writeBack_DBusSimplePlugin_rspShifted;
      end
    endcase
  end

  assign CsrPlugin_misa_base = (2'b01);
  assign CsrPlugin_misa_extensions = (26'b00000000000000000001000010);
  assign CsrPlugin_mtvec_mode = (2'b00);
  assign CsrPlugin_mtvec_base = (30'b000000000000000000000000001000);
  assign CsrPlugin_medeleg = (32'b00000000000000000000000000000000);
  assign CsrPlugin_mideleg = (32'b00000000000000000000000000000000);
  assign _zz_VexRiscv_108_ = (CsrPlugin_mip_MTIP && CsrPlugin_mie_MTIE);
  assign _zz_VexRiscv_109_ = (CsrPlugin_mip_MSIP && CsrPlugin_mie_MSIE);
  assign _zz_VexRiscv_110_ = (CsrPlugin_mip_MEIP && CsrPlugin_mie_MEIE);
  always @ (*) begin
    CsrPlugin_interrupt = 1'b0;
    CsrPlugin_interruptCode = (4'bxxxx);
    if(CsrPlugin_mstatus_MIE)begin
      if(({_zz_VexRiscv_110_,{_zz_VexRiscv_109_,_zz_VexRiscv_108_}} != (3'b000)))begin
        CsrPlugin_interrupt = 1'b1;
      end
      if(_zz_VexRiscv_108_)begin
        CsrPlugin_interruptCode = (4'b0111);
      end
      if(_zz_VexRiscv_109_)begin
        CsrPlugin_interruptCode = (4'b0011);
      end
      if(_zz_VexRiscv_110_)begin
        CsrPlugin_interruptCode = (4'b1011);
      end
    end
    if((! _zz_VexRiscv_79_))begin
      CsrPlugin_interrupt = 1'b0;
    end
  end

  assign CsrPlugin_interruptTargetPrivilege = (2'b11);
  assign CsrPlugin_exception = 1'b0;
  assign CsrPlugin_lastStageWasWfi = 1'b0;
  always @ (*) begin
    CsrPlugin_pipelineLiberator_done = ((! ({writeBack_arbitration_isValid,{memory_arbitration_isValid,execute_arbitration_isValid}} != (3'b000))) && IBusSimplePlugin_injector_nextPcCalc_valids_2);
    if(CsrPlugin_hadException)begin
      CsrPlugin_pipelineLiberator_done = 1'b0;
    end
  end

  assign CsrPlugin_interruptJump = (CsrPlugin_interrupt && CsrPlugin_pipelineLiberator_done);
  assign CsrPlugin_targetPrivilege = CsrPlugin_interruptTargetPrivilege;
  assign CsrPlugin_trapCause = CsrPlugin_interruptCode;
  assign contextSwitching = _zz_VexRiscv_77_;
  assign _zz_VexRiscv_64_ = (! (((decode_INSTRUCTION[14 : 13] == (2'b01)) && (decode_INSTRUCTION[19 : 15] == (5'b00000))) || ((decode_INSTRUCTION[14 : 13] == (2'b11)) && (decode_INSTRUCTION[19 : 15] == (5'b00000)))));
  assign _zz_VexRiscv_63_ = (decode_INSTRUCTION[13 : 7] != (7'b0100000));
  assign execute_CsrPlugin_blockedBySideEffects = ({writeBack_arbitration_isValid,memory_arbitration_isValid} != (2'b00));
  always @ (*) begin
    execute_CsrPlugin_illegalAccess = 1'b1;
    execute_CsrPlugin_readData = (32'b00000000000000000000000000000000);
    case(execute_CsrPlugin_csrAddress)
      12'b001100000000 : begin
        execute_CsrPlugin_illegalAccess = 1'b0;
        execute_CsrPlugin_readData[12 : 11] = CsrPlugin_mstatus_MPP;
        execute_CsrPlugin_readData[7 : 7] = CsrPlugin_mstatus_MPIE;
        execute_CsrPlugin_readData[3 : 3] = CsrPlugin_mstatus_MIE;
      end
      12'b001101000100 : begin
        execute_CsrPlugin_illegalAccess = 1'b0;
        execute_CsrPlugin_readData[11 : 11] = CsrPlugin_mip_MEIP;
        execute_CsrPlugin_readData[7 : 7] = CsrPlugin_mip_MTIP;
        execute_CsrPlugin_readData[3 : 3] = CsrPlugin_mip_MSIP;
      end
      12'b001100000100 : begin
        execute_CsrPlugin_illegalAccess = 1'b0;
        execute_CsrPlugin_readData[11 : 11] = CsrPlugin_mie_MEIE;
        execute_CsrPlugin_readData[7 : 7] = CsrPlugin_mie_MTIE;
        execute_CsrPlugin_readData[3 : 3] = CsrPlugin_mie_MSIE;
      end
      12'b001101000010 : begin
        if(execute_CSR_READ_OPCODE)begin
          execute_CsrPlugin_illegalAccess = 1'b0;
        end
        execute_CsrPlugin_readData[31 : 31] = CsrPlugin_mcause_interrupt;
        execute_CsrPlugin_readData[3 : 0] = CsrPlugin_mcause_exceptionCode;
      end
      default : begin
      end
    endcase
    if((CsrPlugin_privilege < execute_CsrPlugin_csrAddress[9 : 8]))begin
      execute_CsrPlugin_illegalAccess = 1'b1;
    end
    if(((! execute_arbitration_isValid) || (! execute_IS_CSR)))begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
  end

  always @ (*) begin
    execute_CsrPlugin_illegalInstruction = 1'b0;
    if((execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET)))begin
      if((execute_INSTRUCTION[29 : 28] != CsrPlugin_privilege))begin
        execute_CsrPlugin_illegalInstruction = 1'b1;
      end
    end
  end

  assign execute_CsrPlugin_writeInstruction = ((execute_arbitration_isValid && execute_IS_CSR) && execute_CSR_WRITE_OPCODE);
  assign execute_CsrPlugin_readInstruction = ((execute_arbitration_isValid && execute_IS_CSR) && execute_CSR_READ_OPCODE);
  assign execute_CsrPlugin_writeEnable = ((execute_CsrPlugin_writeInstruction && (! execute_CsrPlugin_blockedBySideEffects)) && (! execute_arbitration_isStuckByOthers));
  assign execute_CsrPlugin_readEnable = ((execute_CsrPlugin_readInstruction && (! execute_CsrPlugin_blockedBySideEffects)) && (! execute_arbitration_isStuckByOthers));
  always @ (*) begin
    case(_zz_VexRiscv_166_)
      1'b0 : begin
        execute_CsrPlugin_writeData = execute_SRC1;
      end
      default : begin
        execute_CsrPlugin_writeData = (execute_INSTRUCTION[12] ? (execute_CsrPlugin_readData & (~ execute_SRC1)) : (execute_CsrPlugin_readData | execute_SRC1));
      end
    endcase
  end

  assign execute_CsrPlugin_csrAddress = execute_INSTRUCTION[31 : 20];
  assign _zz_VexRiscv_112_ = ((decode_INSTRUCTION & (32'b00000000000000000110000000000100)) == (32'b00000000000000000010000000000000));
  assign _zz_VexRiscv_113_ = ((decode_INSTRUCTION & (32'b00000000000000000100000001010000)) == (32'b00000000000000000100000001010000));
  assign _zz_VexRiscv_114_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000010100)) == (32'b00000000000000000000000000000100));
  assign _zz_VexRiscv_115_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000100000)) == (32'b00000000000000000000000000000000));
  assign _zz_VexRiscv_116_ = ((decode_INSTRUCTION & (32'b00000000000000000000000000000100)) == (32'b00000000000000000000000000000100));
  assign _zz_VexRiscv_117_ = ((decode_INSTRUCTION & (32'b00000000000000000000000001010000)) == (32'b00000000000000000000000000010000));
  assign _zz_VexRiscv_111_ = {(((decode_INSTRUCTION & _zz_VexRiscv_214_) == (32'b00000000000000000101000000010000)) != (1'b0)),{({_zz_VexRiscv_215_,_zz_VexRiscv_216_} != (2'b00)),{({_zz_VexRiscv_217_,_zz_VexRiscv_218_} != (2'b00)),{(_zz_VexRiscv_219_ != _zz_VexRiscv_220_),{_zz_VexRiscv_221_,{_zz_VexRiscv_222_,_zz_VexRiscv_223_}}}}}};
  assign _zz_VexRiscv_118_ = _zz_VexRiscv_111_[0 : 0];
  assign _zz_VexRiscv_59_ = _zz_VexRiscv_118_;
  assign _zz_VexRiscv_58_ = _zz_VexRiscv_178_[0];
  assign _zz_VexRiscv_57_ = _zz_VexRiscv_179_[0];
  assign _zz_VexRiscv_56_ = _zz_VexRiscv_180_[0];
  assign _zz_VexRiscv_55_ = _zz_VexRiscv_181_[0];
  assign _zz_VexRiscv_119_ = _zz_VexRiscv_111_[6 : 5];
  assign _zz_VexRiscv_54_ = _zz_VexRiscv_119_;
  assign _zz_VexRiscv_53_ = _zz_VexRiscv_182_[0];
  assign _zz_VexRiscv_120_ = _zz_VexRiscv_111_[9 : 8];
  assign _zz_VexRiscv_52_ = _zz_VexRiscv_120_;
  assign _zz_VexRiscv_121_ = _zz_VexRiscv_111_[11 : 10];
  assign _zz_VexRiscv_51_ = _zz_VexRiscv_121_;
  assign _zz_VexRiscv_122_ = _zz_VexRiscv_111_[13 : 12];
  assign _zz_VexRiscv_50_ = _zz_VexRiscv_122_;
  assign _zz_VexRiscv_49_ = _zz_VexRiscv_183_[0];
  assign _zz_VexRiscv_48_ = _zz_VexRiscv_184_[0];
  assign _zz_VexRiscv_47_ = _zz_VexRiscv_185_[0];
  assign _zz_VexRiscv_123_ = _zz_VexRiscv_111_[18 : 17];
  assign _zz_VexRiscv_46_ = _zz_VexRiscv_123_;
  assign _zz_VexRiscv_45_ = _zz_VexRiscv_186_[0];
  assign _zz_VexRiscv_44_ = _zz_VexRiscv_187_[0];
  assign _zz_VexRiscv_124_ = _zz_VexRiscv_111_[23 : 22];
  assign _zz_VexRiscv_43_ = _zz_VexRiscv_124_;
  assign decode_RegFilePlugin_regFileReadAddress1 = decode_INSTRUCTION_ANTICIPATED[19 : 15];
  assign decode_RegFilePlugin_regFileReadAddress2 = decode_INSTRUCTION_ANTICIPATED[24 : 20];
  assign decode_RegFilePlugin_rs1Data = _zz_VexRiscv_153_;
  assign decode_RegFilePlugin_rs2Data = _zz_VexRiscv_154_;
  assign _zz_VexRiscv_42_ = decode_RegFilePlugin_rs1Data;
  assign _zz_VexRiscv_41_ = decode_RegFilePlugin_rs2Data;
  always @ (*) begin
    writeBack_RegFilePlugin_regFileWrite_valid = (_zz_VexRiscv_39_ && writeBack_arbitration_isFiring);
    if(_zz_VexRiscv_125_)begin
      writeBack_RegFilePlugin_regFileWrite_valid = 1'b1;
    end
  end

  assign writeBack_RegFilePlugin_regFileWrite_payload_address = _zz_VexRiscv_38_[11 : 7];
  assign writeBack_RegFilePlugin_regFileWrite_payload_data = _zz_VexRiscv_66_;
  always @ (*) begin
    case(execute_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 & execute_SRC2);
      end
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 | execute_SRC2);
      end
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 ^ execute_SRC2);
      end
      default : begin
        execute_IntAluPlugin_bitwise = execute_SRC1;
      end
    endcase
  end

  always @ (*) begin
    case(execute_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_BITWISE : begin
        _zz_VexRiscv_126_ = execute_IntAluPlugin_bitwise;
      end
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : begin
        _zz_VexRiscv_126_ = {31'd0, _zz_VexRiscv_188_};
      end
      default : begin
        _zz_VexRiscv_126_ = execute_SRC_ADD_SUB;
      end
    endcase
  end

  assign _zz_VexRiscv_36_ = _zz_VexRiscv_126_;
  always @ (*) begin
    case(decode_SRC1_CTRL)
      `Src1CtrlEnum_defaultEncoding_RS : begin
        _zz_VexRiscv_127_ = _zz_VexRiscv_32_;
      end
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : begin
        _zz_VexRiscv_127_ = {29'd0, _zz_VexRiscv_189_};
      end
      `Src1CtrlEnum_defaultEncoding_IMU : begin
        _zz_VexRiscv_127_ = {decode_INSTRUCTION[31 : 12],(12'b000000000000)};
      end
      default : begin
        _zz_VexRiscv_127_ = {27'd0, _zz_VexRiscv_190_};
      end
    endcase
  end

  assign _zz_VexRiscv_34_ = _zz_VexRiscv_127_;
  assign _zz_VexRiscv_128_ = _zz_VexRiscv_191_[11];
  always @ (*) begin
    _zz_VexRiscv_129_[19] = _zz_VexRiscv_128_;
    _zz_VexRiscv_129_[18] = _zz_VexRiscv_128_;
    _zz_VexRiscv_129_[17] = _zz_VexRiscv_128_;
    _zz_VexRiscv_129_[16] = _zz_VexRiscv_128_;
    _zz_VexRiscv_129_[15] = _zz_VexRiscv_128_;
    _zz_VexRiscv_129_[14] = _zz_VexRiscv_128_;
    _zz_VexRiscv_129_[13] = _zz_VexRiscv_128_;
    _zz_VexRiscv_129_[12] = _zz_VexRiscv_128_;
    _zz_VexRiscv_129_[11] = _zz_VexRiscv_128_;
    _zz_VexRiscv_129_[10] = _zz_VexRiscv_128_;
    _zz_VexRiscv_129_[9] = _zz_VexRiscv_128_;
    _zz_VexRiscv_129_[8] = _zz_VexRiscv_128_;
    _zz_VexRiscv_129_[7] = _zz_VexRiscv_128_;
    _zz_VexRiscv_129_[6] = _zz_VexRiscv_128_;
    _zz_VexRiscv_129_[5] = _zz_VexRiscv_128_;
    _zz_VexRiscv_129_[4] = _zz_VexRiscv_128_;
    _zz_VexRiscv_129_[3] = _zz_VexRiscv_128_;
    _zz_VexRiscv_129_[2] = _zz_VexRiscv_128_;
    _zz_VexRiscv_129_[1] = _zz_VexRiscv_128_;
    _zz_VexRiscv_129_[0] = _zz_VexRiscv_128_;
  end

  assign _zz_VexRiscv_130_ = _zz_VexRiscv_192_[11];
  always @ (*) begin
    _zz_VexRiscv_131_[19] = _zz_VexRiscv_130_;
    _zz_VexRiscv_131_[18] = _zz_VexRiscv_130_;
    _zz_VexRiscv_131_[17] = _zz_VexRiscv_130_;
    _zz_VexRiscv_131_[16] = _zz_VexRiscv_130_;
    _zz_VexRiscv_131_[15] = _zz_VexRiscv_130_;
    _zz_VexRiscv_131_[14] = _zz_VexRiscv_130_;
    _zz_VexRiscv_131_[13] = _zz_VexRiscv_130_;
    _zz_VexRiscv_131_[12] = _zz_VexRiscv_130_;
    _zz_VexRiscv_131_[11] = _zz_VexRiscv_130_;
    _zz_VexRiscv_131_[10] = _zz_VexRiscv_130_;
    _zz_VexRiscv_131_[9] = _zz_VexRiscv_130_;
    _zz_VexRiscv_131_[8] = _zz_VexRiscv_130_;
    _zz_VexRiscv_131_[7] = _zz_VexRiscv_130_;
    _zz_VexRiscv_131_[6] = _zz_VexRiscv_130_;
    _zz_VexRiscv_131_[5] = _zz_VexRiscv_130_;
    _zz_VexRiscv_131_[4] = _zz_VexRiscv_130_;
    _zz_VexRiscv_131_[3] = _zz_VexRiscv_130_;
    _zz_VexRiscv_131_[2] = _zz_VexRiscv_130_;
    _zz_VexRiscv_131_[1] = _zz_VexRiscv_130_;
    _zz_VexRiscv_131_[0] = _zz_VexRiscv_130_;
  end

  always @ (*) begin
    case(decode_SRC2_CTRL)
      `Src2CtrlEnum_defaultEncoding_RS : begin
        _zz_VexRiscv_132_ = _zz_VexRiscv_29_;
      end
      `Src2CtrlEnum_defaultEncoding_IMI : begin
        _zz_VexRiscv_132_ = {_zz_VexRiscv_129_,decode_INSTRUCTION[31 : 20]};
      end
      `Src2CtrlEnum_defaultEncoding_IMS : begin
        _zz_VexRiscv_132_ = {_zz_VexRiscv_131_,{decode_INSTRUCTION[31 : 25],decode_INSTRUCTION[11 : 7]}};
      end
      default : begin
        _zz_VexRiscv_132_ = _zz_VexRiscv_28_;
      end
    endcase
  end

  assign _zz_VexRiscv_31_ = _zz_VexRiscv_132_;
  assign execute_SrcPlugin_addSub = _zz_VexRiscv_193_;
  assign execute_SrcPlugin_less = ((execute_SRC1[31] == execute_SRC2[31]) ? execute_SrcPlugin_addSub[31] : (execute_SRC_LESS_UNSIGNED ? execute_SRC2[31] : execute_SRC1[31]));
  assign _zz_VexRiscv_27_ = execute_SrcPlugin_addSub;
  assign _zz_VexRiscv_26_ = execute_SrcPlugin_addSub;
  assign _zz_VexRiscv_25_ = execute_SrcPlugin_less;
  assign execute_LightShifterPlugin_isShift = (execute_SHIFT_CTRL != `ShiftCtrlEnum_defaultEncoding_DISABLE_1);
  assign execute_LightShifterPlugin_amplitude = (execute_LightShifterPlugin_isActive ? execute_LightShifterPlugin_amplitudeReg : execute_SRC2[4 : 0]);
  assign execute_LightShifterPlugin_shiftInput = (execute_LightShifterPlugin_isActive ? memory_REGFILE_WRITE_DATA : execute_SRC1);
  assign execute_LightShifterPlugin_done = (execute_LightShifterPlugin_amplitude[4 : 1] == (4'b0000));
  always @ (*) begin
    case(execute_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : begin
        _zz_VexRiscv_133_ = (execute_LightShifterPlugin_shiftInput <<< 1);
      end
      default : begin
        _zz_VexRiscv_133_ = _zz_VexRiscv_201_;
      end
    endcase
  end

  always @ (*) begin
    _zz_VexRiscv_134_ = 1'b0;
    _zz_VexRiscv_135_ = 1'b0;
    if(_zz_VexRiscv_137_)begin
      if((_zz_VexRiscv_138_ == decode_INSTRUCTION[19 : 15]))begin
        _zz_VexRiscv_134_ = 1'b1;
      end
      if((_zz_VexRiscv_138_ == decode_INSTRUCTION[24 : 20]))begin
        _zz_VexRiscv_135_ = 1'b1;
      end
    end
    if((writeBack_arbitration_isValid && writeBack_REGFILE_WRITE_VALID))begin
      if((1'b1 || (! 1'b1)))begin
        if((writeBack_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]))begin
          _zz_VexRiscv_134_ = 1'b1;
        end
        if((writeBack_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]))begin
          _zz_VexRiscv_135_ = 1'b1;
        end
      end
    end
    if((memory_arbitration_isValid && memory_REGFILE_WRITE_VALID))begin
      if((1'b1 || (! memory_BYPASSABLE_MEMORY_STAGE)))begin
        if((memory_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]))begin
          _zz_VexRiscv_134_ = 1'b1;
        end
        if((memory_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]))begin
          _zz_VexRiscv_135_ = 1'b1;
        end
      end
    end
    if((execute_arbitration_isValid && execute_REGFILE_WRITE_VALID))begin
      if((1'b1 || (! execute_BYPASSABLE_EXECUTE_STAGE)))begin
        if((execute_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]))begin
          _zz_VexRiscv_134_ = 1'b1;
        end
        if((execute_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]))begin
          _zz_VexRiscv_135_ = 1'b1;
        end
      end
    end
    if((! decode_RS1_USE))begin
      _zz_VexRiscv_134_ = 1'b0;
    end
    if((! decode_RS2_USE))begin
      _zz_VexRiscv_135_ = 1'b0;
    end
  end

  assign _zz_VexRiscv_136_ = (_zz_VexRiscv_39_ && writeBack_arbitration_isFiring);
  assign execute_BranchPlugin_eq = (execute_SRC1 == execute_SRC2);
  assign _zz_VexRiscv_139_ = execute_INSTRUCTION[14 : 12];
  always @ (*) begin
    if((_zz_VexRiscv_139_ == (3'b000))) begin
        _zz_VexRiscv_140_ = execute_BranchPlugin_eq;
    end else if((_zz_VexRiscv_139_ == (3'b001))) begin
        _zz_VexRiscv_140_ = (! execute_BranchPlugin_eq);
    end else if((((_zz_VexRiscv_139_ & (3'b101)) == (3'b101)))) begin
        _zz_VexRiscv_140_ = (! execute_SRC_LESS);
    end else begin
        _zz_VexRiscv_140_ = execute_SRC_LESS;
    end
  end

  always @ (*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : begin
        _zz_VexRiscv_141_ = 1'b0;
      end
      `BranchCtrlEnum_defaultEncoding_JAL : begin
        _zz_VexRiscv_141_ = 1'b1;
      end
      `BranchCtrlEnum_defaultEncoding_JALR : begin
        _zz_VexRiscv_141_ = 1'b1;
      end
      default : begin
        _zz_VexRiscv_141_ = _zz_VexRiscv_140_;
      end
    endcase
  end

  assign _zz_VexRiscv_23_ = _zz_VexRiscv_141_;
  assign execute_BranchPlugin_branch_src1 = ((execute_BRANCH_CTRL == `BranchCtrlEnum_defaultEncoding_JALR) ? execute_RS1 : execute_PC);
  assign _zz_VexRiscv_142_ = _zz_VexRiscv_203_[19];
  always @ (*) begin
    _zz_VexRiscv_143_[10] = _zz_VexRiscv_142_;
    _zz_VexRiscv_143_[9] = _zz_VexRiscv_142_;
    _zz_VexRiscv_143_[8] = _zz_VexRiscv_142_;
    _zz_VexRiscv_143_[7] = _zz_VexRiscv_142_;
    _zz_VexRiscv_143_[6] = _zz_VexRiscv_142_;
    _zz_VexRiscv_143_[5] = _zz_VexRiscv_142_;
    _zz_VexRiscv_143_[4] = _zz_VexRiscv_142_;
    _zz_VexRiscv_143_[3] = _zz_VexRiscv_142_;
    _zz_VexRiscv_143_[2] = _zz_VexRiscv_142_;
    _zz_VexRiscv_143_[1] = _zz_VexRiscv_142_;
    _zz_VexRiscv_143_[0] = _zz_VexRiscv_142_;
  end

  assign _zz_VexRiscv_144_ = _zz_VexRiscv_204_[11];
  always @ (*) begin
    _zz_VexRiscv_145_[19] = _zz_VexRiscv_144_;
    _zz_VexRiscv_145_[18] = _zz_VexRiscv_144_;
    _zz_VexRiscv_145_[17] = _zz_VexRiscv_144_;
    _zz_VexRiscv_145_[16] = _zz_VexRiscv_144_;
    _zz_VexRiscv_145_[15] = _zz_VexRiscv_144_;
    _zz_VexRiscv_145_[14] = _zz_VexRiscv_144_;
    _zz_VexRiscv_145_[13] = _zz_VexRiscv_144_;
    _zz_VexRiscv_145_[12] = _zz_VexRiscv_144_;
    _zz_VexRiscv_145_[11] = _zz_VexRiscv_144_;
    _zz_VexRiscv_145_[10] = _zz_VexRiscv_144_;
    _zz_VexRiscv_145_[9] = _zz_VexRiscv_144_;
    _zz_VexRiscv_145_[8] = _zz_VexRiscv_144_;
    _zz_VexRiscv_145_[7] = _zz_VexRiscv_144_;
    _zz_VexRiscv_145_[6] = _zz_VexRiscv_144_;
    _zz_VexRiscv_145_[5] = _zz_VexRiscv_144_;
    _zz_VexRiscv_145_[4] = _zz_VexRiscv_144_;
    _zz_VexRiscv_145_[3] = _zz_VexRiscv_144_;
    _zz_VexRiscv_145_[2] = _zz_VexRiscv_144_;
    _zz_VexRiscv_145_[1] = _zz_VexRiscv_144_;
    _zz_VexRiscv_145_[0] = _zz_VexRiscv_144_;
  end

  assign _zz_VexRiscv_146_ = _zz_VexRiscv_205_[11];
  always @ (*) begin
    _zz_VexRiscv_147_[18] = _zz_VexRiscv_146_;
    _zz_VexRiscv_147_[17] = _zz_VexRiscv_146_;
    _zz_VexRiscv_147_[16] = _zz_VexRiscv_146_;
    _zz_VexRiscv_147_[15] = _zz_VexRiscv_146_;
    _zz_VexRiscv_147_[14] = _zz_VexRiscv_146_;
    _zz_VexRiscv_147_[13] = _zz_VexRiscv_146_;
    _zz_VexRiscv_147_[12] = _zz_VexRiscv_146_;
    _zz_VexRiscv_147_[11] = _zz_VexRiscv_146_;
    _zz_VexRiscv_147_[10] = _zz_VexRiscv_146_;
    _zz_VexRiscv_147_[9] = _zz_VexRiscv_146_;
    _zz_VexRiscv_147_[8] = _zz_VexRiscv_146_;
    _zz_VexRiscv_147_[7] = _zz_VexRiscv_146_;
    _zz_VexRiscv_147_[6] = _zz_VexRiscv_146_;
    _zz_VexRiscv_147_[5] = _zz_VexRiscv_146_;
    _zz_VexRiscv_147_[4] = _zz_VexRiscv_146_;
    _zz_VexRiscv_147_[3] = _zz_VexRiscv_146_;
    _zz_VexRiscv_147_[2] = _zz_VexRiscv_146_;
    _zz_VexRiscv_147_[1] = _zz_VexRiscv_146_;
    _zz_VexRiscv_147_[0] = _zz_VexRiscv_146_;
  end

  always @ (*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_JAL : begin
        _zz_VexRiscv_148_ = {{_zz_VexRiscv_143_,{{{execute_INSTRUCTION[31],execute_INSTRUCTION[19 : 12]},execute_INSTRUCTION[20]},execute_INSTRUCTION[30 : 21]}},1'b0};
      end
      `BranchCtrlEnum_defaultEncoding_JALR : begin
        _zz_VexRiscv_148_ = {_zz_VexRiscv_145_,execute_INSTRUCTION[31 : 20]};
      end
      default : begin
        _zz_VexRiscv_148_ = {{_zz_VexRiscv_147_,{{{execute_INSTRUCTION[31],execute_INSTRUCTION[7]},execute_INSTRUCTION[30 : 25]},execute_INSTRUCTION[11 : 8]}},1'b0};
      end
    endcase
  end

  assign execute_BranchPlugin_branch_src2 = _zz_VexRiscv_148_;
  assign execute_BranchPlugin_branchAdder = (execute_BranchPlugin_branch_src1 + execute_BranchPlugin_branch_src2);
  assign _zz_VexRiscv_21_ = {execute_BranchPlugin_branchAdder[31 : 1],(1'b0)};
  assign _zz_VexRiscv_80_ = ((memory_arbitration_isValid && (! memory_arbitration_isStuckByOthers)) && memory_BRANCH_DO);
  assign _zz_VexRiscv_81_ = memory_BRANCH_CALC;
  assign DebugPlugin_isPipBusy = (DebugPlugin_isPipActive || DebugPlugin_isPipActive_regNext);
  always @ (*) begin
    debug_bus_cmd_ready = 1'b1;
    _zz_VexRiscv_82_ = 1'b0;
    if(debug_bus_cmd_valid)begin
      case(_zz_VexRiscv_163_)
        6'b000000 : begin
        end
        6'b000001 : begin
          if(debug_bus_cmd_payload_wr)begin
            _zz_VexRiscv_82_ = 1'b1;
            debug_bus_cmd_ready = _zz_VexRiscv_83_;
          end
        end
        default : begin
        end
      endcase
    end
  end

  always @ (*) begin
    debug_bus_rsp_data = DebugPlugin_busReadDataReg;
    if((! _zz_VexRiscv_149_))begin
      debug_bus_rsp_data[0] = DebugPlugin_resetIt;
      debug_bus_rsp_data[1] = DebugPlugin_haltIt;
      debug_bus_rsp_data[2] = DebugPlugin_isPipBusy;
      debug_bus_rsp_data[3] = DebugPlugin_haltedByBreak;
      debug_bus_rsp_data[4] = DebugPlugin_stepIt;
    end
  end

  assign _zz_VexRiscv_20_ = ((! DebugPlugin_haltIt) && (decode_IS_EBREAK || 1'b0));
  assign debug_resetOut = DebugPlugin_resetIt_regNext;
  assign _zz_VexRiscv_19_ = decode_ALU_CTRL;
  assign _zz_VexRiscv_17_ = _zz_VexRiscv_46_;
  assign _zz_VexRiscv_35_ = decode_to_execute_ALU_CTRL;
  assign _zz_VexRiscv_33_ = _zz_VexRiscv_54_;
  assign _zz_VexRiscv_16_ = decode_ALU_BITWISE_CTRL;
  assign _zz_VexRiscv_14_ = _zz_VexRiscv_51_;
  assign _zz_VexRiscv_37_ = decode_to_execute_ALU_BITWISE_CTRL;
  assign _zz_VexRiscv_13_ = decode_SHIFT_CTRL;
  assign _zz_VexRiscv_11_ = _zz_VexRiscv_43_;
  assign _zz_VexRiscv_24_ = decode_to_execute_SHIFT_CTRL;
  assign _zz_VexRiscv_10_ = decode_BRANCH_CTRL;
  assign _zz_VexRiscv_8_ = _zz_VexRiscv_50_;
  assign _zz_VexRiscv_22_ = decode_to_execute_BRANCH_CTRL;
  assign _zz_VexRiscv_7_ = decode_ENV_CTRL;
  assign _zz_VexRiscv_4_ = execute_ENV_CTRL;
  assign _zz_VexRiscv_2_ = memory_ENV_CTRL;
  assign _zz_VexRiscv_5_ = _zz_VexRiscv_59_;
  assign _zz_VexRiscv_62_ = decode_to_execute_ENV_CTRL;
  assign _zz_VexRiscv_61_ = execute_to_memory_ENV_CTRL;
  assign _zz_VexRiscv_65_ = memory_to_writeBack_ENV_CTRL;
  assign _zz_VexRiscv_30_ = _zz_VexRiscv_52_;
  assign decode_arbitration_isFlushed = ({writeBack_arbitration_flushAll,{memory_arbitration_flushAll,{execute_arbitration_flushAll,decode_arbitration_flushAll}}} != (4'b0000));
  assign execute_arbitration_isFlushed = ({writeBack_arbitration_flushAll,{memory_arbitration_flushAll,execute_arbitration_flushAll}} != (3'b000));
  assign memory_arbitration_isFlushed = ({writeBack_arbitration_flushAll,memory_arbitration_flushAll} != (2'b00));
  assign writeBack_arbitration_isFlushed = (writeBack_arbitration_flushAll != (1'b0));
  assign decode_arbitration_isStuckByOthers = (decode_arbitration_haltByOther || (((1'b0 || execute_arbitration_isStuck) || memory_arbitration_isStuck) || writeBack_arbitration_isStuck));
  assign decode_arbitration_isStuck = (decode_arbitration_haltItself || decode_arbitration_isStuckByOthers);
  assign decode_arbitration_isMoving = ((! decode_arbitration_isStuck) && (! decode_arbitration_removeIt));
  assign decode_arbitration_isFiring = ((decode_arbitration_isValid && (! decode_arbitration_isStuck)) && (! decode_arbitration_removeIt));
  assign execute_arbitration_isStuckByOthers = (execute_arbitration_haltByOther || ((1'b0 || memory_arbitration_isStuck) || writeBack_arbitration_isStuck));
  assign execute_arbitration_isStuck = (execute_arbitration_haltItself || execute_arbitration_isStuckByOthers);
  assign execute_arbitration_isMoving = ((! execute_arbitration_isStuck) && (! execute_arbitration_removeIt));
  assign execute_arbitration_isFiring = ((execute_arbitration_isValid && (! execute_arbitration_isStuck)) && (! execute_arbitration_removeIt));
  assign memory_arbitration_isStuckByOthers = (memory_arbitration_haltByOther || (1'b0 || writeBack_arbitration_isStuck));
  assign memory_arbitration_isStuck = (memory_arbitration_haltItself || memory_arbitration_isStuckByOthers);
  assign memory_arbitration_isMoving = ((! memory_arbitration_isStuck) && (! memory_arbitration_removeIt));
  assign memory_arbitration_isFiring = ((memory_arbitration_isValid && (! memory_arbitration_isStuck)) && (! memory_arbitration_removeIt));
  assign writeBack_arbitration_isStuckByOthers = (writeBack_arbitration_haltByOther || 1'b0);
  assign writeBack_arbitration_isStuck = (writeBack_arbitration_haltItself || writeBack_arbitration_isStuckByOthers);
  assign writeBack_arbitration_isMoving = ((! writeBack_arbitration_isStuck) && (! writeBack_arbitration_removeIt));
  assign writeBack_arbitration_isFiring = ((writeBack_arbitration_isValid && (! writeBack_arbitration_isStuck)) && (! writeBack_arbitration_removeIt));
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      CsrPlugin_privilege <= (2'b11);
      IBusSimplePlugin_fetchPc_pcReg <= (32'b00000000000000000000000000000000);
      IBusSimplePlugin_fetchPc_inc <= 1'b0;
      _zz_VexRiscv_86_ <= 1'b0;
      _zz_VexRiscv_92_ <= 1'b0;
      _zz_VexRiscv_94_ <= 1'b0;
      _zz_VexRiscv_96_ <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_0 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_2 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_3 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_4 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_5 <= 1'b0;
      IBusSimplePlugin_injector_decodeRemoved <= 1'b0;
      IBusSimplePlugin_pendingCmd <= (3'b000);
      IBusSimplePlugin_rspJoin_discardCounter <= (3'b000);
      CsrPlugin_mstatus_MIE <= 1'b0;
      CsrPlugin_mstatus_MPIE <= 1'b0;
      CsrPlugin_mstatus_MPP <= (2'b11);
      CsrPlugin_mip_MEIP <= 1'b0;
      CsrPlugin_mip_MTIP <= 1'b0;
      CsrPlugin_mip_MSIP <= 1'b0;
      CsrPlugin_mie_MEIE <= 1'b0;
      CsrPlugin_mie_MTIE <= 1'b0;
      CsrPlugin_mie_MSIE <= 1'b0;
      CsrPlugin_hadException <= 1'b0;
      _zz_VexRiscv_125_ <= 1'b1;
      execute_LightShifterPlugin_isActive <= 1'b0;
      _zz_VexRiscv_137_ <= 1'b0;
      execute_arbitration_isValid <= 1'b0;
      memory_arbitration_isValid <= 1'b0;
      writeBack_arbitration_isValid <= 1'b0;
      _zz_VexRiscv_150_ <= (3'b000);
      memory_to_writeBack_REGFILE_WRITE_DATA <= (32'b00000000000000000000000000000000);
      memory_to_writeBack_INSTRUCTION <= (32'b00000000000000000000000000000000);
    end else begin
      if(IBusSimplePlugin_fetchPc_propagatePc)begin
        IBusSimplePlugin_fetchPc_inc <= 1'b0;
      end
      if(IBusSimplePlugin_jump_pcLoad_valid)begin
        IBusSimplePlugin_fetchPc_inc <= 1'b0;
      end
      if(_zz_VexRiscv_162_)begin
        IBusSimplePlugin_fetchPc_inc <= 1'b1;
      end
      if(IBusSimplePlugin_fetchPc_samplePcNext)begin
        IBusSimplePlugin_fetchPc_pcReg <= IBusSimplePlugin_fetchPc_pc;
      end
      _zz_VexRiscv_86_ <= 1'b1;
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_VexRiscv_75_))begin
        _zz_VexRiscv_92_ <= 1'b0;
      end
      if(_zz_VexRiscv_90_)begin
        _zz_VexRiscv_92_ <= IBusSimplePlugin_iBusRsp_stages_0_output_valid;
      end
      if(IBusSimplePlugin_iBusRsp_stages_1_output_ready)begin
        _zz_VexRiscv_94_ <= IBusSimplePlugin_iBusRsp_stages_1_output_valid;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_VexRiscv_75_))begin
        _zz_VexRiscv_94_ <= 1'b0;
      end
      if(IBusSimplePlugin_iBusRsp_inputBeforeStage_ready)begin
        _zz_VexRiscv_96_ <= IBusSimplePlugin_iBusRsp_inputBeforeStage_valid;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_VexRiscv_75_))begin
        _zz_VexRiscv_96_ <= 1'b0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_VexRiscv_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_0 <= 1'b0;
      end
      if((! (! IBusSimplePlugin_iBusRsp_stages_1_input_ready)))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_0 <= 1'b1;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_VexRiscv_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      end
      if((! (! IBusSimplePlugin_iBusRsp_stages_2_input_ready)))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= IBusSimplePlugin_injector_nextPcCalc_valids_0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_VexRiscv_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_VexRiscv_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_2 <= 1'b0;
      end
      if((! (! IBusSimplePlugin_injector_decodeInput_ready)))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_2 <= IBusSimplePlugin_injector_nextPcCalc_valids_1;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_VexRiscv_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_2 <= 1'b0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_VexRiscv_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_3 <= 1'b0;
      end
      if((! execute_arbitration_isStuck))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_3 <= IBusSimplePlugin_injector_nextPcCalc_valids_2;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_VexRiscv_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_3 <= 1'b0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_VexRiscv_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_4 <= 1'b0;
      end
      if((! memory_arbitration_isStuck))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_4 <= IBusSimplePlugin_injector_nextPcCalc_valids_3;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_VexRiscv_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_4 <= 1'b0;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_VexRiscv_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_5 <= 1'b0;
      end
      if((! writeBack_arbitration_isStuck))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_5 <= IBusSimplePlugin_injector_nextPcCalc_valids_4;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_VexRiscv_75_))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_5 <= 1'b0;
      end
      if(decode_arbitration_removeIt)begin
        IBusSimplePlugin_injector_decodeRemoved <= 1'b1;
      end
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_VexRiscv_75_))begin
        IBusSimplePlugin_injector_decodeRemoved <= 1'b0;
      end
      IBusSimplePlugin_pendingCmd <= IBusSimplePlugin_pendingCmdNext;
      IBusSimplePlugin_rspJoin_discardCounter <= (IBusSimplePlugin_rspJoin_discardCounter - _zz_VexRiscv_177_);
      if((IBusSimplePlugin_jump_pcLoad_valid || _zz_VexRiscv_75_))begin
        IBusSimplePlugin_rspJoin_discardCounter <= IBusSimplePlugin_pendingCmdNext;
      end
      CsrPlugin_mip_MEIP <= externalInterrupt;
      CsrPlugin_mip_MTIP <= timerInterrupt;
      CsrPlugin_hadException <= CsrPlugin_exception;
      if(_zz_VexRiscv_160_)begin
        CsrPlugin_privilege <= CsrPlugin_targetPrivilege;
        case(CsrPlugin_targetPrivilege)
          2'b11 : begin
            CsrPlugin_mstatus_MIE <= 1'b0;
            CsrPlugin_mstatus_MPIE <= CsrPlugin_mstatus_MIE;
            CsrPlugin_mstatus_MPP <= CsrPlugin_privilege;
          end
          default : begin
          end
        endcase
      end
      if(_zz_VexRiscv_161_)begin
        case(_zz_VexRiscv_165_)
          2'b11 : begin
            CsrPlugin_mstatus_MIE <= CsrPlugin_mstatus_MPIE;
            CsrPlugin_mstatus_MPP <= (2'b00);
            CsrPlugin_mstatus_MPIE <= 1'b1;
            CsrPlugin_privilege <= CsrPlugin_mstatus_MPP;
          end
          default : begin
          end
        endcase
      end
      _zz_VexRiscv_125_ <= 1'b0;
      if(_zz_VexRiscv_155_)begin
        if(_zz_VexRiscv_156_)begin
          execute_LightShifterPlugin_isActive <= 1'b1;
          if(execute_LightShifterPlugin_done)begin
            execute_LightShifterPlugin_isActive <= 1'b0;
          end
        end
      end
      if(execute_arbitration_removeIt)begin
        execute_LightShifterPlugin_isActive <= 1'b0;
      end
      _zz_VexRiscv_137_ <= _zz_VexRiscv_136_;
      if((! writeBack_arbitration_isStuck))begin
        memory_to_writeBack_INSTRUCTION <= memory_INSTRUCTION;
      end
      if((! writeBack_arbitration_isStuck))begin
        memory_to_writeBack_REGFILE_WRITE_DATA <= memory_REGFILE_WRITE_DATA;
      end
      if(((! execute_arbitration_isStuck) || execute_arbitration_removeIt))begin
        execute_arbitration_isValid <= 1'b0;
      end
      if(((! decode_arbitration_isStuck) && (! decode_arbitration_removeIt)))begin
        execute_arbitration_isValid <= decode_arbitration_isValid;
      end
      if(((! memory_arbitration_isStuck) || memory_arbitration_removeIt))begin
        memory_arbitration_isValid <= 1'b0;
      end
      if(((! execute_arbitration_isStuck) && (! execute_arbitration_removeIt)))begin
        memory_arbitration_isValid <= execute_arbitration_isValid;
      end
      if(((! writeBack_arbitration_isStuck) || writeBack_arbitration_removeIt))begin
        writeBack_arbitration_isValid <= 1'b0;
      end
      if(((! memory_arbitration_isStuck) && (! memory_arbitration_removeIt)))begin
        writeBack_arbitration_isValid <= memory_arbitration_isValid;
      end
      case(_zz_VexRiscv_150_)
        3'b000 : begin
          if(_zz_VexRiscv_82_)begin
            _zz_VexRiscv_150_ <= (3'b001);
          end
        end
        3'b001 : begin
          _zz_VexRiscv_150_ <= (3'b010);
        end
        3'b010 : begin
          _zz_VexRiscv_150_ <= (3'b011);
        end
        3'b011 : begin
          if((! decode_arbitration_isStuck))begin
            _zz_VexRiscv_150_ <= (3'b100);
          end
        end
        3'b100 : begin
          _zz_VexRiscv_150_ <= (3'b000);
        end
        default : begin
        end
      endcase
      case(execute_CsrPlugin_csrAddress)
        12'b001100000000 : begin
          if(execute_CsrPlugin_writeEnable)begin
            CsrPlugin_mstatus_MPP <= execute_CsrPlugin_writeData[12 : 11];
            CsrPlugin_mstatus_MPIE <= _zz_VexRiscv_206_[0];
            CsrPlugin_mstatus_MIE <= _zz_VexRiscv_207_[0];
          end
        end
        12'b001101000100 : begin
          if(execute_CsrPlugin_writeEnable)begin
            CsrPlugin_mip_MSIP <= _zz_VexRiscv_208_[0];
          end
        end
        12'b001100000100 : begin
          if(execute_CsrPlugin_writeEnable)begin
            CsrPlugin_mie_MEIE <= _zz_VexRiscv_209_[0];
            CsrPlugin_mie_MTIE <= _zz_VexRiscv_210_[0];
            CsrPlugin_mie_MSIE <= _zz_VexRiscv_211_[0];
          end
        end
        12'b001101000010 : begin
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge toplevel_main_clk) begin
    if(IBusSimplePlugin_iBusRsp_stages_1_output_ready)begin
      _zz_VexRiscv_95_ <= IBusSimplePlugin_iBusRsp_stages_1_output_payload;
    end
    if(IBusSimplePlugin_iBusRsp_inputBeforeStage_ready)begin
      _zz_VexRiscv_97_ <= IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_pc;
      _zz_VexRiscv_98_ <= IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_error;
      _zz_VexRiscv_99_ <= IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_inst;
      _zz_VexRiscv_100_ <= IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_isRvc;
    end
    if(IBusSimplePlugin_injector_decodeInput_ready)begin
      IBusSimplePlugin_injector_formal_rawInDecode <= IBusSimplePlugin_iBusRsp_inputBeforeStage_payload_rsp_inst;
    end
    if(!(! (((dBus_rsp_ready && memory_MEMORY_ENABLE) && memory_arbitration_isValid) && memory_arbitration_isStuck))) begin
      $display("ERROR DBusSimplePlugin doesn't allow memory stage stall when read happend");
    end
    if(!(! (((writeBack_arbitration_isValid && writeBack_MEMORY_ENABLE) && (! writeBack_INSTRUCTION[5])) && writeBack_arbitration_isStuck))) begin
      $display("ERROR DBusSimplePlugin doesn't allow writeback stage stall when read happend");
    end
    CsrPlugin_mcycle <= (CsrPlugin_mcycle + (64'b0000000000000000000000000000000000000000000000000000000000000001));
    if(writeBack_arbitration_isFiring)begin
      CsrPlugin_minstret <= (CsrPlugin_minstret + (64'b0000000000000000000000000000000000000000000000000000000000000001));
    end
    if((CsrPlugin_exception || CsrPlugin_interruptJump))begin
      case(CsrPlugin_privilege)
        2'b11 : begin
          CsrPlugin_mepc <= decode_PC;
        end
        default : begin
        end
      endcase
    end
    if(_zz_VexRiscv_160_)begin
      case(CsrPlugin_targetPrivilege)
        2'b11 : begin
          CsrPlugin_mcause_interrupt <= (! CsrPlugin_hadException);
          CsrPlugin_mcause_exceptionCode <= CsrPlugin_trapCause;
        end
        default : begin
        end
      endcase
    end
    if(_zz_VexRiscv_155_)begin
      if(_zz_VexRiscv_156_)begin
        execute_LightShifterPlugin_amplitudeReg <= (execute_LightShifterPlugin_amplitude - (5'b00001));
      end
    end
    if(_zz_VexRiscv_136_)begin
      _zz_VexRiscv_138_ <= _zz_VexRiscv_38_[11 : 7];
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ALU_CTRL <= _zz_VexRiscv_18_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_INSTRUCTION <= decode_INSTRUCTION;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_INSTRUCTION <= execute_INSTRUCTION;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ALU_BITWISE_CTRL <= _zz_VexRiscv_15_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC_USE_SUB_LESS <= decode_SRC_USE_SUB_LESS;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_MEMORY_ENABLE <= decode_MEMORY_ENABLE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_ENABLE <= execute_MEMORY_ENABLE;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_ENABLE <= memory_MEMORY_ENABLE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SHIFT_CTRL <= _zz_VexRiscv_12_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC2 <= decode_SRC2;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BRANCH_CTRL <= _zz_VexRiscv_9_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC_LESS_UNSIGNED <= decode_SRC_LESS_UNSIGNED;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ENV_CTRL <= _zz_VexRiscv_6_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_ENV_CTRL <= _zz_VexRiscv_3_;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_ENV_CTRL <= _zz_VexRiscv_1_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BRANCH_DO <= execute_BRANCH_DO;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_READ_DATA <= memory_MEMORY_READ_DATA;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BRANCH_CALC <= execute_BRANCH_CALC;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_RS1 <= _zz_VexRiscv_32_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BYPASSABLE_MEMORY_STAGE <= decode_BYPASSABLE_MEMORY_STAGE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BYPASSABLE_MEMORY_STAGE <= execute_BYPASSABLE_MEMORY_STAGE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC1 <= decode_SRC1;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_DO_EBREAK <= decode_DO_EBREAK;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_RS2 <= _zz_VexRiscv_29_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_REGFILE_WRITE_DATA <= _zz_VexRiscv_60_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_PC <= _zz_VexRiscv_28_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_PC <= execute_PC;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_PC <= memory_PC;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_REGFILE_WRITE_VALID <= decode_REGFILE_WRITE_VALID;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_REGFILE_WRITE_VALID <= execute_REGFILE_WRITE_VALID;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_REGFILE_WRITE_VALID <= memory_REGFILE_WRITE_VALID;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_ADDRESS_LOW <= execute_MEMORY_ADDRESS_LOW;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_ADDRESS_LOW <= memory_MEMORY_ADDRESS_LOW;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BYPASSABLE_EXECUTE_STAGE <= decode_BYPASSABLE_EXECUTE_STAGE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_FORMAL_PC_NEXT <= decode_FORMAL_PC_NEXT;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_FORMAL_PC_NEXT <= execute_FORMAL_PC_NEXT;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_FORMAL_PC_NEXT <= _zz_VexRiscv_69_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_CSR <= decode_IS_CSR;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_CSR_WRITE_OPCODE <= decode_CSR_WRITE_OPCODE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_CSR_READ_OPCODE <= decode_CSR_READ_OPCODE;
    end
    if((_zz_VexRiscv_150_ != (3'b000)))begin
      _zz_VexRiscv_99_ <= debug_bus_cmd_payload_data;
    end
  end

  always @ (posedge toplevel_main_clk) begin
    DebugPlugin_firstCycle <= 1'b0;
    if(debug_bus_cmd_ready)begin
      DebugPlugin_firstCycle <= 1'b1;
    end
    DebugPlugin_secondCycle <= DebugPlugin_firstCycle;
    DebugPlugin_isPipActive <= ({writeBack_arbitration_isValid,{memory_arbitration_isValid,{execute_arbitration_isValid,decode_arbitration_isValid}}} != (4'b0000));
    DebugPlugin_isPipActive_regNext <= DebugPlugin_isPipActive;
    if(writeBack_arbitration_isValid)begin
      DebugPlugin_busReadDataReg <= _zz_VexRiscv_66_;
    end
    _zz_VexRiscv_149_ <= debug_bus_cmd_payload_address[2];
    if(_zz_VexRiscv_157_)begin
      DebugPlugin_busReadDataReg <= execute_PC;
    end
    DebugPlugin_resetIt_regNext <= DebugPlugin_resetIt;
  end

  always @ (posedge toplevel_main_clk) begin
    if(!_zz_VexRiscv_151_) begin
      DebugPlugin_resetIt <= 1'b0;
      DebugPlugin_haltIt <= 1'b0;
      DebugPlugin_stepIt <= 1'b0;
      DebugPlugin_haltedByBreak <= 1'b0;
    end else begin
      if(debug_bus_cmd_valid)begin
        case(_zz_VexRiscv_163_)
          6'b000000 : begin
            if(debug_bus_cmd_payload_wr)begin
              DebugPlugin_stepIt <= debug_bus_cmd_payload_data[4];
              if(debug_bus_cmd_payload_data[16])begin
                DebugPlugin_resetIt <= 1'b1;
              end
              if(debug_bus_cmd_payload_data[24])begin
                DebugPlugin_resetIt <= 1'b0;
              end
              if(debug_bus_cmd_payload_data[17])begin
                DebugPlugin_haltIt <= 1'b1;
              end
              if(debug_bus_cmd_payload_data[25])begin
                DebugPlugin_haltIt <= 1'b0;
              end
              if(debug_bus_cmd_payload_data[25])begin
                DebugPlugin_haltedByBreak <= 1'b0;
              end
            end
          end
          6'b000001 : begin
          end
          default : begin
          end
        endcase
      end
      if(_zz_VexRiscv_157_)begin
        if(_zz_VexRiscv_158_)begin
          DebugPlugin_haltIt <= 1'b1;
          DebugPlugin_haltedByBreak <= 1'b1;
        end
      end
      if(_zz_VexRiscv_159_)begin
        if(decode_arbitration_isValid)begin
          DebugPlugin_haltIt <= 1'b1;
        end
      end
      if((DebugPlugin_stepIt && ({writeBack_arbitration_redoIt,{memory_arbitration_redoIt,{execute_arbitration_redoIt,decode_arbitration_redoIt}}} != (4'b0000))))begin
        DebugPlugin_haltIt <= 1'b0;
      end
    end
  end

endmodule

module StreamFork_4_ (
      input   io_input_valid,
      output reg  io_input_ready,
      input   io_input_payload_wr,
      input  [31:0] io_input_payload_address,
      input  [31:0] io_input_payload_data,
      input  [1:0] io_input_payload_size,
      output  io_outputs_0_valid,
      input   io_outputs_0_ready,
      output  io_outputs_0_payload_wr,
      output [31:0] io_outputs_0_payload_address,
      output [31:0] io_outputs_0_payload_data,
      output [1:0] io_outputs_0_payload_size,
      output  io_outputs_1_valid,
      input   io_outputs_1_ready,
      output  io_outputs_1_payload_wr,
      output [31:0] io_outputs_1_payload_address,
      output [31:0] io_outputs_1_payload_data,
      output [1:0] io_outputs_1_payload_size,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg  linkEnable_0;
  reg  linkEnable_1;
  always @ (*) begin
    io_input_ready = 1'b1;
    if(((! io_outputs_0_ready) && linkEnable_0))begin
      io_input_ready = 1'b0;
    end
    if(((! io_outputs_1_ready) && linkEnable_1))begin
      io_input_ready = 1'b0;
    end
  end

  assign io_outputs_0_valid = (io_input_valid && linkEnable_0);
  assign io_outputs_0_payload_wr = io_input_payload_wr;
  assign io_outputs_0_payload_address = io_input_payload_address;
  assign io_outputs_0_payload_data = io_input_payload_data;
  assign io_outputs_0_payload_size = io_input_payload_size;
  assign io_outputs_1_valid = (io_input_valid && linkEnable_1);
  assign io_outputs_1_payload_wr = io_input_payload_wr;
  assign io_outputs_1_payload_address = io_input_payload_address;
  assign io_outputs_1_payload_data = io_input_payload_data;
  assign io_outputs_1_payload_size = io_input_payload_size;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      linkEnable_0 <= 1'b1;
      linkEnable_1 <= 1'b1;
    end else begin
      if((io_outputs_0_valid && io_outputs_0_ready))begin
        linkEnable_0 <= 1'b0;
      end
      if((io_outputs_1_valid && io_outputs_1_ready))begin
        linkEnable_1 <= 1'b0;
      end
      if(io_input_ready)begin
        linkEnable_0 <= 1'b1;
        linkEnable_1 <= 1'b1;
      end
    end
  end

endmodule

module JtagBridge (
      input   io_jtag_tms,
      input   io_jtag_tdi,
      output reg  io_jtag_tdo,
      input   io_jtag_tck,
      output  io_remote_cmd_valid,
      input   io_remote_cmd_ready,
      output  io_remote_cmd_payload_last,
      output [0:0] io_remote_cmd_payload_fragment,
      input   io_remote_rsp_valid,
      output  io_remote_rsp_ready,
      input   io_remote_rsp_payload_error,
      input  [31:0] io_remote_rsp_payload_data,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire  flowCCByToggle_1__io_output_valid;
  wire  flowCCByToggle_1__io_output_payload_last;
  wire [0:0] flowCCByToggle_1__io_output_payload_fragment;
  wire  _zz_JtagBridge_2_;
  wire  _zz_JtagBridge_3_;
  wire [3:0] _zz_JtagBridge_4_;
  wire [3:0] _zz_JtagBridge_5_;
  wire [3:0] _zz_JtagBridge_6_;
  wire  system_cmd_valid;
  wire  system_cmd_payload_last;
  wire [0:0] system_cmd_payload_fragment;
  reg  system_rsp_valid;
  reg  system_rsp_payload_error;
  reg [31:0] system_rsp_payload_data;
  wire `JtagState_defaultEncoding_type jtag_tap_fsm_stateNext;
  reg `JtagState_defaultEncoding_type jtag_tap_fsm_state = `JtagState_defaultEncoding_RESET;
  reg `JtagState_defaultEncoding_type _zz_JtagBridge_1_;
  reg [3:0] jtag_tap_instruction;
  reg [3:0] jtag_tap_instructionShift;
  reg  jtag_tap_bypass;
  wire [0:0] jtag_idcodeArea_instructionId;
  wire  jtag_idcodeArea_instructionHit;
  reg [31:0] jtag_idcodeArea_shifter;
  wire [1:0] jtag_writeArea_instructionId;
  wire  jtag_writeArea_instructionHit;
  reg  jtag_writeArea_source_valid;
  wire  jtag_writeArea_source_payload_last;
  wire [0:0] jtag_writeArea_source_payload_fragment;
  wire [1:0] jtag_readArea_instructionId;
  wire  jtag_readArea_instructionHit;
  reg [33:0] jtag_readArea_shifter;
  `ifndef SYNTHESIS
  reg [79:0] jtag_tap_fsm_stateNext_string;
  reg [79:0] jtag_tap_fsm_state_string;
  reg [79:0] _zz_JtagBridge_1__string;
  `endif

  assign _zz_JtagBridge_2_ = (jtag_tap_fsm_state == `JtagState_defaultEncoding_DR_SHIFT);
  assign _zz_JtagBridge_3_ = (jtag_tap_fsm_state == `JtagState_defaultEncoding_DR_SHIFT);
  assign _zz_JtagBridge_4_ = {3'd0, jtag_idcodeArea_instructionId};
  assign _zz_JtagBridge_5_ = {2'd0, jtag_writeArea_instructionId};
  assign _zz_JtagBridge_6_ = {2'd0, jtag_readArea_instructionId};
  FlowCCByToggle flowCCByToggle_1_ ( 
    .io_input_valid(jtag_writeArea_source_valid),
    .io_input_payload_last(jtag_writeArea_source_payload_last),
    .io_input_payload_fragment(jtag_writeArea_source_payload_fragment),
    .io_output_valid(flowCCByToggle_1__io_output_valid),
    .io_output_payload_last(flowCCByToggle_1__io_output_payload_last),
    .io_output_payload_fragment(flowCCByToggle_1__io_output_payload_fragment),
    ._zz_FlowCCByToggle_1_(io_jtag_tck),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(jtag_tap_fsm_stateNext)
      `JtagState_defaultEncoding_RESET : jtag_tap_fsm_stateNext_string = "RESET     ";
      `JtagState_defaultEncoding_IDLE : jtag_tap_fsm_stateNext_string = "IDLE      ";
      `JtagState_defaultEncoding_IR_SELECT : jtag_tap_fsm_stateNext_string = "IR_SELECT ";
      `JtagState_defaultEncoding_IR_CAPTURE : jtag_tap_fsm_stateNext_string = "IR_CAPTURE";
      `JtagState_defaultEncoding_IR_SHIFT : jtag_tap_fsm_stateNext_string = "IR_SHIFT  ";
      `JtagState_defaultEncoding_IR_EXIT1 : jtag_tap_fsm_stateNext_string = "IR_EXIT1  ";
      `JtagState_defaultEncoding_IR_PAUSE : jtag_tap_fsm_stateNext_string = "IR_PAUSE  ";
      `JtagState_defaultEncoding_IR_EXIT2 : jtag_tap_fsm_stateNext_string = "IR_EXIT2  ";
      `JtagState_defaultEncoding_IR_UPDATE : jtag_tap_fsm_stateNext_string = "IR_UPDATE ";
      `JtagState_defaultEncoding_DR_SELECT : jtag_tap_fsm_stateNext_string = "DR_SELECT ";
      `JtagState_defaultEncoding_DR_CAPTURE : jtag_tap_fsm_stateNext_string = "DR_CAPTURE";
      `JtagState_defaultEncoding_DR_SHIFT : jtag_tap_fsm_stateNext_string = "DR_SHIFT  ";
      `JtagState_defaultEncoding_DR_EXIT1 : jtag_tap_fsm_stateNext_string = "DR_EXIT1  ";
      `JtagState_defaultEncoding_DR_PAUSE : jtag_tap_fsm_stateNext_string = "DR_PAUSE  ";
      `JtagState_defaultEncoding_DR_EXIT2 : jtag_tap_fsm_stateNext_string = "DR_EXIT2  ";
      `JtagState_defaultEncoding_DR_UPDATE : jtag_tap_fsm_stateNext_string = "DR_UPDATE ";
      default : jtag_tap_fsm_stateNext_string = "??????????";
    endcase
  end
  always @(*) begin
    case(jtag_tap_fsm_state)
      `JtagState_defaultEncoding_RESET : jtag_tap_fsm_state_string = "RESET     ";
      `JtagState_defaultEncoding_IDLE : jtag_tap_fsm_state_string = "IDLE      ";
      `JtagState_defaultEncoding_IR_SELECT : jtag_tap_fsm_state_string = "IR_SELECT ";
      `JtagState_defaultEncoding_IR_CAPTURE : jtag_tap_fsm_state_string = "IR_CAPTURE";
      `JtagState_defaultEncoding_IR_SHIFT : jtag_tap_fsm_state_string = "IR_SHIFT  ";
      `JtagState_defaultEncoding_IR_EXIT1 : jtag_tap_fsm_state_string = "IR_EXIT1  ";
      `JtagState_defaultEncoding_IR_PAUSE : jtag_tap_fsm_state_string = "IR_PAUSE  ";
      `JtagState_defaultEncoding_IR_EXIT2 : jtag_tap_fsm_state_string = "IR_EXIT2  ";
      `JtagState_defaultEncoding_IR_UPDATE : jtag_tap_fsm_state_string = "IR_UPDATE ";
      `JtagState_defaultEncoding_DR_SELECT : jtag_tap_fsm_state_string = "DR_SELECT ";
      `JtagState_defaultEncoding_DR_CAPTURE : jtag_tap_fsm_state_string = "DR_CAPTURE";
      `JtagState_defaultEncoding_DR_SHIFT : jtag_tap_fsm_state_string = "DR_SHIFT  ";
      `JtagState_defaultEncoding_DR_EXIT1 : jtag_tap_fsm_state_string = "DR_EXIT1  ";
      `JtagState_defaultEncoding_DR_PAUSE : jtag_tap_fsm_state_string = "DR_PAUSE  ";
      `JtagState_defaultEncoding_DR_EXIT2 : jtag_tap_fsm_state_string = "DR_EXIT2  ";
      `JtagState_defaultEncoding_DR_UPDATE : jtag_tap_fsm_state_string = "DR_UPDATE ";
      default : jtag_tap_fsm_state_string = "??????????";
    endcase
  end
  always @(*) begin
    case(_zz_JtagBridge_1_)
      `JtagState_defaultEncoding_RESET : _zz_JtagBridge_1__string = "RESET     ";
      `JtagState_defaultEncoding_IDLE : _zz_JtagBridge_1__string = "IDLE      ";
      `JtagState_defaultEncoding_IR_SELECT : _zz_JtagBridge_1__string = "IR_SELECT ";
      `JtagState_defaultEncoding_IR_CAPTURE : _zz_JtagBridge_1__string = "IR_CAPTURE";
      `JtagState_defaultEncoding_IR_SHIFT : _zz_JtagBridge_1__string = "IR_SHIFT  ";
      `JtagState_defaultEncoding_IR_EXIT1 : _zz_JtagBridge_1__string = "IR_EXIT1  ";
      `JtagState_defaultEncoding_IR_PAUSE : _zz_JtagBridge_1__string = "IR_PAUSE  ";
      `JtagState_defaultEncoding_IR_EXIT2 : _zz_JtagBridge_1__string = "IR_EXIT2  ";
      `JtagState_defaultEncoding_IR_UPDATE : _zz_JtagBridge_1__string = "IR_UPDATE ";
      `JtagState_defaultEncoding_DR_SELECT : _zz_JtagBridge_1__string = "DR_SELECT ";
      `JtagState_defaultEncoding_DR_CAPTURE : _zz_JtagBridge_1__string = "DR_CAPTURE";
      `JtagState_defaultEncoding_DR_SHIFT : _zz_JtagBridge_1__string = "DR_SHIFT  ";
      `JtagState_defaultEncoding_DR_EXIT1 : _zz_JtagBridge_1__string = "DR_EXIT1  ";
      `JtagState_defaultEncoding_DR_PAUSE : _zz_JtagBridge_1__string = "DR_PAUSE  ";
      `JtagState_defaultEncoding_DR_EXIT2 : _zz_JtagBridge_1__string = "DR_EXIT2  ";
      `JtagState_defaultEncoding_DR_UPDATE : _zz_JtagBridge_1__string = "DR_UPDATE ";
      default : _zz_JtagBridge_1__string = "??????????";
    endcase
  end
  `endif

  assign io_remote_cmd_valid = system_cmd_valid;
  assign io_remote_cmd_payload_last = system_cmd_payload_last;
  assign io_remote_cmd_payload_fragment = system_cmd_payload_fragment;
  assign io_remote_rsp_ready = 1'b1;
  always @ (*) begin
    case(jtag_tap_fsm_state)
      `JtagState_defaultEncoding_IDLE : begin
        _zz_JtagBridge_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_SELECT : `JtagState_defaultEncoding_IDLE);
      end
      `JtagState_defaultEncoding_IR_SELECT : begin
        _zz_JtagBridge_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_RESET : `JtagState_defaultEncoding_IR_CAPTURE);
      end
      `JtagState_defaultEncoding_IR_CAPTURE : begin
        _zz_JtagBridge_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_EXIT1 : `JtagState_defaultEncoding_IR_SHIFT);
      end
      `JtagState_defaultEncoding_IR_SHIFT : begin
        _zz_JtagBridge_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_EXIT1 : `JtagState_defaultEncoding_IR_SHIFT);
      end
      `JtagState_defaultEncoding_IR_EXIT1 : begin
        _zz_JtagBridge_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_UPDATE : `JtagState_defaultEncoding_IR_PAUSE);
      end
      `JtagState_defaultEncoding_IR_PAUSE : begin
        _zz_JtagBridge_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_EXIT2 : `JtagState_defaultEncoding_IR_PAUSE);
      end
      `JtagState_defaultEncoding_IR_EXIT2 : begin
        _zz_JtagBridge_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_UPDATE : `JtagState_defaultEncoding_IR_SHIFT);
      end
      `JtagState_defaultEncoding_IR_UPDATE : begin
        _zz_JtagBridge_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_SELECT : `JtagState_defaultEncoding_IDLE);
      end
      `JtagState_defaultEncoding_DR_SELECT : begin
        _zz_JtagBridge_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_SELECT : `JtagState_defaultEncoding_DR_CAPTURE);
      end
      `JtagState_defaultEncoding_DR_CAPTURE : begin
        _zz_JtagBridge_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_EXIT1 : `JtagState_defaultEncoding_DR_SHIFT);
      end
      `JtagState_defaultEncoding_DR_SHIFT : begin
        _zz_JtagBridge_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_EXIT1 : `JtagState_defaultEncoding_DR_SHIFT);
      end
      `JtagState_defaultEncoding_DR_EXIT1 : begin
        _zz_JtagBridge_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_UPDATE : `JtagState_defaultEncoding_DR_PAUSE);
      end
      `JtagState_defaultEncoding_DR_PAUSE : begin
        _zz_JtagBridge_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_EXIT2 : `JtagState_defaultEncoding_DR_PAUSE);
      end
      `JtagState_defaultEncoding_DR_EXIT2 : begin
        _zz_JtagBridge_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_UPDATE : `JtagState_defaultEncoding_DR_SHIFT);
      end
      `JtagState_defaultEncoding_DR_UPDATE : begin
        _zz_JtagBridge_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_SELECT : `JtagState_defaultEncoding_IDLE);
      end
      default : begin
        _zz_JtagBridge_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_RESET : `JtagState_defaultEncoding_IDLE);
      end
    endcase
  end

  assign jtag_tap_fsm_stateNext = _zz_JtagBridge_1_;
  always @ (*) begin
    io_jtag_tdo = jtag_tap_bypass;
    case(jtag_tap_fsm_state)
      `JtagState_defaultEncoding_IR_CAPTURE : begin
      end
      `JtagState_defaultEncoding_IR_SHIFT : begin
        io_jtag_tdo = jtag_tap_instructionShift[0];
      end
      `JtagState_defaultEncoding_IR_UPDATE : begin
      end
      `JtagState_defaultEncoding_DR_SHIFT : begin
      end
      default : begin
      end
    endcase
    if(jtag_idcodeArea_instructionHit)begin
      if(_zz_JtagBridge_2_)begin
        io_jtag_tdo = jtag_idcodeArea_shifter[0];
      end
    end
    if(jtag_readArea_instructionHit)begin
      if(_zz_JtagBridge_3_)begin
        io_jtag_tdo = jtag_readArea_shifter[0];
      end
    end
  end

  assign jtag_idcodeArea_instructionId = (1'b1);
  assign jtag_idcodeArea_instructionHit = (jtag_tap_instruction == _zz_JtagBridge_4_);
  assign jtag_writeArea_instructionId = (2'b10);
  assign jtag_writeArea_instructionHit = (jtag_tap_instruction == _zz_JtagBridge_5_);
  always @ (*) begin
    jtag_writeArea_source_valid = 1'b0;
    if(jtag_writeArea_instructionHit)begin
      if((jtag_tap_fsm_state == `JtagState_defaultEncoding_DR_SHIFT))begin
        jtag_writeArea_source_valid = 1'b1;
      end
    end
  end

  assign jtag_writeArea_source_payload_last = io_jtag_tms;
  assign jtag_writeArea_source_payload_fragment[0] = io_jtag_tdi;
  assign system_cmd_valid = flowCCByToggle_1__io_output_valid;
  assign system_cmd_payload_last = flowCCByToggle_1__io_output_payload_last;
  assign system_cmd_payload_fragment = flowCCByToggle_1__io_output_payload_fragment;
  assign jtag_readArea_instructionId = (2'b11);
  assign jtag_readArea_instructionHit = (jtag_tap_instruction == _zz_JtagBridge_6_);
  always @ (posedge toplevel_main_clk) begin
    if(io_remote_cmd_valid)begin
      system_rsp_valid <= 1'b0;
    end
    if((io_remote_rsp_valid && io_remote_rsp_ready))begin
      system_rsp_valid <= 1'b1;
      system_rsp_payload_error <= io_remote_rsp_payload_error;
      system_rsp_payload_data <= io_remote_rsp_payload_data;
    end
  end

  always @ (posedge io_jtag_tck) begin
    jtag_tap_fsm_state <= jtag_tap_fsm_stateNext;
    case(jtag_tap_fsm_state)
      `JtagState_defaultEncoding_IR_CAPTURE : begin
        jtag_tap_instructionShift <= jtag_tap_instruction;
      end
      `JtagState_defaultEncoding_IR_SHIFT : begin
        jtag_tap_instructionShift <= ({io_jtag_tdi,jtag_tap_instructionShift} >>> 1);
      end
      `JtagState_defaultEncoding_IR_UPDATE : begin
        jtag_tap_instruction <= jtag_tap_instructionShift;
      end
      `JtagState_defaultEncoding_DR_SHIFT : begin
        jtag_tap_bypass <= io_jtag_tdi;
      end
      default : begin
      end
    endcase
    if(jtag_idcodeArea_instructionHit)begin
      if(_zz_JtagBridge_2_)begin
        jtag_idcodeArea_shifter <= ({io_jtag_tdi,jtag_idcodeArea_shifter} >>> 1);
      end
    end
    if((jtag_tap_fsm_state == `JtagState_defaultEncoding_RESET))begin
      jtag_idcodeArea_shifter <= (32'b00010000000000000001111111111111);
      jtag_tap_instruction <= {3'd0, jtag_idcodeArea_instructionId};
    end
    if(jtag_readArea_instructionHit)begin
      if((jtag_tap_fsm_state == `JtagState_defaultEncoding_DR_CAPTURE))begin
        jtag_readArea_shifter <= {{system_rsp_payload_data,system_rsp_payload_error},system_rsp_valid};
      end
      if(_zz_JtagBridge_3_)begin
        jtag_readArea_shifter <= ({io_jtag_tdi,jtag_readArea_shifter} >>> 1);
      end
    end
  end

endmodule

module SystemDebugger (
      input   io_remote_cmd_valid,
      output  io_remote_cmd_ready,
      input   io_remote_cmd_payload_last,
      input  [0:0] io_remote_cmd_payload_fragment,
      output  io_remote_rsp_valid,
      input   io_remote_rsp_ready,
      output  io_remote_rsp_payload_error,
      output [31:0] io_remote_rsp_payload_data,
      output  io_mem_cmd_valid,
      input   io_mem_cmd_ready,
      output [31:0] io_mem_cmd_payload_address,
      output [31:0] io_mem_cmd_payload_data,
      output  io_mem_cmd_payload_wr,
      output [1:0] io_mem_cmd_payload_size,
      input   io_mem_rsp_valid,
      input  [31:0] io_mem_rsp_payload,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire  _zz_SystemDebugger_2_;
  wire [0:0] _zz_SystemDebugger_3_;
  reg [66:0] dispatcher_dataShifter;
  reg  dispatcher_dataLoaded;
  reg [7:0] dispatcher_headerShifter;
  wire [7:0] dispatcher_header;
  reg  dispatcher_headerLoaded;
  reg [2:0] dispatcher_counter;
  wire [66:0] _zz_SystemDebugger_1_;
  assign _zz_SystemDebugger_2_ = (dispatcher_headerLoaded == 1'b0);
  assign _zz_SystemDebugger_3_ = _zz_SystemDebugger_1_[64 : 64];
  assign dispatcher_header = dispatcher_headerShifter[7 : 0];
  assign io_remote_cmd_ready = (! dispatcher_dataLoaded);
  assign _zz_SystemDebugger_1_ = dispatcher_dataShifter[66 : 0];
  assign io_mem_cmd_payload_address = _zz_SystemDebugger_1_[31 : 0];
  assign io_mem_cmd_payload_data = _zz_SystemDebugger_1_[63 : 32];
  assign io_mem_cmd_payload_wr = _zz_SystemDebugger_3_[0];
  assign io_mem_cmd_payload_size = _zz_SystemDebugger_1_[66 : 65];
  assign io_mem_cmd_valid = (dispatcher_dataLoaded && (dispatcher_header == (8'b00000000)));
  assign io_remote_rsp_valid = io_mem_rsp_valid;
  assign io_remote_rsp_payload_error = 1'b0;
  assign io_remote_rsp_payload_data = io_mem_rsp_payload;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      dispatcher_dataLoaded <= 1'b0;
      dispatcher_headerLoaded <= 1'b0;
      dispatcher_counter <= (3'b000);
    end else begin
      if(io_remote_cmd_valid)begin
        if(_zz_SystemDebugger_2_)begin
          dispatcher_counter <= (dispatcher_counter + (3'b001));
          if((dispatcher_counter == (3'b111)))begin
            dispatcher_headerLoaded <= 1'b1;
          end
        end
        if(io_remote_cmd_payload_last)begin
          dispatcher_headerLoaded <= 1'b1;
          dispatcher_dataLoaded <= 1'b1;
          dispatcher_counter <= (3'b000);
        end
      end
      if((io_mem_cmd_valid && io_mem_cmd_ready))begin
        dispatcher_headerLoaded <= 1'b0;
        dispatcher_dataLoaded <= 1'b0;
      end
    end
  end

  always @ (posedge toplevel_main_clk) begin
    if(io_remote_cmd_valid)begin
      if(_zz_SystemDebugger_2_)begin
        dispatcher_headerShifter <= ({io_remote_cmd_payload_fragment,dispatcher_headerShifter} >>> 1);
      end else begin
        dispatcher_dataShifter <= ({io_remote_cmd_payload_fragment,dispatcher_dataShifter} >>> 1);
      end
    end
  end

endmodule

module Axi4SharedOnChipRam (
      input   io_axi_arw_valid,
      output reg  io_axi_arw_ready,
      input  [12:0] io_axi_arw_payload_addr,
      input  [3:0] io_axi_arw_payload_id,
      input  [7:0] io_axi_arw_payload_len,
      input  [2:0] io_axi_arw_payload_size,
      input  [1:0] io_axi_arw_payload_burst,
      input   io_axi_arw_payload_write,
      input   io_axi_w_valid,
      output  io_axi_w_ready,
      input  [31:0] io_axi_w_payload_data,
      input  [3:0] io_axi_w_payload_strb,
      input   io_axi_w_payload_last,
      output  io_axi_b_valid,
      input   io_axi_b_ready,
      output [3:0] io_axi_b_payload_id,
      output [1:0] io_axi_b_payload_resp,
      output  io_axi_r_valid,
      input   io_axi_r_ready,
      output [31:0] io_axi_r_payload_data,
      output [3:0] io_axi_r_payload_id,
      output [1:0] io_axi_r_payload_resp,
      output  io_axi_r_payload_last,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg [31:0] _zz_Axi4SharedOnChipRam_13_;
  reg [11:0] _zz_Axi4SharedOnChipRam_14_;
  wire  _zz_Axi4SharedOnChipRam_15_;
  wire [1:0] _zz_Axi4SharedOnChipRam_16_;
  wire [11:0] _zz_Axi4SharedOnChipRam_17_;
  wire [11:0] _zz_Axi4SharedOnChipRam_18_;
  wire [11:0] _zz_Axi4SharedOnChipRam_19_;
  wire [2:0] _zz_Axi4SharedOnChipRam_20_;
  wire [2:0] _zz_Axi4SharedOnChipRam_21_;
  reg  arw_valid;
  wire  arw_ready;
  reg  arw_payload_last;
  reg [12:0] arw_payload_fragment_addr;
  reg [3:0] arw_payload_fragment_id;
  reg [2:0] arw_payload_fragment_size;
  reg [1:0] arw_payload_fragment_burst;
  reg  arw_payload_fragment_write;
  wire  unburstify_doResult;
  reg  unburstify_buffer_valid;
  reg [7:0] unburstify_buffer_len;
  reg [7:0] unburstify_buffer_beat;
  reg [12:0] unburstify_buffer_transaction_addr;
  reg [3:0] unburstify_buffer_transaction_id;
  reg [2:0] unburstify_buffer_transaction_size;
  reg [1:0] unburstify_buffer_transaction_burst;
  reg  unburstify_buffer_transaction_write;
  wire  unburstify_buffer_last;
  wire [1:0] Axi4Incr_validSize;
  reg [12:0] unburstify_buffer_result;
  wire [0:0] Axi4Incr_highCat;
  wire [2:0] Axi4Incr_sizeValue;
  wire [11:0] Axi4Incr_alignMask;
  wire [11:0] Axi4Incr_base;
  wire [11:0] Axi4Incr_baseIncr;
  reg [1:0] _zz_Axi4SharedOnChipRam_1_;
  wire [2:0] Axi4Incr_wrapCase;
  wire  _zz_Axi4SharedOnChipRam_2_;
  wire  stage0_valid;
  wire  stage0_ready;
  wire  stage0_payload_last;
  wire [12:0] stage0_payload_fragment_addr;
  wire [3:0] stage0_payload_fragment_id;
  wire [2:0] stage0_payload_fragment_size;
  wire [1:0] stage0_payload_fragment_burst;
  wire  stage0_payload_fragment_write;
  wire [10:0] _zz_Axi4SharedOnChipRam_3_;
  wire  _zz_Axi4SharedOnChipRam_4_;
  wire [31:0] _zz_Axi4SharedOnChipRam_5_;
  wire  stage1_valid;
  wire  stage1_ready;
  wire  stage1_payload_last;
  wire [12:0] stage1_payload_fragment_addr;
  wire [3:0] stage1_payload_fragment_id;
  wire [2:0] stage1_payload_fragment_size;
  wire [1:0] stage1_payload_fragment_burst;
  wire  stage1_payload_fragment_write;
  reg  _zz_Axi4SharedOnChipRam_6_;
  reg  _zz_Axi4SharedOnChipRam_7_;
  reg [12:0] _zz_Axi4SharedOnChipRam_8_;
  reg [3:0] _zz_Axi4SharedOnChipRam_9_;
  reg [2:0] _zz_Axi4SharedOnChipRam_10_;
  reg [1:0] _zz_Axi4SharedOnChipRam_11_;
  reg  _zz_Axi4SharedOnChipRam_12_;
  reg [7:0] ram_symbol0 [0:2047];
  reg [7:0] ram_symbol1 [0:2047];
  reg [7:0] ram_symbol2 [0:2047];
  reg [7:0] ram_symbol3 [0:2047];
  reg [7:0] _zz_Axi4SharedOnChipRam_22_;
  reg [7:0] _zz_Axi4SharedOnChipRam_23_;
  reg [7:0] _zz_Axi4SharedOnChipRam_24_;
  reg [7:0] _zz_Axi4SharedOnChipRam_25_;
  assign _zz_Axi4SharedOnChipRam_15_ = (io_axi_arw_payload_len == (8'b00000000));
  assign _zz_Axi4SharedOnChipRam_16_ = {((2'b01) < Axi4Incr_validSize),((2'b00) < Axi4Incr_validSize)};
  assign _zz_Axi4SharedOnChipRam_17_ = unburstify_buffer_transaction_addr[11 : 0];
  assign _zz_Axi4SharedOnChipRam_18_ = _zz_Axi4SharedOnChipRam_17_;
  assign _zz_Axi4SharedOnChipRam_19_ = {9'd0, Axi4Incr_sizeValue};
  assign _zz_Axi4SharedOnChipRam_20_ = {1'd0, Axi4Incr_validSize};
  assign _zz_Axi4SharedOnChipRam_21_ = {1'd0, _zz_Axi4SharedOnChipRam_1_};
  always @ (*) begin
    _zz_Axi4SharedOnChipRam_13_ = {_zz_Axi4SharedOnChipRam_25_, _zz_Axi4SharedOnChipRam_24_, _zz_Axi4SharedOnChipRam_23_, _zz_Axi4SharedOnChipRam_22_};
  end
  always @ (posedge toplevel_main_clk) begin
    if(io_axi_w_payload_strb[0] && _zz_Axi4SharedOnChipRam_4_ && stage0_payload_fragment_write ) begin
      ram_symbol0[_zz_Axi4SharedOnChipRam_3_] <= _zz_Axi4SharedOnChipRam_5_[7 : 0];
    end
    if(io_axi_w_payload_strb[1] && _zz_Axi4SharedOnChipRam_4_ && stage0_payload_fragment_write ) begin
      ram_symbol1[_zz_Axi4SharedOnChipRam_3_] <= _zz_Axi4SharedOnChipRam_5_[15 : 8];
    end
    if(io_axi_w_payload_strb[2] && _zz_Axi4SharedOnChipRam_4_ && stage0_payload_fragment_write ) begin
      ram_symbol2[_zz_Axi4SharedOnChipRam_3_] <= _zz_Axi4SharedOnChipRam_5_[23 : 16];
    end
    if(io_axi_w_payload_strb[3] && _zz_Axi4SharedOnChipRam_4_ && stage0_payload_fragment_write ) begin
      ram_symbol3[_zz_Axi4SharedOnChipRam_3_] <= _zz_Axi4SharedOnChipRam_5_[31 : 24];
    end
    if(_zz_Axi4SharedOnChipRam_4_) begin
      _zz_Axi4SharedOnChipRam_22_ <= ram_symbol0[_zz_Axi4SharedOnChipRam_3_];
      _zz_Axi4SharedOnChipRam_23_ <= ram_symbol1[_zz_Axi4SharedOnChipRam_3_];
      _zz_Axi4SharedOnChipRam_24_ <= ram_symbol2[_zz_Axi4SharedOnChipRam_3_];
      _zz_Axi4SharedOnChipRam_25_ <= ram_symbol3[_zz_Axi4SharedOnChipRam_3_];
    end
  end

  always @(*) begin
    case(Axi4Incr_wrapCase)
      3'b000 : begin
        _zz_Axi4SharedOnChipRam_14_ = {Axi4Incr_base[11 : 1],Axi4Incr_baseIncr[0 : 0]};
      end
      3'b001 : begin
        _zz_Axi4SharedOnChipRam_14_ = {Axi4Incr_base[11 : 2],Axi4Incr_baseIncr[1 : 0]};
      end
      3'b010 : begin
        _zz_Axi4SharedOnChipRam_14_ = {Axi4Incr_base[11 : 3],Axi4Incr_baseIncr[2 : 0]};
      end
      3'b011 : begin
        _zz_Axi4SharedOnChipRam_14_ = {Axi4Incr_base[11 : 4],Axi4Incr_baseIncr[3 : 0]};
      end
      3'b100 : begin
        _zz_Axi4SharedOnChipRam_14_ = {Axi4Incr_base[11 : 5],Axi4Incr_baseIncr[4 : 0]};
      end
      default : begin
        _zz_Axi4SharedOnChipRam_14_ = {Axi4Incr_base[11 : 6],Axi4Incr_baseIncr[5 : 0]};
      end
    endcase
  end

  assign unburstify_buffer_last = (unburstify_buffer_beat == (8'b00000001));
  assign Axi4Incr_validSize = unburstify_buffer_transaction_size[1 : 0];
  assign Axi4Incr_highCat = unburstify_buffer_transaction_addr[12 : 12];
  assign Axi4Incr_sizeValue = {((2'b10) == Axi4Incr_validSize),{((2'b01) == Axi4Incr_validSize),((2'b00) == Axi4Incr_validSize)}};
  assign Axi4Incr_alignMask = {10'd0, _zz_Axi4SharedOnChipRam_16_};
  assign Axi4Incr_base = (_zz_Axi4SharedOnChipRam_18_ & (~ Axi4Incr_alignMask));
  assign Axi4Incr_baseIncr = (Axi4Incr_base + _zz_Axi4SharedOnChipRam_19_);
  always @ (*) begin
    if((((unburstify_buffer_len & (8'b00001000)) == (8'b00001000)))) begin
        _zz_Axi4SharedOnChipRam_1_ = (2'b11);
    end else if((((unburstify_buffer_len & (8'b00000100)) == (8'b00000100)))) begin
        _zz_Axi4SharedOnChipRam_1_ = (2'b10);
    end else if((((unburstify_buffer_len & (8'b00000010)) == (8'b00000010)))) begin
        _zz_Axi4SharedOnChipRam_1_ = (2'b01);
    end else begin
        _zz_Axi4SharedOnChipRam_1_ = (2'b00);
    end
  end

  assign Axi4Incr_wrapCase = (_zz_Axi4SharedOnChipRam_20_ + _zz_Axi4SharedOnChipRam_21_);
  always @ (*) begin
    case(unburstify_buffer_transaction_burst)
      2'b00 : begin
        unburstify_buffer_result = unburstify_buffer_transaction_addr;
      end
      2'b10 : begin
        unburstify_buffer_result = {Axi4Incr_highCat,_zz_Axi4SharedOnChipRam_14_};
      end
      default : begin
        unburstify_buffer_result = {Axi4Incr_highCat,Axi4Incr_baseIncr};
      end
    endcase
  end

  always @ (*) begin
    io_axi_arw_ready = 1'b0;
    if(! unburstify_buffer_valid) begin
      io_axi_arw_ready = arw_ready;
    end
  end

  always @ (*) begin
    if(unburstify_buffer_valid)begin
      arw_valid = 1'b1;
      arw_payload_last = unburstify_buffer_last;
      arw_payload_fragment_id = unburstify_buffer_transaction_id;
      arw_payload_fragment_size = unburstify_buffer_transaction_size;
      arw_payload_fragment_burst = unburstify_buffer_transaction_burst;
      arw_payload_fragment_write = unburstify_buffer_transaction_write;
      arw_payload_fragment_addr = unburstify_buffer_result;
    end else begin
      arw_valid = io_axi_arw_valid;
      arw_payload_fragment_addr = io_axi_arw_payload_addr;
      arw_payload_fragment_id = io_axi_arw_payload_id;
      arw_payload_fragment_size = io_axi_arw_payload_size;
      arw_payload_fragment_burst = io_axi_arw_payload_burst;
      arw_payload_fragment_write = io_axi_arw_payload_write;
      if(_zz_Axi4SharedOnChipRam_15_)begin
        arw_payload_last = 1'b1;
      end else begin
        arw_payload_last = 1'b0;
      end
    end
  end

  assign _zz_Axi4SharedOnChipRam_2_ = (! (arw_payload_fragment_write && (! io_axi_w_valid)));
  assign stage0_valid = (arw_valid && _zz_Axi4SharedOnChipRam_2_);
  assign arw_ready = (stage0_ready && _zz_Axi4SharedOnChipRam_2_);
  assign stage0_payload_last = arw_payload_last;
  assign stage0_payload_fragment_addr = arw_payload_fragment_addr;
  assign stage0_payload_fragment_id = arw_payload_fragment_id;
  assign stage0_payload_fragment_size = arw_payload_fragment_size;
  assign stage0_payload_fragment_burst = arw_payload_fragment_burst;
  assign stage0_payload_fragment_write = arw_payload_fragment_write;
  assign _zz_Axi4SharedOnChipRam_3_ = stage0_payload_fragment_addr[12 : 2];
  assign _zz_Axi4SharedOnChipRam_4_ = (stage0_valid && stage0_ready);
  assign _zz_Axi4SharedOnChipRam_5_ = io_axi_w_payload_data;
  assign io_axi_r_payload_data = _zz_Axi4SharedOnChipRam_13_;
  assign io_axi_w_ready = ((arw_valid && arw_payload_fragment_write) && stage0_ready);
  assign stage0_ready = ((1'b1 && (! stage1_valid)) || stage1_ready);
  assign stage1_valid = _zz_Axi4SharedOnChipRam_6_;
  assign stage1_payload_last = _zz_Axi4SharedOnChipRam_7_;
  assign stage1_payload_fragment_addr = _zz_Axi4SharedOnChipRam_8_;
  assign stage1_payload_fragment_id = _zz_Axi4SharedOnChipRam_9_;
  assign stage1_payload_fragment_size = _zz_Axi4SharedOnChipRam_10_;
  assign stage1_payload_fragment_burst = _zz_Axi4SharedOnChipRam_11_;
  assign stage1_payload_fragment_write = _zz_Axi4SharedOnChipRam_12_;
  assign stage1_ready = ((io_axi_r_ready && (! stage1_payload_fragment_write)) || ((io_axi_b_ready || (! stage1_payload_last)) && stage1_payload_fragment_write));
  assign io_axi_r_valid = (stage1_valid && (! stage1_payload_fragment_write));
  assign io_axi_r_payload_id = stage1_payload_fragment_id;
  assign io_axi_r_payload_last = stage1_payload_last;
  assign io_axi_r_payload_resp = (2'b00);
  assign io_axi_b_valid = ((stage1_valid && stage1_payload_fragment_write) && stage1_payload_last);
  assign io_axi_b_payload_resp = (2'b00);
  assign io_axi_b_payload_id = stage1_payload_fragment_id;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      unburstify_buffer_valid <= 1'b0;
      _zz_Axi4SharedOnChipRam_6_ <= 1'b0;
    end else begin
      if(arw_ready)begin
        if(unburstify_buffer_last)begin
          unburstify_buffer_valid <= 1'b0;
        end
      end
      if(! unburstify_buffer_valid) begin
        if(! _zz_Axi4SharedOnChipRam_15_) begin
          if(arw_ready)begin
            unburstify_buffer_valid <= io_axi_arw_valid;
          end
        end
      end
      if(stage0_ready)begin
        _zz_Axi4SharedOnChipRam_6_ <= stage0_valid;
      end
    end
  end

  always @ (posedge toplevel_main_clk) begin
    if(arw_ready)begin
      unburstify_buffer_beat <= (unburstify_buffer_beat - (8'b00000001));
      unburstify_buffer_transaction_addr[11 : 0] <= unburstify_buffer_result[11 : 0];
    end
    if(! unburstify_buffer_valid) begin
      if(! _zz_Axi4SharedOnChipRam_15_) begin
        if(arw_ready)begin
          unburstify_buffer_transaction_addr <= io_axi_arw_payload_addr;
          unburstify_buffer_transaction_id <= io_axi_arw_payload_id;
          unburstify_buffer_transaction_size <= io_axi_arw_payload_size;
          unburstify_buffer_transaction_burst <= io_axi_arw_payload_burst;
          unburstify_buffer_transaction_write <= io_axi_arw_payload_write;
          unburstify_buffer_beat <= io_axi_arw_payload_len;
          unburstify_buffer_len <= io_axi_arw_payload_len;
        end
      end
    end
    if(stage0_ready)begin
      _zz_Axi4SharedOnChipRam_7_ <= stage0_payload_last;
      _zz_Axi4SharedOnChipRam_8_ <= stage0_payload_fragment_addr;
      _zz_Axi4SharedOnChipRam_9_ <= stage0_payload_fragment_id;
      _zz_Axi4SharedOnChipRam_10_ <= stage0_payload_fragment_size;
      _zz_Axi4SharedOnChipRam_11_ <= stage0_payload_fragment_burst;
      _zz_Axi4SharedOnChipRam_12_ <= stage0_payload_fragment_write;
    end
  end

endmodule

module Axi4SharedToApb3Bridge (
      input   io_axi_arw_valid,
      output reg  io_axi_arw_ready,
      input  [19:0] io_axi_arw_payload_addr,
      input  [3:0] io_axi_arw_payload_id,
      input  [7:0] io_axi_arw_payload_len,
      input  [2:0] io_axi_arw_payload_size,
      input  [1:0] io_axi_arw_payload_burst,
      input   io_axi_arw_payload_write,
      input   io_axi_w_valid,
      output reg  io_axi_w_ready,
      input  [31:0] io_axi_w_payload_data,
      input  [3:0] io_axi_w_payload_strb,
      input   io_axi_w_payload_last,
      output reg  io_axi_b_valid,
      input   io_axi_b_ready,
      output [3:0] io_axi_b_payload_id,
      output [1:0] io_axi_b_payload_resp,
      output reg  io_axi_r_valid,
      input   io_axi_r_ready,
      output [31:0] io_axi_r_payload_data,
      output [3:0] io_axi_r_payload_id,
      output [1:0] io_axi_r_payload_resp,
      output  io_axi_r_payload_last,
      output [19:0] io_apb_PADDR,
      output reg [0:0] io_apb_PSEL,
      output reg  io_apb_PENABLE,
      input   io_apb_PREADY,
      output  io_apb_PWRITE,
      output [31:0] io_apb_PWDATA,
      input  [31:0] io_apb_PRDATA,
      input   io_apb_PSLVERROR,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire  _zz_Axi4SharedToApb3Bridge_1_;
  reg `Axi4ToApb3BridgePhase_defaultEncoding_type phase;
  reg  write;
  reg [31:0] readedData;
  reg [3:0] id;
  `ifndef SYNTHESIS
  reg [63:0] phase_string;
  `endif

  assign _zz_Axi4SharedToApb3Bridge_1_ = (io_axi_arw_valid && ((! io_axi_arw_payload_write) || io_axi_w_valid));
  `ifndef SYNTHESIS
  always @(*) begin
    case(phase)
      `Axi4ToApb3BridgePhase_defaultEncoding_SETUP : phase_string = "SETUP   ";
      `Axi4ToApb3BridgePhase_defaultEncoding_ACCESS_1 : phase_string = "ACCESS_1";
      `Axi4ToApb3BridgePhase_defaultEncoding_RESPONSE : phase_string = "RESPONSE";
      default : phase_string = "????????";
    endcase
  end
  `endif

  always @ (*) begin
    io_axi_arw_ready = 1'b0;
    io_axi_w_ready = 1'b0;
    io_axi_b_valid = 1'b0;
    io_axi_r_valid = 1'b0;
    io_apb_PSEL[0] = 1'b0;
    io_apb_PENABLE = 1'b0;
    case(phase)
      `Axi4ToApb3BridgePhase_defaultEncoding_SETUP : begin
        if(_zz_Axi4SharedToApb3Bridge_1_)begin
          io_apb_PSEL[0] = 1'b1;
        end
      end
      `Axi4ToApb3BridgePhase_defaultEncoding_ACCESS_1 : begin
        io_apb_PSEL[0] = 1'b1;
        io_apb_PENABLE = 1'b1;
        if(io_apb_PREADY)begin
          io_axi_arw_ready = 1'b1;
          io_axi_w_ready = write;
        end
      end
      default : begin
        if(write)begin
          io_axi_b_valid = 1'b1;
        end else begin
          io_axi_r_valid = 1'b1;
        end
      end
    endcase
  end

  assign io_apb_PADDR = io_axi_arw_payload_addr;
  assign io_apb_PWDATA = io_axi_w_payload_data;
  assign io_apb_PWRITE = io_axi_arw_payload_write;
  assign io_axi_r_payload_resp = {io_apb_PSLVERROR,(1'b0)};
  assign io_axi_b_payload_resp = {io_apb_PSLVERROR,(1'b0)};
  assign io_axi_r_payload_id = id;
  assign io_axi_b_payload_id = id;
  assign io_axi_r_payload_data = readedData;
  assign io_axi_r_payload_last = 1'b1;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      phase <= `Axi4ToApb3BridgePhase_defaultEncoding_SETUP;
    end else begin
      case(phase)
        `Axi4ToApb3BridgePhase_defaultEncoding_SETUP : begin
          if(_zz_Axi4SharedToApb3Bridge_1_)begin
            phase <= `Axi4ToApb3BridgePhase_defaultEncoding_ACCESS_1;
          end
        end
        `Axi4ToApb3BridgePhase_defaultEncoding_ACCESS_1 : begin
          if(io_apb_PREADY)begin
            phase <= `Axi4ToApb3BridgePhase_defaultEncoding_RESPONSE;
          end
        end
        default : begin
          if(write)begin
            if(io_axi_b_ready)begin
              phase <= `Axi4ToApb3BridgePhase_defaultEncoding_SETUP;
            end
          end else begin
            if(io_axi_r_ready)begin
              phase <= `Axi4ToApb3BridgePhase_defaultEncoding_SETUP;
            end
          end
        end
      endcase
    end
  end

  always @ (posedge toplevel_main_clk) begin
    case(phase)
      `Axi4ToApb3BridgePhase_defaultEncoding_SETUP : begin
        write <= io_axi_arw_payload_write;
        id <= io_axi_arw_payload_id;
      end
      `Axi4ToApb3BridgePhase_defaultEncoding_ACCESS_1 : begin
        if(io_apb_PREADY)begin
          readedData <= io_apb_PRDATA;
        end
      end
      default : begin
      end
    endcase
  end

endmodule

module Axi4ReadOnlyDecoder (
      input   io_input_ar_valid,
      output  io_input_ar_ready,
      input  [31:0] io_input_ar_payload_addr,
      input  [3:0] io_input_ar_payload_cache,
      input  [2:0] io_input_ar_payload_prot,
      output  io_input_r_valid,
      input   io_input_r_ready,
      output [31:0] io_input_r_payload_data,
      output reg [1:0] io_input_r_payload_resp,
      output reg  io_input_r_payload_last,
      output  io_outputs_0_ar_valid,
      input   io_outputs_0_ar_ready,
      output [31:0] io_outputs_0_ar_payload_addr,
      output [3:0] io_outputs_0_ar_payload_cache,
      output [2:0] io_outputs_0_ar_payload_prot,
      input   io_outputs_0_r_valid,
      output  io_outputs_0_r_ready,
      input  [31:0] io_outputs_0_r_payload_data,
      input  [1:0] io_outputs_0_r_payload_resp,
      input   io_outputs_0_r_payload_last,
      output  io_outputs_1_ar_valid,
      input   io_outputs_1_ar_ready,
      output [31:0] io_outputs_1_ar_payload_addr,
      output [3:0] io_outputs_1_ar_payload_cache,
      output [2:0] io_outputs_1_ar_payload_prot,
      input   io_outputs_1_r_valid,
      output  io_outputs_1_r_ready,
      input  [31:0] io_outputs_1_r_payload_data,
      input  [1:0] io_outputs_1_r_payload_resp,
      input   io_outputs_1_r_payload_last,
      output  io_outputs_2_ar_valid,
      input   io_outputs_2_ar_ready,
      output [31:0] io_outputs_2_ar_payload_addr,
      output [3:0] io_outputs_2_ar_payload_cache,
      output [2:0] io_outputs_2_ar_payload_prot,
      input   io_outputs_2_r_valid,
      output  io_outputs_2_r_ready,
      input  [31:0] io_outputs_2_r_payload_data,
      input  [1:0] io_outputs_2_r_payload_resp,
      input   io_outputs_2_r_payload_last,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire  _zz_Axi4ReadOnlyDecoder_6_;
  reg [31:0] _zz_Axi4ReadOnlyDecoder_7_;
  reg [1:0] _zz_Axi4ReadOnlyDecoder_8_;
  reg  _zz_Axi4ReadOnlyDecoder_9_;
  wire  errorSlave_io_axi_ar_ready;
  wire  errorSlave_io_axi_r_valid;
  wire [31:0] errorSlave_io_axi_r_payload_data;
  wire [1:0] errorSlave_io_axi_r_payload_resp;
  wire  errorSlave_io_axi_r_payload_last;
  wire [31:0] _zz_Axi4ReadOnlyDecoder_10_;
  wire [31:0] _zz_Axi4ReadOnlyDecoder_11_;
  wire [31:0] _zz_Axi4ReadOnlyDecoder_12_;
  reg  pendingCmdCounter_incrementIt;
  reg  pendingCmdCounter_decrementIt;
  wire [2:0] pendingCmdCounter_valueNext;
  reg [2:0] pendingCmdCounter_value;
  wire  pendingCmdCounter_willOverflowIfInc;
  wire  pendingCmdCounter_willOverflow;
  reg [2:0] pendingCmdCounter_finalIncrement;
  wire [2:0] decodedCmdSels;
  wire  decodedCmdError;
  reg [2:0] pendingSels;
  reg  pendingError;
  wire  allowCmd;
  wire  _zz_Axi4ReadOnlyDecoder_1_;
  wire  _zz_Axi4ReadOnlyDecoder_2_;
  wire [1:0] readRspIndex;
  wire  _zz_Axi4ReadOnlyDecoder_3_;
  wire  _zz_Axi4ReadOnlyDecoder_4_;
  wire [1:0] _zz_Axi4ReadOnlyDecoder_5_;
  assign _zz_Axi4ReadOnlyDecoder_10_ = (32'b11111100000000000000000000000000);
  assign _zz_Axi4ReadOnlyDecoder_11_ = (32'b11111111111111111110000000000000);
  assign _zz_Axi4ReadOnlyDecoder_12_ = (32'b11111100000000000000000000000000);
  Axi4ReadOnlyErrorSlave errorSlave ( 
    .io_axi_ar_valid(_zz_Axi4ReadOnlyDecoder_6_),
    .io_axi_ar_ready(errorSlave_io_axi_ar_ready),
    .io_axi_ar_payload_addr(io_input_ar_payload_addr),
    .io_axi_ar_payload_cache(io_input_ar_payload_cache),
    .io_axi_ar_payload_prot(io_input_ar_payload_prot),
    .io_axi_r_valid(errorSlave_io_axi_r_valid),
    .io_axi_r_ready(io_input_r_ready),
    .io_axi_r_payload_data(errorSlave_io_axi_r_payload_data),
    .io_axi_r_payload_resp(errorSlave_io_axi_r_payload_resp),
    .io_axi_r_payload_last(errorSlave_io_axi_r_payload_last),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  always @(*) begin
    case(_zz_Axi4ReadOnlyDecoder_5_)
      2'b00 : begin
        _zz_Axi4ReadOnlyDecoder_7_ = io_outputs_0_r_payload_data;
        _zz_Axi4ReadOnlyDecoder_8_ = io_outputs_0_r_payload_resp;
        _zz_Axi4ReadOnlyDecoder_9_ = io_outputs_0_r_payload_last;
      end
      2'b01 : begin
        _zz_Axi4ReadOnlyDecoder_7_ = io_outputs_1_r_payload_data;
        _zz_Axi4ReadOnlyDecoder_8_ = io_outputs_1_r_payload_resp;
        _zz_Axi4ReadOnlyDecoder_9_ = io_outputs_1_r_payload_last;
      end
      default : begin
        _zz_Axi4ReadOnlyDecoder_7_ = io_outputs_2_r_payload_data;
        _zz_Axi4ReadOnlyDecoder_8_ = io_outputs_2_r_payload_resp;
        _zz_Axi4ReadOnlyDecoder_9_ = io_outputs_2_r_payload_last;
      end
    endcase
  end

  always @ (*) begin
    pendingCmdCounter_incrementIt = 1'b0;
    if((io_input_ar_valid && io_input_ar_ready))begin
      pendingCmdCounter_incrementIt = 1'b1;
    end
  end

  always @ (*) begin
    pendingCmdCounter_decrementIt = 1'b0;
    if(((io_input_r_valid && io_input_r_ready) && io_input_r_payload_last))begin
      pendingCmdCounter_decrementIt = 1'b1;
    end
  end

  assign pendingCmdCounter_willOverflowIfInc = ((pendingCmdCounter_value == (3'b111)) && (! pendingCmdCounter_decrementIt));
  assign pendingCmdCounter_willOverflow = (pendingCmdCounter_willOverflowIfInc && pendingCmdCounter_incrementIt);
  always @ (*) begin
    if((pendingCmdCounter_incrementIt && (! pendingCmdCounter_decrementIt)))begin
      pendingCmdCounter_finalIncrement = (3'b001);
    end else begin
      if(((! pendingCmdCounter_incrementIt) && pendingCmdCounter_decrementIt))begin
        pendingCmdCounter_finalIncrement = (3'b111);
      end else begin
        pendingCmdCounter_finalIncrement = (3'b000);
      end
    end
  end

  assign pendingCmdCounter_valueNext = (pendingCmdCounter_value + pendingCmdCounter_finalIncrement);
  assign decodedCmdSels = {(((io_input_ar_payload_addr & _zz_Axi4ReadOnlyDecoder_10_) == (32'b01000100000000000000000000000000)) && io_input_ar_valid),{(((io_input_ar_payload_addr & _zz_Axi4ReadOnlyDecoder_11_) == (32'b00000000000000000000000000000000)) && io_input_ar_valid),(((io_input_ar_payload_addr & _zz_Axi4ReadOnlyDecoder_12_) == (32'b01000000000000000000000000000000)) && io_input_ar_valid)}};
  assign decodedCmdError = (decodedCmdSels == (3'b000));
  assign allowCmd = ((pendingCmdCounter_value == (3'b000)) || ((pendingCmdCounter_value != (3'b111)) && (pendingSels == decodedCmdSels)));
  assign io_input_ar_ready = ((((decodedCmdSels & {io_outputs_2_ar_ready,{io_outputs_1_ar_ready,io_outputs_0_ar_ready}}) != (3'b000)) || (decodedCmdError && errorSlave_io_axi_ar_ready)) && allowCmd);
  assign _zz_Axi4ReadOnlyDecoder_6_ = ((io_input_ar_valid && decodedCmdError) && allowCmd);
  assign io_outputs_0_ar_valid = ((io_input_ar_valid && decodedCmdSels[0]) && allowCmd);
  assign io_outputs_0_ar_payload_addr = io_input_ar_payload_addr;
  assign io_outputs_0_ar_payload_cache = io_input_ar_payload_cache;
  assign io_outputs_0_ar_payload_prot = io_input_ar_payload_prot;
  assign io_outputs_1_ar_valid = ((io_input_ar_valid && decodedCmdSels[1]) && allowCmd);
  assign io_outputs_1_ar_payload_addr = io_input_ar_payload_addr;
  assign io_outputs_1_ar_payload_cache = io_input_ar_payload_cache;
  assign io_outputs_1_ar_payload_prot = io_input_ar_payload_prot;
  assign io_outputs_2_ar_valid = ((io_input_ar_valid && decodedCmdSels[2]) && allowCmd);
  assign io_outputs_2_ar_payload_addr = io_input_ar_payload_addr;
  assign io_outputs_2_ar_payload_cache = io_input_ar_payload_cache;
  assign io_outputs_2_ar_payload_prot = io_input_ar_payload_prot;
  assign _zz_Axi4ReadOnlyDecoder_1_ = pendingSels[1];
  assign _zz_Axi4ReadOnlyDecoder_2_ = pendingSels[2];
  assign readRspIndex = {_zz_Axi4ReadOnlyDecoder_2_,_zz_Axi4ReadOnlyDecoder_1_};
  assign io_input_r_valid = (({io_outputs_2_r_valid,{io_outputs_1_r_valid,io_outputs_0_r_valid}} != (3'b000)) || errorSlave_io_axi_r_valid);
  assign _zz_Axi4ReadOnlyDecoder_3_ = pendingSels[1];
  assign _zz_Axi4ReadOnlyDecoder_4_ = pendingSels[2];
  assign _zz_Axi4ReadOnlyDecoder_5_ = {_zz_Axi4ReadOnlyDecoder_4_,_zz_Axi4ReadOnlyDecoder_3_};
  assign io_input_r_payload_data = _zz_Axi4ReadOnlyDecoder_7_;
  always @ (*) begin
    io_input_r_payload_resp = _zz_Axi4ReadOnlyDecoder_8_;
    io_input_r_payload_last = _zz_Axi4ReadOnlyDecoder_9_;
    if(pendingError)begin
      io_input_r_payload_resp = errorSlave_io_axi_r_payload_resp;
      io_input_r_payload_last = errorSlave_io_axi_r_payload_last;
    end
  end

  assign io_outputs_0_r_ready = io_input_r_ready;
  assign io_outputs_1_r_ready = io_input_r_ready;
  assign io_outputs_2_r_ready = io_input_r_ready;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      pendingCmdCounter_value <= (3'b000);
      pendingSels <= (3'b000);
      pendingError <= 1'b0;
    end else begin
      pendingCmdCounter_value <= pendingCmdCounter_valueNext;
      if(io_input_ar_ready)begin
        pendingSels <= decodedCmdSels;
      end
      if(io_input_ar_ready)begin
        pendingError <= decodedCmdError;
      end
    end
  end

endmodule

module Axi4SharedDecoder (
      input   io_input_arw_valid,
      output  io_input_arw_ready,
      input  [31:0] io_input_arw_payload_addr,
      input  [2:0] io_input_arw_payload_size,
      input  [3:0] io_input_arw_payload_cache,
      input  [2:0] io_input_arw_payload_prot,
      input   io_input_arw_payload_write,
      input   io_input_w_valid,
      output  io_input_w_ready,
      input  [31:0] io_input_w_payload_data,
      input  [3:0] io_input_w_payload_strb,
      input   io_input_w_payload_last,
      output  io_input_b_valid,
      input   io_input_b_ready,
      output reg [1:0] io_input_b_payload_resp,
      output  io_input_r_valid,
      input   io_input_r_ready,
      output [31:0] io_input_r_payload_data,
      output reg [1:0] io_input_r_payload_resp,
      output reg  io_input_r_payload_last,
      output  io_sharedOutputs_0_arw_valid,
      input   io_sharedOutputs_0_arw_ready,
      output [31:0] io_sharedOutputs_0_arw_payload_addr,
      output [2:0] io_sharedOutputs_0_arw_payload_size,
      output [3:0] io_sharedOutputs_0_arw_payload_cache,
      output [2:0] io_sharedOutputs_0_arw_payload_prot,
      output  io_sharedOutputs_0_arw_payload_write,
      output  io_sharedOutputs_0_w_valid,
      input   io_sharedOutputs_0_w_ready,
      output [31:0] io_sharedOutputs_0_w_payload_data,
      output [3:0] io_sharedOutputs_0_w_payload_strb,
      output  io_sharedOutputs_0_w_payload_last,
      input   io_sharedOutputs_0_b_valid,
      output  io_sharedOutputs_0_b_ready,
      input  [1:0] io_sharedOutputs_0_b_payload_resp,
      input   io_sharedOutputs_0_r_valid,
      output  io_sharedOutputs_0_r_ready,
      input  [31:0] io_sharedOutputs_0_r_payload_data,
      input  [1:0] io_sharedOutputs_0_r_payload_resp,
      input   io_sharedOutputs_0_r_payload_last,
      output  io_sharedOutputs_1_arw_valid,
      input   io_sharedOutputs_1_arw_ready,
      output [31:0] io_sharedOutputs_1_arw_payload_addr,
      output [2:0] io_sharedOutputs_1_arw_payload_size,
      output [3:0] io_sharedOutputs_1_arw_payload_cache,
      output [2:0] io_sharedOutputs_1_arw_payload_prot,
      output  io_sharedOutputs_1_arw_payload_write,
      output  io_sharedOutputs_1_w_valid,
      input   io_sharedOutputs_1_w_ready,
      output [31:0] io_sharedOutputs_1_w_payload_data,
      output [3:0] io_sharedOutputs_1_w_payload_strb,
      output  io_sharedOutputs_1_w_payload_last,
      input   io_sharedOutputs_1_b_valid,
      output  io_sharedOutputs_1_b_ready,
      input  [1:0] io_sharedOutputs_1_b_payload_resp,
      input   io_sharedOutputs_1_r_valid,
      output  io_sharedOutputs_1_r_ready,
      input  [31:0] io_sharedOutputs_1_r_payload_data,
      input  [1:0] io_sharedOutputs_1_r_payload_resp,
      input   io_sharedOutputs_1_r_payload_last,
      output  io_sharedOutputs_2_arw_valid,
      input   io_sharedOutputs_2_arw_ready,
      output [31:0] io_sharedOutputs_2_arw_payload_addr,
      output [2:0] io_sharedOutputs_2_arw_payload_size,
      output [3:0] io_sharedOutputs_2_arw_payload_cache,
      output [2:0] io_sharedOutputs_2_arw_payload_prot,
      output  io_sharedOutputs_2_arw_payload_write,
      output  io_sharedOutputs_2_w_valid,
      input   io_sharedOutputs_2_w_ready,
      output [31:0] io_sharedOutputs_2_w_payload_data,
      output [3:0] io_sharedOutputs_2_w_payload_strb,
      output  io_sharedOutputs_2_w_payload_last,
      input   io_sharedOutputs_2_b_valid,
      output  io_sharedOutputs_2_b_ready,
      input  [1:0] io_sharedOutputs_2_b_payload_resp,
      input   io_sharedOutputs_2_r_valid,
      output  io_sharedOutputs_2_r_ready,
      input  [31:0] io_sharedOutputs_2_r_payload_data,
      input  [1:0] io_sharedOutputs_2_r_payload_resp,
      input   io_sharedOutputs_2_r_payload_last,
      output  io_sharedOutputs_3_arw_valid,
      input   io_sharedOutputs_3_arw_ready,
      output [31:0] io_sharedOutputs_3_arw_payload_addr,
      output [2:0] io_sharedOutputs_3_arw_payload_size,
      output [3:0] io_sharedOutputs_3_arw_payload_cache,
      output [2:0] io_sharedOutputs_3_arw_payload_prot,
      output  io_sharedOutputs_3_arw_payload_write,
      output  io_sharedOutputs_3_w_valid,
      input   io_sharedOutputs_3_w_ready,
      output [31:0] io_sharedOutputs_3_w_payload_data,
      output [3:0] io_sharedOutputs_3_w_payload_strb,
      output  io_sharedOutputs_3_w_payload_last,
      input   io_sharedOutputs_3_b_valid,
      output  io_sharedOutputs_3_b_ready,
      input  [1:0] io_sharedOutputs_3_b_payload_resp,
      input   io_sharedOutputs_3_r_valid,
      output  io_sharedOutputs_3_r_ready,
      input  [31:0] io_sharedOutputs_3_r_payload_data,
      input  [1:0] io_sharedOutputs_3_r_payload_resp,
      input   io_sharedOutputs_3_r_payload_last,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire  _zz_Axi4SharedDecoder_16_;
  wire  _zz_Axi4SharedDecoder_17_;
  reg [1:0] _zz_Axi4SharedDecoder_18_;
  reg [31:0] _zz_Axi4SharedDecoder_19_;
  reg [1:0] _zz_Axi4SharedDecoder_20_;
  reg  _zz_Axi4SharedDecoder_21_;
  wire  errorSlave_io_axi_arw_ready;
  wire  errorSlave_io_axi_w_ready;
  wire  errorSlave_io_axi_b_valid;
  wire [1:0] errorSlave_io_axi_b_payload_resp;
  wire  errorSlave_io_axi_r_valid;
  wire [31:0] errorSlave_io_axi_r_payload_data;
  wire [1:0] errorSlave_io_axi_r_payload_resp;
  wire  errorSlave_io_axi_r_payload_last;
  wire [31:0] _zz_Axi4SharedDecoder_22_;
  wire [31:0] _zz_Axi4SharedDecoder_23_;
  wire [31:0] _zz_Axi4SharedDecoder_24_;
  wire [31:0] _zz_Axi4SharedDecoder_25_;
  reg [2:0] _zz_Axi4SharedDecoder_1_;
  reg [2:0] _zz_Axi4SharedDecoder_2_;
  reg [2:0] _zz_Axi4SharedDecoder_3_;
  wire  cmdAllowedStart;
  reg [2:0] pendingCmdCounter;
  wire [2:0] _zz_Axi4SharedDecoder_4_;
  reg  pendingDataCounter_incrementIt;
  reg  pendingDataCounter_decrementIt;
  wire [2:0] pendingDataCounter_valueNext;
  reg [2:0] pendingDataCounter_value;
  wire  pendingDataCounter_willOverflowIfInc;
  wire  pendingDataCounter_willOverflow;
  reg [2:0] pendingDataCounter_finalIncrement;
  wire [3:0] decodedCmdSels;
  wire  decodedCmdError;
  reg [3:0] pendingSels;
  reg  pendingError;
  wire  allowCmd;
  wire  allowData;
  reg  _zz_Axi4SharedDecoder_5_;
  wire [3:0] _zz_Axi4SharedDecoder_6_;
  wire [3:0] _zz_Axi4SharedDecoder_7_;
  wire [3:0] _zz_Axi4SharedDecoder_8_;
  wire  _zz_Axi4SharedDecoder_9_;
  wire  _zz_Axi4SharedDecoder_10_;
  wire  _zz_Axi4SharedDecoder_11_;
  wire [1:0] writeRspIndex;
  wire [3:0] _zz_Axi4SharedDecoder_12_;
  wire  _zz_Axi4SharedDecoder_13_;
  wire  _zz_Axi4SharedDecoder_14_;
  wire  _zz_Axi4SharedDecoder_15_;
  wire [1:0] readRspIndex;
  assign _zz_Axi4SharedDecoder_22_ = (32'b11111111111100000000000000000000);
  assign _zz_Axi4SharedDecoder_23_ = (32'b11111100000000000000000000000000);
  assign _zz_Axi4SharedDecoder_24_ = (32'b11111111111111111110000000000000);
  assign _zz_Axi4SharedDecoder_25_ = (32'b11111100000000000000000000000000);
  Axi4SharedErrorSlave errorSlave ( 
    .io_axi_arw_valid(_zz_Axi4SharedDecoder_16_),
    .io_axi_arw_ready(errorSlave_io_axi_arw_ready),
    .io_axi_arw_payload_addr(io_input_arw_payload_addr),
    .io_axi_arw_payload_size(io_input_arw_payload_size),
    .io_axi_arw_payload_cache(io_input_arw_payload_cache),
    .io_axi_arw_payload_prot(io_input_arw_payload_prot),
    .io_axi_arw_payload_write(io_input_arw_payload_write),
    .io_axi_w_valid(_zz_Axi4SharedDecoder_17_),
    .io_axi_w_ready(errorSlave_io_axi_w_ready),
    .io_axi_w_payload_data(io_input_w_payload_data),
    .io_axi_w_payload_strb(io_input_w_payload_strb),
    .io_axi_w_payload_last(io_input_w_payload_last),
    .io_axi_b_valid(errorSlave_io_axi_b_valid),
    .io_axi_b_ready(io_input_b_ready),
    .io_axi_b_payload_resp(errorSlave_io_axi_b_payload_resp),
    .io_axi_r_valid(errorSlave_io_axi_r_valid),
    .io_axi_r_ready(io_input_r_ready),
    .io_axi_r_payload_data(errorSlave_io_axi_r_payload_data),
    .io_axi_r_payload_resp(errorSlave_io_axi_r_payload_resp),
    .io_axi_r_payload_last(errorSlave_io_axi_r_payload_last),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  always @(*) begin
    case(writeRspIndex)
      2'b00 : begin
        _zz_Axi4SharedDecoder_18_ = io_sharedOutputs_0_b_payload_resp;
      end
      2'b01 : begin
        _zz_Axi4SharedDecoder_18_ = io_sharedOutputs_1_b_payload_resp;
      end
      2'b10 : begin
        _zz_Axi4SharedDecoder_18_ = io_sharedOutputs_2_b_payload_resp;
      end
      default : begin
        _zz_Axi4SharedDecoder_18_ = io_sharedOutputs_3_b_payload_resp;
      end
    endcase
  end

  always @(*) begin
    case(readRspIndex)
      2'b00 : begin
        _zz_Axi4SharedDecoder_19_ = io_sharedOutputs_0_r_payload_data;
        _zz_Axi4SharedDecoder_20_ = io_sharedOutputs_0_r_payload_resp;
        _zz_Axi4SharedDecoder_21_ = io_sharedOutputs_0_r_payload_last;
      end
      2'b01 : begin
        _zz_Axi4SharedDecoder_19_ = io_sharedOutputs_1_r_payload_data;
        _zz_Axi4SharedDecoder_20_ = io_sharedOutputs_1_r_payload_resp;
        _zz_Axi4SharedDecoder_21_ = io_sharedOutputs_1_r_payload_last;
      end
      2'b10 : begin
        _zz_Axi4SharedDecoder_19_ = io_sharedOutputs_2_r_payload_data;
        _zz_Axi4SharedDecoder_20_ = io_sharedOutputs_2_r_payload_resp;
        _zz_Axi4SharedDecoder_21_ = io_sharedOutputs_2_r_payload_last;
      end
      default : begin
        _zz_Axi4SharedDecoder_19_ = io_sharedOutputs_3_r_payload_data;
        _zz_Axi4SharedDecoder_20_ = io_sharedOutputs_3_r_payload_resp;
        _zz_Axi4SharedDecoder_21_ = io_sharedOutputs_3_r_payload_last;
      end
    endcase
  end

  always @ (*) begin
    _zz_Axi4SharedDecoder_1_ = _zz_Axi4SharedDecoder_2_;
    if(((io_input_r_valid && io_input_r_ready) && io_input_r_payload_last))begin
      _zz_Axi4SharedDecoder_1_ = (_zz_Axi4SharedDecoder_2_ - (3'b001));
    end
  end

  always @ (*) begin
    _zz_Axi4SharedDecoder_2_ = _zz_Axi4SharedDecoder_3_;
    if((io_input_b_valid && io_input_b_ready))begin
      _zz_Axi4SharedDecoder_2_ = (_zz_Axi4SharedDecoder_3_ - (3'b001));
    end
  end

  always @ (*) begin
    _zz_Axi4SharedDecoder_3_ = _zz_Axi4SharedDecoder_4_;
    if((io_input_arw_valid && io_input_arw_ready))begin
      _zz_Axi4SharedDecoder_3_ = (_zz_Axi4SharedDecoder_4_ + (3'b001));
    end
  end

  assign _zz_Axi4SharedDecoder_4_ = pendingCmdCounter;
  always @ (*) begin
    pendingDataCounter_incrementIt = 1'b0;
    if((cmdAllowedStart && io_input_arw_payload_write))begin
      pendingDataCounter_incrementIt = 1'b1;
    end
  end

  always @ (*) begin
    pendingDataCounter_decrementIt = 1'b0;
    if(((io_input_w_valid && io_input_w_ready) && io_input_w_payload_last))begin
      pendingDataCounter_decrementIt = 1'b1;
    end
  end

  assign pendingDataCounter_willOverflowIfInc = ((pendingDataCounter_value == (3'b111)) && (! pendingDataCounter_decrementIt));
  assign pendingDataCounter_willOverflow = (pendingDataCounter_willOverflowIfInc && pendingDataCounter_incrementIt);
  always @ (*) begin
    if((pendingDataCounter_incrementIt && (! pendingDataCounter_decrementIt)))begin
      pendingDataCounter_finalIncrement = (3'b001);
    end else begin
      if(((! pendingDataCounter_incrementIt) && pendingDataCounter_decrementIt))begin
        pendingDataCounter_finalIncrement = (3'b111);
      end else begin
        pendingDataCounter_finalIncrement = (3'b000);
      end
    end
  end

  assign pendingDataCounter_valueNext = (pendingDataCounter_value + pendingDataCounter_finalIncrement);
  assign decodedCmdSels = {((io_input_arw_payload_addr & _zz_Axi4SharedDecoder_22_) == (32'b10000000000000000000000000000000)),{((io_input_arw_payload_addr & _zz_Axi4SharedDecoder_23_) == (32'b01000100000000000000000000000000)),{((io_input_arw_payload_addr & _zz_Axi4SharedDecoder_24_) == (32'b00000000000000000000000000000000)),((io_input_arw_payload_addr & _zz_Axi4SharedDecoder_25_) == (32'b01000000000000000000000000000000))}}};
  assign decodedCmdError = (decodedCmdSels == (4'b0000));
  assign allowCmd = ((pendingCmdCounter == (3'b000)) || ((pendingCmdCounter != (3'b111)) && (pendingSels == decodedCmdSels)));
  assign allowData = (pendingDataCounter_value != (3'b000));
  assign cmdAllowedStart = ((io_input_arw_valid && allowCmd) && _zz_Axi4SharedDecoder_5_);
  assign io_input_arw_ready = ((((decodedCmdSels & {io_sharedOutputs_3_arw_ready,{io_sharedOutputs_2_arw_ready,{io_sharedOutputs_1_arw_ready,io_sharedOutputs_0_arw_ready}}}) != (4'b0000)) || (decodedCmdError && errorSlave_io_axi_arw_ready)) && allowCmd);
  assign _zz_Axi4SharedDecoder_16_ = ((io_input_arw_valid && decodedCmdError) && allowCmd);
  assign _zz_Axi4SharedDecoder_6_ = decodedCmdSels[3 : 0];
  assign io_sharedOutputs_0_arw_valid = ((io_input_arw_valid && _zz_Axi4SharedDecoder_6_[0]) && allowCmd);
  assign io_sharedOutputs_0_arw_payload_addr = io_input_arw_payload_addr;
  assign io_sharedOutputs_0_arw_payload_size = io_input_arw_payload_size;
  assign io_sharedOutputs_0_arw_payload_cache = io_input_arw_payload_cache;
  assign io_sharedOutputs_0_arw_payload_prot = io_input_arw_payload_prot;
  assign io_sharedOutputs_0_arw_payload_write = io_input_arw_payload_write;
  assign io_sharedOutputs_1_arw_valid = ((io_input_arw_valid && _zz_Axi4SharedDecoder_6_[1]) && allowCmd);
  assign io_sharedOutputs_1_arw_payload_addr = io_input_arw_payload_addr;
  assign io_sharedOutputs_1_arw_payload_size = io_input_arw_payload_size;
  assign io_sharedOutputs_1_arw_payload_cache = io_input_arw_payload_cache;
  assign io_sharedOutputs_1_arw_payload_prot = io_input_arw_payload_prot;
  assign io_sharedOutputs_1_arw_payload_write = io_input_arw_payload_write;
  assign io_sharedOutputs_2_arw_valid = ((io_input_arw_valid && _zz_Axi4SharedDecoder_6_[2]) && allowCmd);
  assign io_sharedOutputs_2_arw_payload_addr = io_input_arw_payload_addr;
  assign io_sharedOutputs_2_arw_payload_size = io_input_arw_payload_size;
  assign io_sharedOutputs_2_arw_payload_cache = io_input_arw_payload_cache;
  assign io_sharedOutputs_2_arw_payload_prot = io_input_arw_payload_prot;
  assign io_sharedOutputs_2_arw_payload_write = io_input_arw_payload_write;
  assign io_sharedOutputs_3_arw_valid = ((io_input_arw_valid && _zz_Axi4SharedDecoder_6_[3]) && allowCmd);
  assign io_sharedOutputs_3_arw_payload_addr = io_input_arw_payload_addr;
  assign io_sharedOutputs_3_arw_payload_size = io_input_arw_payload_size;
  assign io_sharedOutputs_3_arw_payload_cache = io_input_arw_payload_cache;
  assign io_sharedOutputs_3_arw_payload_prot = io_input_arw_payload_prot;
  assign io_sharedOutputs_3_arw_payload_write = io_input_arw_payload_write;
  assign io_input_w_ready = ((((pendingSels[3 : 0] & {io_sharedOutputs_3_w_ready,{io_sharedOutputs_2_w_ready,{io_sharedOutputs_1_w_ready,io_sharedOutputs_0_w_ready}}}) != (4'b0000)) || (pendingError && errorSlave_io_axi_w_ready)) && allowData);
  assign _zz_Axi4SharedDecoder_17_ = ((io_input_w_valid && pendingError) && allowData);
  assign _zz_Axi4SharedDecoder_7_ = pendingSels[3 : 0];
  assign io_sharedOutputs_0_w_valid = ((io_input_w_valid && _zz_Axi4SharedDecoder_7_[0]) && allowData);
  assign io_sharedOutputs_0_w_payload_data = io_input_w_payload_data;
  assign io_sharedOutputs_0_w_payload_strb = io_input_w_payload_strb;
  assign io_sharedOutputs_0_w_payload_last = io_input_w_payload_last;
  assign io_sharedOutputs_1_w_valid = ((io_input_w_valid && _zz_Axi4SharedDecoder_7_[1]) && allowData);
  assign io_sharedOutputs_1_w_payload_data = io_input_w_payload_data;
  assign io_sharedOutputs_1_w_payload_strb = io_input_w_payload_strb;
  assign io_sharedOutputs_1_w_payload_last = io_input_w_payload_last;
  assign io_sharedOutputs_2_w_valid = ((io_input_w_valid && _zz_Axi4SharedDecoder_7_[2]) && allowData);
  assign io_sharedOutputs_2_w_payload_data = io_input_w_payload_data;
  assign io_sharedOutputs_2_w_payload_strb = io_input_w_payload_strb;
  assign io_sharedOutputs_2_w_payload_last = io_input_w_payload_last;
  assign io_sharedOutputs_3_w_valid = ((io_input_w_valid && _zz_Axi4SharedDecoder_7_[3]) && allowData);
  assign io_sharedOutputs_3_w_payload_data = io_input_w_payload_data;
  assign io_sharedOutputs_3_w_payload_strb = io_input_w_payload_strb;
  assign io_sharedOutputs_3_w_payload_last = io_input_w_payload_last;
  assign _zz_Axi4SharedDecoder_8_ = pendingSels[3 : 0];
  assign _zz_Axi4SharedDecoder_9_ = _zz_Axi4SharedDecoder_8_[3];
  assign _zz_Axi4SharedDecoder_10_ = (_zz_Axi4SharedDecoder_8_[1] || _zz_Axi4SharedDecoder_9_);
  assign _zz_Axi4SharedDecoder_11_ = (_zz_Axi4SharedDecoder_8_[2] || _zz_Axi4SharedDecoder_9_);
  assign writeRspIndex = {_zz_Axi4SharedDecoder_11_,_zz_Axi4SharedDecoder_10_};
  assign io_input_b_valid = (({io_sharedOutputs_3_b_valid,{io_sharedOutputs_2_b_valid,{io_sharedOutputs_1_b_valid,io_sharedOutputs_0_b_valid}}} != (4'b0000)) || errorSlave_io_axi_b_valid);
  always @ (*) begin
    io_input_b_payload_resp = _zz_Axi4SharedDecoder_18_;
    if(pendingError)begin
      io_input_b_payload_resp = errorSlave_io_axi_b_payload_resp;
    end
  end

  assign io_sharedOutputs_0_b_ready = io_input_b_ready;
  assign io_sharedOutputs_1_b_ready = io_input_b_ready;
  assign io_sharedOutputs_2_b_ready = io_input_b_ready;
  assign io_sharedOutputs_3_b_ready = io_input_b_ready;
  assign _zz_Axi4SharedDecoder_12_ = pendingSels[3 : 0];
  assign _zz_Axi4SharedDecoder_13_ = _zz_Axi4SharedDecoder_12_[3];
  assign _zz_Axi4SharedDecoder_14_ = (_zz_Axi4SharedDecoder_12_[1] || _zz_Axi4SharedDecoder_13_);
  assign _zz_Axi4SharedDecoder_15_ = (_zz_Axi4SharedDecoder_12_[2] || _zz_Axi4SharedDecoder_13_);
  assign readRspIndex = {_zz_Axi4SharedDecoder_15_,_zz_Axi4SharedDecoder_14_};
  assign io_input_r_valid = (({io_sharedOutputs_3_r_valid,{io_sharedOutputs_2_r_valid,{io_sharedOutputs_1_r_valid,io_sharedOutputs_0_r_valid}}} != (4'b0000)) || errorSlave_io_axi_r_valid);
  assign io_input_r_payload_data = _zz_Axi4SharedDecoder_19_;
  always @ (*) begin
    io_input_r_payload_resp = _zz_Axi4SharedDecoder_20_;
    io_input_r_payload_last = _zz_Axi4SharedDecoder_21_;
    if(pendingError)begin
      io_input_r_payload_resp = errorSlave_io_axi_r_payload_resp;
      io_input_r_payload_last = errorSlave_io_axi_r_payload_last;
    end
  end

  assign io_sharedOutputs_0_r_ready = io_input_r_ready;
  assign io_sharedOutputs_1_r_ready = io_input_r_ready;
  assign io_sharedOutputs_2_r_ready = io_input_r_ready;
  assign io_sharedOutputs_3_r_ready = io_input_r_ready;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      pendingCmdCounter <= (3'b000);
      pendingDataCounter_value <= (3'b000);
      pendingSels <= (4'b0000);
      pendingError <= 1'b0;
      _zz_Axi4SharedDecoder_5_ <= 1'b1;
    end else begin
      pendingCmdCounter <= _zz_Axi4SharedDecoder_1_;
      pendingDataCounter_value <= pendingDataCounter_valueNext;
      if(cmdAllowedStart)begin
        pendingSels <= decodedCmdSels;
      end
      if(cmdAllowedStart)begin
        pendingError <= decodedCmdError;
      end
      if(cmdAllowedStart)begin
        _zz_Axi4SharedDecoder_5_ <= 1'b0;
      end
      if(io_input_arw_ready)begin
        _zz_Axi4SharedDecoder_5_ <= 1'b1;
      end
    end
  end

endmodule

module Axi4SharedArbiter (
      input   io_readInputs_0_ar_valid,
      output  io_readInputs_0_ar_ready,
      input  [31:0] io_readInputs_0_ar_payload_addr,
      input  [2:0] io_readInputs_0_ar_payload_id,
      input  [3:0] io_readInputs_0_ar_payload_region,
      input  [7:0] io_readInputs_0_ar_payload_len,
      input  [2:0] io_readInputs_0_ar_payload_size,
      input  [1:0] io_readInputs_0_ar_payload_burst,
      input  [0:0] io_readInputs_0_ar_payload_lock,
      input  [3:0] io_readInputs_0_ar_payload_cache,
      input  [3:0] io_readInputs_0_ar_payload_qos,
      input  [2:0] io_readInputs_0_ar_payload_prot,
      output  io_readInputs_0_r_valid,
      input   io_readInputs_0_r_ready,
      output [31:0] io_readInputs_0_r_payload_data,
      output [2:0] io_readInputs_0_r_payload_id,
      output [1:0] io_readInputs_0_r_payload_resp,
      output  io_readInputs_0_r_payload_last,
      input   io_sharedInputs_0_arw_valid,
      output  io_sharedInputs_0_arw_ready,
      input  [31:0] io_sharedInputs_0_arw_payload_addr,
      input  [2:0] io_sharedInputs_0_arw_payload_id,
      input  [3:0] io_sharedInputs_0_arw_payload_region,
      input  [7:0] io_sharedInputs_0_arw_payload_len,
      input  [2:0] io_sharedInputs_0_arw_payload_size,
      input  [1:0] io_sharedInputs_0_arw_payload_burst,
      input  [0:0] io_sharedInputs_0_arw_payload_lock,
      input  [3:0] io_sharedInputs_0_arw_payload_cache,
      input  [3:0] io_sharedInputs_0_arw_payload_qos,
      input  [2:0] io_sharedInputs_0_arw_payload_prot,
      input   io_sharedInputs_0_arw_payload_write,
      input   io_sharedInputs_0_w_valid,
      output  io_sharedInputs_0_w_ready,
      input  [31:0] io_sharedInputs_0_w_payload_data,
      input  [3:0] io_sharedInputs_0_w_payload_strb,
      input   io_sharedInputs_0_w_payload_last,
      output  io_sharedInputs_0_b_valid,
      input   io_sharedInputs_0_b_ready,
      output [2:0] io_sharedInputs_0_b_payload_id,
      output [1:0] io_sharedInputs_0_b_payload_resp,
      output  io_sharedInputs_0_r_valid,
      input   io_sharedInputs_0_r_ready,
      output [31:0] io_sharedInputs_0_r_payload_data,
      output [2:0] io_sharedInputs_0_r_payload_id,
      output [1:0] io_sharedInputs_0_r_payload_resp,
      output  io_sharedInputs_0_r_payload_last,
      output  io_output_arw_valid,
      input   io_output_arw_ready,
      output [31:0] io_output_arw_payload_addr,
      output [3:0] io_output_arw_payload_id,
      output [3:0] io_output_arw_payload_region,
      output [7:0] io_output_arw_payload_len,
      output [2:0] io_output_arw_payload_size,
      output [1:0] io_output_arw_payload_burst,
      output [0:0] io_output_arw_payload_lock,
      output [3:0] io_output_arw_payload_cache,
      output [3:0] io_output_arw_payload_qos,
      output [2:0] io_output_arw_payload_prot,
      output  io_output_arw_payload_write,
      output  io_output_w_valid,
      input   io_output_w_ready,
      output [31:0] io_output_w_payload_data,
      output [3:0] io_output_w_payload_strb,
      output  io_output_w_payload_last,
      input   io_output_b_valid,
      output  io_output_b_ready,
      input  [3:0] io_output_b_payload_id,
      input  [1:0] io_output_b_payload_resp,
      input   io_output_r_valid,
      output  io_output_r_ready,
      input  [31:0] io_output_r_payload_data,
      input  [3:0] io_output_r_payload_id,
      input  [1:0] io_output_r_payload_resp,
      input   io_output_r_payload_last,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg  _zz_Axi4SharedArbiter_4_;
  wire  _zz_Axi4SharedArbiter_5_;
  wire  _zz_Axi4SharedArbiter_6_;
  reg  _zz_Axi4SharedArbiter_7_;
  wire  cmdArbiter_io_inputs_0_ready;
  wire  cmdArbiter_io_inputs_1_ready;
  wire  cmdArbiter_io_output_valid;
  wire [31:0] cmdArbiter_io_output_payload_addr;
  wire [2:0] cmdArbiter_io_output_payload_id;
  wire [3:0] cmdArbiter_io_output_payload_region;
  wire [7:0] cmdArbiter_io_output_payload_len;
  wire [2:0] cmdArbiter_io_output_payload_size;
  wire [1:0] cmdArbiter_io_output_payload_burst;
  wire [0:0] cmdArbiter_io_output_payload_lock;
  wire [3:0] cmdArbiter_io_output_payload_cache;
  wire [3:0] cmdArbiter_io_output_payload_qos;
  wire [2:0] cmdArbiter_io_output_payload_prot;
  wire  cmdArbiter_io_output_payload_write;
  wire [0:0] cmdArbiter_io_chosen;
  wire [1:0] cmdArbiter_io_chosenOH;
  wire  streamFork_5__io_input_ready;
  wire  streamFork_5__io_outputs_0_valid;
  wire [31:0] streamFork_5__io_outputs_0_payload_addr;
  wire [2:0] streamFork_5__io_outputs_0_payload_id;
  wire [3:0] streamFork_5__io_outputs_0_payload_region;
  wire [7:0] streamFork_5__io_outputs_0_payload_len;
  wire [2:0] streamFork_5__io_outputs_0_payload_size;
  wire [1:0] streamFork_5__io_outputs_0_payload_burst;
  wire [0:0] streamFork_5__io_outputs_0_payload_lock;
  wire [3:0] streamFork_5__io_outputs_0_payload_cache;
  wire [3:0] streamFork_5__io_outputs_0_payload_qos;
  wire [2:0] streamFork_5__io_outputs_0_payload_prot;
  wire  streamFork_5__io_outputs_0_payload_write;
  wire  streamFork_5__io_outputs_1_valid;
  wire [31:0] streamFork_5__io_outputs_1_payload_addr;
  wire [2:0] streamFork_5__io_outputs_1_payload_id;
  wire [3:0] streamFork_5__io_outputs_1_payload_region;
  wire [7:0] streamFork_5__io_outputs_1_payload_len;
  wire [2:0] streamFork_5__io_outputs_1_payload_size;
  wire [1:0] streamFork_5__io_outputs_1_payload_burst;
  wire [0:0] streamFork_5__io_outputs_1_payload_lock;
  wire [3:0] streamFork_5__io_outputs_1_payload_cache;
  wire [3:0] streamFork_5__io_outputs_1_payload_qos;
  wire [2:0] streamFork_5__io_outputs_1_payload_prot;
  wire  streamFork_5__io_outputs_1_payload_write;
  wire  streamFifoLowLatency_5__io_push_ready;
  wire  streamFifoLowLatency_5__io_pop_valid;
  wire [2:0] streamFifoLowLatency_5__io_occupancy;
  wire [1:0] _zz_Axi4SharedArbiter_8_;
  wire [2:0] _zz_Axi4SharedArbiter_9_;
  wire [3:0] _zz_Axi4SharedArbiter_10_;
  wire  inputsCmd_0_valid;
  wire  inputsCmd_0_ready;
  wire [31:0] inputsCmd_0_payload_addr;
  wire [2:0] inputsCmd_0_payload_id;
  wire [3:0] inputsCmd_0_payload_region;
  wire [7:0] inputsCmd_0_payload_len;
  wire [2:0] inputsCmd_0_payload_size;
  wire [1:0] inputsCmd_0_payload_burst;
  wire [0:0] inputsCmd_0_payload_lock;
  wire [3:0] inputsCmd_0_payload_cache;
  wire [3:0] inputsCmd_0_payload_qos;
  wire [2:0] inputsCmd_0_payload_prot;
  wire  inputsCmd_0_payload_write;
  wire  inputsCmd_1_valid;
  wire  inputsCmd_1_ready;
  wire [31:0] inputsCmd_1_payload_addr;
  wire [2:0] inputsCmd_1_payload_id;
  wire [3:0] inputsCmd_1_payload_region;
  wire [7:0] inputsCmd_1_payload_len;
  wire [2:0] inputsCmd_1_payload_size;
  wire [1:0] inputsCmd_1_payload_burst;
  wire [0:0] inputsCmd_1_payload_lock;
  wire [3:0] inputsCmd_1_payload_cache;
  wire [3:0] inputsCmd_1_payload_qos;
  wire [2:0] inputsCmd_1_payload_prot;
  wire  inputsCmd_1_payload_write;
  wire  _zz_Axi4SharedArbiter_1_;
  reg  _zz_Axi4SharedArbiter_2_;
  wire  _zz_Axi4SharedArbiter_3_;
  wire  routeDataInput_valid;
  wire  routeDataInput_ready;
  wire [31:0] routeDataInput_payload_data;
  wire [3:0] routeDataInput_payload_strb;
  wire  routeDataInput_payload_last;
  wire  writeRspSels_0;
  wire [0:0] readRspIndex;
  wire  readRspSels_0;
  wire  readRspSels_1;
  assign _zz_Axi4SharedArbiter_8_ = {cmdArbiter_io_chosenOH[1 : 1],cmdArbiter_io_chosenOH[0 : 0]};
  assign _zz_Axi4SharedArbiter_9_ = streamFork_5__io_outputs_0_payload_id;
  assign _zz_Axi4SharedArbiter_10_ = {1'd0, _zz_Axi4SharedArbiter_9_};
  StreamArbiter cmdArbiter ( 
    .io_inputs_0_valid(inputsCmd_0_valid),
    .io_inputs_0_ready(cmdArbiter_io_inputs_0_ready),
    .io_inputs_0_payload_addr(inputsCmd_0_payload_addr),
    .io_inputs_0_payload_id(inputsCmd_0_payload_id),
    .io_inputs_0_payload_region(inputsCmd_0_payload_region),
    .io_inputs_0_payload_len(inputsCmd_0_payload_len),
    .io_inputs_0_payload_size(inputsCmd_0_payload_size),
    .io_inputs_0_payload_burst(inputsCmd_0_payload_burst),
    .io_inputs_0_payload_lock(inputsCmd_0_payload_lock),
    .io_inputs_0_payload_cache(inputsCmd_0_payload_cache),
    .io_inputs_0_payload_qos(inputsCmd_0_payload_qos),
    .io_inputs_0_payload_prot(inputsCmd_0_payload_prot),
    .io_inputs_0_payload_write(inputsCmd_0_payload_write),
    .io_inputs_1_valid(inputsCmd_1_valid),
    .io_inputs_1_ready(cmdArbiter_io_inputs_1_ready),
    .io_inputs_1_payload_addr(inputsCmd_1_payload_addr),
    .io_inputs_1_payload_id(inputsCmd_1_payload_id),
    .io_inputs_1_payload_region(inputsCmd_1_payload_region),
    .io_inputs_1_payload_len(inputsCmd_1_payload_len),
    .io_inputs_1_payload_size(inputsCmd_1_payload_size),
    .io_inputs_1_payload_burst(inputsCmd_1_payload_burst),
    .io_inputs_1_payload_lock(inputsCmd_1_payload_lock),
    .io_inputs_1_payload_cache(inputsCmd_1_payload_cache),
    .io_inputs_1_payload_qos(inputsCmd_1_payload_qos),
    .io_inputs_1_payload_prot(inputsCmd_1_payload_prot),
    .io_inputs_1_payload_write(inputsCmd_1_payload_write),
    .io_output_valid(cmdArbiter_io_output_valid),
    .io_output_ready(streamFork_5__io_input_ready),
    .io_output_payload_addr(cmdArbiter_io_output_payload_addr),
    .io_output_payload_id(cmdArbiter_io_output_payload_id),
    .io_output_payload_region(cmdArbiter_io_output_payload_region),
    .io_output_payload_len(cmdArbiter_io_output_payload_len),
    .io_output_payload_size(cmdArbiter_io_output_payload_size),
    .io_output_payload_burst(cmdArbiter_io_output_payload_burst),
    .io_output_payload_lock(cmdArbiter_io_output_payload_lock),
    .io_output_payload_cache(cmdArbiter_io_output_payload_cache),
    .io_output_payload_qos(cmdArbiter_io_output_payload_qos),
    .io_output_payload_prot(cmdArbiter_io_output_payload_prot),
    .io_output_payload_write(cmdArbiter_io_output_payload_write),
    .io_chosen(cmdArbiter_io_chosen),
    .io_chosenOH(cmdArbiter_io_chosenOH),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  StreamFork streamFork_5_ ( 
    .io_input_valid(cmdArbiter_io_output_valid),
    .io_input_ready(streamFork_5__io_input_ready),
    .io_input_payload_addr(cmdArbiter_io_output_payload_addr),
    .io_input_payload_id(cmdArbiter_io_output_payload_id),
    .io_input_payload_region(cmdArbiter_io_output_payload_region),
    .io_input_payload_len(cmdArbiter_io_output_payload_len),
    .io_input_payload_size(cmdArbiter_io_output_payload_size),
    .io_input_payload_burst(cmdArbiter_io_output_payload_burst),
    .io_input_payload_lock(cmdArbiter_io_output_payload_lock),
    .io_input_payload_cache(cmdArbiter_io_output_payload_cache),
    .io_input_payload_qos(cmdArbiter_io_output_payload_qos),
    .io_input_payload_prot(cmdArbiter_io_output_payload_prot),
    .io_input_payload_write(cmdArbiter_io_output_payload_write),
    .io_outputs_0_valid(streamFork_5__io_outputs_0_valid),
    .io_outputs_0_ready(io_output_arw_ready),
    .io_outputs_0_payload_addr(streamFork_5__io_outputs_0_payload_addr),
    .io_outputs_0_payload_id(streamFork_5__io_outputs_0_payload_id),
    .io_outputs_0_payload_region(streamFork_5__io_outputs_0_payload_region),
    .io_outputs_0_payload_len(streamFork_5__io_outputs_0_payload_len),
    .io_outputs_0_payload_size(streamFork_5__io_outputs_0_payload_size),
    .io_outputs_0_payload_burst(streamFork_5__io_outputs_0_payload_burst),
    .io_outputs_0_payload_lock(streamFork_5__io_outputs_0_payload_lock),
    .io_outputs_0_payload_cache(streamFork_5__io_outputs_0_payload_cache),
    .io_outputs_0_payload_qos(streamFork_5__io_outputs_0_payload_qos),
    .io_outputs_0_payload_prot(streamFork_5__io_outputs_0_payload_prot),
    .io_outputs_0_payload_write(streamFork_5__io_outputs_0_payload_write),
    .io_outputs_1_valid(streamFork_5__io_outputs_1_valid),
    .io_outputs_1_ready(_zz_Axi4SharedArbiter_4_),
    .io_outputs_1_payload_addr(streamFork_5__io_outputs_1_payload_addr),
    .io_outputs_1_payload_id(streamFork_5__io_outputs_1_payload_id),
    .io_outputs_1_payload_region(streamFork_5__io_outputs_1_payload_region),
    .io_outputs_1_payload_len(streamFork_5__io_outputs_1_payload_len),
    .io_outputs_1_payload_size(streamFork_5__io_outputs_1_payload_size),
    .io_outputs_1_payload_burst(streamFork_5__io_outputs_1_payload_burst),
    .io_outputs_1_payload_lock(streamFork_5__io_outputs_1_payload_lock),
    .io_outputs_1_payload_cache(streamFork_5__io_outputs_1_payload_cache),
    .io_outputs_1_payload_qos(streamFork_5__io_outputs_1_payload_qos),
    .io_outputs_1_payload_prot(streamFork_5__io_outputs_1_payload_prot),
    .io_outputs_1_payload_write(streamFork_5__io_outputs_1_payload_write),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  StreamFifoLowLatency_1_ streamFifoLowLatency_5_ ( 
    .io_push_valid(_zz_Axi4SharedArbiter_2_),
    .io_push_ready(streamFifoLowLatency_5__io_push_ready),
    .io_pop_valid(streamFifoLowLatency_5__io_pop_valid),
    .io_pop_ready(_zz_Axi4SharedArbiter_5_),
    .io_flush(_zz_Axi4SharedArbiter_6_),
    .io_occupancy(streamFifoLowLatency_5__io_occupancy),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  always @(*) begin
    case(readRspIndex)
      1'b0 : begin
        _zz_Axi4SharedArbiter_7_ = io_readInputs_0_r_ready;
      end
      default : begin
        _zz_Axi4SharedArbiter_7_ = io_sharedInputs_0_r_ready;
      end
    endcase
  end

  assign inputsCmd_0_valid = io_readInputs_0_ar_valid;
  assign io_readInputs_0_ar_ready = inputsCmd_0_ready;
  assign inputsCmd_0_payload_addr = io_readInputs_0_ar_payload_addr;
  assign inputsCmd_0_payload_id = io_readInputs_0_ar_payload_id;
  assign inputsCmd_0_payload_region = io_readInputs_0_ar_payload_region;
  assign inputsCmd_0_payload_len = io_readInputs_0_ar_payload_len;
  assign inputsCmd_0_payload_size = io_readInputs_0_ar_payload_size;
  assign inputsCmd_0_payload_burst = io_readInputs_0_ar_payload_burst;
  assign inputsCmd_0_payload_lock = io_readInputs_0_ar_payload_lock;
  assign inputsCmd_0_payload_cache = io_readInputs_0_ar_payload_cache;
  assign inputsCmd_0_payload_qos = io_readInputs_0_ar_payload_qos;
  assign inputsCmd_0_payload_prot = io_readInputs_0_ar_payload_prot;
  assign inputsCmd_0_payload_write = 1'b0;
  assign inputsCmd_1_valid = io_sharedInputs_0_arw_valid;
  assign io_sharedInputs_0_arw_ready = inputsCmd_1_ready;
  assign inputsCmd_1_payload_addr = io_sharedInputs_0_arw_payload_addr;
  assign inputsCmd_1_payload_id = io_sharedInputs_0_arw_payload_id;
  assign inputsCmd_1_payload_region = io_sharedInputs_0_arw_payload_region;
  assign inputsCmd_1_payload_len = io_sharedInputs_0_arw_payload_len;
  assign inputsCmd_1_payload_size = io_sharedInputs_0_arw_payload_size;
  assign inputsCmd_1_payload_burst = io_sharedInputs_0_arw_payload_burst;
  assign inputsCmd_1_payload_lock = io_sharedInputs_0_arw_payload_lock;
  assign inputsCmd_1_payload_cache = io_sharedInputs_0_arw_payload_cache;
  assign inputsCmd_1_payload_qos = io_sharedInputs_0_arw_payload_qos;
  assign inputsCmd_1_payload_prot = io_sharedInputs_0_arw_payload_prot;
  assign inputsCmd_1_payload_write = io_sharedInputs_0_arw_payload_write;
  assign inputsCmd_0_ready = cmdArbiter_io_inputs_0_ready;
  assign inputsCmd_1_ready = cmdArbiter_io_inputs_1_ready;
  assign io_output_arw_valid = streamFork_5__io_outputs_0_valid;
  assign io_output_arw_payload_addr = streamFork_5__io_outputs_0_payload_addr;
  assign io_output_arw_payload_region = streamFork_5__io_outputs_0_payload_region;
  assign io_output_arw_payload_len = streamFork_5__io_outputs_0_payload_len;
  assign io_output_arw_payload_size = streamFork_5__io_outputs_0_payload_size;
  assign io_output_arw_payload_burst = streamFork_5__io_outputs_0_payload_burst;
  assign io_output_arw_payload_lock = streamFork_5__io_outputs_0_payload_lock;
  assign io_output_arw_payload_cache = streamFork_5__io_outputs_0_payload_cache;
  assign io_output_arw_payload_qos = streamFork_5__io_outputs_0_payload_qos;
  assign io_output_arw_payload_prot = streamFork_5__io_outputs_0_payload_prot;
  assign io_output_arw_payload_write = streamFork_5__io_outputs_0_payload_write;
  assign _zz_Axi4SharedArbiter_1_ = _zz_Axi4SharedArbiter_8_[1];
  assign io_output_arw_payload_id = (streamFork_5__io_outputs_0_payload_write ? _zz_Axi4SharedArbiter_10_ : {_zz_Axi4SharedArbiter_1_,streamFork_5__io_outputs_0_payload_id});
  always @ (*) begin
    _zz_Axi4SharedArbiter_2_ = streamFork_5__io_outputs_1_valid;
    _zz_Axi4SharedArbiter_4_ = _zz_Axi4SharedArbiter_3_;
    if((! streamFork_5__io_outputs_1_payload_write))begin
      _zz_Axi4SharedArbiter_2_ = 1'b0;
      _zz_Axi4SharedArbiter_4_ = 1'b1;
    end
  end

  assign _zz_Axi4SharedArbiter_3_ = streamFifoLowLatency_5__io_push_ready;
  assign routeDataInput_valid = io_sharedInputs_0_w_valid;
  assign routeDataInput_ready = io_sharedInputs_0_w_ready;
  assign routeDataInput_payload_data = io_sharedInputs_0_w_payload_data;
  assign routeDataInput_payload_strb = io_sharedInputs_0_w_payload_strb;
  assign routeDataInput_payload_last = io_sharedInputs_0_w_payload_last;
  assign io_output_w_valid = (streamFifoLowLatency_5__io_pop_valid && routeDataInput_valid);
  assign io_output_w_payload_data = routeDataInput_payload_data;
  assign io_output_w_payload_strb = routeDataInput_payload_strb;
  assign io_output_w_payload_last = routeDataInput_payload_last;
  assign io_sharedInputs_0_w_ready = ((streamFifoLowLatency_5__io_pop_valid && io_output_w_ready) && 1'b1);
  assign _zz_Axi4SharedArbiter_5_ = ((io_output_w_valid && io_output_w_ready) && io_output_w_payload_last);
  assign writeRspSels_0 = 1'b1;
  assign io_sharedInputs_0_b_valid = (io_output_b_valid && writeRspSels_0);
  assign io_sharedInputs_0_b_payload_resp = io_output_b_payload_resp;
  assign io_sharedInputs_0_b_payload_id = io_output_b_payload_id[2:0];
  assign io_output_b_ready = io_sharedInputs_0_b_ready;
  assign readRspIndex = io_output_r_payload_id[3 : 3];
  assign readRspSels_0 = (readRspIndex == (1'b0));
  assign readRspSels_1 = (readRspIndex == (1'b1));
  assign io_readInputs_0_r_valid = (io_output_r_valid && readRspSels_0);
  assign io_readInputs_0_r_payload_data = io_output_r_payload_data;
  assign io_readInputs_0_r_payload_resp = io_output_r_payload_resp;
  assign io_readInputs_0_r_payload_last = io_output_r_payload_last;
  assign io_readInputs_0_r_payload_id = io_output_r_payload_id[2:0];
  assign io_sharedInputs_0_r_valid = (io_output_r_valid && readRspSels_1);
  assign io_sharedInputs_0_r_payload_data = io_output_r_payload_data;
  assign io_sharedInputs_0_r_payload_resp = io_output_r_payload_resp;
  assign io_sharedInputs_0_r_payload_last = io_output_r_payload_last;
  assign io_sharedInputs_0_r_payload_id = io_output_r_payload_id[2:0];
  assign io_output_r_ready = _zz_Axi4SharedArbiter_7_;
  assign _zz_Axi4SharedArbiter_6_ = 1'b0;
endmodule

module Axi4SharedArbiter_1_ (
      input   io_readInputs_0_ar_valid,
      output  io_readInputs_0_ar_ready,
      input  [31:0] io_readInputs_0_ar_payload_addr,
      input  [2:0] io_readInputs_0_ar_payload_id,
      input  [3:0] io_readInputs_0_ar_payload_region,
      input  [7:0] io_readInputs_0_ar_payload_len,
      input  [2:0] io_readInputs_0_ar_payload_size,
      input  [1:0] io_readInputs_0_ar_payload_burst,
      input  [0:0] io_readInputs_0_ar_payload_lock,
      input  [3:0] io_readInputs_0_ar_payload_cache,
      input  [3:0] io_readInputs_0_ar_payload_qos,
      input  [2:0] io_readInputs_0_ar_payload_prot,
      output  io_readInputs_0_r_valid,
      input   io_readInputs_0_r_ready,
      output [31:0] io_readInputs_0_r_payload_data,
      output [2:0] io_readInputs_0_r_payload_id,
      output [1:0] io_readInputs_0_r_payload_resp,
      output  io_readInputs_0_r_payload_last,
      input   io_sharedInputs_0_arw_valid,
      output  io_sharedInputs_0_arw_ready,
      input  [31:0] io_sharedInputs_0_arw_payload_addr,
      input  [2:0] io_sharedInputs_0_arw_payload_id,
      input  [3:0] io_sharedInputs_0_arw_payload_region,
      input  [7:0] io_sharedInputs_0_arw_payload_len,
      input  [2:0] io_sharedInputs_0_arw_payload_size,
      input  [1:0] io_sharedInputs_0_arw_payload_burst,
      input  [0:0] io_sharedInputs_0_arw_payload_lock,
      input  [3:0] io_sharedInputs_0_arw_payload_cache,
      input  [3:0] io_sharedInputs_0_arw_payload_qos,
      input  [2:0] io_sharedInputs_0_arw_payload_prot,
      input   io_sharedInputs_0_arw_payload_write,
      input   io_sharedInputs_0_w_valid,
      output  io_sharedInputs_0_w_ready,
      input  [31:0] io_sharedInputs_0_w_payload_data,
      input  [3:0] io_sharedInputs_0_w_payload_strb,
      input   io_sharedInputs_0_w_payload_last,
      output  io_sharedInputs_0_b_valid,
      input   io_sharedInputs_0_b_ready,
      output [2:0] io_sharedInputs_0_b_payload_id,
      output [1:0] io_sharedInputs_0_b_payload_resp,
      output  io_sharedInputs_0_r_valid,
      input   io_sharedInputs_0_r_ready,
      output [31:0] io_sharedInputs_0_r_payload_data,
      output [2:0] io_sharedInputs_0_r_payload_id,
      output [1:0] io_sharedInputs_0_r_payload_resp,
      output  io_sharedInputs_0_r_payload_last,
      output  io_output_arw_valid,
      input   io_output_arw_ready,
      output [31:0] io_output_arw_payload_addr,
      output [3:0] io_output_arw_payload_id,
      output [3:0] io_output_arw_payload_region,
      output [7:0] io_output_arw_payload_len,
      output [2:0] io_output_arw_payload_size,
      output [1:0] io_output_arw_payload_burst,
      output [0:0] io_output_arw_payload_lock,
      output [3:0] io_output_arw_payload_cache,
      output [3:0] io_output_arw_payload_qos,
      output [2:0] io_output_arw_payload_prot,
      output  io_output_arw_payload_write,
      output  io_output_w_valid,
      input   io_output_w_ready,
      output [31:0] io_output_w_payload_data,
      output [3:0] io_output_w_payload_strb,
      output  io_output_w_payload_last,
      input   io_output_b_valid,
      output  io_output_b_ready,
      input  [3:0] io_output_b_payload_id,
      input  [1:0] io_output_b_payload_resp,
      input   io_output_r_valid,
      output  io_output_r_ready,
      input  [31:0] io_output_r_payload_data,
      input  [3:0] io_output_r_payload_id,
      input  [1:0] io_output_r_payload_resp,
      input   io_output_r_payload_last,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg  _zz_Axi4SharedArbiter_1__4_;
  wire  _zz_Axi4SharedArbiter_1__5_;
  wire  _zz_Axi4SharedArbiter_1__6_;
  reg  _zz_Axi4SharedArbiter_1__7_;
  wire  cmdArbiter_io_inputs_0_ready;
  wire  cmdArbiter_io_inputs_1_ready;
  wire  cmdArbiter_io_output_valid;
  wire [31:0] cmdArbiter_io_output_payload_addr;
  wire [2:0] cmdArbiter_io_output_payload_id;
  wire [3:0] cmdArbiter_io_output_payload_region;
  wire [7:0] cmdArbiter_io_output_payload_len;
  wire [2:0] cmdArbiter_io_output_payload_size;
  wire [1:0] cmdArbiter_io_output_payload_burst;
  wire [0:0] cmdArbiter_io_output_payload_lock;
  wire [3:0] cmdArbiter_io_output_payload_cache;
  wire [3:0] cmdArbiter_io_output_payload_qos;
  wire [2:0] cmdArbiter_io_output_payload_prot;
  wire  cmdArbiter_io_output_payload_write;
  wire [0:0] cmdArbiter_io_chosen;
  wire [1:0] cmdArbiter_io_chosenOH;
  wire  streamFork_5__io_input_ready;
  wire  streamFork_5__io_outputs_0_valid;
  wire [31:0] streamFork_5__io_outputs_0_payload_addr;
  wire [2:0] streamFork_5__io_outputs_0_payload_id;
  wire [3:0] streamFork_5__io_outputs_0_payload_region;
  wire [7:0] streamFork_5__io_outputs_0_payload_len;
  wire [2:0] streamFork_5__io_outputs_0_payload_size;
  wire [1:0] streamFork_5__io_outputs_0_payload_burst;
  wire [0:0] streamFork_5__io_outputs_0_payload_lock;
  wire [3:0] streamFork_5__io_outputs_0_payload_cache;
  wire [3:0] streamFork_5__io_outputs_0_payload_qos;
  wire [2:0] streamFork_5__io_outputs_0_payload_prot;
  wire  streamFork_5__io_outputs_0_payload_write;
  wire  streamFork_5__io_outputs_1_valid;
  wire [31:0] streamFork_5__io_outputs_1_payload_addr;
  wire [2:0] streamFork_5__io_outputs_1_payload_id;
  wire [3:0] streamFork_5__io_outputs_1_payload_region;
  wire [7:0] streamFork_5__io_outputs_1_payload_len;
  wire [2:0] streamFork_5__io_outputs_1_payload_size;
  wire [1:0] streamFork_5__io_outputs_1_payload_burst;
  wire [0:0] streamFork_5__io_outputs_1_payload_lock;
  wire [3:0] streamFork_5__io_outputs_1_payload_cache;
  wire [3:0] streamFork_5__io_outputs_1_payload_qos;
  wire [2:0] streamFork_5__io_outputs_1_payload_prot;
  wire  streamFork_5__io_outputs_1_payload_write;
  wire  streamFifoLowLatency_5__io_push_ready;
  wire  streamFifoLowLatency_5__io_pop_valid;
  wire [2:0] streamFifoLowLatency_5__io_occupancy;
  wire [1:0] _zz_Axi4SharedArbiter_1__8_;
  wire [2:0] _zz_Axi4SharedArbiter_1__9_;
  wire [3:0] _zz_Axi4SharedArbiter_1__10_;
  wire  inputsCmd_0_valid;
  wire  inputsCmd_0_ready;
  wire [31:0] inputsCmd_0_payload_addr;
  wire [2:0] inputsCmd_0_payload_id;
  wire [3:0] inputsCmd_0_payload_region;
  wire [7:0] inputsCmd_0_payload_len;
  wire [2:0] inputsCmd_0_payload_size;
  wire [1:0] inputsCmd_0_payload_burst;
  wire [0:0] inputsCmd_0_payload_lock;
  wire [3:0] inputsCmd_0_payload_cache;
  wire [3:0] inputsCmd_0_payload_qos;
  wire [2:0] inputsCmd_0_payload_prot;
  wire  inputsCmd_0_payload_write;
  wire  inputsCmd_1_valid;
  wire  inputsCmd_1_ready;
  wire [31:0] inputsCmd_1_payload_addr;
  wire [2:0] inputsCmd_1_payload_id;
  wire [3:0] inputsCmd_1_payload_region;
  wire [7:0] inputsCmd_1_payload_len;
  wire [2:0] inputsCmd_1_payload_size;
  wire [1:0] inputsCmd_1_payload_burst;
  wire [0:0] inputsCmd_1_payload_lock;
  wire [3:0] inputsCmd_1_payload_cache;
  wire [3:0] inputsCmd_1_payload_qos;
  wire [2:0] inputsCmd_1_payload_prot;
  wire  inputsCmd_1_payload_write;
  wire  _zz_Axi4SharedArbiter_1__1_;
  reg  _zz_Axi4SharedArbiter_1__2_;
  wire  _zz_Axi4SharedArbiter_1__3_;
  wire  routeDataInput_valid;
  wire  routeDataInput_ready;
  wire [31:0] routeDataInput_payload_data;
  wire [3:0] routeDataInput_payload_strb;
  wire  routeDataInput_payload_last;
  wire  writeRspSels_0;
  wire [0:0] readRspIndex;
  wire  readRspSels_0;
  wire  readRspSels_1;
  assign _zz_Axi4SharedArbiter_1__8_ = {cmdArbiter_io_chosenOH[1 : 1],cmdArbiter_io_chosenOH[0 : 0]};
  assign _zz_Axi4SharedArbiter_1__9_ = streamFork_5__io_outputs_0_payload_id;
  assign _zz_Axi4SharedArbiter_1__10_ = {1'd0, _zz_Axi4SharedArbiter_1__9_};
  StreamArbiter_1_ cmdArbiter ( 
    .io_inputs_0_valid(inputsCmd_0_valid),
    .io_inputs_0_ready(cmdArbiter_io_inputs_0_ready),
    .io_inputs_0_payload_addr(inputsCmd_0_payload_addr),
    .io_inputs_0_payload_id(inputsCmd_0_payload_id),
    .io_inputs_0_payload_region(inputsCmd_0_payload_region),
    .io_inputs_0_payload_len(inputsCmd_0_payload_len),
    .io_inputs_0_payload_size(inputsCmd_0_payload_size),
    .io_inputs_0_payload_burst(inputsCmd_0_payload_burst),
    .io_inputs_0_payload_lock(inputsCmd_0_payload_lock),
    .io_inputs_0_payload_cache(inputsCmd_0_payload_cache),
    .io_inputs_0_payload_qos(inputsCmd_0_payload_qos),
    .io_inputs_0_payload_prot(inputsCmd_0_payload_prot),
    .io_inputs_0_payload_write(inputsCmd_0_payload_write),
    .io_inputs_1_valid(inputsCmd_1_valid),
    .io_inputs_1_ready(cmdArbiter_io_inputs_1_ready),
    .io_inputs_1_payload_addr(inputsCmd_1_payload_addr),
    .io_inputs_1_payload_id(inputsCmd_1_payload_id),
    .io_inputs_1_payload_region(inputsCmd_1_payload_region),
    .io_inputs_1_payload_len(inputsCmd_1_payload_len),
    .io_inputs_1_payload_size(inputsCmd_1_payload_size),
    .io_inputs_1_payload_burst(inputsCmd_1_payload_burst),
    .io_inputs_1_payload_lock(inputsCmd_1_payload_lock),
    .io_inputs_1_payload_cache(inputsCmd_1_payload_cache),
    .io_inputs_1_payload_qos(inputsCmd_1_payload_qos),
    .io_inputs_1_payload_prot(inputsCmd_1_payload_prot),
    .io_inputs_1_payload_write(inputsCmd_1_payload_write),
    .io_output_valid(cmdArbiter_io_output_valid),
    .io_output_ready(streamFork_5__io_input_ready),
    .io_output_payload_addr(cmdArbiter_io_output_payload_addr),
    .io_output_payload_id(cmdArbiter_io_output_payload_id),
    .io_output_payload_region(cmdArbiter_io_output_payload_region),
    .io_output_payload_len(cmdArbiter_io_output_payload_len),
    .io_output_payload_size(cmdArbiter_io_output_payload_size),
    .io_output_payload_burst(cmdArbiter_io_output_payload_burst),
    .io_output_payload_lock(cmdArbiter_io_output_payload_lock),
    .io_output_payload_cache(cmdArbiter_io_output_payload_cache),
    .io_output_payload_qos(cmdArbiter_io_output_payload_qos),
    .io_output_payload_prot(cmdArbiter_io_output_payload_prot),
    .io_output_payload_write(cmdArbiter_io_output_payload_write),
    .io_chosen(cmdArbiter_io_chosen),
    .io_chosenOH(cmdArbiter_io_chosenOH),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  StreamFork streamFork_5_ ( 
    .io_input_valid(cmdArbiter_io_output_valid),
    .io_input_ready(streamFork_5__io_input_ready),
    .io_input_payload_addr(cmdArbiter_io_output_payload_addr),
    .io_input_payload_id(cmdArbiter_io_output_payload_id),
    .io_input_payload_region(cmdArbiter_io_output_payload_region),
    .io_input_payload_len(cmdArbiter_io_output_payload_len),
    .io_input_payload_size(cmdArbiter_io_output_payload_size),
    .io_input_payload_burst(cmdArbiter_io_output_payload_burst),
    .io_input_payload_lock(cmdArbiter_io_output_payload_lock),
    .io_input_payload_cache(cmdArbiter_io_output_payload_cache),
    .io_input_payload_qos(cmdArbiter_io_output_payload_qos),
    .io_input_payload_prot(cmdArbiter_io_output_payload_prot),
    .io_input_payload_write(cmdArbiter_io_output_payload_write),
    .io_outputs_0_valid(streamFork_5__io_outputs_0_valid),
    .io_outputs_0_ready(io_output_arw_ready),
    .io_outputs_0_payload_addr(streamFork_5__io_outputs_0_payload_addr),
    .io_outputs_0_payload_id(streamFork_5__io_outputs_0_payload_id),
    .io_outputs_0_payload_region(streamFork_5__io_outputs_0_payload_region),
    .io_outputs_0_payload_len(streamFork_5__io_outputs_0_payload_len),
    .io_outputs_0_payload_size(streamFork_5__io_outputs_0_payload_size),
    .io_outputs_0_payload_burst(streamFork_5__io_outputs_0_payload_burst),
    .io_outputs_0_payload_lock(streamFork_5__io_outputs_0_payload_lock),
    .io_outputs_0_payload_cache(streamFork_5__io_outputs_0_payload_cache),
    .io_outputs_0_payload_qos(streamFork_5__io_outputs_0_payload_qos),
    .io_outputs_0_payload_prot(streamFork_5__io_outputs_0_payload_prot),
    .io_outputs_0_payload_write(streamFork_5__io_outputs_0_payload_write),
    .io_outputs_1_valid(streamFork_5__io_outputs_1_valid),
    .io_outputs_1_ready(_zz_Axi4SharedArbiter_1__4_),
    .io_outputs_1_payload_addr(streamFork_5__io_outputs_1_payload_addr),
    .io_outputs_1_payload_id(streamFork_5__io_outputs_1_payload_id),
    .io_outputs_1_payload_region(streamFork_5__io_outputs_1_payload_region),
    .io_outputs_1_payload_len(streamFork_5__io_outputs_1_payload_len),
    .io_outputs_1_payload_size(streamFork_5__io_outputs_1_payload_size),
    .io_outputs_1_payload_burst(streamFork_5__io_outputs_1_payload_burst),
    .io_outputs_1_payload_lock(streamFork_5__io_outputs_1_payload_lock),
    .io_outputs_1_payload_cache(streamFork_5__io_outputs_1_payload_cache),
    .io_outputs_1_payload_qos(streamFork_5__io_outputs_1_payload_qos),
    .io_outputs_1_payload_prot(streamFork_5__io_outputs_1_payload_prot),
    .io_outputs_1_payload_write(streamFork_5__io_outputs_1_payload_write),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  StreamFifoLowLatency_2_ streamFifoLowLatency_5_ ( 
    .io_push_valid(_zz_Axi4SharedArbiter_1__2_),
    .io_push_ready(streamFifoLowLatency_5__io_push_ready),
    .io_pop_valid(streamFifoLowLatency_5__io_pop_valid),
    .io_pop_ready(_zz_Axi4SharedArbiter_1__5_),
    .io_flush(_zz_Axi4SharedArbiter_1__6_),
    .io_occupancy(streamFifoLowLatency_5__io_occupancy),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  always @(*) begin
    case(readRspIndex)
      1'b0 : begin
        _zz_Axi4SharedArbiter_1__7_ = io_readInputs_0_r_ready;
      end
      default : begin
        _zz_Axi4SharedArbiter_1__7_ = io_sharedInputs_0_r_ready;
      end
    endcase
  end

  assign inputsCmd_0_valid = io_readInputs_0_ar_valid;
  assign io_readInputs_0_ar_ready = inputsCmd_0_ready;
  assign inputsCmd_0_payload_addr = io_readInputs_0_ar_payload_addr;
  assign inputsCmd_0_payload_id = io_readInputs_0_ar_payload_id;
  assign inputsCmd_0_payload_region = io_readInputs_0_ar_payload_region;
  assign inputsCmd_0_payload_len = io_readInputs_0_ar_payload_len;
  assign inputsCmd_0_payload_size = io_readInputs_0_ar_payload_size;
  assign inputsCmd_0_payload_burst = io_readInputs_0_ar_payload_burst;
  assign inputsCmd_0_payload_lock = io_readInputs_0_ar_payload_lock;
  assign inputsCmd_0_payload_cache = io_readInputs_0_ar_payload_cache;
  assign inputsCmd_0_payload_qos = io_readInputs_0_ar_payload_qos;
  assign inputsCmd_0_payload_prot = io_readInputs_0_ar_payload_prot;
  assign inputsCmd_0_payload_write = 1'b0;
  assign inputsCmd_1_valid = io_sharedInputs_0_arw_valid;
  assign io_sharedInputs_0_arw_ready = inputsCmd_1_ready;
  assign inputsCmd_1_payload_addr = io_sharedInputs_0_arw_payload_addr;
  assign inputsCmd_1_payload_id = io_sharedInputs_0_arw_payload_id;
  assign inputsCmd_1_payload_region = io_sharedInputs_0_arw_payload_region;
  assign inputsCmd_1_payload_len = io_sharedInputs_0_arw_payload_len;
  assign inputsCmd_1_payload_size = io_sharedInputs_0_arw_payload_size;
  assign inputsCmd_1_payload_burst = io_sharedInputs_0_arw_payload_burst;
  assign inputsCmd_1_payload_lock = io_sharedInputs_0_arw_payload_lock;
  assign inputsCmd_1_payload_cache = io_sharedInputs_0_arw_payload_cache;
  assign inputsCmd_1_payload_qos = io_sharedInputs_0_arw_payload_qos;
  assign inputsCmd_1_payload_prot = io_sharedInputs_0_arw_payload_prot;
  assign inputsCmd_1_payload_write = io_sharedInputs_0_arw_payload_write;
  assign inputsCmd_0_ready = cmdArbiter_io_inputs_0_ready;
  assign inputsCmd_1_ready = cmdArbiter_io_inputs_1_ready;
  assign io_output_arw_valid = streamFork_5__io_outputs_0_valid;
  assign io_output_arw_payload_addr = streamFork_5__io_outputs_0_payload_addr;
  assign io_output_arw_payload_region = streamFork_5__io_outputs_0_payload_region;
  assign io_output_arw_payload_len = streamFork_5__io_outputs_0_payload_len;
  assign io_output_arw_payload_size = streamFork_5__io_outputs_0_payload_size;
  assign io_output_arw_payload_burst = streamFork_5__io_outputs_0_payload_burst;
  assign io_output_arw_payload_lock = streamFork_5__io_outputs_0_payload_lock;
  assign io_output_arw_payload_cache = streamFork_5__io_outputs_0_payload_cache;
  assign io_output_arw_payload_qos = streamFork_5__io_outputs_0_payload_qos;
  assign io_output_arw_payload_prot = streamFork_5__io_outputs_0_payload_prot;
  assign io_output_arw_payload_write = streamFork_5__io_outputs_0_payload_write;
  assign _zz_Axi4SharedArbiter_1__1_ = _zz_Axi4SharedArbiter_1__8_[1];
  assign io_output_arw_payload_id = (streamFork_5__io_outputs_0_payload_write ? _zz_Axi4SharedArbiter_1__10_ : {_zz_Axi4SharedArbiter_1__1_,streamFork_5__io_outputs_0_payload_id});
  always @ (*) begin
    _zz_Axi4SharedArbiter_1__2_ = streamFork_5__io_outputs_1_valid;
    _zz_Axi4SharedArbiter_1__4_ = _zz_Axi4SharedArbiter_1__3_;
    if((! streamFork_5__io_outputs_1_payload_write))begin
      _zz_Axi4SharedArbiter_1__2_ = 1'b0;
      _zz_Axi4SharedArbiter_1__4_ = 1'b1;
    end
  end

  assign _zz_Axi4SharedArbiter_1__3_ = streamFifoLowLatency_5__io_push_ready;
  assign routeDataInput_valid = io_sharedInputs_0_w_valid;
  assign routeDataInput_ready = io_sharedInputs_0_w_ready;
  assign routeDataInput_payload_data = io_sharedInputs_0_w_payload_data;
  assign routeDataInput_payload_strb = io_sharedInputs_0_w_payload_strb;
  assign routeDataInput_payload_last = io_sharedInputs_0_w_payload_last;
  assign io_output_w_valid = (streamFifoLowLatency_5__io_pop_valid && routeDataInput_valid);
  assign io_output_w_payload_data = routeDataInput_payload_data;
  assign io_output_w_payload_strb = routeDataInput_payload_strb;
  assign io_output_w_payload_last = routeDataInput_payload_last;
  assign io_sharedInputs_0_w_ready = ((streamFifoLowLatency_5__io_pop_valid && io_output_w_ready) && 1'b1);
  assign _zz_Axi4SharedArbiter_1__5_ = ((io_output_w_valid && io_output_w_ready) && io_output_w_payload_last);
  assign writeRspSels_0 = 1'b1;
  assign io_sharedInputs_0_b_valid = (io_output_b_valid && writeRspSels_0);
  assign io_sharedInputs_0_b_payload_resp = io_output_b_payload_resp;
  assign io_sharedInputs_0_b_payload_id = io_output_b_payload_id[2:0];
  assign io_output_b_ready = io_sharedInputs_0_b_ready;
  assign readRspIndex = io_output_r_payload_id[3 : 3];
  assign readRspSels_0 = (readRspIndex == (1'b0));
  assign readRspSels_1 = (readRspIndex == (1'b1));
  assign io_readInputs_0_r_valid = (io_output_r_valid && readRspSels_0);
  assign io_readInputs_0_r_payload_data = io_output_r_payload_data;
  assign io_readInputs_0_r_payload_resp = io_output_r_payload_resp;
  assign io_readInputs_0_r_payload_last = io_output_r_payload_last;
  assign io_readInputs_0_r_payload_id = io_output_r_payload_id[2:0];
  assign io_sharedInputs_0_r_valid = (io_output_r_valid && readRspSels_1);
  assign io_sharedInputs_0_r_payload_data = io_output_r_payload_data;
  assign io_sharedInputs_0_r_payload_resp = io_output_r_payload_resp;
  assign io_sharedInputs_0_r_payload_last = io_output_r_payload_last;
  assign io_sharedInputs_0_r_payload_id = io_output_r_payload_id[2:0];
  assign io_output_r_ready = _zz_Axi4SharedArbiter_1__7_;
  assign _zz_Axi4SharedArbiter_1__6_ = 1'b0;
endmodule

module Axi4SharedArbiter_2_ (
      input   io_readInputs_0_ar_valid,
      output  io_readInputs_0_ar_ready,
      input  [12:0] io_readInputs_0_ar_payload_addr,
      input  [2:0] io_readInputs_0_ar_payload_id,
      input  [7:0] io_readInputs_0_ar_payload_len,
      input  [2:0] io_readInputs_0_ar_payload_size,
      input  [1:0] io_readInputs_0_ar_payload_burst,
      output  io_readInputs_0_r_valid,
      input   io_readInputs_0_r_ready,
      output [31:0] io_readInputs_0_r_payload_data,
      output [2:0] io_readInputs_0_r_payload_id,
      output [1:0] io_readInputs_0_r_payload_resp,
      output  io_readInputs_0_r_payload_last,
      input   io_sharedInputs_0_arw_valid,
      output  io_sharedInputs_0_arw_ready,
      input  [12:0] io_sharedInputs_0_arw_payload_addr,
      input  [2:0] io_sharedInputs_0_arw_payload_id,
      input  [7:0] io_sharedInputs_0_arw_payload_len,
      input  [2:0] io_sharedInputs_0_arw_payload_size,
      input  [1:0] io_sharedInputs_0_arw_payload_burst,
      input   io_sharedInputs_0_arw_payload_write,
      input   io_sharedInputs_0_w_valid,
      output  io_sharedInputs_0_w_ready,
      input  [31:0] io_sharedInputs_0_w_payload_data,
      input  [3:0] io_sharedInputs_0_w_payload_strb,
      input   io_sharedInputs_0_w_payload_last,
      output  io_sharedInputs_0_b_valid,
      input   io_sharedInputs_0_b_ready,
      output [2:0] io_sharedInputs_0_b_payload_id,
      output [1:0] io_sharedInputs_0_b_payload_resp,
      output  io_sharedInputs_0_r_valid,
      input   io_sharedInputs_0_r_ready,
      output [31:0] io_sharedInputs_0_r_payload_data,
      output [2:0] io_sharedInputs_0_r_payload_id,
      output [1:0] io_sharedInputs_0_r_payload_resp,
      output  io_sharedInputs_0_r_payload_last,
      output  io_output_arw_valid,
      input   io_output_arw_ready,
      output [12:0] io_output_arw_payload_addr,
      output [3:0] io_output_arw_payload_id,
      output [7:0] io_output_arw_payload_len,
      output [2:0] io_output_arw_payload_size,
      output [1:0] io_output_arw_payload_burst,
      output  io_output_arw_payload_write,
      output  io_output_w_valid,
      input   io_output_w_ready,
      output [31:0] io_output_w_payload_data,
      output [3:0] io_output_w_payload_strb,
      output  io_output_w_payload_last,
      input   io_output_b_valid,
      output  io_output_b_ready,
      input  [3:0] io_output_b_payload_id,
      input  [1:0] io_output_b_payload_resp,
      input   io_output_r_valid,
      output  io_output_r_ready,
      input  [31:0] io_output_r_payload_data,
      input  [3:0] io_output_r_payload_id,
      input  [1:0] io_output_r_payload_resp,
      input   io_output_r_payload_last,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg  _zz_Axi4SharedArbiter_2__4_;
  wire  _zz_Axi4SharedArbiter_2__5_;
  wire  _zz_Axi4SharedArbiter_2__6_;
  reg  _zz_Axi4SharedArbiter_2__7_;
  wire  cmdArbiter_io_inputs_0_ready;
  wire  cmdArbiter_io_inputs_1_ready;
  wire  cmdArbiter_io_output_valid;
  wire [12:0] cmdArbiter_io_output_payload_addr;
  wire [2:0] cmdArbiter_io_output_payload_id;
  wire [7:0] cmdArbiter_io_output_payload_len;
  wire [2:0] cmdArbiter_io_output_payload_size;
  wire [1:0] cmdArbiter_io_output_payload_burst;
  wire  cmdArbiter_io_output_payload_write;
  wire [0:0] cmdArbiter_io_chosen;
  wire [1:0] cmdArbiter_io_chosenOH;
  wire  streamFork_5__io_input_ready;
  wire  streamFork_5__io_outputs_0_valid;
  wire [12:0] streamFork_5__io_outputs_0_payload_addr;
  wire [2:0] streamFork_5__io_outputs_0_payload_id;
  wire [7:0] streamFork_5__io_outputs_0_payload_len;
  wire [2:0] streamFork_5__io_outputs_0_payload_size;
  wire [1:0] streamFork_5__io_outputs_0_payload_burst;
  wire  streamFork_5__io_outputs_0_payload_write;
  wire  streamFork_5__io_outputs_1_valid;
  wire [12:0] streamFork_5__io_outputs_1_payload_addr;
  wire [2:0] streamFork_5__io_outputs_1_payload_id;
  wire [7:0] streamFork_5__io_outputs_1_payload_len;
  wire [2:0] streamFork_5__io_outputs_1_payload_size;
  wire [1:0] streamFork_5__io_outputs_1_payload_burst;
  wire  streamFork_5__io_outputs_1_payload_write;
  wire  streamFifoLowLatency_5__io_push_ready;
  wire  streamFifoLowLatency_5__io_pop_valid;
  wire [2:0] streamFifoLowLatency_5__io_occupancy;
  wire [1:0] _zz_Axi4SharedArbiter_2__8_;
  wire [2:0] _zz_Axi4SharedArbiter_2__9_;
  wire [3:0] _zz_Axi4SharedArbiter_2__10_;
  wire  inputsCmd_0_valid;
  wire  inputsCmd_0_ready;
  wire [12:0] inputsCmd_0_payload_addr;
  wire [2:0] inputsCmd_0_payload_id;
  wire [7:0] inputsCmd_0_payload_len;
  wire [2:0] inputsCmd_0_payload_size;
  wire [1:0] inputsCmd_0_payload_burst;
  wire  inputsCmd_0_payload_write;
  wire  inputsCmd_1_valid;
  wire  inputsCmd_1_ready;
  wire [12:0] inputsCmd_1_payload_addr;
  wire [2:0] inputsCmd_1_payload_id;
  wire [7:0] inputsCmd_1_payload_len;
  wire [2:0] inputsCmd_1_payload_size;
  wire [1:0] inputsCmd_1_payload_burst;
  wire  inputsCmd_1_payload_write;
  wire  _zz_Axi4SharedArbiter_2__1_;
  reg  _zz_Axi4SharedArbiter_2__2_;
  wire  _zz_Axi4SharedArbiter_2__3_;
  wire  routeDataInput_valid;
  wire  routeDataInput_ready;
  wire [31:0] routeDataInput_payload_data;
  wire [3:0] routeDataInput_payload_strb;
  wire  routeDataInput_payload_last;
  wire  writeRspSels_0;
  wire [0:0] readRspIndex;
  wire  readRspSels_0;
  wire  readRspSels_1;
  assign _zz_Axi4SharedArbiter_2__8_ = {cmdArbiter_io_chosenOH[1 : 1],cmdArbiter_io_chosenOH[0 : 0]};
  assign _zz_Axi4SharedArbiter_2__9_ = streamFork_5__io_outputs_0_payload_id;
  assign _zz_Axi4SharedArbiter_2__10_ = {1'd0, _zz_Axi4SharedArbiter_2__9_};
  StreamArbiter_2_ cmdArbiter ( 
    .io_inputs_0_valid(inputsCmd_0_valid),
    .io_inputs_0_ready(cmdArbiter_io_inputs_0_ready),
    .io_inputs_0_payload_addr(inputsCmd_0_payload_addr),
    .io_inputs_0_payload_id(inputsCmd_0_payload_id),
    .io_inputs_0_payload_len(inputsCmd_0_payload_len),
    .io_inputs_0_payload_size(inputsCmd_0_payload_size),
    .io_inputs_0_payload_burst(inputsCmd_0_payload_burst),
    .io_inputs_0_payload_write(inputsCmd_0_payload_write),
    .io_inputs_1_valid(inputsCmd_1_valid),
    .io_inputs_1_ready(cmdArbiter_io_inputs_1_ready),
    .io_inputs_1_payload_addr(inputsCmd_1_payload_addr),
    .io_inputs_1_payload_id(inputsCmd_1_payload_id),
    .io_inputs_1_payload_len(inputsCmd_1_payload_len),
    .io_inputs_1_payload_size(inputsCmd_1_payload_size),
    .io_inputs_1_payload_burst(inputsCmd_1_payload_burst),
    .io_inputs_1_payload_write(inputsCmd_1_payload_write),
    .io_output_valid(cmdArbiter_io_output_valid),
    .io_output_ready(streamFork_5__io_input_ready),
    .io_output_payload_addr(cmdArbiter_io_output_payload_addr),
    .io_output_payload_id(cmdArbiter_io_output_payload_id),
    .io_output_payload_len(cmdArbiter_io_output_payload_len),
    .io_output_payload_size(cmdArbiter_io_output_payload_size),
    .io_output_payload_burst(cmdArbiter_io_output_payload_burst),
    .io_output_payload_write(cmdArbiter_io_output_payload_write),
    .io_chosen(cmdArbiter_io_chosen),
    .io_chosenOH(cmdArbiter_io_chosenOH),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  StreamFork_2_ streamFork_5_ ( 
    .io_input_valid(cmdArbiter_io_output_valid),
    .io_input_ready(streamFork_5__io_input_ready),
    .io_input_payload_addr(cmdArbiter_io_output_payload_addr),
    .io_input_payload_id(cmdArbiter_io_output_payload_id),
    .io_input_payload_len(cmdArbiter_io_output_payload_len),
    .io_input_payload_size(cmdArbiter_io_output_payload_size),
    .io_input_payload_burst(cmdArbiter_io_output_payload_burst),
    .io_input_payload_write(cmdArbiter_io_output_payload_write),
    .io_outputs_0_valid(streamFork_5__io_outputs_0_valid),
    .io_outputs_0_ready(io_output_arw_ready),
    .io_outputs_0_payload_addr(streamFork_5__io_outputs_0_payload_addr),
    .io_outputs_0_payload_id(streamFork_5__io_outputs_0_payload_id),
    .io_outputs_0_payload_len(streamFork_5__io_outputs_0_payload_len),
    .io_outputs_0_payload_size(streamFork_5__io_outputs_0_payload_size),
    .io_outputs_0_payload_burst(streamFork_5__io_outputs_0_payload_burst),
    .io_outputs_0_payload_write(streamFork_5__io_outputs_0_payload_write),
    .io_outputs_1_valid(streamFork_5__io_outputs_1_valid),
    .io_outputs_1_ready(_zz_Axi4SharedArbiter_2__4_),
    .io_outputs_1_payload_addr(streamFork_5__io_outputs_1_payload_addr),
    .io_outputs_1_payload_id(streamFork_5__io_outputs_1_payload_id),
    .io_outputs_1_payload_len(streamFork_5__io_outputs_1_payload_len),
    .io_outputs_1_payload_size(streamFork_5__io_outputs_1_payload_size),
    .io_outputs_1_payload_burst(streamFork_5__io_outputs_1_payload_burst),
    .io_outputs_1_payload_write(streamFork_5__io_outputs_1_payload_write),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  StreamFifoLowLatency_3_ streamFifoLowLatency_5_ ( 
    .io_push_valid(_zz_Axi4SharedArbiter_2__2_),
    .io_push_ready(streamFifoLowLatency_5__io_push_ready),
    .io_pop_valid(streamFifoLowLatency_5__io_pop_valid),
    .io_pop_ready(_zz_Axi4SharedArbiter_2__5_),
    .io_flush(_zz_Axi4SharedArbiter_2__6_),
    .io_occupancy(streamFifoLowLatency_5__io_occupancy),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  always @(*) begin
    case(readRspIndex)
      1'b0 : begin
        _zz_Axi4SharedArbiter_2__7_ = io_readInputs_0_r_ready;
      end
      default : begin
        _zz_Axi4SharedArbiter_2__7_ = io_sharedInputs_0_r_ready;
      end
    endcase
  end

  assign inputsCmd_0_valid = io_readInputs_0_ar_valid;
  assign io_readInputs_0_ar_ready = inputsCmd_0_ready;
  assign inputsCmd_0_payload_addr = io_readInputs_0_ar_payload_addr;
  assign inputsCmd_0_payload_id = io_readInputs_0_ar_payload_id;
  assign inputsCmd_0_payload_len = io_readInputs_0_ar_payload_len;
  assign inputsCmd_0_payload_size = io_readInputs_0_ar_payload_size;
  assign inputsCmd_0_payload_burst = io_readInputs_0_ar_payload_burst;
  assign inputsCmd_0_payload_write = 1'b0;
  assign inputsCmd_1_valid = io_sharedInputs_0_arw_valid;
  assign io_sharedInputs_0_arw_ready = inputsCmd_1_ready;
  assign inputsCmd_1_payload_addr = io_sharedInputs_0_arw_payload_addr;
  assign inputsCmd_1_payload_id = io_sharedInputs_0_arw_payload_id;
  assign inputsCmd_1_payload_len = io_sharedInputs_0_arw_payload_len;
  assign inputsCmd_1_payload_size = io_sharedInputs_0_arw_payload_size;
  assign inputsCmd_1_payload_burst = io_sharedInputs_0_arw_payload_burst;
  assign inputsCmd_1_payload_write = io_sharedInputs_0_arw_payload_write;
  assign inputsCmd_0_ready = cmdArbiter_io_inputs_0_ready;
  assign inputsCmd_1_ready = cmdArbiter_io_inputs_1_ready;
  assign io_output_arw_valid = streamFork_5__io_outputs_0_valid;
  assign io_output_arw_payload_addr = streamFork_5__io_outputs_0_payload_addr;
  assign io_output_arw_payload_len = streamFork_5__io_outputs_0_payload_len;
  assign io_output_arw_payload_size = streamFork_5__io_outputs_0_payload_size;
  assign io_output_arw_payload_burst = streamFork_5__io_outputs_0_payload_burst;
  assign io_output_arw_payload_write = streamFork_5__io_outputs_0_payload_write;
  assign _zz_Axi4SharedArbiter_2__1_ = _zz_Axi4SharedArbiter_2__8_[1];
  assign io_output_arw_payload_id = (streamFork_5__io_outputs_0_payload_write ? _zz_Axi4SharedArbiter_2__10_ : {_zz_Axi4SharedArbiter_2__1_,streamFork_5__io_outputs_0_payload_id});
  always @ (*) begin
    _zz_Axi4SharedArbiter_2__2_ = streamFork_5__io_outputs_1_valid;
    _zz_Axi4SharedArbiter_2__4_ = _zz_Axi4SharedArbiter_2__3_;
    if((! streamFork_5__io_outputs_1_payload_write))begin
      _zz_Axi4SharedArbiter_2__2_ = 1'b0;
      _zz_Axi4SharedArbiter_2__4_ = 1'b1;
    end
  end

  assign _zz_Axi4SharedArbiter_2__3_ = streamFifoLowLatency_5__io_push_ready;
  assign routeDataInput_valid = io_sharedInputs_0_w_valid;
  assign routeDataInput_ready = io_sharedInputs_0_w_ready;
  assign routeDataInput_payload_data = io_sharedInputs_0_w_payload_data;
  assign routeDataInput_payload_strb = io_sharedInputs_0_w_payload_strb;
  assign routeDataInput_payload_last = io_sharedInputs_0_w_payload_last;
  assign io_output_w_valid = (streamFifoLowLatency_5__io_pop_valid && routeDataInput_valid);
  assign io_output_w_payload_data = routeDataInput_payload_data;
  assign io_output_w_payload_strb = routeDataInput_payload_strb;
  assign io_output_w_payload_last = routeDataInput_payload_last;
  assign io_sharedInputs_0_w_ready = ((streamFifoLowLatency_5__io_pop_valid && io_output_w_ready) && 1'b1);
  assign _zz_Axi4SharedArbiter_2__5_ = ((io_output_w_valid && io_output_w_ready) && io_output_w_payload_last);
  assign writeRspSels_0 = 1'b1;
  assign io_sharedInputs_0_b_valid = (io_output_b_valid && writeRspSels_0);
  assign io_sharedInputs_0_b_payload_resp = io_output_b_payload_resp;
  assign io_sharedInputs_0_b_payload_id = io_output_b_payload_id[2:0];
  assign io_output_b_ready = io_sharedInputs_0_b_ready;
  assign readRspIndex = io_output_r_payload_id[3 : 3];
  assign readRspSels_0 = (readRspIndex == (1'b0));
  assign readRspSels_1 = (readRspIndex == (1'b1));
  assign io_readInputs_0_r_valid = (io_output_r_valid && readRspSels_0);
  assign io_readInputs_0_r_payload_data = io_output_r_payload_data;
  assign io_readInputs_0_r_payload_resp = io_output_r_payload_resp;
  assign io_readInputs_0_r_payload_last = io_output_r_payload_last;
  assign io_readInputs_0_r_payload_id = io_output_r_payload_id[2:0];
  assign io_sharedInputs_0_r_valid = (io_output_r_valid && readRspSels_1);
  assign io_sharedInputs_0_r_payload_data = io_output_r_payload_data;
  assign io_sharedInputs_0_r_payload_resp = io_output_r_payload_resp;
  assign io_sharedInputs_0_r_payload_last = io_output_r_payload_last;
  assign io_sharedInputs_0_r_payload_id = io_output_r_payload_id[2:0];
  assign io_output_r_ready = _zz_Axi4SharedArbiter_2__7_;
  assign _zz_Axi4SharedArbiter_2__6_ = 1'b0;
endmodule

module Axi4SharedArbiter_3_ (
      input   io_sharedInputs_0_arw_valid,
      output  io_sharedInputs_0_arw_ready,
      input  [19:0] io_sharedInputs_0_arw_payload_addr,
      input  [3:0] io_sharedInputs_0_arw_payload_id,
      input  [7:0] io_sharedInputs_0_arw_payload_len,
      input  [2:0] io_sharedInputs_0_arw_payload_size,
      input  [1:0] io_sharedInputs_0_arw_payload_burst,
      input   io_sharedInputs_0_arw_payload_write,
      input   io_sharedInputs_0_w_valid,
      output  io_sharedInputs_0_w_ready,
      input  [31:0] io_sharedInputs_0_w_payload_data,
      input  [3:0] io_sharedInputs_0_w_payload_strb,
      input   io_sharedInputs_0_w_payload_last,
      output  io_sharedInputs_0_b_valid,
      input   io_sharedInputs_0_b_ready,
      output [3:0] io_sharedInputs_0_b_payload_id,
      output [1:0] io_sharedInputs_0_b_payload_resp,
      output  io_sharedInputs_0_r_valid,
      input   io_sharedInputs_0_r_ready,
      output [31:0] io_sharedInputs_0_r_payload_data,
      output [3:0] io_sharedInputs_0_r_payload_id,
      output [1:0] io_sharedInputs_0_r_payload_resp,
      output  io_sharedInputs_0_r_payload_last,
      output  io_output_arw_valid,
      input   io_output_arw_ready,
      output [19:0] io_output_arw_payload_addr,
      output [3:0] io_output_arw_payload_id,
      output [7:0] io_output_arw_payload_len,
      output [2:0] io_output_arw_payload_size,
      output [1:0] io_output_arw_payload_burst,
      output  io_output_arw_payload_write,
      output  io_output_w_valid,
      input   io_output_w_ready,
      output [31:0] io_output_w_payload_data,
      output [3:0] io_output_w_payload_strb,
      output  io_output_w_payload_last,
      input   io_output_b_valid,
      output  io_output_b_ready,
      input  [3:0] io_output_b_payload_id,
      input  [1:0] io_output_b_payload_resp,
      input   io_output_r_valid,
      output  io_output_r_ready,
      input  [31:0] io_output_r_payload_data,
      input  [3:0] io_output_r_payload_id,
      input  [1:0] io_output_r_payload_resp,
      input   io_output_r_payload_last,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg  _zz_Axi4SharedArbiter_3__3_;
  wire  _zz_Axi4SharedArbiter_3__4_;
  wire  _zz_Axi4SharedArbiter_3__5_;
  wire  cmdArbiter_io_inputs_0_ready;
  wire  cmdArbiter_io_output_valid;
  wire [19:0] cmdArbiter_io_output_payload_addr;
  wire [3:0] cmdArbiter_io_output_payload_id;
  wire [7:0] cmdArbiter_io_output_payload_len;
  wire [2:0] cmdArbiter_io_output_payload_size;
  wire [1:0] cmdArbiter_io_output_payload_burst;
  wire  cmdArbiter_io_output_payload_write;
  wire [0:0] cmdArbiter_io_chosenOH;
  wire  streamFork_5__io_input_ready;
  wire  streamFork_5__io_outputs_0_valid;
  wire [19:0] streamFork_5__io_outputs_0_payload_addr;
  wire [3:0] streamFork_5__io_outputs_0_payload_id;
  wire [7:0] streamFork_5__io_outputs_0_payload_len;
  wire [2:0] streamFork_5__io_outputs_0_payload_size;
  wire [1:0] streamFork_5__io_outputs_0_payload_burst;
  wire  streamFork_5__io_outputs_0_payload_write;
  wire  streamFork_5__io_outputs_1_valid;
  wire [19:0] streamFork_5__io_outputs_1_payload_addr;
  wire [3:0] streamFork_5__io_outputs_1_payload_id;
  wire [7:0] streamFork_5__io_outputs_1_payload_len;
  wire [2:0] streamFork_5__io_outputs_1_payload_size;
  wire [1:0] streamFork_5__io_outputs_1_payload_burst;
  wire  streamFork_5__io_outputs_1_payload_write;
  wire  streamFifoLowLatency_5__io_push_ready;
  wire  streamFifoLowLatency_5__io_pop_valid;
  wire [2:0] streamFifoLowLatency_5__io_occupancy;
  wire  inputsCmd_0_valid;
  wire  inputsCmd_0_ready;
  wire [19:0] inputsCmd_0_payload_addr;
  wire [3:0] inputsCmd_0_payload_id;
  wire [7:0] inputsCmd_0_payload_len;
  wire [2:0] inputsCmd_0_payload_size;
  wire [1:0] inputsCmd_0_payload_burst;
  wire  inputsCmd_0_payload_write;
  reg  _zz_Axi4SharedArbiter_3__1_;
  wire  _zz_Axi4SharedArbiter_3__2_;
  wire  routeDataInput_valid;
  wire  routeDataInput_ready;
  wire [31:0] routeDataInput_payload_data;
  wire [3:0] routeDataInput_payload_strb;
  wire  routeDataInput_payload_last;
  wire  writeRspSels_0;
  wire  readRspSels_0;
  StreamArbiter_3_ cmdArbiter ( 
    .io_inputs_0_valid(inputsCmd_0_valid),
    .io_inputs_0_ready(cmdArbiter_io_inputs_0_ready),
    .io_inputs_0_payload_addr(inputsCmd_0_payload_addr),
    .io_inputs_0_payload_id(inputsCmd_0_payload_id),
    .io_inputs_0_payload_len(inputsCmd_0_payload_len),
    .io_inputs_0_payload_size(inputsCmd_0_payload_size),
    .io_inputs_0_payload_burst(inputsCmd_0_payload_burst),
    .io_inputs_0_payload_write(inputsCmd_0_payload_write),
    .io_output_valid(cmdArbiter_io_output_valid),
    .io_output_ready(streamFork_5__io_input_ready),
    .io_output_payload_addr(cmdArbiter_io_output_payload_addr),
    .io_output_payload_id(cmdArbiter_io_output_payload_id),
    .io_output_payload_len(cmdArbiter_io_output_payload_len),
    .io_output_payload_size(cmdArbiter_io_output_payload_size),
    .io_output_payload_burst(cmdArbiter_io_output_payload_burst),
    .io_output_payload_write(cmdArbiter_io_output_payload_write),
    .io_chosenOH(cmdArbiter_io_chosenOH),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  StreamFork_3_ streamFork_5_ ( 
    .io_input_valid(cmdArbiter_io_output_valid),
    .io_input_ready(streamFork_5__io_input_ready),
    .io_input_payload_addr(cmdArbiter_io_output_payload_addr),
    .io_input_payload_id(cmdArbiter_io_output_payload_id),
    .io_input_payload_len(cmdArbiter_io_output_payload_len),
    .io_input_payload_size(cmdArbiter_io_output_payload_size),
    .io_input_payload_burst(cmdArbiter_io_output_payload_burst),
    .io_input_payload_write(cmdArbiter_io_output_payload_write),
    .io_outputs_0_valid(streamFork_5__io_outputs_0_valid),
    .io_outputs_0_ready(io_output_arw_ready),
    .io_outputs_0_payload_addr(streamFork_5__io_outputs_0_payload_addr),
    .io_outputs_0_payload_id(streamFork_5__io_outputs_0_payload_id),
    .io_outputs_0_payload_len(streamFork_5__io_outputs_0_payload_len),
    .io_outputs_0_payload_size(streamFork_5__io_outputs_0_payload_size),
    .io_outputs_0_payload_burst(streamFork_5__io_outputs_0_payload_burst),
    .io_outputs_0_payload_write(streamFork_5__io_outputs_0_payload_write),
    .io_outputs_1_valid(streamFork_5__io_outputs_1_valid),
    .io_outputs_1_ready(_zz_Axi4SharedArbiter_3__3_),
    .io_outputs_1_payload_addr(streamFork_5__io_outputs_1_payload_addr),
    .io_outputs_1_payload_id(streamFork_5__io_outputs_1_payload_id),
    .io_outputs_1_payload_len(streamFork_5__io_outputs_1_payload_len),
    .io_outputs_1_payload_size(streamFork_5__io_outputs_1_payload_size),
    .io_outputs_1_payload_burst(streamFork_5__io_outputs_1_payload_burst),
    .io_outputs_1_payload_write(streamFork_5__io_outputs_1_payload_write),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  StreamFifoLowLatency_4_ streamFifoLowLatency_5_ ( 
    .io_push_valid(_zz_Axi4SharedArbiter_3__1_),
    .io_push_ready(streamFifoLowLatency_5__io_push_ready),
    .io_pop_valid(streamFifoLowLatency_5__io_pop_valid),
    .io_pop_ready(_zz_Axi4SharedArbiter_3__4_),
    .io_flush(_zz_Axi4SharedArbiter_3__5_),
    .io_occupancy(streamFifoLowLatency_5__io_occupancy),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  assign inputsCmd_0_valid = io_sharedInputs_0_arw_valid;
  assign io_sharedInputs_0_arw_ready = inputsCmd_0_ready;
  assign inputsCmd_0_payload_addr = io_sharedInputs_0_arw_payload_addr;
  assign inputsCmd_0_payload_id = io_sharedInputs_0_arw_payload_id;
  assign inputsCmd_0_payload_len = io_sharedInputs_0_arw_payload_len;
  assign inputsCmd_0_payload_size = io_sharedInputs_0_arw_payload_size;
  assign inputsCmd_0_payload_burst = io_sharedInputs_0_arw_payload_burst;
  assign inputsCmd_0_payload_write = io_sharedInputs_0_arw_payload_write;
  assign inputsCmd_0_ready = cmdArbiter_io_inputs_0_ready;
  assign io_output_arw_valid = streamFork_5__io_outputs_0_valid;
  assign io_output_arw_payload_addr = streamFork_5__io_outputs_0_payload_addr;
  assign io_output_arw_payload_len = streamFork_5__io_outputs_0_payload_len;
  assign io_output_arw_payload_size = streamFork_5__io_outputs_0_payload_size;
  assign io_output_arw_payload_burst = streamFork_5__io_outputs_0_payload_burst;
  assign io_output_arw_payload_write = streamFork_5__io_outputs_0_payload_write;
  assign io_output_arw_payload_id = (streamFork_5__io_outputs_0_payload_write ? streamFork_5__io_outputs_0_payload_id : streamFork_5__io_outputs_0_payload_id);
  always @ (*) begin
    _zz_Axi4SharedArbiter_3__1_ = streamFork_5__io_outputs_1_valid;
    _zz_Axi4SharedArbiter_3__3_ = _zz_Axi4SharedArbiter_3__2_;
    if((! streamFork_5__io_outputs_1_payload_write))begin
      _zz_Axi4SharedArbiter_3__1_ = 1'b0;
      _zz_Axi4SharedArbiter_3__3_ = 1'b1;
    end
  end

  assign _zz_Axi4SharedArbiter_3__2_ = streamFifoLowLatency_5__io_push_ready;
  assign routeDataInput_valid = io_sharedInputs_0_w_valid;
  assign routeDataInput_ready = io_sharedInputs_0_w_ready;
  assign routeDataInput_payload_data = io_sharedInputs_0_w_payload_data;
  assign routeDataInput_payload_strb = io_sharedInputs_0_w_payload_strb;
  assign routeDataInput_payload_last = io_sharedInputs_0_w_payload_last;
  assign io_output_w_valid = (streamFifoLowLatency_5__io_pop_valid && routeDataInput_valid);
  assign io_output_w_payload_data = routeDataInput_payload_data;
  assign io_output_w_payload_strb = routeDataInput_payload_strb;
  assign io_output_w_payload_last = routeDataInput_payload_last;
  assign io_sharedInputs_0_w_ready = ((streamFifoLowLatency_5__io_pop_valid && io_output_w_ready) && 1'b1);
  assign _zz_Axi4SharedArbiter_3__4_ = ((io_output_w_valid && io_output_w_ready) && io_output_w_payload_last);
  assign writeRspSels_0 = 1'b1;
  assign io_sharedInputs_0_b_valid = (io_output_b_valid && writeRspSels_0);
  assign io_sharedInputs_0_b_payload_resp = io_output_b_payload_resp;
  assign io_sharedInputs_0_b_payload_id = io_output_b_payload_id;
  assign io_output_b_ready = io_sharedInputs_0_b_ready;
  assign readRspSels_0 = 1'b1;
  assign io_sharedInputs_0_r_valid = (io_output_r_valid && readRspSels_0);
  assign io_sharedInputs_0_r_payload_data = io_output_r_payload_data;
  assign io_sharedInputs_0_r_payload_resp = io_output_r_payload_resp;
  assign io_sharedInputs_0_r_payload_last = io_output_r_payload_last;
  assign io_sharedInputs_0_r_payload_id = io_output_r_payload_id;
  assign io_output_r_ready = io_sharedInputs_0_r_ready;
  assign _zz_Axi4SharedArbiter_3__5_ = 1'b0;
endmodule

module JtagUart (
      input   io_tx_valid,
      output  io_tx_ready,
      input  [7:0] io_tx_payload,
      output  io_rx_valid,
      input   io_rx_ready,
      output [7:0] io_rx_payload,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire  _zz_JtagUart_1_;
  reg  _zz_JtagUart_2_;
  reg  _zz_JtagUart_3_;
  wire  jtag_CAPTURE;
  wire  jtag_DRCK;
  wire  jtag_RESET;
  wire  jtag_RUNTEST;
  wire  jtag_SEL;
  wire  jtag_SHIFT;
  wire  jtag_TCK;
  wire  jtag_TDI;
  wire  jtag_TMS;
  wire  jtag_UPDATE;
  wire  txStreamCC_io_push_ready;
  wire  txStreamCC_io_pop_valid;
  wire [7:0] txStreamCC_io_pop_payload;
  wire [8:0] txStreamCC_io_pushOccupancy;
  wire [8:0] txStreamCC_io_popOccupancy;
  wire  rxStreamCC_io_push_ready;
  wire  rxStreamCC_io_pop_valid;
  wire [7:0] rxStreamCC_io_pop_payload;
  wire [5:0] rxStreamCC_io_pushOccupancy;
  wire [5:0] rxStreamCC_io_popOccupancy;
  wire [8:0] _zz_JtagUart_4_;
  wire [0:0] _zz_JtagUart_5_;
  wire [8:0] _zz_JtagUart_6_;
  wire [8:0] _zz_JtagUart_7_;
  reg [8:0] jtagArea_dr_shift;
  reg  jtagArea_push_valid;
  reg [7:0] jtagArea_push_payload;
  assign _zz_JtagUart_4_ = (_zz_JtagUart_6_ <<< 8);
  assign _zz_JtagUart_5_ = jtag_TDI;
  assign _zz_JtagUart_6_ = {8'd0, _zz_JtagUart_5_};
  assign _zz_JtagUart_7_ = (jtagArea_dr_shift >>> 1);
  BSCAN_SPARTAN6 #( 
    .JTAG_CHAIN(3) 
  ) jtag ( 
    .CAPTURE(jtag_CAPTURE),
    .DRCK(jtag_DRCK),
    .RESET(jtag_RESET),
    .RUNTEST(jtag_RUNTEST),
    .SEL(jtag_SEL),
    .SHIFT(jtag_SHIFT),
    .TCK(jtag_TCK),
    .TDI(jtag_TDI),
    .TMS(jtag_TMS),
    .UPDATE(jtag_UPDATE),
    .TDO(_zz_JtagUart_1_) 
  );
  StreamFifoCC txStreamCC ( 
    .io_push_valid(io_tx_valid),
    .io_push_ready(txStreamCC_io_push_ready),
    .io_push_payload(io_tx_payload),
    .io_pop_valid(txStreamCC_io_pop_valid),
    .io_pop_ready(_zz_JtagUart_2_),
    .io_pop_payload(txStreamCC_io_pop_payload),
    .io_pushOccupancy(txStreamCC_io_pushOccupancy),
    .io_popOccupancy(txStreamCC_io_popOccupancy),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_),
    .jtag_TCK(jtag_TCK),
    .jtag_RESET(jtag_RESET) 
  );
  StreamFifoCC_1_ rxStreamCC ( 
    .io_push_valid(_zz_JtagUart_3_),
    .io_push_ready(rxStreamCC_io_push_ready),
    .io_push_payload(jtagArea_push_payload),
    .io_pop_valid(rxStreamCC_io_pop_valid),
    .io_pop_ready(io_rx_ready),
    .io_pop_payload(rxStreamCC_io_pop_payload),
    .io_pushOccupancy(rxStreamCC_io_pushOccupancy),
    .io_popOccupancy(rxStreamCC_io_popOccupancy),
    .jtag_TCK(jtag_TCK),
    .jtag_RESET(jtag_RESET),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  assign io_tx_ready = txStreamCC_io_push_ready;
  assign io_rx_valid = rxStreamCC_io_pop_valid;
  assign io_rx_payload = rxStreamCC_io_pop_payload;
  always @ (*) begin
    _zz_JtagUart_3_ = jtagArea_push_valid;
    if(rxStreamCC_io_push_ready)begin
      _zz_JtagUart_3_ = 1'b0;
    end
  end

  always @ (*) begin
    _zz_JtagUart_2_ = 1'b0;
    if(jtag_SEL)begin
      if(jtag_CAPTURE)begin
        _zz_JtagUart_2_ = 1'b1;
      end
    end
  end

  assign _zz_JtagUart_1_ = jtagArea_dr_shift[0];
  always @ (posedge jtag_TCK or posedge jtag_RESET) begin
    if (jtag_RESET) begin
      jtagArea_dr_shift <= (9'b000000000);
      jtagArea_push_valid <= 1'b0;
      jtagArea_push_payload <= (8'b00000000);
    end else begin
      if(rxStreamCC_io_push_ready)begin
        jtagArea_push_valid <= 1'b0;
      end
      if(jtag_SEL)begin
        if(jtag_CAPTURE)begin
          jtagArea_dr_shift[8] <= txStreamCC_io_pop_valid;
          jtagArea_dr_shift[7 : 0] <= txStreamCC_io_pop_payload;
        end
        if(jtag_SHIFT)begin
          jtagArea_dr_shift <= (_zz_JtagUart_4_ | _zz_JtagUart_7_);
        end
        if(jtag_UPDATE)begin
          if(jtagArea_dr_shift[8])begin
            jtagArea_push_valid <= 1'b1;
            jtagArea_push_payload <= jtagArea_dr_shift[7 : 0];
          end
        end
      end
    end
  end

endmodule

module Prescaler (
      input   io_clear,
      input  [15:0] io_limit,
      output  io_overflow,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg [15:0] counter;
  assign io_overflow = (counter == io_limit);
  always @ (posedge toplevel_main_clk) begin
    counter <= (counter + (16'b0000000000000001));
    if((io_clear || io_overflow))begin
      counter <= (16'b0000000000000000);
    end
  end

endmodule

module Timer (
      input   io_tick,
      input   io_clear,
      input  [15:0] io_limit,
      output  io_full,
      output [15:0] io_value,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire [0:0] _zz_Timer_1_;
  wire [15:0] _zz_Timer_2_;
  reg [15:0] counter;
  wire  limitHit;
  reg  inhibitFull;
  assign _zz_Timer_1_ = (! limitHit);
  assign _zz_Timer_2_ = {15'd0, _zz_Timer_1_};
  assign limitHit = (counter == io_limit);
  assign io_full = ((limitHit && io_tick) && (! inhibitFull));
  assign io_value = counter;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      inhibitFull <= 1'b0;
    end else begin
      if(io_tick)begin
        inhibitFull <= limitHit;
      end
      if(io_clear)begin
        inhibitFull <= 1'b0;
      end
    end
  end

  always @ (posedge toplevel_main_clk) begin
    if(io_tick)begin
      counter <= (counter + _zz_Timer_2_);
    end
    if(io_clear)begin
      counter <= (16'b0000000000000000);
    end
  end

endmodule

module Timer_1_ (
      input   io_tick,
      input   io_clear,
      input  [15:0] io_limit,
      output  io_full,
      output [15:0] io_value,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire [0:0] _zz_Timer_1__1_;
  wire [15:0] _zz_Timer_1__2_;
  reg [15:0] counter;
  wire  limitHit;
  reg  inhibitFull;
  assign _zz_Timer_1__1_ = (! limitHit);
  assign _zz_Timer_1__2_ = {15'd0, _zz_Timer_1__1_};
  assign limitHit = (counter == io_limit);
  assign io_full = ((limitHit && io_tick) && (! inhibitFull));
  assign io_value = counter;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      inhibitFull <= 1'b0;
    end else begin
      if(io_tick)begin
        inhibitFull <= limitHit;
      end
      if(io_clear)begin
        inhibitFull <= 1'b0;
      end
    end
  end

  always @ (posedge toplevel_main_clk) begin
    if(io_tick)begin
      counter <= (counter + _zz_Timer_1__2_);
    end
    if(io_clear)begin
      counter <= (16'b0000000000000000);
    end
  end

endmodule

module InterruptCtrl (
      input  [1:0] io_inputs,
      input  [1:0] io_clears,
      input  [1:0] io_masks,
      output [1:0] io_pendings,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg [1:0] pendings;
  assign io_pendings = (pendings & io_masks);
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      pendings <= (2'b00);
    end else begin
      pendings <= ((pendings & (~ io_clears)) | io_inputs);
    end
  end

endmodule

module StreamFifoCC_2_ (
      input   io_push_valid,
      output  io_push_ready,
      input  [9:0] io_push_payload,
      output  io_pop_valid,
      input   io_pop_ready,
      output [9:0] io_pop_payload,
      output [11:0] io_pushOccupancy,
      output [11:0] io_popOccupancy,
      input   u_gmii_rx_io_rx_clk,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire [11:0] _zz_StreamFifoCC_2__25_;
  wire [11:0] _zz_StreamFifoCC_2__26_;
  reg [9:0] _zz_StreamFifoCC_2__27_;
  wire [11:0] bufferCC_19__io_dataOut;
  wire [11:0] bufferCC_20__io_dataOut;
  wire [0:0] _zz_StreamFifoCC_2__28_;
  wire [11:0] _zz_StreamFifoCC_2__29_;
  wire [11:0] _zz_StreamFifoCC_2__30_;
  wire [10:0] _zz_StreamFifoCC_2__31_;
  wire [0:0] _zz_StreamFifoCC_2__32_;
  wire [11:0] _zz_StreamFifoCC_2__33_;
  wire [11:0] _zz_StreamFifoCC_2__34_;
  wire [10:0] _zz_StreamFifoCC_2__35_;
  wire  _zz_StreamFifoCC_2__36_;
  wire [0:0] _zz_StreamFifoCC_2__37_;
  wire [1:0] _zz_StreamFifoCC_2__38_;
  wire [0:0] _zz_StreamFifoCC_2__39_;
  wire [1:0] _zz_StreamFifoCC_2__40_;
  reg  _zz_StreamFifoCC_2__1_;
  wire [11:0] popToPushGray;
  wire [11:0] pushToPopGray;
  reg  pushCC_pushPtr_willIncrement;
  wire  pushCC_pushPtr_willClear;
  reg [11:0] pushCC_pushPtr_valueNext;
  reg [11:0] pushCC_pushPtr_value = (12'b000000000000);
  wire  pushCC_pushPtr_willOverflowIfInc;
  wire  pushCC_pushPtr_willOverflow;
  reg [11:0] pushCC_pushPtrGray = (12'b000000000000);
  wire [11:0] pushCC_popPtrGray;
  wire  pushCC_full;
  wire  _zz_StreamFifoCC_2__2_;
  wire  _zz_StreamFifoCC_2__3_;
  wire  _zz_StreamFifoCC_2__4_;
  wire  _zz_StreamFifoCC_2__5_;
  wire  _zz_StreamFifoCC_2__6_;
  wire  _zz_StreamFifoCC_2__7_;
  wire  _zz_StreamFifoCC_2__8_;
  wire  _zz_StreamFifoCC_2__9_;
  wire  _zz_StreamFifoCC_2__10_;
  wire  _zz_StreamFifoCC_2__11_;
  wire  _zz_StreamFifoCC_2__12_;
  reg  popCC_popPtr_willIncrement;
  wire  popCC_popPtr_willClear;
  reg [11:0] popCC_popPtr_valueNext;
  reg [11:0] popCC_popPtr_value;
  wire  popCC_popPtr_willOverflowIfInc;
  wire  popCC_popPtr_willOverflow;
  reg [11:0] popCC_popPtrGray;
  wire [11:0] popCC_pushPtrGray;
  wire  popCC_empty;
  wire [11:0] _zz_StreamFifoCC_2__13_;
  wire  _zz_StreamFifoCC_2__14_;
  wire  _zz_StreamFifoCC_2__15_;
  wire  _zz_StreamFifoCC_2__16_;
  wire  _zz_StreamFifoCC_2__17_;
  wire  _zz_StreamFifoCC_2__18_;
  wire  _zz_StreamFifoCC_2__19_;
  wire  _zz_StreamFifoCC_2__20_;
  wire  _zz_StreamFifoCC_2__21_;
  wire  _zz_StreamFifoCC_2__22_;
  wire  _zz_StreamFifoCC_2__23_;
  wire  _zz_StreamFifoCC_2__24_;
  reg [9:0] ram [0:2047];
  assign _zz_StreamFifoCC_2__28_ = pushCC_pushPtr_willIncrement;
  assign _zz_StreamFifoCC_2__29_ = {11'd0, _zz_StreamFifoCC_2__28_};
  assign _zz_StreamFifoCC_2__30_ = (pushCC_pushPtr_valueNext >>> (1'b1));
  assign _zz_StreamFifoCC_2__31_ = pushCC_pushPtr_value[10:0];
  assign _zz_StreamFifoCC_2__32_ = popCC_popPtr_willIncrement;
  assign _zz_StreamFifoCC_2__33_ = {11'd0, _zz_StreamFifoCC_2__32_};
  assign _zz_StreamFifoCC_2__34_ = (popCC_popPtr_valueNext >>> (1'b1));
  assign _zz_StreamFifoCC_2__35_ = _zz_StreamFifoCC_2__13_[10:0];
  assign _zz_StreamFifoCC_2__36_ = 1'b1;
  assign _zz_StreamFifoCC_2__37_ = _zz_StreamFifoCC_2__3_;
  assign _zz_StreamFifoCC_2__38_ = {_zz_StreamFifoCC_2__2_,(pushCC_popPtrGray[0] ^ _zz_StreamFifoCC_2__2_)};
  assign _zz_StreamFifoCC_2__39_ = _zz_StreamFifoCC_2__15_;
  assign _zz_StreamFifoCC_2__40_ = {_zz_StreamFifoCC_2__14_,(popCC_pushPtrGray[0] ^ _zz_StreamFifoCC_2__14_)};
  always @ (posedge u_gmii_rx_io_rx_clk) begin
    if(_zz_StreamFifoCC_2__1_) begin
      ram[_zz_StreamFifoCC_2__31_] <= io_push_payload;
    end
  end

  always @ (posedge toplevel_main_clk) begin
  end

  always @ (posedge toplevel_main_clk) begin
    if(_zz_StreamFifoCC_2__36_) begin
      _zz_StreamFifoCC_2__27_ <= ram[_zz_StreamFifoCC_2__35_];
    end
  end

  BufferCC_5_ bufferCC_19_ ( 
    .io_initial(_zz_StreamFifoCC_2__25_),
    .io_dataIn(popToPushGray),
    .io_dataOut(bufferCC_19__io_dataOut),
    .u_gmii_rx_io_rx_clk(u_gmii_rx_io_rx_clk) 
  );
  BufferCC_6_ bufferCC_20_ ( 
    .io_initial(_zz_StreamFifoCC_2__26_),
    .io_dataIn(pushToPopGray),
    .io_dataOut(bufferCC_20__io_dataOut),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  always @ (*) begin
    _zz_StreamFifoCC_2__1_ = 1'b0;
    pushCC_pushPtr_willIncrement = 1'b0;
    if((io_push_valid && io_push_ready))begin
      _zz_StreamFifoCC_2__1_ = 1'b1;
      pushCC_pushPtr_willIncrement = 1'b1;
    end
  end

  assign pushCC_pushPtr_willClear = 1'b0;
  assign pushCC_pushPtr_willOverflowIfInc = (pushCC_pushPtr_value == (12'b111111111111));
  assign pushCC_pushPtr_willOverflow = (pushCC_pushPtr_willOverflowIfInc && pushCC_pushPtr_willIncrement);
  always @ (*) begin
    pushCC_pushPtr_valueNext = (pushCC_pushPtr_value + _zz_StreamFifoCC_2__29_);
    if(pushCC_pushPtr_willClear)begin
      pushCC_pushPtr_valueNext = (12'b000000000000);
    end
  end

  assign _zz_StreamFifoCC_2__25_ = (12'b000000000000);
  assign pushCC_popPtrGray = bufferCC_19__io_dataOut;
  assign pushCC_full = ((pushCC_pushPtrGray[11 : 10] == (~ pushCC_popPtrGray[11 : 10])) && (pushCC_pushPtrGray[9 : 0] == pushCC_popPtrGray[9 : 0]));
  assign io_push_ready = (! pushCC_full);
  assign _zz_StreamFifoCC_2__2_ = (pushCC_popPtrGray[1] ^ _zz_StreamFifoCC_2__3_);
  assign _zz_StreamFifoCC_2__3_ = (pushCC_popPtrGray[2] ^ _zz_StreamFifoCC_2__4_);
  assign _zz_StreamFifoCC_2__4_ = (pushCC_popPtrGray[3] ^ _zz_StreamFifoCC_2__5_);
  assign _zz_StreamFifoCC_2__5_ = (pushCC_popPtrGray[4] ^ _zz_StreamFifoCC_2__6_);
  assign _zz_StreamFifoCC_2__6_ = (pushCC_popPtrGray[5] ^ _zz_StreamFifoCC_2__7_);
  assign _zz_StreamFifoCC_2__7_ = (pushCC_popPtrGray[6] ^ _zz_StreamFifoCC_2__8_);
  assign _zz_StreamFifoCC_2__8_ = (pushCC_popPtrGray[7] ^ _zz_StreamFifoCC_2__9_);
  assign _zz_StreamFifoCC_2__9_ = (pushCC_popPtrGray[8] ^ _zz_StreamFifoCC_2__10_);
  assign _zz_StreamFifoCC_2__10_ = (pushCC_popPtrGray[9] ^ _zz_StreamFifoCC_2__11_);
  assign _zz_StreamFifoCC_2__11_ = (pushCC_popPtrGray[10] ^ _zz_StreamFifoCC_2__12_);
  assign _zz_StreamFifoCC_2__12_ = pushCC_popPtrGray[11];
  assign io_pushOccupancy = (pushCC_pushPtr_value - {_zz_StreamFifoCC_2__12_,{_zz_StreamFifoCC_2__11_,{_zz_StreamFifoCC_2__10_,{_zz_StreamFifoCC_2__9_,{_zz_StreamFifoCC_2__8_,{_zz_StreamFifoCC_2__7_,{_zz_StreamFifoCC_2__6_,{_zz_StreamFifoCC_2__5_,{_zz_StreamFifoCC_2__4_,{_zz_StreamFifoCC_2__37_,_zz_StreamFifoCC_2__38_}}}}}}}}}});
  always @ (*) begin
    popCC_popPtr_willIncrement = 1'b0;
    if((io_pop_valid && io_pop_ready))begin
      popCC_popPtr_willIncrement = 1'b1;
    end
  end

  assign popCC_popPtr_willClear = 1'b0;
  assign popCC_popPtr_willOverflowIfInc = (popCC_popPtr_value == (12'b111111111111));
  assign popCC_popPtr_willOverflow = (popCC_popPtr_willOverflowIfInc && popCC_popPtr_willIncrement);
  always @ (*) begin
    popCC_popPtr_valueNext = (popCC_popPtr_value + _zz_StreamFifoCC_2__33_);
    if(popCC_popPtr_willClear)begin
      popCC_popPtr_valueNext = (12'b000000000000);
    end
  end

  assign _zz_StreamFifoCC_2__26_ = (12'b000000000000);
  assign popCC_pushPtrGray = bufferCC_20__io_dataOut;
  assign popCC_empty = (popCC_popPtrGray == popCC_pushPtrGray);
  assign io_pop_valid = (! popCC_empty);
  assign _zz_StreamFifoCC_2__13_ = popCC_popPtr_valueNext;
  assign io_pop_payload = _zz_StreamFifoCC_2__27_;
  assign _zz_StreamFifoCC_2__14_ = (popCC_pushPtrGray[1] ^ _zz_StreamFifoCC_2__15_);
  assign _zz_StreamFifoCC_2__15_ = (popCC_pushPtrGray[2] ^ _zz_StreamFifoCC_2__16_);
  assign _zz_StreamFifoCC_2__16_ = (popCC_pushPtrGray[3] ^ _zz_StreamFifoCC_2__17_);
  assign _zz_StreamFifoCC_2__17_ = (popCC_pushPtrGray[4] ^ _zz_StreamFifoCC_2__18_);
  assign _zz_StreamFifoCC_2__18_ = (popCC_pushPtrGray[5] ^ _zz_StreamFifoCC_2__19_);
  assign _zz_StreamFifoCC_2__19_ = (popCC_pushPtrGray[6] ^ _zz_StreamFifoCC_2__20_);
  assign _zz_StreamFifoCC_2__20_ = (popCC_pushPtrGray[7] ^ _zz_StreamFifoCC_2__21_);
  assign _zz_StreamFifoCC_2__21_ = (popCC_pushPtrGray[8] ^ _zz_StreamFifoCC_2__22_);
  assign _zz_StreamFifoCC_2__22_ = (popCC_pushPtrGray[9] ^ _zz_StreamFifoCC_2__23_);
  assign _zz_StreamFifoCC_2__23_ = (popCC_pushPtrGray[10] ^ _zz_StreamFifoCC_2__24_);
  assign _zz_StreamFifoCC_2__24_ = popCC_pushPtrGray[11];
  assign io_popOccupancy = ({_zz_StreamFifoCC_2__24_,{_zz_StreamFifoCC_2__23_,{_zz_StreamFifoCC_2__22_,{_zz_StreamFifoCC_2__21_,{_zz_StreamFifoCC_2__20_,{_zz_StreamFifoCC_2__19_,{_zz_StreamFifoCC_2__18_,{_zz_StreamFifoCC_2__17_,{_zz_StreamFifoCC_2__16_,{_zz_StreamFifoCC_2__39_,_zz_StreamFifoCC_2__40_}}}}}}}}}} - popCC_popPtr_value);
  assign pushToPopGray = pushCC_pushPtrGray;
  assign popToPushGray = popCC_popPtrGray;
  always @ (posedge u_gmii_rx_io_rx_clk) begin
    pushCC_pushPtr_value <= pushCC_pushPtr_valueNext;
    pushCC_pushPtrGray <= (_zz_StreamFifoCC_2__30_ ^ pushCC_pushPtr_valueNext);
  end

  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      popCC_popPtr_value <= (12'b000000000000);
      popCC_popPtrGray <= (12'b000000000000);
    end else begin
      popCC_popPtr_value <= popCC_popPtr_valueNext;
      popCC_popPtrGray <= (_zz_StreamFifoCC_2__34_ ^ popCC_popPtr_valueNext);
    end
  end

endmodule

module BufferCC_7_ (
      input   io_initial,
      input   io_dataIn,
      output  io_dataOut,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg  buffers_0;
  reg  buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      buffers_0 <= io_initial;
      buffers_1 <= io_initial;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end

endmodule

module BufferCC_8_ (
      input   io_initial,
      input   io_dataIn,
      output  io_dataOut,
      input   core_u_pano_core_io_ulpi_clk,
      input   _zz_BufferCC_8__1_);
  reg  buffers_0;
  reg  buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge core_u_pano_core_io_ulpi_clk) begin
    if(!_zz_BufferCC_8__1_) begin
      buffers_0 <= io_initial;
      buffers_1 <= io_initial;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end

endmodule

module CpuComplex (
      output [19:0] io_apb_PADDR,
      output [0:0] io_apb_PSEL,
      output  io_apb_PENABLE,
      input   io_apb_PREADY,
      output  io_apb_PWRITE,
      output [31:0] io_apb_PWDATA,
      input  [31:0] io_apb_PRDATA,
      input   io_apb_PSLVERROR,
      output  io_axiMem1_arw_valid,
      input   io_axiMem1_arw_ready,
      output [31:0] io_axiMem1_arw_payload_addr,
      output [3:0] io_axiMem1_arw_payload_id,
      output [3:0] io_axiMem1_arw_payload_region,
      output [7:0] io_axiMem1_arw_payload_len,
      output [2:0] io_axiMem1_arw_payload_size,
      output [1:0] io_axiMem1_arw_payload_burst,
      output [0:0] io_axiMem1_arw_payload_lock,
      output [3:0] io_axiMem1_arw_payload_cache,
      output [3:0] io_axiMem1_arw_payload_qos,
      output [2:0] io_axiMem1_arw_payload_prot,
      output  io_axiMem1_arw_payload_write,
      output  io_axiMem1_w_valid,
      input   io_axiMem1_w_ready,
      output [31:0] io_axiMem1_w_payload_data,
      output [3:0] io_axiMem1_w_payload_strb,
      output  io_axiMem1_w_payload_last,
      input   io_axiMem1_b_valid,
      output  io_axiMem1_b_ready,
      input  [3:0] io_axiMem1_b_payload_id,
      input  [1:0] io_axiMem1_b_payload_resp,
      input   io_axiMem1_r_valid,
      output  io_axiMem1_r_ready,
      input  [31:0] io_axiMem1_r_payload_data,
      input  [3:0] io_axiMem1_r_payload_id,
      input  [1:0] io_axiMem1_r_payload_resp,
      input   io_axiMem1_r_payload_last,
      output  io_axiMem2_arw_valid,
      input   io_axiMem2_arw_ready,
      output [31:0] io_axiMem2_arw_payload_addr,
      output [3:0] io_axiMem2_arw_payload_id,
      output [3:0] io_axiMem2_arw_payload_region,
      output [7:0] io_axiMem2_arw_payload_len,
      output [2:0] io_axiMem2_arw_payload_size,
      output [1:0] io_axiMem2_arw_payload_burst,
      output [0:0] io_axiMem2_arw_payload_lock,
      output [3:0] io_axiMem2_arw_payload_cache,
      output [3:0] io_axiMem2_arw_payload_qos,
      output [2:0] io_axiMem2_arw_payload_prot,
      output  io_axiMem2_arw_payload_write,
      output  io_axiMem2_w_valid,
      input   io_axiMem2_w_ready,
      output [31:0] io_axiMem2_w_payload_data,
      output [3:0] io_axiMem2_w_payload_strb,
      output  io_axiMem2_w_payload_last,
      input   io_axiMem2_b_valid,
      output  io_axiMem2_b_ready,
      input  [3:0] io_axiMem2_b_payload_id,
      input  [1:0] io_axiMem2_b_payload_resp,
      input   io_axiMem2_r_valid,
      output  io_axiMem2_r_ready,
      input  [31:0] io_axiMem2_r_payload_data,
      input  [3:0] io_axiMem2_r_payload_id,
      input  [1:0] io_axiMem2_r_payload_resp,
      input   io_axiMem2_r_payload_last,
      input   io_externalInterrupt,
      input   io_timerInterrupt,
      input   io_jtag_tms,
      input   io_jtag_tdi,
      output  io_jtag_tdo,
      input   io_jtag_tck,
      input   toplevel_main_clk,
      input   toplevel_main_reset_,
      input   _zz_CpuComplex_109_);
  wire  _zz_CpuComplex_110_;
  wire [7:0] _zz_CpuComplex_111_;
  wire  _zz_CpuComplex_112_;
  wire  _zz_CpuComplex_113_;
  wire  _zz_CpuComplex_114_;
  reg  _zz_CpuComplex_115_;
  wire  _zz_CpuComplex_116_;
  wire  _zz_CpuComplex_117_;
  wire  _zz_CpuComplex_118_;
  wire  _zz_CpuComplex_119_;
  wire  _zz_CpuComplex_120_;
  wire  _zz_CpuComplex_121_;
  wire  _zz_CpuComplex_122_;
  wire  _zz_CpuComplex_123_;
  wire [2:0] _zz_CpuComplex_124_;
  wire [1:0] _zz_CpuComplex_125_;
  wire [0:0] _zz_CpuComplex_126_;
  wire [3:0] _zz_CpuComplex_127_;
  wire [1:0] _zz_CpuComplex_128_;
  wire [0:0] _zz_CpuComplex_129_;
  wire [3:0] _zz_CpuComplex_130_;
  wire  _zz_CpuComplex_131_;
  wire [2:0] _zz_CpuComplex_132_;
  wire [1:0] _zz_CpuComplex_133_;
  wire [0:0] _zz_CpuComplex_134_;
  wire [3:0] _zz_CpuComplex_135_;
  wire [1:0] _zz_CpuComplex_136_;
  wire [0:0] _zz_CpuComplex_137_;
  wire [3:0] _zz_CpuComplex_138_;
  wire  _zz_CpuComplex_139_;
  wire [12:0] _zz_CpuComplex_140_;
  wire [2:0] _zz_CpuComplex_141_;
  wire [1:0] _zz_CpuComplex_142_;
  wire [12:0] _zz_CpuComplex_143_;
  wire [1:0] _zz_CpuComplex_144_;
  wire  _zz_CpuComplex_145_;
  wire [19:0] _zz_CpuComplex_146_;
  wire [1:0] _zz_CpuComplex_147_;
  wire  core_cpu_iBus_cmd_valid;
  wire [31:0] core_cpu_iBus_cmd_payload_pc;
  wire  core_cpu_debug_bus_cmd_ready;
  wire [31:0] core_cpu_debug_bus_rsp_data;
  wire  core_cpu_debug_resetOut;
  wire  core_cpu_dBus_cmd_valid;
  wire  core_cpu_dBus_cmd_payload_wr;
  wire [31:0] core_cpu_dBus_cmd_payload_address;
  wire [31:0] core_cpu_dBus_cmd_payload_data;
  wire [1:0] core_cpu_dBus_cmd_payload_size;
  wire  streamFork_5__io_input_ready;
  wire  streamFork_5__io_outputs_0_valid;
  wire  streamFork_5__io_outputs_0_payload_wr;
  wire [31:0] streamFork_5__io_outputs_0_payload_address;
  wire [31:0] streamFork_5__io_outputs_0_payload_data;
  wire [1:0] streamFork_5__io_outputs_0_payload_size;
  wire  streamFork_5__io_outputs_1_valid;
  wire  streamFork_5__io_outputs_1_payload_wr;
  wire [31:0] streamFork_5__io_outputs_1_payload_address;
  wire [31:0] streamFork_5__io_outputs_1_payload_data;
  wire [1:0] streamFork_5__io_outputs_1_payload_size;
  wire  jtagBridge_1__io_jtag_tdo;
  wire  jtagBridge_1__io_remote_cmd_valid;
  wire  jtagBridge_1__io_remote_cmd_payload_last;
  wire [0:0] jtagBridge_1__io_remote_cmd_payload_fragment;
  wire  jtagBridge_1__io_remote_rsp_ready;
  wire  systemDebugger_1__io_remote_cmd_ready;
  wire  systemDebugger_1__io_remote_rsp_valid;
  wire  systemDebugger_1__io_remote_rsp_payload_error;
  wire [31:0] systemDebugger_1__io_remote_rsp_payload_data;
  wire  systemDebugger_1__io_mem_cmd_valid;
  wire [31:0] systemDebugger_1__io_mem_cmd_payload_address;
  wire [31:0] systemDebugger_1__io_mem_cmd_payload_data;
  wire  systemDebugger_1__io_mem_cmd_payload_wr;
  wire [1:0] systemDebugger_1__io_mem_cmd_payload_size;
  wire  ram_io_axi_arw_ready;
  wire  ram_io_axi_w_ready;
  wire  ram_io_axi_b_valid;
  wire [3:0] ram_io_axi_b_payload_id;
  wire [1:0] ram_io_axi_b_payload_resp;
  wire  ram_io_axi_r_valid;
  wire [31:0] ram_io_axi_r_payload_data;
  wire [3:0] ram_io_axi_r_payload_id;
  wire [1:0] ram_io_axi_r_payload_resp;
  wire  ram_io_axi_r_payload_last;
  wire  apbBridge_io_axi_arw_ready;
  wire  apbBridge_io_axi_w_ready;
  wire  apbBridge_io_axi_b_valid;
  wire [3:0] apbBridge_io_axi_b_payload_id;
  wire [1:0] apbBridge_io_axi_b_payload_resp;
  wire  apbBridge_io_axi_r_valid;
  wire [31:0] apbBridge_io_axi_r_payload_data;
  wire [3:0] apbBridge_io_axi_r_payload_id;
  wire [1:0] apbBridge_io_axi_r_payload_resp;
  wire  apbBridge_io_axi_r_payload_last;
  wire [19:0] apbBridge_io_apb_PADDR;
  wire [0:0] apbBridge_io_apb_PSEL;
  wire  apbBridge_io_apb_PENABLE;
  wire  apbBridge_io_apb_PWRITE;
  wire [31:0] apbBridge_io_apb_PWDATA;
  wire  core_iBus_decoder_io_input_ar_ready;
  wire  core_iBus_decoder_io_input_r_valid;
  wire [31:0] core_iBus_decoder_io_input_r_payload_data;
  wire [1:0] core_iBus_decoder_io_input_r_payload_resp;
  wire  core_iBus_decoder_io_input_r_payload_last;
  wire  core_iBus_decoder_io_outputs_0_ar_valid;
  wire [31:0] core_iBus_decoder_io_outputs_0_ar_payload_addr;
  wire [3:0] core_iBus_decoder_io_outputs_0_ar_payload_cache;
  wire [2:0] core_iBus_decoder_io_outputs_0_ar_payload_prot;
  wire  core_iBus_decoder_io_outputs_0_r_ready;
  wire  core_iBus_decoder_io_outputs_1_ar_valid;
  wire [31:0] core_iBus_decoder_io_outputs_1_ar_payload_addr;
  wire [3:0] core_iBus_decoder_io_outputs_1_ar_payload_cache;
  wire [2:0] core_iBus_decoder_io_outputs_1_ar_payload_prot;
  wire  core_iBus_decoder_io_outputs_1_r_ready;
  wire  core_iBus_decoder_io_outputs_2_ar_valid;
  wire [31:0] core_iBus_decoder_io_outputs_2_ar_payload_addr;
  wire [3:0] core_iBus_decoder_io_outputs_2_ar_payload_cache;
  wire [2:0] core_iBus_decoder_io_outputs_2_ar_payload_prot;
  wire  core_iBus_decoder_io_outputs_2_r_ready;
  wire  core_dBus_decoder_io_input_arw_ready;
  wire  core_dBus_decoder_io_input_w_ready;
  wire  core_dBus_decoder_io_input_b_valid;
  wire [1:0] core_dBus_decoder_io_input_b_payload_resp;
  wire  core_dBus_decoder_io_input_r_valid;
  wire [31:0] core_dBus_decoder_io_input_r_payload_data;
  wire [1:0] core_dBus_decoder_io_input_r_payload_resp;
  wire  core_dBus_decoder_io_input_r_payload_last;
  wire  core_dBus_decoder_io_sharedOutputs_0_arw_valid;
  wire [31:0] core_dBus_decoder_io_sharedOutputs_0_arw_payload_addr;
  wire [2:0] core_dBus_decoder_io_sharedOutputs_0_arw_payload_size;
  wire [3:0] core_dBus_decoder_io_sharedOutputs_0_arw_payload_cache;
  wire [2:0] core_dBus_decoder_io_sharedOutputs_0_arw_payload_prot;
  wire  core_dBus_decoder_io_sharedOutputs_0_arw_payload_write;
  wire  core_dBus_decoder_io_sharedOutputs_0_w_valid;
  wire [31:0] core_dBus_decoder_io_sharedOutputs_0_w_payload_data;
  wire [3:0] core_dBus_decoder_io_sharedOutputs_0_w_payload_strb;
  wire  core_dBus_decoder_io_sharedOutputs_0_w_payload_last;
  wire  core_dBus_decoder_io_sharedOutputs_0_b_ready;
  wire  core_dBus_decoder_io_sharedOutputs_0_r_ready;
  wire  core_dBus_decoder_io_sharedOutputs_1_arw_valid;
  wire [31:0] core_dBus_decoder_io_sharedOutputs_1_arw_payload_addr;
  wire [2:0] core_dBus_decoder_io_sharedOutputs_1_arw_payload_size;
  wire [3:0] core_dBus_decoder_io_sharedOutputs_1_arw_payload_cache;
  wire [2:0] core_dBus_decoder_io_sharedOutputs_1_arw_payload_prot;
  wire  core_dBus_decoder_io_sharedOutputs_1_arw_payload_write;
  wire  core_dBus_decoder_io_sharedOutputs_1_w_valid;
  wire [31:0] core_dBus_decoder_io_sharedOutputs_1_w_payload_data;
  wire [3:0] core_dBus_decoder_io_sharedOutputs_1_w_payload_strb;
  wire  core_dBus_decoder_io_sharedOutputs_1_w_payload_last;
  wire  core_dBus_decoder_io_sharedOutputs_1_b_ready;
  wire  core_dBus_decoder_io_sharedOutputs_1_r_ready;
  wire  core_dBus_decoder_io_sharedOutputs_2_arw_valid;
  wire [31:0] core_dBus_decoder_io_sharedOutputs_2_arw_payload_addr;
  wire [2:0] core_dBus_decoder_io_sharedOutputs_2_arw_payload_size;
  wire [3:0] core_dBus_decoder_io_sharedOutputs_2_arw_payload_cache;
  wire [2:0] core_dBus_decoder_io_sharedOutputs_2_arw_payload_prot;
  wire  core_dBus_decoder_io_sharedOutputs_2_arw_payload_write;
  wire  core_dBus_decoder_io_sharedOutputs_2_w_valid;
  wire [31:0] core_dBus_decoder_io_sharedOutputs_2_w_payload_data;
  wire [3:0] core_dBus_decoder_io_sharedOutputs_2_w_payload_strb;
  wire  core_dBus_decoder_io_sharedOutputs_2_w_payload_last;
  wire  core_dBus_decoder_io_sharedOutputs_2_b_ready;
  wire  core_dBus_decoder_io_sharedOutputs_2_r_ready;
  wire  core_dBus_decoder_io_sharedOutputs_3_arw_valid;
  wire [31:0] core_dBus_decoder_io_sharedOutputs_3_arw_payload_addr;
  wire [2:0] core_dBus_decoder_io_sharedOutputs_3_arw_payload_size;
  wire [3:0] core_dBus_decoder_io_sharedOutputs_3_arw_payload_cache;
  wire [2:0] core_dBus_decoder_io_sharedOutputs_3_arw_payload_prot;
  wire  core_dBus_decoder_io_sharedOutputs_3_arw_payload_write;
  wire  core_dBus_decoder_io_sharedOutputs_3_w_valid;
  wire [31:0] core_dBus_decoder_io_sharedOutputs_3_w_payload_data;
  wire [3:0] core_dBus_decoder_io_sharedOutputs_3_w_payload_strb;
  wire  core_dBus_decoder_io_sharedOutputs_3_w_payload_last;
  wire  core_dBus_decoder_io_sharedOutputs_3_b_ready;
  wire  core_dBus_decoder_io_sharedOutputs_3_r_ready;
  wire  io_axiMem1_arbiter_io_readInputs_0_ar_ready;
  wire  io_axiMem1_arbiter_io_readInputs_0_r_valid;
  wire [31:0] io_axiMem1_arbiter_io_readInputs_0_r_payload_data;
  wire [2:0] io_axiMem1_arbiter_io_readInputs_0_r_payload_id;
  wire [1:0] io_axiMem1_arbiter_io_readInputs_0_r_payload_resp;
  wire  io_axiMem1_arbiter_io_readInputs_0_r_payload_last;
  wire  io_axiMem1_arbiter_io_sharedInputs_0_arw_ready;
  wire  io_axiMem1_arbiter_io_sharedInputs_0_w_ready;
  wire  io_axiMem1_arbiter_io_sharedInputs_0_b_valid;
  wire [2:0] io_axiMem1_arbiter_io_sharedInputs_0_b_payload_id;
  wire [1:0] io_axiMem1_arbiter_io_sharedInputs_0_b_payload_resp;
  wire  io_axiMem1_arbiter_io_sharedInputs_0_r_valid;
  wire [31:0] io_axiMem1_arbiter_io_sharedInputs_0_r_payload_data;
  wire [2:0] io_axiMem1_arbiter_io_sharedInputs_0_r_payload_id;
  wire [1:0] io_axiMem1_arbiter_io_sharedInputs_0_r_payload_resp;
  wire  io_axiMem1_arbiter_io_sharedInputs_0_r_payload_last;
  wire  io_axiMem1_arbiter_io_output_arw_valid;
  wire [31:0] io_axiMem1_arbiter_io_output_arw_payload_addr;
  wire [3:0] io_axiMem1_arbiter_io_output_arw_payload_id;
  wire [3:0] io_axiMem1_arbiter_io_output_arw_payload_region;
  wire [7:0] io_axiMem1_arbiter_io_output_arw_payload_len;
  wire [2:0] io_axiMem1_arbiter_io_output_arw_payload_size;
  wire [1:0] io_axiMem1_arbiter_io_output_arw_payload_burst;
  wire [0:0] io_axiMem1_arbiter_io_output_arw_payload_lock;
  wire [3:0] io_axiMem1_arbiter_io_output_arw_payload_cache;
  wire [3:0] io_axiMem1_arbiter_io_output_arw_payload_qos;
  wire [2:0] io_axiMem1_arbiter_io_output_arw_payload_prot;
  wire  io_axiMem1_arbiter_io_output_arw_payload_write;
  wire  io_axiMem1_arbiter_io_output_w_valid;
  wire [31:0] io_axiMem1_arbiter_io_output_w_payload_data;
  wire [3:0] io_axiMem1_arbiter_io_output_w_payload_strb;
  wire  io_axiMem1_arbiter_io_output_w_payload_last;
  wire  io_axiMem1_arbiter_io_output_b_ready;
  wire  io_axiMem1_arbiter_io_output_r_ready;
  wire  io_axiMem2_arbiter_io_readInputs_0_ar_ready;
  wire  io_axiMem2_arbiter_io_readInputs_0_r_valid;
  wire [31:0] io_axiMem2_arbiter_io_readInputs_0_r_payload_data;
  wire [2:0] io_axiMem2_arbiter_io_readInputs_0_r_payload_id;
  wire [1:0] io_axiMem2_arbiter_io_readInputs_0_r_payload_resp;
  wire  io_axiMem2_arbiter_io_readInputs_0_r_payload_last;
  wire  io_axiMem2_arbiter_io_sharedInputs_0_arw_ready;
  wire  io_axiMem2_arbiter_io_sharedInputs_0_w_ready;
  wire  io_axiMem2_arbiter_io_sharedInputs_0_b_valid;
  wire [2:0] io_axiMem2_arbiter_io_sharedInputs_0_b_payload_id;
  wire [1:0] io_axiMem2_arbiter_io_sharedInputs_0_b_payload_resp;
  wire  io_axiMem2_arbiter_io_sharedInputs_0_r_valid;
  wire [31:0] io_axiMem2_arbiter_io_sharedInputs_0_r_payload_data;
  wire [2:0] io_axiMem2_arbiter_io_sharedInputs_0_r_payload_id;
  wire [1:0] io_axiMem2_arbiter_io_sharedInputs_0_r_payload_resp;
  wire  io_axiMem2_arbiter_io_sharedInputs_0_r_payload_last;
  wire  io_axiMem2_arbiter_io_output_arw_valid;
  wire [31:0] io_axiMem2_arbiter_io_output_arw_payload_addr;
  wire [3:0] io_axiMem2_arbiter_io_output_arw_payload_id;
  wire [3:0] io_axiMem2_arbiter_io_output_arw_payload_region;
  wire [7:0] io_axiMem2_arbiter_io_output_arw_payload_len;
  wire [2:0] io_axiMem2_arbiter_io_output_arw_payload_size;
  wire [1:0] io_axiMem2_arbiter_io_output_arw_payload_burst;
  wire [0:0] io_axiMem2_arbiter_io_output_arw_payload_lock;
  wire [3:0] io_axiMem2_arbiter_io_output_arw_payload_cache;
  wire [3:0] io_axiMem2_arbiter_io_output_arw_payload_qos;
  wire [2:0] io_axiMem2_arbiter_io_output_arw_payload_prot;
  wire  io_axiMem2_arbiter_io_output_arw_payload_write;
  wire  io_axiMem2_arbiter_io_output_w_valid;
  wire [31:0] io_axiMem2_arbiter_io_output_w_payload_data;
  wire [3:0] io_axiMem2_arbiter_io_output_w_payload_strb;
  wire  io_axiMem2_arbiter_io_output_w_payload_last;
  wire  io_axiMem2_arbiter_io_output_b_ready;
  wire  io_axiMem2_arbiter_io_output_r_ready;
  wire  ram_io_axi_arbiter_io_readInputs_0_ar_ready;
  wire  ram_io_axi_arbiter_io_readInputs_0_r_valid;
  wire [31:0] ram_io_axi_arbiter_io_readInputs_0_r_payload_data;
  wire [2:0] ram_io_axi_arbiter_io_readInputs_0_r_payload_id;
  wire [1:0] ram_io_axi_arbiter_io_readInputs_0_r_payload_resp;
  wire  ram_io_axi_arbiter_io_readInputs_0_r_payload_last;
  wire  ram_io_axi_arbiter_io_sharedInputs_0_arw_ready;
  wire  ram_io_axi_arbiter_io_sharedInputs_0_w_ready;
  wire  ram_io_axi_arbiter_io_sharedInputs_0_b_valid;
  wire [2:0] ram_io_axi_arbiter_io_sharedInputs_0_b_payload_id;
  wire [1:0] ram_io_axi_arbiter_io_sharedInputs_0_b_payload_resp;
  wire  ram_io_axi_arbiter_io_sharedInputs_0_r_valid;
  wire [31:0] ram_io_axi_arbiter_io_sharedInputs_0_r_payload_data;
  wire [2:0] ram_io_axi_arbiter_io_sharedInputs_0_r_payload_id;
  wire [1:0] ram_io_axi_arbiter_io_sharedInputs_0_r_payload_resp;
  wire  ram_io_axi_arbiter_io_sharedInputs_0_r_payload_last;
  wire  ram_io_axi_arbiter_io_output_arw_valid;
  wire [12:0] ram_io_axi_arbiter_io_output_arw_payload_addr;
  wire [3:0] ram_io_axi_arbiter_io_output_arw_payload_id;
  wire [7:0] ram_io_axi_arbiter_io_output_arw_payload_len;
  wire [2:0] ram_io_axi_arbiter_io_output_arw_payload_size;
  wire [1:0] ram_io_axi_arbiter_io_output_arw_payload_burst;
  wire  ram_io_axi_arbiter_io_output_arw_payload_write;
  wire  ram_io_axi_arbiter_io_output_w_valid;
  wire [31:0] ram_io_axi_arbiter_io_output_w_payload_data;
  wire [3:0] ram_io_axi_arbiter_io_output_w_payload_strb;
  wire  ram_io_axi_arbiter_io_output_w_payload_last;
  wire  ram_io_axi_arbiter_io_output_b_ready;
  wire  ram_io_axi_arbiter_io_output_r_ready;
  wire  apbBridge_io_axi_arbiter_io_sharedInputs_0_arw_ready;
  wire  apbBridge_io_axi_arbiter_io_sharedInputs_0_w_ready;
  wire  apbBridge_io_axi_arbiter_io_sharedInputs_0_b_valid;
  wire [3:0] apbBridge_io_axi_arbiter_io_sharedInputs_0_b_payload_id;
  wire [1:0] apbBridge_io_axi_arbiter_io_sharedInputs_0_b_payload_resp;
  wire  apbBridge_io_axi_arbiter_io_sharedInputs_0_r_valid;
  wire [31:0] apbBridge_io_axi_arbiter_io_sharedInputs_0_r_payload_data;
  wire [3:0] apbBridge_io_axi_arbiter_io_sharedInputs_0_r_payload_id;
  wire [1:0] apbBridge_io_axi_arbiter_io_sharedInputs_0_r_payload_resp;
  wire  apbBridge_io_axi_arbiter_io_sharedInputs_0_r_payload_last;
  wire  apbBridge_io_axi_arbiter_io_output_arw_valid;
  wire [19:0] apbBridge_io_axi_arbiter_io_output_arw_payload_addr;
  wire [3:0] apbBridge_io_axi_arbiter_io_output_arw_payload_id;
  wire [7:0] apbBridge_io_axi_arbiter_io_output_arw_payload_len;
  wire [2:0] apbBridge_io_axi_arbiter_io_output_arw_payload_size;
  wire [1:0] apbBridge_io_axi_arbiter_io_output_arw_payload_burst;
  wire  apbBridge_io_axi_arbiter_io_output_arw_payload_write;
  wire  apbBridge_io_axi_arbiter_io_output_w_valid;
  wire [31:0] apbBridge_io_axi_arbiter_io_output_w_payload_data;
  wire [3:0] apbBridge_io_axi_arbiter_io_output_w_payload_strb;
  wire  apbBridge_io_axi_arbiter_io_output_w_payload_last;
  wire  apbBridge_io_axi_arbiter_io_output_b_ready;
  wire  apbBridge_io_axi_arbiter_io_output_r_ready;
  wire  _zz_CpuComplex_148_;
  wire  _zz_CpuComplex_149_;
  wire  _zz_CpuComplex_150_;
  wire  _zz_CpuComplex_151_;
  wire  _zz_CpuComplex_152_;
  wire  _zz_CpuComplex_153_;
  wire  _zz_CpuComplex_154_;
  wire  _zz_CpuComplex_155_;
  wire  _zz_CpuComplex_156_;
  wire [6:0] _zz_CpuComplex_157_;
  wire  _zz_CpuComplex_1_;
  wire  core_iBus_ar_valid;
  wire  core_iBus_ar_ready;
  wire [31:0] core_iBus_ar_payload_addr;
  wire [3:0] core_iBus_ar_payload_cache;
  wire [2:0] core_iBus_ar_payload_prot;
  wire  core_iBus_r_valid;
  wire  core_iBus_r_ready;
  wire [31:0] core_iBus_r_payload_data;
  wire [1:0] core_iBus_r_payload_resp;
  wire  core_iBus_r_payload_last;
  wire  _zz_CpuComplex_2_;
  reg  _zz_CpuComplex_3_;
  reg [31:0] _zz_CpuComplex_4_;
  reg [3:0] _zz_CpuComplex_5_;
  reg [2:0] _zz_CpuComplex_6_;
  wire  _zz_CpuComplex_7_;
  wire  _zz_CpuComplex_8_;
  wire  _zz_CpuComplex_9_;
  wire  _zz_CpuComplex_10_;
  reg  _zz_CpuComplex_11_;
  reg  _zz_CpuComplex_12_;
  reg [2:0] _zz_CpuComplex_13_;
  reg [2:0] _zz_CpuComplex_14_;
  wire [2:0] _zz_CpuComplex_15_;
  wire  core_cpu_dBus_cmd_m2sPipe_valid;
  wire  core_cpu_dBus_cmd_m2sPipe_ready;
  wire  core_cpu_dBus_cmd_m2sPipe_payload_wr;
  wire [31:0] core_cpu_dBus_cmd_m2sPipe_payload_address;
  wire [31:0] core_cpu_dBus_cmd_m2sPipe_payload_data;
  wire [1:0] core_cpu_dBus_cmd_m2sPipe_payload_size;
  reg  _zz_CpuComplex_16_;
  reg  _zz_CpuComplex_17_;
  reg [31:0] _zz_CpuComplex_18_;
  reg [31:0] _zz_CpuComplex_19_;
  reg [1:0] _zz_CpuComplex_20_;
  wire  core_cpu_dBus_cmd_m2sPipe_m2sPipe_valid;
  wire  core_cpu_dBus_cmd_m2sPipe_m2sPipe_ready;
  wire  core_cpu_dBus_cmd_m2sPipe_m2sPipe_payload_wr;
  wire [31:0] core_cpu_dBus_cmd_m2sPipe_m2sPipe_payload_address;
  wire [31:0] core_cpu_dBus_cmd_m2sPipe_m2sPipe_payload_data;
  wire [1:0] core_cpu_dBus_cmd_m2sPipe_m2sPipe_payload_size;
  reg  _zz_CpuComplex_21_;
  reg  _zz_CpuComplex_22_;
  reg [31:0] _zz_CpuComplex_23_;
  reg [31:0] _zz_CpuComplex_24_;
  reg [1:0] _zz_CpuComplex_25_;
  wire  core_cpu_dBus_cmd_m2sPipe_m2sPipe_s2mPipe_valid;
  wire  core_cpu_dBus_cmd_m2sPipe_m2sPipe_s2mPipe_ready;
  wire  core_cpu_dBus_cmd_m2sPipe_m2sPipe_s2mPipe_payload_wr;
  wire [31:0] core_cpu_dBus_cmd_m2sPipe_m2sPipe_s2mPipe_payload_address;
  wire [31:0] core_cpu_dBus_cmd_m2sPipe_m2sPipe_s2mPipe_payload_data;
  wire [1:0] core_cpu_dBus_cmd_m2sPipe_m2sPipe_s2mPipe_payload_size;
  reg  _zz_CpuComplex_26_;
  reg  _zz_CpuComplex_27_;
  reg [31:0] _zz_CpuComplex_28_;
  reg [31:0] _zz_CpuComplex_29_;
  reg [1:0] _zz_CpuComplex_30_;
  wire  _zz_CpuComplex_31_;
  reg  _zz_CpuComplex_32_;
  reg [3:0] _zz_CpuComplex_33_;
  wire  core_dBus_arw_valid;
  wire  core_dBus_arw_ready;
  wire [31:0] core_dBus_arw_payload_addr;
  wire [2:0] core_dBus_arw_payload_size;
  wire [3:0] core_dBus_arw_payload_cache;
  wire [2:0] core_dBus_arw_payload_prot;
  wire  core_dBus_arw_payload_write;
  wire  core_dBus_w_valid;
  wire  core_dBus_w_ready;
  wire [31:0] core_dBus_w_payload_data;
  wire [3:0] core_dBus_w_payload_strb;
  wire  core_dBus_w_payload_last;
  wire  core_dBus_b_valid;
  wire  core_dBus_b_ready;
  wire [1:0] core_dBus_b_payload_resp;
  wire  core_dBus_r_valid;
  wire  core_dBus_r_ready;
  wire [31:0] core_dBus_r_payload_data;
  wire [1:0] core_dBus_r_payload_resp;
  wire  core_dBus_r_payload_last;
  wire  _zz_CpuComplex_34_;
  reg  _zz_CpuComplex_35_;
  reg [31:0] _zz_CpuComplex_36_;
  reg [2:0] _zz_CpuComplex_37_;
  reg [3:0] _zz_CpuComplex_38_;
  reg [2:0] _zz_CpuComplex_39_;
  reg  _zz_CpuComplex_40_;
  reg  _zz_CpuComplex_41_;
  wire  _zz_CpuComplex_42_;
  wire  _zz_CpuComplex_43_;
  reg  _zz_CpuComplex_44_;
  wire  _zz_CpuComplex_45_;
  wire  _zz_CpuComplex_46_;
  reg  _zz_CpuComplex_47_;
  wire  _zz_CpuComplex_48_;
  wire  _zz_CpuComplex_49_;
  reg  _zz_CpuComplex_50_;
  wire  _zz_CpuComplex_51_;
  wire  _zz_CpuComplex_52_;
  reg  _zz_CpuComplex_53_;
  wire  _zz_CpuComplex_54_;
  wire  _zz_CpuComplex_55_;
  reg  _zz_CpuComplex_56_;
  wire  _zz_CpuComplex_57_;
  wire  _zz_CpuComplex_58_;
  reg  _zz_CpuComplex_59_;
  wire  _zz_CpuComplex_60_;
  wire  _zz_CpuComplex_61_;
  reg  _zz_CpuComplex_62_;
  wire  core_dBus_decoder_io_input_r_m2sPipe_valid;
  wire  core_dBus_decoder_io_input_r_m2sPipe_ready;
  wire [31:0] core_dBus_decoder_io_input_r_m2sPipe_payload_data;
  wire [1:0] core_dBus_decoder_io_input_r_m2sPipe_payload_resp;
  wire  core_dBus_decoder_io_input_r_m2sPipe_payload_last;
  reg  _zz_CpuComplex_63_;
  reg [31:0] _zz_CpuComplex_64_;
  reg [1:0] _zz_CpuComplex_65_;
  reg  _zz_CpuComplex_66_;
  wire [2:0] _zz_CpuComplex_67_;
  wire [3:0] _zz_CpuComplex_68_;
  wire [7:0] _zz_CpuComplex_69_;
  wire [2:0] _zz_CpuComplex_70_;
  wire [3:0] _zz_CpuComplex_71_;
  wire [7:0] _zz_CpuComplex_72_;
  wire  io_axiMem1_arbiter_io_output_arw_halfPipe_valid;
  wire  io_axiMem1_arbiter_io_output_arw_halfPipe_ready;
  wire [31:0] io_axiMem1_arbiter_io_output_arw_halfPipe_payload_addr;
  wire [3:0] io_axiMem1_arbiter_io_output_arw_halfPipe_payload_id;
  wire [3:0] io_axiMem1_arbiter_io_output_arw_halfPipe_payload_region;
  wire [7:0] io_axiMem1_arbiter_io_output_arw_halfPipe_payload_len;
  wire [2:0] io_axiMem1_arbiter_io_output_arw_halfPipe_payload_size;
  wire [1:0] io_axiMem1_arbiter_io_output_arw_halfPipe_payload_burst;
  wire [0:0] io_axiMem1_arbiter_io_output_arw_halfPipe_payload_lock;
  wire [3:0] io_axiMem1_arbiter_io_output_arw_halfPipe_payload_cache;
  wire [3:0] io_axiMem1_arbiter_io_output_arw_halfPipe_payload_qos;
  wire [2:0] io_axiMem1_arbiter_io_output_arw_halfPipe_payload_prot;
  wire  io_axiMem1_arbiter_io_output_arw_halfPipe_payload_write;
  reg  io_axiMem1_arbiter_io_output_arw_halfPipe_regs_valid;
  reg  io_axiMem1_arbiter_io_output_arw_halfPipe_regs_ready;
  reg [31:0] io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_addr;
  reg [3:0] io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_id;
  reg [3:0] io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_region;
  reg [7:0] io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_len;
  reg [2:0] io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_size;
  reg [1:0] io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_burst;
  reg [0:0] io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_lock;
  reg [3:0] io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_cache;
  reg [3:0] io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_qos;
  reg [2:0] io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_prot;
  reg  io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_write;
  wire  io_axiMem1_arbiter_io_output_w_s2mPipe_valid;
  wire  io_axiMem1_arbiter_io_output_w_s2mPipe_ready;
  wire [31:0] io_axiMem1_arbiter_io_output_w_s2mPipe_payload_data;
  wire [3:0] io_axiMem1_arbiter_io_output_w_s2mPipe_payload_strb;
  wire  io_axiMem1_arbiter_io_output_w_s2mPipe_payload_last;
  reg  _zz_CpuComplex_73_;
  reg [31:0] _zz_CpuComplex_74_;
  reg [3:0] _zz_CpuComplex_75_;
  reg  _zz_CpuComplex_76_;
  wire  io_axiMem1_arbiter_io_output_w_s2mPipe_m2sPipe_valid;
  wire  io_axiMem1_arbiter_io_output_w_s2mPipe_m2sPipe_ready;
  wire [31:0] io_axiMem1_arbiter_io_output_w_s2mPipe_m2sPipe_payload_data;
  wire [3:0] io_axiMem1_arbiter_io_output_w_s2mPipe_m2sPipe_payload_strb;
  wire  io_axiMem1_arbiter_io_output_w_s2mPipe_m2sPipe_payload_last;
  reg  _zz_CpuComplex_77_;
  reg [31:0] _zz_CpuComplex_78_;
  reg [3:0] _zz_CpuComplex_79_;
  reg  _zz_CpuComplex_80_;
  wire [2:0] _zz_CpuComplex_81_;
  wire [3:0] _zz_CpuComplex_82_;
  wire [7:0] _zz_CpuComplex_83_;
  wire [2:0] _zz_CpuComplex_84_;
  wire [3:0] _zz_CpuComplex_85_;
  wire [7:0] _zz_CpuComplex_86_;
  wire  io_axiMem2_arbiter_io_output_arw_halfPipe_valid;
  wire  io_axiMem2_arbiter_io_output_arw_halfPipe_ready;
  wire [31:0] io_axiMem2_arbiter_io_output_arw_halfPipe_payload_addr;
  wire [3:0] io_axiMem2_arbiter_io_output_arw_halfPipe_payload_id;
  wire [3:0] io_axiMem2_arbiter_io_output_arw_halfPipe_payload_region;
  wire [7:0] io_axiMem2_arbiter_io_output_arw_halfPipe_payload_len;
  wire [2:0] io_axiMem2_arbiter_io_output_arw_halfPipe_payload_size;
  wire [1:0] io_axiMem2_arbiter_io_output_arw_halfPipe_payload_burst;
  wire [0:0] io_axiMem2_arbiter_io_output_arw_halfPipe_payload_lock;
  wire [3:0] io_axiMem2_arbiter_io_output_arw_halfPipe_payload_cache;
  wire [3:0] io_axiMem2_arbiter_io_output_arw_halfPipe_payload_qos;
  wire [2:0] io_axiMem2_arbiter_io_output_arw_halfPipe_payload_prot;
  wire  io_axiMem2_arbiter_io_output_arw_halfPipe_payload_write;
  reg  io_axiMem2_arbiter_io_output_arw_halfPipe_regs_valid;
  reg  io_axiMem2_arbiter_io_output_arw_halfPipe_regs_ready;
  reg [31:0] io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_addr;
  reg [3:0] io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_id;
  reg [3:0] io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_region;
  reg [7:0] io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_len;
  reg [2:0] io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_size;
  reg [1:0] io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_burst;
  reg [0:0] io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_lock;
  reg [3:0] io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_cache;
  reg [3:0] io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_qos;
  reg [2:0] io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_prot;
  reg  io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_write;
  wire  io_axiMem2_arbiter_io_output_w_s2mPipe_valid;
  wire  io_axiMem2_arbiter_io_output_w_s2mPipe_ready;
  wire [31:0] io_axiMem2_arbiter_io_output_w_s2mPipe_payload_data;
  wire [3:0] io_axiMem2_arbiter_io_output_w_s2mPipe_payload_strb;
  wire  io_axiMem2_arbiter_io_output_w_s2mPipe_payload_last;
  reg  _zz_CpuComplex_87_;
  reg [31:0] _zz_CpuComplex_88_;
  reg [3:0] _zz_CpuComplex_89_;
  reg  _zz_CpuComplex_90_;
  wire  io_axiMem2_arbiter_io_output_w_s2mPipe_m2sPipe_valid;
  wire  io_axiMem2_arbiter_io_output_w_s2mPipe_m2sPipe_ready;
  wire [31:0] io_axiMem2_arbiter_io_output_w_s2mPipe_m2sPipe_payload_data;
  wire [3:0] io_axiMem2_arbiter_io_output_w_s2mPipe_m2sPipe_payload_strb;
  wire  io_axiMem2_arbiter_io_output_w_s2mPipe_m2sPipe_payload_last;
  reg  _zz_CpuComplex_91_;
  reg [31:0] _zz_CpuComplex_92_;
  reg [3:0] _zz_CpuComplex_93_;
  reg  _zz_CpuComplex_94_;
  wire [2:0] _zz_CpuComplex_95_;
  wire [7:0] _zz_CpuComplex_96_;
  wire [2:0] _zz_CpuComplex_97_;
  wire [7:0] _zz_CpuComplex_98_;
  wire  ram_io_axi_arbiter_io_output_arw_halfPipe_valid;
  wire  ram_io_axi_arbiter_io_output_arw_halfPipe_ready;
  wire [12:0] ram_io_axi_arbiter_io_output_arw_halfPipe_payload_addr;
  wire [3:0] ram_io_axi_arbiter_io_output_arw_halfPipe_payload_id;
  wire [7:0] ram_io_axi_arbiter_io_output_arw_halfPipe_payload_len;
  wire [2:0] ram_io_axi_arbiter_io_output_arw_halfPipe_payload_size;
  wire [1:0] ram_io_axi_arbiter_io_output_arw_halfPipe_payload_burst;
  wire  ram_io_axi_arbiter_io_output_arw_halfPipe_payload_write;
  reg  ram_io_axi_arbiter_io_output_arw_halfPipe_regs_valid;
  reg  ram_io_axi_arbiter_io_output_arw_halfPipe_regs_ready;
  reg [12:0] ram_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_addr;
  reg [3:0] ram_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_id;
  reg [7:0] ram_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_len;
  reg [2:0] ram_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_size;
  reg [1:0] ram_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_burst;
  reg  ram_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_write;
  wire  ram_io_axi_arbiter_io_output_w_s2mPipe_valid;
  wire  ram_io_axi_arbiter_io_output_w_s2mPipe_ready;
  wire [31:0] ram_io_axi_arbiter_io_output_w_s2mPipe_payload_data;
  wire [3:0] ram_io_axi_arbiter_io_output_w_s2mPipe_payload_strb;
  wire  ram_io_axi_arbiter_io_output_w_s2mPipe_payload_last;
  reg  _zz_CpuComplex_99_;
  reg [31:0] _zz_CpuComplex_100_;
  reg [3:0] _zz_CpuComplex_101_;
  reg  _zz_CpuComplex_102_;
  wire  ram_io_axi_arbiter_io_output_w_s2mPipe_m2sPipe_valid;
  wire  ram_io_axi_arbiter_io_output_w_s2mPipe_m2sPipe_ready;
  wire [31:0] ram_io_axi_arbiter_io_output_w_s2mPipe_m2sPipe_payload_data;
  wire [3:0] ram_io_axi_arbiter_io_output_w_s2mPipe_m2sPipe_payload_strb;
  wire  ram_io_axi_arbiter_io_output_w_s2mPipe_m2sPipe_payload_last;
  reg  _zz_CpuComplex_103_;
  reg [31:0] _zz_CpuComplex_104_;
  reg [3:0] _zz_CpuComplex_105_;
  reg  _zz_CpuComplex_106_;
  wire [3:0] _zz_CpuComplex_107_;
  wire [7:0] _zz_CpuComplex_108_;
  wire  apbBridge_io_axi_arbiter_io_output_arw_halfPipe_valid;
  wire  apbBridge_io_axi_arbiter_io_output_arw_halfPipe_ready;
  wire [19:0] apbBridge_io_axi_arbiter_io_output_arw_halfPipe_payload_addr;
  wire [3:0] apbBridge_io_axi_arbiter_io_output_arw_halfPipe_payload_id;
  wire [7:0] apbBridge_io_axi_arbiter_io_output_arw_halfPipe_payload_len;
  wire [2:0] apbBridge_io_axi_arbiter_io_output_arw_halfPipe_payload_size;
  wire [1:0] apbBridge_io_axi_arbiter_io_output_arw_halfPipe_payload_burst;
  wire  apbBridge_io_axi_arbiter_io_output_arw_halfPipe_payload_write;
  reg  apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_valid;
  reg  apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_ready;
  reg [19:0] apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_addr;
  reg [3:0] apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_id;
  reg [7:0] apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_len;
  reg [2:0] apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_size;
  reg [1:0] apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_burst;
  reg  apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_write;
  wire  apbBridge_io_axi_arbiter_io_output_w_halfPipe_valid;
  wire  apbBridge_io_axi_arbiter_io_output_w_halfPipe_ready;
  wire [31:0] apbBridge_io_axi_arbiter_io_output_w_halfPipe_payload_data;
  wire [3:0] apbBridge_io_axi_arbiter_io_output_w_halfPipe_payload_strb;
  wire  apbBridge_io_axi_arbiter_io_output_w_halfPipe_payload_last;
  reg  apbBridge_io_axi_arbiter_io_output_w_halfPipe_regs_valid;
  reg  apbBridge_io_axi_arbiter_io_output_w_halfPipe_regs_ready;
  reg [31:0] apbBridge_io_axi_arbiter_io_output_w_halfPipe_regs_payload_data;
  reg [3:0] apbBridge_io_axi_arbiter_io_output_w_halfPipe_regs_payload_strb;
  reg  apbBridge_io_axi_arbiter_io_output_w_halfPipe_regs_payload_last;
  assign _zz_CpuComplex_148_ = (core_cpu_dBus_cmd_m2sPipe_m2sPipe_ready && (! core_cpu_dBus_cmd_m2sPipe_m2sPipe_s2mPipe_ready));
  assign _zz_CpuComplex_149_ = (! io_axiMem1_arbiter_io_output_arw_halfPipe_regs_valid);
  assign _zz_CpuComplex_150_ = (_zz_CpuComplex_131_ && (! io_axiMem1_arbiter_io_output_w_s2mPipe_ready));
  assign _zz_CpuComplex_151_ = (! io_axiMem2_arbiter_io_output_arw_halfPipe_regs_valid);
  assign _zz_CpuComplex_152_ = (_zz_CpuComplex_139_ && (! io_axiMem2_arbiter_io_output_w_s2mPipe_ready));
  assign _zz_CpuComplex_153_ = (! ram_io_axi_arbiter_io_output_arw_halfPipe_regs_valid);
  assign _zz_CpuComplex_154_ = (_zz_CpuComplex_145_ && (! ram_io_axi_arbiter_io_output_w_s2mPipe_ready));
  assign _zz_CpuComplex_155_ = (! apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_valid);
  assign _zz_CpuComplex_156_ = (! apbBridge_io_axi_arbiter_io_output_w_halfPipe_regs_valid);
  assign _zz_CpuComplex_157_ = ({3'd0,_zz_CpuComplex_33_} <<< streamFork_5__io_outputs_1_payload_address[1 : 0]);
  VexRiscv core_cpu ( 
    .iBus_cmd_valid(core_cpu_iBus_cmd_valid),
    .iBus_cmd_ready(_zz_CpuComplex_1_),
    .iBus_cmd_payload_pc(core_cpu_iBus_cmd_payload_pc),
    .iBus_rsp_valid(core_iBus_r_valid),
    .iBus_rsp_payload_error(_zz_CpuComplex_110_),
    .iBus_rsp_payload_inst(core_iBus_r_payload_data),
    .timerInterrupt(io_timerInterrupt),
    .externalInterrupt(io_externalInterrupt),
    .debug_bus_cmd_valid(systemDebugger_1__io_mem_cmd_valid),
    .debug_bus_cmd_ready(core_cpu_debug_bus_cmd_ready),
    .debug_bus_cmd_payload_wr(systemDebugger_1__io_mem_cmd_payload_wr),
    .debug_bus_cmd_payload_address(_zz_CpuComplex_111_),
    .debug_bus_cmd_payload_data(systemDebugger_1__io_mem_cmd_payload_data),
    .debug_bus_rsp_data(core_cpu_debug_bus_rsp_data),
    .debug_resetOut(core_cpu_debug_resetOut),
    .dBus_cmd_valid(core_cpu_dBus_cmd_valid),
    .dBus_cmd_ready(_zz_CpuComplex_112_),
    .dBus_cmd_payload_wr(core_cpu_dBus_cmd_payload_wr),
    .dBus_cmd_payload_address(core_cpu_dBus_cmd_payload_address),
    .dBus_cmd_payload_data(core_cpu_dBus_cmd_payload_data),
    .dBus_cmd_payload_size(core_cpu_dBus_cmd_payload_size),
    .dBus_rsp_ready(core_dBus_r_valid),
    .dBus_rsp_error(_zz_CpuComplex_113_),
    .dBus_rsp_data(core_dBus_r_payload_data),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_),
    ._zz_VexRiscv_151_(_zz_CpuComplex_109_) 
  );
  StreamFork_4_ streamFork_5_ ( 
    .io_input_valid(_zz_CpuComplex_114_),
    .io_input_ready(streamFork_5__io_input_ready),
    .io_input_payload_wr(core_cpu_dBus_cmd_m2sPipe_m2sPipe_s2mPipe_payload_wr),
    .io_input_payload_address(core_cpu_dBus_cmd_m2sPipe_m2sPipe_s2mPipe_payload_address),
    .io_input_payload_data(core_cpu_dBus_cmd_m2sPipe_m2sPipe_s2mPipe_payload_data),
    .io_input_payload_size(core_cpu_dBus_cmd_m2sPipe_m2sPipe_s2mPipe_payload_size),
    .io_outputs_0_valid(streamFork_5__io_outputs_0_valid),
    .io_outputs_0_ready(_zz_CpuComplex_8_),
    .io_outputs_0_payload_wr(streamFork_5__io_outputs_0_payload_wr),
    .io_outputs_0_payload_address(streamFork_5__io_outputs_0_payload_address),
    .io_outputs_0_payload_data(streamFork_5__io_outputs_0_payload_data),
    .io_outputs_0_payload_size(streamFork_5__io_outputs_0_payload_size),
    .io_outputs_1_valid(streamFork_5__io_outputs_1_valid),
    .io_outputs_1_ready(_zz_CpuComplex_115_),
    .io_outputs_1_payload_wr(streamFork_5__io_outputs_1_payload_wr),
    .io_outputs_1_payload_address(streamFork_5__io_outputs_1_payload_address),
    .io_outputs_1_payload_data(streamFork_5__io_outputs_1_payload_data),
    .io_outputs_1_payload_size(streamFork_5__io_outputs_1_payload_size),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  JtagBridge jtagBridge_1_ ( 
    .io_jtag_tms(io_jtag_tms),
    .io_jtag_tdi(io_jtag_tdi),
    .io_jtag_tdo(jtagBridge_1__io_jtag_tdo),
    .io_jtag_tck(io_jtag_tck),
    .io_remote_cmd_valid(jtagBridge_1__io_remote_cmd_valid),
    .io_remote_cmd_ready(systemDebugger_1__io_remote_cmd_ready),
    .io_remote_cmd_payload_last(jtagBridge_1__io_remote_cmd_payload_last),
    .io_remote_cmd_payload_fragment(jtagBridge_1__io_remote_cmd_payload_fragment),
    .io_remote_rsp_valid(systemDebugger_1__io_remote_rsp_valid),
    .io_remote_rsp_ready(jtagBridge_1__io_remote_rsp_ready),
    .io_remote_rsp_payload_error(systemDebugger_1__io_remote_rsp_payload_error),
    .io_remote_rsp_payload_data(systemDebugger_1__io_remote_rsp_payload_data),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  SystemDebugger systemDebugger_1_ ( 
    .io_remote_cmd_valid(jtagBridge_1__io_remote_cmd_valid),
    .io_remote_cmd_ready(systemDebugger_1__io_remote_cmd_ready),
    .io_remote_cmd_payload_last(jtagBridge_1__io_remote_cmd_payload_last),
    .io_remote_cmd_payload_fragment(jtagBridge_1__io_remote_cmd_payload_fragment),
    .io_remote_rsp_valid(systemDebugger_1__io_remote_rsp_valid),
    .io_remote_rsp_ready(jtagBridge_1__io_remote_rsp_ready),
    .io_remote_rsp_payload_error(systemDebugger_1__io_remote_rsp_payload_error),
    .io_remote_rsp_payload_data(systemDebugger_1__io_remote_rsp_payload_data),
    .io_mem_cmd_valid(systemDebugger_1__io_mem_cmd_valid),
    .io_mem_cmd_ready(core_cpu_debug_bus_cmd_ready),
    .io_mem_cmd_payload_address(systemDebugger_1__io_mem_cmd_payload_address),
    .io_mem_cmd_payload_data(systemDebugger_1__io_mem_cmd_payload_data),
    .io_mem_cmd_payload_wr(systemDebugger_1__io_mem_cmd_payload_wr),
    .io_mem_cmd_payload_size(systemDebugger_1__io_mem_cmd_payload_size),
    .io_mem_rsp_valid(_zz_CpuComplex_41_),
    .io_mem_rsp_payload(core_cpu_debug_bus_rsp_data),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  Axi4SharedOnChipRam ram ( 
    .io_axi_arw_valid(ram_io_axi_arbiter_io_output_arw_halfPipe_valid),
    .io_axi_arw_ready(ram_io_axi_arw_ready),
    .io_axi_arw_payload_addr(ram_io_axi_arbiter_io_output_arw_halfPipe_payload_addr),
    .io_axi_arw_payload_id(ram_io_axi_arbiter_io_output_arw_halfPipe_payload_id),
    .io_axi_arw_payload_len(ram_io_axi_arbiter_io_output_arw_halfPipe_payload_len),
    .io_axi_arw_payload_size(ram_io_axi_arbiter_io_output_arw_halfPipe_payload_size),
    .io_axi_arw_payload_burst(ram_io_axi_arbiter_io_output_arw_halfPipe_payload_burst),
    .io_axi_arw_payload_write(ram_io_axi_arbiter_io_output_arw_halfPipe_payload_write),
    .io_axi_w_valid(ram_io_axi_arbiter_io_output_w_s2mPipe_m2sPipe_valid),
    .io_axi_w_ready(ram_io_axi_w_ready),
    .io_axi_w_payload_data(ram_io_axi_arbiter_io_output_w_s2mPipe_m2sPipe_payload_data),
    .io_axi_w_payload_strb(ram_io_axi_arbiter_io_output_w_s2mPipe_m2sPipe_payload_strb),
    .io_axi_w_payload_last(ram_io_axi_arbiter_io_output_w_s2mPipe_m2sPipe_payload_last),
    .io_axi_b_valid(ram_io_axi_b_valid),
    .io_axi_b_ready(ram_io_axi_arbiter_io_output_b_ready),
    .io_axi_b_payload_id(ram_io_axi_b_payload_id),
    .io_axi_b_payload_resp(ram_io_axi_b_payload_resp),
    .io_axi_r_valid(ram_io_axi_r_valid),
    .io_axi_r_ready(ram_io_axi_arbiter_io_output_r_ready),
    .io_axi_r_payload_data(ram_io_axi_r_payload_data),
    .io_axi_r_payload_id(ram_io_axi_r_payload_id),
    .io_axi_r_payload_resp(ram_io_axi_r_payload_resp),
    .io_axi_r_payload_last(ram_io_axi_r_payload_last),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  Axi4SharedToApb3Bridge apbBridge ( 
    .io_axi_arw_valid(apbBridge_io_axi_arbiter_io_output_arw_halfPipe_valid),
    .io_axi_arw_ready(apbBridge_io_axi_arw_ready),
    .io_axi_arw_payload_addr(apbBridge_io_axi_arbiter_io_output_arw_halfPipe_payload_addr),
    .io_axi_arw_payload_id(apbBridge_io_axi_arbiter_io_output_arw_halfPipe_payload_id),
    .io_axi_arw_payload_len(apbBridge_io_axi_arbiter_io_output_arw_halfPipe_payload_len),
    .io_axi_arw_payload_size(apbBridge_io_axi_arbiter_io_output_arw_halfPipe_payload_size),
    .io_axi_arw_payload_burst(apbBridge_io_axi_arbiter_io_output_arw_halfPipe_payload_burst),
    .io_axi_arw_payload_write(apbBridge_io_axi_arbiter_io_output_arw_halfPipe_payload_write),
    .io_axi_w_valid(apbBridge_io_axi_arbiter_io_output_w_halfPipe_valid),
    .io_axi_w_ready(apbBridge_io_axi_w_ready),
    .io_axi_w_payload_data(apbBridge_io_axi_arbiter_io_output_w_halfPipe_payload_data),
    .io_axi_w_payload_strb(apbBridge_io_axi_arbiter_io_output_w_halfPipe_payload_strb),
    .io_axi_w_payload_last(apbBridge_io_axi_arbiter_io_output_w_halfPipe_payload_last),
    .io_axi_b_valid(apbBridge_io_axi_b_valid),
    .io_axi_b_ready(apbBridge_io_axi_arbiter_io_output_b_ready),
    .io_axi_b_payload_id(apbBridge_io_axi_b_payload_id),
    .io_axi_b_payload_resp(apbBridge_io_axi_b_payload_resp),
    .io_axi_r_valid(apbBridge_io_axi_r_valid),
    .io_axi_r_ready(apbBridge_io_axi_arbiter_io_output_r_ready),
    .io_axi_r_payload_data(apbBridge_io_axi_r_payload_data),
    .io_axi_r_payload_id(apbBridge_io_axi_r_payload_id),
    .io_axi_r_payload_resp(apbBridge_io_axi_r_payload_resp),
    .io_axi_r_payload_last(apbBridge_io_axi_r_payload_last),
    .io_apb_PADDR(apbBridge_io_apb_PADDR),
    .io_apb_PSEL(apbBridge_io_apb_PSEL),
    .io_apb_PENABLE(apbBridge_io_apb_PENABLE),
    .io_apb_PREADY(io_apb_PREADY),
    .io_apb_PWRITE(apbBridge_io_apb_PWRITE),
    .io_apb_PWDATA(apbBridge_io_apb_PWDATA),
    .io_apb_PRDATA(io_apb_PRDATA),
    .io_apb_PSLVERROR(io_apb_PSLVERROR),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  Axi4ReadOnlyDecoder core_iBus_decoder ( 
    .io_input_ar_valid(core_iBus_ar_valid),
    .io_input_ar_ready(core_iBus_decoder_io_input_ar_ready),
    .io_input_ar_payload_addr(core_iBus_ar_payload_addr),
    .io_input_ar_payload_cache(core_iBus_ar_payload_cache),
    .io_input_ar_payload_prot(core_iBus_ar_payload_prot),
    .io_input_r_valid(core_iBus_decoder_io_input_r_valid),
    .io_input_r_ready(core_iBus_r_ready),
    .io_input_r_payload_data(core_iBus_decoder_io_input_r_payload_data),
    .io_input_r_payload_resp(core_iBus_decoder_io_input_r_payload_resp),
    .io_input_r_payload_last(core_iBus_decoder_io_input_r_payload_last),
    .io_outputs_0_ar_valid(core_iBus_decoder_io_outputs_0_ar_valid),
    .io_outputs_0_ar_ready(_zz_CpuComplex_116_),
    .io_outputs_0_ar_payload_addr(core_iBus_decoder_io_outputs_0_ar_payload_addr),
    .io_outputs_0_ar_payload_cache(core_iBus_decoder_io_outputs_0_ar_payload_cache),
    .io_outputs_0_ar_payload_prot(core_iBus_decoder_io_outputs_0_ar_payload_prot),
    .io_outputs_0_r_valid(io_axiMem1_arbiter_io_readInputs_0_r_valid),
    .io_outputs_0_r_ready(core_iBus_decoder_io_outputs_0_r_ready),
    .io_outputs_0_r_payload_data(io_axiMem1_arbiter_io_readInputs_0_r_payload_data),
    .io_outputs_0_r_payload_resp(io_axiMem1_arbiter_io_readInputs_0_r_payload_resp),
    .io_outputs_0_r_payload_last(io_axiMem1_arbiter_io_readInputs_0_r_payload_last),
    .io_outputs_1_ar_valid(core_iBus_decoder_io_outputs_1_ar_valid),
    .io_outputs_1_ar_ready(_zz_CpuComplex_117_),
    .io_outputs_1_ar_payload_addr(core_iBus_decoder_io_outputs_1_ar_payload_addr),
    .io_outputs_1_ar_payload_cache(core_iBus_decoder_io_outputs_1_ar_payload_cache),
    .io_outputs_1_ar_payload_prot(core_iBus_decoder_io_outputs_1_ar_payload_prot),
    .io_outputs_1_r_valid(ram_io_axi_arbiter_io_readInputs_0_r_valid),
    .io_outputs_1_r_ready(core_iBus_decoder_io_outputs_1_r_ready),
    .io_outputs_1_r_payload_data(ram_io_axi_arbiter_io_readInputs_0_r_payload_data),
    .io_outputs_1_r_payload_resp(ram_io_axi_arbiter_io_readInputs_0_r_payload_resp),
    .io_outputs_1_r_payload_last(ram_io_axi_arbiter_io_readInputs_0_r_payload_last),
    .io_outputs_2_ar_valid(core_iBus_decoder_io_outputs_2_ar_valid),
    .io_outputs_2_ar_ready(_zz_CpuComplex_118_),
    .io_outputs_2_ar_payload_addr(core_iBus_decoder_io_outputs_2_ar_payload_addr),
    .io_outputs_2_ar_payload_cache(core_iBus_decoder_io_outputs_2_ar_payload_cache),
    .io_outputs_2_ar_payload_prot(core_iBus_decoder_io_outputs_2_ar_payload_prot),
    .io_outputs_2_r_valid(io_axiMem2_arbiter_io_readInputs_0_r_valid),
    .io_outputs_2_r_ready(core_iBus_decoder_io_outputs_2_r_ready),
    .io_outputs_2_r_payload_data(io_axiMem2_arbiter_io_readInputs_0_r_payload_data),
    .io_outputs_2_r_payload_resp(io_axiMem2_arbiter_io_readInputs_0_r_payload_resp),
    .io_outputs_2_r_payload_last(io_axiMem2_arbiter_io_readInputs_0_r_payload_last),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  Axi4SharedDecoder core_dBus_decoder ( 
    .io_input_arw_valid(core_dBus_arw_valid),
    .io_input_arw_ready(core_dBus_decoder_io_input_arw_ready),
    .io_input_arw_payload_addr(core_dBus_arw_payload_addr),
    .io_input_arw_payload_size(core_dBus_arw_payload_size),
    .io_input_arw_payload_cache(core_dBus_arw_payload_cache),
    .io_input_arw_payload_prot(core_dBus_arw_payload_prot),
    .io_input_arw_payload_write(core_dBus_arw_payload_write),
    .io_input_w_valid(core_dBus_w_valid),
    .io_input_w_ready(core_dBus_decoder_io_input_w_ready),
    .io_input_w_payload_data(core_dBus_w_payload_data),
    .io_input_w_payload_strb(core_dBus_w_payload_strb),
    .io_input_w_payload_last(core_dBus_w_payload_last),
    .io_input_b_valid(core_dBus_decoder_io_input_b_valid),
    .io_input_b_ready(core_dBus_b_ready),
    .io_input_b_payload_resp(core_dBus_decoder_io_input_b_payload_resp),
    .io_input_r_valid(core_dBus_decoder_io_input_r_valid),
    .io_input_r_ready(_zz_CpuComplex_119_),
    .io_input_r_payload_data(core_dBus_decoder_io_input_r_payload_data),
    .io_input_r_payload_resp(core_dBus_decoder_io_input_r_payload_resp),
    .io_input_r_payload_last(core_dBus_decoder_io_input_r_payload_last),
    .io_sharedOutputs_0_arw_valid(core_dBus_decoder_io_sharedOutputs_0_arw_valid),
    .io_sharedOutputs_0_arw_ready(_zz_CpuComplex_120_),
    .io_sharedOutputs_0_arw_payload_addr(core_dBus_decoder_io_sharedOutputs_0_arw_payload_addr),
    .io_sharedOutputs_0_arw_payload_size(core_dBus_decoder_io_sharedOutputs_0_arw_payload_size),
    .io_sharedOutputs_0_arw_payload_cache(core_dBus_decoder_io_sharedOutputs_0_arw_payload_cache),
    .io_sharedOutputs_0_arw_payload_prot(core_dBus_decoder_io_sharedOutputs_0_arw_payload_prot),
    .io_sharedOutputs_0_arw_payload_write(core_dBus_decoder_io_sharedOutputs_0_arw_payload_write),
    .io_sharedOutputs_0_w_valid(core_dBus_decoder_io_sharedOutputs_0_w_valid),
    .io_sharedOutputs_0_w_ready(io_axiMem1_arbiter_io_sharedInputs_0_w_ready),
    .io_sharedOutputs_0_w_payload_data(core_dBus_decoder_io_sharedOutputs_0_w_payload_data),
    .io_sharedOutputs_0_w_payload_strb(core_dBus_decoder_io_sharedOutputs_0_w_payload_strb),
    .io_sharedOutputs_0_w_payload_last(core_dBus_decoder_io_sharedOutputs_0_w_payload_last),
    .io_sharedOutputs_0_b_valid(io_axiMem1_arbiter_io_sharedInputs_0_b_valid),
    .io_sharedOutputs_0_b_ready(core_dBus_decoder_io_sharedOutputs_0_b_ready),
    .io_sharedOutputs_0_b_payload_resp(io_axiMem1_arbiter_io_sharedInputs_0_b_payload_resp),
    .io_sharedOutputs_0_r_valid(io_axiMem1_arbiter_io_sharedInputs_0_r_valid),
    .io_sharedOutputs_0_r_ready(core_dBus_decoder_io_sharedOutputs_0_r_ready),
    .io_sharedOutputs_0_r_payload_data(io_axiMem1_arbiter_io_sharedInputs_0_r_payload_data),
    .io_sharedOutputs_0_r_payload_resp(io_axiMem1_arbiter_io_sharedInputs_0_r_payload_resp),
    .io_sharedOutputs_0_r_payload_last(io_axiMem1_arbiter_io_sharedInputs_0_r_payload_last),
    .io_sharedOutputs_1_arw_valid(core_dBus_decoder_io_sharedOutputs_1_arw_valid),
    .io_sharedOutputs_1_arw_ready(_zz_CpuComplex_121_),
    .io_sharedOutputs_1_arw_payload_addr(core_dBus_decoder_io_sharedOutputs_1_arw_payload_addr),
    .io_sharedOutputs_1_arw_payload_size(core_dBus_decoder_io_sharedOutputs_1_arw_payload_size),
    .io_sharedOutputs_1_arw_payload_cache(core_dBus_decoder_io_sharedOutputs_1_arw_payload_cache),
    .io_sharedOutputs_1_arw_payload_prot(core_dBus_decoder_io_sharedOutputs_1_arw_payload_prot),
    .io_sharedOutputs_1_arw_payload_write(core_dBus_decoder_io_sharedOutputs_1_arw_payload_write),
    .io_sharedOutputs_1_w_valid(core_dBus_decoder_io_sharedOutputs_1_w_valid),
    .io_sharedOutputs_1_w_ready(ram_io_axi_arbiter_io_sharedInputs_0_w_ready),
    .io_sharedOutputs_1_w_payload_data(core_dBus_decoder_io_sharedOutputs_1_w_payload_data),
    .io_sharedOutputs_1_w_payload_strb(core_dBus_decoder_io_sharedOutputs_1_w_payload_strb),
    .io_sharedOutputs_1_w_payload_last(core_dBus_decoder_io_sharedOutputs_1_w_payload_last),
    .io_sharedOutputs_1_b_valid(ram_io_axi_arbiter_io_sharedInputs_0_b_valid),
    .io_sharedOutputs_1_b_ready(core_dBus_decoder_io_sharedOutputs_1_b_ready),
    .io_sharedOutputs_1_b_payload_resp(ram_io_axi_arbiter_io_sharedInputs_0_b_payload_resp),
    .io_sharedOutputs_1_r_valid(ram_io_axi_arbiter_io_sharedInputs_0_r_valid),
    .io_sharedOutputs_1_r_ready(core_dBus_decoder_io_sharedOutputs_1_r_ready),
    .io_sharedOutputs_1_r_payload_data(ram_io_axi_arbiter_io_sharedInputs_0_r_payload_data),
    .io_sharedOutputs_1_r_payload_resp(ram_io_axi_arbiter_io_sharedInputs_0_r_payload_resp),
    .io_sharedOutputs_1_r_payload_last(ram_io_axi_arbiter_io_sharedInputs_0_r_payload_last),
    .io_sharedOutputs_2_arw_valid(core_dBus_decoder_io_sharedOutputs_2_arw_valid),
    .io_sharedOutputs_2_arw_ready(_zz_CpuComplex_122_),
    .io_sharedOutputs_2_arw_payload_addr(core_dBus_decoder_io_sharedOutputs_2_arw_payload_addr),
    .io_sharedOutputs_2_arw_payload_size(core_dBus_decoder_io_sharedOutputs_2_arw_payload_size),
    .io_sharedOutputs_2_arw_payload_cache(core_dBus_decoder_io_sharedOutputs_2_arw_payload_cache),
    .io_sharedOutputs_2_arw_payload_prot(core_dBus_decoder_io_sharedOutputs_2_arw_payload_prot),
    .io_sharedOutputs_2_arw_payload_write(core_dBus_decoder_io_sharedOutputs_2_arw_payload_write),
    .io_sharedOutputs_2_w_valid(core_dBus_decoder_io_sharedOutputs_2_w_valid),
    .io_sharedOutputs_2_w_ready(io_axiMem2_arbiter_io_sharedInputs_0_w_ready),
    .io_sharedOutputs_2_w_payload_data(core_dBus_decoder_io_sharedOutputs_2_w_payload_data),
    .io_sharedOutputs_2_w_payload_strb(core_dBus_decoder_io_sharedOutputs_2_w_payload_strb),
    .io_sharedOutputs_2_w_payload_last(core_dBus_decoder_io_sharedOutputs_2_w_payload_last),
    .io_sharedOutputs_2_b_valid(io_axiMem2_arbiter_io_sharedInputs_0_b_valid),
    .io_sharedOutputs_2_b_ready(core_dBus_decoder_io_sharedOutputs_2_b_ready),
    .io_sharedOutputs_2_b_payload_resp(io_axiMem2_arbiter_io_sharedInputs_0_b_payload_resp),
    .io_sharedOutputs_2_r_valid(io_axiMem2_arbiter_io_sharedInputs_0_r_valid),
    .io_sharedOutputs_2_r_ready(core_dBus_decoder_io_sharedOutputs_2_r_ready),
    .io_sharedOutputs_2_r_payload_data(io_axiMem2_arbiter_io_sharedInputs_0_r_payload_data),
    .io_sharedOutputs_2_r_payload_resp(io_axiMem2_arbiter_io_sharedInputs_0_r_payload_resp),
    .io_sharedOutputs_2_r_payload_last(io_axiMem2_arbiter_io_sharedInputs_0_r_payload_last),
    .io_sharedOutputs_3_arw_valid(core_dBus_decoder_io_sharedOutputs_3_arw_valid),
    .io_sharedOutputs_3_arw_ready(_zz_CpuComplex_123_),
    .io_sharedOutputs_3_arw_payload_addr(core_dBus_decoder_io_sharedOutputs_3_arw_payload_addr),
    .io_sharedOutputs_3_arw_payload_size(core_dBus_decoder_io_sharedOutputs_3_arw_payload_size),
    .io_sharedOutputs_3_arw_payload_cache(core_dBus_decoder_io_sharedOutputs_3_arw_payload_cache),
    .io_sharedOutputs_3_arw_payload_prot(core_dBus_decoder_io_sharedOutputs_3_arw_payload_prot),
    .io_sharedOutputs_3_arw_payload_write(core_dBus_decoder_io_sharedOutputs_3_arw_payload_write),
    .io_sharedOutputs_3_w_valid(core_dBus_decoder_io_sharedOutputs_3_w_valid),
    .io_sharedOutputs_3_w_ready(apbBridge_io_axi_arbiter_io_sharedInputs_0_w_ready),
    .io_sharedOutputs_3_w_payload_data(core_dBus_decoder_io_sharedOutputs_3_w_payload_data),
    .io_sharedOutputs_3_w_payload_strb(core_dBus_decoder_io_sharedOutputs_3_w_payload_strb),
    .io_sharedOutputs_3_w_payload_last(core_dBus_decoder_io_sharedOutputs_3_w_payload_last),
    .io_sharedOutputs_3_b_valid(apbBridge_io_axi_arbiter_io_sharedInputs_0_b_valid),
    .io_sharedOutputs_3_b_ready(core_dBus_decoder_io_sharedOutputs_3_b_ready),
    .io_sharedOutputs_3_b_payload_resp(apbBridge_io_axi_arbiter_io_sharedInputs_0_b_payload_resp),
    .io_sharedOutputs_3_r_valid(apbBridge_io_axi_arbiter_io_sharedInputs_0_r_valid),
    .io_sharedOutputs_3_r_ready(core_dBus_decoder_io_sharedOutputs_3_r_ready),
    .io_sharedOutputs_3_r_payload_data(apbBridge_io_axi_arbiter_io_sharedInputs_0_r_payload_data),
    .io_sharedOutputs_3_r_payload_resp(apbBridge_io_axi_arbiter_io_sharedInputs_0_r_payload_resp),
    .io_sharedOutputs_3_r_payload_last(apbBridge_io_axi_arbiter_io_sharedInputs_0_r_payload_last),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  Axi4SharedArbiter io_axiMem1_arbiter ( 
    .io_readInputs_0_ar_valid(_zz_CpuComplex_42_),
    .io_readInputs_0_ar_ready(io_axiMem1_arbiter_io_readInputs_0_ar_ready),
    .io_readInputs_0_ar_payload_addr(core_iBus_decoder_io_outputs_0_ar_payload_addr),
    .io_readInputs_0_ar_payload_id(_zz_CpuComplex_67_),
    .io_readInputs_0_ar_payload_region(_zz_CpuComplex_68_),
    .io_readInputs_0_ar_payload_len(_zz_CpuComplex_69_),
    .io_readInputs_0_ar_payload_size(_zz_CpuComplex_124_),
    .io_readInputs_0_ar_payload_burst(_zz_CpuComplex_125_),
    .io_readInputs_0_ar_payload_lock(_zz_CpuComplex_126_),
    .io_readInputs_0_ar_payload_cache(core_iBus_decoder_io_outputs_0_ar_payload_cache),
    .io_readInputs_0_ar_payload_qos(_zz_CpuComplex_127_),
    .io_readInputs_0_ar_payload_prot(core_iBus_decoder_io_outputs_0_ar_payload_prot),
    .io_readInputs_0_r_valid(io_axiMem1_arbiter_io_readInputs_0_r_valid),
    .io_readInputs_0_r_ready(core_iBus_decoder_io_outputs_0_r_ready),
    .io_readInputs_0_r_payload_data(io_axiMem1_arbiter_io_readInputs_0_r_payload_data),
    .io_readInputs_0_r_payload_id(io_axiMem1_arbiter_io_readInputs_0_r_payload_id),
    .io_readInputs_0_r_payload_resp(io_axiMem1_arbiter_io_readInputs_0_r_payload_resp),
    .io_readInputs_0_r_payload_last(io_axiMem1_arbiter_io_readInputs_0_r_payload_last),
    .io_sharedInputs_0_arw_valid(_zz_CpuComplex_51_),
    .io_sharedInputs_0_arw_ready(io_axiMem1_arbiter_io_sharedInputs_0_arw_ready),
    .io_sharedInputs_0_arw_payload_addr(core_dBus_decoder_io_sharedOutputs_0_arw_payload_addr),
    .io_sharedInputs_0_arw_payload_id(_zz_CpuComplex_70_),
    .io_sharedInputs_0_arw_payload_region(_zz_CpuComplex_71_),
    .io_sharedInputs_0_arw_payload_len(_zz_CpuComplex_72_),
    .io_sharedInputs_0_arw_payload_size(core_dBus_decoder_io_sharedOutputs_0_arw_payload_size),
    .io_sharedInputs_0_arw_payload_burst(_zz_CpuComplex_128_),
    .io_sharedInputs_0_arw_payload_lock(_zz_CpuComplex_129_),
    .io_sharedInputs_0_arw_payload_cache(core_dBus_decoder_io_sharedOutputs_0_arw_payload_cache),
    .io_sharedInputs_0_arw_payload_qos(_zz_CpuComplex_130_),
    .io_sharedInputs_0_arw_payload_prot(core_dBus_decoder_io_sharedOutputs_0_arw_payload_prot),
    .io_sharedInputs_0_arw_payload_write(core_dBus_decoder_io_sharedOutputs_0_arw_payload_write),
    .io_sharedInputs_0_w_valid(core_dBus_decoder_io_sharedOutputs_0_w_valid),
    .io_sharedInputs_0_w_ready(io_axiMem1_arbiter_io_sharedInputs_0_w_ready),
    .io_sharedInputs_0_w_payload_data(core_dBus_decoder_io_sharedOutputs_0_w_payload_data),
    .io_sharedInputs_0_w_payload_strb(core_dBus_decoder_io_sharedOutputs_0_w_payload_strb),
    .io_sharedInputs_0_w_payload_last(core_dBus_decoder_io_sharedOutputs_0_w_payload_last),
    .io_sharedInputs_0_b_valid(io_axiMem1_arbiter_io_sharedInputs_0_b_valid),
    .io_sharedInputs_0_b_ready(core_dBus_decoder_io_sharedOutputs_0_b_ready),
    .io_sharedInputs_0_b_payload_id(io_axiMem1_arbiter_io_sharedInputs_0_b_payload_id),
    .io_sharedInputs_0_b_payload_resp(io_axiMem1_arbiter_io_sharedInputs_0_b_payload_resp),
    .io_sharedInputs_0_r_valid(io_axiMem1_arbiter_io_sharedInputs_0_r_valid),
    .io_sharedInputs_0_r_ready(core_dBus_decoder_io_sharedOutputs_0_r_ready),
    .io_sharedInputs_0_r_payload_data(io_axiMem1_arbiter_io_sharedInputs_0_r_payload_data),
    .io_sharedInputs_0_r_payload_id(io_axiMem1_arbiter_io_sharedInputs_0_r_payload_id),
    .io_sharedInputs_0_r_payload_resp(io_axiMem1_arbiter_io_sharedInputs_0_r_payload_resp),
    .io_sharedInputs_0_r_payload_last(io_axiMem1_arbiter_io_sharedInputs_0_r_payload_last),
    .io_output_arw_valid(io_axiMem1_arbiter_io_output_arw_valid),
    .io_output_arw_ready(io_axiMem1_arbiter_io_output_arw_halfPipe_regs_ready),
    .io_output_arw_payload_addr(io_axiMem1_arbiter_io_output_arw_payload_addr),
    .io_output_arw_payload_id(io_axiMem1_arbiter_io_output_arw_payload_id),
    .io_output_arw_payload_region(io_axiMem1_arbiter_io_output_arw_payload_region),
    .io_output_arw_payload_len(io_axiMem1_arbiter_io_output_arw_payload_len),
    .io_output_arw_payload_size(io_axiMem1_arbiter_io_output_arw_payload_size),
    .io_output_arw_payload_burst(io_axiMem1_arbiter_io_output_arw_payload_burst),
    .io_output_arw_payload_lock(io_axiMem1_arbiter_io_output_arw_payload_lock),
    .io_output_arw_payload_cache(io_axiMem1_arbiter_io_output_arw_payload_cache),
    .io_output_arw_payload_qos(io_axiMem1_arbiter_io_output_arw_payload_qos),
    .io_output_arw_payload_prot(io_axiMem1_arbiter_io_output_arw_payload_prot),
    .io_output_arw_payload_write(io_axiMem1_arbiter_io_output_arw_payload_write),
    .io_output_w_valid(io_axiMem1_arbiter_io_output_w_valid),
    .io_output_w_ready(_zz_CpuComplex_131_),
    .io_output_w_payload_data(io_axiMem1_arbiter_io_output_w_payload_data),
    .io_output_w_payload_strb(io_axiMem1_arbiter_io_output_w_payload_strb),
    .io_output_w_payload_last(io_axiMem1_arbiter_io_output_w_payload_last),
    .io_output_b_valid(io_axiMem1_b_valid),
    .io_output_b_ready(io_axiMem1_arbiter_io_output_b_ready),
    .io_output_b_payload_id(io_axiMem1_b_payload_id),
    .io_output_b_payload_resp(io_axiMem1_b_payload_resp),
    .io_output_r_valid(io_axiMem1_r_valid),
    .io_output_r_ready(io_axiMem1_arbiter_io_output_r_ready),
    .io_output_r_payload_data(io_axiMem1_r_payload_data),
    .io_output_r_payload_id(io_axiMem1_r_payload_id),
    .io_output_r_payload_resp(io_axiMem1_r_payload_resp),
    .io_output_r_payload_last(io_axiMem1_r_payload_last),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  Axi4SharedArbiter_1_ io_axiMem2_arbiter ( 
    .io_readInputs_0_ar_valid(_zz_CpuComplex_48_),
    .io_readInputs_0_ar_ready(io_axiMem2_arbiter_io_readInputs_0_ar_ready),
    .io_readInputs_0_ar_payload_addr(core_iBus_decoder_io_outputs_2_ar_payload_addr),
    .io_readInputs_0_ar_payload_id(_zz_CpuComplex_81_),
    .io_readInputs_0_ar_payload_region(_zz_CpuComplex_82_),
    .io_readInputs_0_ar_payload_len(_zz_CpuComplex_83_),
    .io_readInputs_0_ar_payload_size(_zz_CpuComplex_132_),
    .io_readInputs_0_ar_payload_burst(_zz_CpuComplex_133_),
    .io_readInputs_0_ar_payload_lock(_zz_CpuComplex_134_),
    .io_readInputs_0_ar_payload_cache(core_iBus_decoder_io_outputs_2_ar_payload_cache),
    .io_readInputs_0_ar_payload_qos(_zz_CpuComplex_135_),
    .io_readInputs_0_ar_payload_prot(core_iBus_decoder_io_outputs_2_ar_payload_prot),
    .io_readInputs_0_r_valid(io_axiMem2_arbiter_io_readInputs_0_r_valid),
    .io_readInputs_0_r_ready(core_iBus_decoder_io_outputs_2_r_ready),
    .io_readInputs_0_r_payload_data(io_axiMem2_arbiter_io_readInputs_0_r_payload_data),
    .io_readInputs_0_r_payload_id(io_axiMem2_arbiter_io_readInputs_0_r_payload_id),
    .io_readInputs_0_r_payload_resp(io_axiMem2_arbiter_io_readInputs_0_r_payload_resp),
    .io_readInputs_0_r_payload_last(io_axiMem2_arbiter_io_readInputs_0_r_payload_last),
    .io_sharedInputs_0_arw_valid(_zz_CpuComplex_57_),
    .io_sharedInputs_0_arw_ready(io_axiMem2_arbiter_io_sharedInputs_0_arw_ready),
    .io_sharedInputs_0_arw_payload_addr(core_dBus_decoder_io_sharedOutputs_2_arw_payload_addr),
    .io_sharedInputs_0_arw_payload_id(_zz_CpuComplex_84_),
    .io_sharedInputs_0_arw_payload_region(_zz_CpuComplex_85_),
    .io_sharedInputs_0_arw_payload_len(_zz_CpuComplex_86_),
    .io_sharedInputs_0_arw_payload_size(core_dBus_decoder_io_sharedOutputs_2_arw_payload_size),
    .io_sharedInputs_0_arw_payload_burst(_zz_CpuComplex_136_),
    .io_sharedInputs_0_arw_payload_lock(_zz_CpuComplex_137_),
    .io_sharedInputs_0_arw_payload_cache(core_dBus_decoder_io_sharedOutputs_2_arw_payload_cache),
    .io_sharedInputs_0_arw_payload_qos(_zz_CpuComplex_138_),
    .io_sharedInputs_0_arw_payload_prot(core_dBus_decoder_io_sharedOutputs_2_arw_payload_prot),
    .io_sharedInputs_0_arw_payload_write(core_dBus_decoder_io_sharedOutputs_2_arw_payload_write),
    .io_sharedInputs_0_w_valid(core_dBus_decoder_io_sharedOutputs_2_w_valid),
    .io_sharedInputs_0_w_ready(io_axiMem2_arbiter_io_sharedInputs_0_w_ready),
    .io_sharedInputs_0_w_payload_data(core_dBus_decoder_io_sharedOutputs_2_w_payload_data),
    .io_sharedInputs_0_w_payload_strb(core_dBus_decoder_io_sharedOutputs_2_w_payload_strb),
    .io_sharedInputs_0_w_payload_last(core_dBus_decoder_io_sharedOutputs_2_w_payload_last),
    .io_sharedInputs_0_b_valid(io_axiMem2_arbiter_io_sharedInputs_0_b_valid),
    .io_sharedInputs_0_b_ready(core_dBus_decoder_io_sharedOutputs_2_b_ready),
    .io_sharedInputs_0_b_payload_id(io_axiMem2_arbiter_io_sharedInputs_0_b_payload_id),
    .io_sharedInputs_0_b_payload_resp(io_axiMem2_arbiter_io_sharedInputs_0_b_payload_resp),
    .io_sharedInputs_0_r_valid(io_axiMem2_arbiter_io_sharedInputs_0_r_valid),
    .io_sharedInputs_0_r_ready(core_dBus_decoder_io_sharedOutputs_2_r_ready),
    .io_sharedInputs_0_r_payload_data(io_axiMem2_arbiter_io_sharedInputs_0_r_payload_data),
    .io_sharedInputs_0_r_payload_id(io_axiMem2_arbiter_io_sharedInputs_0_r_payload_id),
    .io_sharedInputs_0_r_payload_resp(io_axiMem2_arbiter_io_sharedInputs_0_r_payload_resp),
    .io_sharedInputs_0_r_payload_last(io_axiMem2_arbiter_io_sharedInputs_0_r_payload_last),
    .io_output_arw_valid(io_axiMem2_arbiter_io_output_arw_valid),
    .io_output_arw_ready(io_axiMem2_arbiter_io_output_arw_halfPipe_regs_ready),
    .io_output_arw_payload_addr(io_axiMem2_arbiter_io_output_arw_payload_addr),
    .io_output_arw_payload_id(io_axiMem2_arbiter_io_output_arw_payload_id),
    .io_output_arw_payload_region(io_axiMem2_arbiter_io_output_arw_payload_region),
    .io_output_arw_payload_len(io_axiMem2_arbiter_io_output_arw_payload_len),
    .io_output_arw_payload_size(io_axiMem2_arbiter_io_output_arw_payload_size),
    .io_output_arw_payload_burst(io_axiMem2_arbiter_io_output_arw_payload_burst),
    .io_output_arw_payload_lock(io_axiMem2_arbiter_io_output_arw_payload_lock),
    .io_output_arw_payload_cache(io_axiMem2_arbiter_io_output_arw_payload_cache),
    .io_output_arw_payload_qos(io_axiMem2_arbiter_io_output_arw_payload_qos),
    .io_output_arw_payload_prot(io_axiMem2_arbiter_io_output_arw_payload_prot),
    .io_output_arw_payload_write(io_axiMem2_arbiter_io_output_arw_payload_write),
    .io_output_w_valid(io_axiMem2_arbiter_io_output_w_valid),
    .io_output_w_ready(_zz_CpuComplex_139_),
    .io_output_w_payload_data(io_axiMem2_arbiter_io_output_w_payload_data),
    .io_output_w_payload_strb(io_axiMem2_arbiter_io_output_w_payload_strb),
    .io_output_w_payload_last(io_axiMem2_arbiter_io_output_w_payload_last),
    .io_output_b_valid(io_axiMem2_b_valid),
    .io_output_b_ready(io_axiMem2_arbiter_io_output_b_ready),
    .io_output_b_payload_id(io_axiMem2_b_payload_id),
    .io_output_b_payload_resp(io_axiMem2_b_payload_resp),
    .io_output_r_valid(io_axiMem2_r_valid),
    .io_output_r_ready(io_axiMem2_arbiter_io_output_r_ready),
    .io_output_r_payload_data(io_axiMem2_r_payload_data),
    .io_output_r_payload_id(io_axiMem2_r_payload_id),
    .io_output_r_payload_resp(io_axiMem2_r_payload_resp),
    .io_output_r_payload_last(io_axiMem2_r_payload_last),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  Axi4SharedArbiter_2_ ram_io_axi_arbiter ( 
    .io_readInputs_0_ar_valid(_zz_CpuComplex_45_),
    .io_readInputs_0_ar_ready(ram_io_axi_arbiter_io_readInputs_0_ar_ready),
    .io_readInputs_0_ar_payload_addr(_zz_CpuComplex_140_),
    .io_readInputs_0_ar_payload_id(_zz_CpuComplex_95_),
    .io_readInputs_0_ar_payload_len(_zz_CpuComplex_96_),
    .io_readInputs_0_ar_payload_size(_zz_CpuComplex_141_),
    .io_readInputs_0_ar_payload_burst(_zz_CpuComplex_142_),
    .io_readInputs_0_r_valid(ram_io_axi_arbiter_io_readInputs_0_r_valid),
    .io_readInputs_0_r_ready(core_iBus_decoder_io_outputs_1_r_ready),
    .io_readInputs_0_r_payload_data(ram_io_axi_arbiter_io_readInputs_0_r_payload_data),
    .io_readInputs_0_r_payload_id(ram_io_axi_arbiter_io_readInputs_0_r_payload_id),
    .io_readInputs_0_r_payload_resp(ram_io_axi_arbiter_io_readInputs_0_r_payload_resp),
    .io_readInputs_0_r_payload_last(ram_io_axi_arbiter_io_readInputs_0_r_payload_last),
    .io_sharedInputs_0_arw_valid(_zz_CpuComplex_54_),
    .io_sharedInputs_0_arw_ready(ram_io_axi_arbiter_io_sharedInputs_0_arw_ready),
    .io_sharedInputs_0_arw_payload_addr(_zz_CpuComplex_143_),
    .io_sharedInputs_0_arw_payload_id(_zz_CpuComplex_97_),
    .io_sharedInputs_0_arw_payload_len(_zz_CpuComplex_98_),
    .io_sharedInputs_0_arw_payload_size(core_dBus_decoder_io_sharedOutputs_1_arw_payload_size),
    .io_sharedInputs_0_arw_payload_burst(_zz_CpuComplex_144_),
    .io_sharedInputs_0_arw_payload_write(core_dBus_decoder_io_sharedOutputs_1_arw_payload_write),
    .io_sharedInputs_0_w_valid(core_dBus_decoder_io_sharedOutputs_1_w_valid),
    .io_sharedInputs_0_w_ready(ram_io_axi_arbiter_io_sharedInputs_0_w_ready),
    .io_sharedInputs_0_w_payload_data(core_dBus_decoder_io_sharedOutputs_1_w_payload_data),
    .io_sharedInputs_0_w_payload_strb(core_dBus_decoder_io_sharedOutputs_1_w_payload_strb),
    .io_sharedInputs_0_w_payload_last(core_dBus_decoder_io_sharedOutputs_1_w_payload_last),
    .io_sharedInputs_0_b_valid(ram_io_axi_arbiter_io_sharedInputs_0_b_valid),
    .io_sharedInputs_0_b_ready(core_dBus_decoder_io_sharedOutputs_1_b_ready),
    .io_sharedInputs_0_b_payload_id(ram_io_axi_arbiter_io_sharedInputs_0_b_payload_id),
    .io_sharedInputs_0_b_payload_resp(ram_io_axi_arbiter_io_sharedInputs_0_b_payload_resp),
    .io_sharedInputs_0_r_valid(ram_io_axi_arbiter_io_sharedInputs_0_r_valid),
    .io_sharedInputs_0_r_ready(core_dBus_decoder_io_sharedOutputs_1_r_ready),
    .io_sharedInputs_0_r_payload_data(ram_io_axi_arbiter_io_sharedInputs_0_r_payload_data),
    .io_sharedInputs_0_r_payload_id(ram_io_axi_arbiter_io_sharedInputs_0_r_payload_id),
    .io_sharedInputs_0_r_payload_resp(ram_io_axi_arbiter_io_sharedInputs_0_r_payload_resp),
    .io_sharedInputs_0_r_payload_last(ram_io_axi_arbiter_io_sharedInputs_0_r_payload_last),
    .io_output_arw_valid(ram_io_axi_arbiter_io_output_arw_valid),
    .io_output_arw_ready(ram_io_axi_arbiter_io_output_arw_halfPipe_regs_ready),
    .io_output_arw_payload_addr(ram_io_axi_arbiter_io_output_arw_payload_addr),
    .io_output_arw_payload_id(ram_io_axi_arbiter_io_output_arw_payload_id),
    .io_output_arw_payload_len(ram_io_axi_arbiter_io_output_arw_payload_len),
    .io_output_arw_payload_size(ram_io_axi_arbiter_io_output_arw_payload_size),
    .io_output_arw_payload_burst(ram_io_axi_arbiter_io_output_arw_payload_burst),
    .io_output_arw_payload_write(ram_io_axi_arbiter_io_output_arw_payload_write),
    .io_output_w_valid(ram_io_axi_arbiter_io_output_w_valid),
    .io_output_w_ready(_zz_CpuComplex_145_),
    .io_output_w_payload_data(ram_io_axi_arbiter_io_output_w_payload_data),
    .io_output_w_payload_strb(ram_io_axi_arbiter_io_output_w_payload_strb),
    .io_output_w_payload_last(ram_io_axi_arbiter_io_output_w_payload_last),
    .io_output_b_valid(ram_io_axi_b_valid),
    .io_output_b_ready(ram_io_axi_arbiter_io_output_b_ready),
    .io_output_b_payload_id(ram_io_axi_b_payload_id),
    .io_output_b_payload_resp(ram_io_axi_b_payload_resp),
    .io_output_r_valid(ram_io_axi_r_valid),
    .io_output_r_ready(ram_io_axi_arbiter_io_output_r_ready),
    .io_output_r_payload_data(ram_io_axi_r_payload_data),
    .io_output_r_payload_id(ram_io_axi_r_payload_id),
    .io_output_r_payload_resp(ram_io_axi_r_payload_resp),
    .io_output_r_payload_last(ram_io_axi_r_payload_last),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  Axi4SharedArbiter_3_ apbBridge_io_axi_arbiter ( 
    .io_sharedInputs_0_arw_valid(_zz_CpuComplex_60_),
    .io_sharedInputs_0_arw_ready(apbBridge_io_axi_arbiter_io_sharedInputs_0_arw_ready),
    .io_sharedInputs_0_arw_payload_addr(_zz_CpuComplex_146_),
    .io_sharedInputs_0_arw_payload_id(_zz_CpuComplex_107_),
    .io_sharedInputs_0_arw_payload_len(_zz_CpuComplex_108_),
    .io_sharedInputs_0_arw_payload_size(core_dBus_decoder_io_sharedOutputs_3_arw_payload_size),
    .io_sharedInputs_0_arw_payload_burst(_zz_CpuComplex_147_),
    .io_sharedInputs_0_arw_payload_write(core_dBus_decoder_io_sharedOutputs_3_arw_payload_write),
    .io_sharedInputs_0_w_valid(core_dBus_decoder_io_sharedOutputs_3_w_valid),
    .io_sharedInputs_0_w_ready(apbBridge_io_axi_arbiter_io_sharedInputs_0_w_ready),
    .io_sharedInputs_0_w_payload_data(core_dBus_decoder_io_sharedOutputs_3_w_payload_data),
    .io_sharedInputs_0_w_payload_strb(core_dBus_decoder_io_sharedOutputs_3_w_payload_strb),
    .io_sharedInputs_0_w_payload_last(core_dBus_decoder_io_sharedOutputs_3_w_payload_last),
    .io_sharedInputs_0_b_valid(apbBridge_io_axi_arbiter_io_sharedInputs_0_b_valid),
    .io_sharedInputs_0_b_ready(core_dBus_decoder_io_sharedOutputs_3_b_ready),
    .io_sharedInputs_0_b_payload_id(apbBridge_io_axi_arbiter_io_sharedInputs_0_b_payload_id),
    .io_sharedInputs_0_b_payload_resp(apbBridge_io_axi_arbiter_io_sharedInputs_0_b_payload_resp),
    .io_sharedInputs_0_r_valid(apbBridge_io_axi_arbiter_io_sharedInputs_0_r_valid),
    .io_sharedInputs_0_r_ready(core_dBus_decoder_io_sharedOutputs_3_r_ready),
    .io_sharedInputs_0_r_payload_data(apbBridge_io_axi_arbiter_io_sharedInputs_0_r_payload_data),
    .io_sharedInputs_0_r_payload_id(apbBridge_io_axi_arbiter_io_sharedInputs_0_r_payload_id),
    .io_sharedInputs_0_r_payload_resp(apbBridge_io_axi_arbiter_io_sharedInputs_0_r_payload_resp),
    .io_sharedInputs_0_r_payload_last(apbBridge_io_axi_arbiter_io_sharedInputs_0_r_payload_last),
    .io_output_arw_valid(apbBridge_io_axi_arbiter_io_output_arw_valid),
    .io_output_arw_ready(apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_ready),
    .io_output_arw_payload_addr(apbBridge_io_axi_arbiter_io_output_arw_payload_addr),
    .io_output_arw_payload_id(apbBridge_io_axi_arbiter_io_output_arw_payload_id),
    .io_output_arw_payload_len(apbBridge_io_axi_arbiter_io_output_arw_payload_len),
    .io_output_arw_payload_size(apbBridge_io_axi_arbiter_io_output_arw_payload_size),
    .io_output_arw_payload_burst(apbBridge_io_axi_arbiter_io_output_arw_payload_burst),
    .io_output_arw_payload_write(apbBridge_io_axi_arbiter_io_output_arw_payload_write),
    .io_output_w_valid(apbBridge_io_axi_arbiter_io_output_w_valid),
    .io_output_w_ready(apbBridge_io_axi_arbiter_io_output_w_halfPipe_regs_ready),
    .io_output_w_payload_data(apbBridge_io_axi_arbiter_io_output_w_payload_data),
    .io_output_w_payload_strb(apbBridge_io_axi_arbiter_io_output_w_payload_strb),
    .io_output_w_payload_last(apbBridge_io_axi_arbiter_io_output_w_payload_last),
    .io_output_b_valid(apbBridge_io_axi_b_valid),
    .io_output_b_ready(apbBridge_io_axi_arbiter_io_output_b_ready),
    .io_output_b_payload_id(apbBridge_io_axi_b_payload_id),
    .io_output_b_payload_resp(apbBridge_io_axi_b_payload_resp),
    .io_output_r_valid(apbBridge_io_axi_r_valid),
    .io_output_r_ready(apbBridge_io_axi_arbiter_io_output_r_ready),
    .io_output_r_payload_data(apbBridge_io_axi_r_payload_data),
    .io_output_r_payload_id(apbBridge_io_axi_r_payload_id),
    .io_output_r_payload_resp(apbBridge_io_axi_r_payload_resp),
    .io_output_r_payload_last(apbBridge_io_axi_r_payload_last),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  assign _zz_CpuComplex_110_ = (! (core_iBus_r_payload_resp == (2'b00)));
  assign _zz_CpuComplex_1_ = ((1'b1 && (! _zz_CpuComplex_2_)) || core_iBus_ar_ready);
  assign _zz_CpuComplex_2_ = _zz_CpuComplex_3_;
  assign core_iBus_ar_valid = _zz_CpuComplex_2_;
  assign core_iBus_ar_payload_addr = _zz_CpuComplex_4_;
  assign core_iBus_ar_payload_cache = _zz_CpuComplex_5_;
  assign core_iBus_ar_payload_prot = _zz_CpuComplex_6_;
  assign core_iBus_r_ready = 1'b1;
  always @ (*) begin
    _zz_CpuComplex_11_ = 1'b0;
    if(((_zz_CpuComplex_7_ && _zz_CpuComplex_8_) && _zz_CpuComplex_9_))begin
      _zz_CpuComplex_11_ = 1'b1;
    end
  end

  always @ (*) begin
    _zz_CpuComplex_12_ = 1'b0;
    if((core_dBus_b_valid && 1'b1))begin
      _zz_CpuComplex_12_ = 1'b1;
    end
  end

  always @ (*) begin
    if((_zz_CpuComplex_11_ && (! _zz_CpuComplex_12_)))begin
      _zz_CpuComplex_14_ = (3'b001);
    end else begin
      if(((! _zz_CpuComplex_11_) && _zz_CpuComplex_12_))begin
        _zz_CpuComplex_14_ = (3'b111);
      end else begin
        _zz_CpuComplex_14_ = (3'b000);
      end
    end
  end

  assign _zz_CpuComplex_15_ = (_zz_CpuComplex_13_ + _zz_CpuComplex_14_);
  assign _zz_CpuComplex_112_ = ((1'b1 && (! core_cpu_dBus_cmd_m2sPipe_valid)) || core_cpu_dBus_cmd_m2sPipe_ready);
  assign core_cpu_dBus_cmd_m2sPipe_valid = _zz_CpuComplex_16_;
  assign core_cpu_dBus_cmd_m2sPipe_payload_wr = _zz_CpuComplex_17_;
  assign core_cpu_dBus_cmd_m2sPipe_payload_address = _zz_CpuComplex_18_;
  assign core_cpu_dBus_cmd_m2sPipe_payload_data = _zz_CpuComplex_19_;
  assign core_cpu_dBus_cmd_m2sPipe_payload_size = _zz_CpuComplex_20_;
  assign core_cpu_dBus_cmd_m2sPipe_ready = ((1'b1 && (! core_cpu_dBus_cmd_m2sPipe_m2sPipe_valid)) || core_cpu_dBus_cmd_m2sPipe_m2sPipe_ready);
  assign core_cpu_dBus_cmd_m2sPipe_m2sPipe_valid = _zz_CpuComplex_21_;
  assign core_cpu_dBus_cmd_m2sPipe_m2sPipe_payload_wr = _zz_CpuComplex_22_;
  assign core_cpu_dBus_cmd_m2sPipe_m2sPipe_payload_address = _zz_CpuComplex_23_;
  assign core_cpu_dBus_cmd_m2sPipe_m2sPipe_payload_data = _zz_CpuComplex_24_;
  assign core_cpu_dBus_cmd_m2sPipe_m2sPipe_payload_size = _zz_CpuComplex_25_;
  assign core_cpu_dBus_cmd_m2sPipe_m2sPipe_s2mPipe_valid = (core_cpu_dBus_cmd_m2sPipe_m2sPipe_valid || _zz_CpuComplex_26_);
  assign core_cpu_dBus_cmd_m2sPipe_m2sPipe_ready = (! _zz_CpuComplex_26_);
  assign core_cpu_dBus_cmd_m2sPipe_m2sPipe_s2mPipe_payload_wr = (_zz_CpuComplex_26_ ? _zz_CpuComplex_27_ : core_cpu_dBus_cmd_m2sPipe_m2sPipe_payload_wr);
  assign core_cpu_dBus_cmd_m2sPipe_m2sPipe_s2mPipe_payload_address = (_zz_CpuComplex_26_ ? _zz_CpuComplex_28_ : core_cpu_dBus_cmd_m2sPipe_m2sPipe_payload_address);
  assign core_cpu_dBus_cmd_m2sPipe_m2sPipe_s2mPipe_payload_data = (_zz_CpuComplex_26_ ? _zz_CpuComplex_29_ : core_cpu_dBus_cmd_m2sPipe_m2sPipe_payload_data);
  assign core_cpu_dBus_cmd_m2sPipe_m2sPipe_s2mPipe_payload_size = (_zz_CpuComplex_26_ ? _zz_CpuComplex_30_ : core_cpu_dBus_cmd_m2sPipe_m2sPipe_payload_size);
  assign _zz_CpuComplex_31_ = (! (((_zz_CpuComplex_13_ != (3'b000)) && (! core_cpu_dBus_cmd_m2sPipe_m2sPipe_s2mPipe_payload_wr)) || (_zz_CpuComplex_13_ == (3'b111))));
  assign core_cpu_dBus_cmd_m2sPipe_m2sPipe_s2mPipe_ready = (streamFork_5__io_input_ready && _zz_CpuComplex_31_);
  assign _zz_CpuComplex_114_ = (core_cpu_dBus_cmd_m2sPipe_m2sPipe_s2mPipe_valid && _zz_CpuComplex_31_);
  assign _zz_CpuComplex_7_ = streamFork_5__io_outputs_0_valid;
  assign _zz_CpuComplex_9_ = streamFork_5__io_outputs_0_payload_wr;
  always @ (*) begin
    _zz_CpuComplex_32_ = streamFork_5__io_outputs_1_valid;
    _zz_CpuComplex_115_ = _zz_CpuComplex_10_;
    if((! streamFork_5__io_outputs_1_payload_wr))begin
      _zz_CpuComplex_32_ = 1'b0;
      _zz_CpuComplex_115_ = 1'b1;
    end
  end

  always @ (*) begin
    case(streamFork_5__io_outputs_1_payload_size)
      2'b00 : begin
        _zz_CpuComplex_33_ = (4'b0001);
      end
      2'b01 : begin
        _zz_CpuComplex_33_ = (4'b0011);
      end
      default : begin
        _zz_CpuComplex_33_ = (4'b1111);
      end
    endcase
  end

  assign _zz_CpuComplex_113_ = (! (core_dBus_r_payload_resp == (2'b00)));
  assign _zz_CpuComplex_8_ = ((1'b1 && (! _zz_CpuComplex_34_)) || core_dBus_arw_ready);
  assign _zz_CpuComplex_34_ = _zz_CpuComplex_35_;
  assign core_dBus_arw_valid = _zz_CpuComplex_34_;
  assign core_dBus_arw_payload_addr = _zz_CpuComplex_36_;
  assign core_dBus_arw_payload_size = _zz_CpuComplex_37_;
  assign core_dBus_arw_payload_cache = _zz_CpuComplex_38_;
  assign core_dBus_arw_payload_prot = _zz_CpuComplex_39_;
  assign core_dBus_arw_payload_write = _zz_CpuComplex_40_;
  assign core_dBus_w_valid = _zz_CpuComplex_32_;
  assign _zz_CpuComplex_10_ = core_dBus_w_ready;
  assign core_dBus_w_payload_data = streamFork_5__io_outputs_1_payload_data;
  assign core_dBus_w_payload_strb = _zz_CpuComplex_157_[3:0];
  assign core_dBus_w_payload_last = 1'b1;
  assign core_dBus_r_ready = 1'b1;
  assign core_dBus_b_ready = 1'b1;
  assign _zz_CpuComplex_111_ = systemDebugger_1__io_mem_cmd_payload_address[7:0];
  assign io_jtag_tdo = jtagBridge_1__io_jtag_tdo;
  assign io_apb_PADDR = apbBridge_io_apb_PADDR;
  assign io_apb_PSEL = apbBridge_io_apb_PSEL;
  assign io_apb_PENABLE = apbBridge_io_apb_PENABLE;
  assign io_apb_PWRITE = apbBridge_io_apb_PWRITE;
  assign io_apb_PWDATA = apbBridge_io_apb_PWDATA;
  assign _zz_CpuComplex_42_ = _zz_CpuComplex_44_;
  assign _zz_CpuComplex_116_ = (_zz_CpuComplex_43_ && _zz_CpuComplex_44_);
  assign _zz_CpuComplex_43_ = io_axiMem1_arbiter_io_readInputs_0_ar_ready;
  assign _zz_CpuComplex_45_ = _zz_CpuComplex_47_;
  assign _zz_CpuComplex_117_ = (_zz_CpuComplex_46_ && _zz_CpuComplex_47_);
  assign _zz_CpuComplex_46_ = ram_io_axi_arbiter_io_readInputs_0_ar_ready;
  assign _zz_CpuComplex_48_ = _zz_CpuComplex_50_;
  assign _zz_CpuComplex_118_ = (_zz_CpuComplex_49_ && _zz_CpuComplex_50_);
  assign _zz_CpuComplex_49_ = io_axiMem2_arbiter_io_readInputs_0_ar_ready;
  assign core_iBus_ar_ready = core_iBus_decoder_io_input_ar_ready;
  assign core_iBus_r_valid = core_iBus_decoder_io_input_r_valid;
  assign core_iBus_r_payload_data = core_iBus_decoder_io_input_r_payload_data;
  assign core_iBus_r_payload_last = core_iBus_decoder_io_input_r_payload_last;
  assign core_iBus_r_payload_resp = core_iBus_decoder_io_input_r_payload_resp;
  assign _zz_CpuComplex_51_ = _zz_CpuComplex_53_;
  assign _zz_CpuComplex_120_ = (_zz_CpuComplex_52_ && _zz_CpuComplex_53_);
  assign _zz_CpuComplex_52_ = io_axiMem1_arbiter_io_sharedInputs_0_arw_ready;
  assign _zz_CpuComplex_54_ = _zz_CpuComplex_56_;
  assign _zz_CpuComplex_121_ = (_zz_CpuComplex_55_ && _zz_CpuComplex_56_);
  assign _zz_CpuComplex_55_ = ram_io_axi_arbiter_io_sharedInputs_0_arw_ready;
  assign _zz_CpuComplex_57_ = _zz_CpuComplex_59_;
  assign _zz_CpuComplex_122_ = (_zz_CpuComplex_58_ && _zz_CpuComplex_59_);
  assign _zz_CpuComplex_58_ = io_axiMem2_arbiter_io_sharedInputs_0_arw_ready;
  assign _zz_CpuComplex_60_ = _zz_CpuComplex_62_;
  assign _zz_CpuComplex_123_ = (_zz_CpuComplex_61_ && _zz_CpuComplex_62_);
  assign _zz_CpuComplex_61_ = apbBridge_io_axi_arbiter_io_sharedInputs_0_arw_ready;
  assign core_dBus_arw_ready = core_dBus_decoder_io_input_arw_ready;
  assign core_dBus_w_ready = core_dBus_decoder_io_input_w_ready;
  assign core_dBus_b_valid = core_dBus_decoder_io_input_b_valid;
  assign core_dBus_b_payload_resp = core_dBus_decoder_io_input_b_payload_resp;
  assign _zz_CpuComplex_119_ = ((1'b1 && (! core_dBus_decoder_io_input_r_m2sPipe_valid)) || core_dBus_decoder_io_input_r_m2sPipe_ready);
  assign core_dBus_decoder_io_input_r_m2sPipe_valid = _zz_CpuComplex_63_;
  assign core_dBus_decoder_io_input_r_m2sPipe_payload_data = _zz_CpuComplex_64_;
  assign core_dBus_decoder_io_input_r_m2sPipe_payload_resp = _zz_CpuComplex_65_;
  assign core_dBus_decoder_io_input_r_m2sPipe_payload_last = _zz_CpuComplex_66_;
  assign core_dBus_r_valid = core_dBus_decoder_io_input_r_m2sPipe_valid;
  assign core_dBus_decoder_io_input_r_m2sPipe_ready = core_dBus_r_ready;
  assign core_dBus_r_payload_data = core_dBus_decoder_io_input_r_m2sPipe_payload_data;
  assign core_dBus_r_payload_resp = core_dBus_decoder_io_input_r_m2sPipe_payload_resp;
  assign core_dBus_r_payload_last = core_dBus_decoder_io_input_r_m2sPipe_payload_last;
  assign _zz_CpuComplex_67_[2 : 0] = (3'b000);
  assign _zz_CpuComplex_68_[3 : 0] = (4'b0000);
  assign _zz_CpuComplex_69_[7 : 0] = (8'b00000000);
  assign _zz_CpuComplex_124_ = (3'b010);
  assign _zz_CpuComplex_125_ = (2'b01);
  assign _zz_CpuComplex_126_ = (1'b0);
  assign _zz_CpuComplex_127_ = (4'b0000);
  assign _zz_CpuComplex_70_[2 : 0] = (3'b000);
  assign _zz_CpuComplex_71_[3 : 0] = (4'b0000);
  assign _zz_CpuComplex_72_[7 : 0] = (8'b00000000);
  assign _zz_CpuComplex_128_ = (2'b01);
  assign _zz_CpuComplex_129_ = (1'b0);
  assign _zz_CpuComplex_130_ = (4'b0000);
  assign io_axiMem1_arbiter_io_output_arw_halfPipe_valid = io_axiMem1_arbiter_io_output_arw_halfPipe_regs_valid;
  assign io_axiMem1_arbiter_io_output_arw_halfPipe_payload_addr = io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_addr;
  assign io_axiMem1_arbiter_io_output_arw_halfPipe_payload_id = io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_id;
  assign io_axiMem1_arbiter_io_output_arw_halfPipe_payload_region = io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_region;
  assign io_axiMem1_arbiter_io_output_arw_halfPipe_payload_len = io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_len;
  assign io_axiMem1_arbiter_io_output_arw_halfPipe_payload_size = io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_size;
  assign io_axiMem1_arbiter_io_output_arw_halfPipe_payload_burst = io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_burst;
  assign io_axiMem1_arbiter_io_output_arw_halfPipe_payload_lock = io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_lock;
  assign io_axiMem1_arbiter_io_output_arw_halfPipe_payload_cache = io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_cache;
  assign io_axiMem1_arbiter_io_output_arw_halfPipe_payload_qos = io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_qos;
  assign io_axiMem1_arbiter_io_output_arw_halfPipe_payload_prot = io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_prot;
  assign io_axiMem1_arbiter_io_output_arw_halfPipe_payload_write = io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_write;
  assign io_axiMem1_arw_valid = io_axiMem1_arbiter_io_output_arw_halfPipe_valid;
  assign io_axiMem1_arbiter_io_output_arw_halfPipe_ready = io_axiMem1_arw_ready;
  assign io_axiMem1_arw_payload_addr = io_axiMem1_arbiter_io_output_arw_halfPipe_payload_addr;
  assign io_axiMem1_arw_payload_id = io_axiMem1_arbiter_io_output_arw_halfPipe_payload_id;
  assign io_axiMem1_arw_payload_region = io_axiMem1_arbiter_io_output_arw_halfPipe_payload_region;
  assign io_axiMem1_arw_payload_len = io_axiMem1_arbiter_io_output_arw_halfPipe_payload_len;
  assign io_axiMem1_arw_payload_size = io_axiMem1_arbiter_io_output_arw_halfPipe_payload_size;
  assign io_axiMem1_arw_payload_burst = io_axiMem1_arbiter_io_output_arw_halfPipe_payload_burst;
  assign io_axiMem1_arw_payload_lock = io_axiMem1_arbiter_io_output_arw_halfPipe_payload_lock;
  assign io_axiMem1_arw_payload_cache = io_axiMem1_arbiter_io_output_arw_halfPipe_payload_cache;
  assign io_axiMem1_arw_payload_qos = io_axiMem1_arbiter_io_output_arw_halfPipe_payload_qos;
  assign io_axiMem1_arw_payload_prot = io_axiMem1_arbiter_io_output_arw_halfPipe_payload_prot;
  assign io_axiMem1_arw_payload_write = io_axiMem1_arbiter_io_output_arw_halfPipe_payload_write;
  assign io_axiMem1_arbiter_io_output_w_s2mPipe_valid = (io_axiMem1_arbiter_io_output_w_valid || _zz_CpuComplex_73_);
  assign _zz_CpuComplex_131_ = (! _zz_CpuComplex_73_);
  assign io_axiMem1_arbiter_io_output_w_s2mPipe_payload_data = (_zz_CpuComplex_73_ ? _zz_CpuComplex_74_ : io_axiMem1_arbiter_io_output_w_payload_data);
  assign io_axiMem1_arbiter_io_output_w_s2mPipe_payload_strb = (_zz_CpuComplex_73_ ? _zz_CpuComplex_75_ : io_axiMem1_arbiter_io_output_w_payload_strb);
  assign io_axiMem1_arbiter_io_output_w_s2mPipe_payload_last = (_zz_CpuComplex_73_ ? _zz_CpuComplex_76_ : io_axiMem1_arbiter_io_output_w_payload_last);
  assign io_axiMem1_arbiter_io_output_w_s2mPipe_ready = ((1'b1 && (! io_axiMem1_arbiter_io_output_w_s2mPipe_m2sPipe_valid)) || io_axiMem1_arbiter_io_output_w_s2mPipe_m2sPipe_ready);
  assign io_axiMem1_arbiter_io_output_w_s2mPipe_m2sPipe_valid = _zz_CpuComplex_77_;
  assign io_axiMem1_arbiter_io_output_w_s2mPipe_m2sPipe_payload_data = _zz_CpuComplex_78_;
  assign io_axiMem1_arbiter_io_output_w_s2mPipe_m2sPipe_payload_strb = _zz_CpuComplex_79_;
  assign io_axiMem1_arbiter_io_output_w_s2mPipe_m2sPipe_payload_last = _zz_CpuComplex_80_;
  assign io_axiMem1_w_valid = io_axiMem1_arbiter_io_output_w_s2mPipe_m2sPipe_valid;
  assign io_axiMem1_arbiter_io_output_w_s2mPipe_m2sPipe_ready = io_axiMem1_w_ready;
  assign io_axiMem1_w_payload_data = io_axiMem1_arbiter_io_output_w_s2mPipe_m2sPipe_payload_data;
  assign io_axiMem1_w_payload_strb = io_axiMem1_arbiter_io_output_w_s2mPipe_m2sPipe_payload_strb;
  assign io_axiMem1_w_payload_last = io_axiMem1_arbiter_io_output_w_s2mPipe_m2sPipe_payload_last;
  assign io_axiMem1_b_ready = io_axiMem1_arbiter_io_output_b_ready;
  assign io_axiMem1_r_ready = io_axiMem1_arbiter_io_output_r_ready;
  assign _zz_CpuComplex_81_[2 : 0] = (3'b000);
  assign _zz_CpuComplex_82_[3 : 0] = (4'b0000);
  assign _zz_CpuComplex_83_[7 : 0] = (8'b00000000);
  assign _zz_CpuComplex_132_ = (3'b010);
  assign _zz_CpuComplex_133_ = (2'b01);
  assign _zz_CpuComplex_134_ = (1'b0);
  assign _zz_CpuComplex_135_ = (4'b0000);
  assign _zz_CpuComplex_84_[2 : 0] = (3'b000);
  assign _zz_CpuComplex_85_[3 : 0] = (4'b0000);
  assign _zz_CpuComplex_86_[7 : 0] = (8'b00000000);
  assign _zz_CpuComplex_136_ = (2'b01);
  assign _zz_CpuComplex_137_ = (1'b0);
  assign _zz_CpuComplex_138_ = (4'b0000);
  assign io_axiMem2_arbiter_io_output_arw_halfPipe_valid = io_axiMem2_arbiter_io_output_arw_halfPipe_regs_valid;
  assign io_axiMem2_arbiter_io_output_arw_halfPipe_payload_addr = io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_addr;
  assign io_axiMem2_arbiter_io_output_arw_halfPipe_payload_id = io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_id;
  assign io_axiMem2_arbiter_io_output_arw_halfPipe_payload_region = io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_region;
  assign io_axiMem2_arbiter_io_output_arw_halfPipe_payload_len = io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_len;
  assign io_axiMem2_arbiter_io_output_arw_halfPipe_payload_size = io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_size;
  assign io_axiMem2_arbiter_io_output_arw_halfPipe_payload_burst = io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_burst;
  assign io_axiMem2_arbiter_io_output_arw_halfPipe_payload_lock = io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_lock;
  assign io_axiMem2_arbiter_io_output_arw_halfPipe_payload_cache = io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_cache;
  assign io_axiMem2_arbiter_io_output_arw_halfPipe_payload_qos = io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_qos;
  assign io_axiMem2_arbiter_io_output_arw_halfPipe_payload_prot = io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_prot;
  assign io_axiMem2_arbiter_io_output_arw_halfPipe_payload_write = io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_write;
  assign io_axiMem2_arw_valid = io_axiMem2_arbiter_io_output_arw_halfPipe_valid;
  assign io_axiMem2_arbiter_io_output_arw_halfPipe_ready = io_axiMem2_arw_ready;
  assign io_axiMem2_arw_payload_addr = io_axiMem2_arbiter_io_output_arw_halfPipe_payload_addr;
  assign io_axiMem2_arw_payload_id = io_axiMem2_arbiter_io_output_arw_halfPipe_payload_id;
  assign io_axiMem2_arw_payload_region = io_axiMem2_arbiter_io_output_arw_halfPipe_payload_region;
  assign io_axiMem2_arw_payload_len = io_axiMem2_arbiter_io_output_arw_halfPipe_payload_len;
  assign io_axiMem2_arw_payload_size = io_axiMem2_arbiter_io_output_arw_halfPipe_payload_size;
  assign io_axiMem2_arw_payload_burst = io_axiMem2_arbiter_io_output_arw_halfPipe_payload_burst;
  assign io_axiMem2_arw_payload_lock = io_axiMem2_arbiter_io_output_arw_halfPipe_payload_lock;
  assign io_axiMem2_arw_payload_cache = io_axiMem2_arbiter_io_output_arw_halfPipe_payload_cache;
  assign io_axiMem2_arw_payload_qos = io_axiMem2_arbiter_io_output_arw_halfPipe_payload_qos;
  assign io_axiMem2_arw_payload_prot = io_axiMem2_arbiter_io_output_arw_halfPipe_payload_prot;
  assign io_axiMem2_arw_payload_write = io_axiMem2_arbiter_io_output_arw_halfPipe_payload_write;
  assign io_axiMem2_arbiter_io_output_w_s2mPipe_valid = (io_axiMem2_arbiter_io_output_w_valid || _zz_CpuComplex_87_);
  assign _zz_CpuComplex_139_ = (! _zz_CpuComplex_87_);
  assign io_axiMem2_arbiter_io_output_w_s2mPipe_payload_data = (_zz_CpuComplex_87_ ? _zz_CpuComplex_88_ : io_axiMem2_arbiter_io_output_w_payload_data);
  assign io_axiMem2_arbiter_io_output_w_s2mPipe_payload_strb = (_zz_CpuComplex_87_ ? _zz_CpuComplex_89_ : io_axiMem2_arbiter_io_output_w_payload_strb);
  assign io_axiMem2_arbiter_io_output_w_s2mPipe_payload_last = (_zz_CpuComplex_87_ ? _zz_CpuComplex_90_ : io_axiMem2_arbiter_io_output_w_payload_last);
  assign io_axiMem2_arbiter_io_output_w_s2mPipe_ready = ((1'b1 && (! io_axiMem2_arbiter_io_output_w_s2mPipe_m2sPipe_valid)) || io_axiMem2_arbiter_io_output_w_s2mPipe_m2sPipe_ready);
  assign io_axiMem2_arbiter_io_output_w_s2mPipe_m2sPipe_valid = _zz_CpuComplex_91_;
  assign io_axiMem2_arbiter_io_output_w_s2mPipe_m2sPipe_payload_data = _zz_CpuComplex_92_;
  assign io_axiMem2_arbiter_io_output_w_s2mPipe_m2sPipe_payload_strb = _zz_CpuComplex_93_;
  assign io_axiMem2_arbiter_io_output_w_s2mPipe_m2sPipe_payload_last = _zz_CpuComplex_94_;
  assign io_axiMem2_w_valid = io_axiMem2_arbiter_io_output_w_s2mPipe_m2sPipe_valid;
  assign io_axiMem2_arbiter_io_output_w_s2mPipe_m2sPipe_ready = io_axiMem2_w_ready;
  assign io_axiMem2_w_payload_data = io_axiMem2_arbiter_io_output_w_s2mPipe_m2sPipe_payload_data;
  assign io_axiMem2_w_payload_strb = io_axiMem2_arbiter_io_output_w_s2mPipe_m2sPipe_payload_strb;
  assign io_axiMem2_w_payload_last = io_axiMem2_arbiter_io_output_w_s2mPipe_m2sPipe_payload_last;
  assign io_axiMem2_b_ready = io_axiMem2_arbiter_io_output_b_ready;
  assign io_axiMem2_r_ready = io_axiMem2_arbiter_io_output_r_ready;
  assign _zz_CpuComplex_140_ = core_iBus_decoder_io_outputs_1_ar_payload_addr[12:0];
  assign _zz_CpuComplex_95_[2 : 0] = (3'b000);
  assign _zz_CpuComplex_96_[7 : 0] = (8'b00000000);
  assign _zz_CpuComplex_141_ = (3'b010);
  assign _zz_CpuComplex_142_ = (2'b01);
  assign _zz_CpuComplex_143_ = core_dBus_decoder_io_sharedOutputs_1_arw_payload_addr[12:0];
  assign _zz_CpuComplex_97_[2 : 0] = (3'b000);
  assign _zz_CpuComplex_98_[7 : 0] = (8'b00000000);
  assign _zz_CpuComplex_144_ = (2'b01);
  assign ram_io_axi_arbiter_io_output_arw_halfPipe_valid = ram_io_axi_arbiter_io_output_arw_halfPipe_regs_valid;
  assign ram_io_axi_arbiter_io_output_arw_halfPipe_payload_addr = ram_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_addr;
  assign ram_io_axi_arbiter_io_output_arw_halfPipe_payload_id = ram_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_id;
  assign ram_io_axi_arbiter_io_output_arw_halfPipe_payload_len = ram_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_len;
  assign ram_io_axi_arbiter_io_output_arw_halfPipe_payload_size = ram_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_size;
  assign ram_io_axi_arbiter_io_output_arw_halfPipe_payload_burst = ram_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_burst;
  assign ram_io_axi_arbiter_io_output_arw_halfPipe_payload_write = ram_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_write;
  assign ram_io_axi_arbiter_io_output_arw_halfPipe_ready = ram_io_axi_arw_ready;
  assign ram_io_axi_arbiter_io_output_w_s2mPipe_valid = (ram_io_axi_arbiter_io_output_w_valid || _zz_CpuComplex_99_);
  assign _zz_CpuComplex_145_ = (! _zz_CpuComplex_99_);
  assign ram_io_axi_arbiter_io_output_w_s2mPipe_payload_data = (_zz_CpuComplex_99_ ? _zz_CpuComplex_100_ : ram_io_axi_arbiter_io_output_w_payload_data);
  assign ram_io_axi_arbiter_io_output_w_s2mPipe_payload_strb = (_zz_CpuComplex_99_ ? _zz_CpuComplex_101_ : ram_io_axi_arbiter_io_output_w_payload_strb);
  assign ram_io_axi_arbiter_io_output_w_s2mPipe_payload_last = (_zz_CpuComplex_99_ ? _zz_CpuComplex_102_ : ram_io_axi_arbiter_io_output_w_payload_last);
  assign ram_io_axi_arbiter_io_output_w_s2mPipe_ready = ((1'b1 && (! ram_io_axi_arbiter_io_output_w_s2mPipe_m2sPipe_valid)) || ram_io_axi_arbiter_io_output_w_s2mPipe_m2sPipe_ready);
  assign ram_io_axi_arbiter_io_output_w_s2mPipe_m2sPipe_valid = _zz_CpuComplex_103_;
  assign ram_io_axi_arbiter_io_output_w_s2mPipe_m2sPipe_payload_data = _zz_CpuComplex_104_;
  assign ram_io_axi_arbiter_io_output_w_s2mPipe_m2sPipe_payload_strb = _zz_CpuComplex_105_;
  assign ram_io_axi_arbiter_io_output_w_s2mPipe_m2sPipe_payload_last = _zz_CpuComplex_106_;
  assign ram_io_axi_arbiter_io_output_w_s2mPipe_m2sPipe_ready = ram_io_axi_w_ready;
  assign _zz_CpuComplex_146_ = core_dBus_decoder_io_sharedOutputs_3_arw_payload_addr[19:0];
  assign _zz_CpuComplex_107_[3 : 0] = (4'b0000);
  assign _zz_CpuComplex_108_[7 : 0] = (8'b00000000);
  assign _zz_CpuComplex_147_ = (2'b01);
  assign apbBridge_io_axi_arbiter_io_output_arw_halfPipe_valid = apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_valid;
  assign apbBridge_io_axi_arbiter_io_output_arw_halfPipe_payload_addr = apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_addr;
  assign apbBridge_io_axi_arbiter_io_output_arw_halfPipe_payload_id = apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_id;
  assign apbBridge_io_axi_arbiter_io_output_arw_halfPipe_payload_len = apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_len;
  assign apbBridge_io_axi_arbiter_io_output_arw_halfPipe_payload_size = apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_size;
  assign apbBridge_io_axi_arbiter_io_output_arw_halfPipe_payload_burst = apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_burst;
  assign apbBridge_io_axi_arbiter_io_output_arw_halfPipe_payload_write = apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_write;
  assign apbBridge_io_axi_arbiter_io_output_arw_halfPipe_ready = apbBridge_io_axi_arw_ready;
  assign apbBridge_io_axi_arbiter_io_output_w_halfPipe_valid = apbBridge_io_axi_arbiter_io_output_w_halfPipe_regs_valid;
  assign apbBridge_io_axi_arbiter_io_output_w_halfPipe_payload_data = apbBridge_io_axi_arbiter_io_output_w_halfPipe_regs_payload_data;
  assign apbBridge_io_axi_arbiter_io_output_w_halfPipe_payload_strb = apbBridge_io_axi_arbiter_io_output_w_halfPipe_regs_payload_strb;
  assign apbBridge_io_axi_arbiter_io_output_w_halfPipe_payload_last = apbBridge_io_axi_arbiter_io_output_w_halfPipe_regs_payload_last;
  assign apbBridge_io_axi_arbiter_io_output_w_halfPipe_ready = apbBridge_io_axi_w_ready;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      _zz_CpuComplex_3_ <= 1'b0;
      _zz_CpuComplex_13_ <= (3'b000);
      _zz_CpuComplex_16_ <= 1'b0;
      _zz_CpuComplex_21_ <= 1'b0;
      _zz_CpuComplex_26_ <= 1'b0;
      _zz_CpuComplex_35_ <= 1'b0;
      _zz_CpuComplex_41_ <= 1'b0;
      _zz_CpuComplex_44_ <= 1'b0;
      _zz_CpuComplex_47_ <= 1'b0;
      _zz_CpuComplex_50_ <= 1'b0;
      _zz_CpuComplex_53_ <= 1'b0;
      _zz_CpuComplex_56_ <= 1'b0;
      _zz_CpuComplex_59_ <= 1'b0;
      _zz_CpuComplex_62_ <= 1'b0;
      _zz_CpuComplex_63_ <= 1'b0;
      io_axiMem1_arbiter_io_output_arw_halfPipe_regs_valid <= 1'b0;
      io_axiMem1_arbiter_io_output_arw_halfPipe_regs_ready <= 1'b1;
      _zz_CpuComplex_73_ <= 1'b0;
      _zz_CpuComplex_77_ <= 1'b0;
      io_axiMem2_arbiter_io_output_arw_halfPipe_regs_valid <= 1'b0;
      io_axiMem2_arbiter_io_output_arw_halfPipe_regs_ready <= 1'b1;
      _zz_CpuComplex_87_ <= 1'b0;
      _zz_CpuComplex_91_ <= 1'b0;
      ram_io_axi_arbiter_io_output_arw_halfPipe_regs_valid <= 1'b0;
      ram_io_axi_arbiter_io_output_arw_halfPipe_regs_ready <= 1'b1;
      _zz_CpuComplex_99_ <= 1'b0;
      _zz_CpuComplex_103_ <= 1'b0;
      apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_valid <= 1'b0;
      apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_ready <= 1'b1;
      apbBridge_io_axi_arbiter_io_output_w_halfPipe_regs_valid <= 1'b0;
      apbBridge_io_axi_arbiter_io_output_w_halfPipe_regs_ready <= 1'b1;
    end else begin
      if(_zz_CpuComplex_1_)begin
        _zz_CpuComplex_3_ <= core_cpu_iBus_cmd_valid;
      end
      _zz_CpuComplex_13_ <= _zz_CpuComplex_15_;
      if(_zz_CpuComplex_112_)begin
        _zz_CpuComplex_16_ <= core_cpu_dBus_cmd_valid;
      end
      if(core_cpu_dBus_cmd_m2sPipe_ready)begin
        _zz_CpuComplex_21_ <= core_cpu_dBus_cmd_m2sPipe_valid;
      end
      if(core_cpu_dBus_cmd_m2sPipe_m2sPipe_s2mPipe_ready)begin
        _zz_CpuComplex_26_ <= 1'b0;
      end
      if(_zz_CpuComplex_148_)begin
        _zz_CpuComplex_26_ <= core_cpu_dBus_cmd_m2sPipe_m2sPipe_valid;
      end
      if(_zz_CpuComplex_8_)begin
        _zz_CpuComplex_35_ <= _zz_CpuComplex_7_;
      end
      _zz_CpuComplex_41_ <= (systemDebugger_1__io_mem_cmd_valid && core_cpu_debug_bus_cmd_ready);
      if(core_iBus_decoder_io_outputs_0_ar_valid)begin
        _zz_CpuComplex_44_ <= 1'b1;
      end
      if((_zz_CpuComplex_42_ && _zz_CpuComplex_43_))begin
        _zz_CpuComplex_44_ <= 1'b0;
      end
      if(core_iBus_decoder_io_outputs_1_ar_valid)begin
        _zz_CpuComplex_47_ <= 1'b1;
      end
      if((_zz_CpuComplex_45_ && _zz_CpuComplex_46_))begin
        _zz_CpuComplex_47_ <= 1'b0;
      end
      if(core_iBus_decoder_io_outputs_2_ar_valid)begin
        _zz_CpuComplex_50_ <= 1'b1;
      end
      if((_zz_CpuComplex_48_ && _zz_CpuComplex_49_))begin
        _zz_CpuComplex_50_ <= 1'b0;
      end
      if(core_dBus_decoder_io_sharedOutputs_0_arw_valid)begin
        _zz_CpuComplex_53_ <= 1'b1;
      end
      if((_zz_CpuComplex_51_ && _zz_CpuComplex_52_))begin
        _zz_CpuComplex_53_ <= 1'b0;
      end
      if(core_dBus_decoder_io_sharedOutputs_1_arw_valid)begin
        _zz_CpuComplex_56_ <= 1'b1;
      end
      if((_zz_CpuComplex_54_ && _zz_CpuComplex_55_))begin
        _zz_CpuComplex_56_ <= 1'b0;
      end
      if(core_dBus_decoder_io_sharedOutputs_2_arw_valid)begin
        _zz_CpuComplex_59_ <= 1'b1;
      end
      if((_zz_CpuComplex_57_ && _zz_CpuComplex_58_))begin
        _zz_CpuComplex_59_ <= 1'b0;
      end
      if(core_dBus_decoder_io_sharedOutputs_3_arw_valid)begin
        _zz_CpuComplex_62_ <= 1'b1;
      end
      if((_zz_CpuComplex_60_ && _zz_CpuComplex_61_))begin
        _zz_CpuComplex_62_ <= 1'b0;
      end
      if(_zz_CpuComplex_119_)begin
        _zz_CpuComplex_63_ <= core_dBus_decoder_io_input_r_valid;
      end
      if(_zz_CpuComplex_149_)begin
        io_axiMem1_arbiter_io_output_arw_halfPipe_regs_valid <= io_axiMem1_arbiter_io_output_arw_valid;
        io_axiMem1_arbiter_io_output_arw_halfPipe_regs_ready <= (! io_axiMem1_arbiter_io_output_arw_valid);
      end else begin
        io_axiMem1_arbiter_io_output_arw_halfPipe_regs_valid <= (! io_axiMem1_arbiter_io_output_arw_halfPipe_ready);
        io_axiMem1_arbiter_io_output_arw_halfPipe_regs_ready <= io_axiMem1_arbiter_io_output_arw_halfPipe_ready;
      end
      if(io_axiMem1_arbiter_io_output_w_s2mPipe_ready)begin
        _zz_CpuComplex_73_ <= 1'b0;
      end
      if(_zz_CpuComplex_150_)begin
        _zz_CpuComplex_73_ <= io_axiMem1_arbiter_io_output_w_valid;
      end
      if(io_axiMem1_arbiter_io_output_w_s2mPipe_ready)begin
        _zz_CpuComplex_77_ <= io_axiMem1_arbiter_io_output_w_s2mPipe_valid;
      end
      if(_zz_CpuComplex_151_)begin
        io_axiMem2_arbiter_io_output_arw_halfPipe_regs_valid <= io_axiMem2_arbiter_io_output_arw_valid;
        io_axiMem2_arbiter_io_output_arw_halfPipe_regs_ready <= (! io_axiMem2_arbiter_io_output_arw_valid);
      end else begin
        io_axiMem2_arbiter_io_output_arw_halfPipe_regs_valid <= (! io_axiMem2_arbiter_io_output_arw_halfPipe_ready);
        io_axiMem2_arbiter_io_output_arw_halfPipe_regs_ready <= io_axiMem2_arbiter_io_output_arw_halfPipe_ready;
      end
      if(io_axiMem2_arbiter_io_output_w_s2mPipe_ready)begin
        _zz_CpuComplex_87_ <= 1'b0;
      end
      if(_zz_CpuComplex_152_)begin
        _zz_CpuComplex_87_ <= io_axiMem2_arbiter_io_output_w_valid;
      end
      if(io_axiMem2_arbiter_io_output_w_s2mPipe_ready)begin
        _zz_CpuComplex_91_ <= io_axiMem2_arbiter_io_output_w_s2mPipe_valid;
      end
      if(_zz_CpuComplex_153_)begin
        ram_io_axi_arbiter_io_output_arw_halfPipe_regs_valid <= ram_io_axi_arbiter_io_output_arw_valid;
        ram_io_axi_arbiter_io_output_arw_halfPipe_regs_ready <= (! ram_io_axi_arbiter_io_output_arw_valid);
      end else begin
        ram_io_axi_arbiter_io_output_arw_halfPipe_regs_valid <= (! ram_io_axi_arbiter_io_output_arw_halfPipe_ready);
        ram_io_axi_arbiter_io_output_arw_halfPipe_regs_ready <= ram_io_axi_arbiter_io_output_arw_halfPipe_ready;
      end
      if(ram_io_axi_arbiter_io_output_w_s2mPipe_ready)begin
        _zz_CpuComplex_99_ <= 1'b0;
      end
      if(_zz_CpuComplex_154_)begin
        _zz_CpuComplex_99_ <= ram_io_axi_arbiter_io_output_w_valid;
      end
      if(ram_io_axi_arbiter_io_output_w_s2mPipe_ready)begin
        _zz_CpuComplex_103_ <= ram_io_axi_arbiter_io_output_w_s2mPipe_valid;
      end
      if(_zz_CpuComplex_155_)begin
        apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_valid <= apbBridge_io_axi_arbiter_io_output_arw_valid;
        apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_ready <= (! apbBridge_io_axi_arbiter_io_output_arw_valid);
      end else begin
        apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_valid <= (! apbBridge_io_axi_arbiter_io_output_arw_halfPipe_ready);
        apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_ready <= apbBridge_io_axi_arbiter_io_output_arw_halfPipe_ready;
      end
      if(_zz_CpuComplex_156_)begin
        apbBridge_io_axi_arbiter_io_output_w_halfPipe_regs_valid <= apbBridge_io_axi_arbiter_io_output_w_valid;
        apbBridge_io_axi_arbiter_io_output_w_halfPipe_regs_ready <= (! apbBridge_io_axi_arbiter_io_output_w_valid);
      end else begin
        apbBridge_io_axi_arbiter_io_output_w_halfPipe_regs_valid <= (! apbBridge_io_axi_arbiter_io_output_w_halfPipe_ready);
        apbBridge_io_axi_arbiter_io_output_w_halfPipe_regs_ready <= apbBridge_io_axi_arbiter_io_output_w_halfPipe_ready;
      end
    end
  end

  always @ (posedge toplevel_main_clk) begin
    if(_zz_CpuComplex_1_)begin
      _zz_CpuComplex_4_ <= {core_cpu_iBus_cmd_payload_pc[31 : 2],(2'b00)};
      _zz_CpuComplex_5_ <= (4'b1111);
      _zz_CpuComplex_6_ <= (3'b110);
    end
    if(_zz_CpuComplex_112_)begin
      _zz_CpuComplex_17_ <= core_cpu_dBus_cmd_payload_wr;
      _zz_CpuComplex_18_ <= core_cpu_dBus_cmd_payload_address;
      _zz_CpuComplex_19_ <= core_cpu_dBus_cmd_payload_data;
      _zz_CpuComplex_20_ <= core_cpu_dBus_cmd_payload_size;
    end
    if(core_cpu_dBus_cmd_m2sPipe_ready)begin
      _zz_CpuComplex_22_ <= core_cpu_dBus_cmd_m2sPipe_payload_wr;
      _zz_CpuComplex_23_ <= core_cpu_dBus_cmd_m2sPipe_payload_address;
      _zz_CpuComplex_24_ <= core_cpu_dBus_cmd_m2sPipe_payload_data;
      _zz_CpuComplex_25_ <= core_cpu_dBus_cmd_m2sPipe_payload_size;
    end
    if(_zz_CpuComplex_148_)begin
      _zz_CpuComplex_27_ <= core_cpu_dBus_cmd_m2sPipe_m2sPipe_payload_wr;
      _zz_CpuComplex_28_ <= core_cpu_dBus_cmd_m2sPipe_m2sPipe_payload_address;
      _zz_CpuComplex_29_ <= core_cpu_dBus_cmd_m2sPipe_m2sPipe_payload_data;
      _zz_CpuComplex_30_ <= core_cpu_dBus_cmd_m2sPipe_m2sPipe_payload_size;
    end
    if(_zz_CpuComplex_8_)begin
      _zz_CpuComplex_36_ <= streamFork_5__io_outputs_0_payload_address;
      _zz_CpuComplex_37_ <= {1'd0, streamFork_5__io_outputs_0_payload_size};
      _zz_CpuComplex_38_ <= (4'b1111);
      _zz_CpuComplex_39_ <= (3'b010);
      _zz_CpuComplex_40_ <= _zz_CpuComplex_9_;
    end
    if(_zz_CpuComplex_119_)begin
      _zz_CpuComplex_64_ <= core_dBus_decoder_io_input_r_payload_data;
      _zz_CpuComplex_65_ <= core_dBus_decoder_io_input_r_payload_resp;
      _zz_CpuComplex_66_ <= core_dBus_decoder_io_input_r_payload_last;
    end
    if(_zz_CpuComplex_149_)begin
      io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_addr <= io_axiMem1_arbiter_io_output_arw_payload_addr;
      io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_id <= io_axiMem1_arbiter_io_output_arw_payload_id;
      io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_region <= io_axiMem1_arbiter_io_output_arw_payload_region;
      io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_len <= io_axiMem1_arbiter_io_output_arw_payload_len;
      io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_size <= io_axiMem1_arbiter_io_output_arw_payload_size;
      io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_burst <= io_axiMem1_arbiter_io_output_arw_payload_burst;
      io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_lock <= io_axiMem1_arbiter_io_output_arw_payload_lock;
      io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_cache <= io_axiMem1_arbiter_io_output_arw_payload_cache;
      io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_qos <= io_axiMem1_arbiter_io_output_arw_payload_qos;
      io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_prot <= io_axiMem1_arbiter_io_output_arw_payload_prot;
      io_axiMem1_arbiter_io_output_arw_halfPipe_regs_payload_write <= io_axiMem1_arbiter_io_output_arw_payload_write;
    end
    if(_zz_CpuComplex_150_)begin
      _zz_CpuComplex_74_ <= io_axiMem1_arbiter_io_output_w_payload_data;
      _zz_CpuComplex_75_ <= io_axiMem1_arbiter_io_output_w_payload_strb;
      _zz_CpuComplex_76_ <= io_axiMem1_arbiter_io_output_w_payload_last;
    end
    if(io_axiMem1_arbiter_io_output_w_s2mPipe_ready)begin
      _zz_CpuComplex_78_ <= io_axiMem1_arbiter_io_output_w_s2mPipe_payload_data;
      _zz_CpuComplex_79_ <= io_axiMem1_arbiter_io_output_w_s2mPipe_payload_strb;
      _zz_CpuComplex_80_ <= io_axiMem1_arbiter_io_output_w_s2mPipe_payload_last;
    end
    if(_zz_CpuComplex_151_)begin
      io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_addr <= io_axiMem2_arbiter_io_output_arw_payload_addr;
      io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_id <= io_axiMem2_arbiter_io_output_arw_payload_id;
      io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_region <= io_axiMem2_arbiter_io_output_arw_payload_region;
      io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_len <= io_axiMem2_arbiter_io_output_arw_payload_len;
      io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_size <= io_axiMem2_arbiter_io_output_arw_payload_size;
      io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_burst <= io_axiMem2_arbiter_io_output_arw_payload_burst;
      io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_lock <= io_axiMem2_arbiter_io_output_arw_payload_lock;
      io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_cache <= io_axiMem2_arbiter_io_output_arw_payload_cache;
      io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_qos <= io_axiMem2_arbiter_io_output_arw_payload_qos;
      io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_prot <= io_axiMem2_arbiter_io_output_arw_payload_prot;
      io_axiMem2_arbiter_io_output_arw_halfPipe_regs_payload_write <= io_axiMem2_arbiter_io_output_arw_payload_write;
    end
    if(_zz_CpuComplex_152_)begin
      _zz_CpuComplex_88_ <= io_axiMem2_arbiter_io_output_w_payload_data;
      _zz_CpuComplex_89_ <= io_axiMem2_arbiter_io_output_w_payload_strb;
      _zz_CpuComplex_90_ <= io_axiMem2_arbiter_io_output_w_payload_last;
    end
    if(io_axiMem2_arbiter_io_output_w_s2mPipe_ready)begin
      _zz_CpuComplex_92_ <= io_axiMem2_arbiter_io_output_w_s2mPipe_payload_data;
      _zz_CpuComplex_93_ <= io_axiMem2_arbiter_io_output_w_s2mPipe_payload_strb;
      _zz_CpuComplex_94_ <= io_axiMem2_arbiter_io_output_w_s2mPipe_payload_last;
    end
    if(_zz_CpuComplex_153_)begin
      ram_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_addr <= ram_io_axi_arbiter_io_output_arw_payload_addr;
      ram_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_id <= ram_io_axi_arbiter_io_output_arw_payload_id;
      ram_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_len <= ram_io_axi_arbiter_io_output_arw_payload_len;
      ram_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_size <= ram_io_axi_arbiter_io_output_arw_payload_size;
      ram_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_burst <= ram_io_axi_arbiter_io_output_arw_payload_burst;
      ram_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_write <= ram_io_axi_arbiter_io_output_arw_payload_write;
    end
    if(_zz_CpuComplex_154_)begin
      _zz_CpuComplex_100_ <= ram_io_axi_arbiter_io_output_w_payload_data;
      _zz_CpuComplex_101_ <= ram_io_axi_arbiter_io_output_w_payload_strb;
      _zz_CpuComplex_102_ <= ram_io_axi_arbiter_io_output_w_payload_last;
    end
    if(ram_io_axi_arbiter_io_output_w_s2mPipe_ready)begin
      _zz_CpuComplex_104_ <= ram_io_axi_arbiter_io_output_w_s2mPipe_payload_data;
      _zz_CpuComplex_105_ <= ram_io_axi_arbiter_io_output_w_s2mPipe_payload_strb;
      _zz_CpuComplex_106_ <= ram_io_axi_arbiter_io_output_w_s2mPipe_payload_last;
    end
    if(_zz_CpuComplex_155_)begin
      apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_addr <= apbBridge_io_axi_arbiter_io_output_arw_payload_addr;
      apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_id <= apbBridge_io_axi_arbiter_io_output_arw_payload_id;
      apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_len <= apbBridge_io_axi_arbiter_io_output_arw_payload_len;
      apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_size <= apbBridge_io_axi_arbiter_io_output_arw_payload_size;
      apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_burst <= apbBridge_io_axi_arbiter_io_output_arw_payload_burst;
      apbBridge_io_axi_arbiter_io_output_arw_halfPipe_regs_payload_write <= apbBridge_io_axi_arbiter_io_output_arw_payload_write;
    end
    if(_zz_CpuComplex_156_)begin
      apbBridge_io_axi_arbiter_io_output_w_halfPipe_regs_payload_data <= apbBridge_io_axi_arbiter_io_output_w_payload_data;
      apbBridge_io_axi_arbiter_io_output_w_halfPipe_regs_payload_strb <= apbBridge_io_axi_arbiter_io_output_w_payload_strb;
      apbBridge_io_axi_arbiter_io_output_w_halfPipe_regs_payload_last <= apbBridge_io_axi_arbiter_io_output_w_payload_last;
    end
  end

endmodule

module Apb3JtagUart (
      input  [3:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output reg [31:0] io_apb_PRDATA,
      output  io_apb_PSLVERROR,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg  _zz_Apb3JtagUart_3_;
  wire  jtagUart_1__io_tx_ready;
  wire  jtagUart_1__io_rx_valid;
  wire [7:0] jtagUart_1__io_rx_payload;
  wire  busCtrl_askWrite;
  wire  busCtrl_askRead;
  wire  busCtrl_doWrite;
  wire  busCtrl_doRead;
  reg  _zz_Apb3JtagUart_1_;
  wire [7:0] _zz_Apb3JtagUart_2_;
  JtagUart jtagUart_1_ ( 
    .io_tx_valid(_zz_Apb3JtagUart_1_),
    .io_tx_ready(jtagUart_1__io_tx_ready),
    .io_tx_payload(_zz_Apb3JtagUart_2_),
    .io_rx_valid(jtagUart_1__io_rx_valid),
    .io_rx_ready(_zz_Apb3JtagUart_3_),
    .io_rx_payload(jtagUart_1__io_rx_payload),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = (32'b00000000000000000000000000000000);
    _zz_Apb3JtagUart_1_ = 1'b0;
    _zz_Apb3JtagUart_3_ = 1'b0;
    case(io_apb_PADDR)
      4'b1000 : begin
        if(busCtrl_doWrite)begin
          _zz_Apb3JtagUart_1_ = 1'b1;
        end
      end
      4'b1100 : begin
        if(busCtrl_doRead)begin
          _zz_Apb3JtagUart_3_ = 1'b1;
        end
        io_apb_PRDATA[31 : 31] = jtagUart_1__io_rx_valid;
        io_apb_PRDATA[7 : 0] = jtagUart_1__io_rx_payload;
      end
      default : begin
      end
    endcase
  end

  assign io_apb_PSLVERROR = 1'b0;
  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  assign _zz_Apb3JtagUart_2_ = io_apb_PWDATA[7 : 0];
endmodule

module MuraxApb3Timer (
      input  [7:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output reg [31:0] io_apb_PRDATA,
      output  io_apb_PSLVERROR,
      output  io_interrupt,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire  _zz_MuraxApb3Timer_7_;
  wire  _zz_MuraxApb3Timer_8_;
  wire  _zz_MuraxApb3Timer_9_;
  wire  _zz_MuraxApb3Timer_10_;
  reg [1:0] _zz_MuraxApb3Timer_11_;
  reg [1:0] _zz_MuraxApb3Timer_12_;
  wire  prescaler_1__io_overflow;
  wire  timerA_io_full;
  wire [15:0] timerA_io_value;
  wire  timerB_io_full;
  wire [15:0] timerB_io_value;
  wire [1:0] interruptCtrl_1__io_pendings;
  wire  busCtrl_askWrite;
  wire  busCtrl_askRead;
  wire  busCtrl_doWrite;
  wire  busCtrl_doRead;
  reg [15:0] _zz_MuraxApb3Timer_1_;
  reg  _zz_MuraxApb3Timer_2_;
  reg [1:0] timerABridge_ticksEnable;
  reg [0:0] timerABridge_clearsEnable;
  reg  timerABridge_busClearing;
  reg [15:0] timerA_io_limit__driver;
  reg  _zz_MuraxApb3Timer_3_;
  reg  _zz_MuraxApb3Timer_4_;
  reg [1:0] timerBBridge_ticksEnable;
  reg [0:0] timerBBridge_clearsEnable;
  reg  timerBBridge_busClearing;
  reg [15:0] timerB_io_limit__driver;
  reg  _zz_MuraxApb3Timer_5_;
  reg  _zz_MuraxApb3Timer_6_;
  reg [1:0] interruptCtrl_1__io_masks__driver;
  Prescaler prescaler_1_ ( 
    .io_clear(_zz_MuraxApb3Timer_2_),
    .io_limit(_zz_MuraxApb3Timer_1_),
    .io_overflow(prescaler_1__io_overflow),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  Timer timerA ( 
    .io_tick(_zz_MuraxApb3Timer_7_),
    .io_clear(_zz_MuraxApb3Timer_8_),
    .io_limit(timerA_io_limit__driver),
    .io_full(timerA_io_full),
    .io_value(timerA_io_value),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  Timer_1_ timerB ( 
    .io_tick(_zz_MuraxApb3Timer_9_),
    .io_clear(_zz_MuraxApb3Timer_10_),
    .io_limit(timerB_io_limit__driver),
    .io_full(timerB_io_full),
    .io_value(timerB_io_value),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  InterruptCtrl interruptCtrl_1_ ( 
    .io_inputs(_zz_MuraxApb3Timer_11_),
    .io_clears(_zz_MuraxApb3Timer_12_),
    .io_masks(interruptCtrl_1__io_masks__driver),
    .io_pendings(interruptCtrl_1__io_pendings),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = (32'b00000000000000000000000000000000);
    _zz_MuraxApb3Timer_2_ = 1'b0;
    _zz_MuraxApb3Timer_3_ = 1'b0;
    _zz_MuraxApb3Timer_4_ = 1'b0;
    _zz_MuraxApb3Timer_5_ = 1'b0;
    _zz_MuraxApb3Timer_6_ = 1'b0;
    _zz_MuraxApb3Timer_12_ = (2'b00);
    case(io_apb_PADDR)
      8'b00000000 : begin
        if(busCtrl_doWrite)begin
          _zz_MuraxApb3Timer_2_ = 1'b1;
        end
        io_apb_PRDATA[15 : 0] = _zz_MuraxApb3Timer_1_;
      end
      8'b01000000 : begin
        io_apb_PRDATA[1 : 0] = timerABridge_ticksEnable;
        io_apb_PRDATA[16 : 16] = timerABridge_clearsEnable;
      end
      8'b01000100 : begin
        if(busCtrl_doWrite)begin
          _zz_MuraxApb3Timer_3_ = 1'b1;
        end
        io_apb_PRDATA[15 : 0] = timerA_io_limit__driver;
      end
      8'b01001000 : begin
        if(busCtrl_doWrite)begin
          _zz_MuraxApb3Timer_4_ = 1'b1;
        end
        io_apb_PRDATA[15 : 0] = timerA_io_value;
      end
      8'b01010000 : begin
        io_apb_PRDATA[1 : 0] = timerBBridge_ticksEnable;
        io_apb_PRDATA[16 : 16] = timerBBridge_clearsEnable;
      end
      8'b01010100 : begin
        if(busCtrl_doWrite)begin
          _zz_MuraxApb3Timer_5_ = 1'b1;
        end
        io_apb_PRDATA[15 : 0] = timerB_io_limit__driver;
      end
      8'b01011000 : begin
        if(busCtrl_doWrite)begin
          _zz_MuraxApb3Timer_6_ = 1'b1;
        end
        io_apb_PRDATA[15 : 0] = timerB_io_value;
      end
      8'b00010000 : begin
        if(busCtrl_doWrite)begin
          _zz_MuraxApb3Timer_12_ = io_apb_PWDATA[1 : 0];
        end
        io_apb_PRDATA[1 : 0] = interruptCtrl_1__io_pendings;
      end
      8'b00010100 : begin
        io_apb_PRDATA[1 : 0] = interruptCtrl_1__io_masks__driver;
      end
      default : begin
      end
    endcase
  end

  assign io_apb_PSLVERROR = 1'b0;
  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  always @ (*) begin
    timerABridge_busClearing = 1'b0;
    if(_zz_MuraxApb3Timer_3_)begin
      timerABridge_busClearing = 1'b1;
    end
    if(_zz_MuraxApb3Timer_4_)begin
      timerABridge_busClearing = 1'b1;
    end
  end

  assign _zz_MuraxApb3Timer_8_ = (((timerABridge_clearsEnable & timerA_io_full) != (1'b0)) || timerABridge_busClearing);
  assign _zz_MuraxApb3Timer_7_ = ((timerABridge_ticksEnable & {prescaler_1__io_overflow,1'b1}) != (2'b00));
  always @ (*) begin
    timerBBridge_busClearing = 1'b0;
    if(_zz_MuraxApb3Timer_5_)begin
      timerBBridge_busClearing = 1'b1;
    end
    if(_zz_MuraxApb3Timer_6_)begin
      timerBBridge_busClearing = 1'b1;
    end
  end

  assign _zz_MuraxApb3Timer_10_ = (((timerBBridge_clearsEnable & timerB_io_full) != (1'b0)) || timerBBridge_busClearing);
  assign _zz_MuraxApb3Timer_9_ = ((timerBBridge_ticksEnable & {prescaler_1__io_overflow,1'b1}) != (2'b00));
  always @ (*) begin
    _zz_MuraxApb3Timer_11_[0] = timerA_io_full;
    _zz_MuraxApb3Timer_11_[1] = timerB_io_full;
  end

  assign io_interrupt = (interruptCtrl_1__io_pendings != (2'b00));
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      timerABridge_ticksEnable <= (2'b00);
      timerABridge_clearsEnable <= (1'b0);
      timerBBridge_ticksEnable <= (2'b00);
      timerBBridge_clearsEnable <= (1'b0);
      interruptCtrl_1__io_masks__driver <= (2'b00);
    end else begin
      case(io_apb_PADDR)
        8'b00000000 : begin
        end
        8'b01000000 : begin
          if(busCtrl_doWrite)begin
            timerABridge_ticksEnable <= io_apb_PWDATA[1 : 0];
            timerABridge_clearsEnable <= io_apb_PWDATA[16 : 16];
          end
        end
        8'b01000100 : begin
        end
        8'b01001000 : begin
        end
        8'b01010000 : begin
          if(busCtrl_doWrite)begin
            timerBBridge_ticksEnable <= io_apb_PWDATA[1 : 0];
            timerBBridge_clearsEnable <= io_apb_PWDATA[16 : 16];
          end
        end
        8'b01010100 : begin
        end
        8'b01011000 : begin
        end
        8'b00010000 : begin
        end
        8'b00010100 : begin
          if(busCtrl_doWrite)begin
            interruptCtrl_1__io_masks__driver <= io_apb_PWDATA[1 : 0];
          end
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge toplevel_main_clk) begin
    case(io_apb_PADDR)
      8'b00000000 : begin
        if(busCtrl_doWrite)begin
          _zz_MuraxApb3Timer_1_ <= io_apb_PWDATA[15 : 0];
        end
      end
      8'b01000000 : begin
      end
      8'b01000100 : begin
        if(busCtrl_doWrite)begin
          timerA_io_limit__driver <= io_apb_PWDATA[15 : 0];
        end
      end
      8'b01001000 : begin
      end
      8'b01010000 : begin
      end
      8'b01010100 : begin
        if(busCtrl_doWrite)begin
          timerB_io_limit__driver <= io_apb_PWDATA[15 : 0];
        end
      end
      8'b01011000 : begin
      end
      8'b00010000 : begin
      end
      8'b00010100 : begin
      end
      default : begin
      end
    endcase
  end

endmodule

module Apb3Decoder (
      input  [19:0] io_input_PADDR,
      input  [0:0] io_input_PSEL,
      input   io_input_PENABLE,
      output reg  io_input_PREADY,
      input   io_input_PWRITE,
      input  [31:0] io_input_PWDATA,
      output [31:0] io_input_PRDATA,
      output reg  io_input_PSLVERROR,
      output [19:0] io_output_PADDR,
      output reg [9:0] io_output_PSEL,
      output  io_output_PENABLE,
      input   io_output_PREADY,
      output  io_output_PWRITE,
      output [31:0] io_output_PWDATA,
      input  [31:0] io_output_PRDATA,
      input   io_output_PSLVERROR);
  wire [19:0] _zz_Apb3Decoder_1_;
  wire [19:0] _zz_Apb3Decoder_2_;
  wire [19:0] _zz_Apb3Decoder_3_;
  wire [19:0] _zz_Apb3Decoder_4_;
  wire [19:0] _zz_Apb3Decoder_5_;
  wire [19:0] _zz_Apb3Decoder_6_;
  wire [19:0] _zz_Apb3Decoder_7_;
  wire [19:0] _zz_Apb3Decoder_8_;
  wire [19:0] _zz_Apb3Decoder_9_;
  wire [19:0] _zz_Apb3Decoder_10_;
  assign _zz_Apb3Decoder_1_ = (20'b11111111111100000000);
  assign _zz_Apb3Decoder_2_ = (20'b11111111111100000000);
  assign _zz_Apb3Decoder_3_ = (20'b11111111111100000000);
  assign _zz_Apb3Decoder_4_ = (20'b11111111111100000000);
  assign _zz_Apb3Decoder_5_ = (20'b11111111111100000000);
  assign _zz_Apb3Decoder_6_ = (20'b11111111111111110000);
  assign _zz_Apb3Decoder_7_ = (20'b11111111111100000000);
  assign _zz_Apb3Decoder_8_ = (20'b11111111000000000000);
  assign _zz_Apb3Decoder_9_ = (20'b11110000000000000000);
  assign _zz_Apb3Decoder_10_ = (20'b11111111000000000000);
  assign io_output_PADDR = io_input_PADDR;
  assign io_output_PENABLE = io_input_PENABLE;
  assign io_output_PWRITE = io_input_PWRITE;
  assign io_output_PWDATA = io_input_PWDATA;
  always @ (*) begin
    io_output_PSEL[0] = (((io_input_PADDR & _zz_Apb3Decoder_1_) == (20'b00000000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[1] = (((io_input_PADDR & _zz_Apb3Decoder_2_) == (20'b00000000000100000000)) && io_input_PSEL[0]);
    io_output_PSEL[2] = (((io_input_PADDR & _zz_Apb3Decoder_3_) == (20'b00000000001000000000)) && io_input_PSEL[0]);
    io_output_PSEL[3] = (((io_input_PADDR & _zz_Apb3Decoder_4_) == (20'b00000000001100000000)) && io_input_PSEL[0]);
    io_output_PSEL[4] = (((io_input_PADDR & _zz_Apb3Decoder_5_) == (20'b00000000010000000000)) && io_input_PSEL[0]);
    io_output_PSEL[5] = (((io_input_PADDR & _zz_Apb3Decoder_6_) == (20'b00000000011000000000)) && io_input_PSEL[0]);
    io_output_PSEL[6] = (((io_input_PADDR & _zz_Apb3Decoder_7_) == (20'b00000000011100000000)) && io_input_PSEL[0]);
    io_output_PSEL[7] = (((io_input_PADDR & _zz_Apb3Decoder_8_) == (20'b00010000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[8] = (((io_input_PADDR & _zz_Apb3Decoder_9_) == (20'b00100000000000000000)) && io_input_PSEL[0]);
    io_output_PSEL[9] = (((io_input_PADDR & _zz_Apb3Decoder_10_) == (20'b00110000000000000000)) && io_input_PSEL[0]);
  end

  always @ (*) begin
    io_input_PREADY = io_output_PREADY;
    io_input_PSLVERROR = io_output_PSLVERROR;
    if((io_input_PSEL[0] && (io_output_PSEL == (10'b0000000000))))begin
      io_input_PREADY = 1'b1;
      io_input_PSLVERROR = 1'b1;
    end
  end

  assign io_input_PRDATA = io_output_PRDATA;
endmodule

module Apb3Router (
      input  [19:0] io_input_PADDR,
      input  [9:0] io_input_PSEL,
      input   io_input_PENABLE,
      output  io_input_PREADY,
      input   io_input_PWRITE,
      input  [31:0] io_input_PWDATA,
      output [31:0] io_input_PRDATA,
      output  io_input_PSLVERROR,
      output [19:0] io_outputs_0_PADDR,
      output [0:0] io_outputs_0_PSEL,
      output  io_outputs_0_PENABLE,
      input   io_outputs_0_PREADY,
      output  io_outputs_0_PWRITE,
      output [31:0] io_outputs_0_PWDATA,
      input  [31:0] io_outputs_0_PRDATA,
      input   io_outputs_0_PSLVERROR,
      output [19:0] io_outputs_1_PADDR,
      output [0:0] io_outputs_1_PSEL,
      output  io_outputs_1_PENABLE,
      input   io_outputs_1_PREADY,
      output  io_outputs_1_PWRITE,
      output [31:0] io_outputs_1_PWDATA,
      input  [31:0] io_outputs_1_PRDATA,
      input   io_outputs_1_PSLVERROR,
      output [19:0] io_outputs_2_PADDR,
      output [0:0] io_outputs_2_PSEL,
      output  io_outputs_2_PENABLE,
      input   io_outputs_2_PREADY,
      output  io_outputs_2_PWRITE,
      output [31:0] io_outputs_2_PWDATA,
      input  [31:0] io_outputs_2_PRDATA,
      input   io_outputs_2_PSLVERROR,
      output [19:0] io_outputs_3_PADDR,
      output [0:0] io_outputs_3_PSEL,
      output  io_outputs_3_PENABLE,
      input   io_outputs_3_PREADY,
      output  io_outputs_3_PWRITE,
      output [31:0] io_outputs_3_PWDATA,
      input  [31:0] io_outputs_3_PRDATA,
      input   io_outputs_3_PSLVERROR,
      output [19:0] io_outputs_4_PADDR,
      output [0:0] io_outputs_4_PSEL,
      output  io_outputs_4_PENABLE,
      input   io_outputs_4_PREADY,
      output  io_outputs_4_PWRITE,
      output [31:0] io_outputs_4_PWDATA,
      input  [31:0] io_outputs_4_PRDATA,
      input   io_outputs_4_PSLVERROR,
      output [19:0] io_outputs_5_PADDR,
      output [0:0] io_outputs_5_PSEL,
      output  io_outputs_5_PENABLE,
      input   io_outputs_5_PREADY,
      output  io_outputs_5_PWRITE,
      output [31:0] io_outputs_5_PWDATA,
      input  [31:0] io_outputs_5_PRDATA,
      input   io_outputs_5_PSLVERROR,
      output [19:0] io_outputs_6_PADDR,
      output [0:0] io_outputs_6_PSEL,
      output  io_outputs_6_PENABLE,
      input   io_outputs_6_PREADY,
      output  io_outputs_6_PWRITE,
      output [31:0] io_outputs_6_PWDATA,
      input  [31:0] io_outputs_6_PRDATA,
      input   io_outputs_6_PSLVERROR,
      output [19:0] io_outputs_7_PADDR,
      output [0:0] io_outputs_7_PSEL,
      output  io_outputs_7_PENABLE,
      input   io_outputs_7_PREADY,
      output  io_outputs_7_PWRITE,
      output [31:0] io_outputs_7_PWDATA,
      input  [31:0] io_outputs_7_PRDATA,
      input   io_outputs_7_PSLVERROR,
      output [19:0] io_outputs_8_PADDR,
      output [0:0] io_outputs_8_PSEL,
      output  io_outputs_8_PENABLE,
      input   io_outputs_8_PREADY,
      output  io_outputs_8_PWRITE,
      output [31:0] io_outputs_8_PWDATA,
      input  [31:0] io_outputs_8_PRDATA,
      input   io_outputs_8_PSLVERROR,
      output [19:0] io_outputs_9_PADDR,
      output [0:0] io_outputs_9_PSEL,
      output  io_outputs_9_PENABLE,
      input   io_outputs_9_PREADY,
      output  io_outputs_9_PWRITE,
      output [31:0] io_outputs_9_PWDATA,
      input  [31:0] io_outputs_9_PRDATA,
      input   io_outputs_9_PSLVERROR,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg  _zz_Apb3Router_10_;
  reg [31:0] _zz_Apb3Router_11_;
  reg  _zz_Apb3Router_12_;
  wire  _zz_Apb3Router_1_;
  wire  _zz_Apb3Router_2_;
  wire  _zz_Apb3Router_3_;
  wire  _zz_Apb3Router_4_;
  wire  _zz_Apb3Router_5_;
  wire  _zz_Apb3Router_6_;
  wire  _zz_Apb3Router_7_;
  wire  _zz_Apb3Router_8_;
  wire  _zz_Apb3Router_9_;
  reg [3:0] selIndex;
  always @(*) begin
    case(selIndex)
      4'b0000 : begin
        _zz_Apb3Router_10_ = io_outputs_0_PREADY;
        _zz_Apb3Router_11_ = io_outputs_0_PRDATA;
        _zz_Apb3Router_12_ = io_outputs_0_PSLVERROR;
      end
      4'b0001 : begin
        _zz_Apb3Router_10_ = io_outputs_1_PREADY;
        _zz_Apb3Router_11_ = io_outputs_1_PRDATA;
        _zz_Apb3Router_12_ = io_outputs_1_PSLVERROR;
      end
      4'b0010 : begin
        _zz_Apb3Router_10_ = io_outputs_2_PREADY;
        _zz_Apb3Router_11_ = io_outputs_2_PRDATA;
        _zz_Apb3Router_12_ = io_outputs_2_PSLVERROR;
      end
      4'b0011 : begin
        _zz_Apb3Router_10_ = io_outputs_3_PREADY;
        _zz_Apb3Router_11_ = io_outputs_3_PRDATA;
        _zz_Apb3Router_12_ = io_outputs_3_PSLVERROR;
      end
      4'b0100 : begin
        _zz_Apb3Router_10_ = io_outputs_4_PREADY;
        _zz_Apb3Router_11_ = io_outputs_4_PRDATA;
        _zz_Apb3Router_12_ = io_outputs_4_PSLVERROR;
      end
      4'b0101 : begin
        _zz_Apb3Router_10_ = io_outputs_5_PREADY;
        _zz_Apb3Router_11_ = io_outputs_5_PRDATA;
        _zz_Apb3Router_12_ = io_outputs_5_PSLVERROR;
      end
      4'b0110 : begin
        _zz_Apb3Router_10_ = io_outputs_6_PREADY;
        _zz_Apb3Router_11_ = io_outputs_6_PRDATA;
        _zz_Apb3Router_12_ = io_outputs_6_PSLVERROR;
      end
      4'b0111 : begin
        _zz_Apb3Router_10_ = io_outputs_7_PREADY;
        _zz_Apb3Router_11_ = io_outputs_7_PRDATA;
        _zz_Apb3Router_12_ = io_outputs_7_PSLVERROR;
      end
      4'b1000 : begin
        _zz_Apb3Router_10_ = io_outputs_8_PREADY;
        _zz_Apb3Router_11_ = io_outputs_8_PRDATA;
        _zz_Apb3Router_12_ = io_outputs_8_PSLVERROR;
      end
      default : begin
        _zz_Apb3Router_10_ = io_outputs_9_PREADY;
        _zz_Apb3Router_11_ = io_outputs_9_PRDATA;
        _zz_Apb3Router_12_ = io_outputs_9_PSLVERROR;
      end
    endcase
  end

  assign io_outputs_0_PADDR = io_input_PADDR;
  assign io_outputs_0_PENABLE = io_input_PENABLE;
  assign io_outputs_0_PSEL[0] = io_input_PSEL[0];
  assign io_outputs_0_PWRITE = io_input_PWRITE;
  assign io_outputs_0_PWDATA = io_input_PWDATA;
  assign io_outputs_1_PADDR = io_input_PADDR;
  assign io_outputs_1_PENABLE = io_input_PENABLE;
  assign io_outputs_1_PSEL[0] = io_input_PSEL[1];
  assign io_outputs_1_PWRITE = io_input_PWRITE;
  assign io_outputs_1_PWDATA = io_input_PWDATA;
  assign io_outputs_2_PADDR = io_input_PADDR;
  assign io_outputs_2_PENABLE = io_input_PENABLE;
  assign io_outputs_2_PSEL[0] = io_input_PSEL[2];
  assign io_outputs_2_PWRITE = io_input_PWRITE;
  assign io_outputs_2_PWDATA = io_input_PWDATA;
  assign io_outputs_3_PADDR = io_input_PADDR;
  assign io_outputs_3_PENABLE = io_input_PENABLE;
  assign io_outputs_3_PSEL[0] = io_input_PSEL[3];
  assign io_outputs_3_PWRITE = io_input_PWRITE;
  assign io_outputs_3_PWDATA = io_input_PWDATA;
  assign io_outputs_4_PADDR = io_input_PADDR;
  assign io_outputs_4_PENABLE = io_input_PENABLE;
  assign io_outputs_4_PSEL[0] = io_input_PSEL[4];
  assign io_outputs_4_PWRITE = io_input_PWRITE;
  assign io_outputs_4_PWDATA = io_input_PWDATA;
  assign io_outputs_5_PADDR = io_input_PADDR;
  assign io_outputs_5_PENABLE = io_input_PENABLE;
  assign io_outputs_5_PSEL[0] = io_input_PSEL[5];
  assign io_outputs_5_PWRITE = io_input_PWRITE;
  assign io_outputs_5_PWDATA = io_input_PWDATA;
  assign io_outputs_6_PADDR = io_input_PADDR;
  assign io_outputs_6_PENABLE = io_input_PENABLE;
  assign io_outputs_6_PSEL[0] = io_input_PSEL[6];
  assign io_outputs_6_PWRITE = io_input_PWRITE;
  assign io_outputs_6_PWDATA = io_input_PWDATA;
  assign io_outputs_7_PADDR = io_input_PADDR;
  assign io_outputs_7_PENABLE = io_input_PENABLE;
  assign io_outputs_7_PSEL[0] = io_input_PSEL[7];
  assign io_outputs_7_PWRITE = io_input_PWRITE;
  assign io_outputs_7_PWDATA = io_input_PWDATA;
  assign io_outputs_8_PADDR = io_input_PADDR;
  assign io_outputs_8_PENABLE = io_input_PENABLE;
  assign io_outputs_8_PSEL[0] = io_input_PSEL[8];
  assign io_outputs_8_PWRITE = io_input_PWRITE;
  assign io_outputs_8_PWDATA = io_input_PWDATA;
  assign io_outputs_9_PADDR = io_input_PADDR;
  assign io_outputs_9_PENABLE = io_input_PENABLE;
  assign io_outputs_9_PSEL[0] = io_input_PSEL[9];
  assign io_outputs_9_PWRITE = io_input_PWRITE;
  assign io_outputs_9_PWDATA = io_input_PWDATA;
  assign _zz_Apb3Router_1_ = io_input_PSEL[3];
  assign _zz_Apb3Router_2_ = io_input_PSEL[5];
  assign _zz_Apb3Router_3_ = io_input_PSEL[6];
  assign _zz_Apb3Router_4_ = io_input_PSEL[7];
  assign _zz_Apb3Router_5_ = io_input_PSEL[9];
  assign _zz_Apb3Router_6_ = ((((io_input_PSEL[1] || _zz_Apb3Router_1_) || _zz_Apb3Router_2_) || _zz_Apb3Router_4_) || _zz_Apb3Router_5_);
  assign _zz_Apb3Router_7_ = (((io_input_PSEL[2] || _zz_Apb3Router_1_) || _zz_Apb3Router_3_) || _zz_Apb3Router_4_);
  assign _zz_Apb3Router_8_ = (((io_input_PSEL[4] || _zz_Apb3Router_2_) || _zz_Apb3Router_3_) || _zz_Apb3Router_4_);
  assign _zz_Apb3Router_9_ = (io_input_PSEL[8] || _zz_Apb3Router_5_);
  assign io_input_PREADY = _zz_Apb3Router_10_;
  assign io_input_PRDATA = _zz_Apb3Router_11_;
  assign io_input_PSLVERROR = _zz_Apb3Router_12_;
  always @ (posedge toplevel_main_clk) begin
    selIndex <= {_zz_Apb3Router_9_,{_zz_Apb3Router_8_,{_zz_Apb3Router_7_,_zz_Apb3Router_6_}}};
  end

endmodule

module BufferCC_9_ (
      input  [3:0] io_dataIn,
      output [3:0] io_dataOut,
      input   toplevel_u_vo_clk_gen_vo_clk,
      input   toplevel_u_vo_clk_gen_vo_reset_);
  reg [3:0] buffers_0;
  reg [3:0] buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge toplevel_u_vo_clk_gen_vo_clk) begin
    buffers_0 <= io_dataIn;
    buffers_1 <= buffers_0;
  end

endmodule

module BufferCC_10_ (
      input  [7:0] io_dataIn_r,
      input  [7:0] io_dataIn_g,
      input  [7:0] io_dataIn_b,
      output [7:0] io_dataOut_r,
      output [7:0] io_dataOut_g,
      output [7:0] io_dataOut_b,
      input   toplevel_u_vo_clk_gen_vo_clk,
      input   toplevel_u_vo_clk_gen_vo_reset_);
  reg [7:0] buffers_0_r;
  reg [7:0] buffers_0_g;
  reg [7:0] buffers_0_b;
  reg [7:0] buffers_1_r;
  reg [7:0] buffers_1_g;
  reg [7:0] buffers_1_b;
  assign io_dataOut_r = buffers_1_r;
  assign io_dataOut_g = buffers_1_g;
  assign io_dataOut_b = buffers_1_b;
  always @ (posedge toplevel_u_vo_clk_gen_vo_clk) begin
    buffers_0_r <= io_dataIn_r;
    buffers_0_g <= io_dataIn_g;
    buffers_0_b <= io_dataIn_b;
    buffers_1_r <= buffers_0_r;
    buffers_1_g <= buffers_0_g;
    buffers_1_b <= buffers_0_b;
  end

endmodule

module GmiiRxCtrl (
      input   io_rx_clk,
      input   io_rx_dv,
      input   io_rx_er,
      input  [7:0] io_rx_d,
      output  io_rx_fifo_rd_valid,
      input   io_rx_fifo_rd_ready,
      output [9:0] io_rx_fifo_rd_payload,
      output [15:0] io_rx_fifo_rd_count,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire  u_rx_fifo_io_push_ready;
  wire  u_rx_fifo_io_pop_valid;
  wire [9:0] u_rx_fifo_io_pop_payload;
  wire [11:0] u_rx_fifo_io_pushOccupancy;
  wire [11:0] u_rx_fifo_io_popOccupancy;
  wire  rx_domain_rx_fifo_wr_valid;
  wire  rx_domain_rx_fifo_wr_ready;
  wire [9:0] rx_domain_rx_fifo_wr_payload;
  StreamFifoCC_2_ u_rx_fifo ( 
    .io_push_valid(rx_domain_rx_fifo_wr_valid),
    .io_push_ready(u_rx_fifo_io_push_ready),
    .io_push_payload(rx_domain_rx_fifo_wr_payload),
    .io_pop_valid(u_rx_fifo_io_pop_valid),
    .io_pop_ready(io_rx_fifo_rd_ready),
    .io_pop_payload(u_rx_fifo_io_pop_payload),
    .io_pushOccupancy(u_rx_fifo_io_pushOccupancy),
    .io_popOccupancy(u_rx_fifo_io_popOccupancy),
    .u_gmii_rx_io_rx_clk(io_rx_clk),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  assign rx_domain_rx_fifo_wr_valid = ((io_rx_dv || io_rx_er) && rx_domain_rx_fifo_wr_ready);
  assign rx_domain_rx_fifo_wr_payload = {{io_rx_dv,io_rx_er},io_rx_d};
  assign rx_domain_rx_fifo_wr_ready = u_rx_fifo_io_push_ready;
  assign io_rx_fifo_rd_valid = u_rx_fifo_io_pop_valid;
  assign io_rx_fifo_rd_payload = u_rx_fifo_io_pop_payload;
  assign io_rx_fifo_rd_count = {4'd0, u_rx_fifo_io_popOccupancy};
endmodule

module GmiiTxCtrl (
      input   io_tx_gclk,
      input   io_tx_clk,
      output  io_tx_en,
      output  io_tx_er,
      output [7:0] io_tx_d);
  assign io_tx_en = 1'b0;
  assign io_tx_er = 1'b0;
  assign io_tx_d = (8'b00000000);
endmodule

module BufferCC_11_ (
      input  [1:0] io_initial,
      input  [1:0] io_dataIn,
      output [1:0] io_dataOut,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg [1:0] buffers_0;
  reg [1:0] buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      buffers_0 <= io_initial;
      buffers_1 <= io_initial;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end

endmodule

module BufferCC_12_ (
      input  [1:0] io_initial,
      input  [1:0] io_dataIn,
      output [1:0] io_dataOut,
      input   _zz_BufferCC_12__1_,
      input   _zz_BufferCC_12__2_);
  reg [1:0] buffers_0;
  reg [1:0] buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge _zz_BufferCC_12__1_) begin
    if(!_zz_BufferCC_12__2_) begin
      buffers_0 <= io_initial;
      buffers_1 <= io_initial;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end

endmodule


//BufferCC_13_ remplaced by BufferCC_7_

module BufferCC_14_ (
      input  [10:0] io_initial,
      input  [10:0] io_dataIn,
      output [10:0] io_dataOut,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg [10:0] buffers_0;
  reg [10:0] buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      buffers_0 <= io_initial;
      buffers_1 <= io_initial;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end

endmodule

module BufferCC_15_ (
      input  [10:0] io_initial,
      input  [10:0] io_dataIn,
      output [10:0] io_dataOut,
      input   _zz_BufferCC_15__1_,
      input   _zz_BufferCC_15__2_);
  reg [10:0] buffers_0;
  reg [10:0] buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge _zz_BufferCC_15__1_) begin
    if(!_zz_BufferCC_15__2_) begin
      buffers_0 <= io_initial;
      buffers_1 <= io_initial;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end

endmodule

module BufferCC_16_ (
      input   io_initial,
      input   io_dataIn,
      output  io_dataOut,
      input   _zz_BufferCC_16__1_,
      input   _zz_BufferCC_16__2_);
  reg  buffers_0;
  reg  buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge _zz_BufferCC_16__1_) begin
    if(!_zz_BufferCC_16__2_) begin
      buffers_0 <= io_initial;
      buffers_1 <= io_initial;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end

endmodule

module PulseCCByToggle (
      input   io_pulseIn,
      output  io_pulseOut,
      input   core_u_pano_core_io_ulpi_clk,
      input   _zz_PulseCCByToggle_1_,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire  _zz_PulseCCByToggle_2_;
  wire  bufferCC_19__io_dataOut;
  reg  inArea_target;
  wire  outArea_target;
  reg  outArea_hit;
  BufferCC_7_ bufferCC_19_ ( 
    .io_initial(_zz_PulseCCByToggle_2_),
    .io_dataIn(inArea_target),
    .io_dataOut(bufferCC_19__io_dataOut),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  assign _zz_PulseCCByToggle_2_ = 1'b0;
  assign outArea_target = bufferCC_19__io_dataOut;
  assign io_pulseOut = (outArea_target != outArea_hit);
  always @ (posedge core_u_pano_core_io_ulpi_clk) begin
    if(!_zz_PulseCCByToggle_1_) begin
      inArea_target <= 1'b0;
    end else begin
      if(io_pulseIn)begin
        inArea_target <= (! inArea_target);
      end
    end
  end

  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      outArea_hit <= 1'b0;
    end else begin
      if((outArea_target != outArea_hit))begin
        outArea_hit <= (! outArea_hit);
      end
    end
  end

endmodule

module PulseCCByToggle_1_ (
      input   io_pulseIn,
      output  io_pulseOut,
      input   toplevel_main_clk,
      input   toplevel_main_reset_,
      input   core_u_pano_core_io_ulpi_clk,
      input   _zz_PulseCCByToggle_1__1_);
  wire  _zz_PulseCCByToggle_1__2_;
  wire  bufferCC_19__io_dataOut;
  reg  inArea_target;
  wire  outArea_target;
  reg  outArea_hit;
  BufferCC_8_ bufferCC_19_ ( 
    .io_initial(_zz_PulseCCByToggle_1__2_),
    .io_dataIn(inArea_target),
    .io_dataOut(bufferCC_19__io_dataOut),
    .core_u_pano_core_io_ulpi_clk(core_u_pano_core_io_ulpi_clk),
    ._zz_BufferCC_8__1_(_zz_PulseCCByToggle_1__1_) 
  );
  assign _zz_PulseCCByToggle_1__2_ = 1'b0;
  assign outArea_target = bufferCC_19__io_dataOut;
  assign io_pulseOut = (outArea_target != outArea_hit);
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      inArea_target <= 1'b0;
    end else begin
      if(io_pulseIn)begin
        inArea_target <= (! inArea_target);
      end
    end
  end

  always @ (posedge core_u_pano_core_io_ulpi_clk) begin
    if(!_zz_PulseCCByToggle_1__1_) begin
      outArea_hit <= 1'b0;
    end else begin
      if((outArea_target != outArea_hit))begin
        outArea_hit <= (! outArea_hit);
      end
    end
  end

endmodule

module BufferCC_17_ (
      input  [2:0] io_dataIn,
      output [2:0] io_dataOut,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg [2:0] buffers_0;
  reg [2:0] buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge toplevel_main_clk) begin
    buffers_0 <= io_dataIn;
    buffers_1 <= buffers_0;
  end

endmodule

module SpiMasterCtrl (
      input   io_config_kind_cpol,
      input   io_config_kind_cpha,
      input  [15:0] io_config_sclkToogle,
      input  [0:0] io_config_ss_activeHigh,
      input  [15:0] io_config_ss_setup,
      input  [15:0] io_config_ss_hold,
      input  [15:0] io_config_ss_disable,
      input   io_cmd_valid,
      output reg  io_cmd_ready,
      input  `SpiMasterCtrlCmdMode_defaultEncoding_type io_cmd_payload_mode,
      input  [8:0] io_cmd_payload_args,
      output  io_rsp_valid,
      output [7:0] io_rsp_payload,
      output [0:0] io_spi_ss,
      output  io_spi_sclk,
      output  io_spi_mosi,
      input   io_spi_miso,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire  _zz_SpiMasterCtrl_4_;
  wire  _zz_SpiMasterCtrl_5_;
  wire  _zz_SpiMasterCtrl_6_;
  wire  _zz_SpiMasterCtrl_7_;
  wire [0:0] _zz_SpiMasterCtrl_8_;
  wire [3:0] _zz_SpiMasterCtrl_9_;
  wire [8:0] _zz_SpiMasterCtrl_10_;
  wire [0:0] _zz_SpiMasterCtrl_11_;
  wire [0:0] _zz_SpiMasterCtrl_12_;
  wire [7:0] _zz_SpiMasterCtrl_13_;
  wire [2:0] _zz_SpiMasterCtrl_14_;
  wire [2:0] _zz_SpiMasterCtrl_15_;
  reg [15:0] timer_counter;
  reg  timer_reset;
  wire  timer_ss_setupHit;
  wire  timer_ss_holdHit;
  wire  timer_ss_disableHit;
  wire  timer_sclkToogleHit;
  reg  fsm_counter_willIncrement;
  wire  fsm_counter_willClear;
  reg [3:0] fsm_counter_valueNext;
  reg [3:0] fsm_counter_value;
  wire  fsm_counter_willOverflowIfInc;
  wire  fsm_counter_willOverflow;
  reg [7:0] fsm_buffer;
  reg [0:0] fsm_ss;
  reg  _zz_SpiMasterCtrl_1_;
  reg  _zz_SpiMasterCtrl_2_;
  reg  _zz_SpiMasterCtrl_3_;
  `ifndef SYNTHESIS
  reg [31:0] io_cmd_payload_mode_string;
  `endif

  assign _zz_SpiMasterCtrl_4_ = (io_cmd_payload_mode == `SpiMasterCtrlCmdMode_defaultEncoding_DATA);
  assign _zz_SpiMasterCtrl_5_ = _zz_SpiMasterCtrl_11_[0];
  assign _zz_SpiMasterCtrl_6_ = (! fsm_counter_value[0]);
  assign _zz_SpiMasterCtrl_7_ = ((! io_cmd_valid) || io_cmd_ready);
  assign _zz_SpiMasterCtrl_8_ = fsm_counter_willIncrement;
  assign _zz_SpiMasterCtrl_9_ = {3'd0, _zz_SpiMasterCtrl_8_};
  assign _zz_SpiMasterCtrl_10_ = {fsm_buffer,io_spi_miso};
  assign _zz_SpiMasterCtrl_11_ = io_cmd_payload_args[0 : 0];
  assign _zz_SpiMasterCtrl_12_ = io_cmd_payload_args[8 : 8];
  assign _zz_SpiMasterCtrl_13_ = io_cmd_payload_args[7 : 0];
  assign _zz_SpiMasterCtrl_14_ = ((3'b111) - _zz_SpiMasterCtrl_15_);
  assign _zz_SpiMasterCtrl_15_ = (fsm_counter_value >>> 1);
  `ifndef SYNTHESIS
  always @(*) begin
    case(io_cmd_payload_mode)
      `SpiMasterCtrlCmdMode_defaultEncoding_DATA : io_cmd_payload_mode_string = "DATA";
      `SpiMasterCtrlCmdMode_defaultEncoding_SS : io_cmd_payload_mode_string = "SS  ";
      default : io_cmd_payload_mode_string = "????";
    endcase
  end
  `endif

  always @ (*) begin
    timer_reset = 1'b0;
    fsm_counter_willIncrement = 1'b0;
    io_cmd_ready = 1'b0;
    if(io_cmd_valid)begin
      if(_zz_SpiMasterCtrl_4_)begin
        if(timer_sclkToogleHit)begin
          fsm_counter_willIncrement = 1'b1;
          timer_reset = 1'b1;
          io_cmd_ready = fsm_counter_willOverflowIfInc;
        end
      end else begin
        if(_zz_SpiMasterCtrl_5_)begin
          if(timer_ss_setupHit)begin
            io_cmd_ready = 1'b1;
          end
        end else begin
          if(_zz_SpiMasterCtrl_6_)begin
            if(timer_ss_holdHit)begin
              fsm_counter_willIncrement = 1'b1;
              timer_reset = 1'b1;
            end
          end else begin
            if(timer_ss_disableHit)begin
              io_cmd_ready = 1'b1;
            end
          end
        end
      end
    end
    if(_zz_SpiMasterCtrl_7_)begin
      timer_reset = 1'b1;
    end
  end

  assign timer_ss_setupHit = (timer_counter == io_config_ss_setup);
  assign timer_ss_holdHit = (timer_counter == io_config_ss_hold);
  assign timer_ss_disableHit = (timer_counter == io_config_ss_disable);
  assign timer_sclkToogleHit = (timer_counter == io_config_sclkToogle);
  assign fsm_counter_willClear = 1'b0;
  assign fsm_counter_willOverflowIfInc = (fsm_counter_value == (4'b1111));
  assign fsm_counter_willOverflow = (fsm_counter_willOverflowIfInc && fsm_counter_willIncrement);
  always @ (*) begin
    fsm_counter_valueNext = (fsm_counter_value + _zz_SpiMasterCtrl_9_);
    if(fsm_counter_willClear)begin
      fsm_counter_valueNext = (4'b0000);
    end
  end

  assign io_rsp_valid = _zz_SpiMasterCtrl_1_;
  assign io_rsp_payload = fsm_buffer;
  assign io_spi_ss = (fsm_ss ^ io_config_ss_activeHigh);
  assign io_spi_sclk = _zz_SpiMasterCtrl_2_;
  assign io_spi_mosi = _zz_SpiMasterCtrl_3_;
  always @ (posedge toplevel_main_clk) begin
    timer_counter <= (timer_counter + (16'b0000000000000001));
    if(timer_reset)begin
      timer_counter <= (16'b0000000000000000);
    end
    if(io_cmd_valid)begin
      if(_zz_SpiMasterCtrl_4_)begin
        if(timer_sclkToogleHit)begin
          if(fsm_counter_value[0])begin
            fsm_buffer <= _zz_SpiMasterCtrl_10_[7:0];
          end
        end
      end
    end
    _zz_SpiMasterCtrl_2_ <= (((io_cmd_valid && (io_cmd_payload_mode == `SpiMasterCtrlCmdMode_defaultEncoding_DATA)) && (fsm_counter_value[0] ^ io_config_kind_cpha)) ^ io_config_kind_cpol);
    _zz_SpiMasterCtrl_3_ <= _zz_SpiMasterCtrl_13_[_zz_SpiMasterCtrl_14_];
  end

  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      fsm_counter_value <= (4'b0000);
      fsm_ss <= (1'b1);
      _zz_SpiMasterCtrl_1_ <= 1'b0;
    end else begin
      fsm_counter_value <= fsm_counter_valueNext;
      if(io_cmd_valid)begin
        if(! _zz_SpiMasterCtrl_4_) begin
          if(_zz_SpiMasterCtrl_5_)begin
            fsm_ss[0] <= 1'b0;
          end else begin
            if(! _zz_SpiMasterCtrl_6_) begin
              fsm_ss[0] <= 1'b1;
            end
          end
        end
      end
      _zz_SpiMasterCtrl_1_ <= (((io_cmd_valid && io_cmd_ready) && (io_cmd_payload_mode == `SpiMasterCtrlCmdMode_defaultEncoding_DATA)) && _zz_SpiMasterCtrl_12_[0]);
      if(_zz_SpiMasterCtrl_7_)begin
        fsm_counter_value <= (4'b0000);
      end
    end
  end

endmodule

module StreamFifo (
      input   io_push_valid,
      output  io_push_ready,
      input  `SpiMasterCtrlCmdMode_defaultEncoding_type io_push_payload_mode,
      input  [8:0] io_push_payload_args,
      output  io_pop_valid,
      input   io_pop_ready,
      output `SpiMasterCtrlCmdMode_defaultEncoding_type io_pop_payload_mode,
      output [8:0] io_pop_payload_args,
      input   io_flush,
      output [5:0] io_occupancy,
      output [5:0] io_availability,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg [9:0] _zz_StreamFifo_6_;
  wire [0:0] _zz_StreamFifo_7_;
  wire [4:0] _zz_StreamFifo_8_;
  wire [0:0] _zz_StreamFifo_9_;
  wire [4:0] _zz_StreamFifo_10_;
  wire [4:0] _zz_StreamFifo_11_;
  wire  _zz_StreamFifo_12_;
  wire [9:0] _zz_StreamFifo_13_;
  reg  _zz_StreamFifo_1_;
  reg  logic_pushPtr_willIncrement;
  reg  logic_pushPtr_willClear;
  reg [4:0] logic_pushPtr_valueNext;
  reg [4:0] logic_pushPtr_value;
  wire  logic_pushPtr_willOverflowIfInc;
  wire  logic_pushPtr_willOverflow;
  reg  logic_popPtr_willIncrement;
  reg  logic_popPtr_willClear;
  reg [4:0] logic_popPtr_valueNext;
  reg [4:0] logic_popPtr_value;
  wire  logic_popPtr_willOverflowIfInc;
  wire  logic_popPtr_willOverflow;
  wire  logic_ptrMatch;
  reg  logic_risingOccupancy;
  wire  logic_pushing;
  wire  logic_popping;
  wire  logic_empty;
  wire  logic_full;
  reg  _zz_StreamFifo_2_;
  wire `SpiMasterCtrlCmdMode_defaultEncoding_type _zz_StreamFifo_3_;
  wire [9:0] _zz_StreamFifo_4_;
  wire `SpiMasterCtrlCmdMode_defaultEncoding_type _zz_StreamFifo_5_;
  wire [4:0] logic_ptrDif;
  `ifndef SYNTHESIS
  reg [31:0] io_push_payload_mode_string;
  reg [31:0] io_pop_payload_mode_string;
  reg [31:0] _zz_StreamFifo_3__string;
  reg [31:0] _zz_StreamFifo_5__string;
  `endif

  reg [9:0] logic_ram [0:31];
  assign _zz_StreamFifo_7_ = logic_pushPtr_willIncrement;
  assign _zz_StreamFifo_8_ = {4'd0, _zz_StreamFifo_7_};
  assign _zz_StreamFifo_9_ = logic_popPtr_willIncrement;
  assign _zz_StreamFifo_10_ = {4'd0, _zz_StreamFifo_9_};
  assign _zz_StreamFifo_11_ = (logic_popPtr_value - logic_pushPtr_value);
  assign _zz_StreamFifo_12_ = 1'b1;
  assign _zz_StreamFifo_13_ = {io_push_payload_args,io_push_payload_mode};
  always @ (posedge toplevel_main_clk) begin
    if(_zz_StreamFifo_1_) begin
      logic_ram[logic_pushPtr_value] <= _zz_StreamFifo_13_;
    end
  end

  always @ (posedge toplevel_main_clk) begin
    if(_zz_StreamFifo_12_) begin
      _zz_StreamFifo_6_ <= logic_ram[logic_popPtr_valueNext];
    end
  end

  `ifndef SYNTHESIS
  always @(*) begin
    case(io_push_payload_mode)
      `SpiMasterCtrlCmdMode_defaultEncoding_DATA : io_push_payload_mode_string = "DATA";
      `SpiMasterCtrlCmdMode_defaultEncoding_SS : io_push_payload_mode_string = "SS  ";
      default : io_push_payload_mode_string = "????";
    endcase
  end
  always @(*) begin
    case(io_pop_payload_mode)
      `SpiMasterCtrlCmdMode_defaultEncoding_DATA : io_pop_payload_mode_string = "DATA";
      `SpiMasterCtrlCmdMode_defaultEncoding_SS : io_pop_payload_mode_string = "SS  ";
      default : io_pop_payload_mode_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_StreamFifo_3_)
      `SpiMasterCtrlCmdMode_defaultEncoding_DATA : _zz_StreamFifo_3__string = "DATA";
      `SpiMasterCtrlCmdMode_defaultEncoding_SS : _zz_StreamFifo_3__string = "SS  ";
      default : _zz_StreamFifo_3__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_StreamFifo_5_)
      `SpiMasterCtrlCmdMode_defaultEncoding_DATA : _zz_StreamFifo_5__string = "DATA";
      `SpiMasterCtrlCmdMode_defaultEncoding_SS : _zz_StreamFifo_5__string = "SS  ";
      default : _zz_StreamFifo_5__string = "????";
    endcase
  end
  `endif

  always @ (*) begin
    _zz_StreamFifo_1_ = 1'b0;
    logic_pushPtr_willIncrement = 1'b0;
    if(logic_pushing)begin
      _zz_StreamFifo_1_ = 1'b1;
      logic_pushPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    logic_pushPtr_willClear = 1'b0;
    logic_popPtr_willClear = 1'b0;
    if(io_flush)begin
      logic_pushPtr_willClear = 1'b1;
      logic_popPtr_willClear = 1'b1;
    end
  end

  assign logic_pushPtr_willOverflowIfInc = (logic_pushPtr_value == (5'b11111));
  assign logic_pushPtr_willOverflow = (logic_pushPtr_willOverflowIfInc && logic_pushPtr_willIncrement);
  always @ (*) begin
    logic_pushPtr_valueNext = (logic_pushPtr_value + _zz_StreamFifo_8_);
    if(logic_pushPtr_willClear)begin
      logic_pushPtr_valueNext = (5'b00000);
    end
  end

  always @ (*) begin
    logic_popPtr_willIncrement = 1'b0;
    if(logic_popping)begin
      logic_popPtr_willIncrement = 1'b1;
    end
  end

  assign logic_popPtr_willOverflowIfInc = (logic_popPtr_value == (5'b11111));
  assign logic_popPtr_willOverflow = (logic_popPtr_willOverflowIfInc && logic_popPtr_willIncrement);
  always @ (*) begin
    logic_popPtr_valueNext = (logic_popPtr_value + _zz_StreamFifo_10_);
    if(logic_popPtr_willClear)begin
      logic_popPtr_valueNext = (5'b00000);
    end
  end

  assign logic_ptrMatch = (logic_pushPtr_value == logic_popPtr_value);
  assign logic_pushing = (io_push_valid && io_push_ready);
  assign logic_popping = (io_pop_valid && io_pop_ready);
  assign logic_empty = (logic_ptrMatch && (! logic_risingOccupancy));
  assign logic_full = (logic_ptrMatch && logic_risingOccupancy);
  assign io_push_ready = (! logic_full);
  assign io_pop_valid = ((! logic_empty) && (! (_zz_StreamFifo_2_ && (! logic_full))));
  assign _zz_StreamFifo_4_ = _zz_StreamFifo_6_;
  assign _zz_StreamFifo_5_ = _zz_StreamFifo_4_[0 : 0];
  assign _zz_StreamFifo_3_ = _zz_StreamFifo_5_;
  assign io_pop_payload_mode = _zz_StreamFifo_3_;
  assign io_pop_payload_args = _zz_StreamFifo_4_[9 : 1];
  assign logic_ptrDif = (logic_pushPtr_value - logic_popPtr_value);
  assign io_occupancy = {(logic_risingOccupancy && logic_ptrMatch),logic_ptrDif};
  assign io_availability = {((! logic_risingOccupancy) && logic_ptrMatch),_zz_StreamFifo_11_};
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      logic_pushPtr_value <= (5'b00000);
      logic_popPtr_value <= (5'b00000);
      logic_risingOccupancy <= 1'b0;
      _zz_StreamFifo_2_ <= 1'b0;
    end else begin
      logic_pushPtr_value <= logic_pushPtr_valueNext;
      logic_popPtr_value <= logic_popPtr_valueNext;
      _zz_StreamFifo_2_ <= (logic_popPtr_valueNext == logic_pushPtr_value);
      if((logic_pushing != logic_popping))begin
        logic_risingOccupancy <= logic_pushing;
      end
      if(io_flush)begin
        logic_risingOccupancy <= 1'b0;
      end
    end
  end

endmodule

module StreamFifo_1_ (
      input   io_push_valid,
      output  io_push_ready,
      input  [7:0] io_push_payload,
      output  io_pop_valid,
      input   io_pop_ready,
      output [7:0] io_pop_payload,
      input   io_flush,
      output [5:0] io_occupancy,
      output [5:0] io_availability,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg [7:0] _zz_StreamFifo_1__3_;
  wire [0:0] _zz_StreamFifo_1__4_;
  wire [4:0] _zz_StreamFifo_1__5_;
  wire [0:0] _zz_StreamFifo_1__6_;
  wire [4:0] _zz_StreamFifo_1__7_;
  wire [4:0] _zz_StreamFifo_1__8_;
  wire  _zz_StreamFifo_1__9_;
  reg  _zz_StreamFifo_1__1_;
  reg  logic_pushPtr_willIncrement;
  reg  logic_pushPtr_willClear;
  reg [4:0] logic_pushPtr_valueNext;
  reg [4:0] logic_pushPtr_value;
  wire  logic_pushPtr_willOverflowIfInc;
  wire  logic_pushPtr_willOverflow;
  reg  logic_popPtr_willIncrement;
  reg  logic_popPtr_willClear;
  reg [4:0] logic_popPtr_valueNext;
  reg [4:0] logic_popPtr_value;
  wire  logic_popPtr_willOverflowIfInc;
  wire  logic_popPtr_willOverflow;
  wire  logic_ptrMatch;
  reg  logic_risingOccupancy;
  wire  logic_pushing;
  wire  logic_popping;
  wire  logic_empty;
  wire  logic_full;
  reg  _zz_StreamFifo_1__2_;
  wire [4:0] logic_ptrDif;
  reg [7:0] logic_ram [0:31];
  assign _zz_StreamFifo_1__4_ = logic_pushPtr_willIncrement;
  assign _zz_StreamFifo_1__5_ = {4'd0, _zz_StreamFifo_1__4_};
  assign _zz_StreamFifo_1__6_ = logic_popPtr_willIncrement;
  assign _zz_StreamFifo_1__7_ = {4'd0, _zz_StreamFifo_1__6_};
  assign _zz_StreamFifo_1__8_ = (logic_popPtr_value - logic_pushPtr_value);
  assign _zz_StreamFifo_1__9_ = 1'b1;
  always @ (posedge toplevel_main_clk) begin
    if(_zz_StreamFifo_1__1_) begin
      logic_ram[logic_pushPtr_value] <= io_push_payload;
    end
  end

  always @ (posedge toplevel_main_clk) begin
    if(_zz_StreamFifo_1__9_) begin
      _zz_StreamFifo_1__3_ <= logic_ram[logic_popPtr_valueNext];
    end
  end

  always @ (*) begin
    _zz_StreamFifo_1__1_ = 1'b0;
    logic_pushPtr_willIncrement = 1'b0;
    if(logic_pushing)begin
      _zz_StreamFifo_1__1_ = 1'b1;
      logic_pushPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    logic_pushPtr_willClear = 1'b0;
    logic_popPtr_willClear = 1'b0;
    if(io_flush)begin
      logic_pushPtr_willClear = 1'b1;
      logic_popPtr_willClear = 1'b1;
    end
  end

  assign logic_pushPtr_willOverflowIfInc = (logic_pushPtr_value == (5'b11111));
  assign logic_pushPtr_willOverflow = (logic_pushPtr_willOverflowIfInc && logic_pushPtr_willIncrement);
  always @ (*) begin
    logic_pushPtr_valueNext = (logic_pushPtr_value + _zz_StreamFifo_1__5_);
    if(logic_pushPtr_willClear)begin
      logic_pushPtr_valueNext = (5'b00000);
    end
  end

  always @ (*) begin
    logic_popPtr_willIncrement = 1'b0;
    if(logic_popping)begin
      logic_popPtr_willIncrement = 1'b1;
    end
  end

  assign logic_popPtr_willOverflowIfInc = (logic_popPtr_value == (5'b11111));
  assign logic_popPtr_willOverflow = (logic_popPtr_willOverflowIfInc && logic_popPtr_willIncrement);
  always @ (*) begin
    logic_popPtr_valueNext = (logic_popPtr_value + _zz_StreamFifo_1__7_);
    if(logic_popPtr_willClear)begin
      logic_popPtr_valueNext = (5'b00000);
    end
  end

  assign logic_ptrMatch = (logic_pushPtr_value == logic_popPtr_value);
  assign logic_pushing = (io_push_valid && io_push_ready);
  assign logic_popping = (io_pop_valid && io_pop_ready);
  assign logic_empty = (logic_ptrMatch && (! logic_risingOccupancy));
  assign logic_full = (logic_ptrMatch && logic_risingOccupancy);
  assign io_push_ready = (! logic_full);
  assign io_pop_valid = ((! logic_empty) && (! (_zz_StreamFifo_1__2_ && (! logic_full))));
  assign io_pop_payload = _zz_StreamFifo_1__3_;
  assign logic_ptrDif = (logic_pushPtr_value - logic_popPtr_value);
  assign io_occupancy = {(logic_risingOccupancy && logic_ptrMatch),logic_ptrDif};
  assign io_availability = {((! logic_risingOccupancy) && logic_ptrMatch),_zz_StreamFifo_1__8_};
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      logic_pushPtr_value <= (5'b00000);
      logic_popPtr_value <= (5'b00000);
      logic_risingOccupancy <= 1'b0;
      _zz_StreamFifo_1__2_ <= 1'b0;
    end else begin
      logic_pushPtr_value <= logic_pushPtr_valueNext;
      logic_popPtr_value <= logic_popPtr_valueNext;
      _zz_StreamFifo_1__2_ <= (logic_popPtr_valueNext == logic_pushPtr_value);
      if((logic_pushing != logic_popping))begin
        logic_risingOccupancy <= logic_pushing;
      end
      if(io_flush)begin
        logic_risingOccupancy <= 1'b0;
      end
    end
  end

endmodule

module CpuTop (
      output [3:0] io_led_ctrl_apb_PADDR,
      output [0:0] io_led_ctrl_apb_PSEL,
      output  io_led_ctrl_apb_PENABLE,
      input   io_led_ctrl_apb_PREADY,
      output  io_led_ctrl_apb_PWRITE,
      output [31:0] io_led_ctrl_apb_PWDATA,
      input  [31:0] io_led_ctrl_apb_PRDATA,
      input   io_led_ctrl_apb_PSLVERROR,
      output [4:0] io_dvi_ctrl_apb_PADDR,
      output [0:0] io_dvi_ctrl_apb_PSEL,
      output  io_dvi_ctrl_apb_PENABLE,
      input   io_dvi_ctrl_apb_PREADY,
      output  io_dvi_ctrl_apb_PWRITE,
      output [31:0] io_dvi_ctrl_apb_PWDATA,
      input  [31:0] io_dvi_ctrl_apb_PRDATA,
      input   io_dvi_ctrl_apb_PSLVERROR,
      output [4:0] io_gmii_ctrl_apb_PADDR,
      output [0:0] io_gmii_ctrl_apb_PSEL,
      output  io_gmii_ctrl_apb_PENABLE,
      input   io_gmii_ctrl_apb_PREADY,
      output  io_gmii_ctrl_apb_PWRITE,
      output [31:0] io_gmii_ctrl_apb_PWDATA,
      input  [31:0] io_gmii_ctrl_apb_PRDATA,
      input   io_gmii_ctrl_apb_PSLVERROR,
      output [4:0] io_test_patt_apb_PADDR,
      output [0:0] io_test_patt_apb_PSEL,
      output  io_test_patt_apb_PENABLE,
      input   io_test_patt_apb_PREADY,
      output  io_test_patt_apb_PWRITE,
      output [31:0] io_test_patt_apb_PWDATA,
      input  [31:0] io_test_patt_apb_PRDATA,
      input   io_test_patt_apb_PSLVERROR,
      output [15:0] io_txt_gen_apb_PADDR,
      output [0:0] io_txt_gen_apb_PSEL,
      output  io_txt_gen_apb_PENABLE,
      input   io_txt_gen_apb_PREADY,
      output  io_txt_gen_apb_PWRITE,
      output [31:0] io_txt_gen_apb_PWDATA,
      input  [31:0] io_txt_gen_apb_PRDATA,
      input   io_txt_gen_apb_PSLVERROR,
      output [5:0] io_ulpi_apb_PADDR,
      output [0:0] io_ulpi_apb_PSEL,
      output  io_ulpi_apb_PENABLE,
      input   io_ulpi_apb_PREADY,
      output  io_ulpi_apb_PWRITE,
      output [31:0] io_ulpi_apb_PWDATA,
      input  [31:0] io_ulpi_apb_PRDATA,
      input   io_ulpi_apb_PSLVERROR,
      output [6:0] io_usb_host_apb_PADDR,
      output [0:0] io_usb_host_apb_PSEL,
      output  io_usb_host_apb_PENABLE,
      input   io_usb_host_apb_PREADY,
      output  io_usb_host_apb_PWRITE,
      output [31:0] io_usb_host_apb_PWDATA,
      input  [31:0] io_usb_host_apb_PRDATA,
      input   io_usb_host_apb_PSLVERROR,
      output [7:0] io_spi_flash_ctrl_apb_PADDR,
      output [0:0] io_spi_flash_ctrl_apb_PSEL,
      output  io_spi_flash_ctrl_apb_PENABLE,
      input   io_spi_flash_ctrl_apb_PREADY,
      output  io_spi_flash_ctrl_apb_PWRITE,
      output [31:0] io_spi_flash_ctrl_apb_PWDATA,
      input  [31:0] io_spi_flash_ctrl_apb_PRDATA,
      input   io_switch_,
      output  io_axi1_aw_valid,
      input   io_axi1_aw_ready,
      output [31:0] io_axi1_aw_payload_addr,
      output [3:0] io_axi1_aw_payload_id,
      output [3:0] io_axi1_aw_payload_region,
      output [7:0] io_axi1_aw_payload_len,
      output [2:0] io_axi1_aw_payload_size,
      output [1:0] io_axi1_aw_payload_burst,
      output [0:0] io_axi1_aw_payload_lock,
      output [3:0] io_axi1_aw_payload_cache,
      output [3:0] io_axi1_aw_payload_qos,
      output [2:0] io_axi1_aw_payload_prot,
      output  io_axi1_w_valid,
      input   io_axi1_w_ready,
      output [31:0] io_axi1_w_payload_data,
      output [3:0] io_axi1_w_payload_strb,
      output  io_axi1_w_payload_last,
      input   io_axi1_b_valid,
      output  io_axi1_b_ready,
      input  [3:0] io_axi1_b_payload_id,
      input  [1:0] io_axi1_b_payload_resp,
      output  io_axi1_ar_valid,
      input   io_axi1_ar_ready,
      output [31:0] io_axi1_ar_payload_addr,
      output [3:0] io_axi1_ar_payload_id,
      output [3:0] io_axi1_ar_payload_region,
      output [7:0] io_axi1_ar_payload_len,
      output [2:0] io_axi1_ar_payload_size,
      output [1:0] io_axi1_ar_payload_burst,
      output [0:0] io_axi1_ar_payload_lock,
      output [3:0] io_axi1_ar_payload_cache,
      output [3:0] io_axi1_ar_payload_qos,
      output [2:0] io_axi1_ar_payload_prot,
      input   io_axi1_r_valid,
      output  io_axi1_r_ready,
      input  [31:0] io_axi1_r_payload_data,
      input  [3:0] io_axi1_r_payload_id,
      input  [1:0] io_axi1_r_payload_resp,
      input   io_axi1_r_payload_last,
      output  io_axi2_aw_valid,
      input   io_axi2_aw_ready,
      output [31:0] io_axi2_aw_payload_addr,
      output [3:0] io_axi2_aw_payload_id,
      output [3:0] io_axi2_aw_payload_region,
      output [7:0] io_axi2_aw_payload_len,
      output [2:0] io_axi2_aw_payload_size,
      output [1:0] io_axi2_aw_payload_burst,
      output [0:0] io_axi2_aw_payload_lock,
      output [3:0] io_axi2_aw_payload_cache,
      output [3:0] io_axi2_aw_payload_qos,
      output [2:0] io_axi2_aw_payload_prot,
      output  io_axi2_w_valid,
      input   io_axi2_w_ready,
      output [31:0] io_axi2_w_payload_data,
      output [3:0] io_axi2_w_payload_strb,
      output  io_axi2_w_payload_last,
      input   io_axi2_b_valid,
      output  io_axi2_b_ready,
      input  [3:0] io_axi2_b_payload_id,
      input  [1:0] io_axi2_b_payload_resp,
      output  io_axi2_ar_valid,
      input   io_axi2_ar_ready,
      output [31:0] io_axi2_ar_payload_addr,
      output [3:0] io_axi2_ar_payload_id,
      output [3:0] io_axi2_ar_payload_region,
      output [7:0] io_axi2_ar_payload_len,
      output [2:0] io_axi2_ar_payload_size,
      output [1:0] io_axi2_ar_payload_burst,
      output [0:0] io_axi2_ar_payload_lock,
      output [3:0] io_axi2_ar_payload_cache,
      output [3:0] io_axi2_ar_payload_qos,
      output [2:0] io_axi2_ar_payload_prot,
      input   io_axi2_r_valid,
      output  io_axi2_r_ready,
      input  [31:0] io_axi2_r_payload_data,
      input  [3:0] io_axi2_r_payload_id,
      input  [1:0] io_axi2_r_payload_resp,
      input   io_axi2_r_payload_last,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire  _zz_CpuTop_5_;
  wire  _zz_CpuTop_6_;
  wire  _zz_CpuTop_7_;
  wire  _zz_CpuTop_8_;
  wire [3:0] _zz_CpuTop_9_;
  wire [7:0] _zz_CpuTop_10_;
  wire  _zz_CpuTop_11_;
  wire [19:0] u_cpu_io_apb_PADDR;
  wire [0:0] u_cpu_io_apb_PSEL;
  wire  u_cpu_io_apb_PENABLE;
  wire  u_cpu_io_apb_PWRITE;
  wire [31:0] u_cpu_io_apb_PWDATA;
  wire  u_cpu_io_axiMem1_arw_valid;
  wire [31:0] u_cpu_io_axiMem1_arw_payload_addr;
  wire [3:0] u_cpu_io_axiMem1_arw_payload_id;
  wire [3:0] u_cpu_io_axiMem1_arw_payload_region;
  wire [7:0] u_cpu_io_axiMem1_arw_payload_len;
  wire [2:0] u_cpu_io_axiMem1_arw_payload_size;
  wire [1:0] u_cpu_io_axiMem1_arw_payload_burst;
  wire [0:0] u_cpu_io_axiMem1_arw_payload_lock;
  wire [3:0] u_cpu_io_axiMem1_arw_payload_cache;
  wire [3:0] u_cpu_io_axiMem1_arw_payload_qos;
  wire [2:0] u_cpu_io_axiMem1_arw_payload_prot;
  wire  u_cpu_io_axiMem1_arw_payload_write;
  wire  u_cpu_io_axiMem1_w_valid;
  wire [31:0] u_cpu_io_axiMem1_w_payload_data;
  wire [3:0] u_cpu_io_axiMem1_w_payload_strb;
  wire  u_cpu_io_axiMem1_w_payload_last;
  wire  u_cpu_io_axiMem1_b_ready;
  wire  u_cpu_io_axiMem1_r_ready;
  wire  u_cpu_io_axiMem2_arw_valid;
  wire [31:0] u_cpu_io_axiMem2_arw_payload_addr;
  wire [3:0] u_cpu_io_axiMem2_arw_payload_id;
  wire [3:0] u_cpu_io_axiMem2_arw_payload_region;
  wire [7:0] u_cpu_io_axiMem2_arw_payload_len;
  wire [2:0] u_cpu_io_axiMem2_arw_payload_size;
  wire [1:0] u_cpu_io_axiMem2_arw_payload_burst;
  wire [0:0] u_cpu_io_axiMem2_arw_payload_lock;
  wire [3:0] u_cpu_io_axiMem2_arw_payload_cache;
  wire [3:0] u_cpu_io_axiMem2_arw_payload_qos;
  wire [2:0] u_cpu_io_axiMem2_arw_payload_prot;
  wire  u_cpu_io_axiMem2_arw_payload_write;
  wire  u_cpu_io_axiMem2_w_valid;
  wire [31:0] u_cpu_io_axiMem2_w_payload_data;
  wire [3:0] u_cpu_io_axiMem2_w_payload_strb;
  wire  u_cpu_io_axiMem2_w_payload_last;
  wire  u_cpu_io_axiMem2_b_ready;
  wire  u_cpu_io_axiMem2_r_ready;
  wire  u_cpu_io_jtag_tdo;
  wire  jtagUart_1__io_apb_PREADY;
  wire [31:0] jtagUart_1__io_apb_PRDATA;
  wire  jtagUart_1__io_apb_PSLVERROR;
  wire  u_timer_io_apb_PREADY;
  wire [31:0] u_timer_io_apb_PRDATA;
  wire  u_timer_io_apb_PSLVERROR;
  wire  u_timer_io_interrupt;
  wire  io_apb_decoder_io_input_PREADY;
  wire [31:0] io_apb_decoder_io_input_PRDATA;
  wire  io_apb_decoder_io_input_PSLVERROR;
  wire [19:0] io_apb_decoder_io_output_PADDR;
  wire [9:0] io_apb_decoder_io_output_PSEL;
  wire  io_apb_decoder_io_output_PENABLE;
  wire  io_apb_decoder_io_output_PWRITE;
  wire [31:0] io_apb_decoder_io_output_PWDATA;
  wire  apb3Router_1__io_input_PREADY;
  wire [31:0] apb3Router_1__io_input_PRDATA;
  wire  apb3Router_1__io_input_PSLVERROR;
  wire [19:0] apb3Router_1__io_outputs_0_PADDR;
  wire [0:0] apb3Router_1__io_outputs_0_PSEL;
  wire  apb3Router_1__io_outputs_0_PENABLE;
  wire  apb3Router_1__io_outputs_0_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_0_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_1_PADDR;
  wire [0:0] apb3Router_1__io_outputs_1_PSEL;
  wire  apb3Router_1__io_outputs_1_PENABLE;
  wire  apb3Router_1__io_outputs_1_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_1_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_2_PADDR;
  wire [0:0] apb3Router_1__io_outputs_2_PSEL;
  wire  apb3Router_1__io_outputs_2_PENABLE;
  wire  apb3Router_1__io_outputs_2_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_2_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_3_PADDR;
  wire [0:0] apb3Router_1__io_outputs_3_PSEL;
  wire  apb3Router_1__io_outputs_3_PENABLE;
  wire  apb3Router_1__io_outputs_3_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_3_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_4_PADDR;
  wire [0:0] apb3Router_1__io_outputs_4_PSEL;
  wire  apb3Router_1__io_outputs_4_PENABLE;
  wire  apb3Router_1__io_outputs_4_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_4_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_5_PADDR;
  wire [0:0] apb3Router_1__io_outputs_5_PSEL;
  wire  apb3Router_1__io_outputs_5_PENABLE;
  wire  apb3Router_1__io_outputs_5_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_5_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_6_PADDR;
  wire [0:0] apb3Router_1__io_outputs_6_PSEL;
  wire  apb3Router_1__io_outputs_6_PENABLE;
  wire  apb3Router_1__io_outputs_6_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_6_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_7_PADDR;
  wire [0:0] apb3Router_1__io_outputs_7_PSEL;
  wire  apb3Router_1__io_outputs_7_PENABLE;
  wire  apb3Router_1__io_outputs_7_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_7_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_8_PADDR;
  wire [0:0] apb3Router_1__io_outputs_8_PSEL;
  wire  apb3Router_1__io_outputs_8_PENABLE;
  wire  apb3Router_1__io_outputs_8_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_8_PWDATA;
  wire [19:0] apb3Router_1__io_outputs_9_PADDR;
  wire [0:0] apb3Router_1__io_outputs_9_PSEL;
  wire  apb3Router_1__io_outputs_9_PENABLE;
  wire  apb3Router_1__io_outputs_9_PWRITE;
  wire [31:0] apb3Router_1__io_outputs_9_PWDATA;
  wire  jtag_CAPTURE;
  wire  jtag_DRCK;
  wire  jtag_RESET;
  wire  jtag_RUNTEST;
  wire  jtag_SEL;
  wire  jtag_SHIFT;
  wire  jtag_TCK;
  wire  jtag_TDI;
  wire  jtag_TMS;
  wire  jtag_UPDATE;
  wire [3:0] _zz_CpuTop_1_;
  wire [3:0] _zz_CpuTop_2_;
  wire [3:0] _zz_CpuTop_3_;
  wire [3:0] _zz_CpuTop_4_;
  CpuComplex u_cpu ( 
    .io_apb_PADDR(u_cpu_io_apb_PADDR),
    .io_apb_PSEL(u_cpu_io_apb_PSEL),
    .io_apb_PENABLE(u_cpu_io_apb_PENABLE),
    .io_apb_PREADY(io_apb_decoder_io_input_PREADY),
    .io_apb_PWRITE(u_cpu_io_apb_PWRITE),
    .io_apb_PWDATA(u_cpu_io_apb_PWDATA),
    .io_apb_PRDATA(io_apb_decoder_io_input_PRDATA),
    .io_apb_PSLVERROR(io_apb_decoder_io_input_PSLVERROR),
    .io_axiMem1_arw_valid(u_cpu_io_axiMem1_arw_valid),
    .io_axiMem1_arw_ready(_zz_CpuTop_5_),
    .io_axiMem1_arw_payload_addr(u_cpu_io_axiMem1_arw_payload_addr),
    .io_axiMem1_arw_payload_id(u_cpu_io_axiMem1_arw_payload_id),
    .io_axiMem1_arw_payload_region(u_cpu_io_axiMem1_arw_payload_region),
    .io_axiMem1_arw_payload_len(u_cpu_io_axiMem1_arw_payload_len),
    .io_axiMem1_arw_payload_size(u_cpu_io_axiMem1_arw_payload_size),
    .io_axiMem1_arw_payload_burst(u_cpu_io_axiMem1_arw_payload_burst),
    .io_axiMem1_arw_payload_lock(u_cpu_io_axiMem1_arw_payload_lock),
    .io_axiMem1_arw_payload_cache(u_cpu_io_axiMem1_arw_payload_cache),
    .io_axiMem1_arw_payload_qos(u_cpu_io_axiMem1_arw_payload_qos),
    .io_axiMem1_arw_payload_prot(u_cpu_io_axiMem1_arw_payload_prot),
    .io_axiMem1_arw_payload_write(u_cpu_io_axiMem1_arw_payload_write),
    .io_axiMem1_w_valid(u_cpu_io_axiMem1_w_valid),
    .io_axiMem1_w_ready(io_axi1_w_ready),
    .io_axiMem1_w_payload_data(u_cpu_io_axiMem1_w_payload_data),
    .io_axiMem1_w_payload_strb(u_cpu_io_axiMem1_w_payload_strb),
    .io_axiMem1_w_payload_last(u_cpu_io_axiMem1_w_payload_last),
    .io_axiMem1_b_valid(io_axi1_b_valid),
    .io_axiMem1_b_ready(u_cpu_io_axiMem1_b_ready),
    .io_axiMem1_b_payload_id(_zz_CpuTop_2_),
    .io_axiMem1_b_payload_resp(io_axi1_b_payload_resp),
    .io_axiMem1_r_valid(io_axi1_r_valid),
    .io_axiMem1_r_ready(u_cpu_io_axiMem1_r_ready),
    .io_axiMem1_r_payload_data(io_axi1_r_payload_data),
    .io_axiMem1_r_payload_id(_zz_CpuTop_1_),
    .io_axiMem1_r_payload_resp(io_axi1_r_payload_resp),
    .io_axiMem1_r_payload_last(io_axi1_r_payload_last),
    .io_axiMem2_arw_valid(u_cpu_io_axiMem2_arw_valid),
    .io_axiMem2_arw_ready(_zz_CpuTop_6_),
    .io_axiMem2_arw_payload_addr(u_cpu_io_axiMem2_arw_payload_addr),
    .io_axiMem2_arw_payload_id(u_cpu_io_axiMem2_arw_payload_id),
    .io_axiMem2_arw_payload_region(u_cpu_io_axiMem2_arw_payload_region),
    .io_axiMem2_arw_payload_len(u_cpu_io_axiMem2_arw_payload_len),
    .io_axiMem2_arw_payload_size(u_cpu_io_axiMem2_arw_payload_size),
    .io_axiMem2_arw_payload_burst(u_cpu_io_axiMem2_arw_payload_burst),
    .io_axiMem2_arw_payload_lock(u_cpu_io_axiMem2_arw_payload_lock),
    .io_axiMem2_arw_payload_cache(u_cpu_io_axiMem2_arw_payload_cache),
    .io_axiMem2_arw_payload_qos(u_cpu_io_axiMem2_arw_payload_qos),
    .io_axiMem2_arw_payload_prot(u_cpu_io_axiMem2_arw_payload_prot),
    .io_axiMem2_arw_payload_write(u_cpu_io_axiMem2_arw_payload_write),
    .io_axiMem2_w_valid(u_cpu_io_axiMem2_w_valid),
    .io_axiMem2_w_ready(io_axi2_w_ready),
    .io_axiMem2_w_payload_data(u_cpu_io_axiMem2_w_payload_data),
    .io_axiMem2_w_payload_strb(u_cpu_io_axiMem2_w_payload_strb),
    .io_axiMem2_w_payload_last(u_cpu_io_axiMem2_w_payload_last),
    .io_axiMem2_b_valid(io_axi2_b_valid),
    .io_axiMem2_b_ready(u_cpu_io_axiMem2_b_ready),
    .io_axiMem2_b_payload_id(_zz_CpuTop_4_),
    .io_axiMem2_b_payload_resp(io_axi2_b_payload_resp),
    .io_axiMem2_r_valid(io_axi2_r_valid),
    .io_axiMem2_r_ready(u_cpu_io_axiMem2_r_ready),
    .io_axiMem2_r_payload_data(io_axi2_r_payload_data),
    .io_axiMem2_r_payload_id(_zz_CpuTop_3_),
    .io_axiMem2_r_payload_resp(io_axi2_r_payload_resp),
    .io_axiMem2_r_payload_last(io_axi2_r_payload_last),
    .io_externalInterrupt(_zz_CpuTop_7_),
    .io_timerInterrupt(u_timer_io_interrupt),
    .io_jtag_tms(jtag_TMS),
    .io_jtag_tdi(jtag_TDI),
    .io_jtag_tdo(u_cpu_io_jtag_tdo),
    .io_jtag_tck(jtag_TCK),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_),
    ._zz_CpuComplex_109_(_zz_CpuTop_8_) 
  );
  Apb3JtagUart jtagUart_1_ ( 
    .io_apb_PADDR(_zz_CpuTop_9_),
    .io_apb_PSEL(apb3Router_1__io_outputs_5_PSEL),
    .io_apb_PENABLE(apb3Router_1__io_outputs_5_PENABLE),
    .io_apb_PREADY(jtagUart_1__io_apb_PREADY),
    .io_apb_PWRITE(apb3Router_1__io_outputs_5_PWRITE),
    .io_apb_PWDATA(apb3Router_1__io_outputs_5_PWDATA),
    .io_apb_PRDATA(jtagUart_1__io_apb_PRDATA),
    .io_apb_PSLVERROR(jtagUart_1__io_apb_PSLVERROR),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  MuraxApb3Timer u_timer ( 
    .io_apb_PADDR(_zz_CpuTop_10_),
    .io_apb_PSEL(apb3Router_1__io_outputs_9_PSEL),
    .io_apb_PENABLE(apb3Router_1__io_outputs_9_PENABLE),
    .io_apb_PREADY(u_timer_io_apb_PREADY),
    .io_apb_PWRITE(apb3Router_1__io_outputs_9_PWRITE),
    .io_apb_PWDATA(apb3Router_1__io_outputs_9_PWDATA),
    .io_apb_PRDATA(u_timer_io_apb_PRDATA),
    .io_apb_PSLVERROR(u_timer_io_apb_PSLVERROR),
    .io_interrupt(u_timer_io_interrupt),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  Apb3Decoder io_apb_decoder ( 
    .io_input_PADDR(u_cpu_io_apb_PADDR),
    .io_input_PSEL(u_cpu_io_apb_PSEL),
    .io_input_PENABLE(u_cpu_io_apb_PENABLE),
    .io_input_PREADY(io_apb_decoder_io_input_PREADY),
    .io_input_PWRITE(u_cpu_io_apb_PWRITE),
    .io_input_PWDATA(u_cpu_io_apb_PWDATA),
    .io_input_PRDATA(io_apb_decoder_io_input_PRDATA),
    .io_input_PSLVERROR(io_apb_decoder_io_input_PSLVERROR),
    .io_output_PADDR(io_apb_decoder_io_output_PADDR),
    .io_output_PSEL(io_apb_decoder_io_output_PSEL),
    .io_output_PENABLE(io_apb_decoder_io_output_PENABLE),
    .io_output_PREADY(apb3Router_1__io_input_PREADY),
    .io_output_PWRITE(io_apb_decoder_io_output_PWRITE),
    .io_output_PWDATA(io_apb_decoder_io_output_PWDATA),
    .io_output_PRDATA(apb3Router_1__io_input_PRDATA),
    .io_output_PSLVERROR(apb3Router_1__io_input_PSLVERROR) 
  );
  Apb3Router apb3Router_1_ ( 
    .io_input_PADDR(io_apb_decoder_io_output_PADDR),
    .io_input_PSEL(io_apb_decoder_io_output_PSEL),
    .io_input_PENABLE(io_apb_decoder_io_output_PENABLE),
    .io_input_PREADY(apb3Router_1__io_input_PREADY),
    .io_input_PWRITE(io_apb_decoder_io_output_PWRITE),
    .io_input_PWDATA(io_apb_decoder_io_output_PWDATA),
    .io_input_PRDATA(apb3Router_1__io_input_PRDATA),
    .io_input_PSLVERROR(apb3Router_1__io_input_PSLVERROR),
    .io_outputs_0_PADDR(apb3Router_1__io_outputs_0_PADDR),
    .io_outputs_0_PSEL(apb3Router_1__io_outputs_0_PSEL),
    .io_outputs_0_PENABLE(apb3Router_1__io_outputs_0_PENABLE),
    .io_outputs_0_PREADY(io_led_ctrl_apb_PREADY),
    .io_outputs_0_PWRITE(apb3Router_1__io_outputs_0_PWRITE),
    .io_outputs_0_PWDATA(apb3Router_1__io_outputs_0_PWDATA),
    .io_outputs_0_PRDATA(io_led_ctrl_apb_PRDATA),
    .io_outputs_0_PSLVERROR(io_led_ctrl_apb_PSLVERROR),
    .io_outputs_1_PADDR(apb3Router_1__io_outputs_1_PADDR),
    .io_outputs_1_PSEL(apb3Router_1__io_outputs_1_PSEL),
    .io_outputs_1_PENABLE(apb3Router_1__io_outputs_1_PENABLE),
    .io_outputs_1_PREADY(io_dvi_ctrl_apb_PREADY),
    .io_outputs_1_PWRITE(apb3Router_1__io_outputs_1_PWRITE),
    .io_outputs_1_PWDATA(apb3Router_1__io_outputs_1_PWDATA),
    .io_outputs_1_PRDATA(io_dvi_ctrl_apb_PRDATA),
    .io_outputs_1_PSLVERROR(io_dvi_ctrl_apb_PSLVERROR),
    .io_outputs_2_PADDR(apb3Router_1__io_outputs_2_PADDR),
    .io_outputs_2_PSEL(apb3Router_1__io_outputs_2_PSEL),
    .io_outputs_2_PENABLE(apb3Router_1__io_outputs_2_PENABLE),
    .io_outputs_2_PREADY(io_test_patt_apb_PREADY),
    .io_outputs_2_PWRITE(apb3Router_1__io_outputs_2_PWRITE),
    .io_outputs_2_PWDATA(apb3Router_1__io_outputs_2_PWDATA),
    .io_outputs_2_PRDATA(io_test_patt_apb_PRDATA),
    .io_outputs_2_PSLVERROR(io_test_patt_apb_PSLVERROR),
    .io_outputs_3_PADDR(apb3Router_1__io_outputs_3_PADDR),
    .io_outputs_3_PSEL(apb3Router_1__io_outputs_3_PSEL),
    .io_outputs_3_PENABLE(apb3Router_1__io_outputs_3_PENABLE),
    .io_outputs_3_PREADY(io_ulpi_apb_PREADY),
    .io_outputs_3_PWRITE(apb3Router_1__io_outputs_3_PWRITE),
    .io_outputs_3_PWDATA(apb3Router_1__io_outputs_3_PWDATA),
    .io_outputs_3_PRDATA(io_ulpi_apb_PRDATA),
    .io_outputs_3_PSLVERROR(io_ulpi_apb_PSLVERROR),
    .io_outputs_4_PADDR(apb3Router_1__io_outputs_4_PADDR),
    .io_outputs_4_PSEL(apb3Router_1__io_outputs_4_PSEL),
    .io_outputs_4_PENABLE(apb3Router_1__io_outputs_4_PENABLE),
    .io_outputs_4_PREADY(io_usb_host_apb_PREADY),
    .io_outputs_4_PWRITE(apb3Router_1__io_outputs_4_PWRITE),
    .io_outputs_4_PWDATA(apb3Router_1__io_outputs_4_PWDATA),
    .io_outputs_4_PRDATA(io_usb_host_apb_PRDATA),
    .io_outputs_4_PSLVERROR(io_usb_host_apb_PSLVERROR),
    .io_outputs_5_PADDR(apb3Router_1__io_outputs_5_PADDR),
    .io_outputs_5_PSEL(apb3Router_1__io_outputs_5_PSEL),
    .io_outputs_5_PENABLE(apb3Router_1__io_outputs_5_PENABLE),
    .io_outputs_5_PREADY(jtagUart_1__io_apb_PREADY),
    .io_outputs_5_PWRITE(apb3Router_1__io_outputs_5_PWRITE),
    .io_outputs_5_PWDATA(apb3Router_1__io_outputs_5_PWDATA),
    .io_outputs_5_PRDATA(jtagUart_1__io_apb_PRDATA),
    .io_outputs_5_PSLVERROR(jtagUart_1__io_apb_PSLVERROR),
    .io_outputs_6_PADDR(apb3Router_1__io_outputs_6_PADDR),
    .io_outputs_6_PSEL(apb3Router_1__io_outputs_6_PSEL),
    .io_outputs_6_PENABLE(apb3Router_1__io_outputs_6_PENABLE),
    .io_outputs_6_PREADY(io_spi_flash_ctrl_apb_PREADY),
    .io_outputs_6_PWRITE(apb3Router_1__io_outputs_6_PWRITE),
    .io_outputs_6_PWDATA(apb3Router_1__io_outputs_6_PWDATA),
    .io_outputs_6_PRDATA(io_spi_flash_ctrl_apb_PRDATA),
    .io_outputs_6_PSLVERROR(_zz_CpuTop_11_),
    .io_outputs_7_PADDR(apb3Router_1__io_outputs_7_PADDR),
    .io_outputs_7_PSEL(apb3Router_1__io_outputs_7_PSEL),
    .io_outputs_7_PENABLE(apb3Router_1__io_outputs_7_PENABLE),
    .io_outputs_7_PREADY(io_gmii_ctrl_apb_PREADY),
    .io_outputs_7_PWRITE(apb3Router_1__io_outputs_7_PWRITE),
    .io_outputs_7_PWDATA(apb3Router_1__io_outputs_7_PWDATA),
    .io_outputs_7_PRDATA(io_gmii_ctrl_apb_PRDATA),
    .io_outputs_7_PSLVERROR(io_gmii_ctrl_apb_PSLVERROR),
    .io_outputs_8_PADDR(apb3Router_1__io_outputs_8_PADDR),
    .io_outputs_8_PSEL(apb3Router_1__io_outputs_8_PSEL),
    .io_outputs_8_PENABLE(apb3Router_1__io_outputs_8_PENABLE),
    .io_outputs_8_PREADY(io_txt_gen_apb_PREADY),
    .io_outputs_8_PWRITE(apb3Router_1__io_outputs_8_PWRITE),
    .io_outputs_8_PWDATA(apb3Router_1__io_outputs_8_PWDATA),
    .io_outputs_8_PRDATA(io_txt_gen_apb_PRDATA),
    .io_outputs_8_PSLVERROR(io_txt_gen_apb_PSLVERROR),
    .io_outputs_9_PADDR(apb3Router_1__io_outputs_9_PADDR),
    .io_outputs_9_PSEL(apb3Router_1__io_outputs_9_PSEL),
    .io_outputs_9_PENABLE(apb3Router_1__io_outputs_9_PENABLE),
    .io_outputs_9_PREADY(u_timer_io_apb_PREADY),
    .io_outputs_9_PWRITE(apb3Router_1__io_outputs_9_PWRITE),
    .io_outputs_9_PWDATA(apb3Router_1__io_outputs_9_PWDATA),
    .io_outputs_9_PRDATA(u_timer_io_apb_PRDATA),
    .io_outputs_9_PSLVERROR(u_timer_io_apb_PSLVERROR),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  BSCAN_SPARTAN6 #( 
    .JTAG_CHAIN(1) 
  ) jtag ( 
    .CAPTURE(jtag_CAPTURE),
    .DRCK(jtag_DRCK),
    .RESET(jtag_RESET),
    .RUNTEST(jtag_RUNTEST),
    .SEL(jtag_SEL),
    .SHIFT(jtag_SHIFT),
    .TCK(jtag_TCK),
    .TDI(jtag_TDI),
    .TMS(jtag_TMS),
    .UPDATE(jtag_UPDATE),
    .TDO(u_cpu_io_jtag_tdo) 
  );
  assign _zz_CpuTop_7_ = 1'b0;
  assign _zz_CpuTop_5_ = (u_cpu_io_axiMem1_arw_payload_write ? io_axi1_aw_ready : io_axi1_ar_ready);
  assign io_axi1_ar_valid = (u_cpu_io_axiMem1_arw_valid && (! u_cpu_io_axiMem1_arw_payload_write));
  assign io_axi1_ar_payload_addr = u_cpu_io_axiMem1_arw_payload_addr;
  assign io_axi1_ar_payload_id = u_cpu_io_axiMem1_arw_payload_id;
  assign io_axi1_ar_payload_region = u_cpu_io_axiMem1_arw_payload_region;
  assign io_axi1_ar_payload_len = u_cpu_io_axiMem1_arw_payload_len;
  assign io_axi1_ar_payload_size = u_cpu_io_axiMem1_arw_payload_size;
  assign io_axi1_ar_payload_burst = u_cpu_io_axiMem1_arw_payload_burst;
  assign io_axi1_ar_payload_lock = u_cpu_io_axiMem1_arw_payload_lock;
  assign io_axi1_ar_payload_cache = u_cpu_io_axiMem1_arw_payload_cache;
  assign io_axi1_ar_payload_qos = u_cpu_io_axiMem1_arw_payload_qos;
  assign io_axi1_ar_payload_prot = u_cpu_io_axiMem1_arw_payload_prot;
  assign io_axi1_aw_valid = (u_cpu_io_axiMem1_arw_valid && u_cpu_io_axiMem1_arw_payload_write);
  assign io_axi1_aw_payload_addr = u_cpu_io_axiMem1_arw_payload_addr;
  assign io_axi1_aw_payload_id = u_cpu_io_axiMem1_arw_payload_id;
  assign io_axi1_aw_payload_region = u_cpu_io_axiMem1_arw_payload_region;
  assign io_axi1_aw_payload_len = u_cpu_io_axiMem1_arw_payload_len;
  assign io_axi1_aw_payload_size = u_cpu_io_axiMem1_arw_payload_size;
  assign io_axi1_aw_payload_burst = u_cpu_io_axiMem1_arw_payload_burst;
  assign io_axi1_aw_payload_lock = u_cpu_io_axiMem1_arw_payload_lock;
  assign io_axi1_aw_payload_cache = u_cpu_io_axiMem1_arw_payload_cache;
  assign io_axi1_aw_payload_qos = u_cpu_io_axiMem1_arw_payload_qos;
  assign io_axi1_aw_payload_prot = u_cpu_io_axiMem1_arw_payload_prot;
  assign io_axi1_w_valid = u_cpu_io_axiMem1_w_valid;
  assign io_axi1_w_payload_data = u_cpu_io_axiMem1_w_payload_data;
  assign io_axi1_w_payload_strb = u_cpu_io_axiMem1_w_payload_strb;
  assign io_axi1_w_payload_last = u_cpu_io_axiMem1_w_payload_last;
  assign io_axi1_r_ready = u_cpu_io_axiMem1_r_ready;
  assign _zz_CpuTop_1_ = io_axi1_r_payload_id;
  assign io_axi1_b_ready = u_cpu_io_axiMem1_b_ready;
  assign _zz_CpuTop_2_ = io_axi1_b_payload_id;
  assign _zz_CpuTop_6_ = (u_cpu_io_axiMem2_arw_payload_write ? io_axi2_aw_ready : io_axi2_ar_ready);
  assign io_axi2_ar_valid = (u_cpu_io_axiMem2_arw_valid && (! u_cpu_io_axiMem2_arw_payload_write));
  assign io_axi2_ar_payload_addr = u_cpu_io_axiMem2_arw_payload_addr;
  assign io_axi2_ar_payload_id = u_cpu_io_axiMem2_arw_payload_id;
  assign io_axi2_ar_payload_region = u_cpu_io_axiMem2_arw_payload_region;
  assign io_axi2_ar_payload_len = u_cpu_io_axiMem2_arw_payload_len;
  assign io_axi2_ar_payload_size = u_cpu_io_axiMem2_arw_payload_size;
  assign io_axi2_ar_payload_burst = u_cpu_io_axiMem2_arw_payload_burst;
  assign io_axi2_ar_payload_lock = u_cpu_io_axiMem2_arw_payload_lock;
  assign io_axi2_ar_payload_cache = u_cpu_io_axiMem2_arw_payload_cache;
  assign io_axi2_ar_payload_qos = u_cpu_io_axiMem2_arw_payload_qos;
  assign io_axi2_ar_payload_prot = u_cpu_io_axiMem2_arw_payload_prot;
  assign io_axi2_aw_valid = (u_cpu_io_axiMem2_arw_valid && u_cpu_io_axiMem2_arw_payload_write);
  assign io_axi2_aw_payload_addr = u_cpu_io_axiMem2_arw_payload_addr;
  assign io_axi2_aw_payload_id = u_cpu_io_axiMem2_arw_payload_id;
  assign io_axi2_aw_payload_region = u_cpu_io_axiMem2_arw_payload_region;
  assign io_axi2_aw_payload_len = u_cpu_io_axiMem2_arw_payload_len;
  assign io_axi2_aw_payload_size = u_cpu_io_axiMem2_arw_payload_size;
  assign io_axi2_aw_payload_burst = u_cpu_io_axiMem2_arw_payload_burst;
  assign io_axi2_aw_payload_lock = u_cpu_io_axiMem2_arw_payload_lock;
  assign io_axi2_aw_payload_cache = u_cpu_io_axiMem2_arw_payload_cache;
  assign io_axi2_aw_payload_qos = u_cpu_io_axiMem2_arw_payload_qos;
  assign io_axi2_aw_payload_prot = u_cpu_io_axiMem2_arw_payload_prot;
  assign io_axi2_w_valid = u_cpu_io_axiMem2_w_valid;
  assign io_axi2_w_payload_data = u_cpu_io_axiMem2_w_payload_data;
  assign io_axi2_w_payload_strb = u_cpu_io_axiMem2_w_payload_strb;
  assign io_axi2_w_payload_last = u_cpu_io_axiMem2_w_payload_last;
  assign io_axi2_r_ready = u_cpu_io_axiMem2_r_ready;
  assign _zz_CpuTop_3_ = io_axi2_r_payload_id;
  assign io_axi2_b_ready = u_cpu_io_axiMem2_b_ready;
  assign _zz_CpuTop_4_ = io_axi2_b_payload_id;
  assign io_led_ctrl_apb_PADDR = apb3Router_1__io_outputs_0_PADDR[3:0];
  assign io_led_ctrl_apb_PSEL = apb3Router_1__io_outputs_0_PSEL;
  assign io_led_ctrl_apb_PENABLE = apb3Router_1__io_outputs_0_PENABLE;
  assign io_led_ctrl_apb_PWRITE = apb3Router_1__io_outputs_0_PWRITE;
  assign io_led_ctrl_apb_PWDATA = apb3Router_1__io_outputs_0_PWDATA;
  assign io_dvi_ctrl_apb_PADDR = apb3Router_1__io_outputs_1_PADDR[4:0];
  assign io_dvi_ctrl_apb_PSEL = apb3Router_1__io_outputs_1_PSEL;
  assign io_dvi_ctrl_apb_PENABLE = apb3Router_1__io_outputs_1_PENABLE;
  assign io_dvi_ctrl_apb_PWRITE = apb3Router_1__io_outputs_1_PWRITE;
  assign io_dvi_ctrl_apb_PWDATA = apb3Router_1__io_outputs_1_PWDATA;
  assign io_test_patt_apb_PADDR = apb3Router_1__io_outputs_2_PADDR[4:0];
  assign io_test_patt_apb_PSEL = apb3Router_1__io_outputs_2_PSEL;
  assign io_test_patt_apb_PENABLE = apb3Router_1__io_outputs_2_PENABLE;
  assign io_test_patt_apb_PWRITE = apb3Router_1__io_outputs_2_PWRITE;
  assign io_test_patt_apb_PWDATA = apb3Router_1__io_outputs_2_PWDATA;
  assign io_ulpi_apb_PADDR = apb3Router_1__io_outputs_3_PADDR[5:0];
  assign io_ulpi_apb_PSEL = apb3Router_1__io_outputs_3_PSEL;
  assign io_ulpi_apb_PENABLE = apb3Router_1__io_outputs_3_PENABLE;
  assign io_ulpi_apb_PWRITE = apb3Router_1__io_outputs_3_PWRITE;
  assign io_ulpi_apb_PWDATA = apb3Router_1__io_outputs_3_PWDATA;
  assign io_usb_host_apb_PADDR = apb3Router_1__io_outputs_4_PADDR[6:0];
  assign io_usb_host_apb_PSEL = apb3Router_1__io_outputs_4_PSEL;
  assign io_usb_host_apb_PENABLE = apb3Router_1__io_outputs_4_PENABLE;
  assign io_usb_host_apb_PWRITE = apb3Router_1__io_outputs_4_PWRITE;
  assign io_usb_host_apb_PWDATA = apb3Router_1__io_outputs_4_PWDATA;
  assign _zz_CpuTop_9_ = apb3Router_1__io_outputs_5_PADDR[3:0];
  assign io_spi_flash_ctrl_apb_PADDR = apb3Router_1__io_outputs_6_PADDR[7:0];
  assign io_spi_flash_ctrl_apb_PSEL = apb3Router_1__io_outputs_6_PSEL;
  assign io_spi_flash_ctrl_apb_PENABLE = apb3Router_1__io_outputs_6_PENABLE;
  assign io_spi_flash_ctrl_apb_PWRITE = apb3Router_1__io_outputs_6_PWRITE;
  assign io_spi_flash_ctrl_apb_PWDATA = apb3Router_1__io_outputs_6_PWDATA;
  assign _zz_CpuTop_11_ = 1'b0;
  assign io_gmii_ctrl_apb_PADDR = apb3Router_1__io_outputs_7_PADDR[4:0];
  assign io_gmii_ctrl_apb_PSEL = apb3Router_1__io_outputs_7_PSEL;
  assign io_gmii_ctrl_apb_PENABLE = apb3Router_1__io_outputs_7_PENABLE;
  assign io_gmii_ctrl_apb_PWRITE = apb3Router_1__io_outputs_7_PWRITE;
  assign io_gmii_ctrl_apb_PWDATA = apb3Router_1__io_outputs_7_PWDATA;
  assign io_txt_gen_apb_PADDR = apb3Router_1__io_outputs_8_PADDR[15:0];
  assign io_txt_gen_apb_PSEL = apb3Router_1__io_outputs_8_PSEL;
  assign io_txt_gen_apb_PENABLE = apb3Router_1__io_outputs_8_PENABLE;
  assign io_txt_gen_apb_PWRITE = apb3Router_1__io_outputs_8_PWRITE;
  assign io_txt_gen_apb_PWDATA = apb3Router_1__io_outputs_8_PWDATA;
  assign _zz_CpuTop_10_ = apb3Router_1__io_outputs_9_PADDR[7:0];
  assign _zz_CpuTop_8_ = 1'b0;
endmodule

module VideoTimingGen (
      input  [11:0] io_timings_h_active,
      input  [8:0] io_timings_h_fp,
      input  [8:0] io_timings_h_sync,
      input  [8:0] io_timings_h_bp,
      input   io_timings_h_sync_positive,
      input  [10:0] io_timings_v_active,
      input  [8:0] io_timings_v_fp,
      input  [8:0] io_timings_v_sync,
      input  [8:0] io_timings_v_bp,
      input   io_timings_v_sync_positive,
      output reg  io_pixel_out_vsync,
      output reg  io_pixel_out_req,
      output reg  io_pixel_out_last_col,
      output reg  io_pixel_out_last_line,
      output reg [7:0] io_pixel_out_pixel_r,
      output reg [7:0] io_pixel_out_pixel_g,
      output reg [7:0] io_pixel_out_pixel_b,
      input   toplevel_u_vo_clk_gen_vo_clk,
      input   toplevel_u_vo_clk_gen_vo_reset_);
  wire [11:0] _zz_VideoTimingGen_1_;
  wire [11:0] _zz_VideoTimingGen_2_;
  wire [11:0] _zz_VideoTimingGen_3_;
  wire [11:0] _zz_VideoTimingGen_4_;
  wire [11:0] _zz_VideoTimingGen_5_;
  wire [11:0] _zz_VideoTimingGen_6_;
  wire [11:0] _zz_VideoTimingGen_7_;
  wire [10:0] _zz_VideoTimingGen_8_;
  wire [10:0] _zz_VideoTimingGen_9_;
  wire [10:0] _zz_VideoTimingGen_10_;
  wire [10:0] _zz_VideoTimingGen_11_;
  wire [10:0] _zz_VideoTimingGen_12_;
  wire [10:0] _zz_VideoTimingGen_13_;
  wire [10:0] _zz_VideoTimingGen_14_;
  wire [8:0] _zz_VideoTimingGen_15_;
  wire [8:0] _zz_VideoTimingGen_16_;
  wire [11:0] _zz_VideoTimingGen_17_;
  wire [10:0] _zz_VideoTimingGen_18_;
  reg [11:0] col_cntr;
  reg [10:0] line_cntr;
  wire  last_col;
  wire  last_line;
  wire [8:0] h_blank;
  wire [8:0] v_blank;
  wire  pixel_active;
  assign _zz_VideoTimingGen_1_ = (_zz_VideoTimingGen_2_ - (12'b000000000001));
  assign _zz_VideoTimingGen_2_ = (_zz_VideoTimingGen_3_ + _zz_VideoTimingGen_7_);
  assign _zz_VideoTimingGen_3_ = (_zz_VideoTimingGen_4_ + _zz_VideoTimingGen_6_);
  assign _zz_VideoTimingGen_4_ = (io_timings_h_active + _zz_VideoTimingGen_5_);
  assign _zz_VideoTimingGen_5_ = {3'd0, io_timings_h_fp};
  assign _zz_VideoTimingGen_6_ = {3'd0, io_timings_h_sync};
  assign _zz_VideoTimingGen_7_ = {3'd0, io_timings_h_bp};
  assign _zz_VideoTimingGen_8_ = (_zz_VideoTimingGen_9_ - (11'b00000000001));
  assign _zz_VideoTimingGen_9_ = (_zz_VideoTimingGen_10_ + _zz_VideoTimingGen_14_);
  assign _zz_VideoTimingGen_10_ = (_zz_VideoTimingGen_11_ + _zz_VideoTimingGen_13_);
  assign _zz_VideoTimingGen_11_ = (io_timings_v_active + _zz_VideoTimingGen_12_);
  assign _zz_VideoTimingGen_12_ = {2'd0, io_timings_v_fp};
  assign _zz_VideoTimingGen_13_ = {2'd0, io_timings_v_sync};
  assign _zz_VideoTimingGen_14_ = {2'd0, io_timings_v_bp};
  assign _zz_VideoTimingGen_15_ = (io_timings_h_fp + io_timings_h_sync);
  assign _zz_VideoTimingGen_16_ = (io_timings_v_fp + io_timings_v_sync);
  assign _zz_VideoTimingGen_17_ = {3'd0, h_blank};
  assign _zz_VideoTimingGen_18_ = {2'd0, v_blank};
  assign last_col = (col_cntr == _zz_VideoTimingGen_1_);
  assign last_line = (line_cntr == _zz_VideoTimingGen_8_);
  assign h_blank = (_zz_VideoTimingGen_15_ + io_timings_h_bp);
  assign v_blank = (_zz_VideoTimingGen_16_ + io_timings_v_bp);
  assign pixel_active = ((_zz_VideoTimingGen_17_ <= col_cntr) && (_zz_VideoTimingGen_18_ <= line_cntr));
  always @ (posedge toplevel_u_vo_clk_gen_vo_clk) begin
    if(!toplevel_u_vo_clk_gen_vo_reset_) begin
      col_cntr <= (12'b000000000000);
      line_cntr <= (11'b00000000000);
    end else begin
      if((! last_col))begin
        col_cntr <= (col_cntr + (12'b000000000001));
      end else begin
        col_cntr <= (12'b000000000000);
        if((! last_line))begin
          line_cntr <= (line_cntr + (11'b00000000001));
        end else begin
          line_cntr <= (11'b00000000000);
        end
      end
    end
  end

  always @ (posedge toplevel_u_vo_clk_gen_vo_clk) begin
    io_pixel_out_vsync <= ((col_cntr == (12'b000000000000)) && (line_cntr == (11'b00000000000)));
    io_pixel_out_req <= pixel_active;
    io_pixel_out_last_col <= (pixel_active ? last_col : 1'b0);
    io_pixel_out_last_line <= (pixel_active ? last_line : 1'b0);
    io_pixel_out_pixel_r <= (8'b10000000);
    io_pixel_out_pixel_g <= (8'b10000000);
    io_pixel_out_pixel_b <= (8'b10000000);
  end

endmodule

module VideoTestPattern (
      input  [11:0] io_timings_h_active,
      input  [8:0] io_timings_h_fp,
      input  [8:0] io_timings_h_sync,
      input  [8:0] io_timings_h_bp,
      input   io_timings_h_sync_positive,
      input  [10:0] io_timings_v_active,
      input  [8:0] io_timings_v_fp,
      input  [8:0] io_timings_v_sync,
      input  [8:0] io_timings_v_bp,
      input   io_timings_v_sync_positive,
      input   io_pixel_in_vsync,
      input   io_pixel_in_req,
      input   io_pixel_in_last_col,
      input   io_pixel_in_last_line,
      input  [7:0] io_pixel_in_pixel_r,
      input  [7:0] io_pixel_in_pixel_g,
      input  [7:0] io_pixel_in_pixel_b,
      output reg  io_pixel_out_vsync,
      output reg  io_pixel_out_req,
      output reg  io_pixel_out_last_col,
      output reg  io_pixel_out_last_line,
      output reg [7:0] io_pixel_out_pixel_r,
      output reg [7:0] io_pixel_out_pixel_g,
      output reg [7:0] io_pixel_out_pixel_b,
      input  [3:0] io_pattern_nr,
      input  [7:0] io_const_color_r,
      input  [7:0] io_const_color_g,
      input  [7:0] io_const_color_b,
      input   toplevel_u_vo_clk_gen_vo_clk,
      input   toplevel_u_vo_clk_gen_vo_reset_);
  wire [3:0] bufferCC_19__io_dataOut;
  wire [7:0] bufferCC_20__io_dataOut_r;
  wire [7:0] bufferCC_20__io_dataOut_g;
  wire [7:0] bufferCC_20__io_dataOut_b;
  wire [13:0] _zz_VideoTestPattern_1_;
  wire [13:0] _zz_VideoTestPattern_2_;
  wire [13:0] _zz_VideoTestPattern_3_;
  wire [13:0] _zz_VideoTestPattern_4_;
  wire [14:0] _zz_VideoTestPattern_5_;
  wire [14:0] _zz_VideoTestPattern_6_;
  wire [12:0] _zz_VideoTestPattern_7_;
  wire [12:0] _zz_VideoTestPattern_8_;
  wire [12:0] _zz_VideoTestPattern_9_;
  wire [12:0] _zz_VideoTestPattern_10_;
  wire [13:0] _zz_VideoTestPattern_11_;
  wire [13:0] _zz_VideoTestPattern_12_;
  wire [7:0] _zz_VideoTestPattern_13_;
  wire [7:0] _zz_VideoTestPattern_14_;
  wire [7:0] _zz_VideoTestPattern_15_;
  wire [10:0] _zz_VideoTestPattern_16_;
  wire [11:0] _zz_VideoTestPattern_17_;
  reg [11:0] col_cntr;
  reg [10:0] line_cntr;
  wire [11:0] h_active_div4;
  wire [10:0] v_active_div4;
  wire  h1;
  wire  h2;
  wire  h3;
  wire  h4;
  wire  v1;
  wire  v2;
  wire  v3;
  wire  v4;
  assign _zz_VideoTestPattern_1_ = {2'd0, col_cntr};
  assign _zz_VideoTestPattern_2_ = (h_active_div4 * (2'b10));
  assign _zz_VideoTestPattern_3_ = {2'd0, col_cntr};
  assign _zz_VideoTestPattern_4_ = (h_active_div4 * (2'b11));
  assign _zz_VideoTestPattern_5_ = {3'd0, col_cntr};
  assign _zz_VideoTestPattern_6_ = (h_active_div4 * (3'b100));
  assign _zz_VideoTestPattern_7_ = {2'd0, line_cntr};
  assign _zz_VideoTestPattern_8_ = (v_active_div4 * (2'b10));
  assign _zz_VideoTestPattern_9_ = {2'd0, line_cntr};
  assign _zz_VideoTestPattern_10_ = (v_active_div4 * (2'b11));
  assign _zz_VideoTestPattern_11_ = {3'd0, line_cntr};
  assign _zz_VideoTestPattern_12_ = (v_active_div4 * (3'b100));
  assign _zz_VideoTestPattern_13_ = (col_cntr[7 : 0] + line_cntr[7 : 0]);
  assign _zz_VideoTestPattern_14_ = (col_cntr[7 : 0] + line_cntr[7 : 0]);
  assign _zz_VideoTestPattern_15_ = (col_cntr[7 : 0] + line_cntr[7 : 0]);
  assign _zz_VideoTestPattern_16_ = (line_cntr <<< 3);
  assign _zz_VideoTestPattern_17_ = (col_cntr <<< 3);
  BufferCC_9_ bufferCC_19_ ( 
    .io_dataIn(io_pattern_nr),
    .io_dataOut(bufferCC_19__io_dataOut),
    .toplevel_u_vo_clk_gen_vo_clk(toplevel_u_vo_clk_gen_vo_clk),
    .toplevel_u_vo_clk_gen_vo_reset_(toplevel_u_vo_clk_gen_vo_reset_) 
  );
  BufferCC_10_ bufferCC_20_ ( 
    .io_dataIn_r(io_const_color_r),
    .io_dataIn_g(io_const_color_g),
    .io_dataIn_b(io_const_color_b),
    .io_dataOut_r(bufferCC_20__io_dataOut_r),
    .io_dataOut_g(bufferCC_20__io_dataOut_g),
    .io_dataOut_b(bufferCC_20__io_dataOut_b),
    .toplevel_u_vo_clk_gen_vo_clk(toplevel_u_vo_clk_gen_vo_clk),
    .toplevel_u_vo_clk_gen_vo_reset_(toplevel_u_vo_clk_gen_vo_reset_) 
  );
  assign h_active_div4 = (io_timings_h_active >>> 2);
  assign v_active_div4 = (io_timings_v_active >>> 2);
  assign h1 = (col_cntr < h_active_div4);
  assign h2 = (_zz_VideoTestPattern_1_ < _zz_VideoTestPattern_2_);
  assign h3 = (_zz_VideoTestPattern_3_ < _zz_VideoTestPattern_4_);
  assign h4 = (_zz_VideoTestPattern_5_ < _zz_VideoTestPattern_6_);
  assign v1 = (line_cntr < v_active_div4);
  assign v2 = (_zz_VideoTestPattern_7_ < _zz_VideoTestPattern_8_);
  assign v3 = (_zz_VideoTestPattern_9_ < _zz_VideoTestPattern_10_);
  assign v4 = (_zz_VideoTestPattern_11_ < _zz_VideoTestPattern_12_);
  always @ (posedge toplevel_u_vo_clk_gen_vo_clk) begin
    if(!toplevel_u_vo_clk_gen_vo_reset_) begin
      col_cntr <= (12'b000000000000);
      line_cntr <= (11'b00000000000);
    end else begin
      if(io_pixel_in_vsync)begin
        line_cntr <= (11'b00000000000);
        col_cntr <= (12'b000000000000);
      end else begin
        if(io_pixel_in_req)begin
          if((io_pixel_in_last_col && io_pixel_in_last_line))begin
            line_cntr <= (11'b00000000000);
            col_cntr <= (12'b000000000000);
          end else begin
            if(io_pixel_in_last_col)begin
              line_cntr <= (line_cntr + (11'b00000000001));
              col_cntr <= (12'b000000000000);
            end else begin
              col_cntr <= (col_cntr + (12'b000000000001));
            end
          end
        end
      end
    end
  end

  always @ (posedge toplevel_u_vo_clk_gen_vo_clk) begin
    io_pixel_out_vsync <= io_pixel_in_vsync;
    io_pixel_out_req <= io_pixel_in_req;
    io_pixel_out_last_col <= io_pixel_in_last_col;
    io_pixel_out_last_line <= io_pixel_in_last_line;
    io_pixel_out_pixel_r <= io_pixel_in_pixel_r;
    io_pixel_out_pixel_g <= io_pixel_in_pixel_g;
    io_pixel_out_pixel_b <= io_pixel_in_pixel_b;
    case(bufferCC_19__io_dataOut)
      4'b0000 : begin
        io_pixel_out_pixel_r <= bufferCC_20__io_dataOut_r;
        io_pixel_out_pixel_g <= bufferCC_20__io_dataOut_g;
        io_pixel_out_pixel_b <= bufferCC_20__io_dataOut_b;
      end
      4'b0001 : begin
        io_pixel_out_pixel_r <= _zz_VideoTestPattern_13_;
        io_pixel_out_pixel_g <= (8'b00000000);
        io_pixel_out_pixel_b <= (8'b00000000);
      end
      4'b0010 : begin
        io_pixel_out_pixel_r <= (8'b00000000);
        io_pixel_out_pixel_g <= _zz_VideoTestPattern_14_;
        io_pixel_out_pixel_b <= (8'b00000000);
      end
      4'b0011 : begin
        io_pixel_out_pixel_r <= (8'b00000000);
        io_pixel_out_pixel_g <= (8'b00000000);
        io_pixel_out_pixel_b <= _zz_VideoTestPattern_15_;
      end
      4'b0100 : begin
        if(h1)begin
          io_pixel_out_pixel_r <= (8'b11111111);
          io_pixel_out_pixel_g <= (8'b00000000);
          io_pixel_out_pixel_b <= (8'b00000000);
        end else begin
          if(h2)begin
            io_pixel_out_pixel_r <= (8'b00000000);
            io_pixel_out_pixel_g <= (8'b11111111);
            io_pixel_out_pixel_b <= (8'b00000000);
          end else begin
            if(h3)begin
              io_pixel_out_pixel_r <= (8'b00000000);
              io_pixel_out_pixel_g <= (8'b00000000);
              io_pixel_out_pixel_b <= (8'b11111111);
            end else begin
              io_pixel_out_pixel_r <= (8'b11111111);
              io_pixel_out_pixel_g <= (8'b11111111);
              io_pixel_out_pixel_b <= (8'b11111111);
            end
          end
        end
      end
      4'b0101 : begin
        if(v1)begin
          io_pixel_out_pixel_r <= (8'b11111111);
          io_pixel_out_pixel_g <= (8'b00000000);
          io_pixel_out_pixel_b <= (8'b00000000);
        end else begin
          if(v2)begin
            io_pixel_out_pixel_r <= (8'b00000000);
            io_pixel_out_pixel_g <= (8'b11111111);
            io_pixel_out_pixel_b <= (8'b00000000);
          end else begin
            if(v3)begin
              io_pixel_out_pixel_r <= (8'b00000000);
              io_pixel_out_pixel_g <= (8'b00000000);
              io_pixel_out_pixel_b <= (8'b11111111);
            end else begin
              io_pixel_out_pixel_r <= (8'b11111111);
              io_pixel_out_pixel_g <= (8'b11111111);
              io_pixel_out_pixel_b <= (8'b11111111);
            end
          end
        end
      end
      4'b0110 : begin
        io_pixel_out_pixel_r <= {line_cntr[3 : 0],col_cntr[3 : 0]};
        io_pixel_out_pixel_g <= _zz_VideoTestPattern_16_[7 : 0];
        io_pixel_out_pixel_b <= _zz_VideoTestPattern_17_[7 : 0];
      end
      default : begin
      end
    endcase
  end

endmodule

module VideoTxtGen (
      input   io_pixel_in_vsync,
      input   io_pixel_in_req,
      input   io_pixel_in_last_col,
      input   io_pixel_in_last_line,
      input  [7:0] io_pixel_in_pixel_r,
      input  [7:0] io_pixel_in_pixel_g,
      input  [7:0] io_pixel_in_pixel_b,
      output  io_pixel_out_vsync,
      output  io_pixel_out_req,
      output  io_pixel_out_last_col,
      output  io_pixel_out_last_line,
      output reg [7:0] io_pixel_out_pixel_r,
      output reg [7:0] io_pixel_out_pixel_g,
      output reg [7:0] io_pixel_out_pixel_b,
      input   io_txt_buf_wr,
      input   io_txt_buf_rd,
      input  [12:0] io_txt_buf_addr,
      input  [7:0] io_txt_buf_wr_data,
      output [7:0] io_txt_buf_rd_data,
      input   toplevel_u_vo_clk_gen_vo_clk,
      input   toplevel_u_vo_clk_gen_vo_reset_,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg [7:0] _zz_VideoTxtGen_5_;
  reg [7:0] _zz_VideoTxtGen_6_;
  reg [7:0] _zz_VideoTxtGen_7_;
  wire [12:0] _zz_VideoTxtGen_8_;
  wire [7:0] _zz_VideoTxtGen_9_;
  wire [11:0] _zz_VideoTxtGen_10_;
  wire [8:0] _zz_VideoTxtGen_11_;
  wire [11:0] _zz_VideoTxtGen_12_;
  wire [12:0] _zz_VideoTxtGen_13_;
  wire [3:0] _zz_VideoTxtGen_14_;
  wire [7:0] _zz_VideoTxtGen_15_;
  reg [11:0] pix_x;
  reg [10:0] pix_y;
  reg [7:0] char_x;
  reg [6:0] char_y;
  reg [3:0] char_sub_x;
  reg [3:0] char_sub_y;
  reg [12:0] txt_buf_addr_sol;
  wire [12:0] txt_buf_addr;
  wire  txt_buf_rd_p0;
  wire [12:0] _zz_VideoTxtGen_1_;
  wire [7:0] cur_char;
  wire  _zz_VideoTxtGen_2_;
  wire [12:0] _zz_VideoTxtGen_3_;
  wire [7:0] _zz_VideoTxtGen_4_;
  reg  txt_buf_rd_p1;
  reg [3:0] char_sub_x_p1;
  wire [11:0] bitmap_lsb_addr;
  wire [11:0] bitmap_msb_addr;
  wire [11:0] bitmap_addr;
  wire [7:0] bitmap_byte;
  reg  txt_buf_rd_p2;
  reg [3:0] char_sub_x_p2;
  wire  bitmap_pixel;
  reg  io_pixel_in_regNext_vsync;
  reg  io_pixel_in_regNext_req;
  reg  io_pixel_in_regNext_last_col;
  reg  io_pixel_in_regNext_last_line;
  reg [7:0] io_pixel_in_regNext_pixel_r;
  reg [7:0] io_pixel_in_regNext_pixel_g;
  reg [7:0] io_pixel_in_regNext_pixel_b;
  reg  pixel_in_p2_vsync;
  reg  pixel_in_p2_req;
  reg  pixel_in_p2_last_col;
  reg  pixel_in_p2_last_line;
  reg [7:0] pixel_in_p2_pixel_r;
  reg [7:0] pixel_in_p2_pixel_g;
  reg [7:0] pixel_in_p2_pixel_b;
  reg [7:0] u_txt_buf [0:8191];
  reg [7:0] u_font_bitmap_ram [0:4095];
  assign _zz_VideoTxtGen_8_ = {5'd0, char_x};
  assign _zz_VideoTxtGen_9_ = (cur_char & (8'b00001111));
  assign _zz_VideoTxtGen_10_ = {4'd0, _zz_VideoTxtGen_9_};
  assign _zz_VideoTxtGen_11_ = (char_sub_y[3 : 0] * (5'b10000));
  assign _zz_VideoTxtGen_12_ = {3'd0, _zz_VideoTxtGen_11_};
  assign _zz_VideoTxtGen_13_ = (_zz_VideoTxtGen_14_ * (9'b100000000));
  assign _zz_VideoTxtGen_14_ = (cur_char >>> 4);
  assign _zz_VideoTxtGen_15_ = (bitmap_byte >>> ((3'b111) ^ char_sub_x_p2[2 : 0]));
  always @ (posedge toplevel_u_vo_clk_gen_vo_clk) begin
  end

  always @ (posedge toplevel_main_clk) begin
    if(_zz_VideoTxtGen_2_ && io_txt_buf_wr ) begin
      u_txt_buf[_zz_VideoTxtGen_3_] <= _zz_VideoTxtGen_4_;
    end
    if(_zz_VideoTxtGen_2_) begin
      _zz_VideoTxtGen_6_ <= u_txt_buf[_zz_VideoTxtGen_3_];
    end
  end

  always @ (posedge toplevel_u_vo_clk_gen_vo_clk) begin
    if(txt_buf_rd_p0) begin
      _zz_VideoTxtGen_5_ <= u_txt_buf[_zz_VideoTxtGen_1_];
    end
  end

  initial begin
    $readmemb("Pano.v_toplevel_core_u_pano_core_vo_area_u_txt_gen_u_font_bitmap_ram.bin",u_font_bitmap_ram);
  end
  always @ (posedge toplevel_u_vo_clk_gen_vo_clk) begin
  end

  always @ (posedge toplevel_u_vo_clk_gen_vo_clk) begin
    if(txt_buf_rd_p1) begin
      _zz_VideoTxtGen_7_ <= u_font_bitmap_ram[bitmap_addr];
    end
  end

  assign txt_buf_addr = (txt_buf_addr_sol + _zz_VideoTxtGen_8_);
  assign txt_buf_rd_p0 = (((char_x < (8'b10000010)) && (char_y < (7'b0111100))) && io_pixel_in_req);
  assign _zz_VideoTxtGen_1_ = txt_buf_addr;
  assign cur_char = _zz_VideoTxtGen_5_;
  assign _zz_VideoTxtGen_2_ = (io_txt_buf_wr || io_txt_buf_rd);
  assign _zz_VideoTxtGen_3_ = io_txt_buf_addr;
  assign _zz_VideoTxtGen_4_ = io_txt_buf_wr_data;
  assign io_txt_buf_rd_data = _zz_VideoTxtGen_6_;
  assign bitmap_lsb_addr = (_zz_VideoTxtGen_10_ + _zz_VideoTxtGen_12_);
  assign bitmap_msb_addr = _zz_VideoTxtGen_13_[11:0];
  assign bitmap_addr = (bitmap_msb_addr + bitmap_lsb_addr);
  assign bitmap_byte = _zz_VideoTxtGen_7_;
  assign bitmap_pixel = (_zz_VideoTxtGen_15_[0] && (! char_sub_x_p2[3]));
  assign io_pixel_out_vsync = pixel_in_p2_vsync;
  assign io_pixel_out_req = pixel_in_p2_req;
  assign io_pixel_out_last_col = pixel_in_p2_last_col;
  assign io_pixel_out_last_line = pixel_in_p2_last_line;
  always @ (*) begin
    io_pixel_out_pixel_r = pixel_in_p2_pixel_r;
    io_pixel_out_pixel_g = pixel_in_p2_pixel_g;
    io_pixel_out_pixel_b = pixel_in_p2_pixel_b;
    if((bitmap_pixel && txt_buf_rd_p2))begin
      io_pixel_out_pixel_r = (8'b11111111);
      io_pixel_out_pixel_g = (8'b11111111);
      io_pixel_out_pixel_b = (8'b11111111);
    end
  end

  always @ (posedge toplevel_u_vo_clk_gen_vo_clk) begin
    if(!toplevel_u_vo_clk_gen_vo_reset_) begin
      pix_x <= (12'b000000000000);
      pix_y <= (11'b00000000000);
      char_x <= (8'b00000000);
      char_y <= (7'b0000000);
      char_sub_x <= (4'b0000);
      char_sub_y <= (4'b0000);
      txt_buf_addr_sol <= (13'b0000000000000);
    end else begin
      if((io_pixel_in_vsync || (io_pixel_in_req && (io_pixel_in_last_col && io_pixel_in_last_line))))begin
        pix_x <= (12'b000000000000);
        pix_y <= (11'b00000000000);
        char_x <= (8'b00000000);
        char_y <= (7'b0000000);
        char_sub_x <= (4'b0000);
        char_sub_y <= (4'b0000);
        txt_buf_addr_sol <= (13'b0000000000000);
      end else begin
        if(io_pixel_in_req)begin
          if(io_pixel_in_last_col)begin
            pix_x <= (12'b000000000000);
            pix_y <= (pix_y + (11'b00000000001));
            char_x <= (8'b00000000);
            char_sub_x <= (4'b0000);
            if((char_sub_y == (4'b1111)))begin
              char_y <= (char_y + (7'b0000001));
              char_sub_y <= (4'b0000);
              txt_buf_addr_sol <= (txt_buf_addr_sol + (13'b0000010000010));
            end else begin
              char_sub_y <= (char_sub_y + (4'b0001));
            end
          end else begin
            pix_x <= (pix_x + (12'b000000000001));
            if((char_sub_x == (4'b1000)))begin
              char_x <= (char_x + (8'b00000001));
              char_sub_x <= (4'b0000);
            end else begin
              char_sub_x <= (char_sub_x + (4'b0001));
            end
          end
        end
      end
    end
  end

  always @ (posedge toplevel_u_vo_clk_gen_vo_clk) begin
    txt_buf_rd_p1 <= txt_buf_rd_p0;
    char_sub_x_p1 <= char_sub_x;
    txt_buf_rd_p2 <= txt_buf_rd_p1;
    char_sub_x_p2 <= char_sub_x_p1;
    io_pixel_in_regNext_vsync <= io_pixel_in_vsync;
    io_pixel_in_regNext_req <= io_pixel_in_req;
    io_pixel_in_regNext_last_col <= io_pixel_in_last_col;
    io_pixel_in_regNext_last_line <= io_pixel_in_last_line;
    io_pixel_in_regNext_pixel_r <= io_pixel_in_pixel_r;
    io_pixel_in_regNext_pixel_g <= io_pixel_in_pixel_g;
    io_pixel_in_regNext_pixel_b <= io_pixel_in_pixel_b;
    pixel_in_p2_vsync <= io_pixel_in_regNext_vsync;
    pixel_in_p2_req <= io_pixel_in_regNext_req;
    pixel_in_p2_last_col <= io_pixel_in_regNext_last_col;
    pixel_in_p2_last_line <= io_pixel_in_regNext_last_line;
    pixel_in_p2_pixel_r <= io_pixel_in_regNext_pixel_r;
    pixel_in_p2_pixel_g <= io_pixel_in_regNext_pixel_g;
    pixel_in_p2_pixel_b <= io_pixel_in_regNext_pixel_b;
  end

endmodule

module VideoOut (
      input  [11:0] io_timings_h_active,
      input  [8:0] io_timings_h_fp,
      input  [8:0] io_timings_h_sync,
      input  [8:0] io_timings_h_bp,
      input   io_timings_h_sync_positive,
      input  [10:0] io_timings_v_active,
      input  [8:0] io_timings_v_fp,
      input  [8:0] io_timings_v_sync,
      input  [8:0] io_timings_v_bp,
      input   io_timings_v_sync_positive,
      input   io_pixel_in_vsync,
      input   io_pixel_in_req,
      input   io_pixel_in_last_col,
      input   io_pixel_in_last_line,
      input  [7:0] io_pixel_in_pixel_r,
      input  [7:0] io_pixel_in_pixel_g,
      input  [7:0] io_pixel_in_pixel_b,
      output reg  io_vga_out_vsync,
      output reg  io_vga_out_hsync,
      output reg  io_vga_out_blank_,
      output reg  io_vga_out_de,
      output reg [7:0] io_vga_out_r,
      output reg [7:0] io_vga_out_g,
      output reg [7:0] io_vga_out_b,
      input   toplevel_u_vo_clk_gen_vo_clk,
      input   toplevel_u_vo_clk_gen_vo_reset_);
  wire [11:0] _zz_VideoOut_1_;
  wire [11:0] _zz_VideoOut_2_;
  wire [11:0] _zz_VideoOut_3_;
  wire [11:0] _zz_VideoOut_4_;
  wire [11:0] _zz_VideoOut_5_;
  wire [11:0] _zz_VideoOut_6_;
  wire [11:0] _zz_VideoOut_7_;
  wire [8:0] _zz_VideoOut_8_;
  wire [8:0] _zz_VideoOut_9_;
  wire [10:0] _zz_VideoOut_10_;
  wire [11:0] _zz_VideoOut_11_;
  wire [11:0] _zz_VideoOut_12_;
  wire [8:0] _zz_VideoOut_13_;
  wire [11:0] _zz_VideoOut_14_;
  wire [10:0] _zz_VideoOut_15_;
  wire [8:0] _zz_VideoOut_16_;
  wire [10:0] _zz_VideoOut_17_;
  reg [11:0] h_cntr;
  reg [10:0] v_cntr;
  wire [8:0] h_blank;
  wire [8:0] v_blank;
  wire  blank;
  assign _zz_VideoOut_1_ = (_zz_VideoOut_2_ - (12'b000000000001));
  assign _zz_VideoOut_2_ = (_zz_VideoOut_3_ + _zz_VideoOut_7_);
  assign _zz_VideoOut_3_ = (_zz_VideoOut_4_ + _zz_VideoOut_6_);
  assign _zz_VideoOut_4_ = (io_timings_h_active + _zz_VideoOut_5_);
  assign _zz_VideoOut_5_ = {3'd0, io_timings_h_fp};
  assign _zz_VideoOut_6_ = {3'd0, io_timings_h_sync};
  assign _zz_VideoOut_7_ = {3'd0, io_timings_h_bp};
  assign _zz_VideoOut_8_ = (io_timings_h_fp + io_timings_h_sync);
  assign _zz_VideoOut_9_ = (io_timings_v_fp + io_timings_v_sync);
  assign _zz_VideoOut_10_ = {2'd0, v_blank};
  assign _zz_VideoOut_11_ = {3'd0, h_blank};
  assign _zz_VideoOut_12_ = {3'd0, io_timings_h_fp};
  assign _zz_VideoOut_13_ = (io_timings_h_fp + io_timings_h_sync);
  assign _zz_VideoOut_14_ = {3'd0, _zz_VideoOut_13_};
  assign _zz_VideoOut_15_ = {2'd0, io_timings_v_fp};
  assign _zz_VideoOut_16_ = (io_timings_v_fp + io_timings_v_sync);
  assign _zz_VideoOut_17_ = {2'd0, _zz_VideoOut_16_};
  assign h_blank = (_zz_VideoOut_8_ + io_timings_h_bp);
  assign v_blank = (_zz_VideoOut_9_ + io_timings_v_bp);
  assign blank = ((v_cntr < _zz_VideoOut_10_) || (h_cntr < _zz_VideoOut_11_));
  always @ (posedge toplevel_u_vo_clk_gen_vo_clk) begin
    if(!toplevel_u_vo_clk_gen_vo_reset_) begin
      io_vga_out_vsync <= 1'b0;
      io_vga_out_hsync <= 1'b0;
      io_vga_out_blank_ <= 1'b0;
      io_vga_out_de <= 1'b0;
      io_vga_out_r <= (8'b00000000);
      io_vga_out_g <= (8'b00000000);
      io_vga_out_b <= (8'b00000000);
      h_cntr <= (12'b000000000000);
      v_cntr <= (11'b00000000000);
    end else begin
      if((io_pixel_in_req && (io_pixel_in_last_col && io_pixel_in_last_line)))begin
        h_cntr <= (12'b000000000000);
        v_cntr <= (11'b00000000000);
      end else begin
        if((h_cntr == _zz_VideoOut_1_))begin
          h_cntr <= (12'b000000000000);
          v_cntr <= (v_cntr + (11'b00000000001));
        end else begin
          h_cntr <= (h_cntr + (12'b000000000001));
        end
      end
      io_vga_out_blank_ <= (! blank);
      io_vga_out_de <= (! blank);
      io_vga_out_hsync <= (((_zz_VideoOut_12_ <= h_cntr) && (h_cntr < _zz_VideoOut_14_)) ^ (! io_timings_h_sync_positive));
      io_vga_out_vsync <= (((_zz_VideoOut_15_ <= v_cntr) && (v_cntr < _zz_VideoOut_17_)) ^ (! io_timings_v_sync_positive));
      io_vga_out_r <= (blank ? (8'b00000000) : io_pixel_in_pixel_r);
      io_vga_out_g <= (blank ? (8'b00000000) : io_pixel_in_pixel_g);
      io_vga_out_b <= (blank ? (8'b00000000) : io_pixel_in_pixel_b);
    end
  end

endmodule

module GmiiCtrl (
      input  [4:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output reg [31:0] io_apb_PRDATA,
      output  io_apb_PSLVERROR,
      input   io_gmii_rx_clk,
      input   io_gmii_rx_dv,
      input   io_gmii_rx_er,
      input  [7:0] io_gmii_rx_d,
      input   io_gmii_tx_gclk,
      input   io_gmii_tx_clk,
      output  io_gmii_tx_en,
      output  io_gmii_tx_er,
      output [7:0] io_gmii_tx_d,
      input   io_gmii_col,
      input   io_gmii_crs,
      output  io_gmii_mdio_mdc,
      input   io_gmii_mdio_mdio_read,
      output  io_gmii_mdio_mdio_write,
      output  io_gmii_mdio_mdio_writeEnable,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire  _zz_GmiiCtrl_5_;
  wire  u_gmii_rx_io_rx_fifo_rd_valid;
  wire [9:0] u_gmii_rx_io_rx_fifo_rd_payload;
  wire [15:0] u_gmii_rx_io_rx_fifo_rd_count;
  wire  u_gmii_tx_io_tx_en;
  wire  u_gmii_tx_io_tx_er;
  wire [7:0] u_gmii_tx_io_tx_d;
  wire [0:0] _zz_GmiiCtrl_6_;
  wire [0:0] _zz_GmiiCtrl_7_;
  wire [0:0] _zz_GmiiCtrl_8_;
  wire  ctrl_askWrite;
  wire  ctrl_askRead;
  wire  ctrl_doWrite;
  wire  ctrl_doRead;
  reg  _zz_GmiiCtrl_1_;
  reg  _zz_GmiiCtrl_2_;
  reg  _zz_GmiiCtrl_3_;
  wire  cpu_rx_fifo_rd_valid;
  wire  cpu_rx_fifo_rd_ready;
  wire [9:0] cpu_rx_fifo_rd_payload;
  reg  _zz_GmiiCtrl_4_;
  assign _zz_GmiiCtrl_6_ = io_apb_PWDATA[0 : 0];
  assign _zz_GmiiCtrl_7_ = io_apb_PWDATA[1 : 1];
  assign _zz_GmiiCtrl_8_ = io_apb_PWDATA[2 : 2];
  GmiiRxCtrl u_gmii_rx ( 
    .io_rx_clk(io_gmii_rx_clk),
    .io_rx_dv(io_gmii_rx_dv),
    .io_rx_er(io_gmii_rx_er),
    .io_rx_d(io_gmii_rx_d),
    .io_rx_fifo_rd_valid(u_gmii_rx_io_rx_fifo_rd_valid),
    .io_rx_fifo_rd_ready(_zz_GmiiCtrl_5_),
    .io_rx_fifo_rd_payload(u_gmii_rx_io_rx_fifo_rd_payload),
    .io_rx_fifo_rd_count(u_gmii_rx_io_rx_fifo_rd_count),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  GmiiTxCtrl u_gmii_tx ( 
    .io_tx_gclk(io_gmii_tx_gclk),
    .io_tx_clk(io_gmii_tx_clk),
    .io_tx_en(u_gmii_tx_io_tx_en),
    .io_tx_er(u_gmii_tx_io_tx_er),
    .io_tx_d(u_gmii_tx_io_tx_d) 
  );
  assign io_gmii_tx_en = u_gmii_tx_io_tx_en;
  assign io_gmii_tx_er = u_gmii_tx_io_tx_er;
  assign io_gmii_tx_d = u_gmii_tx_io_tx_d;
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = (32'b00000000000000000000000000000000);
    _zz_GmiiCtrl_4_ = 1'b0;
    case(io_apb_PADDR)
      5'b00000 : begin
        io_apb_PRDATA[0 : 0] = _zz_GmiiCtrl_1_;
        io_apb_PRDATA[1 : 1] = _zz_GmiiCtrl_2_;
        io_apb_PRDATA[2 : 2] = _zz_GmiiCtrl_3_;
        io_apb_PRDATA[3 : 3] = io_gmii_mdio_mdio_read;
      end
      5'b00100 : begin
        if(ctrl_doRead)begin
          _zz_GmiiCtrl_4_ = 1'b1;
        end
        io_apb_PRDATA[16 : 16] = u_gmii_rx_io_rx_fifo_rd_valid;
        io_apb_PRDATA[9 : 0] = u_gmii_rx_io_rx_fifo_rd_payload;
      end
      5'b01000 : begin
        io_apb_PRDATA[15 : 0] = u_gmii_rx_io_rx_fifo_rd_count;
      end
      default : begin
      end
    endcase
  end

  assign io_apb_PSLVERROR = 1'b0;
  assign ctrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign ctrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign ctrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign ctrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  assign io_gmii_mdio_mdc = _zz_GmiiCtrl_1_;
  assign io_gmii_mdio_mdio_write = _zz_GmiiCtrl_2_;
  assign io_gmii_mdio_mdio_writeEnable = _zz_GmiiCtrl_3_;
  assign _zz_GmiiCtrl_5_ = (_zz_GmiiCtrl_4_ && u_gmii_rx_io_rx_fifo_rd_valid);
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      _zz_GmiiCtrl_1_ <= 1'b0;
      _zz_GmiiCtrl_2_ <= 1'b0;
      _zz_GmiiCtrl_3_ <= 1'b0;
    end else begin
      case(io_apb_PADDR)
        5'b00000 : begin
          if(ctrl_doWrite)begin
            _zz_GmiiCtrl_1_ <= _zz_GmiiCtrl_6_[0];
            _zz_GmiiCtrl_2_ <= _zz_GmiiCtrl_7_[0];
            _zz_GmiiCtrl_3_ <= _zz_GmiiCtrl_8_[0];
          end
        end
        5'b00100 : begin
        end
        5'b01000 : begin
        end
        default : begin
        end
      endcase
    end
  end

endmodule

module UlpiCtrl (
      input   io_ulpi_clk,
      input  [7:0] io_ulpi_data_read,
      output [7:0] io_ulpi_data_write,
      output [7:0] io_ulpi_data_writeEnable,
      input   io_ulpi_direction,
      output  io_ulpi_stp,
      input   io_ulpi_nxt,
      output  io_ulpi_reset,
      input   io_tx_start,
      input   io_tx_data_valid,
      output reg  io_tx_data_ready,
      input  [7:0] io_tx_data_payload,
      output reg  io_rx_data_valid,
      output reg [8:0] io_rx_data_payload,
      output  io_rx_cmd_changed,
      output [7:0] io_rx_cmd,
      input   io_reg_rd,
      input   io_reg_wr,
      input  [5:0] io_reg_addr,
      input  [7:0] io_reg_wr_data,
      output [7:0] io_reg_rd_data,
      output  io_reg_done,
      output  ulpi_reset__1_);
  wire  _zz_UlpiCtrl_1_;
  wire  _zz_UlpiCtrl_2_;
  wire  _zz_UlpiCtrl_3_;
  wire  _zz_UlpiCtrl_4_;
  reg  ulpi_reset_ = 1'b0;
  reg `UlpiState_defaultEncoding_type ulpi_domain_cur_state;
  reg  ulpi_domain_ulpi_stp;
  reg [7:0] ulpi_domain_ulpi_data_out;
  reg  ulpi_domain_rx_cmd_changed;
  reg [7:0] ulpi_domain_rx_cmd;
  reg  ulpi_domain_direction_d;
  reg  ulpi_domain_reg_done;
  reg [7:0] ulpi_domain_reg_rd_data;
  reg  ulpi_domain_rx_data_seen;
  `ifndef SYNTHESIS
  reg [71:0] ulpi_domain_cur_state_string;
  `endif

  assign _zz_UlpiCtrl_1_ = (io_tx_start && io_tx_data_valid);
  assign _zz_UlpiCtrl_2_ = (! io_tx_data_valid);
  assign _zz_UlpiCtrl_3_ = (! io_ulpi_direction);
  assign _zz_UlpiCtrl_4_ = (! io_ulpi_nxt);
  `ifndef SYNTHESIS
  always @(*) begin
    case(ulpi_domain_cur_state)
      `UlpiState_defaultEncoding_WaitIdle : ulpi_domain_cur_state_string = "WaitIdle ";
      `UlpiState_defaultEncoding_Idle : ulpi_domain_cur_state_string = "Idle     ";
      `UlpiState_defaultEncoding_Tx : ulpi_domain_cur_state_string = "Tx       ";
      `UlpiState_defaultEncoding_Rx : ulpi_domain_cur_state_string = "Rx       ";
      `UlpiState_defaultEncoding_RegWrAddr : ulpi_domain_cur_state_string = "RegWrAddr";
      `UlpiState_defaultEncoding_RegWrData : ulpi_domain_cur_state_string = "RegWrData";
      `UlpiState_defaultEncoding_RegWrStp : ulpi_domain_cur_state_string = "RegWrStp ";
      `UlpiState_defaultEncoding_RegRdAddr : ulpi_domain_cur_state_string = "RegRdAddr";
      `UlpiState_defaultEncoding_RegRdTurn : ulpi_domain_cur_state_string = "RegRdTurn";
      `UlpiState_defaultEncoding_RegRdData : ulpi_domain_cur_state_string = "RegRdData";
      default : ulpi_domain_cur_state_string = "?????????";
    endcase
  end
  `endif

  assign io_ulpi_reset = 1'b0;
  assign io_ulpi_data_writeEnable = ((! io_ulpi_direction) ? (8'b11111111) : (8'b00000000));
  assign io_ulpi_stp = ulpi_domain_ulpi_stp;
  assign io_ulpi_data_write = ulpi_domain_ulpi_data_out;
  assign io_rx_cmd_changed = ulpi_domain_rx_cmd_changed;
  assign io_rx_cmd = ulpi_domain_rx_cmd;
  assign io_reg_done = ulpi_domain_reg_done;
  assign io_reg_rd_data = ulpi_domain_reg_rd_data;
  always @ (*) begin
    io_rx_data_valid = 1'b0;
    io_rx_data_payload = (9'b000000000);
    io_tx_data_ready = 1'b0;
    case(ulpi_domain_cur_state)
      `UlpiState_defaultEncoding_WaitIdle : begin
      end
      `UlpiState_defaultEncoding_Idle : begin
        if(! io_ulpi_direction) begin
          if(! io_reg_wr) begin
            if(! io_reg_rd) begin
              if(_zz_UlpiCtrl_1_)begin
                io_tx_data_ready = 1'b1;
              end
            end
          end
        end
      end
      `UlpiState_defaultEncoding_Tx : begin
        if(! io_ulpi_direction) begin
          if(io_ulpi_nxt)begin
            if(! _zz_UlpiCtrl_2_) begin
              io_tx_data_ready = 1'b1;
            end
          end
        end
      end
      `UlpiState_defaultEncoding_Rx : begin
        if(_zz_UlpiCtrl_3_)begin
          io_rx_data_valid = ulpi_domain_rx_data_seen;
          io_rx_data_payload = {ulpi_domain_rx_data_seen,(8'b00000000)};
        end else begin
          if(_zz_UlpiCtrl_4_)begin
            if((io_ulpi_data_read[5 : 0] == (6'b000000)))begin
              io_rx_data_valid = ulpi_domain_rx_data_seen;
              io_rx_data_payload = {ulpi_domain_rx_data_seen,(8'b00000000)};
            end
          end else begin
            io_rx_data_valid = 1'b1;
            io_rx_data_payload = {1'b0,io_ulpi_data_read};
          end
        end
      end
      `UlpiState_defaultEncoding_RegWrAddr : begin
      end
      `UlpiState_defaultEncoding_RegWrData : begin
      end
      `UlpiState_defaultEncoding_RegWrStp : begin
      end
      `UlpiState_defaultEncoding_RegRdAddr : begin
      end
      `UlpiState_defaultEncoding_RegRdTurn : begin
      end
      default : begin
      end
    endcase
  end

  assign ulpi_reset__1_ = ulpi_reset_;
  always @ (posedge io_ulpi_clk) begin
    ulpi_reset_ <= 1'b1;
  end

  always @ (posedge io_ulpi_clk) begin
    if(!ulpi_reset_) begin
      ulpi_domain_cur_state <= `UlpiState_defaultEncoding_WaitIdle;
      ulpi_domain_ulpi_stp <= 1'b0;
      ulpi_domain_ulpi_data_out <= (8'b00000000);
      ulpi_domain_rx_cmd_changed <= 1'b0;
      ulpi_domain_rx_cmd <= (8'b00000000);
      ulpi_domain_direction_d <= 1'b1;
      ulpi_domain_reg_done <= 1'b0;
      ulpi_domain_reg_rd_data <= (8'b00000000);
      ulpi_domain_rx_data_seen <= 1'b0;
    end else begin
      ulpi_domain_direction_d <= io_ulpi_direction;
      ulpi_domain_reg_done <= 1'b0;
      ulpi_domain_rx_cmd_changed <= 1'b0;
      case(ulpi_domain_cur_state)
        `UlpiState_defaultEncoding_WaitIdle : begin
          ulpi_domain_ulpi_data_out <= (8'b00000000);
          if((! io_ulpi_direction))begin
            ulpi_domain_cur_state <= `UlpiState_defaultEncoding_Idle;
          end
        end
        `UlpiState_defaultEncoding_Idle : begin
          ulpi_domain_ulpi_data_out <= (8'b00000000);
          ulpi_domain_ulpi_stp <= 1'b0;
          if(io_ulpi_direction)begin
            ulpi_domain_cur_state <= `UlpiState_defaultEncoding_Rx;
            ulpi_domain_rx_data_seen <= 1'b0;
          end else begin
            if(io_reg_wr)begin
              ulpi_domain_ulpi_data_out <= {(2'b10),io_reg_addr};
              ulpi_domain_cur_state <= `UlpiState_defaultEncoding_RegWrAddr;
            end else begin
              if(io_reg_rd)begin
                ulpi_domain_ulpi_data_out <= {(2'b11),io_reg_addr};
                ulpi_domain_cur_state <= `UlpiState_defaultEncoding_RegRdAddr;
              end else begin
                if(_zz_UlpiCtrl_1_)begin
                  ulpi_domain_ulpi_data_out <= {(2'b01),io_tx_data_payload[5 : 0]};
                  ulpi_domain_cur_state <= `UlpiState_defaultEncoding_Tx;
                end
              end
            end
          end
        end
        `UlpiState_defaultEncoding_Tx : begin
          if(io_ulpi_direction)begin
            ulpi_domain_cur_state <= `UlpiState_defaultEncoding_Rx;
          end else begin
            if(io_ulpi_nxt)begin
              if(_zz_UlpiCtrl_2_)begin
                ulpi_domain_ulpi_data_out <= (8'b00000000);
                ulpi_domain_ulpi_stp <= 1'b1;
                ulpi_domain_cur_state <= `UlpiState_defaultEncoding_Idle;
              end else begin
                ulpi_domain_ulpi_data_out <= io_tx_data_payload;
                ulpi_domain_ulpi_stp <= 1'b0;
              end
            end
          end
        end
        `UlpiState_defaultEncoding_Rx : begin
          if(_zz_UlpiCtrl_3_)begin
            ulpi_domain_cur_state <= `UlpiState_defaultEncoding_Idle;
          end else begin
            if(_zz_UlpiCtrl_4_)begin
              ulpi_domain_rx_cmd_changed <= 1'b1;
              ulpi_domain_rx_cmd <= io_ulpi_data_read;
            end else begin
              ulpi_domain_rx_data_seen <= 1'b1;
            end
          end
        end
        `UlpiState_defaultEncoding_RegWrAddr : begin
          if(io_ulpi_direction)begin
            ulpi_domain_cur_state <= `UlpiState_defaultEncoding_Rx;
          end else begin
            if(io_ulpi_nxt)begin
              ulpi_domain_ulpi_data_out <= io_reg_wr_data;
              ulpi_domain_cur_state <= `UlpiState_defaultEncoding_RegWrData;
            end
          end
        end
        `UlpiState_defaultEncoding_RegWrData : begin
          if(io_ulpi_direction)begin
            ulpi_domain_cur_state <= `UlpiState_defaultEncoding_Rx;
          end else begin
            if(io_ulpi_nxt)begin
              ulpi_domain_ulpi_data_out <= (8'b00000000);
              ulpi_domain_ulpi_stp <= 1'b1;
              ulpi_domain_cur_state <= `UlpiState_defaultEncoding_RegWrStp;
            end
          end
        end
        `UlpiState_defaultEncoding_RegWrStp : begin
          if(io_ulpi_direction)begin
            ulpi_domain_cur_state <= `UlpiState_defaultEncoding_Rx;
          end else begin
            ulpi_domain_ulpi_data_out <= (8'b00000000);
            ulpi_domain_ulpi_stp <= 1'b0;
            ulpi_domain_reg_done <= 1'b1;
            ulpi_domain_cur_state <= `UlpiState_defaultEncoding_Idle;
          end
        end
        `UlpiState_defaultEncoding_RegRdAddr : begin
          if(io_ulpi_direction)begin
            ulpi_domain_cur_state <= `UlpiState_defaultEncoding_Rx;
          end else begin
            if(io_ulpi_nxt)begin
              ulpi_domain_cur_state <= `UlpiState_defaultEncoding_RegRdTurn;
            end
          end
        end
        `UlpiState_defaultEncoding_RegRdTurn : begin
          ulpi_domain_cur_state <= `UlpiState_defaultEncoding_RegRdData;
        end
        default : begin
          ulpi_domain_reg_done <= 1'b1;
          ulpi_domain_reg_rd_data <= io_ulpi_data_read;
          ulpi_domain_cur_state <= `UlpiState_defaultEncoding_Idle;
        end
      endcase
    end
  end

endmodule

module StreamFifoCC_3_ (
      input   io_push_valid,
      output  io_push_ready,
      input   io_push_payload,
      output  io_pop_valid,
      input   io_pop_ready,
      output  io_pop_payload,
      output [1:0] io_pushOccupancy,
      output [1:0] io_popOccupancy,
      input   toplevel_main_clk,
      input   toplevel_main_reset_,
      input   _zz_StreamFifoCC_3__5_,
      input   _zz_StreamFifoCC_3__6_);
  wire [1:0] _zz_StreamFifoCC_3__7_;
  wire [1:0] _zz_StreamFifoCC_3__8_;
  reg [0:0] _zz_StreamFifoCC_3__9_;
  wire [1:0] bufferCC_19__io_dataOut;
  wire [1:0] bufferCC_20__io_dataOut;
  wire [0:0] _zz_StreamFifoCC_3__10_;
  wire [1:0] _zz_StreamFifoCC_3__11_;
  wire [1:0] _zz_StreamFifoCC_3__12_;
  wire [0:0] _zz_StreamFifoCC_3__13_;
  wire [0:0] _zz_StreamFifoCC_3__14_;
  wire [1:0] _zz_StreamFifoCC_3__15_;
  wire [1:0] _zz_StreamFifoCC_3__16_;
  wire [0:0] _zz_StreamFifoCC_3__17_;
  wire [0:0] _zz_StreamFifoCC_3__18_;
  wire  _zz_StreamFifoCC_3__19_;
  reg  _zz_StreamFifoCC_3__1_;
  wire [1:0] popToPushGray;
  wire [1:0] pushToPopGray;
  reg  pushCC_pushPtr_willIncrement;
  wire  pushCC_pushPtr_willClear;
  reg [1:0] pushCC_pushPtr_valueNext;
  reg [1:0] pushCC_pushPtr_value;
  wire  pushCC_pushPtr_willOverflowIfInc;
  wire  pushCC_pushPtr_willOverflow;
  reg [1:0] pushCC_pushPtrGray;
  wire [1:0] pushCC_popPtrGray;
  wire  pushCC_full;
  wire  _zz_StreamFifoCC_3__2_;
  reg  popCC_popPtr_willIncrement;
  wire  popCC_popPtr_willClear;
  reg [1:0] popCC_popPtr_valueNext;
  reg [1:0] popCC_popPtr_value;
  wire  popCC_popPtr_willOverflowIfInc;
  wire  popCC_popPtr_willOverflow;
  reg [1:0] popCC_popPtrGray;
  wire [1:0] popCC_pushPtrGray;
  wire  popCC_empty;
  wire [1:0] _zz_StreamFifoCC_3__3_;
  wire  _zz_StreamFifoCC_3__4_;
  reg [0:0] ram [0:1];
  assign _zz_StreamFifoCC_3__10_ = pushCC_pushPtr_willIncrement;
  assign _zz_StreamFifoCC_3__11_ = {1'd0, _zz_StreamFifoCC_3__10_};
  assign _zz_StreamFifoCC_3__12_ = (pushCC_pushPtr_valueNext >>> (1'b1));
  assign _zz_StreamFifoCC_3__13_ = pushCC_pushPtr_value[0:0];
  assign _zz_StreamFifoCC_3__14_ = popCC_popPtr_willIncrement;
  assign _zz_StreamFifoCC_3__15_ = {1'd0, _zz_StreamFifoCC_3__14_};
  assign _zz_StreamFifoCC_3__16_ = (popCC_popPtr_valueNext >>> (1'b1));
  assign _zz_StreamFifoCC_3__17_ = _zz_StreamFifoCC_3__3_[0:0];
  assign _zz_StreamFifoCC_3__18_ = io_push_payload;
  assign _zz_StreamFifoCC_3__19_ = 1'b1;
  always @ (posedge toplevel_main_clk) begin
    if(_zz_StreamFifoCC_3__1_) begin
      ram[_zz_StreamFifoCC_3__13_] <= _zz_StreamFifoCC_3__18_;
    end
  end

  always @ (posedge _zz_StreamFifoCC_3__5_) begin
  end

  always @ (posedge _zz_StreamFifoCC_3__5_) begin
    if(_zz_StreamFifoCC_3__19_) begin
      _zz_StreamFifoCC_3__9_ <= ram[_zz_StreamFifoCC_3__17_];
    end
  end

  BufferCC_11_ bufferCC_19_ ( 
    .io_initial(_zz_StreamFifoCC_3__7_),
    .io_dataIn(popToPushGray),
    .io_dataOut(bufferCC_19__io_dataOut),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  BufferCC_12_ bufferCC_20_ ( 
    .io_initial(_zz_StreamFifoCC_3__8_),
    .io_dataIn(pushToPopGray),
    .io_dataOut(bufferCC_20__io_dataOut),
    ._zz_BufferCC_12__1_(_zz_StreamFifoCC_3__5_),
    ._zz_BufferCC_12__2_(_zz_StreamFifoCC_3__6_) 
  );
  always @ (*) begin
    _zz_StreamFifoCC_3__1_ = 1'b0;
    pushCC_pushPtr_willIncrement = 1'b0;
    if((io_push_valid && io_push_ready))begin
      _zz_StreamFifoCC_3__1_ = 1'b1;
      pushCC_pushPtr_willIncrement = 1'b1;
    end
  end

  assign pushCC_pushPtr_willClear = 1'b0;
  assign pushCC_pushPtr_willOverflowIfInc = (pushCC_pushPtr_value == (2'b11));
  assign pushCC_pushPtr_willOverflow = (pushCC_pushPtr_willOverflowIfInc && pushCC_pushPtr_willIncrement);
  always @ (*) begin
    pushCC_pushPtr_valueNext = (pushCC_pushPtr_value + _zz_StreamFifoCC_3__11_);
    if(pushCC_pushPtr_willClear)begin
      pushCC_pushPtr_valueNext = (2'b00);
    end
  end

  assign _zz_StreamFifoCC_3__7_ = (2'b00);
  assign pushCC_popPtrGray = bufferCC_19__io_dataOut;
  assign pushCC_full = ((pushCC_pushPtrGray[1 : 0] == (~ pushCC_popPtrGray[1 : 0])) && 1'b1);
  assign io_push_ready = (! pushCC_full);
  assign _zz_StreamFifoCC_3__2_ = pushCC_popPtrGray[1];
  assign io_pushOccupancy = (pushCC_pushPtr_value - {_zz_StreamFifoCC_3__2_,(pushCC_popPtrGray[0] ^ _zz_StreamFifoCC_3__2_)});
  always @ (*) begin
    popCC_popPtr_willIncrement = 1'b0;
    if((io_pop_valid && io_pop_ready))begin
      popCC_popPtr_willIncrement = 1'b1;
    end
  end

  assign popCC_popPtr_willClear = 1'b0;
  assign popCC_popPtr_willOverflowIfInc = (popCC_popPtr_value == (2'b11));
  assign popCC_popPtr_willOverflow = (popCC_popPtr_willOverflowIfInc && popCC_popPtr_willIncrement);
  always @ (*) begin
    popCC_popPtr_valueNext = (popCC_popPtr_value + _zz_StreamFifoCC_3__15_);
    if(popCC_popPtr_willClear)begin
      popCC_popPtr_valueNext = (2'b00);
    end
  end

  assign _zz_StreamFifoCC_3__8_ = (2'b00);
  assign popCC_pushPtrGray = bufferCC_20__io_dataOut;
  assign popCC_empty = (popCC_popPtrGray == popCC_pushPtrGray);
  assign io_pop_valid = (! popCC_empty);
  assign _zz_StreamFifoCC_3__3_ = popCC_popPtr_valueNext;
  assign io_pop_payload = _zz_StreamFifoCC_3__9_[0];
  assign _zz_StreamFifoCC_3__4_ = popCC_pushPtrGray[1];
  assign io_popOccupancy = ({_zz_StreamFifoCC_3__4_,(popCC_pushPtrGray[0] ^ _zz_StreamFifoCC_3__4_)} - popCC_popPtr_value);
  assign pushToPopGray = pushCC_pushPtrGray;
  assign popToPushGray = popCC_popPtrGray;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      pushCC_pushPtr_value <= (2'b00);
      pushCC_pushPtrGray <= (2'b00);
    end else begin
      pushCC_pushPtr_value <= pushCC_pushPtr_valueNext;
      pushCC_pushPtrGray <= (_zz_StreamFifoCC_3__12_ ^ pushCC_pushPtr_valueNext);
    end
  end

  always @ (posedge _zz_StreamFifoCC_3__5_) begin
    if(!_zz_StreamFifoCC_3__6_) begin
      popCC_popPtr_value <= (2'b00);
      popCC_popPtrGray <= (2'b00);
    end else begin
      popCC_popPtr_value <= popCC_popPtr_valueNext;
      popCC_popPtrGray <= (_zz_StreamFifoCC_3__16_ ^ popCC_popPtr_valueNext);
    end
  end

endmodule

module PulseCCByToggle_2_ (
      input   io_pulseIn,
      output  io_pulseOut,
      input   _zz_PulseCCByToggle_2__1_,
      input   _zz_PulseCCByToggle_2__2_,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire  _zz_PulseCCByToggle_2__3_;
  wire  bufferCC_19__io_dataOut;
  reg  inArea_target;
  wire  outArea_target;
  reg  outArea_hit;
  BufferCC_7_ bufferCC_19_ ( 
    .io_initial(_zz_PulseCCByToggle_2__3_),
    .io_dataIn(inArea_target),
    .io_dataOut(bufferCC_19__io_dataOut),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  assign _zz_PulseCCByToggle_2__3_ = 1'b0;
  assign outArea_target = bufferCC_19__io_dataOut;
  assign io_pulseOut = (outArea_target != outArea_hit);
  always @ (posedge _zz_PulseCCByToggle_2__1_) begin
    if(!_zz_PulseCCByToggle_2__2_) begin
      inArea_target <= 1'b0;
    end else begin
      if(io_pulseIn)begin
        inArea_target <= (! inArea_target);
      end
    end
  end

  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      outArea_hit <= 1'b0;
    end else begin
      if((outArea_target != outArea_hit))begin
        outArea_hit <= (! outArea_hit);
      end
    end
  end

endmodule

module StreamFifoCC_4_ (
      input   io_push_valid,
      output  io_push_ready,
      input  [7:0] io_push_payload,
      output  io_pop_valid,
      input   io_pop_ready,
      output [7:0] io_pop_payload,
      output [10:0] io_pushOccupancy,
      output [10:0] io_popOccupancy,
      input   toplevel_main_clk,
      input   toplevel_main_reset_,
      input   _zz_StreamFifoCC_4__23_,
      input   _zz_StreamFifoCC_4__24_);
  wire [10:0] _zz_StreamFifoCC_4__25_;
  wire [10:0] _zz_StreamFifoCC_4__26_;
  reg [7:0] _zz_StreamFifoCC_4__27_;
  wire [10:0] bufferCC_19__io_dataOut;
  wire [10:0] bufferCC_20__io_dataOut;
  wire [0:0] _zz_StreamFifoCC_4__28_;
  wire [10:0] _zz_StreamFifoCC_4__29_;
  wire [10:0] _zz_StreamFifoCC_4__30_;
  wire [9:0] _zz_StreamFifoCC_4__31_;
  wire [0:0] _zz_StreamFifoCC_4__32_;
  wire [10:0] _zz_StreamFifoCC_4__33_;
  wire [10:0] _zz_StreamFifoCC_4__34_;
  wire [9:0] _zz_StreamFifoCC_4__35_;
  wire  _zz_StreamFifoCC_4__36_;
  wire [0:0] _zz_StreamFifoCC_4__37_;
  wire [0:0] _zz_StreamFifoCC_4__38_;
  wire [0:0] _zz_StreamFifoCC_4__39_;
  wire [0:0] _zz_StreamFifoCC_4__40_;
  reg  _zz_StreamFifoCC_4__1_;
  wire [10:0] popToPushGray;
  wire [10:0] pushToPopGray;
  reg  pushCC_pushPtr_willIncrement;
  wire  pushCC_pushPtr_willClear;
  reg [10:0] pushCC_pushPtr_valueNext;
  reg [10:0] pushCC_pushPtr_value;
  wire  pushCC_pushPtr_willOverflowIfInc;
  wire  pushCC_pushPtr_willOverflow;
  reg [10:0] pushCC_pushPtrGray;
  wire [10:0] pushCC_popPtrGray;
  wire  pushCC_full;
  wire  _zz_StreamFifoCC_4__2_;
  wire  _zz_StreamFifoCC_4__3_;
  wire  _zz_StreamFifoCC_4__4_;
  wire  _zz_StreamFifoCC_4__5_;
  wire  _zz_StreamFifoCC_4__6_;
  wire  _zz_StreamFifoCC_4__7_;
  wire  _zz_StreamFifoCC_4__8_;
  wire  _zz_StreamFifoCC_4__9_;
  wire  _zz_StreamFifoCC_4__10_;
  wire  _zz_StreamFifoCC_4__11_;
  reg  popCC_popPtr_willIncrement;
  wire  popCC_popPtr_willClear;
  reg [10:0] popCC_popPtr_valueNext;
  reg [10:0] popCC_popPtr_value;
  wire  popCC_popPtr_willOverflowIfInc;
  wire  popCC_popPtr_willOverflow;
  reg [10:0] popCC_popPtrGray;
  wire [10:0] popCC_pushPtrGray;
  wire  popCC_empty;
  wire [10:0] _zz_StreamFifoCC_4__12_;
  wire  _zz_StreamFifoCC_4__13_;
  wire  _zz_StreamFifoCC_4__14_;
  wire  _zz_StreamFifoCC_4__15_;
  wire  _zz_StreamFifoCC_4__16_;
  wire  _zz_StreamFifoCC_4__17_;
  wire  _zz_StreamFifoCC_4__18_;
  wire  _zz_StreamFifoCC_4__19_;
  wire  _zz_StreamFifoCC_4__20_;
  wire  _zz_StreamFifoCC_4__21_;
  wire  _zz_StreamFifoCC_4__22_;
  reg [7:0] ram [0:1023];
  assign _zz_StreamFifoCC_4__28_ = pushCC_pushPtr_willIncrement;
  assign _zz_StreamFifoCC_4__29_ = {10'd0, _zz_StreamFifoCC_4__28_};
  assign _zz_StreamFifoCC_4__30_ = (pushCC_pushPtr_valueNext >>> (1'b1));
  assign _zz_StreamFifoCC_4__31_ = pushCC_pushPtr_value[9:0];
  assign _zz_StreamFifoCC_4__32_ = popCC_popPtr_willIncrement;
  assign _zz_StreamFifoCC_4__33_ = {10'd0, _zz_StreamFifoCC_4__32_};
  assign _zz_StreamFifoCC_4__34_ = (popCC_popPtr_valueNext >>> (1'b1));
  assign _zz_StreamFifoCC_4__35_ = _zz_StreamFifoCC_4__12_[9:0];
  assign _zz_StreamFifoCC_4__36_ = 1'b1;
  assign _zz_StreamFifoCC_4__37_ = _zz_StreamFifoCC_4__2_;
  assign _zz_StreamFifoCC_4__38_ = (pushCC_popPtrGray[0] ^ _zz_StreamFifoCC_4__2_);
  assign _zz_StreamFifoCC_4__39_ = _zz_StreamFifoCC_4__13_;
  assign _zz_StreamFifoCC_4__40_ = (popCC_pushPtrGray[0] ^ _zz_StreamFifoCC_4__13_);
  always @ (posedge toplevel_main_clk) begin
    if(_zz_StreamFifoCC_4__1_) begin
      ram[_zz_StreamFifoCC_4__31_] <= io_push_payload;
    end
  end

  always @ (posedge _zz_StreamFifoCC_4__23_) begin
  end

  always @ (posedge _zz_StreamFifoCC_4__23_) begin
    if(_zz_StreamFifoCC_4__36_) begin
      _zz_StreamFifoCC_4__27_ <= ram[_zz_StreamFifoCC_4__35_];
    end
  end

  BufferCC_14_ bufferCC_19_ ( 
    .io_initial(_zz_StreamFifoCC_4__25_),
    .io_dataIn(popToPushGray),
    .io_dataOut(bufferCC_19__io_dataOut),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  BufferCC_15_ bufferCC_20_ ( 
    .io_initial(_zz_StreamFifoCC_4__26_),
    .io_dataIn(pushToPopGray),
    .io_dataOut(bufferCC_20__io_dataOut),
    ._zz_BufferCC_15__1_(_zz_StreamFifoCC_4__23_),
    ._zz_BufferCC_15__2_(_zz_StreamFifoCC_4__24_) 
  );
  always @ (*) begin
    _zz_StreamFifoCC_4__1_ = 1'b0;
    pushCC_pushPtr_willIncrement = 1'b0;
    if((io_push_valid && io_push_ready))begin
      _zz_StreamFifoCC_4__1_ = 1'b1;
      pushCC_pushPtr_willIncrement = 1'b1;
    end
  end

  assign pushCC_pushPtr_willClear = 1'b0;
  assign pushCC_pushPtr_willOverflowIfInc = (pushCC_pushPtr_value == (11'b11111111111));
  assign pushCC_pushPtr_willOverflow = (pushCC_pushPtr_willOverflowIfInc && pushCC_pushPtr_willIncrement);
  always @ (*) begin
    pushCC_pushPtr_valueNext = (pushCC_pushPtr_value + _zz_StreamFifoCC_4__29_);
    if(pushCC_pushPtr_willClear)begin
      pushCC_pushPtr_valueNext = (11'b00000000000);
    end
  end

  assign _zz_StreamFifoCC_4__25_ = (11'b00000000000);
  assign pushCC_popPtrGray = bufferCC_19__io_dataOut;
  assign pushCC_full = ((pushCC_pushPtrGray[10 : 9] == (~ pushCC_popPtrGray[10 : 9])) && (pushCC_pushPtrGray[8 : 0] == pushCC_popPtrGray[8 : 0]));
  assign io_push_ready = (! pushCC_full);
  assign _zz_StreamFifoCC_4__2_ = (pushCC_popPtrGray[1] ^ _zz_StreamFifoCC_4__3_);
  assign _zz_StreamFifoCC_4__3_ = (pushCC_popPtrGray[2] ^ _zz_StreamFifoCC_4__4_);
  assign _zz_StreamFifoCC_4__4_ = (pushCC_popPtrGray[3] ^ _zz_StreamFifoCC_4__5_);
  assign _zz_StreamFifoCC_4__5_ = (pushCC_popPtrGray[4] ^ _zz_StreamFifoCC_4__6_);
  assign _zz_StreamFifoCC_4__6_ = (pushCC_popPtrGray[5] ^ _zz_StreamFifoCC_4__7_);
  assign _zz_StreamFifoCC_4__7_ = (pushCC_popPtrGray[6] ^ _zz_StreamFifoCC_4__8_);
  assign _zz_StreamFifoCC_4__8_ = (pushCC_popPtrGray[7] ^ _zz_StreamFifoCC_4__9_);
  assign _zz_StreamFifoCC_4__9_ = (pushCC_popPtrGray[8] ^ _zz_StreamFifoCC_4__10_);
  assign _zz_StreamFifoCC_4__10_ = (pushCC_popPtrGray[9] ^ _zz_StreamFifoCC_4__11_);
  assign _zz_StreamFifoCC_4__11_ = pushCC_popPtrGray[10];
  assign io_pushOccupancy = (pushCC_pushPtr_value - {_zz_StreamFifoCC_4__11_,{_zz_StreamFifoCC_4__10_,{_zz_StreamFifoCC_4__9_,{_zz_StreamFifoCC_4__8_,{_zz_StreamFifoCC_4__7_,{_zz_StreamFifoCC_4__6_,{_zz_StreamFifoCC_4__5_,{_zz_StreamFifoCC_4__4_,{_zz_StreamFifoCC_4__3_,{_zz_StreamFifoCC_4__37_,_zz_StreamFifoCC_4__38_}}}}}}}}}});
  always @ (*) begin
    popCC_popPtr_willIncrement = 1'b0;
    if((io_pop_valid && io_pop_ready))begin
      popCC_popPtr_willIncrement = 1'b1;
    end
  end

  assign popCC_popPtr_willClear = 1'b0;
  assign popCC_popPtr_willOverflowIfInc = (popCC_popPtr_value == (11'b11111111111));
  assign popCC_popPtr_willOverflow = (popCC_popPtr_willOverflowIfInc && popCC_popPtr_willIncrement);
  always @ (*) begin
    popCC_popPtr_valueNext = (popCC_popPtr_value + _zz_StreamFifoCC_4__33_);
    if(popCC_popPtr_willClear)begin
      popCC_popPtr_valueNext = (11'b00000000000);
    end
  end

  assign _zz_StreamFifoCC_4__26_ = (11'b00000000000);
  assign popCC_pushPtrGray = bufferCC_20__io_dataOut;
  assign popCC_empty = (popCC_popPtrGray == popCC_pushPtrGray);
  assign io_pop_valid = (! popCC_empty);
  assign _zz_StreamFifoCC_4__12_ = popCC_popPtr_valueNext;
  assign io_pop_payload = _zz_StreamFifoCC_4__27_;
  assign _zz_StreamFifoCC_4__13_ = (popCC_pushPtrGray[1] ^ _zz_StreamFifoCC_4__14_);
  assign _zz_StreamFifoCC_4__14_ = (popCC_pushPtrGray[2] ^ _zz_StreamFifoCC_4__15_);
  assign _zz_StreamFifoCC_4__15_ = (popCC_pushPtrGray[3] ^ _zz_StreamFifoCC_4__16_);
  assign _zz_StreamFifoCC_4__16_ = (popCC_pushPtrGray[4] ^ _zz_StreamFifoCC_4__17_);
  assign _zz_StreamFifoCC_4__17_ = (popCC_pushPtrGray[5] ^ _zz_StreamFifoCC_4__18_);
  assign _zz_StreamFifoCC_4__18_ = (popCC_pushPtrGray[6] ^ _zz_StreamFifoCC_4__19_);
  assign _zz_StreamFifoCC_4__19_ = (popCC_pushPtrGray[7] ^ _zz_StreamFifoCC_4__20_);
  assign _zz_StreamFifoCC_4__20_ = (popCC_pushPtrGray[8] ^ _zz_StreamFifoCC_4__21_);
  assign _zz_StreamFifoCC_4__21_ = (popCC_pushPtrGray[9] ^ _zz_StreamFifoCC_4__22_);
  assign _zz_StreamFifoCC_4__22_ = popCC_pushPtrGray[10];
  assign io_popOccupancy = ({_zz_StreamFifoCC_4__22_,{_zz_StreamFifoCC_4__21_,{_zz_StreamFifoCC_4__20_,{_zz_StreamFifoCC_4__19_,{_zz_StreamFifoCC_4__18_,{_zz_StreamFifoCC_4__17_,{_zz_StreamFifoCC_4__16_,{_zz_StreamFifoCC_4__15_,{_zz_StreamFifoCC_4__14_,{_zz_StreamFifoCC_4__39_,_zz_StreamFifoCC_4__40_}}}}}}}}}} - popCC_popPtr_value);
  assign pushToPopGray = pushCC_pushPtrGray;
  assign popToPushGray = popCC_popPtrGray;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      pushCC_pushPtr_value <= (11'b00000000000);
      pushCC_pushPtrGray <= (11'b00000000000);
    end else begin
      pushCC_pushPtr_value <= pushCC_pushPtr_valueNext;
      pushCC_pushPtrGray <= (_zz_StreamFifoCC_4__30_ ^ pushCC_pushPtr_valueNext);
    end
  end

  always @ (posedge _zz_StreamFifoCC_4__23_) begin
    if(!_zz_StreamFifoCC_4__24_) begin
      popCC_popPtr_value <= (11'b00000000000);
      popCC_popPtrGray <= (11'b00000000000);
    end else begin
      popCC_popPtr_value <= popCC_popPtr_valueNext;
      popCC_popPtrGray <= (_zz_StreamFifoCC_4__34_ ^ popCC_popPtr_valueNext);
    end
  end

endmodule

module PulseCCByToggle_3_ (
      input   io_pulseIn,
      output  io_pulseOut,
      input   toplevel_main_clk,
      input   toplevel_main_reset_,
      input   _zz_PulseCCByToggle_3__1_,
      input   _zz_PulseCCByToggle_3__2_);
  wire  _zz_PulseCCByToggle_3__3_;
  wire  bufferCC_19__io_dataOut;
  reg  inArea_target;
  wire  outArea_target;
  reg  outArea_hit;
  BufferCC_16_ bufferCC_19_ ( 
    .io_initial(_zz_PulseCCByToggle_3__3_),
    .io_dataIn(inArea_target),
    .io_dataOut(bufferCC_19__io_dataOut),
    ._zz_BufferCC_16__1_(_zz_PulseCCByToggle_3__1_),
    ._zz_BufferCC_16__2_(_zz_PulseCCByToggle_3__2_) 
  );
  assign _zz_PulseCCByToggle_3__3_ = 1'b0;
  assign outArea_target = bufferCC_19__io_dataOut;
  assign io_pulseOut = (outArea_target != outArea_hit);
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      inArea_target <= 1'b0;
    end else begin
      if(io_pulseIn)begin
        inArea_target <= (! inArea_target);
      end
    end
  end

  always @ (posedge _zz_PulseCCByToggle_3__1_) begin
    if(!_zz_PulseCCByToggle_3__2_) begin
      outArea_hit <= 1'b0;
    end else begin
      if((outArea_target != outArea_hit))begin
        outArea_hit <= (! outArea_hit);
      end
    end
  end

endmodule


//BufferCC_18_ remplaced by BufferCC_7_

module Apb3CC (
      input  [6:0] io_src_PADDR,
      input  [0:0] io_src_PSEL,
      input   io_src_PENABLE,
      output  io_src_PREADY,
      input   io_src_PWRITE,
      input  [31:0] io_src_PWDATA,
      output [31:0] io_src_PRDATA,
      output  io_src_PSLVERROR,
      output [6:0] io_dest_PADDR,
      output [0:0] io_dest_PSEL,
      output  io_dest_PENABLE,
      input   io_dest_PREADY,
      output  io_dest_PWRITE,
      output [31:0] io_dest_PWDATA,
      input  [31:0] io_dest_PRDATA,
      input   io_dest_PSLVERROR,
      input   toplevel_main_clk,
      input   toplevel_main_reset_,
      input   core_u_pano_core_io_ulpi_clk,
      input   _zz_Apb3CC_2_);
  wire  u_sync_pulse_xfer_done_io_pulseOut;
  wire  u_sync_pulse_xfer_start_io_pulseOut;
  wire [31:0] PRDATA_dest;
  wire  xfer_done_src;
  wire  xfer_done_dest;
  reg  src_xfer_start;
  reg [6:0] src_PADDR;
  reg [0:0] src_PSEL;
  reg  src_PWRITE;
  reg [31:0] src_PWDATA;
  reg [31:0] src_PRDATA;
  reg  src_PREADY;
  reg  src_PSLVERROR;
  wire  _zz_Apb3CC_1_;
  reg  _zz_Apb3CC_1__regNext;
  wire  xfer_start_dest;
  reg  dest_xfer_start_dest_d1;
  reg [6:0] dest_PADDR;
  reg [0:0] dest_PSEL;
  reg  dest_PWRITE;
  reg [31:0] dest_PWDATA;
  reg [31:0] dest_PRDATA;
  reg  dest_PSLVERROR;
  reg  dest_PENABLE;
  reg  dest_xfer_done;
  PulseCCByToggle u_sync_pulse_xfer_done ( 
    .io_pulseIn(xfer_done_dest),
    .io_pulseOut(u_sync_pulse_xfer_done_io_pulseOut),
    .core_u_pano_core_io_ulpi_clk(core_u_pano_core_io_ulpi_clk),
    ._zz_PulseCCByToggle_1_(_zz_Apb3CC_2_),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  PulseCCByToggle_1_ u_sync_pulse_xfer_start ( 
    .io_pulseIn(src_xfer_start),
    .io_pulseOut(u_sync_pulse_xfer_start_io_pulseOut),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_),
    .core_u_pano_core_io_ulpi_clk(core_u_pano_core_io_ulpi_clk),
    ._zz_PulseCCByToggle_1__1_(_zz_Apb3CC_2_) 
  );
  assign xfer_done_src = u_sync_pulse_xfer_done_io_pulseOut;
  assign _zz_Apb3CC_1_ = (io_src_PENABLE && (io_src_PSEL != (1'b0)));
  assign io_src_PRDATA = src_PRDATA;
  assign io_src_PREADY = src_PREADY;
  assign xfer_start_dest = u_sync_pulse_xfer_start_io_pulseOut;
  assign io_dest_PENABLE = dest_PENABLE;
  assign io_dest_PADDR = dest_PADDR;
  assign io_dest_PSEL = dest_PSEL;
  assign io_dest_PWRITE = dest_PWRITE;
  assign io_dest_PWDATA = dest_PWDATA;
  assign PRDATA_dest = dest_PRDATA;
  assign xfer_done_dest = dest_xfer_done;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      src_xfer_start <= 1'b0;
      src_PADDR <= (7'b0000000);
      src_PSEL <= (1'b0);
      src_PWRITE <= 1'b0;
      src_PWDATA <= (32'b00000000000000000000000000000000);
      src_PRDATA <= (32'b00000000000000000000000000000000);
      src_PREADY <= 1'b0;
    end else begin
      src_xfer_start <= 1'b0;
      if((_zz_Apb3CC_1_ && (! _zz_Apb3CC_1__regNext)))begin
        src_xfer_start <= 1'b1;
        src_PADDR <= io_src_PADDR;
        src_PSEL <= io_src_PSEL;
        src_PWRITE <= io_src_PWRITE;
        src_PWDATA <= io_src_PWDATA;
      end
      src_PREADY <= 1'b0;
      if(xfer_done_src)begin
        src_PREADY <= 1'b1;
        if((! io_src_PWRITE))begin
          src_PRDATA <= PRDATA_dest;
        end
      end
    end
  end

  always @ (posedge toplevel_main_clk) begin
    _zz_Apb3CC_1__regNext <= _zz_Apb3CC_1_;
  end

  always @ (posedge core_u_pano_core_io_ulpi_clk) begin
    if(!_zz_Apb3CC_2_) begin
      dest_xfer_start_dest_d1 <= 1'b0;
      dest_PADDR <= (7'b0000000);
      dest_PSEL <= (1'b0);
      dest_PWRITE <= 1'b0;
      dest_PWDATA <= (32'b00000000000000000000000000000000);
      dest_PRDATA <= (32'b00000000000000000000000000000000);
      dest_PENABLE <= 1'b0;
      dest_xfer_done <= 1'b0;
    end else begin
      dest_xfer_start_dest_d1 <= xfer_start_dest;
      if(xfer_start_dest)begin
        dest_PADDR <= src_PADDR;
        dest_PSEL <= src_PSEL;
        dest_PWRITE <= src_PWRITE;
        dest_PWDATA <= src_PWDATA;
      end
      if(dest_xfer_start_dest_d1)begin
        dest_PENABLE <= 1'b1;
      end
      if(io_dest_PREADY)begin
        dest_PENABLE <= 1'b0;
      end
      dest_xfer_done <= 1'b0;
      if((dest_PENABLE && io_dest_PREADY))begin
        dest_PSEL <= (1'b0);
        if((! io_dest_PWRITE))begin
          dest_PRDATA <= io_dest_PRDATA;
        end
        dest_xfer_done <= 1'b1;
      end
    end
  end

endmodule

module UsbHost (
      input   io_cpu_fifo_bus_cmd_valid,
      output  io_cpu_fifo_bus_cmd_ready,
      input   io_cpu_fifo_bus_cmd_payload_write,
      input  [8:0] io_cpu_fifo_bus_cmd_payload_address,
      input  [7:0] io_cpu_fifo_bus_cmd_payload_data,
      input  [0:0] io_cpu_fifo_bus_cmd_payload_mask,
      output  io_cpu_fifo_bus_rsp_valid,
      output [7:0] io_cpu_fifo_bus_rsp_payload_data,
      input  [6:0] io_periph_addr,
      input  [3:0] io_endpoint,
      output  io_send_buf_avail,
      output [0:0] io_send_buf_avail_nr,
      input   io_send_byte_count_valid,
      input  [5:0] io_send_byte_count_payload,
      input   io_xfer_type_valid,
      input  `HostXferType_staticEncoding_type io_xfer_type_payload,
      output `HostXferResult_defaultEncoding_type io_xfer_result,
      output  io_cur_send_data_toggle,
      output  io_cur_rcv_data_toggle,
      input   io_set_send_data_toggle_valid,
      input   io_set_send_data_toggle_payload,
      input   io_set_rcv_data_toggle_valid,
      input   io_set_rcv_data_toggle_payload,
      output  io_ulpi_rx_cmd_changed,
      output [7:0] io_ulpi_rx_cmd,
      output reg  io_ulpi_tx_data_valid,
      input   io_ulpi_tx_data_ready,
      output reg [7:0] io_ulpi_tx_data_payload,
      input   core_u_pano_core_io_ulpi_clk,
      input   _zz_UsbHost_12_);
  reg [7:0] _zz_UsbHost_13_;
  reg [7:0] _zz_UsbHost_14_;
  wire  _zz_UsbHost_15_;
  wire  _zz_UsbHost_16_;
  wire  _zz_UsbHost_17_;
  wire [8:0] _zz_UsbHost_18_;
  wire  _zz_UsbHost_19_;
  wire  _zz_UsbHost_20_;
  wire [0:0] _zz_UsbHost_1_;
  wire [7:0] _zz_UsbHost_2_;
  reg  _zz_UsbHost_3_;
  wire  rxtx_ram_access_tx_rd_req;
  wire [8:0] rxtx_ram_access_tx_rd_addr;
  wire  rxtx_ram_access_rx_wr_req;
  wire [8:0] rxtx_ram_access_rx_wr_addr;
  wire [7:0] rxtx_ram_access_rx_wr_data;
  wire [8:0] rxtx_ram_access_rxtx_addr;
  wire  _zz_UsbHost_4_;
  wire [0:0] _zz_UsbHost_5_;
  wire [7:0] _zz_UsbHost_6_;
  wire [7:0] rxtx_ram_access_tx_rd_data;
  reg [0:0] tx_buf_cur_buf;
  reg [1:0] tx_buf_buf_primed;
  reg [5:0] tx_buf_byte_count0;
  reg [5:0] tx_buf_byte_count1;
  wire [5:0] tx_buf_cur_byte_count;
  wire [8:0] tx_buf_cur_first_byte_ptr;
  wire [7:0] tx_buf_setup_first_byte_ptr;
  wire  data_toggle_toggle_send;
  wire  data_toggle_toggle_rcv;
  reg  data_toggle_cur_send_data_toggle;
  reg  data_toggle_cur_rcv_data_toggle;
  reg `PidType_staticEncoding_type tx_fsm_pid;
  reg  tx_fsm_setup;
  reg `PidType_staticEncoding_type tx_fsm_cur_pid;
  reg  tx_fsm_cur_setup;
  reg `TxState_defaultEncoding_type tx_fsm_tx_state;
  wire [10:0] tx_fsm_frame_cntr;
  reg  tx_fsm_rd_req;
  reg [8:0] tx_fsm_rd_ptr;
  reg [5:0] tx_fsm_data_cntr;
  reg [15:0] tx_fsm_crc16;
  reg [7:0] _zz_UsbHost_7_;
  reg [15:0] tx_fsm_crc16_nxt;
  reg [4:0] tx_fsm_crc5;
  wire [10:0] _zz_UsbHost_8_;
  reg [10:0] _zz_UsbHost_9_;
  wire [4:0] _zz_UsbHost_10_;
  reg [4:0] _zz_UsbHost_11_;
  reg `TopState_defaultEncoding_type top_fsm_top_state;
  reg `HostXferResult_defaultEncoding_type top_fsm_xfer_result;
  `ifndef SYNTHESIS
  reg [63:0] io_xfer_type_payload_string;
  reg [63:0] io_xfer_result_string;
  reg [55:0] tx_fsm_pid_string;
  reg [55:0] tx_fsm_cur_pid_string;
  reg [103:0] tx_fsm_tx_state_string;
  reg [143:0] top_fsm_top_state_string;
  reg [63:0] top_fsm_xfer_result_string;
  `endif

  reg [7:0] fifo_ram [0:263];
  assign _zz_UsbHost_15_ = (tx_fsm_tx_state == `TxState_defaultEncoding_Idle);
  assign _zz_UsbHost_16_ = (tx_fsm_tx_state == `TxState_defaultEncoding_Idle);
  assign _zz_UsbHost_17_ = ((6'b000000) <= tx_buf_cur_byte_count);
  assign _zz_UsbHost_18_ = {1'd0, tx_buf_setup_first_byte_ptr};
  assign _zz_UsbHost_19_ = tx_fsm_crc16[7];
  assign _zz_UsbHost_20_ = tx_fsm_crc16[8];
  always @ (posedge core_u_pano_core_io_ulpi_clk) begin
    if(io_cpu_fifo_bus_cmd_valid && io_cpu_fifo_bus_cmd_payload_write ) begin
      fifo_ram[io_cpu_fifo_bus_cmd_payload_address] <= _zz_UsbHost_2_;
    end
    if(io_cpu_fifo_bus_cmd_valid) begin
      _zz_UsbHost_13_ <= fifo_ram[io_cpu_fifo_bus_cmd_payload_address];
    end
    if(_zz_UsbHost_4_ && rxtx_ram_access_rx_wr_req ) begin
      fifo_ram[rxtx_ram_access_rxtx_addr] <= _zz_UsbHost_6_;
    end
    if(_zz_UsbHost_4_) begin
      _zz_UsbHost_14_ <= fifo_ram[rxtx_ram_access_rxtx_addr];
    end
  end

  `ifndef SYNTHESIS
  always @(*) begin
    case(io_xfer_type_payload)
      `HostXferType_staticEncoding_SETUP : io_xfer_type_payload_string = "SETUP   ";
      `HostXferType_staticEncoding_BULK_IN : io_xfer_type_payload_string = "BULK_IN ";
      `HostXferType_staticEncoding_BULK_OUT : io_xfer_type_payload_string = "BULK_OUT";
      `HostXferType_staticEncoding_HS_IN : io_xfer_type_payload_string = "HS_IN   ";
      `HostXferType_staticEncoding_HS_OUT : io_xfer_type_payload_string = "HS_OUT  ";
      `HostXferType_staticEncoding_ISO_IN : io_xfer_type_payload_string = "ISO_IN  ";
      `HostXferType_staticEncoding_ISO_OUT : io_xfer_type_payload_string = "ISO_OUT ";
      default : io_xfer_type_payload_string = "????????";
    endcase
  end
  always @(*) begin
    case(io_xfer_result)
      `HostXferResult_defaultEncoding_SUCCESS : io_xfer_result_string = "SUCCESS ";
      `HostXferResult_defaultEncoding_BUSY : io_xfer_result_string = "BUSY    ";
      `HostXferResult_defaultEncoding_BADREQ : io_xfer_result_string = "BADREQ  ";
      `HostXferResult_defaultEncoding_UNDEF : io_xfer_result_string = "UNDEF   ";
      `HostXferResult_defaultEncoding_NAK : io_xfer_result_string = "NAK     ";
      `HostXferResult_defaultEncoding_STALL : io_xfer_result_string = "STALL   ";
      `HostXferResult_defaultEncoding_TOGERR : io_xfer_result_string = "TOGERR  ";
      `HostXferResult_defaultEncoding_WRONGPID : io_xfer_result_string = "WRONGPID";
      `HostXferResult_defaultEncoding_BADBC : io_xfer_result_string = "BADBC   ";
      `HostXferResult_defaultEncoding_PIDERR : io_xfer_result_string = "PIDERR  ";
      `HostXferResult_defaultEncoding_PKTERR : io_xfer_result_string = "PKTERR  ";
      `HostXferResult_defaultEncoding_CRCERR : io_xfer_result_string = "CRCERR  ";
      `HostXferResult_defaultEncoding_KERR : io_xfer_result_string = "KERR    ";
      `HostXferResult_defaultEncoding_JERR : io_xfer_result_string = "JERR    ";
      `HostXferResult_defaultEncoding_TIMEOUT : io_xfer_result_string = "TIMEOUT ";
      `HostXferResult_defaultEncoding_BABBLE : io_xfer_result_string = "BABBLE  ";
      default : io_xfer_result_string = "????????";
    endcase
  end
  always @(*) begin
    case(tx_fsm_pid)
      `PidType_staticEncoding_NULL_1 : tx_fsm_pid_string = "NULL_1 ";
      `PidType_staticEncoding_OUT_1 : tx_fsm_pid_string = "OUT_1  ";
      `PidType_staticEncoding_IN_1 : tx_fsm_pid_string = "IN_1   ";
      `PidType_staticEncoding_SOF : tx_fsm_pid_string = "SOF    ";
      `PidType_staticEncoding_SETUP : tx_fsm_pid_string = "SETUP  ";
      `PidType_staticEncoding_DATA0 : tx_fsm_pid_string = "DATA0  ";
      `PidType_staticEncoding_DATA1 : tx_fsm_pid_string = "DATA1  ";
      `PidType_staticEncoding_DATA2 : tx_fsm_pid_string = "DATA2  ";
      `PidType_staticEncoding_MDATA : tx_fsm_pid_string = "MDATA  ";
      `PidType_staticEncoding_ACK : tx_fsm_pid_string = "ACK    ";
      `PidType_staticEncoding_NAK : tx_fsm_pid_string = "NAK    ";
      `PidType_staticEncoding_STALL : tx_fsm_pid_string = "STALL  ";
      `PidType_staticEncoding_NYET : tx_fsm_pid_string = "NYET   ";
      `PidType_staticEncoding_PRE_ERR : tx_fsm_pid_string = "PRE_ERR";
      `PidType_staticEncoding_SPLIT : tx_fsm_pid_string = "SPLIT  ";
      `PidType_staticEncoding_PING : tx_fsm_pid_string = "PING   ";
      default : tx_fsm_pid_string = "???????";
    endcase
  end
  always @(*) begin
    case(tx_fsm_cur_pid)
      `PidType_staticEncoding_NULL_1 : tx_fsm_cur_pid_string = "NULL_1 ";
      `PidType_staticEncoding_OUT_1 : tx_fsm_cur_pid_string = "OUT_1  ";
      `PidType_staticEncoding_IN_1 : tx_fsm_cur_pid_string = "IN_1   ";
      `PidType_staticEncoding_SOF : tx_fsm_cur_pid_string = "SOF    ";
      `PidType_staticEncoding_SETUP : tx_fsm_cur_pid_string = "SETUP  ";
      `PidType_staticEncoding_DATA0 : tx_fsm_cur_pid_string = "DATA0  ";
      `PidType_staticEncoding_DATA1 : tx_fsm_cur_pid_string = "DATA1  ";
      `PidType_staticEncoding_DATA2 : tx_fsm_cur_pid_string = "DATA2  ";
      `PidType_staticEncoding_MDATA : tx_fsm_cur_pid_string = "MDATA  ";
      `PidType_staticEncoding_ACK : tx_fsm_cur_pid_string = "ACK    ";
      `PidType_staticEncoding_NAK : tx_fsm_cur_pid_string = "NAK    ";
      `PidType_staticEncoding_STALL : tx_fsm_cur_pid_string = "STALL  ";
      `PidType_staticEncoding_NYET : tx_fsm_cur_pid_string = "NYET   ";
      `PidType_staticEncoding_PRE_ERR : tx_fsm_cur_pid_string = "PRE_ERR";
      `PidType_staticEncoding_SPLIT : tx_fsm_cur_pid_string = "SPLIT  ";
      `PidType_staticEncoding_PING : tx_fsm_cur_pid_string = "PING   ";
      default : tx_fsm_cur_pid_string = "???????";
    endcase
  end
  always @(*) begin
    case(tx_fsm_tx_state)
      `TxState_defaultEncoding_Idle : tx_fsm_tx_state_string = "Idle         ";
      `TxState_defaultEncoding_TokenPid : tx_fsm_tx_state_string = "TokenPid     ";
      `TxState_defaultEncoding_TokenAddr : tx_fsm_tx_state_string = "TokenAddr    ";
      `TxState_defaultEncoding_TokenEndpoint : tx_fsm_tx_state_string = "TokenEndpoint";
      `TxState_defaultEncoding_DataPid : tx_fsm_tx_state_string = "DataPid      ";
      `TxState_defaultEncoding_DataData : tx_fsm_tx_state_string = "DataData     ";
      `TxState_defaultEncoding_DataCRC0 : tx_fsm_tx_state_string = "DataCRC0     ";
      `TxState_defaultEncoding_DataCRC1 : tx_fsm_tx_state_string = "DataCRC1     ";
      `TxState_defaultEncoding_HandshakePid : tx_fsm_tx_state_string = "HandshakePid ";
      `TxState_defaultEncoding_SpecialPid : tx_fsm_tx_state_string = "SpecialPid   ";
      default : tx_fsm_tx_state_string = "?????????????";
    endcase
  end
  always @(*) begin
    case(top_fsm_top_state)
      `TopState_defaultEncoding_Idle : top_fsm_top_state_string = "Idle              ";
      `TopState_defaultEncoding_SetupSendToken : top_fsm_top_state_string = "SetupSendToken    ";
      `TopState_defaultEncoding_SetupSendData0 : top_fsm_top_state_string = "SetupSendData0    ";
      `TopState_defaultEncoding_SetupWaitHandshake : top_fsm_top_state_string = "SetupWaitHandshake";
      default : top_fsm_top_state_string = "??????????????????";
    endcase
  end
  always @(*) begin
    case(top_fsm_xfer_result)
      `HostXferResult_defaultEncoding_SUCCESS : top_fsm_xfer_result_string = "SUCCESS ";
      `HostXferResult_defaultEncoding_BUSY : top_fsm_xfer_result_string = "BUSY    ";
      `HostXferResult_defaultEncoding_BADREQ : top_fsm_xfer_result_string = "BADREQ  ";
      `HostXferResult_defaultEncoding_UNDEF : top_fsm_xfer_result_string = "UNDEF   ";
      `HostXferResult_defaultEncoding_NAK : top_fsm_xfer_result_string = "NAK     ";
      `HostXferResult_defaultEncoding_STALL : top_fsm_xfer_result_string = "STALL   ";
      `HostXferResult_defaultEncoding_TOGERR : top_fsm_xfer_result_string = "TOGERR  ";
      `HostXferResult_defaultEncoding_WRONGPID : top_fsm_xfer_result_string = "WRONGPID";
      `HostXferResult_defaultEncoding_BADBC : top_fsm_xfer_result_string = "BADBC   ";
      `HostXferResult_defaultEncoding_PIDERR : top_fsm_xfer_result_string = "PIDERR  ";
      `HostXferResult_defaultEncoding_PKTERR : top_fsm_xfer_result_string = "PKTERR  ";
      `HostXferResult_defaultEncoding_CRCERR : top_fsm_xfer_result_string = "CRCERR  ";
      `HostXferResult_defaultEncoding_KERR : top_fsm_xfer_result_string = "KERR    ";
      `HostXferResult_defaultEncoding_JERR : top_fsm_xfer_result_string = "JERR    ";
      `HostXferResult_defaultEncoding_TIMEOUT : top_fsm_xfer_result_string = "TIMEOUT ";
      `HostXferResult_defaultEncoding_BABBLE : top_fsm_xfer_result_string = "BABBLE  ";
      default : top_fsm_xfer_result_string = "????????";
    endcase
  end
  `endif

  assign io_cpu_fifo_bus_cmd_ready = 1'b1;
  assign _zz_UsbHost_1_ = 1'b1;
  assign _zz_UsbHost_2_ = io_cpu_fifo_bus_cmd_payload_data;
  assign io_cpu_fifo_bus_rsp_payload_data = _zz_UsbHost_13_;
  assign io_cpu_fifo_bus_rsp_valid = _zz_UsbHost_3_;
  assign rxtx_ram_access_rxtx_addr = (rxtx_ram_access_tx_rd_req ? rxtx_ram_access_tx_rd_addr : rxtx_ram_access_rx_wr_addr);
  assign _zz_UsbHost_4_ = (rxtx_ram_access_tx_rd_req || rxtx_ram_access_rx_wr_req);
  assign _zz_UsbHost_5_ = 1'b1;
  assign _zz_UsbHost_6_ = rxtx_ram_access_rx_wr_data;
  assign rxtx_ram_access_tx_rd_data = _zz_UsbHost_14_;
  assign tx_buf_cur_byte_count = ((tx_buf_cur_buf == (1'b0)) ? tx_buf_byte_count0 : tx_buf_byte_count1);
  assign tx_buf_cur_first_byte_ptr = {{(2'b01),tx_buf_cur_buf[0]},(6'b000000)};
  assign tx_buf_setup_first_byte_ptr = {(2'b10),(6'b000000)};
  assign io_send_buf_avail = ((! tx_buf_buf_primed[tx_buf_cur_buf]) || (! tx_buf_buf_primed[(~ tx_buf_cur_buf)]));
  assign io_send_buf_avail_nr = ((! tx_buf_buf_primed[tx_buf_cur_buf]) ? tx_buf_cur_buf : (~ tx_buf_cur_buf));
  assign data_toggle_toggle_send = 1'b0;
  assign data_toggle_toggle_rcv = 1'b0;
  assign io_cur_send_data_toggle = data_toggle_cur_send_data_toggle;
  assign io_cur_rcv_data_toggle = data_toggle_cur_rcv_data_toggle;
  always @ (*) begin
    tx_fsm_pid = `PidType_staticEncoding_NULL_1;
    tx_fsm_setup = 1'b0;
    case(top_fsm_top_state)
      `TopState_defaultEncoding_Idle : begin
      end
      `TopState_defaultEncoding_SetupSendToken : begin
        if(_zz_UsbHost_15_)begin
          tx_fsm_pid = `PidType_staticEncoding_SETUP;
        end
      end
      `TopState_defaultEncoding_SetupSendData0 : begin
        if(_zz_UsbHost_16_)begin
          tx_fsm_pid = `PidType_staticEncoding_DATA0;
          tx_fsm_setup = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    io_ulpi_tx_data_valid = 1'b0;
    io_ulpi_tx_data_payload = (8'b00000000);
    tx_fsm_rd_req = 1'b0;
    case(tx_fsm_tx_state)
      `TxState_defaultEncoding_Idle : begin
      end
      `TxState_defaultEncoding_TokenPid : begin
        io_ulpi_tx_data_valid = 1'b1;
        io_ulpi_tx_data_payload = {(4'b0100),tx_fsm_cur_pid};
      end
      `TxState_defaultEncoding_TokenAddr : begin
        io_ulpi_tx_data_valid = 1'b1;
        io_ulpi_tx_data_payload = {io_endpoint[0],io_periph_addr};
      end
      `TxState_defaultEncoding_TokenEndpoint : begin
        io_ulpi_tx_data_valid = 1'b1;
        io_ulpi_tx_data_payload = {tx_fsm_crc5,io_endpoint[3 : 1]};
      end
      `TxState_defaultEncoding_DataPid : begin
        io_ulpi_tx_data_valid = 1'b1;
        io_ulpi_tx_data_payload = {(4'b0100),tx_fsm_cur_pid};
        if(io_ulpi_tx_data_ready)begin
          if(_zz_UsbHost_17_)begin
            tx_fsm_rd_req = 1'b1;
          end
        end
      end
      `TxState_defaultEncoding_DataData : begin
        io_ulpi_tx_data_valid = 1'b1;
        io_ulpi_tx_data_payload = rxtx_ram_access_tx_rd_data;
        tx_fsm_rd_req = 1'b1;
      end
      `TxState_defaultEncoding_DataCRC0 : begin
        io_ulpi_tx_data_valid = 1'b1;
        io_ulpi_tx_data_payload = (~ tx_fsm_crc16[7 : 0]);
      end
      `TxState_defaultEncoding_DataCRC1 : begin
        io_ulpi_tx_data_valid = 1'b1;
        io_ulpi_tx_data_payload = tx_fsm_crc16[15 : 8];
      end
      `TxState_defaultEncoding_HandshakePid : begin
        io_ulpi_tx_data_valid = 1'b1;
        io_ulpi_tx_data_payload = {(4'b0100),tx_fsm_cur_pid};
      end
      default : begin
      end
    endcase
  end

  assign tx_fsm_frame_cntr = (11'b00000000000);
  assign rxtx_ram_access_tx_rd_req = tx_fsm_rd_req;
  assign rxtx_ram_access_tx_rd_addr = tx_fsm_rd_ptr;
  always @ (*) begin
    _zz_UsbHost_7_[0] = rxtx_ram_access_tx_rd_data[7];
    _zz_UsbHost_7_[1] = rxtx_ram_access_tx_rd_data[6];
    _zz_UsbHost_7_[2] = rxtx_ram_access_tx_rd_data[5];
    _zz_UsbHost_7_[3] = rxtx_ram_access_tx_rd_data[4];
    _zz_UsbHost_7_[4] = rxtx_ram_access_tx_rd_data[3];
    _zz_UsbHost_7_[5] = rxtx_ram_access_tx_rd_data[2];
    _zz_UsbHost_7_[6] = rxtx_ram_access_tx_rd_data[1];
    _zz_UsbHost_7_[7] = rxtx_ram_access_tx_rd_data[0];
  end

  always @ (*) begin
    tx_fsm_crc16_nxt[0] = (((((((((((((((tx_fsm_crc16[8] ^ tx_fsm_crc16[9]) ^ tx_fsm_crc16[10]) ^ tx_fsm_crc16[11]) ^ tx_fsm_crc16[12]) ^ tx_fsm_crc16[13]) ^ tx_fsm_crc16[14]) ^ tx_fsm_crc16[15]) ^ _zz_UsbHost_7_[0]) ^ _zz_UsbHost_7_[1]) ^ _zz_UsbHost_7_[2]) ^ _zz_UsbHost_7_[3]) ^ _zz_UsbHost_7_[4]) ^ _zz_UsbHost_7_[5]) ^ _zz_UsbHost_7_[6]) ^ _zz_UsbHost_7_[7]);
    tx_fsm_crc16_nxt[1] = (((((((((((((tx_fsm_crc16[9] ^ tx_fsm_crc16[10]) ^ tx_fsm_crc16[11]) ^ tx_fsm_crc16[12]) ^ tx_fsm_crc16[13]) ^ tx_fsm_crc16[14]) ^ tx_fsm_crc16[15]) ^ _zz_UsbHost_7_[1]) ^ _zz_UsbHost_7_[2]) ^ _zz_UsbHost_7_[3]) ^ _zz_UsbHost_7_[4]) ^ _zz_UsbHost_7_[5]) ^ _zz_UsbHost_7_[6]) ^ _zz_UsbHost_7_[7]);
    tx_fsm_crc16_nxt[2] = (((tx_fsm_crc16[8] ^ tx_fsm_crc16[9]) ^ _zz_UsbHost_7_[0]) ^ _zz_UsbHost_7_[1]);
    tx_fsm_crc16_nxt[3] = (((tx_fsm_crc16[9] ^ tx_fsm_crc16[10]) ^ _zz_UsbHost_7_[1]) ^ _zz_UsbHost_7_[2]);
    tx_fsm_crc16_nxt[4] = (((tx_fsm_crc16[10] ^ tx_fsm_crc16[11]) ^ _zz_UsbHost_7_[2]) ^ _zz_UsbHost_7_[3]);
    tx_fsm_crc16_nxt[5] = (((tx_fsm_crc16[11] ^ tx_fsm_crc16[12]) ^ _zz_UsbHost_7_[3]) ^ _zz_UsbHost_7_[4]);
    tx_fsm_crc16_nxt[6] = (((tx_fsm_crc16[12] ^ tx_fsm_crc16[13]) ^ _zz_UsbHost_7_[4]) ^ _zz_UsbHost_7_[5]);
    tx_fsm_crc16_nxt[7] = (((tx_fsm_crc16[13] ^ tx_fsm_crc16[14]) ^ _zz_UsbHost_7_[5]) ^ _zz_UsbHost_7_[6]);
    tx_fsm_crc16_nxt[8] = ((((tx_fsm_crc16[0] ^ tx_fsm_crc16[14]) ^ tx_fsm_crc16[15]) ^ _zz_UsbHost_7_[6]) ^ _zz_UsbHost_7_[7]);
    tx_fsm_crc16_nxt[9] = ((tx_fsm_crc16[1] ^ tx_fsm_crc16[15]) ^ _zz_UsbHost_7_[7]);
    tx_fsm_crc16_nxt[10] = tx_fsm_crc16[2];
    tx_fsm_crc16_nxt[11] = tx_fsm_crc16[3];
    tx_fsm_crc16_nxt[12] = tx_fsm_crc16[4];
    tx_fsm_crc16_nxt[13] = tx_fsm_crc16[5];
    tx_fsm_crc16_nxt[14] = tx_fsm_crc16[6];
    tx_fsm_crc16_nxt[15] = ((((((((((((((((_zz_UsbHost_19_ ^ _zz_UsbHost_20_) ^ tx_fsm_crc16[9]) ^ tx_fsm_crc16[10]) ^ tx_fsm_crc16[11]) ^ tx_fsm_crc16[12]) ^ tx_fsm_crc16[13]) ^ tx_fsm_crc16[14]) ^ tx_fsm_crc16[15]) ^ _zz_UsbHost_7_[0]) ^ _zz_UsbHost_7_[1]) ^ _zz_UsbHost_7_[2]) ^ _zz_UsbHost_7_[3]) ^ _zz_UsbHost_7_[4]) ^ _zz_UsbHost_7_[5]) ^ _zz_UsbHost_7_[6]) ^ _zz_UsbHost_7_[7]);
  end

  assign _zz_UsbHost_8_ = ((tx_fsm_cur_pid == `PidType_staticEncoding_SOF) ? tx_fsm_frame_cntr : {io_endpoint,io_periph_addr});
  always @ (*) begin
    _zz_UsbHost_9_[0] = _zz_UsbHost_8_[10];
    _zz_UsbHost_9_[1] = _zz_UsbHost_8_[9];
    _zz_UsbHost_9_[2] = _zz_UsbHost_8_[8];
    _zz_UsbHost_9_[3] = _zz_UsbHost_8_[7];
    _zz_UsbHost_9_[4] = _zz_UsbHost_8_[6];
    _zz_UsbHost_9_[5] = _zz_UsbHost_8_[5];
    _zz_UsbHost_9_[6] = _zz_UsbHost_8_[4];
    _zz_UsbHost_9_[7] = _zz_UsbHost_8_[3];
    _zz_UsbHost_9_[8] = _zz_UsbHost_8_[2];
    _zz_UsbHost_9_[9] = _zz_UsbHost_8_[1];
    _zz_UsbHost_9_[10] = _zz_UsbHost_8_[0];
  end

  assign _zz_UsbHost_10_ = (5'b11111);
  always @ (*) begin
    _zz_UsbHost_11_[0] = ((((((((_zz_UsbHost_10_[0] ^ _zz_UsbHost_10_[3]) ^ _zz_UsbHost_10_[4]) ^ _zz_UsbHost_9_[0]) ^ _zz_UsbHost_9_[3]) ^ _zz_UsbHost_9_[5]) ^ _zz_UsbHost_9_[6]) ^ _zz_UsbHost_9_[9]) ^ _zz_UsbHost_9_[10]);
    _zz_UsbHost_11_[1] = (((((((_zz_UsbHost_10_[0] ^ _zz_UsbHost_10_[1]) ^ _zz_UsbHost_10_[4]) ^ _zz_UsbHost_9_[1]) ^ _zz_UsbHost_9_[4]) ^ _zz_UsbHost_9_[6]) ^ _zz_UsbHost_9_[7]) ^ _zz_UsbHost_9_[10]);
    _zz_UsbHost_11_[2] = ((((((((((((_zz_UsbHost_10_[0] ^ _zz_UsbHost_10_[1]) ^ _zz_UsbHost_10_[2]) ^ _zz_UsbHost_10_[3]) ^ _zz_UsbHost_10_[4]) ^ _zz_UsbHost_9_[0]) ^ _zz_UsbHost_9_[2]) ^ _zz_UsbHost_9_[3]) ^ _zz_UsbHost_9_[6]) ^ _zz_UsbHost_9_[7]) ^ _zz_UsbHost_9_[8]) ^ _zz_UsbHost_9_[9]) ^ _zz_UsbHost_9_[10]);
    _zz_UsbHost_11_[3] = ((((((((((_zz_UsbHost_10_[1] ^ _zz_UsbHost_10_[2]) ^ _zz_UsbHost_10_[3]) ^ _zz_UsbHost_10_[4]) ^ _zz_UsbHost_9_[1]) ^ _zz_UsbHost_9_[3]) ^ _zz_UsbHost_9_[4]) ^ _zz_UsbHost_9_[7]) ^ _zz_UsbHost_9_[8]) ^ _zz_UsbHost_9_[9]) ^ _zz_UsbHost_9_[10]);
    _zz_UsbHost_11_[4] = ((((((((_zz_UsbHost_10_[2] ^ _zz_UsbHost_10_[3]) ^ _zz_UsbHost_10_[4]) ^ _zz_UsbHost_9_[2]) ^ _zz_UsbHost_9_[4]) ^ _zz_UsbHost_9_[5]) ^ _zz_UsbHost_9_[8]) ^ _zz_UsbHost_9_[9]) ^ _zz_UsbHost_9_[10]);
  end

  assign rxtx_ram_access_rx_wr_req = 1'b0;
  assign rxtx_ram_access_rx_wr_addr = (9'b000000000);
  assign rxtx_ram_access_rx_wr_data = (8'b00000000);
  assign io_xfer_result = top_fsm_xfer_result;
  always @ (posedge core_u_pano_core_io_ulpi_clk) begin
    if(!_zz_UsbHost_12_) begin
      _zz_UsbHost_3_ <= 1'b0;
      tx_buf_cur_buf <= (1'b0);
      tx_buf_buf_primed <= (2'b00);
      tx_buf_byte_count0 <= (6'b000000);
      tx_buf_byte_count1 <= (6'b000000);
      data_toggle_cur_send_data_toggle <= 1'b0;
      data_toggle_cur_rcv_data_toggle <= 1'b0;
      tx_fsm_cur_pid <= `PidType_staticEncoding_NULL_1;
      tx_fsm_cur_setup <= 1'b0;
      tx_fsm_tx_state <= `TxState_defaultEncoding_Idle;
      tx_fsm_rd_ptr <= (9'b000000000);
      tx_fsm_data_cntr <= (6'b000000);
      tx_fsm_crc16 <= (16'b0000000000000000);
      tx_fsm_crc5 <= (5'b00000);
      top_fsm_top_state <= `TopState_defaultEncoding_Idle;
      top_fsm_xfer_result <= `HostXferResult_defaultEncoding_SUCCESS;
    end else begin
      _zz_UsbHost_3_ <= (io_cpu_fifo_bus_cmd_valid && (! io_cpu_fifo_bus_cmd_payload_write));
      tx_buf_cur_buf <= (1'b0);
      if(io_send_byte_count_valid)begin
        if((tx_buf_cur_buf == (1'b0)))begin
          if((! tx_buf_buf_primed[0]))begin
            tx_buf_byte_count0 <= io_send_byte_count_payload;
            tx_buf_buf_primed[0] <= 1'b1;
          end else begin
            if((! tx_buf_buf_primed[1]))begin
              tx_buf_byte_count1 <= io_send_byte_count_payload;
              tx_buf_buf_primed[1] <= 1'b1;
            end else begin
              tx_buf_byte_count1 <= io_send_byte_count_payload;
              tx_buf_buf_primed[1] <= 1'b1;
            end
          end
        end else begin
          if((tx_buf_cur_buf == (1'b1)))begin
            if((! tx_buf_buf_primed[1]))begin
              tx_buf_byte_count1 <= io_send_byte_count_payload;
              tx_buf_buf_primed[1] <= 1'b1;
            end else begin
              if((! tx_buf_buf_primed[0]))begin
                tx_buf_byte_count0 <= io_send_byte_count_payload;
                tx_buf_buf_primed[0] <= 1'b1;
              end else begin
                tx_buf_byte_count0 <= io_send_byte_count_payload;
                tx_buf_buf_primed[0] <= 1'b1;
              end
            end
          end
        end
      end
      if(io_set_send_data_toggle_valid)begin
        data_toggle_cur_send_data_toggle <= io_set_send_data_toggle_payload;
      end
      if(io_set_rcv_data_toggle_valid)begin
        data_toggle_cur_rcv_data_toggle <= io_set_rcv_data_toggle_payload;
      end
      if(data_toggle_toggle_send)begin
        data_toggle_cur_send_data_toggle <= (! data_toggle_cur_send_data_toggle);
      end
      if(data_toggle_toggle_rcv)begin
        data_toggle_cur_rcv_data_toggle <= (! data_toggle_cur_rcv_data_toggle);
      end
      tx_fsm_crc5 <= (~ _zz_UsbHost_11_);
      case(tx_fsm_tx_state)
        `TxState_defaultEncoding_Idle : begin
          if((tx_fsm_pid != `PidType_staticEncoding_NULL_1))begin
            tx_fsm_cur_pid <= tx_fsm_pid;
            tx_fsm_cur_setup <= tx_fsm_setup;
          end
          case(tx_fsm_pid)
            `PidType_staticEncoding_NULL_1 : begin
            end
            `PidType_staticEncoding_OUT_1, `PidType_staticEncoding_IN_1, `PidType_staticEncoding_SOF, `PidType_staticEncoding_SETUP : begin
              tx_fsm_tx_state <= `TxState_defaultEncoding_TokenPid;
            end
            `PidType_staticEncoding_DATA0, `PidType_staticEncoding_DATA1, `PidType_staticEncoding_DATA2, `PidType_staticEncoding_MDATA : begin
              tx_fsm_tx_state <= `TxState_defaultEncoding_DataPid;
              tx_fsm_rd_ptr <= (tx_fsm_setup ? _zz_UsbHost_18_ : tx_buf_cur_first_byte_ptr);
            end
            `PidType_staticEncoding_ACK, `PidType_staticEncoding_NAK, `PidType_staticEncoding_STALL, `PidType_staticEncoding_NYET : begin
              tx_fsm_tx_state <= `TxState_defaultEncoding_HandshakePid;
            end
            default : begin
              tx_fsm_tx_state <= `TxState_defaultEncoding_Idle;
            end
          endcase
        end
        `TxState_defaultEncoding_TokenPid : begin
          if(io_ulpi_tx_data_ready)begin
            tx_fsm_tx_state <= `TxState_defaultEncoding_TokenAddr;
          end
        end
        `TxState_defaultEncoding_TokenAddr : begin
          if(io_ulpi_tx_data_ready)begin
            tx_fsm_tx_state <= `TxState_defaultEncoding_TokenEndpoint;
          end
        end
        `TxState_defaultEncoding_TokenEndpoint : begin
          if(io_ulpi_tx_data_ready)begin
            tx_fsm_tx_state <= `TxState_defaultEncoding_Idle;
          end
        end
        `TxState_defaultEncoding_DataPid : begin
          if(io_ulpi_tx_data_ready)begin
            tx_fsm_crc16 <= (16'b1111111111111111);
            if(_zz_UsbHost_17_)begin
              tx_fsm_tx_state <= `TxState_defaultEncoding_DataData;
              tx_fsm_data_cntr <= (tx_fsm_cur_setup ? (6'b001000) : tx_buf_cur_byte_count);
            end else begin
              tx_fsm_tx_state <= `TxState_defaultEncoding_DataCRC0;
            end
          end
        end
        `TxState_defaultEncoding_DataData : begin
          if(io_ulpi_tx_data_ready)begin
            tx_fsm_crc16 <= tx_fsm_crc16_nxt;
            if(((6'b000001) < tx_fsm_data_cntr))begin
              tx_fsm_tx_state <= `TxState_defaultEncoding_DataData;
              tx_fsm_data_cntr <= (tx_fsm_data_cntr - (6'b000001));
              tx_fsm_rd_ptr <= (tx_fsm_rd_ptr + (9'b000000001));
            end else begin
              tx_fsm_tx_state <= `TxState_defaultEncoding_DataCRC0;
            end
          end
        end
        `TxState_defaultEncoding_DataCRC0 : begin
          if(io_ulpi_tx_data_ready)begin
            tx_fsm_tx_state <= `TxState_defaultEncoding_DataCRC1;
          end
        end
        `TxState_defaultEncoding_DataCRC1 : begin
          if(io_ulpi_tx_data_ready)begin
            tx_fsm_tx_state <= `TxState_defaultEncoding_Idle;
          end
        end
        `TxState_defaultEncoding_HandshakePid : begin
          if(io_ulpi_tx_data_ready)begin
            tx_fsm_tx_state <= `TxState_defaultEncoding_Idle;
          end
        end
        default : begin
        end
      endcase
      top_fsm_xfer_result <= `HostXferResult_defaultEncoding_SUCCESS;
      case(top_fsm_top_state)
        `TopState_defaultEncoding_Idle : begin
          if(io_xfer_type_valid)begin
            case(io_xfer_type_payload)
              `HostXferType_staticEncoding_SETUP : begin
              end
              default : begin
                top_fsm_top_state <= `TopState_defaultEncoding_Idle;
              end
            endcase
          end
        end
        `TopState_defaultEncoding_SetupSendToken : begin
          if(_zz_UsbHost_15_)begin
            top_fsm_top_state <= `TopState_defaultEncoding_SetupSendData0;
          end
        end
        `TopState_defaultEncoding_SetupSendData0 : begin
          if(_zz_UsbHost_16_)begin
            top_fsm_top_state <= `TopState_defaultEncoding_SetupWaitHandshake;
          end
        end
        default : begin
        end
      endcase
    end
  end

endmodule

module Apb3Gpio (
      input  [3:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output reg [31:0] io_apb_PRDATA,
      output  io_apb_PSLVERROR,
      input  [2:0] io_gpio_read,
      output [2:0] io_gpio_write,
      output [2:0] io_gpio_writeEnable,
      output [2:0] io_value,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire [2:0] bufferCC_19__io_dataOut;
  wire  ctrl_askWrite;
  wire  ctrl_askRead;
  wire  ctrl_doWrite;
  wire  ctrl_doRead;
  reg [2:0] io_gpio_write__driver;
  reg [2:0] io_gpio_writeEnable__driver;
  BufferCC_17_ bufferCC_19_ ( 
    .io_dataIn(io_gpio_read),
    .io_dataOut(bufferCC_19__io_dataOut),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  assign io_value = bufferCC_19__io_dataOut;
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = (32'b00000000000000000000000000000000);
    case(io_apb_PADDR)
      4'b0000 : begin
        io_apb_PRDATA[2 : 0] = io_value;
      end
      4'b0100 : begin
        io_apb_PRDATA[2 : 0] = io_gpio_write__driver;
      end
      4'b1000 : begin
        io_apb_PRDATA[2 : 0] = io_gpio_writeEnable__driver;
      end
      default : begin
      end
    endcase
  end

  assign io_apb_PSLVERROR = 1'b0;
  assign ctrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign ctrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign ctrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign ctrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  assign io_gpio_write = io_gpio_write__driver;
  assign io_gpio_writeEnable = io_gpio_writeEnable__driver;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      io_gpio_writeEnable__driver <= (3'b000);
    end else begin
      case(io_apb_PADDR)
        4'b0000 : begin
        end
        4'b0100 : begin
        end
        4'b1000 : begin
          if(ctrl_doWrite)begin
            io_gpio_writeEnable__driver <= io_apb_PWDATA[2 : 0];
          end
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge toplevel_main_clk) begin
    case(io_apb_PADDR)
      4'b0000 : begin
      end
      4'b0100 : begin
        if(ctrl_doWrite)begin
          io_gpio_write__driver <= io_apb_PWDATA[2 : 0];
        end
      end
      4'b1000 : begin
      end
      default : begin
      end
    endcase
  end

endmodule

module CCGpio (
      input  [4:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output reg [31:0] io_apb_PRDATA,
      output  io_apb_PSLVERROR,
      input  [1:0] io_gpio_read,
      output [1:0] io_gpio_write,
      output [1:0] io_gpio_writeEnable,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  reg [1:0] value;
  wire  ctrl_askWrite;
  wire  ctrl_askRead;
  wire  ctrl_doWrite;
  wire  ctrl_doRead;
  reg [1:0] _zz_CCGpio_1_;
  wire [1:0] wrBits;
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = (32'b00000000000000000000000000000000);
    case(io_apb_PADDR)
      5'b00000 : begin
        io_apb_PRDATA[1 : 0] = _zz_CCGpio_1_;
      end
      5'b00100 : begin
        io_apb_PRDATA[1 : 0] = value;
      end
      5'b01000 : begin
      end
      5'b01100 : begin
      end
      5'b10000 : begin
        io_apb_PRDATA[1 : 0] = io_gpio_read;
      end
      default : begin
      end
    endcase
  end

  assign io_apb_PSLVERROR = 1'b0;
  assign ctrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign ctrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign ctrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign ctrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  assign io_gpio_writeEnable = _zz_CCGpio_1_;
  assign io_gpio_write = value;
  assign wrBits = io_apb_PWDATA[1 : 0];
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      value <= (2'b00);
      _zz_CCGpio_1_ <= (2'b00);
    end else begin
      case(io_apb_PADDR)
        5'b00000 : begin
          if(ctrl_doWrite)begin
            _zz_CCGpio_1_ <= io_apb_PWDATA[1 : 0];
          end
        end
        5'b00100 : begin
          if(ctrl_doWrite)begin
            value <= io_apb_PWDATA[1 : 0];
          end
        end
        5'b01000 : begin
          if(ctrl_doWrite)begin
            if(wrBits[0])begin
              value[0] <= 1'b1;
            end
            if(wrBits[1])begin
              value[1] <= 1'b1;
            end
          end
        end
        5'b01100 : begin
          if(ctrl_doWrite)begin
            if(wrBits[0])begin
              value[0] <= 1'b0;
            end
            if(wrBits[1])begin
              value[1] <= 1'b0;
            end
          end
        end
        5'b10000 : begin
        end
        default : begin
        end
      endcase
    end
  end

endmodule

module Apb3SpiMasterCtrl (
      input  [7:0] io_apb_PADDR,
      input  [0:0] io_apb_PSEL,
      input   io_apb_PENABLE,
      output  io_apb_PREADY,
      input   io_apb_PWRITE,
      input  [31:0] io_apb_PWDATA,
      output reg [31:0] io_apb_PRDATA,
      output [0:0] io_spi_ss,
      output  io_spi_sclk,
      output  io_spi_mosi,
      input   io_spi_miso,
      output  io_interrupt,
      input   toplevel_main_clk,
      input   toplevel_main_reset_);
  wire  _zz_Apb3SpiMasterCtrl_11_;
  reg  _zz_Apb3SpiMasterCtrl_12_;
  wire  _zz_Apb3SpiMasterCtrl_13_;
  wire  spiCtrl_io_cmd_ready;
  wire  spiCtrl_io_rsp_valid;
  wire [7:0] spiCtrl_io_rsp_payload;
  wire  spiCtrl_io_spi_sclk;
  wire  spiCtrl_io_spi_mosi;
  wire [0:0] spiCtrl_io_spi_ss;
  wire  bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_push_ready;
  wire  bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_valid;
  wire `SpiMasterCtrlCmdMode_defaultEncoding_type bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_mode;
  wire [8:0] bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_args;
  wire [5:0] bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_occupancy;
  wire [5:0] bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_availability;
  wire  spiCtrl_io_rsp_queueWithOccupancy_io_push_ready;
  wire  spiCtrl_io_rsp_queueWithOccupancy_io_pop_valid;
  wire [7:0] spiCtrl_io_rsp_queueWithOccupancy_io_pop_payload;
  wire [5:0] spiCtrl_io_rsp_queueWithOccupancy_io_occupancy;
  wire [5:0] spiCtrl_io_rsp_queueWithOccupancy_io_availability;
  wire [0:0] _zz_Apb3SpiMasterCtrl_14_;
  wire [0:0] _zz_Apb3SpiMasterCtrl_15_;
  wire [0:0] _zz_Apb3SpiMasterCtrl_16_;
  wire [0:0] _zz_Apb3SpiMasterCtrl_17_;
  wire [0:0] _zz_Apb3SpiMasterCtrl_18_;
  wire [0:0] _zz_Apb3SpiMasterCtrl_19_;
  wire [0:0] _zz_Apb3SpiMasterCtrl_20_;
  wire  busCtrl_askWrite;
  wire  busCtrl_askRead;
  wire  busCtrl_doWrite;
  wire  busCtrl_doRead;
  wire  bridge_cmdLogic_streamUnbuffered_valid;
  wire  bridge_cmdLogic_streamUnbuffered_ready;
  wire `SpiMasterCtrlCmdMode_defaultEncoding_type bridge_cmdLogic_streamUnbuffered_payload_mode;
  reg [8:0] bridge_cmdLogic_streamUnbuffered_payload_args;
  reg  _zz_Apb3SpiMasterCtrl_1_;
  wire [7:0] bridge_cmdLogic_dataCmd_data;
  wire  bridge_cmdLogic_dataCmd_read;
  reg  bridge_interruptCtrl_cmdIntEnable;
  reg  bridge_interruptCtrl_rspIntEnable;
  wire  bridge_interruptCtrl_cmdInt;
  wire  bridge_interruptCtrl_rspInt;
  wire  bridge_interruptCtrl_interrupt;
  reg  _zz_Apb3SpiMasterCtrl_2_;
  reg  _zz_Apb3SpiMasterCtrl_3_;
  reg [15:0] _zz_Apb3SpiMasterCtrl_4_;
  reg [0:0] _zz_Apb3SpiMasterCtrl_5_;
  reg [15:0] _zz_Apb3SpiMasterCtrl_6_;
  reg [15:0] _zz_Apb3SpiMasterCtrl_7_;
  reg [15:0] _zz_Apb3SpiMasterCtrl_8_;
  wire `SpiMasterCtrlCmdMode_defaultEncoding_type _zz_Apb3SpiMasterCtrl_9_;
  wire [1:0] _zz_Apb3SpiMasterCtrl_10_;
  `ifndef SYNTHESIS
  reg [31:0] bridge_cmdLogic_streamUnbuffered_payload_mode_string;
  reg [31:0] _zz_Apb3SpiMasterCtrl_9__string;
  `endif

  assign _zz_Apb3SpiMasterCtrl_14_ = _zz_Apb3SpiMasterCtrl_15_[0];
  assign _zz_Apb3SpiMasterCtrl_15_ = io_apb_PWDATA[24 : 24];
  assign _zz_Apb3SpiMasterCtrl_16_ = io_apb_PWDATA[24 : 24];
  assign _zz_Apb3SpiMasterCtrl_17_ = io_apb_PWDATA[0 : 0];
  assign _zz_Apb3SpiMasterCtrl_18_ = io_apb_PWDATA[1 : 1];
  assign _zz_Apb3SpiMasterCtrl_19_ = _zz_Apb3SpiMasterCtrl_10_[0 : 0];
  assign _zz_Apb3SpiMasterCtrl_20_ = _zz_Apb3SpiMasterCtrl_10_[1 : 1];
  SpiMasterCtrl spiCtrl ( 
    .io_config_kind_cpol(_zz_Apb3SpiMasterCtrl_2_),
    .io_config_kind_cpha(_zz_Apb3SpiMasterCtrl_3_),
    .io_config_sclkToogle(_zz_Apb3SpiMasterCtrl_4_),
    .io_config_ss_activeHigh(_zz_Apb3SpiMasterCtrl_5_),
    .io_config_ss_setup(_zz_Apb3SpiMasterCtrl_6_),
    .io_config_ss_hold(_zz_Apb3SpiMasterCtrl_7_),
    .io_config_ss_disable(_zz_Apb3SpiMasterCtrl_8_),
    .io_cmd_valid(bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_valid),
    .io_cmd_ready(spiCtrl_io_cmd_ready),
    .io_cmd_payload_mode(bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_mode),
    .io_cmd_payload_args(bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_args),
    .io_rsp_valid(spiCtrl_io_rsp_valid),
    .io_rsp_payload(spiCtrl_io_rsp_payload),
    .io_spi_ss(spiCtrl_io_spi_ss),
    .io_spi_sclk(spiCtrl_io_spi_sclk),
    .io_spi_mosi(spiCtrl_io_spi_mosi),
    .io_spi_miso(io_spi_miso),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  StreamFifo bridge_cmdLogic_streamUnbuffered_queueWithAvailability ( 
    .io_push_valid(bridge_cmdLogic_streamUnbuffered_valid),
    .io_push_ready(bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_push_ready),
    .io_push_payload_mode(bridge_cmdLogic_streamUnbuffered_payload_mode),
    .io_push_payload_args(bridge_cmdLogic_streamUnbuffered_payload_args),
    .io_pop_valid(bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_valid),
    .io_pop_ready(spiCtrl_io_cmd_ready),
    .io_pop_payload_mode(bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_mode),
    .io_pop_payload_args(bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_payload_args),
    .io_flush(_zz_Apb3SpiMasterCtrl_11_),
    .io_occupancy(bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_occupancy),
    .io_availability(bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_availability),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  StreamFifo_1_ spiCtrl_io_rsp_queueWithOccupancy ( 
    .io_push_valid(spiCtrl_io_rsp_valid),
    .io_push_ready(spiCtrl_io_rsp_queueWithOccupancy_io_push_ready),
    .io_push_payload(spiCtrl_io_rsp_payload),
    .io_pop_valid(spiCtrl_io_rsp_queueWithOccupancy_io_pop_valid),
    .io_pop_ready(_zz_Apb3SpiMasterCtrl_12_),
    .io_pop_payload(spiCtrl_io_rsp_queueWithOccupancy_io_pop_payload),
    .io_flush(_zz_Apb3SpiMasterCtrl_13_),
    .io_occupancy(spiCtrl_io_rsp_queueWithOccupancy_io_occupancy),
    .io_availability(spiCtrl_io_rsp_queueWithOccupancy_io_availability),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(bridge_cmdLogic_streamUnbuffered_payload_mode)
      `SpiMasterCtrlCmdMode_defaultEncoding_DATA : bridge_cmdLogic_streamUnbuffered_payload_mode_string = "DATA";
      `SpiMasterCtrlCmdMode_defaultEncoding_SS : bridge_cmdLogic_streamUnbuffered_payload_mode_string = "SS  ";
      default : bridge_cmdLogic_streamUnbuffered_payload_mode_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_Apb3SpiMasterCtrl_9_)
      `SpiMasterCtrlCmdMode_defaultEncoding_DATA : _zz_Apb3SpiMasterCtrl_9__string = "DATA";
      `SpiMasterCtrlCmdMode_defaultEncoding_SS : _zz_Apb3SpiMasterCtrl_9__string = "SS  ";
      default : _zz_Apb3SpiMasterCtrl_9__string = "????";
    endcase
  end
  `endif

  assign io_spi_ss = spiCtrl_io_spi_ss;
  assign io_spi_sclk = spiCtrl_io_spi_sclk;
  assign io_spi_mosi = spiCtrl_io_spi_mosi;
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = (32'b00000000000000000000000000000000);
    _zz_Apb3SpiMasterCtrl_1_ = 1'b0;
    _zz_Apb3SpiMasterCtrl_12_ = 1'b0;
    case(io_apb_PADDR)
      8'b00000000 : begin
        if(busCtrl_doWrite)begin
          _zz_Apb3SpiMasterCtrl_1_ = 1'b1;
        end
        if(busCtrl_doRead)begin
          _zz_Apb3SpiMasterCtrl_12_ = 1'b1;
        end
        io_apb_PRDATA[31 : 31] = spiCtrl_io_rsp_queueWithOccupancy_io_pop_valid;
        io_apb_PRDATA[7 : 0] = spiCtrl_io_rsp_queueWithOccupancy_io_pop_payload;
        io_apb_PRDATA[21 : 16] = spiCtrl_io_rsp_queueWithOccupancy_io_occupancy;
      end
      8'b00000100 : begin
        io_apb_PRDATA[21 : 16] = bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_availability;
        io_apb_PRDATA[0 : 0] = bridge_interruptCtrl_cmdIntEnable;
        io_apb_PRDATA[1 : 1] = bridge_interruptCtrl_rspIntEnable;
        io_apb_PRDATA[8 : 8] = bridge_interruptCtrl_cmdInt;
        io_apb_PRDATA[9 : 9] = bridge_interruptCtrl_rspInt;
      end
      8'b00001000 : begin
      end
      8'b00001100 : begin
      end
      8'b00010000 : begin
      end
      8'b00010100 : begin
      end
      8'b00011000 : begin
      end
      default : begin
      end
    endcase
  end

  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  assign bridge_cmdLogic_streamUnbuffered_valid = _zz_Apb3SpiMasterCtrl_1_;
  always @ (*) begin
    case(bridge_cmdLogic_streamUnbuffered_payload_mode)
      `SpiMasterCtrlCmdMode_defaultEncoding_DATA : begin
        bridge_cmdLogic_streamUnbuffered_payload_args = {bridge_cmdLogic_dataCmd_read,bridge_cmdLogic_dataCmd_data};
      end
      default : begin
        bridge_cmdLogic_streamUnbuffered_payload_args = {8'd0, _zz_Apb3SpiMasterCtrl_14_};
      end
    endcase
  end

  assign bridge_cmdLogic_streamUnbuffered_ready = bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_push_ready;
  assign bridge_interruptCtrl_cmdInt = (bridge_interruptCtrl_cmdIntEnable && (! bridge_cmdLogic_streamUnbuffered_queueWithAvailability_io_pop_valid));
  assign bridge_interruptCtrl_rspInt = (bridge_interruptCtrl_rspIntEnable && spiCtrl_io_rsp_queueWithOccupancy_io_pop_valid);
  assign bridge_interruptCtrl_interrupt = (bridge_interruptCtrl_rspInt || bridge_interruptCtrl_cmdInt);
  assign io_interrupt = bridge_interruptCtrl_interrupt;
  assign bridge_cmdLogic_dataCmd_data = io_apb_PWDATA[7 : 0];
  assign bridge_cmdLogic_dataCmd_read = _zz_Apb3SpiMasterCtrl_16_[0];
  assign _zz_Apb3SpiMasterCtrl_9_ = io_apb_PWDATA[28 : 28];
  assign bridge_cmdLogic_streamUnbuffered_payload_mode = _zz_Apb3SpiMasterCtrl_9_;
  assign _zz_Apb3SpiMasterCtrl_10_ = io_apb_PWDATA[1 : 0];
  assign _zz_Apb3SpiMasterCtrl_11_ = 1'b0;
  assign _zz_Apb3SpiMasterCtrl_13_ = 1'b0;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      bridge_interruptCtrl_cmdIntEnable <= 1'b0;
      bridge_interruptCtrl_rspIntEnable <= 1'b0;
      _zz_Apb3SpiMasterCtrl_5_ <= (1'b0);
    end else begin
      case(io_apb_PADDR)
        8'b00000000 : begin
        end
        8'b00000100 : begin
          if(busCtrl_doWrite)begin
            bridge_interruptCtrl_cmdIntEnable <= _zz_Apb3SpiMasterCtrl_17_[0];
            bridge_interruptCtrl_rspIntEnable <= _zz_Apb3SpiMasterCtrl_18_[0];
          end
        end
        8'b00001000 : begin
          if(busCtrl_doWrite)begin
            _zz_Apb3SpiMasterCtrl_5_ <= io_apb_PWDATA[4 : 4];
          end
        end
        8'b00001100 : begin
        end
        8'b00010000 : begin
        end
        8'b00010100 : begin
        end
        8'b00011000 : begin
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge toplevel_main_clk) begin
    case(io_apb_PADDR)
      8'b00000000 : begin
      end
      8'b00000100 : begin
      end
      8'b00001000 : begin
        if(busCtrl_doWrite)begin
          _zz_Apb3SpiMasterCtrl_2_ <= _zz_Apb3SpiMasterCtrl_19_[0];
          _zz_Apb3SpiMasterCtrl_3_ <= _zz_Apb3SpiMasterCtrl_20_[0];
        end
      end
      8'b00001100 : begin
        if(busCtrl_doWrite)begin
          _zz_Apb3SpiMasterCtrl_4_ <= io_apb_PWDATA[15 : 0];
        end
      end
      8'b00010000 : begin
        if(busCtrl_doWrite)begin
          _zz_Apb3SpiMasterCtrl_6_ <= io_apb_PWDATA[15 : 0];
        end
      end
      8'b00010100 : begin
        if(busCtrl_doWrite)begin
          _zz_Apb3SpiMasterCtrl_7_ <= io_apb_PWDATA[15 : 0];
        end
      end
      8'b00011000 : begin
        if(busCtrl_doWrite)begin
          _zz_Apb3SpiMasterCtrl_8_ <= io_apb_PWDATA[15 : 0];
        end
      end
      default : begin
      end
    endcase
  end

endmodule

module PanoCore (
      output  io_led_red,
      output  io_led_green,
      output  io_led_blue,
      input   io_switch_,
      input   io_dvi_ctrl_scl_read,
      output  io_dvi_ctrl_scl_write,
      output  io_dvi_ctrl_scl_writeEnable,
      input   io_dvi_ctrl_sda_read,
      output  io_dvi_ctrl_sda_write,
      output  io_dvi_ctrl_sda_writeEnable,
      input   io_gmii_rx_clk,
      input   io_gmii_rx_dv,
      input   io_gmii_rx_er,
      input  [7:0] io_gmii_rx_d,
      input   io_gmii_tx_gclk,
      input   io_gmii_tx_clk,
      output  io_gmii_tx_en,
      output  io_gmii_tx_er,
      output [7:0] io_gmii_tx_d,
      input   io_gmii_col,
      input   io_gmii_crs,
      output  io_gmii_mdio_mdc,
      input   io_gmii_mdio_mdio_read,
      output  io_gmii_mdio_mdio_write,
      output  io_gmii_mdio_mdio_writeEnable,
      input   io_ulpi_clk,
      input  [7:0] io_ulpi_data_read,
      output [7:0] io_ulpi_data_write,
      output [7:0] io_ulpi_data_writeEnable,
      input   io_ulpi_direction,
      output  io_ulpi_stp,
      input   io_ulpi_nxt,
      output  io_ulpi_reset,
      output  io_vo_vsync,
      output  io_vo_hsync,
      output  io_vo_blank_,
      output  io_vo_de,
      output [7:0] io_vo_r,
      output [7:0] io_vo_g,
      output [7:0] io_vo_b,
      output  io_axi1_aw_valid,
      input   io_axi1_aw_ready,
      output [31:0] io_axi1_aw_payload_addr,
      output [3:0] io_axi1_aw_payload_id,
      output [3:0] io_axi1_aw_payload_region,
      output [7:0] io_axi1_aw_payload_len,
      output [2:0] io_axi1_aw_payload_size,
      output [1:0] io_axi1_aw_payload_burst,
      output [0:0] io_axi1_aw_payload_lock,
      output [3:0] io_axi1_aw_payload_cache,
      output [3:0] io_axi1_aw_payload_qos,
      output [2:0] io_axi1_aw_payload_prot,
      output  io_axi1_w_valid,
      input   io_axi1_w_ready,
      output [31:0] io_axi1_w_payload_data,
      output [3:0] io_axi1_w_payload_strb,
      output  io_axi1_w_payload_last,
      input   io_axi1_b_valid,
      output  io_axi1_b_ready,
      input  [3:0] io_axi1_b_payload_id,
      input  [1:0] io_axi1_b_payload_resp,
      output  io_axi1_ar_valid,
      input   io_axi1_ar_ready,
      output [31:0] io_axi1_ar_payload_addr,
      output [3:0] io_axi1_ar_payload_id,
      output [3:0] io_axi1_ar_payload_region,
      output [7:0] io_axi1_ar_payload_len,
      output [2:0] io_axi1_ar_payload_size,
      output [1:0] io_axi1_ar_payload_burst,
      output [0:0] io_axi1_ar_payload_lock,
      output [3:0] io_axi1_ar_payload_cache,
      output [3:0] io_axi1_ar_payload_qos,
      output [2:0] io_axi1_ar_payload_prot,
      input   io_axi1_r_valid,
      output  io_axi1_r_ready,
      input  [31:0] io_axi1_r_payload_data,
      input  [3:0] io_axi1_r_payload_id,
      input  [1:0] io_axi1_r_payload_resp,
      input   io_axi1_r_payload_last,
      output  io_axi2_aw_valid,
      input   io_axi2_aw_ready,
      output [31:0] io_axi2_aw_payload_addr,
      output [3:0] io_axi2_aw_payload_id,
      output [3:0] io_axi2_aw_payload_region,
      output [7:0] io_axi2_aw_payload_len,
      output [2:0] io_axi2_aw_payload_size,
      output [1:0] io_axi2_aw_payload_burst,
      output [0:0] io_axi2_aw_payload_lock,
      output [3:0] io_axi2_aw_payload_cache,
      output [3:0] io_axi2_aw_payload_qos,
      output [2:0] io_axi2_aw_payload_prot,
      output  io_axi2_w_valid,
      input   io_axi2_w_ready,
      output [31:0] io_axi2_w_payload_data,
      output [3:0] io_axi2_w_payload_strb,
      output  io_axi2_w_payload_last,
      input   io_axi2_b_valid,
      output  io_axi2_b_ready,
      input  [3:0] io_axi2_b_payload_id,
      input  [1:0] io_axi2_b_payload_resp,
      output  io_axi2_ar_valid,
      input   io_axi2_ar_ready,
      output [31:0] io_axi2_ar_payload_addr,
      output [3:0] io_axi2_ar_payload_id,
      output [3:0] io_axi2_ar_payload_region,
      output [7:0] io_axi2_ar_payload_len,
      output [2:0] io_axi2_ar_payload_size,
      output [1:0] io_axi2_ar_payload_burst,
      output [0:0] io_axi2_ar_payload_lock,
      output [3:0] io_axi2_ar_payload_cache,
      output [3:0] io_axi2_ar_payload_qos,
      output [2:0] io_axi2_ar_payload_prot,
      input   io_axi2_r_valid,
      output  io_axi2_r_ready,
      input  [31:0] io_axi2_r_payload_data,
      input  [3:0] io_axi2_r_payload_id,
      input  [1:0] io_axi2_r_payload_resp,
      input   io_axi2_r_payload_last,
      output [0:0] io_spi_ss,
      output  io_spi_sclk,
      output  io_spi_mosi,
      input   io_spi_miso,
      input   toplevel_main_clk,
      input   toplevel_main_reset_,
      input   toplevel_u_vo_clk_gen_vo_clk,
      input   toplevel_u_vo_clk_gen_vo_reset_);
  wire  _zz_PanoCore_41_;
  reg [31:0] _zz_PanoCore_42_;
  wire  _zz_PanoCore_43_;
  reg  _zz_PanoCore_44_;
  reg [31:0] _zz_PanoCore_45_;
  wire  _zz_PanoCore_46_;
  wire  _zz_PanoCore_47_;
  reg [31:0] _zz_PanoCore_48_;
  wire  _zz_PanoCore_49_;
  reg  _zz_PanoCore_50_;
  reg  _zz_PanoCore_51_;
  reg [12:0] _zz_PanoCore_52_;
  wire [7:0] _zz_PanoCore_53_;
  wire  _zz_PanoCore_54_;
  wire  _zz_PanoCore_55_;
  wire  _zz_PanoCore_56_;
  wire  _zz_PanoCore_57_;
  wire  _zz_PanoCore_58_;
  wire  _zz_PanoCore_59_;
  reg  _zz_PanoCore_60_;
  reg  _zz_PanoCore_61_;
  reg [8:0] _zz_PanoCore_62_;
  wire [7:0] _zz_PanoCore_63_;
  wire [0:0] _zz_PanoCore_64_;
  wire [5:0] _zz_PanoCore_65_;
  wire  _zz_PanoCore_66_;
  wire  _zz_PanoCore_67_;
  wire  _zz_PanoCore_68_;
  wire  _zz_PanoCore_69_;
  wire  _zz_PanoCore_70_;
  reg [2:0] _zz_PanoCore_71_;
  reg [1:0] _zz_PanoCore_72_;
  wire [3:0] u_cpu_top_io_led_ctrl_apb_PADDR;
  wire [0:0] u_cpu_top_io_led_ctrl_apb_PSEL;
  wire  u_cpu_top_io_led_ctrl_apb_PENABLE;
  wire  u_cpu_top_io_led_ctrl_apb_PWRITE;
  wire [31:0] u_cpu_top_io_led_ctrl_apb_PWDATA;
  wire [4:0] u_cpu_top_io_dvi_ctrl_apb_PADDR;
  wire [0:0] u_cpu_top_io_dvi_ctrl_apb_PSEL;
  wire  u_cpu_top_io_dvi_ctrl_apb_PENABLE;
  wire  u_cpu_top_io_dvi_ctrl_apb_PWRITE;
  wire [31:0] u_cpu_top_io_dvi_ctrl_apb_PWDATA;
  wire [4:0] u_cpu_top_io_gmii_ctrl_apb_PADDR;
  wire [0:0] u_cpu_top_io_gmii_ctrl_apb_PSEL;
  wire  u_cpu_top_io_gmii_ctrl_apb_PENABLE;
  wire  u_cpu_top_io_gmii_ctrl_apb_PWRITE;
  wire [31:0] u_cpu_top_io_gmii_ctrl_apb_PWDATA;
  wire [4:0] u_cpu_top_io_test_patt_apb_PADDR;
  wire [0:0] u_cpu_top_io_test_patt_apb_PSEL;
  wire  u_cpu_top_io_test_patt_apb_PENABLE;
  wire  u_cpu_top_io_test_patt_apb_PWRITE;
  wire [31:0] u_cpu_top_io_test_patt_apb_PWDATA;
  wire [15:0] u_cpu_top_io_txt_gen_apb_PADDR;
  wire [0:0] u_cpu_top_io_txt_gen_apb_PSEL;
  wire  u_cpu_top_io_txt_gen_apb_PENABLE;
  wire  u_cpu_top_io_txt_gen_apb_PWRITE;
  wire [31:0] u_cpu_top_io_txt_gen_apb_PWDATA;
  wire [5:0] u_cpu_top_io_ulpi_apb_PADDR;
  wire [0:0] u_cpu_top_io_ulpi_apb_PSEL;
  wire  u_cpu_top_io_ulpi_apb_PENABLE;
  wire  u_cpu_top_io_ulpi_apb_PWRITE;
  wire [31:0] u_cpu_top_io_ulpi_apb_PWDATA;
  wire [6:0] u_cpu_top_io_usb_host_apb_PADDR;
  wire [0:0] u_cpu_top_io_usb_host_apb_PSEL;
  wire  u_cpu_top_io_usb_host_apb_PENABLE;
  wire  u_cpu_top_io_usb_host_apb_PWRITE;
  wire [31:0] u_cpu_top_io_usb_host_apb_PWDATA;
  wire [7:0] u_cpu_top_io_spi_flash_ctrl_apb_PADDR;
  wire [0:0] u_cpu_top_io_spi_flash_ctrl_apb_PSEL;
  wire  u_cpu_top_io_spi_flash_ctrl_apb_PENABLE;
  wire  u_cpu_top_io_spi_flash_ctrl_apb_PWRITE;
  wire [31:0] u_cpu_top_io_spi_flash_ctrl_apb_PWDATA;
  wire  u_cpu_top_io_axi1_ar_valid;
  wire [31:0] u_cpu_top_io_axi1_ar_payload_addr;
  wire [3:0] u_cpu_top_io_axi1_ar_payload_id;
  wire [3:0] u_cpu_top_io_axi1_ar_payload_region;
  wire [7:0] u_cpu_top_io_axi1_ar_payload_len;
  wire [2:0] u_cpu_top_io_axi1_ar_payload_size;
  wire [1:0] u_cpu_top_io_axi1_ar_payload_burst;
  wire [0:0] u_cpu_top_io_axi1_ar_payload_lock;
  wire [3:0] u_cpu_top_io_axi1_ar_payload_cache;
  wire [3:0] u_cpu_top_io_axi1_ar_payload_qos;
  wire [2:0] u_cpu_top_io_axi1_ar_payload_prot;
  wire  u_cpu_top_io_axi1_aw_valid;
  wire [31:0] u_cpu_top_io_axi1_aw_payload_addr;
  wire [3:0] u_cpu_top_io_axi1_aw_payload_id;
  wire [3:0] u_cpu_top_io_axi1_aw_payload_region;
  wire [7:0] u_cpu_top_io_axi1_aw_payload_len;
  wire [2:0] u_cpu_top_io_axi1_aw_payload_size;
  wire [1:0] u_cpu_top_io_axi1_aw_payload_burst;
  wire [0:0] u_cpu_top_io_axi1_aw_payload_lock;
  wire [3:0] u_cpu_top_io_axi1_aw_payload_cache;
  wire [3:0] u_cpu_top_io_axi1_aw_payload_qos;
  wire [2:0] u_cpu_top_io_axi1_aw_payload_prot;
  wire  u_cpu_top_io_axi1_w_valid;
  wire [31:0] u_cpu_top_io_axi1_w_payload_data;
  wire [3:0] u_cpu_top_io_axi1_w_payload_strb;
  wire  u_cpu_top_io_axi1_w_payload_last;
  wire  u_cpu_top_io_axi1_r_ready;
  wire  u_cpu_top_io_axi1_b_ready;
  wire  u_cpu_top_io_axi2_ar_valid;
  wire [31:0] u_cpu_top_io_axi2_ar_payload_addr;
  wire [3:0] u_cpu_top_io_axi2_ar_payload_id;
  wire [3:0] u_cpu_top_io_axi2_ar_payload_region;
  wire [7:0] u_cpu_top_io_axi2_ar_payload_len;
  wire [2:0] u_cpu_top_io_axi2_ar_payload_size;
  wire [1:0] u_cpu_top_io_axi2_ar_payload_burst;
  wire [0:0] u_cpu_top_io_axi2_ar_payload_lock;
  wire [3:0] u_cpu_top_io_axi2_ar_payload_cache;
  wire [3:0] u_cpu_top_io_axi2_ar_payload_qos;
  wire [2:0] u_cpu_top_io_axi2_ar_payload_prot;
  wire  u_cpu_top_io_axi2_aw_valid;
  wire [31:0] u_cpu_top_io_axi2_aw_payload_addr;
  wire [3:0] u_cpu_top_io_axi2_aw_payload_id;
  wire [3:0] u_cpu_top_io_axi2_aw_payload_region;
  wire [7:0] u_cpu_top_io_axi2_aw_payload_len;
  wire [2:0] u_cpu_top_io_axi2_aw_payload_size;
  wire [1:0] u_cpu_top_io_axi2_aw_payload_burst;
  wire [0:0] u_cpu_top_io_axi2_aw_payload_lock;
  wire [3:0] u_cpu_top_io_axi2_aw_payload_cache;
  wire [3:0] u_cpu_top_io_axi2_aw_payload_qos;
  wire [2:0] u_cpu_top_io_axi2_aw_payload_prot;
  wire  u_cpu_top_io_axi2_w_valid;
  wire [31:0] u_cpu_top_io_axi2_w_payload_data;
  wire [3:0] u_cpu_top_io_axi2_w_payload_strb;
  wire  u_cpu_top_io_axi2_w_payload_last;
  wire  u_cpu_top_io_axi2_r_ready;
  wire  u_cpu_top_io_axi2_b_ready;
  wire  vo_area_u_vi_gen_io_pixel_out_vsync;
  wire  vo_area_u_vi_gen_io_pixel_out_req;
  wire  vo_area_u_vi_gen_io_pixel_out_last_col;
  wire  vo_area_u_vi_gen_io_pixel_out_last_line;
  wire [7:0] vo_area_u_vi_gen_io_pixel_out_pixel_r;
  wire [7:0] vo_area_u_vi_gen_io_pixel_out_pixel_g;
  wire [7:0] vo_area_u_vi_gen_io_pixel_out_pixel_b;
  wire  vo_area_u_test_patt_io_pixel_out_vsync;
  wire  vo_area_u_test_patt_io_pixel_out_req;
  wire  vo_area_u_test_patt_io_pixel_out_last_col;
  wire  vo_area_u_test_patt_io_pixel_out_last_line;
  wire [7:0] vo_area_u_test_patt_io_pixel_out_pixel_r;
  wire [7:0] vo_area_u_test_patt_io_pixel_out_pixel_g;
  wire [7:0] vo_area_u_test_patt_io_pixel_out_pixel_b;
  wire  vo_area_u_txt_gen_io_pixel_out_vsync;
  wire  vo_area_u_txt_gen_io_pixel_out_req;
  wire  vo_area_u_txt_gen_io_pixel_out_last_col;
  wire  vo_area_u_txt_gen_io_pixel_out_last_line;
  wire [7:0] vo_area_u_txt_gen_io_pixel_out_pixel_r;
  wire [7:0] vo_area_u_txt_gen_io_pixel_out_pixel_g;
  wire [7:0] vo_area_u_txt_gen_io_pixel_out_pixel_b;
  wire [7:0] vo_area_u_txt_gen_io_txt_buf_rd_data;
  wire  vo_area_u_vo_io_vga_out_vsync;
  wire  vo_area_u_vo_io_vga_out_hsync;
  wire  vo_area_u_vo_io_vga_out_blank_;
  wire  vo_area_u_vo_io_vga_out_de;
  wire [7:0] vo_area_u_vo_io_vga_out_r;
  wire [7:0] vo_area_u_vo_io_vga_out_g;
  wire [7:0] vo_area_u_vo_io_vga_out_b;
  wire  gmiiCtrl_1__io_apb_PREADY;
  wire [31:0] gmiiCtrl_1__io_apb_PRDATA;
  wire  gmiiCtrl_1__io_apb_PSLVERROR;
  wire  gmiiCtrl_1__io_gmii_tx_en;
  wire  gmiiCtrl_1__io_gmii_tx_er;
  wire [7:0] gmiiCtrl_1__io_gmii_tx_d;
  wire  gmiiCtrl_1__io_gmii_mdio_mdc;
  wire  gmiiCtrl_1__io_gmii_mdio_mdio_write;
  wire  gmiiCtrl_1__io_gmii_mdio_mdio_writeEnable;
  wire [7:0] ulpiCtrl_1__io_ulpi_data_write;
  wire [7:0] ulpiCtrl_1__io_ulpi_data_writeEnable;
  wire  ulpiCtrl_1__io_ulpi_stp;
  wire  ulpiCtrl_1__io_ulpi_reset;
  wire  ulpiCtrl_1__io_tx_data_ready;
  wire  ulpiCtrl_1__io_rx_data_valid;
  wire [8:0] ulpiCtrl_1__io_rx_data_payload;
  wire  ulpiCtrl_1__io_rx_cmd_changed;
  wire [7:0] ulpiCtrl_1__io_rx_cmd;
  wire [7:0] ulpiCtrl_1__io_reg_rd_data;
  wire  ulpiCtrl_1__io_reg_done;
  wire  ulpiCtrl_1__ulpi_reset__1_;
  wire  streamFifoCC_5__io_push_ready;
  wire  streamFifoCC_5__io_pop_valid;
  wire  streamFifoCC_5__io_pop_payload;
  wire [1:0] streamFifoCC_5__io_pushOccupancy;
  wire [1:0] streamFifoCC_5__io_popOccupancy;
  wire  pulseCCByToggle_4__io_pulseOut;
  wire  streamFifoCC_6__io_push_ready;
  wire  streamFifoCC_6__io_pop_valid;
  wire [7:0] streamFifoCC_6__io_pop_payload;
  wire [10:0] streamFifoCC_6__io_pushOccupancy;
  wire [10:0] streamFifoCC_6__io_popOccupancy;
  wire  pulseCCByToggle_5__io_pulseOut;
  wire  bufferCC_19__io_dataOut;
  wire  apb3CC_1__io_src_PREADY;
  wire [31:0] apb3CC_1__io_src_PRDATA;
  wire  apb3CC_1__io_src_PSLVERROR;
  wire [6:0] apb3CC_1__io_dest_PADDR;
  wire [0:0] apb3CC_1__io_dest_PSEL;
  wire  apb3CC_1__io_dest_PENABLE;
  wire  apb3CC_1__io_dest_PWRITE;
  wire [31:0] apb3CC_1__io_dest_PWDATA;
  wire  usbHost_1__io_cpu_fifo_bus_cmd_ready;
  wire  usbHost_1__io_cpu_fifo_bus_rsp_valid;
  wire [7:0] usbHost_1__io_cpu_fifo_bus_rsp_payload_data;
  wire  usbHost_1__io_send_buf_avail;
  wire [0:0] usbHost_1__io_send_buf_avail_nr;
  wire `HostXferResult_defaultEncoding_type usbHost_1__io_xfer_result;
  wire  usbHost_1__io_cur_send_data_toggle;
  wire  usbHost_1__io_cur_rcv_data_toggle;
  wire  usbHost_1__io_ulpi_rx_cmd_changed;
  wire [7:0] usbHost_1__io_ulpi_rx_cmd;
  wire  usbHost_1__io_ulpi_tx_data_valid;
  wire [7:0] usbHost_1__io_ulpi_tx_data_payload;
  wire  u_led_ctrl_io_apb_PREADY;
  wire [31:0] u_led_ctrl_io_apb_PRDATA;
  wire  u_led_ctrl_io_apb_PSLVERROR;
  wire [2:0] u_led_ctrl_io_gpio_write;
  wire [2:0] u_led_ctrl_io_gpio_writeEnable;
  wire [2:0] u_led_ctrl_io_value;
  wire  cCGpio_1__io_apb_PREADY;
  wire [31:0] cCGpio_1__io_apb_PRDATA;
  wire  cCGpio_1__io_apb_PSLVERROR;
  wire [1:0] cCGpio_1__io_gpio_write;
  wire [1:0] cCGpio_1__io_gpio_writeEnable;
  wire  u_spi_flash_io_apb_PREADY;
  wire [31:0] u_spi_flash_io_apb_PRDATA;
  wire  u_spi_flash_io_spi_sclk;
  wire  u_spi_flash_io_spi_mosi;
  wire [0:0] u_spi_flash_io_spi_ss;
  wire  u_spi_flash_io_interrupt;
  wire [15:0] _zz_PanoCore_73_;
  wire [14:0] _zz_PanoCore_74_;
  wire [15:0] _zz_PanoCore_75_;
  wire [14:0] _zz_PanoCore_76_;
  wire [0:0] _zz_PanoCore_77_;
  wire [15:0] _zz_PanoCore_78_;
  wire [0:0] _zz_PanoCore_79_;
  reg [23:0] leds_led_cntr;
  wire [23:0] _zz_PanoCore_1_;
  reg  _zz_PanoCore_2_ = 1'b0;
  wire [11:0] vo_area_timings_h_active;
  wire [8:0] vo_area_timings_h_fp;
  wire [8:0] vo_area_timings_h_sync;
  wire [8:0] vo_area_timings_h_bp;
  wire  vo_area_timings_h_sync_positive;
  wire [10:0] vo_area_timings_v_active;
  wire [8:0] vo_area_timings_v_fp;
  wire [8:0] vo_area_timings_v_sync;
  wire [8:0] vo_area_timings_v_bp;
  wire  vo_area_timings_v_sync_positive;
  wire  vo_area_vi_gen_pixel_out_vsync;
  wire  vo_area_vi_gen_pixel_out_req;
  wire  vo_area_vi_gen_pixel_out_last_col;
  wire  vo_area_vi_gen_pixel_out_last_line;
  wire [7:0] vo_area_vi_gen_pixel_out_pixel_r;
  wire [7:0] vo_area_vi_gen_pixel_out_pixel_g;
  wire [7:0] vo_area_vi_gen_pixel_out_pixel_b;
  wire  vo_area_test_patt_pixel_out_vsync;
  wire  vo_area_test_patt_pixel_out_req;
  wire  vo_area_test_patt_pixel_out_last_col;
  wire  vo_area_test_patt_pixel_out_last_line;
  wire [7:0] vo_area_test_patt_pixel_out_pixel_r;
  wire [7:0] vo_area_test_patt_pixel_out_pixel_g;
  wire [7:0] vo_area_test_patt_pixel_out_pixel_b;
  wire  vo_area_test_patt_ctrl_busCtrl_askWrite;
  wire  vo_area_test_patt_ctrl_busCtrl_askRead;
  wire  vo_area_test_patt_ctrl_busCtrl_doWrite;
  wire  vo_area_test_patt_ctrl_busCtrl_doRead;
  reg [3:0] vo_area_test_patt_ctrl_apb_regs_pattern_nr;
  reg [7:0] vo_area_test_patt_ctrl_apb_regs_const_color_r;
  reg [7:0] vo_area_test_patt_ctrl_apb_regs_const_color_g;
  reg [7:0] vo_area_test_patt_ctrl_apb_regs_const_color_b;
  wire  vo_area_txt_gen_pixel_out_vsync;
  wire  vo_area_txt_gen_pixel_out_req;
  wire  vo_area_txt_gen_pixel_out_last_col;
  wire  vo_area_txt_gen_pixel_out_last_line;
  wire [7:0] vo_area_txt_gen_pixel_out_pixel_r;
  wire [7:0] vo_area_txt_gen_pixel_out_pixel_g;
  wire [7:0] vo_area_txt_gen_pixel_out_pixel_b;
  wire  vo_area_txt_gen_ctrl_busCtrl_askWrite;
  wire  vo_area_txt_gen_ctrl_busCtrl_askRead;
  wire  vo_area_txt_gen_ctrl_busCtrl_doWrite;
  wire  vo_area_txt_gen_ctrl_busCtrl_doRead;
  wire [12:0] vo_area_txt_gen_ctrl_apb_regs_txt_buf_rd_addr;
  wire [12:0] vo_area_txt_gen_ctrl_apb_regs_txt_buf_wr_addr;
  reg  _zz_PanoCore_3_;
  reg [0:0] _zz_PanoCore_4_;
  reg [0:0] _zz_PanoCore_5_;
  wire  _zz_PanoCore_6_;
  wire  _zz_PanoCore_7_;
  wire  _zz_PanoCore_8_;
  reg [5:0] _zz_PanoCore_9_;
  reg [7:0] _zz_PanoCore_10_;
  reg  _zz_PanoCore_11_;
  wire  _zz_PanoCore_12_;
  wire  _zz_PanoCore_13_;
  reg  _zz_PanoCore_14_;
  reg  _zz_PanoCore_14__regNext;
  reg  _zz_PanoCore_15_;
  reg  _zz_PanoCore_16_;
  reg  _zz_PanoCore_17_;
  reg  _zz_PanoCore_18_;
  wire [31:0] _zz_PanoCore_19_;
  reg [31:0] _zz_PanoCore_20_;
  wire  _zz_PanoCore_21_;
  reg [6:0] _zz_PanoCore_22_;
  reg [5:0] _zz_PanoCore_23_;
  reg  _zz_PanoCore_24_;
  reg [2:0] _zz_PanoCore_25_;
  reg [8:0] _zz_PanoCore_26_;
  reg  _zz_PanoCore_27_;
  reg  _zz_PanoCore_28_;
  wire [1:0] _zz_PanoCore_29_;
  reg  _zz_PanoCore_30_;
  wire [1:0] _zz_PanoCore_31_;
  reg [3:0] _zz_PanoCore_32_;
  reg  _zz_PanoCore_33_;
  wire `HostXferType_staticEncoding_type _zz_PanoCore_34_;
  reg `HostXferResult_defaultEncoding_type _zz_PanoCore_35_;
  reg  _zz_PanoCore_36_;
  reg  _zz_PanoCore_37_;
  wire [23:0] _zz_PanoCore_38_;
  wire [7:0] _zz_PanoCore_39_;
  wire `HostXferType_staticEncoding_type _zz_PanoCore_40_;
  `ifndef SYNTHESIS
  reg [63:0] _zz_PanoCore_34__string;
  reg [63:0] _zz_PanoCore_35__string;
  reg [63:0] _zz_PanoCore_40__string;
  `endif

  assign _zz_PanoCore_73_ = (u_cpu_top_io_txt_gen_apb_PADDR & (16'b0111111111111111));
  assign _zz_PanoCore_74_ = _zz_PanoCore_73_[14:0];
  assign _zz_PanoCore_75_ = (u_cpu_top_io_txt_gen_apb_PADDR & (16'b0111111111111111));
  assign _zz_PanoCore_76_ = _zz_PanoCore_75_[14:0];
  assign _zz_PanoCore_77_ = u_cpu_top_io_ulpi_apb_PWDATA[0 : 0];
  assign _zz_PanoCore_78_ = (16'b1000000000000000);
  assign _zz_PanoCore_79_ = u_cpu_top_io_ulpi_apb_PWDATA[31 : 31];
  CpuTop u_cpu_top ( 
    .io_led_ctrl_apb_PADDR(u_cpu_top_io_led_ctrl_apb_PADDR),
    .io_led_ctrl_apb_PSEL(u_cpu_top_io_led_ctrl_apb_PSEL),
    .io_led_ctrl_apb_PENABLE(u_cpu_top_io_led_ctrl_apb_PENABLE),
    .io_led_ctrl_apb_PREADY(u_led_ctrl_io_apb_PREADY),
    .io_led_ctrl_apb_PWRITE(u_cpu_top_io_led_ctrl_apb_PWRITE),
    .io_led_ctrl_apb_PWDATA(u_cpu_top_io_led_ctrl_apb_PWDATA),
    .io_led_ctrl_apb_PRDATA(u_led_ctrl_io_apb_PRDATA),
    .io_led_ctrl_apb_PSLVERROR(u_led_ctrl_io_apb_PSLVERROR),
    .io_dvi_ctrl_apb_PADDR(u_cpu_top_io_dvi_ctrl_apb_PADDR),
    .io_dvi_ctrl_apb_PSEL(u_cpu_top_io_dvi_ctrl_apb_PSEL),
    .io_dvi_ctrl_apb_PENABLE(u_cpu_top_io_dvi_ctrl_apb_PENABLE),
    .io_dvi_ctrl_apb_PREADY(cCGpio_1__io_apb_PREADY),
    .io_dvi_ctrl_apb_PWRITE(u_cpu_top_io_dvi_ctrl_apb_PWRITE),
    .io_dvi_ctrl_apb_PWDATA(u_cpu_top_io_dvi_ctrl_apb_PWDATA),
    .io_dvi_ctrl_apb_PRDATA(cCGpio_1__io_apb_PRDATA),
    .io_dvi_ctrl_apb_PSLVERROR(cCGpio_1__io_apb_PSLVERROR),
    .io_gmii_ctrl_apb_PADDR(u_cpu_top_io_gmii_ctrl_apb_PADDR),
    .io_gmii_ctrl_apb_PSEL(u_cpu_top_io_gmii_ctrl_apb_PSEL),
    .io_gmii_ctrl_apb_PENABLE(u_cpu_top_io_gmii_ctrl_apb_PENABLE),
    .io_gmii_ctrl_apb_PREADY(gmiiCtrl_1__io_apb_PREADY),
    .io_gmii_ctrl_apb_PWRITE(u_cpu_top_io_gmii_ctrl_apb_PWRITE),
    .io_gmii_ctrl_apb_PWDATA(u_cpu_top_io_gmii_ctrl_apb_PWDATA),
    .io_gmii_ctrl_apb_PRDATA(gmiiCtrl_1__io_apb_PRDATA),
    .io_gmii_ctrl_apb_PSLVERROR(gmiiCtrl_1__io_apb_PSLVERROR),
    .io_test_patt_apb_PADDR(u_cpu_top_io_test_patt_apb_PADDR),
    .io_test_patt_apb_PSEL(u_cpu_top_io_test_patt_apb_PSEL),
    .io_test_patt_apb_PENABLE(u_cpu_top_io_test_patt_apb_PENABLE),
    .io_test_patt_apb_PREADY(_zz_PanoCore_41_),
    .io_test_patt_apb_PWRITE(u_cpu_top_io_test_patt_apb_PWRITE),
    .io_test_patt_apb_PWDATA(u_cpu_top_io_test_patt_apb_PWDATA),
    .io_test_patt_apb_PRDATA(_zz_PanoCore_42_),
    .io_test_patt_apb_PSLVERROR(_zz_PanoCore_43_),
    .io_txt_gen_apb_PADDR(u_cpu_top_io_txt_gen_apb_PADDR),
    .io_txt_gen_apb_PSEL(u_cpu_top_io_txt_gen_apb_PSEL),
    .io_txt_gen_apb_PENABLE(u_cpu_top_io_txt_gen_apb_PENABLE),
    .io_txt_gen_apb_PREADY(_zz_PanoCore_44_),
    .io_txt_gen_apb_PWRITE(u_cpu_top_io_txt_gen_apb_PWRITE),
    .io_txt_gen_apb_PWDATA(u_cpu_top_io_txt_gen_apb_PWDATA),
    .io_txt_gen_apb_PRDATA(_zz_PanoCore_45_),
    .io_txt_gen_apb_PSLVERROR(_zz_PanoCore_46_),
    .io_ulpi_apb_PADDR(u_cpu_top_io_ulpi_apb_PADDR),
    .io_ulpi_apb_PSEL(u_cpu_top_io_ulpi_apb_PSEL),
    .io_ulpi_apb_PENABLE(u_cpu_top_io_ulpi_apb_PENABLE),
    .io_ulpi_apb_PREADY(_zz_PanoCore_47_),
    .io_ulpi_apb_PWRITE(u_cpu_top_io_ulpi_apb_PWRITE),
    .io_ulpi_apb_PWDATA(u_cpu_top_io_ulpi_apb_PWDATA),
    .io_ulpi_apb_PRDATA(_zz_PanoCore_48_),
    .io_ulpi_apb_PSLVERROR(_zz_PanoCore_49_),
    .io_usb_host_apb_PADDR(u_cpu_top_io_usb_host_apb_PADDR),
    .io_usb_host_apb_PSEL(u_cpu_top_io_usb_host_apb_PSEL),
    .io_usb_host_apb_PENABLE(u_cpu_top_io_usb_host_apb_PENABLE),
    .io_usb_host_apb_PREADY(apb3CC_1__io_src_PREADY),
    .io_usb_host_apb_PWRITE(u_cpu_top_io_usb_host_apb_PWRITE),
    .io_usb_host_apb_PWDATA(u_cpu_top_io_usb_host_apb_PWDATA),
    .io_usb_host_apb_PRDATA(apb3CC_1__io_src_PRDATA),
    .io_usb_host_apb_PSLVERROR(apb3CC_1__io_src_PSLVERROR),
    .io_spi_flash_ctrl_apb_PADDR(u_cpu_top_io_spi_flash_ctrl_apb_PADDR),
    .io_spi_flash_ctrl_apb_PSEL(u_cpu_top_io_spi_flash_ctrl_apb_PSEL),
    .io_spi_flash_ctrl_apb_PENABLE(u_cpu_top_io_spi_flash_ctrl_apb_PENABLE),
    .io_spi_flash_ctrl_apb_PREADY(u_spi_flash_io_apb_PREADY),
    .io_spi_flash_ctrl_apb_PWRITE(u_cpu_top_io_spi_flash_ctrl_apb_PWRITE),
    .io_spi_flash_ctrl_apb_PWDATA(u_cpu_top_io_spi_flash_ctrl_apb_PWDATA),
    .io_spi_flash_ctrl_apb_PRDATA(u_spi_flash_io_apb_PRDATA),
    .io_switch_(io_switch_),
    .io_axi1_aw_valid(u_cpu_top_io_axi1_aw_valid),
    .io_axi1_aw_ready(io_axi1_aw_ready),
    .io_axi1_aw_payload_addr(u_cpu_top_io_axi1_aw_payload_addr),
    .io_axi1_aw_payload_id(u_cpu_top_io_axi1_aw_payload_id),
    .io_axi1_aw_payload_region(u_cpu_top_io_axi1_aw_payload_region),
    .io_axi1_aw_payload_len(u_cpu_top_io_axi1_aw_payload_len),
    .io_axi1_aw_payload_size(u_cpu_top_io_axi1_aw_payload_size),
    .io_axi1_aw_payload_burst(u_cpu_top_io_axi1_aw_payload_burst),
    .io_axi1_aw_payload_lock(u_cpu_top_io_axi1_aw_payload_lock),
    .io_axi1_aw_payload_cache(u_cpu_top_io_axi1_aw_payload_cache),
    .io_axi1_aw_payload_qos(u_cpu_top_io_axi1_aw_payload_qos),
    .io_axi1_aw_payload_prot(u_cpu_top_io_axi1_aw_payload_prot),
    .io_axi1_w_valid(u_cpu_top_io_axi1_w_valid),
    .io_axi1_w_ready(io_axi1_w_ready),
    .io_axi1_w_payload_data(u_cpu_top_io_axi1_w_payload_data),
    .io_axi1_w_payload_strb(u_cpu_top_io_axi1_w_payload_strb),
    .io_axi1_w_payload_last(u_cpu_top_io_axi1_w_payload_last),
    .io_axi1_b_valid(io_axi1_b_valid),
    .io_axi1_b_ready(u_cpu_top_io_axi1_b_ready),
    .io_axi1_b_payload_id(io_axi1_b_payload_id),
    .io_axi1_b_payload_resp(io_axi1_b_payload_resp),
    .io_axi1_ar_valid(u_cpu_top_io_axi1_ar_valid),
    .io_axi1_ar_ready(io_axi1_ar_ready),
    .io_axi1_ar_payload_addr(u_cpu_top_io_axi1_ar_payload_addr),
    .io_axi1_ar_payload_id(u_cpu_top_io_axi1_ar_payload_id),
    .io_axi1_ar_payload_region(u_cpu_top_io_axi1_ar_payload_region),
    .io_axi1_ar_payload_len(u_cpu_top_io_axi1_ar_payload_len),
    .io_axi1_ar_payload_size(u_cpu_top_io_axi1_ar_payload_size),
    .io_axi1_ar_payload_burst(u_cpu_top_io_axi1_ar_payload_burst),
    .io_axi1_ar_payload_lock(u_cpu_top_io_axi1_ar_payload_lock),
    .io_axi1_ar_payload_cache(u_cpu_top_io_axi1_ar_payload_cache),
    .io_axi1_ar_payload_qos(u_cpu_top_io_axi1_ar_payload_qos),
    .io_axi1_ar_payload_prot(u_cpu_top_io_axi1_ar_payload_prot),
    .io_axi1_r_valid(io_axi1_r_valid),
    .io_axi1_r_ready(u_cpu_top_io_axi1_r_ready),
    .io_axi1_r_payload_data(io_axi1_r_payload_data),
    .io_axi1_r_payload_id(io_axi1_r_payload_id),
    .io_axi1_r_payload_resp(io_axi1_r_payload_resp),
    .io_axi1_r_payload_last(io_axi1_r_payload_last),
    .io_axi2_aw_valid(u_cpu_top_io_axi2_aw_valid),
    .io_axi2_aw_ready(io_axi2_aw_ready),
    .io_axi2_aw_payload_addr(u_cpu_top_io_axi2_aw_payload_addr),
    .io_axi2_aw_payload_id(u_cpu_top_io_axi2_aw_payload_id),
    .io_axi2_aw_payload_region(u_cpu_top_io_axi2_aw_payload_region),
    .io_axi2_aw_payload_len(u_cpu_top_io_axi2_aw_payload_len),
    .io_axi2_aw_payload_size(u_cpu_top_io_axi2_aw_payload_size),
    .io_axi2_aw_payload_burst(u_cpu_top_io_axi2_aw_payload_burst),
    .io_axi2_aw_payload_lock(u_cpu_top_io_axi2_aw_payload_lock),
    .io_axi2_aw_payload_cache(u_cpu_top_io_axi2_aw_payload_cache),
    .io_axi2_aw_payload_qos(u_cpu_top_io_axi2_aw_payload_qos),
    .io_axi2_aw_payload_prot(u_cpu_top_io_axi2_aw_payload_prot),
    .io_axi2_w_valid(u_cpu_top_io_axi2_w_valid),
    .io_axi2_w_ready(io_axi2_w_ready),
    .io_axi2_w_payload_data(u_cpu_top_io_axi2_w_payload_data),
    .io_axi2_w_payload_strb(u_cpu_top_io_axi2_w_payload_strb),
    .io_axi2_w_payload_last(u_cpu_top_io_axi2_w_payload_last),
    .io_axi2_b_valid(io_axi2_b_valid),
    .io_axi2_b_ready(u_cpu_top_io_axi2_b_ready),
    .io_axi2_b_payload_id(io_axi2_b_payload_id),
    .io_axi2_b_payload_resp(io_axi2_b_payload_resp),
    .io_axi2_ar_valid(u_cpu_top_io_axi2_ar_valid),
    .io_axi2_ar_ready(io_axi2_ar_ready),
    .io_axi2_ar_payload_addr(u_cpu_top_io_axi2_ar_payload_addr),
    .io_axi2_ar_payload_id(u_cpu_top_io_axi2_ar_payload_id),
    .io_axi2_ar_payload_region(u_cpu_top_io_axi2_ar_payload_region),
    .io_axi2_ar_payload_len(u_cpu_top_io_axi2_ar_payload_len),
    .io_axi2_ar_payload_size(u_cpu_top_io_axi2_ar_payload_size),
    .io_axi2_ar_payload_burst(u_cpu_top_io_axi2_ar_payload_burst),
    .io_axi2_ar_payload_lock(u_cpu_top_io_axi2_ar_payload_lock),
    .io_axi2_ar_payload_cache(u_cpu_top_io_axi2_ar_payload_cache),
    .io_axi2_ar_payload_qos(u_cpu_top_io_axi2_ar_payload_qos),
    .io_axi2_ar_payload_prot(u_cpu_top_io_axi2_ar_payload_prot),
    .io_axi2_r_valid(io_axi2_r_valid),
    .io_axi2_r_ready(u_cpu_top_io_axi2_r_ready),
    .io_axi2_r_payload_data(io_axi2_r_payload_data),
    .io_axi2_r_payload_id(io_axi2_r_payload_id),
    .io_axi2_r_payload_resp(io_axi2_r_payload_resp),
    .io_axi2_r_payload_last(io_axi2_r_payload_last),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  VideoTimingGen vo_area_u_vi_gen ( 
    .io_timings_h_active(vo_area_timings_h_active),
    .io_timings_h_fp(vo_area_timings_h_fp),
    .io_timings_h_sync(vo_area_timings_h_sync),
    .io_timings_h_bp(vo_area_timings_h_bp),
    .io_timings_h_sync_positive(vo_area_timings_h_sync_positive),
    .io_timings_v_active(vo_area_timings_v_active),
    .io_timings_v_fp(vo_area_timings_v_fp),
    .io_timings_v_sync(vo_area_timings_v_sync),
    .io_timings_v_bp(vo_area_timings_v_bp),
    .io_timings_v_sync_positive(vo_area_timings_v_sync_positive),
    .io_pixel_out_vsync(vo_area_u_vi_gen_io_pixel_out_vsync),
    .io_pixel_out_req(vo_area_u_vi_gen_io_pixel_out_req),
    .io_pixel_out_last_col(vo_area_u_vi_gen_io_pixel_out_last_col),
    .io_pixel_out_last_line(vo_area_u_vi_gen_io_pixel_out_last_line),
    .io_pixel_out_pixel_r(vo_area_u_vi_gen_io_pixel_out_pixel_r),
    .io_pixel_out_pixel_g(vo_area_u_vi_gen_io_pixel_out_pixel_g),
    .io_pixel_out_pixel_b(vo_area_u_vi_gen_io_pixel_out_pixel_b),
    .toplevel_u_vo_clk_gen_vo_clk(toplevel_u_vo_clk_gen_vo_clk),
    .toplevel_u_vo_clk_gen_vo_reset_(toplevel_u_vo_clk_gen_vo_reset_) 
  );
  VideoTestPattern vo_area_u_test_patt ( 
    .io_timings_h_active(vo_area_timings_h_active),
    .io_timings_h_fp(vo_area_timings_h_fp),
    .io_timings_h_sync(vo_area_timings_h_sync),
    .io_timings_h_bp(vo_area_timings_h_bp),
    .io_timings_h_sync_positive(vo_area_timings_h_sync_positive),
    .io_timings_v_active(vo_area_timings_v_active),
    .io_timings_v_fp(vo_area_timings_v_fp),
    .io_timings_v_sync(vo_area_timings_v_sync),
    .io_timings_v_bp(vo_area_timings_v_bp),
    .io_timings_v_sync_positive(vo_area_timings_v_sync_positive),
    .io_pixel_in_vsync(vo_area_vi_gen_pixel_out_vsync),
    .io_pixel_in_req(vo_area_vi_gen_pixel_out_req),
    .io_pixel_in_last_col(vo_area_vi_gen_pixel_out_last_col),
    .io_pixel_in_last_line(vo_area_vi_gen_pixel_out_last_line),
    .io_pixel_in_pixel_r(vo_area_vi_gen_pixel_out_pixel_r),
    .io_pixel_in_pixel_g(vo_area_vi_gen_pixel_out_pixel_g),
    .io_pixel_in_pixel_b(vo_area_vi_gen_pixel_out_pixel_b),
    .io_pixel_out_vsync(vo_area_u_test_patt_io_pixel_out_vsync),
    .io_pixel_out_req(vo_area_u_test_patt_io_pixel_out_req),
    .io_pixel_out_last_col(vo_area_u_test_patt_io_pixel_out_last_col),
    .io_pixel_out_last_line(vo_area_u_test_patt_io_pixel_out_last_line),
    .io_pixel_out_pixel_r(vo_area_u_test_patt_io_pixel_out_pixel_r),
    .io_pixel_out_pixel_g(vo_area_u_test_patt_io_pixel_out_pixel_g),
    .io_pixel_out_pixel_b(vo_area_u_test_patt_io_pixel_out_pixel_b),
    .io_pattern_nr(vo_area_test_patt_ctrl_apb_regs_pattern_nr),
    .io_const_color_r(vo_area_test_patt_ctrl_apb_regs_const_color_r),
    .io_const_color_g(vo_area_test_patt_ctrl_apb_regs_const_color_g),
    .io_const_color_b(vo_area_test_patt_ctrl_apb_regs_const_color_b),
    .toplevel_u_vo_clk_gen_vo_clk(toplevel_u_vo_clk_gen_vo_clk),
    .toplevel_u_vo_clk_gen_vo_reset_(toplevel_u_vo_clk_gen_vo_reset_) 
  );
  VideoTxtGen vo_area_u_txt_gen ( 
    .io_pixel_in_vsync(vo_area_test_patt_pixel_out_vsync),
    .io_pixel_in_req(vo_area_test_patt_pixel_out_req),
    .io_pixel_in_last_col(vo_area_test_patt_pixel_out_last_col),
    .io_pixel_in_last_line(vo_area_test_patt_pixel_out_last_line),
    .io_pixel_in_pixel_r(vo_area_test_patt_pixel_out_pixel_r),
    .io_pixel_in_pixel_g(vo_area_test_patt_pixel_out_pixel_g),
    .io_pixel_in_pixel_b(vo_area_test_patt_pixel_out_pixel_b),
    .io_pixel_out_vsync(vo_area_u_txt_gen_io_pixel_out_vsync),
    .io_pixel_out_req(vo_area_u_txt_gen_io_pixel_out_req),
    .io_pixel_out_last_col(vo_area_u_txt_gen_io_pixel_out_last_col),
    .io_pixel_out_last_line(vo_area_u_txt_gen_io_pixel_out_last_line),
    .io_pixel_out_pixel_r(vo_area_u_txt_gen_io_pixel_out_pixel_r),
    .io_pixel_out_pixel_g(vo_area_u_txt_gen_io_pixel_out_pixel_g),
    .io_pixel_out_pixel_b(vo_area_u_txt_gen_io_pixel_out_pixel_b),
    .io_txt_buf_wr(_zz_PanoCore_50_),
    .io_txt_buf_rd(_zz_PanoCore_51_),
    .io_txt_buf_addr(_zz_PanoCore_52_),
    .io_txt_buf_wr_data(_zz_PanoCore_53_),
    .io_txt_buf_rd_data(vo_area_u_txt_gen_io_txt_buf_rd_data),
    .toplevel_u_vo_clk_gen_vo_clk(toplevel_u_vo_clk_gen_vo_clk),
    .toplevel_u_vo_clk_gen_vo_reset_(toplevel_u_vo_clk_gen_vo_reset_),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  VideoOut vo_area_u_vo ( 
    .io_timings_h_active(vo_area_timings_h_active),
    .io_timings_h_fp(vo_area_timings_h_fp),
    .io_timings_h_sync(vo_area_timings_h_sync),
    .io_timings_h_bp(vo_area_timings_h_bp),
    .io_timings_h_sync_positive(vo_area_timings_h_sync_positive),
    .io_timings_v_active(vo_area_timings_v_active),
    .io_timings_v_fp(vo_area_timings_v_fp),
    .io_timings_v_sync(vo_area_timings_v_sync),
    .io_timings_v_bp(vo_area_timings_v_bp),
    .io_timings_v_sync_positive(vo_area_timings_v_sync_positive),
    .io_pixel_in_vsync(vo_area_txt_gen_pixel_out_vsync),
    .io_pixel_in_req(vo_area_txt_gen_pixel_out_req),
    .io_pixel_in_last_col(vo_area_txt_gen_pixel_out_last_col),
    .io_pixel_in_last_line(vo_area_txt_gen_pixel_out_last_line),
    .io_pixel_in_pixel_r(vo_area_txt_gen_pixel_out_pixel_r),
    .io_pixel_in_pixel_g(vo_area_txt_gen_pixel_out_pixel_g),
    .io_pixel_in_pixel_b(vo_area_txt_gen_pixel_out_pixel_b),
    .io_vga_out_vsync(vo_area_u_vo_io_vga_out_vsync),
    .io_vga_out_hsync(vo_area_u_vo_io_vga_out_hsync),
    .io_vga_out_blank_(vo_area_u_vo_io_vga_out_blank_),
    .io_vga_out_de(vo_area_u_vo_io_vga_out_de),
    .io_vga_out_r(vo_area_u_vo_io_vga_out_r),
    .io_vga_out_g(vo_area_u_vo_io_vga_out_g),
    .io_vga_out_b(vo_area_u_vo_io_vga_out_b),
    .toplevel_u_vo_clk_gen_vo_clk(toplevel_u_vo_clk_gen_vo_clk),
    .toplevel_u_vo_clk_gen_vo_reset_(toplevel_u_vo_clk_gen_vo_reset_) 
  );
  GmiiCtrl gmiiCtrl_1_ ( 
    .io_apb_PADDR(u_cpu_top_io_gmii_ctrl_apb_PADDR),
    .io_apb_PSEL(u_cpu_top_io_gmii_ctrl_apb_PSEL),
    .io_apb_PENABLE(u_cpu_top_io_gmii_ctrl_apb_PENABLE),
    .io_apb_PREADY(gmiiCtrl_1__io_apb_PREADY),
    .io_apb_PWRITE(u_cpu_top_io_gmii_ctrl_apb_PWRITE),
    .io_apb_PWDATA(u_cpu_top_io_gmii_ctrl_apb_PWDATA),
    .io_apb_PRDATA(gmiiCtrl_1__io_apb_PRDATA),
    .io_apb_PSLVERROR(gmiiCtrl_1__io_apb_PSLVERROR),
    .io_gmii_rx_clk(io_gmii_rx_clk),
    .io_gmii_rx_dv(io_gmii_rx_dv),
    .io_gmii_rx_er(io_gmii_rx_er),
    .io_gmii_rx_d(io_gmii_rx_d),
    .io_gmii_tx_gclk(io_gmii_tx_gclk),
    .io_gmii_tx_clk(io_gmii_tx_clk),
    .io_gmii_tx_en(gmiiCtrl_1__io_gmii_tx_en),
    .io_gmii_tx_er(gmiiCtrl_1__io_gmii_tx_er),
    .io_gmii_tx_d(gmiiCtrl_1__io_gmii_tx_d),
    .io_gmii_col(io_gmii_col),
    .io_gmii_crs(io_gmii_crs),
    .io_gmii_mdio_mdc(gmiiCtrl_1__io_gmii_mdio_mdc),
    .io_gmii_mdio_mdio_read(io_gmii_mdio_mdio_read),
    .io_gmii_mdio_mdio_write(gmiiCtrl_1__io_gmii_mdio_mdio_write),
    .io_gmii_mdio_mdio_writeEnable(gmiiCtrl_1__io_gmii_mdio_mdio_writeEnable),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  UlpiCtrl ulpiCtrl_1_ ( 
    .io_ulpi_clk(io_ulpi_clk),
    .io_ulpi_data_read(io_ulpi_data_read),
    .io_ulpi_data_write(ulpiCtrl_1__io_ulpi_data_write),
    .io_ulpi_data_writeEnable(ulpiCtrl_1__io_ulpi_data_writeEnable),
    .io_ulpi_direction(io_ulpi_direction),
    .io_ulpi_stp(ulpiCtrl_1__io_ulpi_stp),
    .io_ulpi_nxt(io_ulpi_nxt),
    .io_ulpi_reset(ulpiCtrl_1__io_ulpi_reset),
    .io_tx_start(pulseCCByToggle_5__io_pulseOut),
    .io_tx_data_valid(streamFifoCC_6__io_pop_valid),
    .io_tx_data_ready(ulpiCtrl_1__io_tx_data_ready),
    .io_tx_data_payload(streamFifoCC_6__io_pop_payload),
    .io_rx_data_valid(ulpiCtrl_1__io_rx_data_valid),
    .io_rx_data_payload(ulpiCtrl_1__io_rx_data_payload),
    .io_rx_cmd_changed(ulpiCtrl_1__io_rx_cmd_changed),
    .io_rx_cmd(ulpiCtrl_1__io_rx_cmd),
    .io_reg_rd(_zz_PanoCore_54_),
    .io_reg_wr(_zz_PanoCore_55_),
    .io_reg_addr(_zz_PanoCore_9_),
    .io_reg_wr_data(_zz_PanoCore_10_),
    .io_reg_rd_data(ulpiCtrl_1__io_reg_rd_data),
    .io_reg_done(ulpiCtrl_1__io_reg_done),
    .ulpi_reset__1_(ulpiCtrl_1__ulpi_reset__1_) 
  );
  StreamFifoCC_3_ streamFifoCC_5_ ( 
    .io_push_valid(_zz_PanoCore_14__regNext),
    .io_push_ready(streamFifoCC_5__io_push_ready),
    .io_push_payload(_zz_PanoCore_11_),
    .io_pop_valid(streamFifoCC_5__io_pop_valid),
    .io_pop_ready(ulpiCtrl_1__io_reg_done),
    .io_pop_payload(streamFifoCC_5__io_pop_payload),
    .io_pushOccupancy(streamFifoCC_5__io_pushOccupancy),
    .io_popOccupancy(streamFifoCC_5__io_popOccupancy),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_),
    ._zz_StreamFifoCC_3__5_(io_ulpi_clk),
    ._zz_StreamFifoCC_3__6_(ulpiCtrl_1__ulpi_reset__1_) 
  );
  PulseCCByToggle_2_ pulseCCByToggle_4_ ( 
    .io_pulseIn(ulpiCtrl_1__io_rx_cmd_changed),
    .io_pulseOut(pulseCCByToggle_4__io_pulseOut),
    ._zz_PulseCCByToggle_2__1_(io_ulpi_clk),
    ._zz_PulseCCByToggle_2__2_(ulpiCtrl_1__ulpi_reset__1_),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  StreamFifoCC_4_ streamFifoCC_6_ ( 
    .io_push_valid(_zz_PanoCore_17_),
    .io_push_ready(streamFifoCC_6__io_push_ready),
    .io_push_payload(_zz_PanoCore_39_),
    .io_pop_valid(streamFifoCC_6__io_pop_valid),
    .io_pop_ready(ulpiCtrl_1__io_tx_data_ready),
    .io_pop_payload(streamFifoCC_6__io_pop_payload),
    .io_pushOccupancy(streamFifoCC_6__io_pushOccupancy),
    .io_popOccupancy(streamFifoCC_6__io_popOccupancy),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_),
    ._zz_StreamFifoCC_4__23_(io_ulpi_clk),
    ._zz_StreamFifoCC_4__24_(ulpiCtrl_1__ulpi_reset__1_) 
  );
  PulseCCByToggle_3_ pulseCCByToggle_5_ ( 
    .io_pulseIn(_zz_PanoCore_56_),
    .io_pulseOut(pulseCCByToggle_5__io_pulseOut),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_),
    ._zz_PulseCCByToggle_3__1_(io_ulpi_clk),
    ._zz_PulseCCByToggle_3__2_(ulpiCtrl_1__ulpi_reset__1_) 
  );
  BufferCC_7_ bufferCC_19_ ( 
    .io_initial(_zz_PanoCore_57_),
    .io_dataIn(streamFifoCC_6__io_pop_valid),
    .io_dataOut(bufferCC_19__io_dataOut),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  Apb3CC apb3CC_1_ ( 
    .io_src_PADDR(u_cpu_top_io_usb_host_apb_PADDR),
    .io_src_PSEL(u_cpu_top_io_usb_host_apb_PSEL),
    .io_src_PENABLE(u_cpu_top_io_usb_host_apb_PENABLE),
    .io_src_PREADY(apb3CC_1__io_src_PREADY),
    .io_src_PWRITE(u_cpu_top_io_usb_host_apb_PWRITE),
    .io_src_PWDATA(u_cpu_top_io_usb_host_apb_PWDATA),
    .io_src_PRDATA(apb3CC_1__io_src_PRDATA),
    .io_src_PSLVERROR(apb3CC_1__io_src_PSLVERROR),
    .io_dest_PADDR(apb3CC_1__io_dest_PADDR),
    .io_dest_PSEL(apb3CC_1__io_dest_PSEL),
    .io_dest_PENABLE(apb3CC_1__io_dest_PENABLE),
    .io_dest_PREADY(_zz_PanoCore_58_),
    .io_dest_PWRITE(apb3CC_1__io_dest_PWRITE),
    .io_dest_PWDATA(apb3CC_1__io_dest_PWDATA),
    .io_dest_PRDATA(_zz_PanoCore_20_),
    .io_dest_PSLVERROR(_zz_PanoCore_59_),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_),
    .core_u_pano_core_io_ulpi_clk(io_ulpi_clk),
    ._zz_Apb3CC_2_(_zz_PanoCore_2_) 
  );
  UsbHost usbHost_1_ ( 
    .io_cpu_fifo_bus_cmd_valid(_zz_PanoCore_60_),
    .io_cpu_fifo_bus_cmd_ready(usbHost_1__io_cpu_fifo_bus_cmd_ready),
    .io_cpu_fifo_bus_cmd_payload_write(_zz_PanoCore_61_),
    .io_cpu_fifo_bus_cmd_payload_address(_zz_PanoCore_62_),
    .io_cpu_fifo_bus_cmd_payload_data(_zz_PanoCore_63_),
    .io_cpu_fifo_bus_cmd_payload_mask(_zz_PanoCore_64_),
    .io_cpu_fifo_bus_rsp_valid(usbHost_1__io_cpu_fifo_bus_rsp_valid),
    .io_cpu_fifo_bus_rsp_payload_data(usbHost_1__io_cpu_fifo_bus_rsp_payload_data),
    .io_periph_addr(_zz_PanoCore_22_),
    .io_endpoint(_zz_PanoCore_32_),
    .io_send_buf_avail(usbHost_1__io_send_buf_avail),
    .io_send_buf_avail_nr(usbHost_1__io_send_buf_avail_nr),
    .io_send_byte_count_valid(_zz_PanoCore_24_),
    .io_send_byte_count_payload(_zz_PanoCore_65_),
    .io_xfer_type_valid(_zz_PanoCore_33_),
    .io_xfer_type_payload(_zz_PanoCore_34_),
    .io_xfer_result(usbHost_1__io_xfer_result),
    .io_cur_send_data_toggle(usbHost_1__io_cur_send_data_toggle),
    .io_cur_rcv_data_toggle(usbHost_1__io_cur_rcv_data_toggle),
    .io_set_send_data_toggle_valid(_zz_PanoCore_66_),
    .io_set_send_data_toggle_payload(_zz_PanoCore_67_),
    .io_set_rcv_data_toggle_valid(_zz_PanoCore_68_),
    .io_set_rcv_data_toggle_payload(_zz_PanoCore_69_),
    .io_ulpi_rx_cmd_changed(usbHost_1__io_ulpi_rx_cmd_changed),
    .io_ulpi_rx_cmd(usbHost_1__io_ulpi_rx_cmd),
    .io_ulpi_tx_data_valid(usbHost_1__io_ulpi_tx_data_valid),
    .io_ulpi_tx_data_ready(_zz_PanoCore_70_),
    .io_ulpi_tx_data_payload(usbHost_1__io_ulpi_tx_data_payload),
    .core_u_pano_core_io_ulpi_clk(io_ulpi_clk),
    ._zz_UsbHost_12_(_zz_PanoCore_2_) 
  );
  Apb3Gpio u_led_ctrl ( 
    .io_apb_PADDR(u_cpu_top_io_led_ctrl_apb_PADDR),
    .io_apb_PSEL(u_cpu_top_io_led_ctrl_apb_PSEL),
    .io_apb_PENABLE(u_cpu_top_io_led_ctrl_apb_PENABLE),
    .io_apb_PREADY(u_led_ctrl_io_apb_PREADY),
    .io_apb_PWRITE(u_cpu_top_io_led_ctrl_apb_PWRITE),
    .io_apb_PWDATA(u_cpu_top_io_led_ctrl_apb_PWDATA),
    .io_apb_PRDATA(u_led_ctrl_io_apb_PRDATA),
    .io_apb_PSLVERROR(u_led_ctrl_io_apb_PSLVERROR),
    .io_gpio_read(_zz_PanoCore_71_),
    .io_gpio_write(u_led_ctrl_io_gpio_write),
    .io_gpio_writeEnable(u_led_ctrl_io_gpio_writeEnable),
    .io_value(u_led_ctrl_io_value),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  CCGpio cCGpio_1_ ( 
    .io_apb_PADDR(u_cpu_top_io_dvi_ctrl_apb_PADDR),
    .io_apb_PSEL(u_cpu_top_io_dvi_ctrl_apb_PSEL),
    .io_apb_PENABLE(u_cpu_top_io_dvi_ctrl_apb_PENABLE),
    .io_apb_PREADY(cCGpio_1__io_apb_PREADY),
    .io_apb_PWRITE(u_cpu_top_io_dvi_ctrl_apb_PWRITE),
    .io_apb_PWDATA(u_cpu_top_io_dvi_ctrl_apb_PWDATA),
    .io_apb_PRDATA(cCGpio_1__io_apb_PRDATA),
    .io_apb_PSLVERROR(cCGpio_1__io_apb_PSLVERROR),
    .io_gpio_read(_zz_PanoCore_72_),
    .io_gpio_write(cCGpio_1__io_gpio_write),
    .io_gpio_writeEnable(cCGpio_1__io_gpio_writeEnable),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  Apb3SpiMasterCtrl u_spi_flash ( 
    .io_apb_PADDR(u_cpu_top_io_spi_flash_ctrl_apb_PADDR),
    .io_apb_PSEL(u_cpu_top_io_spi_flash_ctrl_apb_PSEL),
    .io_apb_PENABLE(u_cpu_top_io_spi_flash_ctrl_apb_PENABLE),
    .io_apb_PREADY(u_spi_flash_io_apb_PREADY),
    .io_apb_PWRITE(u_cpu_top_io_spi_flash_ctrl_apb_PWRITE),
    .io_apb_PWDATA(u_cpu_top_io_spi_flash_ctrl_apb_PWDATA),
    .io_apb_PRDATA(u_spi_flash_io_apb_PRDATA),
    .io_spi_ss(u_spi_flash_io_spi_ss),
    .io_spi_sclk(u_spi_flash_io_spi_sclk),
    .io_spi_mosi(u_spi_flash_io_spi_mosi),
    .io_spi_miso(io_spi_miso),
    .io_interrupt(u_spi_flash_io_interrupt),
    .toplevel_main_clk(toplevel_main_clk),
    .toplevel_main_reset_(toplevel_main_reset_) 
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(_zz_PanoCore_34_)
      `HostXferType_staticEncoding_SETUP : _zz_PanoCore_34__string = "SETUP   ";
      `HostXferType_staticEncoding_BULK_IN : _zz_PanoCore_34__string = "BULK_IN ";
      `HostXferType_staticEncoding_BULK_OUT : _zz_PanoCore_34__string = "BULK_OUT";
      `HostXferType_staticEncoding_HS_IN : _zz_PanoCore_34__string = "HS_IN   ";
      `HostXferType_staticEncoding_HS_OUT : _zz_PanoCore_34__string = "HS_OUT  ";
      `HostXferType_staticEncoding_ISO_IN : _zz_PanoCore_34__string = "ISO_IN  ";
      `HostXferType_staticEncoding_ISO_OUT : _zz_PanoCore_34__string = "ISO_OUT ";
      default : _zz_PanoCore_34__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_PanoCore_35_)
      `HostXferResult_defaultEncoding_SUCCESS : _zz_PanoCore_35__string = "SUCCESS ";
      `HostXferResult_defaultEncoding_BUSY : _zz_PanoCore_35__string = "BUSY    ";
      `HostXferResult_defaultEncoding_BADREQ : _zz_PanoCore_35__string = "BADREQ  ";
      `HostXferResult_defaultEncoding_UNDEF : _zz_PanoCore_35__string = "UNDEF   ";
      `HostXferResult_defaultEncoding_NAK : _zz_PanoCore_35__string = "NAK     ";
      `HostXferResult_defaultEncoding_STALL : _zz_PanoCore_35__string = "STALL   ";
      `HostXferResult_defaultEncoding_TOGERR : _zz_PanoCore_35__string = "TOGERR  ";
      `HostXferResult_defaultEncoding_WRONGPID : _zz_PanoCore_35__string = "WRONGPID";
      `HostXferResult_defaultEncoding_BADBC : _zz_PanoCore_35__string = "BADBC   ";
      `HostXferResult_defaultEncoding_PIDERR : _zz_PanoCore_35__string = "PIDERR  ";
      `HostXferResult_defaultEncoding_PKTERR : _zz_PanoCore_35__string = "PKTERR  ";
      `HostXferResult_defaultEncoding_CRCERR : _zz_PanoCore_35__string = "CRCERR  ";
      `HostXferResult_defaultEncoding_KERR : _zz_PanoCore_35__string = "KERR    ";
      `HostXferResult_defaultEncoding_JERR : _zz_PanoCore_35__string = "JERR    ";
      `HostXferResult_defaultEncoding_TIMEOUT : _zz_PanoCore_35__string = "TIMEOUT ";
      `HostXferResult_defaultEncoding_BABBLE : _zz_PanoCore_35__string = "BABBLE  ";
      default : _zz_PanoCore_35__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_PanoCore_40_)
      `HostXferType_staticEncoding_SETUP : _zz_PanoCore_40__string = "SETUP   ";
      `HostXferType_staticEncoding_BULK_IN : _zz_PanoCore_40__string = "BULK_IN ";
      `HostXferType_staticEncoding_BULK_OUT : _zz_PanoCore_40__string = "BULK_OUT";
      `HostXferType_staticEncoding_HS_IN : _zz_PanoCore_40__string = "HS_IN   ";
      `HostXferType_staticEncoding_HS_OUT : _zz_PanoCore_40__string = "HS_OUT  ";
      `HostXferType_staticEncoding_ISO_IN : _zz_PanoCore_40__string = "ISO_IN  ";
      `HostXferType_staticEncoding_ISO_OUT : _zz_PanoCore_40__string = "ISO_OUT ";
      default : _zz_PanoCore_40__string = "????????";
    endcase
  end
  `endif

  assign _zz_PanoCore_1_[23 : 0] = (24'b111111111111111111111111);
  assign io_led_red = leds_led_cntr[23];
  assign io_axi1_ar_valid = u_cpu_top_io_axi1_ar_valid;
  assign io_axi1_ar_payload_addr = u_cpu_top_io_axi1_ar_payload_addr;
  assign io_axi1_ar_payload_id = u_cpu_top_io_axi1_ar_payload_id;
  assign io_axi1_ar_payload_region = u_cpu_top_io_axi1_ar_payload_region;
  assign io_axi1_ar_payload_len = u_cpu_top_io_axi1_ar_payload_len;
  assign io_axi1_ar_payload_size = u_cpu_top_io_axi1_ar_payload_size;
  assign io_axi1_ar_payload_burst = u_cpu_top_io_axi1_ar_payload_burst;
  assign io_axi1_ar_payload_lock = u_cpu_top_io_axi1_ar_payload_lock;
  assign io_axi1_ar_payload_cache = u_cpu_top_io_axi1_ar_payload_cache;
  assign io_axi1_ar_payload_qos = u_cpu_top_io_axi1_ar_payload_qos;
  assign io_axi1_ar_payload_prot = u_cpu_top_io_axi1_ar_payload_prot;
  assign io_axi1_aw_valid = u_cpu_top_io_axi1_aw_valid;
  assign io_axi1_aw_payload_addr = u_cpu_top_io_axi1_aw_payload_addr;
  assign io_axi1_aw_payload_id = u_cpu_top_io_axi1_aw_payload_id;
  assign io_axi1_aw_payload_region = u_cpu_top_io_axi1_aw_payload_region;
  assign io_axi1_aw_payload_len = u_cpu_top_io_axi1_aw_payload_len;
  assign io_axi1_aw_payload_size = u_cpu_top_io_axi1_aw_payload_size;
  assign io_axi1_aw_payload_burst = u_cpu_top_io_axi1_aw_payload_burst;
  assign io_axi1_aw_payload_lock = u_cpu_top_io_axi1_aw_payload_lock;
  assign io_axi1_aw_payload_cache = u_cpu_top_io_axi1_aw_payload_cache;
  assign io_axi1_aw_payload_qos = u_cpu_top_io_axi1_aw_payload_qos;
  assign io_axi1_aw_payload_prot = u_cpu_top_io_axi1_aw_payload_prot;
  assign io_axi1_w_valid = u_cpu_top_io_axi1_w_valid;
  assign io_axi1_w_payload_data = u_cpu_top_io_axi1_w_payload_data;
  assign io_axi1_w_payload_strb = u_cpu_top_io_axi1_w_payload_strb;
  assign io_axi1_w_payload_last = u_cpu_top_io_axi1_w_payload_last;
  assign io_axi1_r_ready = u_cpu_top_io_axi1_r_ready;
  assign io_axi1_b_ready = u_cpu_top_io_axi1_b_ready;
  assign io_axi2_ar_valid = u_cpu_top_io_axi2_ar_valid;
  assign io_axi2_ar_payload_addr = u_cpu_top_io_axi2_ar_payload_addr;
  assign io_axi2_ar_payload_id = u_cpu_top_io_axi2_ar_payload_id;
  assign io_axi2_ar_payload_region = u_cpu_top_io_axi2_ar_payload_region;
  assign io_axi2_ar_payload_len = u_cpu_top_io_axi2_ar_payload_len;
  assign io_axi2_ar_payload_size = u_cpu_top_io_axi2_ar_payload_size;
  assign io_axi2_ar_payload_burst = u_cpu_top_io_axi2_ar_payload_burst;
  assign io_axi2_ar_payload_lock = u_cpu_top_io_axi2_ar_payload_lock;
  assign io_axi2_ar_payload_cache = u_cpu_top_io_axi2_ar_payload_cache;
  assign io_axi2_ar_payload_qos = u_cpu_top_io_axi2_ar_payload_qos;
  assign io_axi2_ar_payload_prot = u_cpu_top_io_axi2_ar_payload_prot;
  assign io_axi2_aw_valid = u_cpu_top_io_axi2_aw_valid;
  assign io_axi2_aw_payload_addr = u_cpu_top_io_axi2_aw_payload_addr;
  assign io_axi2_aw_payload_id = u_cpu_top_io_axi2_aw_payload_id;
  assign io_axi2_aw_payload_region = u_cpu_top_io_axi2_aw_payload_region;
  assign io_axi2_aw_payload_len = u_cpu_top_io_axi2_aw_payload_len;
  assign io_axi2_aw_payload_size = u_cpu_top_io_axi2_aw_payload_size;
  assign io_axi2_aw_payload_burst = u_cpu_top_io_axi2_aw_payload_burst;
  assign io_axi2_aw_payload_lock = u_cpu_top_io_axi2_aw_payload_lock;
  assign io_axi2_aw_payload_cache = u_cpu_top_io_axi2_aw_payload_cache;
  assign io_axi2_aw_payload_qos = u_cpu_top_io_axi2_aw_payload_qos;
  assign io_axi2_aw_payload_prot = u_cpu_top_io_axi2_aw_payload_prot;
  assign io_axi2_w_valid = u_cpu_top_io_axi2_w_valid;
  assign io_axi2_w_payload_data = u_cpu_top_io_axi2_w_payload_data;
  assign io_axi2_w_payload_strb = u_cpu_top_io_axi2_w_payload_strb;
  assign io_axi2_w_payload_last = u_cpu_top_io_axi2_w_payload_last;
  assign io_axi2_r_ready = u_cpu_top_io_axi2_r_ready;
  assign io_axi2_b_ready = u_cpu_top_io_axi2_b_ready;
  assign vo_area_timings_h_active = (12'b011110000000);
  assign vo_area_timings_h_fp = (9'b001011000);
  assign vo_area_timings_h_sync = (9'b000101100);
  assign vo_area_timings_h_bp = (9'b010010100);
  assign vo_area_timings_h_sync_positive = 1'b1;
  assign vo_area_timings_v_active = (11'b10000111000);
  assign vo_area_timings_v_fp = (9'b000000100);
  assign vo_area_timings_v_sync = (9'b000000101);
  assign vo_area_timings_v_bp = (9'b000100100);
  assign vo_area_timings_v_sync_positive = 1'b1;
  assign vo_area_vi_gen_pixel_out_vsync = vo_area_u_vi_gen_io_pixel_out_vsync;
  assign vo_area_vi_gen_pixel_out_req = vo_area_u_vi_gen_io_pixel_out_req;
  assign vo_area_vi_gen_pixel_out_last_col = vo_area_u_vi_gen_io_pixel_out_last_col;
  assign vo_area_vi_gen_pixel_out_last_line = vo_area_u_vi_gen_io_pixel_out_last_line;
  assign vo_area_vi_gen_pixel_out_pixel_r = vo_area_u_vi_gen_io_pixel_out_pixel_r;
  assign vo_area_vi_gen_pixel_out_pixel_g = vo_area_u_vi_gen_io_pixel_out_pixel_g;
  assign vo_area_vi_gen_pixel_out_pixel_b = vo_area_u_vi_gen_io_pixel_out_pixel_b;
  assign vo_area_test_patt_pixel_out_vsync = vo_area_u_test_patt_io_pixel_out_vsync;
  assign vo_area_test_patt_pixel_out_req = vo_area_u_test_patt_io_pixel_out_req;
  assign vo_area_test_patt_pixel_out_last_col = vo_area_u_test_patt_io_pixel_out_last_col;
  assign vo_area_test_patt_pixel_out_last_line = vo_area_u_test_patt_io_pixel_out_last_line;
  assign vo_area_test_patt_pixel_out_pixel_r = vo_area_u_test_patt_io_pixel_out_pixel_r;
  assign vo_area_test_patt_pixel_out_pixel_g = vo_area_u_test_patt_io_pixel_out_pixel_g;
  assign vo_area_test_patt_pixel_out_pixel_b = vo_area_u_test_patt_io_pixel_out_pixel_b;
  assign _zz_PanoCore_41_ = 1'b1;
  always @ (*) begin
    _zz_PanoCore_42_ = (32'b00000000000000000000000000000000);
    case(u_cpu_top_io_test_patt_apb_PADDR)
      5'b00000 : begin
        _zz_PanoCore_42_[3 : 0] = vo_area_test_patt_ctrl_apb_regs_pattern_nr;
      end
      5'b00100 : begin
        _zz_PanoCore_42_[23 : 0] = {vo_area_test_patt_ctrl_apb_regs_const_color_b,{vo_area_test_patt_ctrl_apb_regs_const_color_g,vo_area_test_patt_ctrl_apb_regs_const_color_r}};
      end
      default : begin
      end
    endcase
  end

  assign _zz_PanoCore_43_ = 1'b0;
  assign vo_area_test_patt_ctrl_busCtrl_askWrite = ((u_cpu_top_io_test_patt_apb_PSEL[0] && u_cpu_top_io_test_patt_apb_PENABLE) && u_cpu_top_io_test_patt_apb_PWRITE);
  assign vo_area_test_patt_ctrl_busCtrl_askRead = ((u_cpu_top_io_test_patt_apb_PSEL[0] && u_cpu_top_io_test_patt_apb_PENABLE) && (! u_cpu_top_io_test_patt_apb_PWRITE));
  assign vo_area_test_patt_ctrl_busCtrl_doWrite = (((u_cpu_top_io_test_patt_apb_PSEL[0] && u_cpu_top_io_test_patt_apb_PENABLE) && _zz_PanoCore_41_) && u_cpu_top_io_test_patt_apb_PWRITE);
  assign vo_area_test_patt_ctrl_busCtrl_doRead = (((u_cpu_top_io_test_patt_apb_PSEL[0] && u_cpu_top_io_test_patt_apb_PENABLE) && _zz_PanoCore_41_) && (! u_cpu_top_io_test_patt_apb_PWRITE));
  assign vo_area_txt_gen_pixel_out_vsync = vo_area_u_txt_gen_io_pixel_out_vsync;
  assign vo_area_txt_gen_pixel_out_req = vo_area_u_txt_gen_io_pixel_out_req;
  assign vo_area_txt_gen_pixel_out_last_col = vo_area_u_txt_gen_io_pixel_out_last_col;
  assign vo_area_txt_gen_pixel_out_last_line = vo_area_u_txt_gen_io_pixel_out_last_line;
  assign vo_area_txt_gen_pixel_out_pixel_r = vo_area_u_txt_gen_io_pixel_out_pixel_r;
  assign vo_area_txt_gen_pixel_out_pixel_g = vo_area_u_txt_gen_io_pixel_out_pixel_g;
  assign vo_area_txt_gen_pixel_out_pixel_b = vo_area_u_txt_gen_io_pixel_out_pixel_b;
  always @ (*) begin
    _zz_PanoCore_44_ = 1'b1;
    _zz_PanoCore_45_ = (32'b00000000000000000000000000000000);
    _zz_PanoCore_50_ = 1'b0;
    _zz_PanoCore_51_ = 1'b0;
    _zz_PanoCore_52_ = vo_area_txt_gen_ctrl_apb_regs_txt_buf_wr_addr;
    _zz_PanoCore_3_ = 1'b0;
    if(((u_cpu_top_io_txt_gen_apb_PADDR & _zz_PanoCore_78_) == (16'b0000000000000000)))begin
      if(vo_area_txt_gen_ctrl_busCtrl_doWrite)begin
        _zz_PanoCore_50_ = 1'b1;
        _zz_PanoCore_52_ = vo_area_txt_gen_ctrl_apb_regs_txt_buf_wr_addr;
      end
      if(vo_area_txt_gen_ctrl_busCtrl_askRead)begin
        _zz_PanoCore_3_ = 1'b1;
        if((! _zz_PanoCore_6_))begin
          _zz_PanoCore_44_ = 1'b0;
        end
        _zz_PanoCore_51_ = 1'b1;
        _zz_PanoCore_52_ = vo_area_txt_gen_ctrl_apb_regs_txt_buf_rd_addr;
      end
      _zz_PanoCore_45_[7 : 0] = vo_area_u_txt_gen_io_txt_buf_rd_data;
    end
  end

  assign _zz_PanoCore_46_ = 1'b0;
  assign vo_area_txt_gen_ctrl_busCtrl_askWrite = ((u_cpu_top_io_txt_gen_apb_PSEL[0] && u_cpu_top_io_txt_gen_apb_PENABLE) && u_cpu_top_io_txt_gen_apb_PWRITE);
  assign vo_area_txt_gen_ctrl_busCtrl_askRead = ((u_cpu_top_io_txt_gen_apb_PSEL[0] && u_cpu_top_io_txt_gen_apb_PENABLE) && (! u_cpu_top_io_txt_gen_apb_PWRITE));
  assign vo_area_txt_gen_ctrl_busCtrl_doWrite = (((u_cpu_top_io_txt_gen_apb_PSEL[0] && u_cpu_top_io_txt_gen_apb_PENABLE) && _zz_PanoCore_44_) && u_cpu_top_io_txt_gen_apb_PWRITE);
  assign vo_area_txt_gen_ctrl_busCtrl_doRead = (((u_cpu_top_io_txt_gen_apb_PSEL[0] && u_cpu_top_io_txt_gen_apb_PENABLE) && _zz_PanoCore_44_) && (! u_cpu_top_io_txt_gen_apb_PWRITE));
  assign vo_area_txt_gen_ctrl_apb_regs_txt_buf_rd_addr = (_zz_PanoCore_74_ >>> 2);
  assign vo_area_txt_gen_ctrl_apb_regs_txt_buf_wr_addr = (_zz_PanoCore_76_ >>> 2);
  assign _zz_PanoCore_6_ = (_zz_PanoCore_5_ == (1'b1));
  always @ (*) begin
    _zz_PanoCore_4_ = (_zz_PanoCore_5_ + _zz_PanoCore_3_);
    if(1'b0)begin
      _zz_PanoCore_4_ = (1'b0);
    end
  end

  assign io_vo_vsync = vo_area_u_vo_io_vga_out_vsync;
  assign io_vo_hsync = vo_area_u_vo_io_vga_out_hsync;
  assign io_vo_blank_ = vo_area_u_vo_io_vga_out_blank_;
  assign io_vo_de = vo_area_u_vo_io_vga_out_de;
  assign io_vo_r = vo_area_u_vo_io_vga_out_r;
  assign io_vo_g = vo_area_u_vo_io_vga_out_g;
  assign io_vo_b = vo_area_u_vo_io_vga_out_b;
  assign io_gmii_tx_en = gmiiCtrl_1__io_gmii_tx_en;
  assign io_gmii_tx_er = gmiiCtrl_1__io_gmii_tx_er;
  assign io_gmii_tx_d = gmiiCtrl_1__io_gmii_tx_d;
  assign io_gmii_mdio_mdc = gmiiCtrl_1__io_gmii_mdio_mdc;
  assign io_gmii_mdio_mdio_write = gmiiCtrl_1__io_gmii_mdio_mdio_write;
  assign io_gmii_mdio_mdio_writeEnable = gmiiCtrl_1__io_gmii_mdio_mdio_writeEnable;
  assign io_ulpi_data_write = ulpiCtrl_1__io_ulpi_data_write;
  assign io_ulpi_data_writeEnable = ulpiCtrl_1__io_ulpi_data_writeEnable;
  assign io_ulpi_stp = ulpiCtrl_1__io_ulpi_stp;
  assign io_ulpi_reset = ulpiCtrl_1__io_ulpi_reset;
  assign _zz_PanoCore_47_ = 1'b1;
  always @ (*) begin
    _zz_PanoCore_48_ = (32'b00000000000000000000000000000000);
    _zz_PanoCore_14_ = 1'b0;
    _zz_PanoCore_16_ = 1'b0;
    _zz_PanoCore_17_ = 1'b0;
    _zz_PanoCore_18_ = 1'b0;
    case(u_cpu_top_io_ulpi_apb_PADDR)
      6'b000000 : begin
        if(_zz_PanoCore_7_)begin
          _zz_PanoCore_14_ = 1'b1;
        end
        _zz_PanoCore_48_[5 : 0] = _zz_PanoCore_9_;
        _zz_PanoCore_48_[15 : 8] = _zz_PanoCore_10_;
        _zz_PanoCore_48_[31 : 31] = _zz_PanoCore_11_;
      end
      6'b000100 : begin
        _zz_PanoCore_48_[7 : 0] = ulpiCtrl_1__io_reg_rd_data;
        _zz_PanoCore_48_[8 : 8] = ((2'b00) < streamFifoCC_5__io_pushOccupancy);
      end
      6'b001000 : begin
        if(_zz_PanoCore_8_)begin
          _zz_PanoCore_16_ = 1'b1;
        end
        _zz_PanoCore_48_[7 : 0] = ulpiCtrl_1__io_rx_cmd;
        _zz_PanoCore_48_[8 : 8] = _zz_PanoCore_15_;
      end
      6'b001100 : begin
        if(_zz_PanoCore_7_)begin
          _zz_PanoCore_17_ = 1'b1;
        end
      end
      6'b010000 : begin
        if(_zz_PanoCore_7_)begin
          _zz_PanoCore_18_ = 1'b1;
        end
      end
      6'b010100 : begin
        _zz_PanoCore_48_[10 : 0] = streamFifoCC_6__io_pushOccupancy;
        _zz_PanoCore_48_[16 : 16] = streamFifoCC_6__io_push_ready;
        _zz_PanoCore_48_[17 : 17] = (! bufferCC_19__io_dataOut);
      end
      default : begin
      end
    endcase
  end

  assign _zz_PanoCore_49_ = 1'b0;
  assign _zz_PanoCore_7_ = (((u_cpu_top_io_ulpi_apb_PSEL[0] && u_cpu_top_io_ulpi_apb_PENABLE) && _zz_PanoCore_47_) && u_cpu_top_io_ulpi_apb_PWRITE);
  assign _zz_PanoCore_8_ = (((u_cpu_top_io_ulpi_apb_PSEL[0] && u_cpu_top_io_ulpi_apb_PENABLE) && _zz_PanoCore_47_) && (! u_cpu_top_io_ulpi_apb_PWRITE));
  assign _zz_PanoCore_12_ = streamFifoCC_5__io_pop_valid;
  assign _zz_PanoCore_13_ = streamFifoCC_5__io_pop_payload;
  assign _zz_PanoCore_55_ = (_zz_PanoCore_12_ && _zz_PanoCore_13_);
  assign _zz_PanoCore_54_ = (_zz_PanoCore_12_ && (! _zz_PanoCore_13_));
  assign _zz_PanoCore_56_ = (_zz_PanoCore_18_ && _zz_PanoCore_77_[0]);
  assign _zz_PanoCore_57_ = 1'b0;
  assign _zz_PanoCore_58_ = 1'b1;
  assign _zz_PanoCore_19_ = apb3CC_1__io_dest_PWDATA;
  assign _zz_PanoCore_59_ = 1'b0;
  always @ (*) begin
    _zz_PanoCore_20_ = (32'b00000000000000000000000000000000);
    _zz_PanoCore_60_ = 1'b0;
    _zz_PanoCore_61_ = 1'b0;
    _zz_PanoCore_62_ = (9'b000000000);
    _zz_PanoCore_24_ = 1'b0;
    _zz_PanoCore_28_ = 1'b0;
    _zz_PanoCore_30_ = 1'b0;
    _zz_PanoCore_33_ = 1'b0;
    case(apb3CC_1__io_dest_PADDR)
      7'b1110000 : begin
        _zz_PanoCore_20_[6 : 0] = _zz_PanoCore_22_;
      end
      7'b0001000 : begin
        if(_zz_PanoCore_21_)begin
          _zz_PanoCore_60_ = 1'b1;
          _zz_PanoCore_61_ = 1'b1;
          _zz_PanoCore_62_ = {{(2'b01),usbHost_1__io_send_buf_avail_nr},_zz_PanoCore_23_};
        end
      end
      7'b0011100 : begin
        if(_zz_PanoCore_21_)begin
          _zz_PanoCore_24_ = 1'b1;
        end
      end
      7'b0010000 : begin
        if(_zz_PanoCore_21_)begin
          _zz_PanoCore_60_ = 1'b1;
          _zz_PanoCore_61_ = 1'b1;
          _zz_PanoCore_62_ = _zz_PanoCore_26_;
        end
      end
      7'b1100100 : begin
        _zz_PanoCore_20_[3 : 3] = _zz_PanoCore_27_;
      end
      7'b1110100 : begin
        if(_zz_PanoCore_21_)begin
          _zz_PanoCore_28_ = 1'b1;
          _zz_PanoCore_30_ = 1'b1;
        end
      end
      7'b1111000 : begin
        if(_zz_PanoCore_21_)begin
          _zz_PanoCore_33_ = 1'b1;
        end
      end
      7'b1111100 : begin
        _zz_PanoCore_20_[3 : 0] = _zz_PanoCore_35_;
        _zz_PanoCore_20_[5 : 5] = _zz_PanoCore_36_;
        _zz_PanoCore_20_[4 : 4] = _zz_PanoCore_37_;
      end
      default : begin
      end
    endcase
  end

  assign _zz_PanoCore_21_ = (((apb3CC_1__io_dest_PSEL[0] && apb3CC_1__io_dest_PENABLE) && 1'b1) && apb3CC_1__io_dest_PWRITE);
  assign _zz_PanoCore_65_ = _zz_PanoCore_19_[5 : 0];
  always @ (*) begin
    _zz_PanoCore_26_ = (9'b000000000);
    _zz_PanoCore_26_[8 : 6] = (3'b100);
    _zz_PanoCore_26_[2 : 0] = _zz_PanoCore_25_;
  end

  assign _zz_PanoCore_68_ = ((_zz_PanoCore_28_ && (_zz_PanoCore_29_ != (2'b00))) && (_zz_PanoCore_29_ != (2'b11)));
  assign _zz_PanoCore_69_ = _zz_PanoCore_29_[1];
  assign _zz_PanoCore_66_ = ((_zz_PanoCore_30_ && (_zz_PanoCore_31_ != (2'b00))) && (_zz_PanoCore_31_ != (2'b11)));
  assign _zz_PanoCore_67_ = _zz_PanoCore_31_[1];
  assign io_led_green = u_led_ctrl_io_gpio_write[0];
  always @ (*) begin
    _zz_PanoCore_71_[0] = io_led_green;
    _zz_PanoCore_71_[1] = io_led_blue;
    _zz_PanoCore_71_[2] = 1'b0;
  end

  assign io_led_blue = u_led_ctrl_io_gpio_write[1];
  assign io_dvi_ctrl_scl_writeEnable = (! cCGpio_1__io_gpio_write[0]);
  assign io_dvi_ctrl_scl_write = cCGpio_1__io_gpio_write[0];
  always @ (*) begin
    _zz_PanoCore_72_[0] = io_dvi_ctrl_scl_read;
    _zz_PanoCore_72_[1] = io_dvi_ctrl_sda_read;
  end

  assign io_dvi_ctrl_sda_writeEnable = (! cCGpio_1__io_gpio_write[1]);
  assign io_dvi_ctrl_sda_write = cCGpio_1__io_gpio_write[1];
  assign io_spi_ss = u_spi_flash_io_spi_ss;
  assign io_spi_sclk = u_spi_flash_io_spi_sclk;
  assign io_spi_mosi = u_spi_flash_io_spi_mosi;
  assign _zz_PanoCore_38_ = u_cpu_top_io_test_patt_apb_PWDATA[23 : 0];
  assign _zz_PanoCore_53_ = u_cpu_top_io_txt_gen_apb_PWDATA[7 : 0];
  assign _zz_PanoCore_39_ = u_cpu_top_io_ulpi_apb_PWDATA[7 : 0];
  assign _zz_PanoCore_63_ = _zz_PanoCore_19_[7 : 0];
  assign _zz_PanoCore_29_ = _zz_PanoCore_19_[5 : 4];
  assign _zz_PanoCore_31_ = _zz_PanoCore_19_[7 : 6];
  assign _zz_PanoCore_40_ = _zz_PanoCore_19_[7 : 4];
  assign _zz_PanoCore_34_ = _zz_PanoCore_40_;
  always @ (posedge toplevel_main_clk) begin
    if(!toplevel_main_reset_) begin
      leds_led_cntr <= (24'b000000000000000000000000);
      vo_area_test_patt_ctrl_apb_regs_pattern_nr <= (4'b0000);
      _zz_PanoCore_5_ <= (1'b0);
      _zz_PanoCore_9_ <= (6'b000000);
      _zz_PanoCore_10_ <= (8'b00000000);
      _zz_PanoCore_11_ <= 1'b0;
      _zz_PanoCore_14__regNext <= 1'b0;
      _zz_PanoCore_15_ <= 1'b0;
    end else begin
      if((leds_led_cntr == _zz_PanoCore_1_))begin
        leds_led_cntr <= (24'b000000000000000000000000);
      end else begin
        leds_led_cntr <= (leds_led_cntr + (24'b000000000000000000000001));
      end
      _zz_PanoCore_5_ <= _zz_PanoCore_4_;
      _zz_PanoCore_14__regNext <= _zz_PanoCore_14_;
      if(_zz_PanoCore_16_)begin
        _zz_PanoCore_15_ <= 1'b0;
      end
      if(pulseCCByToggle_4__io_pulseOut)begin
        _zz_PanoCore_15_ <= 1'b1;
      end
      case(u_cpu_top_io_test_patt_apb_PADDR)
        5'b00000 : begin
          if(vo_area_test_patt_ctrl_busCtrl_doWrite)begin
            vo_area_test_patt_ctrl_apb_regs_pattern_nr <= u_cpu_top_io_test_patt_apb_PWDATA[3 : 0];
          end
        end
        5'b00100 : begin
        end
        default : begin
        end
      endcase
      case(u_cpu_top_io_ulpi_apb_PADDR)
        6'b000000 : begin
          if(_zz_PanoCore_7_)begin
            _zz_PanoCore_9_ <= u_cpu_top_io_ulpi_apb_PWDATA[5 : 0];
            _zz_PanoCore_10_ <= u_cpu_top_io_ulpi_apb_PWDATA[15 : 8];
            _zz_PanoCore_11_ <= _zz_PanoCore_79_[0];
          end
        end
        6'b000100 : begin
        end
        6'b001000 : begin
        end
        6'b001100 : begin
        end
        6'b010000 : begin
        end
        6'b010100 : begin
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge io_ulpi_clk) begin
    _zz_PanoCore_2_ <= 1'b1;
  end

  always @ (posedge io_ulpi_clk) begin
    if(!_zz_PanoCore_2_) begin
      _zz_PanoCore_23_ <= (6'b000000);
      _zz_PanoCore_25_ <= (3'b000);
    end else begin
      case(apb3CC_1__io_dest_PADDR)
        7'b1110000 : begin
        end
        7'b0001000 : begin
          if(_zz_PanoCore_21_)begin
            _zz_PanoCore_23_ <= (_zz_PanoCore_23_ + (6'b000001));
          end
        end
        7'b0011100 : begin
        end
        7'b0010000 : begin
          if(_zz_PanoCore_21_)begin
            _zz_PanoCore_25_ <= (_zz_PanoCore_25_ + (3'b001));
          end
        end
        7'b1100100 : begin
        end
        7'b1110100 : begin
        end
        7'b1111000 : begin
        end
        7'b1111100 : begin
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge io_ulpi_clk) begin
    _zz_PanoCore_27_ <= usbHost_1__io_send_buf_avail;
    _zz_PanoCore_35_ <= usbHost_1__io_xfer_result;
    _zz_PanoCore_36_ <= usbHost_1__io_cur_send_data_toggle;
    _zz_PanoCore_37_ <= usbHost_1__io_cur_rcv_data_toggle;
    case(apb3CC_1__io_dest_PADDR)
      7'b1110000 : begin
        if(_zz_PanoCore_21_)begin
          _zz_PanoCore_22_ <= _zz_PanoCore_19_[6 : 0];
        end
      end
      7'b0001000 : begin
      end
      7'b0011100 : begin
      end
      7'b0010000 : begin
      end
      7'b1100100 : begin
      end
      7'b1110100 : begin
      end
      7'b1111000 : begin
        if(_zz_PanoCore_21_)begin
          _zz_PanoCore_32_ <= _zz_PanoCore_19_[3 : 0];
        end
      end
      7'b1111100 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (posedge toplevel_main_clk) begin
    case(u_cpu_top_io_test_patt_apb_PADDR)
      5'b00000 : begin
      end
      5'b00100 : begin
        if(vo_area_test_patt_ctrl_busCtrl_doWrite)begin
          vo_area_test_patt_ctrl_apb_regs_const_color_r <= _zz_PanoCore_38_[7 : 0];
          vo_area_test_patt_ctrl_apb_regs_const_color_g <= _zz_PanoCore_38_[15 : 8];
          vo_area_test_patt_ctrl_apb_regs_const_color_b <= _zz_PanoCore_38_[23 : 16];
        end
      end
      default : begin
      end
    endcase
  end

endmodule

module ChrontelPads (
      output  io_pads_reset_,
      output  io_pads_xclk_p,
      output  io_pads_xclk_n,
      output  io_pads_v,
      output  io_pads_h,
      output  io_pads_de,
      output reg [11:0] io_pads_d,
      input   io_vsync,
      input   io_hsync,
      input   io_de,
      input  [7:0] io_r,
      input  [7:0] io_g,
      input  [7:0] io_b,
      input   clk,
      input   reset_);
  wire  _zz_ChrontelPads_1_;
  wire  _zz_ChrontelPads_2_;
  wire  _zz_ChrontelPads_3_;
  wire  _zz_ChrontelPads_4_;
  wire  _zz_ChrontelPads_5_;
  wire  _zz_ChrontelPads_6_;
  wire  _zz_ChrontelPads_7_;
  wire  _zz_ChrontelPads_8_;
  wire  _zz_ChrontelPads_9_;
  wire  _zz_ChrontelPads_10_;
  wire  _zz_ChrontelPads_11_;
  wire  _zz_ChrontelPads_12_;
  wire  _zz_ChrontelPads_13_;
  wire  _zz_ChrontelPads_14_;
  wire  _zz_ChrontelPads_15_;
  wire  _zz_ChrontelPads_16_;
  wire  _zz_ChrontelPads_17_;
  wire  _zz_ChrontelPads_18_;
  wire  _zz_ChrontelPads_19_;
  wire  _zz_ChrontelPads_20_;
  wire  _zz_ChrontelPads_21_;
  wire  _zz_ChrontelPads_22_;
  wire  _zz_ChrontelPads_23_;
  wire  _zz_ChrontelPads_24_;
  wire  _zz_ChrontelPads_25_;
  wire  _zz_ChrontelPads_26_;
  wire  _zz_ChrontelPads_27_;
  wire  _zz_ChrontelPads_28_;
  wire  _zz_ChrontelPads_29_;
  wire  _zz_ChrontelPads_30_;
  wire  _zz_ChrontelPads_31_;
  wire  _zz_ChrontelPads_32_;
  wire  _zz_ChrontelPads_33_;
  wire  _zz_ChrontelPads_34_;
  wire  _zz_ChrontelPads_35_;
  wire  _zz_ChrontelPads_36_;
  wire  _zz_ChrontelPads_37_;
  wire  _zz_ChrontelPads_38_;
  wire  _zz_ChrontelPads_39_;
  wire  _zz_ChrontelPads_40_;
  wire  _zz_ChrontelPads_41_;
  wire  _zz_ChrontelPads_42_;
  wire  _zz_ChrontelPads_43_;
  wire  _zz_ChrontelPads_44_;
  wire  _zz_ChrontelPads_45_;
  wire  _zz_ChrontelPads_46_;
  wire  _zz_ChrontelPads_47_;
  wire  _zz_ChrontelPads_48_;
  wire  _zz_ChrontelPads_49_;
  wire  _zz_ChrontelPads_50_;
  wire  _zz_ChrontelPads_51_;
  wire  _zz_ChrontelPads_52_;
  wire  _zz_ChrontelPads_53_;
  wire  _zz_ChrontelPads_54_;
  wire  _zz_ChrontelPads_55_;
  wire  _zz_ChrontelPads_56_;
  wire  _zz_ChrontelPads_57_;
  wire  _zz_ChrontelPads_58_;
  wire  _zz_ChrontelPads_59_;
  wire  _zz_ChrontelPads_60_;
  wire  _zz_ChrontelPads_61_;
  wire  _zz_ChrontelPads_62_;
  wire  _zz_ChrontelPads_63_;
  wire  _zz_ChrontelPads_64_;
  wire  _zz_ChrontelPads_65_;
  wire  _zz_ChrontelPads_66_;
  wire  _zz_ChrontelPads_67_;
  wire  u_dcm_CLK0;
  wire  u_dcm_CLK90;
  wire  u_dcm_CLK180;
  wire  u_dcm_CLK270;
  wire  u_dcm_CLK2X;
  wire  u_dcm_CLK2X180;
  wire  u_dcm_CLKDV;
  wire  u_dcm_CLKFX;
  wire  u_dcm_CLKFX180;
  wire  u_dcm_LOCKED;
  wire [7:0] u_dcm_STATUS;
  wire  u_pad_xclk_p_Q;
  wire  oDDR2_1__Q;
  wire  u_pad_vsync_Q;
  wire  u_pad_hsync_Q;
  wire  u_pad_de_Q;
  wire  oDDR2_2__Q;
  wire  oDDR2_3__Q;
  wire  oDDR2_4__Q;
  wire  oDDR2_5__Q;
  wire  oDDR2_6__Q;
  wire  oDDR2_7__Q;
  wire  oDDR2_8__Q;
  wire  oDDR2_9__Q;
  wire  oDDR2_10__Q;
  wire  oDDR2_11__Q;
  wire  oDDR2_12__Q;
  wire  oDDR2_13__Q;
  wire  vsync_p1;
  wire  hsync_p1;
  wire  de_p1;
  wire [7:0] r_p1;
  wire [7:0] g_p1;
  wire [7:0] b_p1;
  (* keep = "true" *) reg  io_vsync_regNext;
  (* keep = "true" *) reg  io_hsync_regNext;
  (* keep = "true" *) reg  io_de_regNext;
  (* keep = "true" *) reg [7:0] io_r_regNext;
  (* keep = "true" *) reg [7:0] io_g_regNext;
  (* keep = "true" *) reg [7:0] io_b_regNext;
  wire  clk0;
  wire  clk90;
  wire  clk180;
  wire  clk270;
  wire  pad_reset;
  wire [11:0] d_p;
  wire [11:0] d_n;
  DCM_SP #( 
    .CLKDV_DIVIDE(2.0),
    .CLK_FEEDBACK("1X"),
    .CLKFX_DIVIDE(1),
    .CLKFX_MULTIPLY(2),
    .CLKIN_DIVIDE_BY_2(1'b0),
    .CLKIN_PERIOD("10.0"),
    .CLKOUT_PHASE_SHIFT("NONE"),
    .DESKEW_ADJUST("SYSTEM_SYNCHRONOUS"),
    .DLL_FREQUENCY_MODE("LOW"),
    .DSS_MODE("NONE"),
    .DUTY_CYCLE_CORRECTION(1'b0),
    .PHASE_SHIFT(0),
    .STARTUP_WAIT(1'b0) 
  ) u_dcm ( 
    .RST(pad_reset),
    .CLKIN(clk),
    .CLKFB(clk0),
    .DSSEN(_zz_ChrontelPads_1_),
    .PSCLK(_zz_ChrontelPads_2_),
    .PSINCDEC(_zz_ChrontelPads_3_),
    .PSEN(_zz_ChrontelPads_4_),
    .PSDONE(_zz_ChrontelPads_5_),
    .CLK0(u_dcm_CLK0),
    .CLK90(u_dcm_CLK90),
    .CLK180(u_dcm_CLK180),
    .CLK270(u_dcm_CLK270),
    .CLK2X(u_dcm_CLK2X),
    .CLK2X180(u_dcm_CLK2X180),
    .CLKDV(u_dcm_CLKDV),
    .CLKFX(u_dcm_CLKFX),
    .CLKFX180(u_dcm_CLKFX180),
    .LOCKED(u_dcm_LOCKED),
    .STATUS(u_dcm_STATUS) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) u_pad_xclk_p ( 
    .D0(_zz_ChrontelPads_6_),
    .D1(_zz_ChrontelPads_7_),
    .C0(clk90),
    .C1(clk270),
    .CE(_zz_ChrontelPads_8_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_9_),
    .Q(u_pad_xclk_p_Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_1_ ( 
    .D0(_zz_ChrontelPads_10_),
    .D1(_zz_ChrontelPads_11_),
    .C0(clk90),
    .C1(clk270),
    .CE(_zz_ChrontelPads_12_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_13_),
    .Q(oDDR2_1__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) u_pad_vsync ( 
    .D0(vsync_p1),
    .D1(vsync_p1),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_14_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_15_),
    .Q(u_pad_vsync_Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) u_pad_hsync ( 
    .D0(hsync_p1),
    .D1(hsync_p1),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_16_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_17_),
    .Q(u_pad_hsync_Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) u_pad_de ( 
    .D0(de_p1),
    .D1(de_p1),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_18_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_19_),
    .Q(u_pad_de_Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_2_ ( 
    .D0(_zz_ChrontelPads_20_),
    .D1(_zz_ChrontelPads_21_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_22_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_23_),
    .Q(oDDR2_2__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_3_ ( 
    .D0(_zz_ChrontelPads_24_),
    .D1(_zz_ChrontelPads_25_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_26_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_27_),
    .Q(oDDR2_3__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_4_ ( 
    .D0(_zz_ChrontelPads_28_),
    .D1(_zz_ChrontelPads_29_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_30_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_31_),
    .Q(oDDR2_4__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_5_ ( 
    .D0(_zz_ChrontelPads_32_),
    .D1(_zz_ChrontelPads_33_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_34_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_35_),
    .Q(oDDR2_5__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_6_ ( 
    .D0(_zz_ChrontelPads_36_),
    .D1(_zz_ChrontelPads_37_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_38_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_39_),
    .Q(oDDR2_6__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_7_ ( 
    .D0(_zz_ChrontelPads_40_),
    .D1(_zz_ChrontelPads_41_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_42_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_43_),
    .Q(oDDR2_7__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_8_ ( 
    .D0(_zz_ChrontelPads_44_),
    .D1(_zz_ChrontelPads_45_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_46_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_47_),
    .Q(oDDR2_8__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_9_ ( 
    .D0(_zz_ChrontelPads_48_),
    .D1(_zz_ChrontelPads_49_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_50_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_51_),
    .Q(oDDR2_9__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_10_ ( 
    .D0(_zz_ChrontelPads_52_),
    .D1(_zz_ChrontelPads_53_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_54_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_55_),
    .Q(oDDR2_10__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_11_ ( 
    .D0(_zz_ChrontelPads_56_),
    .D1(_zz_ChrontelPads_57_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_58_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_59_),
    .Q(oDDR2_11__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_12_ ( 
    .D0(_zz_ChrontelPads_60_),
    .D1(_zz_ChrontelPads_61_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_62_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_63_),
    .Q(oDDR2_12__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_13_ ( 
    .D0(_zz_ChrontelPads_64_),
    .D1(_zz_ChrontelPads_65_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_66_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_67_),
    .Q(oDDR2_13__Q) 
  );
  assign vsync_p1 = io_vsync_regNext;
  assign hsync_p1 = io_hsync_regNext;
  assign de_p1 = io_de_regNext;
  assign r_p1 = io_r_regNext;
  assign g_p1 = io_g_regNext;
  assign b_p1 = io_b_regNext;
  assign io_pads_reset_ = reset_;
  assign pad_reset = 1'b0;
  assign _zz_ChrontelPads_1_ = 1'b0;
  assign _zz_ChrontelPads_2_ = 1'b0;
  assign _zz_ChrontelPads_3_ = 1'b0;
  assign _zz_ChrontelPads_4_ = 1'b0;
  assign _zz_ChrontelPads_5_ = 1'b0;
  assign clk0 = u_dcm_CLK0;
  assign clk90 = u_dcm_CLK90;
  assign clk180 = u_dcm_CLK180;
  assign clk270 = u_dcm_CLK270;
  assign _zz_ChrontelPads_6_ = 1'b1;
  assign _zz_ChrontelPads_7_ = 1'b0;
  assign _zz_ChrontelPads_8_ = 1'b1;
  assign _zz_ChrontelPads_9_ = 1'b0;
  assign io_pads_xclk_p = u_pad_xclk_p_Q;
  assign _zz_ChrontelPads_10_ = 1'b0;
  assign _zz_ChrontelPads_11_ = 1'b1;
  assign _zz_ChrontelPads_12_ = 1'b1;
  assign _zz_ChrontelPads_13_ = 1'b0;
  assign io_pads_xclk_n = oDDR2_1__Q;
  assign _zz_ChrontelPads_14_ = 1'b1;
  assign _zz_ChrontelPads_15_ = 1'b0;
  assign io_pads_v = u_pad_vsync_Q;
  assign _zz_ChrontelPads_16_ = 1'b1;
  assign _zz_ChrontelPads_17_ = 1'b0;
  assign io_pads_h = u_pad_hsync_Q;
  assign _zz_ChrontelPads_18_ = 1'b1;
  assign _zz_ChrontelPads_19_ = 1'b0;
  assign io_pads_de = u_pad_de_Q;
  assign d_p = {g_p1[3 : 0],b_p1[7 : 0]};
  assign d_n = {r_p1[7 : 0],g_p1[7 : 4]};
  assign _zz_ChrontelPads_20_ = d_p[0];
  assign _zz_ChrontelPads_21_ = d_n[0];
  assign _zz_ChrontelPads_22_ = 1'b1;
  assign _zz_ChrontelPads_23_ = 1'b0;
  always @ (*) begin
    io_pads_d[0] = oDDR2_2__Q;
    io_pads_d[1] = oDDR2_3__Q;
    io_pads_d[2] = oDDR2_4__Q;
    io_pads_d[3] = oDDR2_5__Q;
    io_pads_d[4] = oDDR2_6__Q;
    io_pads_d[5] = oDDR2_7__Q;
    io_pads_d[6] = oDDR2_8__Q;
    io_pads_d[7] = oDDR2_9__Q;
    io_pads_d[8] = oDDR2_10__Q;
    io_pads_d[9] = oDDR2_11__Q;
    io_pads_d[10] = oDDR2_12__Q;
    io_pads_d[11] = oDDR2_13__Q;
  end

  assign _zz_ChrontelPads_24_ = d_p[1];
  assign _zz_ChrontelPads_25_ = d_n[1];
  assign _zz_ChrontelPads_26_ = 1'b1;
  assign _zz_ChrontelPads_27_ = 1'b0;
  assign _zz_ChrontelPads_28_ = d_p[2];
  assign _zz_ChrontelPads_29_ = d_n[2];
  assign _zz_ChrontelPads_30_ = 1'b1;
  assign _zz_ChrontelPads_31_ = 1'b0;
  assign _zz_ChrontelPads_32_ = d_p[3];
  assign _zz_ChrontelPads_33_ = d_n[3];
  assign _zz_ChrontelPads_34_ = 1'b1;
  assign _zz_ChrontelPads_35_ = 1'b0;
  assign _zz_ChrontelPads_36_ = d_p[4];
  assign _zz_ChrontelPads_37_ = d_n[4];
  assign _zz_ChrontelPads_38_ = 1'b1;
  assign _zz_ChrontelPads_39_ = 1'b0;
  assign _zz_ChrontelPads_40_ = d_p[5];
  assign _zz_ChrontelPads_41_ = d_n[5];
  assign _zz_ChrontelPads_42_ = 1'b1;
  assign _zz_ChrontelPads_43_ = 1'b0;
  assign _zz_ChrontelPads_44_ = d_p[6];
  assign _zz_ChrontelPads_45_ = d_n[6];
  assign _zz_ChrontelPads_46_ = 1'b1;
  assign _zz_ChrontelPads_47_ = 1'b0;
  assign _zz_ChrontelPads_48_ = d_p[7];
  assign _zz_ChrontelPads_49_ = d_n[7];
  assign _zz_ChrontelPads_50_ = 1'b1;
  assign _zz_ChrontelPads_51_ = 1'b0;
  assign _zz_ChrontelPads_52_ = d_p[8];
  assign _zz_ChrontelPads_53_ = d_n[8];
  assign _zz_ChrontelPads_54_ = 1'b1;
  assign _zz_ChrontelPads_55_ = 1'b0;
  assign _zz_ChrontelPads_56_ = d_p[9];
  assign _zz_ChrontelPads_57_ = d_n[9];
  assign _zz_ChrontelPads_58_ = 1'b1;
  assign _zz_ChrontelPads_59_ = 1'b0;
  assign _zz_ChrontelPads_60_ = d_p[10];
  assign _zz_ChrontelPads_61_ = d_n[10];
  assign _zz_ChrontelPads_62_ = 1'b1;
  assign _zz_ChrontelPads_63_ = 1'b0;
  assign _zz_ChrontelPads_64_ = d_p[11];
  assign _zz_ChrontelPads_65_ = d_n[11];
  assign _zz_ChrontelPads_66_ = 1'b1;
  assign _zz_ChrontelPads_67_ = 1'b0;
  always @ (posedge clk) begin
    io_vsync_regNext <= io_vsync;
    io_hsync_regNext <= io_hsync;
    io_de_regNext <= io_de;
    io_r_regNext <= io_r;
    io_g_regNext <= io_g;
    io_b_regNext <= io_b;
  end

endmodule

module ChrontelPads_1_ (
      output  io_pads_reset_,
      output  io_pads_xclk_p,
      output  io_pads_v,
      output  io_pads_h,
      output  io_pads_de,
      output reg [11:0] io_pads_d,
      input   io_vsync,
      input   io_hsync,
      input   io_de,
      input  [7:0] io_r,
      input  [7:0] io_g,
      input  [7:0] io_b,
      input   clk,
      input   reset_);
  wire  _zz_ChrontelPads_1__1_;
  wire  _zz_ChrontelPads_1__2_;
  wire  _zz_ChrontelPads_1__3_;
  wire  _zz_ChrontelPads_1__4_;
  wire  _zz_ChrontelPads_1__5_;
  wire  _zz_ChrontelPads_1__6_;
  wire  _zz_ChrontelPads_1__7_;
  wire  _zz_ChrontelPads_1__8_;
  wire  _zz_ChrontelPads_1__9_;
  wire  _zz_ChrontelPads_1__10_;
  wire  _zz_ChrontelPads_1__11_;
  wire  _zz_ChrontelPads_1__12_;
  wire  _zz_ChrontelPads_1__13_;
  wire  _zz_ChrontelPads_1__14_;
  wire  _zz_ChrontelPads_1__15_;
  wire  _zz_ChrontelPads_1__16_;
  wire  _zz_ChrontelPads_1__17_;
  wire  _zz_ChrontelPads_1__18_;
  wire  _zz_ChrontelPads_1__19_;
  wire  _zz_ChrontelPads_1__20_;
  wire  _zz_ChrontelPads_1__21_;
  wire  _zz_ChrontelPads_1__22_;
  wire  _zz_ChrontelPads_1__23_;
  wire  _zz_ChrontelPads_1__24_;
  wire  _zz_ChrontelPads_1__25_;
  wire  _zz_ChrontelPads_1__26_;
  wire  _zz_ChrontelPads_1__27_;
  wire  _zz_ChrontelPads_1__28_;
  wire  _zz_ChrontelPads_1__29_;
  wire  _zz_ChrontelPads_1__30_;
  wire  _zz_ChrontelPads_1__31_;
  wire  _zz_ChrontelPads_1__32_;
  wire  _zz_ChrontelPads_1__33_;
  wire  _zz_ChrontelPads_1__34_;
  wire  _zz_ChrontelPads_1__35_;
  wire  _zz_ChrontelPads_1__36_;
  wire  _zz_ChrontelPads_1__37_;
  wire  _zz_ChrontelPads_1__38_;
  wire  _zz_ChrontelPads_1__39_;
  wire  _zz_ChrontelPads_1__40_;
  wire  _zz_ChrontelPads_1__41_;
  wire  _zz_ChrontelPads_1__42_;
  wire  _zz_ChrontelPads_1__43_;
  wire  _zz_ChrontelPads_1__44_;
  wire  _zz_ChrontelPads_1__45_;
  wire  _zz_ChrontelPads_1__46_;
  wire  _zz_ChrontelPads_1__47_;
  wire  _zz_ChrontelPads_1__48_;
  wire  _zz_ChrontelPads_1__49_;
  wire  _zz_ChrontelPads_1__50_;
  wire  _zz_ChrontelPads_1__51_;
  wire  _zz_ChrontelPads_1__52_;
  wire  _zz_ChrontelPads_1__53_;
  wire  _zz_ChrontelPads_1__54_;
  wire  _zz_ChrontelPads_1__55_;
  wire  _zz_ChrontelPads_1__56_;
  wire  _zz_ChrontelPads_1__57_;
  wire  _zz_ChrontelPads_1__58_;
  wire  _zz_ChrontelPads_1__59_;
  wire  _zz_ChrontelPads_1__60_;
  wire  _zz_ChrontelPads_1__61_;
  wire  _zz_ChrontelPads_1__62_;
  wire  _zz_ChrontelPads_1__63_;
  wire  u_dcm_CLK0;
  wire  u_dcm_CLK90;
  wire  u_dcm_CLK180;
  wire  u_dcm_CLK270;
  wire  u_dcm_CLK2X;
  wire  u_dcm_CLK2X180;
  wire  u_dcm_CLKDV;
  wire  u_dcm_CLKFX;
  wire  u_dcm_CLKFX180;
  wire  u_dcm_LOCKED;
  wire [7:0] u_dcm_STATUS;
  wire  u_pad_xclk_p_Q;
  wire  u_pad_vsync_Q;
  wire  u_pad_hsync_Q;
  wire  u_pad_de_Q;
  wire  oDDR2_1__Q;
  wire  oDDR2_2__Q;
  wire  oDDR2_3__Q;
  wire  oDDR2_4__Q;
  wire  oDDR2_5__Q;
  wire  oDDR2_6__Q;
  wire  oDDR2_7__Q;
  wire  oDDR2_8__Q;
  wire  oDDR2_9__Q;
  wire  oDDR2_10__Q;
  wire  oDDR2_11__Q;
  wire  oDDR2_12__Q;
  wire  vsync_p1;
  wire  hsync_p1;
  wire  de_p1;
  wire [7:0] r_p1;
  wire [7:0] g_p1;
  wire [7:0] b_p1;
  (* keep = "true" *) reg  io_vsync_regNext;
  (* keep = "true" *) reg  io_hsync_regNext;
  (* keep = "true" *) reg  io_de_regNext;
  (* keep = "true" *) reg [7:0] io_r_regNext;
  (* keep = "true" *) reg [7:0] io_g_regNext;
  (* keep = "true" *) reg [7:0] io_b_regNext;
  wire  clk0;
  wire  clk90;
  wire  clk180;
  wire  clk270;
  wire  pad_reset;
  wire [11:0] d_p;
  wire [11:0] d_n;
  DCM_SP #( 
    .CLKDV_DIVIDE(2.0),
    .CLK_FEEDBACK("1X"),
    .CLKFX_DIVIDE(1),
    .CLKFX_MULTIPLY(2),
    .CLKIN_DIVIDE_BY_2(1'b0),
    .CLKIN_PERIOD("10.0"),
    .CLKOUT_PHASE_SHIFT("NONE"),
    .DESKEW_ADJUST("SYSTEM_SYNCHRONOUS"),
    .DLL_FREQUENCY_MODE("LOW"),
    .DSS_MODE("NONE"),
    .DUTY_CYCLE_CORRECTION(1'b0),
    .PHASE_SHIFT(0),
    .STARTUP_WAIT(1'b0) 
  ) u_dcm ( 
    .RST(pad_reset),
    .CLKIN(clk),
    .CLKFB(clk0),
    .DSSEN(_zz_ChrontelPads_1__1_),
    .PSCLK(_zz_ChrontelPads_1__2_),
    .PSINCDEC(_zz_ChrontelPads_1__3_),
    .PSEN(_zz_ChrontelPads_1__4_),
    .PSDONE(_zz_ChrontelPads_1__5_),
    .CLK0(u_dcm_CLK0),
    .CLK90(u_dcm_CLK90),
    .CLK180(u_dcm_CLK180),
    .CLK270(u_dcm_CLK270),
    .CLK2X(u_dcm_CLK2X),
    .CLK2X180(u_dcm_CLK2X180),
    .CLKDV(u_dcm_CLKDV),
    .CLKFX(u_dcm_CLKFX),
    .CLKFX180(u_dcm_CLKFX180),
    .LOCKED(u_dcm_LOCKED),
    .STATUS(u_dcm_STATUS) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) u_pad_xclk_p ( 
    .D0(_zz_ChrontelPads_1__6_),
    .D1(_zz_ChrontelPads_1__7_),
    .C0(clk90),
    .C1(clk270),
    .CE(_zz_ChrontelPads_1__8_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_1__9_),
    .Q(u_pad_xclk_p_Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) u_pad_vsync ( 
    .D0(vsync_p1),
    .D1(vsync_p1),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_1__10_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_1__11_),
    .Q(u_pad_vsync_Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) u_pad_hsync ( 
    .D0(hsync_p1),
    .D1(hsync_p1),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_1__12_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_1__13_),
    .Q(u_pad_hsync_Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) u_pad_de ( 
    .D0(de_p1),
    .D1(de_p1),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_1__14_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_1__15_),
    .Q(u_pad_de_Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_1_ ( 
    .D0(_zz_ChrontelPads_1__16_),
    .D1(_zz_ChrontelPads_1__17_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_1__18_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_1__19_),
    .Q(oDDR2_1__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_2_ ( 
    .D0(_zz_ChrontelPads_1__20_),
    .D1(_zz_ChrontelPads_1__21_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_1__22_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_1__23_),
    .Q(oDDR2_2__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_3_ ( 
    .D0(_zz_ChrontelPads_1__24_),
    .D1(_zz_ChrontelPads_1__25_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_1__26_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_1__27_),
    .Q(oDDR2_3__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_4_ ( 
    .D0(_zz_ChrontelPads_1__28_),
    .D1(_zz_ChrontelPads_1__29_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_1__30_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_1__31_),
    .Q(oDDR2_4__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_5_ ( 
    .D0(_zz_ChrontelPads_1__32_),
    .D1(_zz_ChrontelPads_1__33_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_1__34_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_1__35_),
    .Q(oDDR2_5__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_6_ ( 
    .D0(_zz_ChrontelPads_1__36_),
    .D1(_zz_ChrontelPads_1__37_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_1__38_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_1__39_),
    .Q(oDDR2_6__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_7_ ( 
    .D0(_zz_ChrontelPads_1__40_),
    .D1(_zz_ChrontelPads_1__41_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_1__42_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_1__43_),
    .Q(oDDR2_7__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_8_ ( 
    .D0(_zz_ChrontelPads_1__44_),
    .D1(_zz_ChrontelPads_1__45_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_1__46_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_1__47_),
    .Q(oDDR2_8__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_9_ ( 
    .D0(_zz_ChrontelPads_1__48_),
    .D1(_zz_ChrontelPads_1__49_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_1__50_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_1__51_),
    .Q(oDDR2_9__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_10_ ( 
    .D0(_zz_ChrontelPads_1__52_),
    .D1(_zz_ChrontelPads_1__53_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_1__54_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_1__55_),
    .Q(oDDR2_10__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_11_ ( 
    .D0(_zz_ChrontelPads_1__56_),
    .D1(_zz_ChrontelPads_1__57_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_1__58_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_1__59_),
    .Q(oDDR2_11__Q) 
  );
  ODDR2 #( 
    .DDR_ALIGNMENT("C0"),
    .INIT(1'b0),
    .SRTYPE("ASYNC") 
  ) oDDR2_12_ ( 
    .D0(_zz_ChrontelPads_1__60_),
    .D1(_zz_ChrontelPads_1__61_),
    .C0(clk0),
    .C1(clk180),
    .CE(_zz_ChrontelPads_1__62_),
    .R(pad_reset),
    .S(_zz_ChrontelPads_1__63_),
    .Q(oDDR2_12__Q) 
  );
  assign vsync_p1 = io_vsync_regNext;
  assign hsync_p1 = io_hsync_regNext;
  assign de_p1 = io_de_regNext;
  assign r_p1 = io_r_regNext;
  assign g_p1 = io_g_regNext;
  assign b_p1 = io_b_regNext;
  assign io_pads_reset_ = reset_;
  assign pad_reset = 1'b0;
  assign _zz_ChrontelPads_1__1_ = 1'b0;
  assign _zz_ChrontelPads_1__2_ = 1'b0;
  assign _zz_ChrontelPads_1__3_ = 1'b0;
  assign _zz_ChrontelPads_1__4_ = 1'b0;
  assign _zz_ChrontelPads_1__5_ = 1'b0;
  assign clk0 = u_dcm_CLK0;
  assign clk90 = u_dcm_CLK90;
  assign clk180 = u_dcm_CLK180;
  assign clk270 = u_dcm_CLK270;
  assign _zz_ChrontelPads_1__6_ = 1'b1;
  assign _zz_ChrontelPads_1__7_ = 1'b0;
  assign _zz_ChrontelPads_1__8_ = 1'b1;
  assign _zz_ChrontelPads_1__9_ = 1'b0;
  assign io_pads_xclk_p = u_pad_xclk_p_Q;
  assign _zz_ChrontelPads_1__10_ = 1'b1;
  assign _zz_ChrontelPads_1__11_ = 1'b0;
  assign io_pads_v = u_pad_vsync_Q;
  assign _zz_ChrontelPads_1__12_ = 1'b1;
  assign _zz_ChrontelPads_1__13_ = 1'b0;
  assign io_pads_h = u_pad_hsync_Q;
  assign _zz_ChrontelPads_1__14_ = 1'b1;
  assign _zz_ChrontelPads_1__15_ = 1'b0;
  assign io_pads_de = u_pad_de_Q;
  assign d_p = {g_p1[3 : 0],b_p1[7 : 0]};
  assign d_n = {r_p1[7 : 0],g_p1[7 : 4]};
  assign _zz_ChrontelPads_1__16_ = d_p[0];
  assign _zz_ChrontelPads_1__17_ = d_n[0];
  assign _zz_ChrontelPads_1__18_ = 1'b1;
  assign _zz_ChrontelPads_1__19_ = 1'b0;
  always @ (*) begin
    io_pads_d[0] = oDDR2_1__Q;
    io_pads_d[1] = oDDR2_2__Q;
    io_pads_d[2] = oDDR2_3__Q;
    io_pads_d[3] = oDDR2_4__Q;
    io_pads_d[4] = oDDR2_5__Q;
    io_pads_d[5] = oDDR2_6__Q;
    io_pads_d[6] = oDDR2_7__Q;
    io_pads_d[7] = oDDR2_8__Q;
    io_pads_d[8] = oDDR2_9__Q;
    io_pads_d[9] = oDDR2_10__Q;
    io_pads_d[10] = oDDR2_11__Q;
    io_pads_d[11] = oDDR2_12__Q;
  end

  assign _zz_ChrontelPads_1__20_ = d_p[1];
  assign _zz_ChrontelPads_1__21_ = d_n[1];
  assign _zz_ChrontelPads_1__22_ = 1'b1;
  assign _zz_ChrontelPads_1__23_ = 1'b0;
  assign _zz_ChrontelPads_1__24_ = d_p[2];
  assign _zz_ChrontelPads_1__25_ = d_n[2];
  assign _zz_ChrontelPads_1__26_ = 1'b1;
  assign _zz_ChrontelPads_1__27_ = 1'b0;
  assign _zz_ChrontelPads_1__28_ = d_p[3];
  assign _zz_ChrontelPads_1__29_ = d_n[3];
  assign _zz_ChrontelPads_1__30_ = 1'b1;
  assign _zz_ChrontelPads_1__31_ = 1'b0;
  assign _zz_ChrontelPads_1__32_ = d_p[4];
  assign _zz_ChrontelPads_1__33_ = d_n[4];
  assign _zz_ChrontelPads_1__34_ = 1'b1;
  assign _zz_ChrontelPads_1__35_ = 1'b0;
  assign _zz_ChrontelPads_1__36_ = d_p[5];
  assign _zz_ChrontelPads_1__37_ = d_n[5];
  assign _zz_ChrontelPads_1__38_ = 1'b1;
  assign _zz_ChrontelPads_1__39_ = 1'b0;
  assign _zz_ChrontelPads_1__40_ = d_p[6];
  assign _zz_ChrontelPads_1__41_ = d_n[6];
  assign _zz_ChrontelPads_1__42_ = 1'b1;
  assign _zz_ChrontelPads_1__43_ = 1'b0;
  assign _zz_ChrontelPads_1__44_ = d_p[7];
  assign _zz_ChrontelPads_1__45_ = d_n[7];
  assign _zz_ChrontelPads_1__46_ = 1'b1;
  assign _zz_ChrontelPads_1__47_ = 1'b0;
  assign _zz_ChrontelPads_1__48_ = d_p[8];
  assign _zz_ChrontelPads_1__49_ = d_n[8];
  assign _zz_ChrontelPads_1__50_ = 1'b1;
  assign _zz_ChrontelPads_1__51_ = 1'b0;
  assign _zz_ChrontelPads_1__52_ = d_p[9];
  assign _zz_ChrontelPads_1__53_ = d_n[9];
  assign _zz_ChrontelPads_1__54_ = 1'b1;
  assign _zz_ChrontelPads_1__55_ = 1'b0;
  assign _zz_ChrontelPads_1__56_ = d_p[10];
  assign _zz_ChrontelPads_1__57_ = d_n[10];
  assign _zz_ChrontelPads_1__58_ = 1'b1;
  assign _zz_ChrontelPads_1__59_ = 1'b0;
  assign _zz_ChrontelPads_1__60_ = d_p[11];
  assign _zz_ChrontelPads_1__61_ = d_n[11];
  assign _zz_ChrontelPads_1__62_ = 1'b1;
  assign _zz_ChrontelPads_1__63_ = 1'b0;
  always @ (posedge clk) begin
    io_vsync_regNext <= io_vsync;
    io_hsync_regNext <= io_hsync;
    io_de_regNext <= io_de;
    io_r_regNext <= io_r;
    io_g_regNext <= io_g;
    io_b_regNext <= io_b;
  end

endmodule

module MemoryController (
      input   io_clk,
      inout [15:0] io_ddr2a_d,
      output [12:0] io_ddr2a_a,
      output [2:0] io_ddr2a_ba,
      output  io_ddr2a_ras_l,
      output  io_ddr2a_cas_l,
      output  io_ddr2a_we_l,
      output  io_ddr2a_odt,
      output  io_ddr2a_cke,
      output  io_ddr2a_ldm,
      output  io_ddr2a_udm,
      inout  io_ddr2a_rzq,
      inout  io_ddr2a_ldqs_p,
      inout  io_ddr2a_ldqs_n,
      inout  io_ddr2a_udqs_p,
      inout  io_ddr2a_udqs_n,
      output  io_ddr2a_ck_p,
      output  io_ddr2a_ck_n,
      inout [15:0] io_ddr2b_d,
      output [12:0] io_ddr2b_a,
      output [2:0] io_ddr2b_ba,
      output  io_ddr2b_ras_l,
      output  io_ddr2b_cas_l,
      output  io_ddr2b_we_l,
      output  io_ddr2b_odt,
      output  io_ddr2b_cke,
      output  io_ddr2b_ldm,
      output  io_ddr2b_udm,
      inout  io_ddr2b_rzq,
      inout  io_ddr2b_ldqs_p,
      inout  io_ddr2b_ldqs_n,
      inout  io_ddr2b_udqs_p,
      inout  io_ddr2b_udqs_n,
      output  io_ddr2b_ck_p,
      output  io_ddr2b_ck_n,
      input   io_axi1_aw_valid,
      output  io_axi1_aw_ready,
      input  [31:0] io_axi1_aw_payload_addr,
      input  [3:0] io_axi1_aw_payload_id,
      input  [3:0] io_axi1_aw_payload_region,
      input  [7:0] io_axi1_aw_payload_len,
      input  [2:0] io_axi1_aw_payload_size,
      input  [1:0] io_axi1_aw_payload_burst,
      input  [0:0] io_axi1_aw_payload_lock,
      input  [3:0] io_axi1_aw_payload_cache,
      input  [3:0] io_axi1_aw_payload_qos,
      input  [2:0] io_axi1_aw_payload_prot,
      input   io_axi1_w_valid,
      output  io_axi1_w_ready,
      input  [31:0] io_axi1_w_payload_data,
      input  [3:0] io_axi1_w_payload_strb,
      input   io_axi1_w_payload_last,
      output  io_axi1_b_valid,
      input   io_axi1_b_ready,
      output [3:0] io_axi1_b_payload_id,
      output [1:0] io_axi1_b_payload_resp,
      input   io_axi1_ar_valid,
      output  io_axi1_ar_ready,
      input  [31:0] io_axi1_ar_payload_addr,
      input  [3:0] io_axi1_ar_payload_id,
      input  [3:0] io_axi1_ar_payload_region,
      input  [7:0] io_axi1_ar_payload_len,
      input  [2:0] io_axi1_ar_payload_size,
      input  [1:0] io_axi1_ar_payload_burst,
      input  [0:0] io_axi1_ar_payload_lock,
      input  [3:0] io_axi1_ar_payload_cache,
      input  [3:0] io_axi1_ar_payload_qos,
      input  [2:0] io_axi1_ar_payload_prot,
      output  io_axi1_r_valid,
      input   io_axi1_r_ready,
      output [31:0] io_axi1_r_payload_data,
      output [3:0] io_axi1_r_payload_id,
      output [1:0] io_axi1_r_payload_resp,
      output  io_axi1_r_payload_last,
      input   io_axi2_aw_valid,
      output  io_axi2_aw_ready,
      input  [31:0] io_axi2_aw_payload_addr,
      input  [3:0] io_axi2_aw_payload_id,
      input  [3:0] io_axi2_aw_payload_region,
      input  [7:0] io_axi2_aw_payload_len,
      input  [2:0] io_axi2_aw_payload_size,
      input  [1:0] io_axi2_aw_payload_burst,
      input  [0:0] io_axi2_aw_payload_lock,
      input  [3:0] io_axi2_aw_payload_cache,
      input  [3:0] io_axi2_aw_payload_qos,
      input  [2:0] io_axi2_aw_payload_prot,
      input   io_axi2_w_valid,
      output  io_axi2_w_ready,
      input  [31:0] io_axi2_w_payload_data,
      input  [3:0] io_axi2_w_payload_strb,
      input   io_axi2_w_payload_last,
      output  io_axi2_b_valid,
      input   io_axi2_b_ready,
      output [3:0] io_axi2_b_payload_id,
      output [1:0] io_axi2_b_payload_resp,
      input   io_axi2_ar_valid,
      output  io_axi2_ar_ready,
      input  [31:0] io_axi2_ar_payload_addr,
      input  [3:0] io_axi2_ar_payload_id,
      input  [3:0] io_axi2_ar_payload_region,
      input  [7:0] io_axi2_ar_payload_len,
      input  [2:0] io_axi2_ar_payload_size,
      input  [1:0] io_axi2_ar_payload_burst,
      input  [0:0] io_axi2_ar_payload_lock,
      input  [3:0] io_axi2_ar_payload_cache,
      input  [3:0] io_axi2_ar_payload_qos,
      input  [2:0] io_axi2_ar_payload_prot,
      output  io_axi2_r_valid,
      input   io_axi2_r_ready,
      output [31:0] io_axi2_r_payload_data,
      output [3:0] io_axi2_r_payload_id,
      output [1:0] io_axi2_r_payload_resp,
      output  io_axi2_r_payload_last);
  wire  _zz_MemoryController_1_;
  wire  _zz_MemoryController_2_;
  wire  _zz_MemoryController_3_;
  wire [3:0] _zz_MemoryController_4_;
  wire [31:0] _zz_MemoryController_5_;
  wire [7:0] _zz_MemoryController_6_;
  wire [2:0] _zz_MemoryController_7_;
  wire [127:0] _zz_MemoryController_8_;
  wire [15:0] _zz_MemoryController_9_;
  wire [3:0] _zz_MemoryController_10_;
  wire [31:0] _zz_MemoryController_11_;
  wire [7:0] _zz_MemoryController_12_;
  wire [2:0] _zz_MemoryController_13_;
  wire  _zz_MemoryController_14_;
  wire [3:0] _zz_MemoryController_15_;
  wire [31:0] _zz_MemoryController_16_;
  wire [7:0] _zz_MemoryController_17_;
  wire [2:0] _zz_MemoryController_18_;
  wire [127:0] _zz_MemoryController_19_;
  wire [15:0] _zz_MemoryController_20_;
  wire [3:0] _zz_MemoryController_21_;
  wire [31:0] _zz_MemoryController_22_;
  wire [7:0] _zz_MemoryController_23_;
  wire [2:0] _zz_MemoryController_24_;
  wire [12:0] mig_1__mcb1_dram_a;
  wire [2:0] mig_1__mcb1_dram_ba;
  wire  mig_1__mcb1_dram_ras_n;
  wire  mig_1__mcb1_dram_cas_n;
  wire  mig_1__mcb1_dram_we_n;
  wire  mig_1__mcb1_dram_odt;
  wire  mig_1__mcb1_dram_cke;
  wire  mig_1__mcb1_dram_dm;
  wire  mig_1__mcb1_dram_udm;
  wire  mig_1__c1_calib_done;
  wire  mig_1__c1_clk0;
  wire  mig_1__c1_rst0;
  wire  mig_1__mcb1_dram_ck;
  wire  mig_1__mcb1_dram_ck_n;
  wire [12:0] mig_1__mcb3_dram_a;
  wire [2:0] mig_1__mcb3_dram_ba;
  wire  mig_1__mcb3_dram_ras_n;
  wire  mig_1__mcb3_dram_cas_n;
  wire  mig_1__mcb3_dram_we_n;
  wire  mig_1__mcb3_dram_odt;
  wire  mig_1__mcb3_dram_cke;
  wire  mig_1__mcb3_dram_dm;
  wire  mig_1__mcb3_dram_udm;
  wire  mig_1__c3_calib_done;
  wire  mig_1__c3_clk0;
  wire  mig_1__c3_rst0;
  wire  mig_1__mcb3_dram_ck;
  wire  mig_1__mcb3_dram_ck_n;
  wire  mig_1__c1_s0_axi_awready;
  wire  mig_1__c1_s0_axi_wready;
  wire [3:0] mig_1__c1_s0_axi_bid;
  wire [3:0] mig_1__c1_s0_axi_wid;
  wire [1:0] mig_1__c1_s0_axi_bresp;
  wire  mig_1__c1_s0_axi_bvalid;
  wire  mig_1__c1_s0_axi_arready;
  wire [3:0] mig_1__c1_s0_axi_rid;
  wire [127:0] mig_1__c1_s0_axi_rdata;
  wire [1:0] mig_1__c1_s0_axi_rresp;
  wire  mig_1__c1_s0_axi_rlast;
  wire  mig_1__c1_s0_axi_rvalid;
  wire  mig_1__c3_s0_axi_awready;
  wire  mig_1__c3_s0_axi_wready;
  wire [3:0] mig_1__c3_s0_axi_bid;
  wire [3:0] mig_1__c3_s0_axi_wid;
  wire [1:0] mig_1__c3_s0_axi_bresp;
  wire  mig_1__c3_s0_axi_bvalid;
  wire  mig_1__c3_s0_axi_arready;
  wire [3:0] mig_1__c3_s0_axi_rid;
  wire [127:0] mig_1__c3_s0_axi_rdata;
  wire [1:0] mig_1__c3_s0_axi_rresp;
  wire  mig_1__c3_s0_axi_rlast;
  wire  mig_1__c3_s0_axi_rvalid;
  MIG #( 
    .C1_S0_AXI_ID_WIDTH(4),
    .C1_S0_AXI_ADDR_WIDTH(32),
    .C1_S0_AXI_DATA_WIDTH(128),
    .C3_S0_AXI_ID_WIDTH(4),
    .C3_S0_AXI_ADDR_WIDTH(32),
    .C3_S0_AXI_DATA_WIDTH(128) 
  ) mig_1_ ( 
    .mcb1_dram_dq(io_ddr2a_d),
    .mcb1_dram_a(mig_1__mcb1_dram_a),
    .mcb1_dram_ba(mig_1__mcb1_dram_ba),
    .mcb1_dram_ras_n(mig_1__mcb1_dram_ras_n),
    .mcb1_dram_cas_n(mig_1__mcb1_dram_cas_n),
    .mcb1_dram_we_n(mig_1__mcb1_dram_we_n),
    .mcb1_dram_odt(mig_1__mcb1_dram_odt),
    .mcb1_dram_cke(mig_1__mcb1_dram_cke),
    .mcb1_dram_dm(mig_1__mcb1_dram_dm),
    .mcb1_dram_udm(mig_1__mcb1_dram_udm),
    .mcb1_rzq(io_ddr2a_rzq),
    .c1_sys_clk(io_clk),
    .c1_sys_rst_i(_zz_MemoryController_1_),
    .c1_calib_done(mig_1__c1_calib_done),
    .c1_clk0(mig_1__c1_clk0),
    .c1_rst0(mig_1__c1_rst0),
    .mcb1_dram_dqs(io_ddr2a_ldqs_p),
    .mcb1_dram_dqs_n(io_ddr2a_ldqs_n),
    .mcb1_dram_udqs(io_ddr2a_udqs_p),
    .mcb1_dram_udqs_n(io_ddr2a_udqs_n),
    .mcb1_dram_ck(mig_1__mcb1_dram_ck),
    .mcb1_dram_ck_n(mig_1__mcb1_dram_ck_n),
    .mcb3_dram_dq(io_ddr2b_d),
    .mcb3_dram_a(mig_1__mcb3_dram_a),
    .mcb3_dram_ba(mig_1__mcb3_dram_ba),
    .mcb3_dram_ras_n(mig_1__mcb3_dram_ras_n),
    .mcb3_dram_cas_n(mig_1__mcb3_dram_cas_n),
    .mcb3_dram_we_n(mig_1__mcb3_dram_we_n),
    .mcb3_dram_odt(mig_1__mcb3_dram_odt),
    .mcb3_dram_cke(mig_1__mcb3_dram_cke),
    .mcb3_dram_dm(mig_1__mcb3_dram_dm),
    .mcb3_dram_udm(mig_1__mcb3_dram_udm),
    .mcb3_rzq(io_ddr2b_rzq),
    .c3_sys_clk(io_clk),
    .c3_sys_rst_i(_zz_MemoryController_2_),
    .c3_calib_done(mig_1__c3_calib_done),
    .c3_clk0(mig_1__c3_clk0),
    .c3_rst0(mig_1__c3_rst0),
    .mcb3_dram_dqs(io_ddr2b_ldqs_p),
    .mcb3_dram_dqs_n(io_ddr2b_ldqs_n),
    .mcb3_dram_udqs(io_ddr2b_udqs_p),
    .mcb3_dram_udqs_n(io_ddr2b_udqs_n),
    .mcb3_dram_ck(mig_1__mcb3_dram_ck),
    .mcb3_dram_ck_n(mig_1__mcb3_dram_ck_n),
    .c1_s0_axi_aclk(io_clk),
    .c1_s0_axi_aresetn(_zz_MemoryController_3_),
    .c1_s0_axi_awid(_zz_MemoryController_4_),
    .c1_s0_axi_awaddr(_zz_MemoryController_5_),
    .c1_s0_axi_awlen(_zz_MemoryController_6_),
    .c1_s0_axi_awsize(_zz_MemoryController_7_),
    .c1_s0_axi_awburst(io_axi1_aw_payload_burst),
    .c1_s0_axi_awlock(io_axi1_aw_payload_lock),
    .c1_s0_axi_awcache(io_axi1_aw_payload_cache),
    .c1_s0_axi_awprot(io_axi1_aw_payload_prot),
    .c1_s0_axi_awqos(io_axi1_aw_payload_qos),
    .c1_s0_axi_awvalid(io_axi1_aw_valid),
    .c1_s0_axi_awready(mig_1__c1_s0_axi_awready),
    .c1_s0_axi_wdata(_zz_MemoryController_8_),
    .c1_s0_axi_wstrb(_zz_MemoryController_9_),
    .c1_s0_axi_wlast(io_axi1_w_payload_last),
    .c1_s0_axi_wvalid(io_axi1_w_valid),
    .c1_s0_axi_wready(mig_1__c1_s0_axi_wready),
    .c1_s0_axi_bid(mig_1__c1_s0_axi_bid),
    .c1_s0_axi_wid(mig_1__c1_s0_axi_wid),
    .c1_s0_axi_bresp(mig_1__c1_s0_axi_bresp),
    .c1_s0_axi_bvalid(mig_1__c1_s0_axi_bvalid),
    .c1_s0_axi_bready(io_axi1_b_ready),
    .c1_s0_axi_arid(_zz_MemoryController_10_),
    .c1_s0_axi_araddr(_zz_MemoryController_11_),
    .c1_s0_axi_arlen(_zz_MemoryController_12_),
    .c1_s0_axi_arsize(_zz_MemoryController_13_),
    .c1_s0_axi_arburst(io_axi1_ar_payload_burst),
    .c1_s0_axi_arlock(io_axi1_ar_payload_lock),
    .c1_s0_axi_arcache(io_axi1_ar_payload_cache),
    .c1_s0_axi_arprot(io_axi1_ar_payload_prot),
    .c1_s0_axi_arqos(io_axi1_ar_payload_qos),
    .c1_s0_axi_arvalid(io_axi1_ar_valid),
    .c1_s0_axi_arready(mig_1__c1_s0_axi_arready),
    .c1_s0_axi_rid(mig_1__c1_s0_axi_rid),
    .c1_s0_axi_rdata(mig_1__c1_s0_axi_rdata),
    .c1_s0_axi_rresp(mig_1__c1_s0_axi_rresp),
    .c1_s0_axi_rlast(mig_1__c1_s0_axi_rlast),
    .c1_s0_axi_rvalid(mig_1__c1_s0_axi_rvalid),
    .c1_s0_axi_rready(io_axi1_r_ready),
    .c3_s0_axi_aclk(io_clk),
    .c3_s0_axi_aresetn(_zz_MemoryController_14_),
    .c3_s0_axi_awid(_zz_MemoryController_15_),
    .c3_s0_axi_awaddr(_zz_MemoryController_16_),
    .c3_s0_axi_awlen(_zz_MemoryController_17_),
    .c3_s0_axi_awsize(_zz_MemoryController_18_),
    .c3_s0_axi_awburst(io_axi2_aw_payload_burst),
    .c3_s0_axi_awlock(io_axi2_aw_payload_lock),
    .c3_s0_axi_awcache(io_axi2_aw_payload_cache),
    .c3_s0_axi_awprot(io_axi2_aw_payload_prot),
    .c3_s0_axi_awqos(io_axi2_aw_payload_qos),
    .c3_s0_axi_awvalid(io_axi2_aw_valid),
    .c3_s0_axi_awready(mig_1__c3_s0_axi_awready),
    .c3_s0_axi_wdata(_zz_MemoryController_19_),
    .c3_s0_axi_wstrb(_zz_MemoryController_20_),
    .c3_s0_axi_wlast(io_axi2_w_payload_last),
    .c3_s0_axi_wvalid(io_axi2_w_valid),
    .c3_s0_axi_wready(mig_1__c3_s0_axi_wready),
    .c3_s0_axi_bid(mig_1__c3_s0_axi_bid),
    .c3_s0_axi_wid(mig_1__c3_s0_axi_wid),
    .c3_s0_axi_bresp(mig_1__c3_s0_axi_bresp),
    .c3_s0_axi_bvalid(mig_1__c3_s0_axi_bvalid),
    .c3_s0_axi_bready(io_axi2_b_ready),
    .c3_s0_axi_arid(_zz_MemoryController_21_),
    .c3_s0_axi_araddr(_zz_MemoryController_22_),
    .c3_s0_axi_arlen(_zz_MemoryController_23_),
    .c3_s0_axi_arsize(_zz_MemoryController_24_),
    .c3_s0_axi_arburst(io_axi2_ar_payload_burst),
    .c3_s0_axi_arlock(io_axi2_ar_payload_lock),
    .c3_s0_axi_arcache(io_axi2_ar_payload_cache),
    .c3_s0_axi_arprot(io_axi2_ar_payload_prot),
    .c3_s0_axi_arqos(io_axi2_ar_payload_qos),
    .c3_s0_axi_arvalid(io_axi2_ar_valid),
    .c3_s0_axi_arready(mig_1__c3_s0_axi_arready),
    .c3_s0_axi_rid(mig_1__c3_s0_axi_rid),
    .c3_s0_axi_rdata(mig_1__c3_s0_axi_rdata),
    .c3_s0_axi_rresp(mig_1__c3_s0_axi_rresp),
    .c3_s0_axi_rlast(mig_1__c3_s0_axi_rlast),
    .c3_s0_axi_rvalid(mig_1__c3_s0_axi_rvalid),
    .c3_s0_axi_rready(io_axi2_r_ready) 
  );
  assign io_ddr2a_a = mig_1__mcb1_dram_a;
  assign io_ddr2a_ba = mig_1__mcb1_dram_ba;
  assign io_ddr2a_ras_l = mig_1__mcb1_dram_ras_n;
  assign io_ddr2a_cas_l = mig_1__mcb1_dram_cas_n;
  assign io_ddr2a_we_l = mig_1__mcb1_dram_we_n;
  assign io_ddr2a_odt = mig_1__mcb1_dram_odt;
  assign io_ddr2a_cke = mig_1__mcb1_dram_cke;
  assign io_ddr2a_ldm = mig_1__mcb1_dram_dm;
  assign io_ddr2a_udm = mig_1__mcb1_dram_udm;
  assign io_ddr2a_ck_p = mig_1__mcb1_dram_ck;
  assign io_ddr2a_ck_n = mig_1__mcb1_dram_ck_n;
  assign io_ddr2b_a = mig_1__mcb3_dram_a;
  assign io_ddr2b_ba = mig_1__mcb3_dram_ba;
  assign io_ddr2b_ras_l = mig_1__mcb3_dram_ras_n;
  assign io_ddr2b_cas_l = mig_1__mcb3_dram_cas_n;
  assign io_ddr2b_we_l = mig_1__mcb3_dram_we_n;
  assign io_ddr2b_odt = mig_1__mcb3_dram_odt;
  assign io_ddr2b_cke = mig_1__mcb3_dram_cke;
  assign io_ddr2b_ldm = mig_1__mcb3_dram_dm;
  assign io_ddr2b_udm = mig_1__mcb3_dram_udm;
  assign io_ddr2b_ck_p = mig_1__mcb3_dram_ck;
  assign io_ddr2b_ck_n = mig_1__mcb3_dram_ck_n;
  assign _zz_MemoryController_1_ = 1'b1;
  assign _zz_MemoryController_2_ = 1'b1;
  assign _zz_MemoryController_3_ = 1'b1;
  assign _zz_MemoryController_4_ = io_axi1_aw_payload_id;
  assign _zz_MemoryController_5_ = io_axi1_aw_payload_addr;
  assign _zz_MemoryController_6_ = io_axi1_aw_payload_len;
  assign _zz_MemoryController_7_ = io_axi1_aw_payload_size;
  assign io_axi1_aw_ready = mig_1__c1_s0_axi_awready;
  assign _zz_MemoryController_8_ = {96'd0, io_axi1_w_payload_data};
  assign _zz_MemoryController_9_ = {12'd0, io_axi1_w_payload_strb};
  assign io_axi1_w_ready = mig_1__c1_s0_axi_wready;
  assign io_axi1_b_payload_id = mig_1__c1_s0_axi_bid;
  assign io_axi1_b_payload_resp = mig_1__c1_s0_axi_bresp;
  assign io_axi1_b_valid = mig_1__c1_s0_axi_bvalid;
  assign _zz_MemoryController_10_ = io_axi1_ar_payload_id;
  assign _zz_MemoryController_11_ = io_axi1_ar_payload_addr;
  assign _zz_MemoryController_12_ = io_axi1_ar_payload_len;
  assign _zz_MemoryController_13_ = io_axi1_ar_payload_size;
  assign io_axi1_ar_ready = mig_1__c1_s0_axi_arready;
  assign io_axi1_r_payload_id = mig_1__c1_s0_axi_rid;
  assign io_axi1_r_payload_data = mig_1__c1_s0_axi_rdata[31:0];
  assign io_axi1_r_payload_resp = mig_1__c1_s0_axi_rresp;
  assign io_axi1_r_payload_last = mig_1__c1_s0_axi_rlast;
  assign io_axi1_r_valid = mig_1__c1_s0_axi_rvalid;
  assign _zz_MemoryController_14_ = 1'b1;
  assign _zz_MemoryController_15_ = io_axi2_aw_payload_id;
  assign _zz_MemoryController_16_ = io_axi2_aw_payload_addr;
  assign _zz_MemoryController_17_ = io_axi2_aw_payload_len;
  assign _zz_MemoryController_18_ = io_axi2_aw_payload_size;
  assign io_axi2_aw_ready = mig_1__c3_s0_axi_awready;
  assign _zz_MemoryController_19_ = {96'd0, io_axi2_w_payload_data};
  assign _zz_MemoryController_20_ = {12'd0, io_axi2_w_payload_strb};
  assign io_axi2_w_ready = mig_1__c3_s0_axi_wready;
  assign io_axi2_b_payload_id = mig_1__c3_s0_axi_bid;
  assign io_axi2_b_payload_resp = mig_1__c3_s0_axi_bresp;
  assign io_axi2_b_valid = mig_1__c3_s0_axi_bvalid;
  assign _zz_MemoryController_21_ = io_axi2_ar_payload_id;
  assign _zz_MemoryController_22_ = io_axi2_ar_payload_addr;
  assign _zz_MemoryController_23_ = io_axi2_ar_payload_len;
  assign _zz_MemoryController_24_ = io_axi2_ar_payload_size;
  assign io_axi2_ar_ready = mig_1__c3_s0_axi_arready;
  assign io_axi2_r_payload_id = mig_1__c3_s0_axi_rid;
  assign io_axi2_r_payload_data = mig_1__c3_s0_axi_rdata[31:0];
  assign io_axi2_r_payload_resp = mig_1__c3_s0_axi_rresp;
  assign io_axi2_r_payload_last = mig_1__c3_s0_axi_rlast;
  assign io_axi2_r_valid = mig_1__c3_s0_axi_rvalid;
endmodule

module Pano (
      input   osc_clk,
      output  led_red,
      output  led_green,
      output  led_blue,
      input   pano_button,
      output  dvi_reset_,
      output  dvi_xclk_p,
      output  dvi_xclk_n,
      output  dvi_v,
      output  dvi_h,
      output  dvi_de,
      output [11:0] dvi_d,
      output  hdmi_reset_,
      output  hdmi_xclk_p,
      output  hdmi_v,
      output  hdmi_h,
      output  hdmi_de,
      output [11:0] hdmi_d,
      output  gmii_rst_,
      input   gmii_rx_clk,
      input   gmii_rx_dv,
      input   gmii_rx_er,
      input  [7:0] gmii_rx_d,
      input   gmii_tx_gclk,
      input   gmii_tx_clk,
      output  gmii_tx_en,
      output  gmii_tx_er,
      output [7:0] gmii_tx_d,
      input   gmii_col,
      input   gmii_crs,
      output  gmii_mdio_mdc,
      output  usb_reset_,
      output  usb_clk,
      input   ulpi_clk,
      input   ulpi_direction,
      output  ulpi_stp,
      input   ulpi_nxt,
      output  ulpi_reset,
      inout [15:0] ddr2a_d,
      output [12:0] ddr2a_a,
      output [2:0] ddr2a_ba,
      output  ddr2a_ras_l,
      output  ddr2a_cas_l,
      output  ddr2a_we_l,
      output  ddr2a_odt,
      output  ddr2a_cke,
      output  ddr2a_ldm,
      output  ddr2a_udm,
      inout  ddr2a_rzq,
      inout  ddr2a_ldqs_p,
      inout  ddr2a_ldqs_n,
      inout  ddr2a_udqs_p,
      inout  ddr2a_udqs_n,
      output  ddr2a_ck_p,
      output  ddr2a_ck_n,
      inout [15:0] ddr2b_d,
      output [12:0] ddr2b_a,
      output [2:0] ddr2b_ba,
      output  ddr2b_ras_l,
      output  ddr2b_cas_l,
      output  ddr2b_we_l,
      output  ddr2b_odt,
      output  ddr2b_cke,
      output  ddr2b_ldm,
      output  ddr2b_udm,
      inout  ddr2b_rzq,
      inout  ddr2b_ldqs_p,
      inout  ddr2b_ldqs_n,
      inout  ddr2b_udqs_p,
      inout  ddr2b_udqs_n,
      output  ddr2b_ck_p,
      output  ddr2b_ck_n,
      output [0:0] spi_ss,
      output  spi_sclk,
      output  spi_mosi,
      input   spi_miso,
      inout  dvi_spc,
      inout  dvi_spd,
      inout  gmii_mdio_mdio,
      inout [7:0] ulpi_data);
  wire  _zz_Pano_26_;
  wire  _zz_Pano_27_;
  wire  _zz_Pano_28_;
  wire  _zz_Pano_29_;
  wire  _zz_Pano_30_;
  wire  _zz_Pano_31_;
  wire  _zz_Pano_32_;
  wire  _zz_Pano_33_;
  wire  _zz_Pano_34_;
  wire  _zz_Pano_35_;
  wire  _zz_Pano_36_;
  wire  _zz_Pano_37_;
  wire  _zz_Pano_38_;
  wire  _zz_Pano_39_;
  wire  _zz_Pano_40_;
  wire  u_main_clk_gen_u_main_clk_pll_CLKFX;
  wire  u_main_clk_gen_u_main_clk_pll_CLKFX180;
  wire  u_main_clk_gen_u_main_clk_pll_CLKFXDV;
  wire  u_main_clk_gen_u_main_clk_pll_LOCKED;
  wire [1:0] u_main_clk_gen_u_main_clk_pll_STATUS;
  wire  u_main_clk_gen_u_main_clk_pll_PROGDONE;
  wire  u_vo_clk_gen_u_vo_clk_pll_CLKFX;
  wire  u_vo_clk_gen_u_vo_clk_pll_CLKFX180;
  wire  u_vo_clk_gen_u_vo_clk_pll_CLKFXDV;
  wire  u_vo_clk_gen_u_vo_clk_pll_LOCKED;
  wire [1:0] u_vo_clk_gen_u_vo_clk_pll_STATUS;
  wire  u_vo_clk_gen_u_vo_clk_pll_PROGDONE;
  wire  u_usb_clk_gen_u_usb_clk_pll_CLKFX;
  wire  u_usb_clk_gen_u_usb_clk_pll_CLKFX180;
  wire  u_usb_clk_gen_u_usb_clk_pll_CLKFXDV;
  wire  u_usb_clk_gen_u_usb_clk_pll_LOCKED;
  wire [1:0] u_usb_clk_gen_u_usb_clk_pll_STATUS;
  wire  u_usb_clk_gen_u_usb_clk_pll_PROGDONE;
  wire  core_u_pano_core_io_led_red;
  wire  core_u_pano_core_io_led_green;
  wire  core_u_pano_core_io_led_blue;
  wire  core_u_pano_core_io_dvi_ctrl_scl_write;
  wire  core_u_pano_core_io_dvi_ctrl_scl_writeEnable;
  wire  core_u_pano_core_io_dvi_ctrl_sda_write;
  wire  core_u_pano_core_io_dvi_ctrl_sda_writeEnable;
  wire  core_u_pano_core_io_gmii_tx_en;
  wire  core_u_pano_core_io_gmii_tx_er;
  wire [7:0] core_u_pano_core_io_gmii_tx_d;
  wire  core_u_pano_core_io_gmii_mdio_mdc;
  wire  core_u_pano_core_io_gmii_mdio_mdio_write;
  wire  core_u_pano_core_io_gmii_mdio_mdio_writeEnable;
  wire [7:0] core_u_pano_core_io_ulpi_data_write;
  wire [7:0] core_u_pano_core_io_ulpi_data_writeEnable;
  wire  core_u_pano_core_io_ulpi_stp;
  wire  core_u_pano_core_io_ulpi_reset;
  wire  core_u_pano_core_io_vo_vsync;
  wire  core_u_pano_core_io_vo_hsync;
  wire  core_u_pano_core_io_vo_blank_;
  wire  core_u_pano_core_io_vo_de;
  wire [7:0] core_u_pano_core_io_vo_r;
  wire [7:0] core_u_pano_core_io_vo_g;
  wire [7:0] core_u_pano_core_io_vo_b;
  wire  core_u_pano_core_io_axi1_ar_valid;
  wire [31:0] core_u_pano_core_io_axi1_ar_payload_addr;
  wire [3:0] core_u_pano_core_io_axi1_ar_payload_id;
  wire [3:0] core_u_pano_core_io_axi1_ar_payload_region;
  wire [7:0] core_u_pano_core_io_axi1_ar_payload_len;
  wire [2:0] core_u_pano_core_io_axi1_ar_payload_size;
  wire [1:0] core_u_pano_core_io_axi1_ar_payload_burst;
  wire [0:0] core_u_pano_core_io_axi1_ar_payload_lock;
  wire [3:0] core_u_pano_core_io_axi1_ar_payload_cache;
  wire [3:0] core_u_pano_core_io_axi1_ar_payload_qos;
  wire [2:0] core_u_pano_core_io_axi1_ar_payload_prot;
  wire  core_u_pano_core_io_axi1_aw_valid;
  wire [31:0] core_u_pano_core_io_axi1_aw_payload_addr;
  wire [3:0] core_u_pano_core_io_axi1_aw_payload_id;
  wire [3:0] core_u_pano_core_io_axi1_aw_payload_region;
  wire [7:0] core_u_pano_core_io_axi1_aw_payload_len;
  wire [2:0] core_u_pano_core_io_axi1_aw_payload_size;
  wire [1:0] core_u_pano_core_io_axi1_aw_payload_burst;
  wire [0:0] core_u_pano_core_io_axi1_aw_payload_lock;
  wire [3:0] core_u_pano_core_io_axi1_aw_payload_cache;
  wire [3:0] core_u_pano_core_io_axi1_aw_payload_qos;
  wire [2:0] core_u_pano_core_io_axi1_aw_payload_prot;
  wire  core_u_pano_core_io_axi1_w_valid;
  wire [31:0] core_u_pano_core_io_axi1_w_payload_data;
  wire [3:0] core_u_pano_core_io_axi1_w_payload_strb;
  wire  core_u_pano_core_io_axi1_w_payload_last;
  wire  core_u_pano_core_io_axi1_r_ready;
  wire  core_u_pano_core_io_axi1_b_ready;
  wire  core_u_pano_core_io_axi2_ar_valid;
  wire [31:0] core_u_pano_core_io_axi2_ar_payload_addr;
  wire [3:0] core_u_pano_core_io_axi2_ar_payload_id;
  wire [3:0] core_u_pano_core_io_axi2_ar_payload_region;
  wire [7:0] core_u_pano_core_io_axi2_ar_payload_len;
  wire [2:0] core_u_pano_core_io_axi2_ar_payload_size;
  wire [1:0] core_u_pano_core_io_axi2_ar_payload_burst;
  wire [0:0] core_u_pano_core_io_axi2_ar_payload_lock;
  wire [3:0] core_u_pano_core_io_axi2_ar_payload_cache;
  wire [3:0] core_u_pano_core_io_axi2_ar_payload_qos;
  wire [2:0] core_u_pano_core_io_axi2_ar_payload_prot;
  wire  core_u_pano_core_io_axi2_aw_valid;
  wire [31:0] core_u_pano_core_io_axi2_aw_payload_addr;
  wire [3:0] core_u_pano_core_io_axi2_aw_payload_id;
  wire [3:0] core_u_pano_core_io_axi2_aw_payload_region;
  wire [7:0] core_u_pano_core_io_axi2_aw_payload_len;
  wire [2:0] core_u_pano_core_io_axi2_aw_payload_size;
  wire [1:0] core_u_pano_core_io_axi2_aw_payload_burst;
  wire [0:0] core_u_pano_core_io_axi2_aw_payload_lock;
  wire [3:0] core_u_pano_core_io_axi2_aw_payload_cache;
  wire [3:0] core_u_pano_core_io_axi2_aw_payload_qos;
  wire [2:0] core_u_pano_core_io_axi2_aw_payload_prot;
  wire  core_u_pano_core_io_axi2_w_valid;
  wire [31:0] core_u_pano_core_io_axi2_w_payload_data;
  wire [3:0] core_u_pano_core_io_axi2_w_payload_strb;
  wire  core_u_pano_core_io_axi2_w_payload_last;
  wire  core_u_pano_core_io_axi2_r_ready;
  wire  core_u_pano_core_io_axi2_b_ready;
  wire  core_u_pano_core_io_spi_sclk;
  wire  core_u_pano_core_io_spi_mosi;
  wire [0:0] core_u_pano_core_io_spi_ss;
  wire  core_u_dvi_io_pads_reset_;
  wire  core_u_dvi_io_pads_xclk_p;
  wire  core_u_dvi_io_pads_xclk_n;
  wire  core_u_dvi_io_pads_v;
  wire  core_u_dvi_io_pads_h;
  wire  core_u_dvi_io_pads_de;
  wire [11:0] core_u_dvi_io_pads_d;
  wire  core_u_hdmi_io_pads_reset_;
  wire  core_u_hdmi_io_pads_xclk_p;
  wire  core_u_hdmi_io_pads_v;
  wire  core_u_hdmi_io_pads_h;
  wire  core_u_hdmi_io_pads_de;
  wire [11:0] core_u_hdmi_io_pads_d;
  wire [12:0] core_mem_io_ddr2a_a;
  wire [2:0] core_mem_io_ddr2a_ba;
  wire  core_mem_io_ddr2a_ras_l;
  wire  core_mem_io_ddr2a_cas_l;
  wire  core_mem_io_ddr2a_we_l;
  wire  core_mem_io_ddr2a_odt;
  wire  core_mem_io_ddr2a_cke;
  wire  core_mem_io_ddr2a_ldm;
  wire  core_mem_io_ddr2a_udm;
  wire  core_mem_io_ddr2a_ck_p;
  wire  core_mem_io_ddr2a_ck_n;
  wire [12:0] core_mem_io_ddr2b_a;
  wire [2:0] core_mem_io_ddr2b_ba;
  wire  core_mem_io_ddr2b_ras_l;
  wire  core_mem_io_ddr2b_cas_l;
  wire  core_mem_io_ddr2b_we_l;
  wire  core_mem_io_ddr2b_odt;
  wire  core_mem_io_ddr2b_cke;
  wire  core_mem_io_ddr2b_ldm;
  wire  core_mem_io_ddr2b_udm;
  wire  core_mem_io_ddr2b_ck_p;
  wire  core_mem_io_ddr2b_ck_n;
  wire  core_mem_io_axi1_ar_ready;
  wire  core_mem_io_axi1_aw_ready;
  wire  core_mem_io_axi1_w_ready;
  wire  core_mem_io_axi1_r_valid;
  wire [31:0] core_mem_io_axi1_r_payload_data;
  wire [3:0] core_mem_io_axi1_r_payload_id;
  wire [1:0] core_mem_io_axi1_r_payload_resp;
  wire  core_mem_io_axi1_r_payload_last;
  wire  core_mem_io_axi1_b_valid;
  wire [3:0] core_mem_io_axi1_b_payload_id;
  wire [1:0] core_mem_io_axi1_b_payload_resp;
  wire  core_mem_io_axi2_ar_ready;
  wire  core_mem_io_axi2_aw_ready;
  wire  core_mem_io_axi2_w_ready;
  wire  core_mem_io_axi2_r_valid;
  wire [31:0] core_mem_io_axi2_r_payload_data;
  wire [3:0] core_mem_io_axi2_r_payload_id;
  wire [1:0] core_mem_io_axi2_r_payload_resp;
  wire  core_mem_io_axi2_r_payload_last;
  wire  core_mem_io_axi2_b_valid;
  wire [3:0] core_mem_io_axi2_b_payload_id;
  wire [1:0] core_mem_io_axi2_b_payload_resp;
  wire  _zz_Pano_41_;
  wire  _zz_Pano_42_;
  reg  _zz_Pano_1_;
  reg  _zz_Pano_2_;
  reg  _zz_Pano_3_;
  reg  _zz_Pano_4_;
  reg  _zz_Pano_5_;
  reg  _zz_Pano_6_;
  reg  _zz_Pano_7_;
  reg  _zz_Pano_8_;
  reg  _zz_Pano_9_;
  reg  _zz_Pano_10_;
  reg  _zz_Pano_11_;
  wire  _zz_Pano_12_;
  wire  _zz_Pano_13_;
  wire  _zz_Pano_14_;
  wire  _zz_Pano_15_;
  wire  _zz_Pano_16_;
  wire  _zz_Pano_17_;
  wire  _zz_Pano_18_;
  wire  _zz_Pano_19_;
  wire  _zz_Pano_20_;
  wire [7:0] _zz_Pano_21_;
  wire [7:0] _zz_Pano_22_;
  wire [7:0] _zz_Pano_23_;
  wire  main_clk_raw;
  wire  main_reset_;
  reg  main_reset_gen_reset_unbuffered_;
  reg [4:0] main_reset_gen_reset_cntr = (5'b00000);
  wire [4:0] _zz_Pano_24_;
  reg  main_reset_gen_reset_unbuffered__regNext;
  wire  main_clk;
  wire  u_vo_clk_gen_vo_clk;
  wire  u_vo_clk_gen_vo_reset_;
  reg  u_vo_clk_gen_vo_reset_gen_reset_unbuffered_;
  reg [4:0] u_vo_clk_gen_vo_reset_gen_reset_cntr = (5'b00000);
  wire [4:0] _zz_Pano_25_;
  reg  u_vo_clk_gen_vo_reset_gen_reset_unbuffered__regNext;
  reg [23:0] gmii_rx_green_counter;
  reg [23:0] core_red_counter;
  wire  core_vo_vsync;
  wire  core_vo_hsync;
  wire  core_vo_blank_;
  wire  core_vo_de;
  wire [7:0] core_vo_r;
  wire [7:0] core_vo_g;
  wire [7:0] core_vo_b;
  assign _zz_Pano_41_ = (main_reset_gen_reset_cntr != _zz_Pano_24_);
  assign _zz_Pano_42_ = (u_vo_clk_gen_vo_reset_gen_reset_cntr != _zz_Pano_25_);
  DCM_CLKGEN #( 
    .CLKFX_DIVIDE(20),
    .CLKFXDV_DIVIDE(2),
    .CLKFX_MD_MAX(0.0),
    .CLKFX_MULTIPLY(4),
    .CLKIN_PERIOD("8.0"),
    .SPREAD_SPECTRUM("NONE"),
    .STARTUP_WAIT(1'b0) 
  ) u_main_clk_gen_u_main_clk_pll ( 
    .CLKIN(osc_clk),
    .CLKFX(u_main_clk_gen_u_main_clk_pll_CLKFX),
    .CLKFX180(u_main_clk_gen_u_main_clk_pll_CLKFX180),
    .CLKFXDV(u_main_clk_gen_u_main_clk_pll_CLKFXDV),
    .RST(_zz_Pano_26_),
    .FREEZEDCM(_zz_Pano_27_),
    .LOCKED(u_main_clk_gen_u_main_clk_pll_LOCKED),
    .STATUS(u_main_clk_gen_u_main_clk_pll_STATUS),
    .PROGCLK(_zz_Pano_28_),
    .PROGDATA(_zz_Pano_29_),
    .PROGEN(_zz_Pano_30_),
    .PROGDONE(u_main_clk_gen_u_main_clk_pll_PROGDONE) 
  );
  DCM_CLKGEN #( 
    .CLKFX_DIVIDE(125),
    .CLKFXDV_DIVIDE(2),
    .CLKFX_MD_MAX(0.0),
    .CLKFX_MULTIPLY(148),
    .CLKIN_PERIOD("8.0"),
    .SPREAD_SPECTRUM("NONE"),
    .STARTUP_WAIT(1'b0) 
  ) u_vo_clk_gen_u_vo_clk_pll ( 
    .CLKIN(osc_clk),
    .CLKFX(u_vo_clk_gen_u_vo_clk_pll_CLKFX),
    .CLKFX180(u_vo_clk_gen_u_vo_clk_pll_CLKFX180),
    .CLKFXDV(u_vo_clk_gen_u_vo_clk_pll_CLKFXDV),
    .RST(_zz_Pano_31_),
    .FREEZEDCM(_zz_Pano_32_),
    .LOCKED(u_vo_clk_gen_u_vo_clk_pll_LOCKED),
    .STATUS(u_vo_clk_gen_u_vo_clk_pll_STATUS),
    .PROGCLK(_zz_Pano_33_),
    .PROGDATA(_zz_Pano_34_),
    .PROGEN(_zz_Pano_35_),
    .PROGDONE(u_vo_clk_gen_u_vo_clk_pll_PROGDONE) 
  );
  DCM_CLKGEN #( 
    .CLKFX_DIVIDE(125),
    .CLKFXDV_DIVIDE(2),
    .CLKFX_MD_MAX(0.0),
    .CLKFX_MULTIPLY(24),
    .CLKIN_PERIOD("8.0"),
    .SPREAD_SPECTRUM("NONE"),
    .STARTUP_WAIT(1'b0) 
  ) u_usb_clk_gen_u_usb_clk_pll ( 
    .CLKIN(osc_clk),
    .CLKFX(u_usb_clk_gen_u_usb_clk_pll_CLKFX),
    .CLKFX180(u_usb_clk_gen_u_usb_clk_pll_CLKFX180),
    .CLKFXDV(u_usb_clk_gen_u_usb_clk_pll_CLKFXDV),
    .RST(_zz_Pano_36_),
    .FREEZEDCM(_zz_Pano_37_),
    .LOCKED(u_usb_clk_gen_u_usb_clk_pll_LOCKED),
    .STATUS(u_usb_clk_gen_u_usb_clk_pll_STATUS),
    .PROGCLK(_zz_Pano_38_),
    .PROGDATA(_zz_Pano_39_),
    .PROGEN(_zz_Pano_40_),
    .PROGDONE(u_usb_clk_gen_u_usb_clk_pll_PROGDONE) 
  );
  PanoCore core_u_pano_core ( 
    .io_led_red(core_u_pano_core_io_led_red),
    .io_led_green(core_u_pano_core_io_led_green),
    .io_led_blue(core_u_pano_core_io_led_blue),
    .io_switch_(pano_button),
    .io_dvi_ctrl_scl_read(_zz_Pano_12_),
    .io_dvi_ctrl_scl_write(core_u_pano_core_io_dvi_ctrl_scl_write),
    .io_dvi_ctrl_scl_writeEnable(core_u_pano_core_io_dvi_ctrl_scl_writeEnable),
    .io_dvi_ctrl_sda_read(_zz_Pano_15_),
    .io_dvi_ctrl_sda_write(core_u_pano_core_io_dvi_ctrl_sda_write),
    .io_dvi_ctrl_sda_writeEnable(core_u_pano_core_io_dvi_ctrl_sda_writeEnable),
    .io_gmii_rx_clk(gmii_rx_clk),
    .io_gmii_rx_dv(gmii_rx_dv),
    .io_gmii_rx_er(gmii_rx_er),
    .io_gmii_rx_d(gmii_rx_d),
    .io_gmii_tx_gclk(gmii_tx_gclk),
    .io_gmii_tx_clk(gmii_tx_clk),
    .io_gmii_tx_en(core_u_pano_core_io_gmii_tx_en),
    .io_gmii_tx_er(core_u_pano_core_io_gmii_tx_er),
    .io_gmii_tx_d(core_u_pano_core_io_gmii_tx_d),
    .io_gmii_col(gmii_col),
    .io_gmii_crs(gmii_crs),
    .io_gmii_mdio_mdc(core_u_pano_core_io_gmii_mdio_mdc),
    .io_gmii_mdio_mdio_read(_zz_Pano_18_),
    .io_gmii_mdio_mdio_write(core_u_pano_core_io_gmii_mdio_mdio_write),
    .io_gmii_mdio_mdio_writeEnable(core_u_pano_core_io_gmii_mdio_mdio_writeEnable),
    .io_ulpi_clk(ulpi_clk),
    .io_ulpi_data_read(_zz_Pano_21_),
    .io_ulpi_data_write(core_u_pano_core_io_ulpi_data_write),
    .io_ulpi_data_writeEnable(core_u_pano_core_io_ulpi_data_writeEnable),
    .io_ulpi_direction(ulpi_direction),
    .io_ulpi_stp(core_u_pano_core_io_ulpi_stp),
    .io_ulpi_nxt(ulpi_nxt),
    .io_ulpi_reset(core_u_pano_core_io_ulpi_reset),
    .io_vo_vsync(core_u_pano_core_io_vo_vsync),
    .io_vo_hsync(core_u_pano_core_io_vo_hsync),
    .io_vo_blank_(core_u_pano_core_io_vo_blank_),
    .io_vo_de(core_u_pano_core_io_vo_de),
    .io_vo_r(core_u_pano_core_io_vo_r),
    .io_vo_g(core_u_pano_core_io_vo_g),
    .io_vo_b(core_u_pano_core_io_vo_b),
    .io_axi1_aw_valid(core_u_pano_core_io_axi1_aw_valid),
    .io_axi1_aw_ready(core_mem_io_axi1_aw_ready),
    .io_axi1_aw_payload_addr(core_u_pano_core_io_axi1_aw_payload_addr),
    .io_axi1_aw_payload_id(core_u_pano_core_io_axi1_aw_payload_id),
    .io_axi1_aw_payload_region(core_u_pano_core_io_axi1_aw_payload_region),
    .io_axi1_aw_payload_len(core_u_pano_core_io_axi1_aw_payload_len),
    .io_axi1_aw_payload_size(core_u_pano_core_io_axi1_aw_payload_size),
    .io_axi1_aw_payload_burst(core_u_pano_core_io_axi1_aw_payload_burst),
    .io_axi1_aw_payload_lock(core_u_pano_core_io_axi1_aw_payload_lock),
    .io_axi1_aw_payload_cache(core_u_pano_core_io_axi1_aw_payload_cache),
    .io_axi1_aw_payload_qos(core_u_pano_core_io_axi1_aw_payload_qos),
    .io_axi1_aw_payload_prot(core_u_pano_core_io_axi1_aw_payload_prot),
    .io_axi1_w_valid(core_u_pano_core_io_axi1_w_valid),
    .io_axi1_w_ready(core_mem_io_axi1_w_ready),
    .io_axi1_w_payload_data(core_u_pano_core_io_axi1_w_payload_data),
    .io_axi1_w_payload_strb(core_u_pano_core_io_axi1_w_payload_strb),
    .io_axi1_w_payload_last(core_u_pano_core_io_axi1_w_payload_last),
    .io_axi1_b_valid(core_mem_io_axi1_b_valid),
    .io_axi1_b_ready(core_u_pano_core_io_axi1_b_ready),
    .io_axi1_b_payload_id(core_mem_io_axi1_b_payload_id),
    .io_axi1_b_payload_resp(core_mem_io_axi1_b_payload_resp),
    .io_axi1_ar_valid(core_u_pano_core_io_axi1_ar_valid),
    .io_axi1_ar_ready(core_mem_io_axi1_ar_ready),
    .io_axi1_ar_payload_addr(core_u_pano_core_io_axi1_ar_payload_addr),
    .io_axi1_ar_payload_id(core_u_pano_core_io_axi1_ar_payload_id),
    .io_axi1_ar_payload_region(core_u_pano_core_io_axi1_ar_payload_region),
    .io_axi1_ar_payload_len(core_u_pano_core_io_axi1_ar_payload_len),
    .io_axi1_ar_payload_size(core_u_pano_core_io_axi1_ar_payload_size),
    .io_axi1_ar_payload_burst(core_u_pano_core_io_axi1_ar_payload_burst),
    .io_axi1_ar_payload_lock(core_u_pano_core_io_axi1_ar_payload_lock),
    .io_axi1_ar_payload_cache(core_u_pano_core_io_axi1_ar_payload_cache),
    .io_axi1_ar_payload_qos(core_u_pano_core_io_axi1_ar_payload_qos),
    .io_axi1_ar_payload_prot(core_u_pano_core_io_axi1_ar_payload_prot),
    .io_axi1_r_valid(core_mem_io_axi1_r_valid),
    .io_axi1_r_ready(core_u_pano_core_io_axi1_r_ready),
    .io_axi1_r_payload_data(core_mem_io_axi1_r_payload_data),
    .io_axi1_r_payload_id(core_mem_io_axi1_r_payload_id),
    .io_axi1_r_payload_resp(core_mem_io_axi1_r_payload_resp),
    .io_axi1_r_payload_last(core_mem_io_axi1_r_payload_last),
    .io_axi2_aw_valid(core_u_pano_core_io_axi2_aw_valid),
    .io_axi2_aw_ready(core_mem_io_axi2_aw_ready),
    .io_axi2_aw_payload_addr(core_u_pano_core_io_axi2_aw_payload_addr),
    .io_axi2_aw_payload_id(core_u_pano_core_io_axi2_aw_payload_id),
    .io_axi2_aw_payload_region(core_u_pano_core_io_axi2_aw_payload_region),
    .io_axi2_aw_payload_len(core_u_pano_core_io_axi2_aw_payload_len),
    .io_axi2_aw_payload_size(core_u_pano_core_io_axi2_aw_payload_size),
    .io_axi2_aw_payload_burst(core_u_pano_core_io_axi2_aw_payload_burst),
    .io_axi2_aw_payload_lock(core_u_pano_core_io_axi2_aw_payload_lock),
    .io_axi2_aw_payload_cache(core_u_pano_core_io_axi2_aw_payload_cache),
    .io_axi2_aw_payload_qos(core_u_pano_core_io_axi2_aw_payload_qos),
    .io_axi2_aw_payload_prot(core_u_pano_core_io_axi2_aw_payload_prot),
    .io_axi2_w_valid(core_u_pano_core_io_axi2_w_valid),
    .io_axi2_w_ready(core_mem_io_axi2_w_ready),
    .io_axi2_w_payload_data(core_u_pano_core_io_axi2_w_payload_data),
    .io_axi2_w_payload_strb(core_u_pano_core_io_axi2_w_payload_strb),
    .io_axi2_w_payload_last(core_u_pano_core_io_axi2_w_payload_last),
    .io_axi2_b_valid(core_mem_io_axi2_b_valid),
    .io_axi2_b_ready(core_u_pano_core_io_axi2_b_ready),
    .io_axi2_b_payload_id(core_mem_io_axi2_b_payload_id),
    .io_axi2_b_payload_resp(core_mem_io_axi2_b_payload_resp),
    .io_axi2_ar_valid(core_u_pano_core_io_axi2_ar_valid),
    .io_axi2_ar_ready(core_mem_io_axi2_ar_ready),
    .io_axi2_ar_payload_addr(core_u_pano_core_io_axi2_ar_payload_addr),
    .io_axi2_ar_payload_id(core_u_pano_core_io_axi2_ar_payload_id),
    .io_axi2_ar_payload_region(core_u_pano_core_io_axi2_ar_payload_region),
    .io_axi2_ar_payload_len(core_u_pano_core_io_axi2_ar_payload_len),
    .io_axi2_ar_payload_size(core_u_pano_core_io_axi2_ar_payload_size),
    .io_axi2_ar_payload_burst(core_u_pano_core_io_axi2_ar_payload_burst),
    .io_axi2_ar_payload_lock(core_u_pano_core_io_axi2_ar_payload_lock),
    .io_axi2_ar_payload_cache(core_u_pano_core_io_axi2_ar_payload_cache),
    .io_axi2_ar_payload_qos(core_u_pano_core_io_axi2_ar_payload_qos),
    .io_axi2_ar_payload_prot(core_u_pano_core_io_axi2_ar_payload_prot),
    .io_axi2_r_valid(core_mem_io_axi2_r_valid),
    .io_axi2_r_ready(core_u_pano_core_io_axi2_r_ready),
    .io_axi2_r_payload_data(core_mem_io_axi2_r_payload_data),
    .io_axi2_r_payload_id(core_mem_io_axi2_r_payload_id),
    .io_axi2_r_payload_resp(core_mem_io_axi2_r_payload_resp),
    .io_axi2_r_payload_last(core_mem_io_axi2_r_payload_last),
    .io_spi_ss(core_u_pano_core_io_spi_ss),
    .io_spi_sclk(core_u_pano_core_io_spi_sclk),
    .io_spi_mosi(core_u_pano_core_io_spi_mosi),
    .io_spi_miso(spi_miso),
    .toplevel_main_clk(main_clk),
    .toplevel_main_reset_(main_reset_),
    .toplevel_u_vo_clk_gen_vo_clk(u_vo_clk_gen_vo_clk),
    .toplevel_u_vo_clk_gen_vo_reset_(u_vo_clk_gen_vo_reset_) 
  );
  ChrontelPads core_u_dvi ( 
    .io_pads_reset_(core_u_dvi_io_pads_reset_),
    .io_pads_xclk_p(core_u_dvi_io_pads_xclk_p),
    .io_pads_xclk_n(core_u_dvi_io_pads_xclk_n),
    .io_pads_v(core_u_dvi_io_pads_v),
    .io_pads_h(core_u_dvi_io_pads_h),
    .io_pads_de(core_u_dvi_io_pads_de),
    .io_pads_d(core_u_dvi_io_pads_d),
    .io_vsync(core_vo_vsync),
    .io_hsync(core_vo_hsync),
    .io_de(core_vo_de),
    .io_r(core_vo_r),
    .io_g(core_vo_g),
    .io_b(core_vo_b),
    .clk(u_vo_clk_gen_vo_clk),
    .reset_(u_vo_clk_gen_vo_reset_) 
  );
  ChrontelPads_1_ core_u_hdmi ( 
    .io_pads_reset_(core_u_hdmi_io_pads_reset_),
    .io_pads_xclk_p(core_u_hdmi_io_pads_xclk_p),
    .io_pads_v(core_u_hdmi_io_pads_v),
    .io_pads_h(core_u_hdmi_io_pads_h),
    .io_pads_de(core_u_hdmi_io_pads_de),
    .io_pads_d(core_u_hdmi_io_pads_d),
    .io_vsync(core_vo_vsync),
    .io_hsync(core_vo_hsync),
    .io_de(core_vo_de),
    .io_r(core_vo_r),
    .io_g(core_vo_g),
    .io_b(core_vo_b),
    .clk(u_vo_clk_gen_vo_clk),
    .reset_(u_vo_clk_gen_vo_reset_) 
  );
  MemoryController core_mem ( 
    .io_clk(main_clk),
    .io_ddr2a_d(ddr2a_d),
    .io_ddr2a_a(core_mem_io_ddr2a_a),
    .io_ddr2a_ba(core_mem_io_ddr2a_ba),
    .io_ddr2a_ras_l(core_mem_io_ddr2a_ras_l),
    .io_ddr2a_cas_l(core_mem_io_ddr2a_cas_l),
    .io_ddr2a_we_l(core_mem_io_ddr2a_we_l),
    .io_ddr2a_odt(core_mem_io_ddr2a_odt),
    .io_ddr2a_cke(core_mem_io_ddr2a_cke),
    .io_ddr2a_ldm(core_mem_io_ddr2a_ldm),
    .io_ddr2a_udm(core_mem_io_ddr2a_udm),
    .io_ddr2a_rzq(ddr2a_rzq),
    .io_ddr2a_ldqs_p(ddr2a_ldqs_p),
    .io_ddr2a_ldqs_n(ddr2a_ldqs_n),
    .io_ddr2a_udqs_p(ddr2a_udqs_p),
    .io_ddr2a_udqs_n(ddr2a_udqs_n),
    .io_ddr2a_ck_p(core_mem_io_ddr2a_ck_p),
    .io_ddr2a_ck_n(core_mem_io_ddr2a_ck_n),
    .io_ddr2b_d(ddr2b_d),
    .io_ddr2b_a(core_mem_io_ddr2b_a),
    .io_ddr2b_ba(core_mem_io_ddr2b_ba),
    .io_ddr2b_ras_l(core_mem_io_ddr2b_ras_l),
    .io_ddr2b_cas_l(core_mem_io_ddr2b_cas_l),
    .io_ddr2b_we_l(core_mem_io_ddr2b_we_l),
    .io_ddr2b_odt(core_mem_io_ddr2b_odt),
    .io_ddr2b_cke(core_mem_io_ddr2b_cke),
    .io_ddr2b_ldm(core_mem_io_ddr2b_ldm),
    .io_ddr2b_udm(core_mem_io_ddr2b_udm),
    .io_ddr2b_rzq(ddr2b_rzq),
    .io_ddr2b_ldqs_p(ddr2b_ldqs_p),
    .io_ddr2b_ldqs_n(ddr2b_ldqs_n),
    .io_ddr2b_udqs_p(ddr2b_udqs_p),
    .io_ddr2b_udqs_n(ddr2b_udqs_n),
    .io_ddr2b_ck_p(core_mem_io_ddr2b_ck_p),
    .io_ddr2b_ck_n(core_mem_io_ddr2b_ck_n),
    .io_axi1_aw_valid(core_u_pano_core_io_axi1_aw_valid),
    .io_axi1_aw_ready(core_mem_io_axi1_aw_ready),
    .io_axi1_aw_payload_addr(core_u_pano_core_io_axi1_aw_payload_addr),
    .io_axi1_aw_payload_id(core_u_pano_core_io_axi1_aw_payload_id),
    .io_axi1_aw_payload_region(core_u_pano_core_io_axi1_aw_payload_region),
    .io_axi1_aw_payload_len(core_u_pano_core_io_axi1_aw_payload_len),
    .io_axi1_aw_payload_size(core_u_pano_core_io_axi1_aw_payload_size),
    .io_axi1_aw_payload_burst(core_u_pano_core_io_axi1_aw_payload_burst),
    .io_axi1_aw_payload_lock(core_u_pano_core_io_axi1_aw_payload_lock),
    .io_axi1_aw_payload_cache(core_u_pano_core_io_axi1_aw_payload_cache),
    .io_axi1_aw_payload_qos(core_u_pano_core_io_axi1_aw_payload_qos),
    .io_axi1_aw_payload_prot(core_u_pano_core_io_axi1_aw_payload_prot),
    .io_axi1_w_valid(core_u_pano_core_io_axi1_w_valid),
    .io_axi1_w_ready(core_mem_io_axi1_w_ready),
    .io_axi1_w_payload_data(core_u_pano_core_io_axi1_w_payload_data),
    .io_axi1_w_payload_strb(core_u_pano_core_io_axi1_w_payload_strb),
    .io_axi1_w_payload_last(core_u_pano_core_io_axi1_w_payload_last),
    .io_axi1_b_valid(core_mem_io_axi1_b_valid),
    .io_axi1_b_ready(core_u_pano_core_io_axi1_b_ready),
    .io_axi1_b_payload_id(core_mem_io_axi1_b_payload_id),
    .io_axi1_b_payload_resp(core_mem_io_axi1_b_payload_resp),
    .io_axi1_ar_valid(core_u_pano_core_io_axi1_ar_valid),
    .io_axi1_ar_ready(core_mem_io_axi1_ar_ready),
    .io_axi1_ar_payload_addr(core_u_pano_core_io_axi1_ar_payload_addr),
    .io_axi1_ar_payload_id(core_u_pano_core_io_axi1_ar_payload_id),
    .io_axi1_ar_payload_region(core_u_pano_core_io_axi1_ar_payload_region),
    .io_axi1_ar_payload_len(core_u_pano_core_io_axi1_ar_payload_len),
    .io_axi1_ar_payload_size(core_u_pano_core_io_axi1_ar_payload_size),
    .io_axi1_ar_payload_burst(core_u_pano_core_io_axi1_ar_payload_burst),
    .io_axi1_ar_payload_lock(core_u_pano_core_io_axi1_ar_payload_lock),
    .io_axi1_ar_payload_cache(core_u_pano_core_io_axi1_ar_payload_cache),
    .io_axi1_ar_payload_qos(core_u_pano_core_io_axi1_ar_payload_qos),
    .io_axi1_ar_payload_prot(core_u_pano_core_io_axi1_ar_payload_prot),
    .io_axi1_r_valid(core_mem_io_axi1_r_valid),
    .io_axi1_r_ready(core_u_pano_core_io_axi1_r_ready),
    .io_axi1_r_payload_data(core_mem_io_axi1_r_payload_data),
    .io_axi1_r_payload_id(core_mem_io_axi1_r_payload_id),
    .io_axi1_r_payload_resp(core_mem_io_axi1_r_payload_resp),
    .io_axi1_r_payload_last(core_mem_io_axi1_r_payload_last),
    .io_axi2_aw_valid(core_u_pano_core_io_axi2_aw_valid),
    .io_axi2_aw_ready(core_mem_io_axi2_aw_ready),
    .io_axi2_aw_payload_addr(core_u_pano_core_io_axi2_aw_payload_addr),
    .io_axi2_aw_payload_id(core_u_pano_core_io_axi2_aw_payload_id),
    .io_axi2_aw_payload_region(core_u_pano_core_io_axi2_aw_payload_region),
    .io_axi2_aw_payload_len(core_u_pano_core_io_axi2_aw_payload_len),
    .io_axi2_aw_payload_size(core_u_pano_core_io_axi2_aw_payload_size),
    .io_axi2_aw_payload_burst(core_u_pano_core_io_axi2_aw_payload_burst),
    .io_axi2_aw_payload_lock(core_u_pano_core_io_axi2_aw_payload_lock),
    .io_axi2_aw_payload_cache(core_u_pano_core_io_axi2_aw_payload_cache),
    .io_axi2_aw_payload_qos(core_u_pano_core_io_axi2_aw_payload_qos),
    .io_axi2_aw_payload_prot(core_u_pano_core_io_axi2_aw_payload_prot),
    .io_axi2_w_valid(core_u_pano_core_io_axi2_w_valid),
    .io_axi2_w_ready(core_mem_io_axi2_w_ready),
    .io_axi2_w_payload_data(core_u_pano_core_io_axi2_w_payload_data),
    .io_axi2_w_payload_strb(core_u_pano_core_io_axi2_w_payload_strb),
    .io_axi2_w_payload_last(core_u_pano_core_io_axi2_w_payload_last),
    .io_axi2_b_valid(core_mem_io_axi2_b_valid),
    .io_axi2_b_ready(core_u_pano_core_io_axi2_b_ready),
    .io_axi2_b_payload_id(core_mem_io_axi2_b_payload_id),
    .io_axi2_b_payload_resp(core_mem_io_axi2_b_payload_resp),
    .io_axi2_ar_valid(core_u_pano_core_io_axi2_ar_valid),
    .io_axi2_ar_ready(core_mem_io_axi2_ar_ready),
    .io_axi2_ar_payload_addr(core_u_pano_core_io_axi2_ar_payload_addr),
    .io_axi2_ar_payload_id(core_u_pano_core_io_axi2_ar_payload_id),
    .io_axi2_ar_payload_region(core_u_pano_core_io_axi2_ar_payload_region),
    .io_axi2_ar_payload_len(core_u_pano_core_io_axi2_ar_payload_len),
    .io_axi2_ar_payload_size(core_u_pano_core_io_axi2_ar_payload_size),
    .io_axi2_ar_payload_burst(core_u_pano_core_io_axi2_ar_payload_burst),
    .io_axi2_ar_payload_lock(core_u_pano_core_io_axi2_ar_payload_lock),
    .io_axi2_ar_payload_cache(core_u_pano_core_io_axi2_ar_payload_cache),
    .io_axi2_ar_payload_qos(core_u_pano_core_io_axi2_ar_payload_qos),
    .io_axi2_ar_payload_prot(core_u_pano_core_io_axi2_ar_payload_prot),
    .io_axi2_r_valid(core_mem_io_axi2_r_valid),
    .io_axi2_r_ready(core_u_pano_core_io_axi2_r_ready),
    .io_axi2_r_payload_data(core_mem_io_axi2_r_payload_data),
    .io_axi2_r_payload_id(core_mem_io_axi2_r_payload_id),
    .io_axi2_r_payload_resp(core_mem_io_axi2_r_payload_resp),
    .io_axi2_r_payload_last(core_mem_io_axi2_r_payload_last) 
  );
  assign dvi_spc = _zz_Pano_11_ ? _zz_Pano_13_ : 1'bz;
  assign dvi_spd = _zz_Pano_10_ ? _zz_Pano_16_ : 1'bz;
  assign gmii_mdio_mdio = _zz_Pano_9_ ? _zz_Pano_19_ : 1'bz;
  assign ulpi_data[0] = _zz_Pano_8_ ? _zz_Pano_22_[0] : 1'bz;
  assign ulpi_data[1] = _zz_Pano_7_ ? _zz_Pano_22_[1] : 1'bz;
  assign ulpi_data[2] = _zz_Pano_6_ ? _zz_Pano_22_[2] : 1'bz;
  assign ulpi_data[3] = _zz_Pano_5_ ? _zz_Pano_22_[3] : 1'bz;
  assign ulpi_data[4] = _zz_Pano_4_ ? _zz_Pano_22_[4] : 1'bz;
  assign ulpi_data[5] = _zz_Pano_3_ ? _zz_Pano_22_[5] : 1'bz;
  assign ulpi_data[6] = _zz_Pano_2_ ? _zz_Pano_22_[6] : 1'bz;
  assign ulpi_data[7] = _zz_Pano_1_ ? _zz_Pano_22_[7] : 1'bz;
  always @ (*) begin
    _zz_Pano_1_ = 1'b0;
    if(_zz_Pano_23_[7])begin
      _zz_Pano_1_ = 1'b1;
    end
  end

  always @ (*) begin
    _zz_Pano_2_ = 1'b0;
    if(_zz_Pano_23_[6])begin
      _zz_Pano_2_ = 1'b1;
    end
  end

  always @ (*) begin
    _zz_Pano_3_ = 1'b0;
    if(_zz_Pano_23_[5])begin
      _zz_Pano_3_ = 1'b1;
    end
  end

  always @ (*) begin
    _zz_Pano_4_ = 1'b0;
    if(_zz_Pano_23_[4])begin
      _zz_Pano_4_ = 1'b1;
    end
  end

  always @ (*) begin
    _zz_Pano_5_ = 1'b0;
    if(_zz_Pano_23_[3])begin
      _zz_Pano_5_ = 1'b1;
    end
  end

  always @ (*) begin
    _zz_Pano_6_ = 1'b0;
    if(_zz_Pano_23_[2])begin
      _zz_Pano_6_ = 1'b1;
    end
  end

  always @ (*) begin
    _zz_Pano_7_ = 1'b0;
    if(_zz_Pano_23_[1])begin
      _zz_Pano_7_ = 1'b1;
    end
  end

  always @ (*) begin
    _zz_Pano_8_ = 1'b0;
    if(_zz_Pano_23_[0])begin
      _zz_Pano_8_ = 1'b1;
    end
  end

  always @ (*) begin
    _zz_Pano_9_ = 1'b0;
    if(_zz_Pano_20_)begin
      _zz_Pano_9_ = 1'b1;
    end
  end

  always @ (*) begin
    _zz_Pano_10_ = 1'b0;
    if(_zz_Pano_17_)begin
      _zz_Pano_10_ = 1'b1;
    end
  end

  always @ (*) begin
    _zz_Pano_11_ = 1'b0;
    if(_zz_Pano_14_)begin
      _zz_Pano_11_ = 1'b1;
    end
  end

  assign gmii_rst_ = 1'b1;
  assign usb_reset_ = 1'b1;
  assign main_clk_raw = u_main_clk_gen_u_main_clk_pll_CLKFX;
  assign _zz_Pano_26_ = 1'b0;
  assign _zz_Pano_27_ = 1'b0;
  assign _zz_Pano_28_ = 1'b0;
  assign _zz_Pano_29_ = 1'b0;
  assign _zz_Pano_30_ = 1'b0;
  always @ (*) begin
    main_reset_gen_reset_unbuffered_ = 1'b1;
    if(_zz_Pano_41_)begin
      main_reset_gen_reset_unbuffered_ = 1'b0;
    end
  end

  assign _zz_Pano_24_[4 : 0] = (5'b11111);
  assign main_reset_ = main_reset_gen_reset_unbuffered__regNext;
  assign main_clk = main_clk_raw;
  assign u_vo_clk_gen_vo_clk = u_vo_clk_gen_u_vo_clk_pll_CLKFX;
  assign _zz_Pano_31_ = 1'b0;
  assign _zz_Pano_32_ = 1'b0;
  assign _zz_Pano_33_ = 1'b0;
  assign _zz_Pano_34_ = 1'b0;
  assign _zz_Pano_35_ = 1'b0;
  always @ (*) begin
    u_vo_clk_gen_vo_reset_gen_reset_unbuffered_ = 1'b1;
    if(_zz_Pano_42_)begin
      u_vo_clk_gen_vo_reset_gen_reset_unbuffered_ = 1'b0;
    end
  end

  assign _zz_Pano_25_[4 : 0] = (5'b11111);
  assign u_vo_clk_gen_vo_reset_ = u_vo_clk_gen_vo_reset_gen_reset_unbuffered__regNext;
  assign usb_clk = u_usb_clk_gen_u_usb_clk_pll_CLKFX;
  assign _zz_Pano_36_ = 1'b0;
  assign _zz_Pano_37_ = 1'b0;
  assign _zz_Pano_38_ = 1'b0;
  assign _zz_Pano_39_ = 1'b0;
  assign _zz_Pano_40_ = 1'b0;
  assign led_red = core_u_pano_core_io_led_red;
  assign led_green = core_u_pano_core_io_led_green;
  assign led_blue = core_u_pano_core_io_led_blue;
  assign _zz_Pano_13_ = core_u_pano_core_io_dvi_ctrl_scl_write;
  assign _zz_Pano_14_ = core_u_pano_core_io_dvi_ctrl_scl_writeEnable;
  assign _zz_Pano_16_ = core_u_pano_core_io_dvi_ctrl_sda_write;
  assign _zz_Pano_17_ = core_u_pano_core_io_dvi_ctrl_sda_writeEnable;
  assign gmii_tx_en = core_u_pano_core_io_gmii_tx_en;
  assign gmii_tx_er = core_u_pano_core_io_gmii_tx_er;
  assign gmii_tx_d = core_u_pano_core_io_gmii_tx_d;
  assign gmii_mdio_mdc = core_u_pano_core_io_gmii_mdio_mdc;
  assign _zz_Pano_19_ = core_u_pano_core_io_gmii_mdio_mdio_write;
  assign _zz_Pano_20_ = core_u_pano_core_io_gmii_mdio_mdio_writeEnable;
  assign _zz_Pano_22_ = core_u_pano_core_io_ulpi_data_write;
  assign _zz_Pano_23_ = core_u_pano_core_io_ulpi_data_writeEnable;
  assign ulpi_stp = core_u_pano_core_io_ulpi_stp;
  assign ulpi_reset = core_u_pano_core_io_ulpi_reset;
  assign core_vo_vsync = core_u_pano_core_io_vo_vsync;
  assign core_vo_hsync = core_u_pano_core_io_vo_hsync;
  assign core_vo_blank_ = core_u_pano_core_io_vo_blank_;
  assign core_vo_de = core_u_pano_core_io_vo_de;
  assign core_vo_r = core_u_pano_core_io_vo_r;
  assign core_vo_g = core_u_pano_core_io_vo_g;
  assign core_vo_b = core_u_pano_core_io_vo_b;
  assign spi_ss = core_u_pano_core_io_spi_ss;
  assign spi_sclk = core_u_pano_core_io_spi_sclk;
  assign spi_mosi = core_u_pano_core_io_spi_mosi;
  assign dvi_reset_ = core_u_dvi_io_pads_reset_;
  assign dvi_xclk_p = core_u_dvi_io_pads_xclk_p;
  assign dvi_xclk_n = core_u_dvi_io_pads_xclk_n;
  assign dvi_v = core_u_dvi_io_pads_v;
  assign dvi_h = core_u_dvi_io_pads_h;
  assign dvi_de = core_u_dvi_io_pads_de;
  assign dvi_d = core_u_dvi_io_pads_d;
  assign hdmi_reset_ = core_u_hdmi_io_pads_reset_;
  assign hdmi_xclk_p = core_u_hdmi_io_pads_xclk_p;
  assign hdmi_v = core_u_hdmi_io_pads_v;
  assign hdmi_h = core_u_hdmi_io_pads_h;
  assign hdmi_de = core_u_hdmi_io_pads_de;
  assign hdmi_d = core_u_hdmi_io_pads_d;
  assign ddr2a_a = core_mem_io_ddr2a_a;
  assign ddr2a_ba = core_mem_io_ddr2a_ba;
  assign ddr2a_ras_l = core_mem_io_ddr2a_ras_l;
  assign ddr2a_cas_l = core_mem_io_ddr2a_cas_l;
  assign ddr2a_we_l = core_mem_io_ddr2a_we_l;
  assign ddr2a_odt = core_mem_io_ddr2a_odt;
  assign ddr2a_cke = core_mem_io_ddr2a_cke;
  assign ddr2a_ldm = core_mem_io_ddr2a_ldm;
  assign ddr2a_udm = core_mem_io_ddr2a_udm;
  assign ddr2a_ck_p = core_mem_io_ddr2a_ck_p;
  assign ddr2a_ck_n = core_mem_io_ddr2a_ck_n;
  assign ddr2b_a = core_mem_io_ddr2b_a;
  assign ddr2b_ba = core_mem_io_ddr2b_ba;
  assign ddr2b_ras_l = core_mem_io_ddr2b_ras_l;
  assign ddr2b_cas_l = core_mem_io_ddr2b_cas_l;
  assign ddr2b_we_l = core_mem_io_ddr2b_we_l;
  assign ddr2b_odt = core_mem_io_ddr2b_odt;
  assign ddr2b_cke = core_mem_io_ddr2b_cke;
  assign ddr2b_ldm = core_mem_io_ddr2b_ldm;
  assign ddr2b_udm = core_mem_io_ddr2b_udm;
  assign ddr2b_ck_p = core_mem_io_ddr2b_ck_p;
  assign ddr2b_ck_n = core_mem_io_ddr2b_ck_n;
  assign _zz_Pano_12_ = dvi_spc;
  assign _zz_Pano_15_ = dvi_spd;
  assign _zz_Pano_18_ = gmii_mdio_mdio;
  assign _zz_Pano_21_ = ulpi_data;
  always @ (posedge main_clk_raw) begin
    if(_zz_Pano_41_)begin
      main_reset_gen_reset_cntr <= (main_reset_gen_reset_cntr + (5'b00001));
    end
  end

  always @ (posedge main_clk_raw) begin
    main_reset_gen_reset_unbuffered__regNext <= main_reset_gen_reset_unbuffered_;
  end

  always @ (posedge u_vo_clk_gen_vo_clk) begin
    if(_zz_Pano_42_)begin
      u_vo_clk_gen_vo_reset_gen_reset_cntr <= (u_vo_clk_gen_vo_reset_gen_reset_cntr + (5'b00001));
    end
  end

  always @ (posedge u_vo_clk_gen_vo_clk) begin
    u_vo_clk_gen_vo_reset_gen_reset_unbuffered__regNext <= u_vo_clk_gen_vo_reset_gen_reset_unbuffered_;
  end

  always @ (posedge gmii_rx_clk) begin
    gmii_rx_green_counter <= (gmii_rx_green_counter + (24'b000000000000000000000001));
  end

  always @ (posedge main_clk) begin
    core_red_counter <= (core_red_counter + (24'b000000000000000000000001));
  end

endmodule

