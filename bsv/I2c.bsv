package I2c;

import FIFOF::*;
import ConfigReg::*;
import RegIf::*;
import I2cRegs::*;

// 本包不认识任何总线：对外只给中立的 RegIf，接哪种总线由 wrap 或装配决定。
typedef struct {
  Bool slave;
} I2cCfg;

// I2C 是开漏线：只往下拉，放手就让上拉电阻把线拉高。所以对外给的是
// 「拉不拉」而不是「输出什么」——把它当推挽驱动接会烧驱动器。
interface I2cPins;
  (* always_ready, result = "scl_pull" *) method Bit#(1) scl_pull;
  (* always_ready, result = "sda_pull" *) method Bit#(1) sda_pull;
  (* always_ready, always_enabled, prefix = "" *)
  method Action scl_in((* port = "scl_i" *) Bit#(1) v);
  (* always_ready, always_enabled, prefix = "" *)
  method Action sda_in((* port = "sda_i" *) Bit#(1) v);
endinterface

interface I2cIfc#(numeric type aw, numeric type dw, numeric type fifoDepth);
  interface RegIf#(aw, dw) regs;
  interface I2cPins        pins;
  (* always_ready *) method Bool irq;
endinterface

typedef enum { Idle, Start, Bit0, Ack, Stop } Phase deriving (Bits, Eq, FShow);

module mkI2c#(I2cCfg cfg)(I2cIfc#(aw, dw, fifoDepth))
    provisos (Mul#(TDiv#(dw, 8), 8, dw), Add#(_a, 8, aw), Add#(_b, 16, dw),
              Add#(_c, 1, dw), Add#(_d, 8, dw), Add#(_e, 10, dw));

  I2cRegsIfc#(aw, dw, fifoDepth) r <- mkI2cRegs(
      I2cRegsCfg { slave: cfg.slave });

  Reg#(Phase)     ph    <- mkConfigReg(Idle);
  Reg#(Bit#(16))  div   <- mkReg(0);
  Reg#(Bit#(2))   quart <- mkReg(0);   // 一个位分四拍，好在中点采样
  Reg#(Bit#(4))   bitn  <- mkReg(0);
  Reg#(Bit#(8))   sh    <- mkReg(0);
  Reg#(Bit#(1))   ackIn <- mkReg(1);
  Reg#(Bool)      rdDir <- mkReg(False);
  Reg#(Bool)      doStop <- mkReg(False);
  Reg#(Bool)      irqf  <- mkReg(False);

  Wire#(Bit#(1)) sclIn <- mkBypassWire;
  Wire#(Bit#(1)) sdaIn <- mkBypassWire;

  Reg#(Bool) cStart <- mkReg(False);
  Reg#(Bool) cWr    <- mkReg(False);
  Reg#(Bool) cRd    <- mkReg(False);
  Reg#(Bool) cStop  <- mkReg(False);
  Reg#(Bool) cAck   <- mkReg(False);

  // swmod 的脉冲与寄存器的新值差一拍，先记脉冲、下一拍再看命令位
  rule mark;
    cStart <= r.cmd_start_wr;
    cWr    <= r.cmd_wr_wr;
    cRd    <= r.cmd_rd_wr;
    cStop  <= r.cmd_stop_wr;
    cAck   <= r.cmd_ack_wr;
  endrule

  rule accept (ph == Idle && r.ctrl_en == 1);
    if (cStart) begin
      ph    <= Start;
      quart <= 0;
      div   <= r.presc;
      sh    <= r.txdata_data;
      rdDir <= False;
      irqf  <= False;
      doStop <= False;
    end else if (cWr || cRd) begin
      ph    <= Bit0;
      bitn  <= 0;
      quart <= 0;
      div   <= r.presc;
      sh    <= cWr ? r.txdata_data : 8'hFF;   // 读的时候放手让从机驱动
      rdDir <= cRd;
      irqf  <= False;
      doStop <= cStop;
    end
  endrule

  // 四分之一位一步：0 拉低 SCL 换数据，1 放开 SCL，2 采样，3 再拉低
  rule run (ph != Idle && r.ctrl_en == 1);
    if (div != 0)
      div <= div - 1;
    else begin
      div <= r.presc;
      quart <= quart + 1;
      if (quart == 2) begin
        if (ph == Bit0 && rdDir) sh <= {sh[6:0], sdaIn};
        if (ph == Ack) ackIn <= sdaIn;
      end
      if (quart == 3) begin
        case (ph)
          Start: ph <= Bit0;
          Bit0: begin
            if (bitn == 7) ph <= Ack;
            else begin
              bitn <= bitn + 1;
              if (!rdDir) sh <= {sh[6:0], 1'b1};
            end
          end
          Ack: begin
            ph   <= doStop ? Stop : Idle;
            irqf <= True;
          end
          Stop: ph <= Idle;
        endcase
      end
    end
  endrule

  // volatile 字段：没有存储，硬件每拍驱动
  rule status;
    r.status_irqf_in(irqf ? 1 : 0);
    r.status_tip_in(ph != Idle ? 1 : 0);
    r.status_busy_in(ph != Idle ? 1 : 0);
    r.status_rxack_in(ackIn);
    r.rxdata_data_in(sh);
  endrule

  // 从机模式只多一件事：认自己的地址。总线仲裁与时钟延展留给后续版本。
  Bool addressed = cfg.slave && ph == Bit0 && !rdDir
                   && sh[7:1] == truncate(r.saddr);

  interface regs = r.regs;
  interface I2cPins pins;
    // 起始是 SCL 高时拉低 SDA，停止是 SCL 高时放开 SDA
    method Bit#(1) scl_pull = (ph == Idle || quart == 1 || quart == 2) ? 0 : 1;
    method Bit#(1) sda_pull;
      case (ph)
        Idle:  return 0;
        Start: return (quart >= 1) ? 1 : 0;
        Bit0:  return (rdDir || sh[7] == 1) ? 0 : 1;
        Ack:   return (rdDir && !cAck) ? 0 : (addressed ? 1 : 0);
        Stop:  return (quart >= 2) ? 0 : 1;
      endcase
    endmethod
    method Action scl_in(Bit#(1) v); sclIn._write(v); endmethod
    method Action sda_in(Bit#(1) v); sdaIn._write(v); endmethod
  endinterface
  method Bool irq = irqf && r.ctrl_ien == 1;
endmodule

endpackage
