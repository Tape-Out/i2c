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
              Add#(_c, 1, dw), Add#(_d, 8, dw), Add#(_e, 7, dw));

  I2cRegsIfc#(aw, dw, fifoDepth) r <- mkI2cRegs(
      I2cRegsCfg { slave: cfg.slave });

  // 寄存器图说的是「写即压入、读即弹出」，那就得真有队列。原来 txdata 被
  // 直接读、rxdata 直接publish 移位寄存器，fifoDepth 这个参数从 1 到 8
  // 面积一动不动——价目表早就在说它什么也没改变。
  //
  // 两个队列都取无门控：first/deq 一旦带隐式条件，就会被提到整条规则头上，
  // 而 accept 只在写方向才需要数据。守卫由规则自己写明。
  FIFOF#(Bit#(8)) txq <- mkGSizedFIFOF(True, True, valueOf(fifoDepth));
  FIFOF#(Bit#(8)) rxq <- mkGSizedFIFOF(True, True, valueOf(fifoDepth));

  Reg#(Phase)     ph    <- mkConfigReg(Idle);
  Reg#(Bit#(16))  div   <- mkReg(0);
  Reg#(Bit#(2))   quart <- mkReg(0);   // 一个位分四拍，好在中点采样
  Reg#(Bit#(4))   bitn  <- mkReg(0);
  Reg#(Bit#(8))   sh    <- mkReg(0);
  Reg#(Bit#(1))   ackIn <- mkReg(1);
  Reg#(Bool)      rdDir <- mkReg(False);
  Reg#(Bool)      doStop <- mkReg(False);
  Reg#(Bool)      irqf  <- mkReg(False);
  // 「总线忙」不等于「正在传一个字节」：规范说起始之后总线就忙，直到停止。
  // 两件事分别是状态寄存器的 BUSY 与 TIP，原来两位驱动的是同一个表达式。
  Reg#(Bool)      busyR <- mkReg(False);

  Wire#(Bit#(1)) sclIn <- mkBypassWire;
  Wire#(Bit#(1)) sdaIn <- mkBypassWire;

  Reg#(Bool) doAck   <- mkReg(False);
  // 命令要粘住：写方向取不到数就得等，而脉冲只有一拍，等一拍就丢了。
  // 口 0 归 accept 消费、口 1 归 mark 置位——消费必须排在置位之前，
  // 反过来会让「总线 -> mark -> accept -> 总线」成环（G0095）。
  Reg#(Bool) cmdRdy[2] <- mkCReg(2, False);
  Reg#(Bool) cmdDly  <- mkReg(False);
  Reg#(Bool) txPend  <- mkReg(False);

  // 一个寄存器里所有字段的 swmod 脉冲是同一个信号：它说的是「CMD 被写过」，
  // 不是「这一位被置上了」。位的新值要下一拍才落进寄存器，所以先记下写过，
  // 下一拍再看位。把脉冲当位用的话，任何一次写 CMD 都会走起始那一支，
  // wr / rd / stop / ack 四个位全都不起作用。
  rule mark;
    cmdDly <= r.cmd_start_wr;
    if (cmdDly) cmdRdy[1] <= True;
    txPend <= r.txdata_data_wr;
  endrule

  // 压数单列一条规则。写在别处的分支里，队列的隐式条件会被提到整条规则头上。
  rule pushTx (txPend && txq.notFull);
    txq.enq(r.txdata_data);
  endrule

  // 软件读过 rxdata 就弹一个
  rule popRx (r.rxdata_data_rd && rxq.notEmpty);
    rxq.deq;
  endrule

  // 起始与写方向都要一个字节，读方向不要
  Bool needsData = r.cmd_start == 1 || r.cmd_wr == 1;
  // 这一次写 cmd 到底有没有下命令。命令寄存器的 swmod 脉冲是**寄存器级**的，
  // 说的只是「cmd 被写过」——不问这一句，一次纯粹的中断应答会白发九个时钟
  // 加一个 0xFF 的字节到总线上。
  Bool anyCmd = r.cmd_start == 1 || r.cmd_wr == 1
                || r.cmd_rd == 1 || r.cmd_stop == 1;

  rule accept (ph == Idle && r.ctrl_en == 1 && cmdRdy[0]
               && (!needsData || txq.notEmpty));
    cmdRdy[0] <= False;
    if (!anyCmd) begin
      // 只写了应答位：清标志，别的什么也不做
      if (r.cmd_iack == 1) irqf <= False;
    end else begin
      if (needsData) txq.deq;
      // 读的时候放手让从机驱动
      sh <= needsData ? txq.first : 8'hFF;
      quart <= 0;
      div   <= r.presc;
      irqf  <= False;
      if (r.cmd_start == 1) begin
        ph     <= Start;
        busyR  <= True;             // 起始一发出，总线就算忙
        rdDir  <= False;
        doStop <= False;
        doAck  <= False;
      end else if (r.cmd_stop == 1 && r.cmd_wr == 0 && r.cmd_rd == 0) begin
        // 只下停止：就发一个停止条件。原来这一支也走位相，于是白发一个字节
        // 加一个应答位，线上多出九个时钟。
        ph     <= Stop;
        doStop <= True;
      end else begin
        ph     <= Bit0;
        bitn   <= 0;
        rdDir  <= r.cmd_rd == 1;
        doStop <= r.cmd_stop == 1;
        doAck  <= r.cmd_ack == 1;
      end
    end
  endrule

  // 放开 SCL 的那几个四分之一拍里，线要是还低着，就是从机在按着不放。
  // 规范允许从机这么做（3.1.9），主机必须等——时间不走。
  Bool releasing = (ph == Start) || quart == 1 || quart == 2;
  Bool stretched = releasing && sclIn == 0;

  // 四分之一位一步：0 拉低 SCL 换数据，1 放开 SCL，2 采样，3 再拉低
  rule run (ph != Idle && r.ctrl_en == 1 && !stretched);
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
            // 读回来的字节进队列，软件读 rxdata 时再弹
            if (rdDir && rxq.notFull) rxq.enq(sh);
          end
          // 停止也是一次完成。原来只有应答相置 irqf，于是「只下停止」那条命令
          // 从不产生完成中断——软件等不到事务真正结束的那一下。
          Stop: begin ph <= Idle; busyR <= False; irqf <= True; end
        endcase
      end
    end
  endrule

  // volatile 字段：没有存储，硬件每拍驱动
  rule status;
    r.status_irqf_in(irqf ? 1 : 0);
    r.status_tip_in(ph != Idle ? 1 : 0);
    r.status_busy_in(busyR ? 1 : 0);
    r.status_rxack_in(ackIn);
    r.rxdata_data_in(rxq.notEmpty ? rxq.first : 0);
  endrule

  // 从机模式只多一件事：认自己的地址。总线仲裁与时钟延展留给后续版本。
  Bool addressed = cfg.slave && ph == Bit0 && !rdDir
                   && sh[7:1] == truncate(r.saddr);

  interface regs = r.regs;
  interface I2cPins pins;
    // 起始是 SCL 高时拉低 SDA，停止是 SCL 高时放开 SDA。
    // 起始那一相整相不拉 SCL：原来 SDA 的下降与 SCL 的上升撞在同一个边界，
    // 接收方根本看不到起始——规范要求 SDA 在 SCL 已经稳定为高时才下降。
    // 三处「放开 SCL」：总线空着的时候 · 起始那一相全程 · 停止之后留在高位。
    // 「总线还忙就把 SCL 留在低位」这一条是必须的：原来每传完一个字节都回
    // Idle 把 SCL 放回高位，下一个字节再拉低——从机看到的是多出来的一个时钟。
    method Bit#(1) scl_pull =
      ((ph == Idle && !busyR) || ph == Start || (ph == Stop && quart != 0)
       || quart == 1 || quart == 2) ? 0 : 1;
    method Bit#(1) sda_pull;
      case (ph)
        Idle:  return 0;
        Start: return (quart >= 2) ? 1 : 0;
        Bit0:  return (rdDir || sh[7] == 1) ? 0 : 1;
        // 读方向上 cmd.ack 说了要应答，主机就得把线拉低；不应答才放手。
        // 原来两个分支都落到「不是从机就放手」，于是主机永远发不出应答。
        Ack:   return rdDir ? (doAck ? 1 : 0) : (addressed ? 1 : 0);
        Stop:  return (quart >= 2) ? 0 : 1;
      endcase
    endmethod
    method Action scl_in(Bit#(1) v); sclIn._write(v); endmethod
    method Action sda_in(Bit#(1) v); sdaIn._write(v); endmethod
  endinterface
  method Bool irq = irqf && r.ctrl_ien == 1;
endmodule

endpackage
