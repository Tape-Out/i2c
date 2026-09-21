"""i2c 的行为测试台：发一个字节、收一个字节，线上逐位对。

测试台就是那根开漏线加一个极简从机：线的值等于「主机拉」与「从机拉」的或非，
主机拉低就是 0，都放手就靠上拉拉高。把它当推挽驱动接会烧驱动器，所以对外
给的是「拉不拉」而不是「输出什么」——测试台也照这个来。

分频取 0，一个位就是四拍：拉低换数据、放开、采样、再拉低。SCL 的上升沿是
采样点，测试台按上升沿数位。

后半段是多主控（UM10204 3.1.8）：测试台再扮一个主控，先占住总线，看我们的 `busy`
跟不跟线、起始等不等总线空闲、tBUF 够不够；再在第五位上把 SDA 拉低，看我们认不认输、
放不放手、`al` 置没置，下一次起始清没清。

认矩阵：`fifoDepth` 与 `slave` 从这一点的旋钮来。
"""
import json
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)
cfg = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
label = cfg.get("label", "")
k = cfg.get("knobs", {})
depth = int(k.get("fifoDepth", 4))
slave = bool(k.get("slave", False))

TXB = 0xA5      # 发出去的那个字节，故意不对称：循环移位错一位就看得出来
RXB = 0x3C      # 从机送回来的
PSLOW = 99     # 量 tBUF 与时序时的分频：一个位 5 × (PSLOW + 1) 拍（OpenCores 3.2.1）
BIT = 5 * (PSLOW + 1)
GAPMIN = -(-52 * BIT // 100)   # tBUF 至少 0.52 个位（快速模式 1.3 µs 对 2.5 µs）

MULTI = '\n  // ---- 多主控（UM10204 3.1.8）----\n  // 原有那几条的计数只对单主控那一段成立，在这里先查掉，后面的起始停止另算\n  rule count_ (ph == Count);\n    Bool wrong = False;\n    // 起始是「SCL 稳定为高时 SDA 下降」，停止是同样条件下的上升。\n    // 原来 SDA 的下降与 SCL 的上升撞在同一个边界上，线上一次起始都没有。\n    if (starts[1] != 1) begin\n      $display("FAIL saw %0d start conditions on the wire, want 1", starts[1]);\n      wrong = True;\n    end\n    if (stops[1] != 1) begin\n      $display("FAIL saw %0d stop conditions on the wire, want 1", stops[1]);\n      wrong = True;\n    end\n    // 只下停止的那条命令不该再发一个字节。它发了的话这里多九个脉冲。\n    if (allEdges[1] != 19) begin\n      $display("FAIL saw %0d clock pulses on the wire, want 19", allEdges[1]);\n      wrong = True;\n    end\n    if (wrong) bad <= True;\n    ph <= BusA;\n  endrule\n\n  // 另一个主控发起始：SCL 高着，它把 SDA 拉低。线上从此是忙的，而我们什么也没做\n  rule busA (ph == BusA);\n    oSda[1] <= True;\n    ph <= BusW;\n    s  <= 0;\n  endrule\n\n  rule busW (ph == BusW);\n    if (s < 20) s <= s + 1; else ph <= BusB;\n  endrule\n\n  rule busB (ph == BusB);\n    let x <- d.regs.access(RegReq {{ addr: rSTAT, write: False,\n                                     wdata: 0, wstrb: 4\'hF }});\n    if (x.rdata[6] != 1) begin\n      $display("FAIL another controller started a transfer but status.busy reads 0");\n      bad <= True;\n    end\n    ph <= BusC;\n    s  <= 0;\n  endrule\n\n  // 这时软件下起始命令：总线不空，不许动（3.1.8：只有总线空闲才许起始）。\n  // 分频取 99，一个位 400 拍，下面量 tBUF 时装帧那一两拍可以忽略\n  rule busC (ph == BusC);\n    case (s)\n      0: wr(rPRESC, {PSLOW});\n      1: wr(rTXD, 32\'h5A);\n      2: begin wr(rCMD, 32\'h80); pulled[1] <= False; edgeMark <= allEdges[1]; end\n      default: begin ph <= BusD; end\n    endcase\n    if (s < 3) s <= s + 1; else s <= 0;\n  endrule\n\n  rule busD (ph == BusD);\n    if (s < 2000) s <= s + 1;\n    else begin\n      if (pulled[1]) begin\n        $display("FAIL our controller drove the bus while another controller held it");\n        bad <= True;\n      end\n      oSda[1] <= False;               // 另一个主控发停止\n      ph <= BusE;\n      s  <= 0;\n    end\n  endrule\n\n  // 等我们的起始加一个字节走完。定长等待而不是轮询：旧实现早就发过了，\n  // 轮询 tip 会卡住，后面的判据就一条也跑不到\n  rule busE (ph == BusE);\n    if (s < 6000) s <= s + 1; else ph <= BusF;\n  endrule\n\n  rule busF (ph == BusF);\n    Bool wrong = False;\n    if (gapFree[1] < {GAPMIN}) begin\n      $display("FAIL the bus was free for only %0d cycles between the other controller\'s stop and our start, under tBUF (0.52 of a {BIT}-cycle bit)", gapFree[1]);\n      wrong = True;\n    end\n    // 这是本台第二次起始。前一个字节把位计数留在 7 的话，起始之后只发一位就进应答\n    if (allEdges[1] - edgeMark != 9) begin\n      $display("FAIL a start after an earlier transfer clocked out %0d pulses, want 9 (eight bits and the acknowledge)", allEdges[1] - edgeMark);\n      wrong = True;\n    end\n    if (wrong) bad <= True;\n    ph <= BusG;\n    s  <= 0;\n  endrule\n\n  rule busG (ph == BusG);\n    wr(rCMD, 32\'h40);                 // 自己的停止\n    ph <= BusH;\n  endrule\n\n  rule busH (ph == BusH);\n    let x <- d.regs.access(RegReq {{ addr: rSTAT, write: False,\n                                     wdata: 0, wstrb: 4\'hF }});\n    if (x.rdata[6] == 0) begin ph <= ArbA; s <= 0; end\n  endrule\n\n  // 仲裁：我们发 0xFF，另一个主控从第五位起把 SDA 拉低。我们想发 1、线上是 0，\n  // 就是输了：必须放开 SDA 与 SCL，置 al 与 irqf\n  rule arbA (ph == ArbA);\n    case (s)\n      0: wr(rPRESC, 0);\n      1: wr(rTXD, 32\'hFF);\n      2: begin wr(rCMD, 32\'h80); arm[1] <= True; aEdges[1] <= 0; end\n      default: begin ph <= ArbB; end\n    endcase\n    if (s < 3) s <= s + 1; else s <= 0;\n  endrule\n\n  rule arbB (ph == ArbB);\n    if (s < 400) s <= s + 1;\n    else begin\n      if (pullNow[1] != 0) begin\n        $display("FAIL our controller kept driving the bus after losing arbitration: scl_pull sda_pull = %b", pullNow[1]);\n        bad <= True;\n      end\n      ph <= ArbC;\n    end\n  endrule\n\n  rule arbC (ph == ArbC);\n    let x <- d.regs.access(RegReq {{ addr: rSTAT, write: False,\n                                     wdata: 0, wstrb: 4\'hF }});\n    Bool wrong = False;\n    if (x.rdata[5] != 1) begin\n      $display("FAIL arbitration was lost on the fifth bit but status.al reads 0");\n      wrong = True;\n    end\n    if (x.rdata[0] != 1) begin\n      $display("FAIL arbitration was lost but irqf is not set");\n      wrong = True;\n    end\n    if (x.rdata[6] != 1) begin\n      $display("FAIL the other controller still holds the bus but busy reads 0");\n      wrong = True;\n    end\n    if (wrong) bad <= True;\n    ph <= ArbD;\n  endrule\n\n  // 另一个主控发停止；总线闲下来，我们重新起始，al 要清掉\n  rule arbD (ph == ArbD);\n    oSda[1] <= False;\n    arm[1]  <= False;\n    ph <= ArbE;\n    s  <= 0;\n  endrule\n\n  rule arbE (ph == ArbE);\n    if (s < 20) s <= s + 1; else ph <= ArbF;\n  endrule\n\n  rule arbF (ph == ArbF);\n    let x <- d.regs.access(RegReq {{ addr: rSTAT, write: False,\n                                     wdata: 0, wstrb: 4\'hF }});\n    if (x.rdata[6] != 0) begin\n      $display("FAIL the other controller sent a stop but busy still reads 1");\n      bad <= True;\n    end\n    ph <= ArbG;\n    s  <= 0;\n  endrule\n\n  rule arbG (ph == ArbG);\n    case (s)\n      0: wr(rTXD, 32\'h5A);\n      1: wr(rCMD, 32\'h80);\n      default: begin ph <= ArbH; end\n    endcase\n    if (s < 2) s <= s + 1; else s <= 0;\n  endrule\n\n  rule arbH (ph == ArbH);\n    if (s < 40) s <= s + 1; else ph <= ArbI;\n  endrule\n\n  rule arbI (ph == ArbI);\n    let x <- d.regs.access(RegReq {{ addr: rSTAT, write: False,\n                                     wdata: 0, wstrb: 4\'hF }});\n    if (x.rdata[5] != 0) begin\n      $display("FAIL status.al is still set after a new start command");\n      bad <= True;\n    end\n    ph <= ArbJ;\n    s  <= 0;\n  endrule\n\n  rule arbJ (ph == ArbJ);\n    if (s < 200) s <= s + 1;\n    else begin wr(rCMD, 32\'h40); ph <= ArbK; end\n  endrule\n\n  rule arbK (ph == ArbK);\n    let x <- d.regs.access(RegReq {{ addr: rSTAT, write: False,\n                                     wdata: 0, wstrb: 4\'hF }});\n    if (x.rdata[6] == 0) ph <= TimR;\n  endrule\n'.format(PSLOW=PSLOW, BIT=BIT, GAPMIN=GAPMIN)


# 表 10 的下限按各模式最高时钟频率折成「几分之一个位」，三种模式取最严的一档。
# 位长照 OpenCores 规范 3.2.1 的分频公式：prescale = clk / (5·SCL) − 1，一个位 5 × (presc + 1) 拍
TLOW = -(-52 * BIT // 100)     # tLOW 0.52（快速模式 1.3 µs 对 2.5 µs）
THIGH = -(-40 * BIT // 100)    # tHIGH 0.40（标准模式 4.0 µs 对 10 µs）
THDSTA = -(-40 * BIT // 100)   # tHD;STA 0.40
TSUSTA = -(-47 * BIT // 100)   # tSU;STA 0.47（标准模式 4.7 µs）
TSUSTO = -(-40 * BIT // 100)   # tSU;STO 0.40
TBUF = -(-52 * BIT // 100)     # tBUF 0.52

TIMING = f"""
  // ---- 时序（UM10204 表 10）与位长（OpenCores 3.2.1）----
  // 分频取 {PSLOW}，按 OpenCores 的公式一个位 {BIT} 拍。起始加字节、带应答的读、重复起始加字节、
  // 停止、再起始加字节、停止，线上各段取最小值与下限比
  rule timR (ph == TimR);
    timArm[1] <= True;
    lowMin[1] <= '1;
    highMin[1] <= '1;
    hdStaMin[1] <= '1;
    suStaMin[1] <= '1;
    suStoMin[1] <= '1;
    bufMin[1] <= '1;
    periodMin[1] <= '1;
    periodMax[1] <= 0;
    stopsTim[1] <= 0;
    ph <= TimA;
    s  <= 0;
  endrule

  rule timA (ph == TimA);
    case (s)
      0: wr(rPRESC, {PSLOW});
      1: wr(rTXD, 32'h5A);
      2: wr(rCMD, 32'h80);            // 起始加一个字节
      default: ph <= TimW1;
    endcase
    if (s < 3) s <= s + 1; else s <= 0;
  endrule

  rule timW1 (ph == TimW1);
    if (s < {12 * BIT}) s <= s + 1; else begin ph <= TimA2; s <= 0; end
  endrule

  // 带应答的读，读还没传完就把重复起始排上：应答位一结束，控制器只空一两拍就进起始相。
  // 起始相要先把 SCL 拉低几份再抬起来，重复起始之前的 tLOW 才够；等读完空闲很久再下命令的话，
  // 空闲那一段 SCL 本来就按在低位，这件事验不出来
  rule timA2 (ph == TimA2);
    case (s)
      0: wr(rCMD, 32'h28);            // 读一个字节并应答
      1: wr(rTXD, 32'hA5);
      2: wr(rCMD, 32'h80);            // 排上：不发停止，重复起始加一个字节
      default: ph <= TimW2;
    endcase
    if (s < 3) s <= s + 1; else s <= 0;
  endrule

  rule timW2 (ph == TimW2);
    if (s < {24 * BIT}) s <= s + 1; else begin ph <= TimC; s <= 0; end
  endrule

  rule timC (ph == TimC);
    wr(rCMD, 32'h40);
    ph <= TimW3;
    s  <= 0;
  endrule

  rule timW3 (ph == TimW3);
    if (s < {4 * BIT}) s <= s + 1; else begin ph <= TimD; s <= 0; end
  endrule

  rule timD (ph == TimD);
    case (s)
      0: wr(rTXD, 32'h3C);
      1: wr(rCMD, 32'h80);            // 自己停止之后再起始，量 tBUF
      default: ph <= TimW4;
    endcase
    if (s < 2) s <= s + 1; else s <= 0;
  endrule

  rule timW4 (ph == TimW4);
    if (s < {12 * BIT}) s <= s + 1; else begin ph <= TimE; s <= 0; end
  endrule

  rule timE (ph == TimE);
    wr(rCMD, 32'h40);
    ph <= TimW5;
    s  <= 0;
  endrule

  rule timW5 (ph == TimW5);
    if (s < {4 * BIT}) s <= s + 1; else ph <= TimChk;
  endrule

  rule timChk (ph == TimChk);
    Bool wrong = False;
    $display("timing, bit of {BIT} cycles: period %0d..%0d  tLOW %0d  tHIGH %0d  tHD;STA %0d  tSU;STA %0d  tSU;STO %0d  tBUF %0d  stops %0d",
             periodMin[1], periodMax[1], lowMin[1], highMin[1], hdStaMin[1], suStaMin[1], suStoMin[1], bufMin[1], stopsTim[1]);
    // 数据位里相邻两个上升沿隔一个位。按 OpenCores 的公式算出的分频，一个位就该是 {BIT} 拍
    if (periodMin[1] != {BIT} || periodMax[1] != {BIT}) begin
      $display("FAIL a bit takes %0d to %0d cycles at prescale {PSLOW}, want {BIT} from the OpenCores formula clk / (5 * SCL) - 1", periodMin[1], periodMax[1]);
      wrong = True;
    end
    if (lowMin[1] < {TLOW}) begin
      $display("FAIL tLOW is %0d cycles, under 0.52 of a bit ({TLOW}) that Fast-mode needs", lowMin[1]);
      wrong = True;
    end
    if (highMin[1] < {THIGH}) begin
      $display("FAIL tHIGH is %0d cycles, under 0.40 of a bit ({THIGH}) that Standard-mode needs", highMin[1]);
      wrong = True;
    end
    if (hdStaMin[1] < {THDSTA}) begin
      $display("FAIL tHD;STA is %0d cycles, under 0.40 of a bit ({THDSTA}) that Standard-mode needs", hdStaMin[1]);
      wrong = True;
    end
    if (suStaMin[1] < {TSUSTA}) begin
      $display("FAIL tSU;STA is %0d cycles, under 0.47 of a bit ({TSUSTA}) that Standard-mode needs", suStaMin[1]);
      wrong = True;
    end
    if (suStoMin[1] < {TSUSTO}) begin
      $display("FAIL tSU;STO is %0d cycles, under 0.40 of a bit ({TSUSTO}) that Standard-mode needs", suStoMin[1]);
      wrong = True;
    end
    if (bufMin[1] < {TBUF}) begin
      $display("FAIL tBUF is %0d cycles, under 0.52 of a bit ({TBUF}) that Fast-mode needs", bufMin[1]);
      wrong = True;
    end
    // 这一段一共只下了两条停止命令
    if (stopsTim[1] != 2) begin
      $display("FAIL saw %0d stop conditions while timing, want 2: a repeated start after an acknowledged read put a stop on the wire", stopsTim[1]);
      wrong = True;
    end
    if (wrong) bad <= True;
    timArm[1] <= False;
    ph <= Done;
  endrule
"""

txt = f'''package I2c{label}Tb;

import ConfigReg::*;
import RegIf::*;
import I2c::*;

// 由 htest/mki2ctb.py 生成，勿手改。这一点：fifoDepth={depth} slave={slave}

Bit#(8) rPRESC = 8'h00;
Bit#(8) rCTRL  = 8'h04;
Bit#(8) rTXD   = 8'h08;
Bit#(8) rRXD   = 8'h0C;
Bit#(8) rCMD   = 8'h10;
Bit#(8) rSTAT  = 8'h14;

Bit#(8) txByte = 8'h{TXB:02X};
Bit#(8) rxByte = 8'h{RXB:02X};

typedef enum {{ Setup, Write, WrBusy, WrWait, WrCheck,
               Read, RdBusy, RdWait, RdCheck, Stop_, StWait,
               Iack, IackCheck, Count, BusA, BusW, BusB, BusC, BusD, BusE, BusF, BusG, BusH,
               ArbA, ArbB, ArbC, ArbD, ArbE, ArbF, ArbG, ArbH, ArbI, ArbJ, ArbK,
               TimR, TimA, TimW1, TimA2, TimW2, TimC, TimW3, TimD, TimW4, TimE, TimW5, TimChk, Done }}
  Phase deriving (Bits, Eq);

(* synthesize *)
module mkI2c{label}Tb(Empty);
  I2cIfc#(8, 32, {depth}) d <- mkI2c(I2cCfg {{ slave: {"True" if slave else "False"} }});

  Reg#(Phase)    ph  <- mkReg(Setup);
  Reg#(Bit#(16)) s   <- mkReg(0);
  // 线上那条规则也读它：普通寄存器会与各阶段规则绕成环，把计拍的规则整条挡掉
  Reg#(Bit#(32)) cyc <- mkConfigReg(0);
  Reg#(Bool)     bad <- mkReg(False);

  // 线上采到的位与已经数过的上升沿
  Reg#(Bit#(1))  sclPrv <- mkReg(1);
  // 线上那条规则每拍都要驱动 always_enabled 的引脚，必然排在测试序列前面。
  // 于是凡是它写、别人读的量都得用 CReg：口 0 归它写，口 1 归检查规则读，
  // 用普通寄存器的话检查规则会被判成永不触发（G0021）。
  Reg#(Bit#(8))  seen[2]  <- mkCReg(2, 0);
  Reg#(Bit#(1))  ackSeen[2] <- mkCReg(2, 0);   // 主机在应答位上拉低了没有
  Reg#(Bool)     sawIrq[2] <- mkCReg(2, False);
  Reg#(Bit#(4))  edges[2] <- mkCReg(2, 0);
  Reg#(Bit#(4))  sIdx[2]  <- mkCReg(2, 7);
  // SCL 高着的时候 SDA 变化，只可能是起始或停止——数它们
  Reg#(Bit#(1))  sdaPrv  <- mkReg(1);
  Reg#(Bit#(4))  starts[2] <- mkCReg(2, 0);
  Reg#(Bit#(4))  stops[2]  <- mkCReg(2, 0);
  // 整场一共几个时钟脉冲：写 8 位加应答 9 个、读 8 位加应答 9 个、
  // 停止条件本身要把 SCL 抬起来一次，共 19。多出来就是有人白发了字节。
  Reg#(Bit#(8))  allEdges[2] <- mkCReg(2, 0);
  // 从机按住 SCL 不放：验主机认不认时钟延展
  Reg#(Bit#(4))  holdN   <- mkReg(6);
  Reg#(Bool)     holding <- mkReg(False);
  // 从机的输出要打一拍、只在 SCL 低的时候换：它的依据（数过的沿数）是在
  // 上升沿更新的，直接拿来驱动就等于在高电平期间动 SDA，那正是起始/停止的
  // 定义，判据会把它数进去。
  Reg#(Bit#(1))  sPull   <- mkReg(0);
  // 另一个主控：它拉不拉 SDA；武装以后数我们的上升沿，第四个之后在低电平里拉低
  Reg#(Bool)     oSda[2]    <- mkCReg(2, False);
  Reg#(Bool)     arm[2]     <- mkCReg(2, False);
  Reg#(Bit#(8))  aEdges[2]  <- mkCReg(2, 0);
  // 我们拉过线没有、这一拍拉着什么、上一次停止到这一次起始隔了几拍
  Reg#(Bool)     pulled[2]  <- mkCReg(2, False);
  Reg#(Bit#(2))  pullNow[2] <- mkCReg(2, 0);
  Reg#(Bit#(32)) gapFree[2] <- mkCReg(2, 0);
  Reg#(Bit#(32)) tStop      <- mkReg(0);
  Reg#(Bit#(8))  edgeMark   <- mkReg(0);
  // 时序监视器：只在 timArm 期间量，各段取最小值。名字不用 mLow：接线规则里已有同名的局部 Bool，会把寄存器遮住（T0070）
  Reg#(Bool)     timArm[2]    <- mkCReg(2, False);
  Reg#(Bit#(32)) lowMin[2]    <- mkCReg(2, '1);
  Reg#(Bit#(32)) highMin[2]   <- mkCReg(2, '1);
  Reg#(Bit#(32)) hdStaMin[2]  <- mkCReg(2, '1);
  Reg#(Bit#(32)) suStaMin[2]  <- mkCReg(2, '1);
  Reg#(Bit#(32)) suStoMin[2]  <- mkCReg(2, '1);
  Reg#(Bit#(32)) bufMin[2]    <- mkCReg(2, '1);
  Reg#(Bit#(32)) periodMin[2] <- mkCReg(2, '1);
  Reg#(Bit#(32)) periodMax[2] <- mkCReg(2, 0);
  Reg#(Bit#(8))  stopsTim[2]  <- mkCReg(2, 0);
  Reg#(Bit#(32)) tSclRise   <- mkReg(0);
  Reg#(Bit#(32)) tSclFall   <- mkReg(0);
  Reg#(Bit#(32)) tStartC    <- mkReg(0);
  Reg#(Bool)     hdPend     <- mkReg(False);
  // 上一次上升沿是不是数据位里的（两个上升沿之间没有起始或停止），是才拿来量位长
  Reg#(Bool)     inBits     <- mkReg(False);
  Reg#(Bit#(32)) candP      <- mkReg(0);
  Reg#(Bool)     candOk     <- mkReg(False);

  // 从机什么时候拉低：写方向的第 9 位（应答），读方向按 rxByte 逐位送
  function Bit#(1) slavePull(Phase p, Bit#(4) e, Bit#(4) i);
    if (p == WrWait || p == WrBusy) return (e >= 8) ? 1 : 0;
    // 计数在上升沿就加过了，主机却在同一个高电平的后半拍才采——
    // 所以第八位采样时 e 已经是 8，写 e < 8 会让从机提前放手，末位读成 1
    else if (p == RdWait || p == RdBusy)
      return (e <= 8 && rxByte[i] == 0) ? 1 : 0;
    else return 0;
  endfunction

  rule wire_;
    Bit#(1) mp = d.pins.sda_pull;
    Bit#(1) sp = slavePull(ph, edges[0], sIdx[0]);
    Bit#(1) sda = (mp == 1 || sPull == 1 || oSda[0]) ? 0 : 1;
    // 写方向数到第三位时，从机把 SCL 按住几拍——规范允许，主机必须等。
    // 必须从**主机自己拉低的那一拍**开始按：中途去按会先造一个假的下降沿，
    // 放手时再造一个假的上升沿，测试台把它数成一位，字节就错了。
    Bool mLow = d.pins.scl_pull == 1;
    Bool hold = (ph == WrWait || ph == WrBusy) && edges[0] == 3
                && holdN != 0 && (mLow || holding);
    if (hold) begin holding <= True; holdN <= holdN - 1; end
    else if (holding) holding <= False;
    Bit#(1) scl = (mLow || hold) ? 0 : 1;
    if (scl == 0) sPull <= sp;      // 只在低电平期间换
    d.pins.sda_in(sda);
    d.pins.scl_in(scl);
    sclPrv <= scl;
    sdaPrv <= sda;
    // SCL 稳定为高时 SDA 才动，那就是起始（下降）或停止（上升）
    if (scl == 1 && sclPrv == 1 && sda != sdaPrv) begin
      if (sda == 0) begin
        starts[0] <= starts[0] + 1;
        gapFree[0] <= cyc - tStop;
      end else begin
        stops[0] <= stops[0] + 1;
        tStop <= cyc;
      end
    end
    if (d.irq) sawIrq[0] <= True;
    // 时序：SCL 的高段、低段与上升沿间隔，起始到第一个下降沿，上升沿到起始或停止，停止到下一次起始
    if (timArm[0]) begin
      if (scl == 1 && sclPrv == 0) begin
        tSclRise <= cyc;
        inBits <= True;
        if (cyc - tSclFall < lowMin[0]) lowMin[0] <= cyc - tSclFall;
        // 两个上升沿的间隔先记成候选：前面的低段不超过一个位（两次命令之间控制器按着 SCL 空等，
        // 那一段不是位长），而且到下一个下降沿之前没有出现起始或停止，才算一个位
        candP  <= cyc - tSclRise;
        candOk <= inBits && cyc - tSclFall <= {BIT};
      end else if (scl == 0 && sclPrv == 1) begin
        tSclFall <= cyc;
        if (candOk && inBits && candP < periodMin[0]) periodMin[0] <= candP;
        if (candOk && inBits && candP > periodMax[0]) periodMax[0] <= candP;
        if (cyc - tSclRise < highMin[0]) highMin[0] <= cyc - tSclRise;
        if (hdPend && cyc - tStartC < hdStaMin[0]) hdStaMin[0] <= cyc - tStartC;
        hdPend <= False;
      end else if (scl == 1 && sclPrv == 1 && sda != sdaPrv) begin
        inBits <= False;
        if (sda == 0) begin
          tStartC <= cyc;
          hdPend <= True;
          if (cyc - tSclRise < suStaMin[0]) suStaMin[0] <= cyc - tSclRise;
          if (cyc - tStop < bufMin[0]) bufMin[0] <= cyc - tStop;
        end else begin
          stopsTim[0] <= stopsTim[0] + 1;
          if (cyc - tSclRise < suStoMin[0]) suStoMin[0] <= cyc - tSclRise;
        end
      end
    end
    pullNow[0] <= {{d.pins.scl_pull, mp}};
    if (d.pins.scl_pull == 1 || mp == 1) pulled[0] <= True;
    if (arm[0] && scl == 1 && sclPrv == 0) aEdges[0] <= aEdges[0] + 1;
    if (arm[0] && aEdges[0] >= 4 && scl == 0) oSda[0] <= True;
    // 上升沿采样，下降沿换从机的数据——I2C 本来就是这么定的
    if (scl == 1 && sclPrv == 0) allEdges[0] <= allEdges[0] + 1;
    if (scl == 1 && sclPrv == 0) begin
      if (ph == WrWait || ph == WrBusy) begin
        if (edges[0] < 8) seen[0] <= {{seen[0][6:0], sda}};
        edges[0] <= edges[0] + 1;
      end else if (ph == RdWait || ph == RdBusy) begin
        // 第 9 位是主机的应答：它拉低才算 ACK
        if (edges[0] == 8) ackSeen[0] <= mp;
        edges[0] <= edges[0] + 1;
      end
    end
    // 进入位相那一下也是个下降沿，但第一位还没采过，这时候不能换数据
    if (scl == 0 && sclPrv == 1 && (ph == RdWait || ph == RdBusy) && edges[0] != 0
        && sIdx[0] != 0) sIdx[0] <= sIdx[0] - 1;
  endrule

  rule tick;
    cyc <= cyc + 1;
    if (cyc > 200000) begin
      $display("TIMEOUT in phase %0d", pack(ph));
      $finish(1);
    end
  endrule

  function Action wr(Bit#(8) a, Bit#(32) v) = action
    let _ <- d.regs.access(RegReq {{ addr: a, write: True,
                                     wdata: v, wstrb: 4'hF }});
  endaction;

  rule setup (ph == Setup);
    case (s)
      0: wr(rPRESC, 0);            // 一个位四拍，跑得最快
      1: wr(rCTRL, 32'hC0);        // en + ien
      default: begin ph <= Write; s <= 0; end
    endcase
    if (s < 2) s <= s + 1;
  endrule

  // 发一个字节：写 txdata 再下 wr 命令
  rule write_ (ph == Write);
    case (s)
      0: wr(rTXD, zeroExtend(txByte));
      1: wr(rCMD, 32'h80);         // start + 这个字节（起始命令也发一个字节）
      default: begin ph <= WrBusy; s <= 0; end
    endcase
    if (s < 2) s <= s + 1;
  endrule

  // 先等 busy 抬起来。直接等「busy 落下且 irqf 抬起」的话，上一帧留下的
  // irqf 会让这一步当场成立——一次帧都没走就去检查了。
  // 等的是 TIP（这一个字节在传），不是 BUSY（总线被占着）。两者含义不同：
  // 起始之后 BUSY 一直是 1，直到停止；一个字节传完 TIP 就落下。
  rule wrBusy (ph == WrBusy);
    let x <- d.regs.access(RegReq {{ addr: rSTAT, write: False,
                                     wdata: 0, wstrb: 4'hF }});
    if (x.rdata[1] == 1) ph <= WrWait;
  endrule

  rule wrWait (ph == WrWait);
    let x <- d.regs.access(RegReq {{ addr: rSTAT, write: False,
                                     wdata: 0, wstrb: 4'hF }});
    // tip 落下且 irqf 抬起来才算这一字节走完
    if (x.rdata[1] == 0 && x.rdata[0] == 1) begin ph <= WrCheck; s <= 0; end
  endrule

  rule wrCheck (ph == WrCheck);
    let x <- d.regs.access(RegReq {{ addr: rSTAT, write: False,
                                     wdata: 0, wstrb: 4'hF }});
    Bool wrong = False;
    if (seen[1] != txByte) begin
      $display("FAIL the byte on the wire is %02h, want %02h",
               seen[1], txByte);
      wrong = True;
    end
    if (x.rdata[7] != 0) begin
      $display("FAIL the slave acked but rxack reads %0d", x.rdata[7]);
      wrong = True;
    end
    if (!sawIrq[1]) begin
      $display("FAIL interrupts are enabled but the line never rose");
      wrong = True;
    end
    if (wrong) bad <= True;
    ph <= Read;
    s  <= 0;
  endrule

  // 收一个字节：下 rd 命令并要求应答
  rule read_ (ph == Read);
    case (s)
      0: begin edges[1] <= 0; sIdx[1] <= 7; end
      1: wr(rCMD, 32'h28);         // rd + ack
      default: begin ph <= RdBusy; s <= 0; end
    endcase
    if (s < 2) s <= s + 1;
  endrule

  rule rdBusy (ph == RdBusy);
    let x <- d.regs.access(RegReq {{ addr: rSTAT, write: False,
                                     wdata: 0, wstrb: 4'hF }});
    if (x.rdata[1] == 1) ph <= RdWait;
  endrule

  rule rdWait (ph == RdWait);
    let x <- d.regs.access(RegReq {{ addr: rSTAT, write: False,
                                     wdata: 0, wstrb: 4'hF }});
    if (x.rdata[1] == 0 && x.rdata[0] == 1) begin ph <= RdCheck; s <= 0; end
  endrule

  rule rdCheck (ph == RdCheck);
    let x <- d.regs.access(RegReq {{ addr: rRXD, write: False,
                                     wdata: 0, wstrb: 4'hF }});
    Bool wrong = False;
    if (x.rdata[7:0] != rxByte) begin
      $display("FAIL rxdata is %02h, want %02h", x.rdata[7:0], rxByte);
      wrong = True;
    end
    // cmd.ack 说了要应答，主机就该在第 9 位把线拉低
    if (ackSeen[1] != 1) begin
      $display("FAIL cmd.ack was set but the master never pulled sda low");
      wrong = True;
    end
    if (wrong) bad <= True;
    ph <= Stop_;
    s  <= 0;
  endrule

  // 收尾发一个停止。到这里还没停过，总线该一直是忙的（规范：起始之后忙，
  // 停止之后才闲）。原来 BUSY 与 TIP 驱动的是同一个表达式，这一条查的就是它。
  rule stop_ (ph == Stop_);
    case (s)
      0: action
           let x <- d.regs.access(RegReq {{ addr: rSTAT, write: False,
                                            wdata: 0, wstrb: 4'hF }});
           if (x.rdata[6] != 1) begin
             $display("FAIL no stop has been sent yet, but busy reads 0");
             bad <= True;
           end
         endaction
      1: wr(rCMD, 32'h40);         // stop
      default: begin ph <= StWait; s <= 0; end
    endcase
    if (s < 2) s <= s + 1;
  endrule

  rule stWait (ph == StWait);
    let x <- d.regs.access(RegReq {{ addr: rSTAT, write: False,
                                     wdata: 0, wstrb: 4'hF }});
    if (x.rdata[6] == 0) begin ph <= Iack; s <= 0; end
  endrule

  // 一次以停止收尾的传输之后中断标志还举着，而唯一能清它的动作是**再下一条命令**
  // ——那等于「想关掉中断就得再发一次总线事务」。开了 ien 的话中断一直举着，
  // 服务程序退不出去。OpenCores 那一套里 cmd 的第 0 位是应答位，写 1 清标志；
  // 我们的第 0 位正好空着。
  //
  // 还要顺带验一件事：只写应答位**不许在线上多发一个字节**。命令寄存器的 swmod
  // 脉冲是寄存器级的（说的是「cmd 被写过」），落到取指那一支就会白发九个时钟，
  // 而末尾那条「一共 19 个时钟」的判据正好会把它逮住。
  rule iack (ph == Iack);
    case (s)
      0: action
           let x <- d.regs.access(RegReq {{ addr: rSTAT, write: False,
                                            wdata: 0, wstrb: 4'hF }});
           if (x.rdata[0] == 0) begin
             $display("FAIL the transfer finished but irqf never rose");
             bad <= True;
           end
         endaction
      1: wr(rCMD, 32'h01);         // 只写应答位
      default: noAction;
    endcase
    if (s > 60) begin ph <= IackCheck; s <= 0; end
    else s <= s + 1;
  endrule

  rule iackCheck (ph == IackCheck);
    let x <- d.regs.access(RegReq {{ addr: rSTAT, write: False,
                                     wdata: 0, wstrb: 4'hF }});
    if (x.rdata[0] == 1) begin
      $display("FAIL writing the acknowledge bit left irqf set");
      bad <= True;
    end
    ph <= Count;
  endrule
{MULTI}
{TIMING}
  rule fin (ph == Done);
    Bool wrong = bad;
    if (wrong) $display("FAILED");
    else $display("PASS i2c: start and stop on the wire, a byte out, a byte in, "
                  + "the ack is sent, a stretched clock is honoured, the "
                  + "interrupt flag can be cleared without another transfer, a busy bus is "
                  + "waited for with tBUF, lost arbitration lets go and sets al, a bit is five prescaled clocks, "
                  + "and the bus timing meets Table 10");
    $finish(wrong ? 1 : 0);
  endrule
endmodule

endpackage
'''

(out / f"I2c{label}Tb.bsv").write_text(txt, encoding="utf-8")
print(f"  i2c 行为测试台就位：fifoDepth={depth} slave={slave}")
