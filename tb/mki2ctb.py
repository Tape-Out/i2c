"""i2c 的行为测试台：发一个字节、收一个字节，线上逐位对。

测试台就是那根开漏线加一个极简从机：线的值等于「主机拉」与「从机拉」的或非，
主机拉低就是 0，都放手就靠上拉拉高。把它当推挽驱动接会烧驱动器，所以对外
给的是「拉不拉」而不是「输出什么」——测试台也照这个来。

分频取 0，一个位就是四拍：拉低换数据、放开、采样、再拉低。SCL 的上升沿是
采样点，测试台按上升沿数位。

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

txt = f'''package I2c{label}Tb;

import RegIf::*;
import I2c::*;

// 由 tb/mki2ctb.py 生成，勿手改。这一点：fifoDepth={depth} slave={slave}

Bit#(8) rPRESC = 8'h00;
Bit#(8) rCTRL  = 8'h04;
Bit#(8) rTXD   = 8'h08;
Bit#(8) rRXD   = 8'h0C;
Bit#(8) rCMD   = 8'h10;
Bit#(8) rSTAT  = 8'h14;

Bit#(8) txByte = 8'h{TXB:02X};
Bit#(8) rxByte = 8'h{RXB:02X};

typedef enum {{ Setup, Write, WrBusy, WrWait, WrCheck,
               Read, RdBusy, RdWait, RdCheck, Stop_, StWait, Done }}
  Phase deriving (Bits, Eq);

(* synthesize *)
module mkI2c{label}Tb(Empty);
  I2cIfc#(8, 32, {depth}) d <- mkI2c(I2cCfg {{ slave: {"True" if slave else "False"} }});

  Reg#(Phase)    ph  <- mkReg(Setup);
  Reg#(Bit#(8))  s   <- mkReg(0);
  Reg#(Bit#(32)) cyc <- mkReg(0);
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
    Bit#(1) sda = (mp == 1 || sPull == 1) ? 0 : 1;
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
      if (sda == 0) starts[0] <= starts[0] + 1;
      else          stops[0]  <= stops[0] + 1;
    end
    if (d.irq) sawIrq[0] <= True;
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
    if (cyc > 40000) begin
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
    if (x.rdata[6] == 0) ph <= Done;
  endrule

  rule fin (ph == Done);
    Bool wrong = bad;
    // 起始是「SCL 稳定为高时 SDA 下降」，停止是同样条件下的上升。
    // 原来 SDA 的下降与 SCL 的上升撞在同一个边界上，线上一次起始都没有。
    if (starts[1] != 1) begin
      $display("FAIL saw %0d start conditions on the wire, want 1", starts[1]);
      wrong = True;
    end
    if (stops[1] != 1) begin
      $display("FAIL saw %0d stop conditions on the wire, want 1", stops[1]);
      wrong = True;
    end
    // 只下停止的那条命令不该再发一个字节。它发了的话这里多九个脉冲。
    if (allEdges[1] != 19) begin
      $display("FAIL saw %0d clock pulses on the wire, want 19", allEdges[1]);
      wrong = True;
    end
    if (wrong) $display("FAILED");
    else $display("PASS i2c: start and stop on the wire, a byte out, a byte in, "
                  + "the ack is sent, and a stretched clock is honoured");
    $finish(wrong ? 1 : 0);
  endrule
endmodule

endpackage
'''

(out / f"I2c{label}Tb.bsv").write_text(txt, encoding="utf-8")
print(f"  i2c 行为测试台就位：fifoDepth={depth} slave={slave}")
