`ifndef I2C_MMIO_V
`define I2C_MMIO_V

`timescale 1ns/1ps

`ifndef I2C_DEFAULT_FREQ
`define I2C_DEFAULT_FREQ 1000                               // 1kHz
`endif

`ifndef I2C_FIFO_DEPTH
`define I2C_FIFO_DEPTH 16
`endif

module i2cm_mmio #(
    parameter [31:0]  BASE_ADDR     = 32'h8200_1000,        // 内存映射基地址
    parameter [31:0]  CLK_FREQ      = 32'd100_000_000,      // 系统时钟频率100MHz
    parameter integer DEFAULT_FREQ  = `I2C_DEFAULT_FREQ,    // 默认I2C频率
    parameter integer FIFO_DEPTH    = `I2C_FIFO_DEPTH,      // FIFO深度
    parameter integer MIN_FREQ      = 1000                  // 最小I2C频率1kHz
)(
    input  wire                     clk,                    // 系统时钟
    input  wire                     resetn,                 // 低电平有效复位

    // 内存映射接口
    input  wire                     mem_valid,              // 内存访问有效
    input  wire                     mem_instr,              // 指令访问（通常为0）
    output reg                      mem_ready,              // 内存访问完成
    input  wire [31:0]              mem_addr,               // 内存地址
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [31:0]              mem_wdata,              // 写入数据
    /* verilator lint_on  UNUSEDSIGNAL */
    input  wire [3:0]               mem_wstrb,              // 字节使能
    output reg  [31:0]              mem_rdata,              // 读取数据

    // I2C物理接口
    output reg                      i2c_scl,               // I2C串行时钟线
    inout  wire                     i2c_sda,               // I2C串行数据线（双向）

    // I2C控制信号
    output reg                      i2c_sda_oe,            // SDA输出使能（1=输出，0=输入）
    input  wire                     i2c_sda_in,            // SDA输入值

    // 中断接口
    output reg                      irq,                   // 中断请求
    input  wire                     eoi                    // 中断结束确认
);

    // 寄存器地址偏移定义
    localparam [31:0] RW_REG_CTRL      = BASE_ADDR + 32'h00;  // 控制寄存器
    localparam [31:0] RO_REG_STATUS    = BASE_ADDR + 32'h04;  // 状态寄存器
    localparam [31:0] RW_REG_DATA      = BASE_ADDR + 32'h08;  // 数据寄存器
    localparam [31:0] RW_REG_CLK_DIV   = BASE_ADDR + 32'h0C;  // 时钟分频寄存器
    localparam [31:0] RW_REG_ADDR      = BASE_ADDR + 32'h10;  // 从设备地址寄存器
    localparam [31:0] RW_REG_IRQ_STAT  = BASE_ADDR + 32'h14;  // 中断状态寄存器
    localparam [31:0] RW_REG_IRQ_MASK  = BASE_ADDR + 32'h18;  // 中断掩码寄存器

    // 内部寄存器定义
    reg [31:0] ctrl_reg, status_reg, data_reg, clk_div_reg, addr_reg, irq_stat_reg, irq_mask_reg;

    // 控制寄存器位域定义
    wire ctrl_enable     = ctrl_reg[0];      // 1: I2C模块使能
    wire ctrl_start      = ctrl_reg[1];      // 1: 产生START条件
    wire ctrl_stop       = ctrl_reg[2];      // 1: 产生STOP条件
    wire ctrl_ack        = ctrl_reg[3];      // 1: 接收字节后发送ACK
    wire ctrl_tx_mode    = ctrl_reg[4];      // 1: 发送模式，0: 接收模式
    wire ctrl_gc_en      = ctrl_reg[5];      // 1: 广播呼叫使能
    wire ctrl_smbus_en   = ctrl_reg[6];      // 1: SMBus模式使能

    // 状态寄存器位域定义
    wire status_tx_empty = status_reg[0];    // 1: TX FIFO空
    wire status_tx_full  = status_reg[1];    // 1: TX FIFO满
    wire status_rx_empty = status_reg[2];    // 1: RX FIFO空
    wire status_rx_full  = status_reg[3];    // 1: RX FIFO满
    wire status_busy     = status_reg[4];    // 1: I2C总线忙
    wire status_arb_lost = status_reg[5];    // 1: 仲裁丢失
    wire status_ack_recv = status_reg[6];    // 1: 收到ACK
    wire status_bus_err  = status_reg[7];    // 1: 检测到总线错误

    // 中断状态寄存器位域定义
    wire irq_tx_empty    = irq_stat_reg[0];  // TX FIFO空中断
    wire irq_rx_full     = irq_stat_reg[1];  // RX FIFO满中断
    wire irq_nack        = irq_stat_reg[2];  // 收到NACK中断
    wire irq_arb_lost    = irq_stat_reg[3];  // 仲裁丢失中断
    wire irq_bus_error   = irq_stat_reg[4];  // 总线错误中断
    wire irq_stop        = irq_stat_reg[5];  // 检测到STOP条件中断

    // FIFO存储器定义
    reg [7:0] tx_fifo [0:FIFO_DEPTH-1];      // 发送FIFO
    reg [7:0] rx_fifo [0:FIFO_DEPTH-1];      // 接收FIFO

    // FIFO指针和计数器
    reg [3:0] tx_wptr, tx_rptr, tx_count;    // TX FIFO写指针、读指针、数据计数
    reg [3:0] rx_wptr, rx_rptr, rx_count;    // RX FIFO写指针、读指针、数据计数

    // I2C状态机相关寄存器
    reg [2:0] i2c_state;                     // I2C状态机状态
    reg [3:0] bit_counter;                   // 位计数器（0-7）
    reg [7:0] shift_reg;                     // 移位寄存器
    reg scl_out, sda_out;                    // SCL和SDA输出值
    reg [15:0] clk_counter;                  // 时钟分频计数器
    wire scl_gen;                            // SCL时钟生成标志

    // SCL时钟周期计算
    // 公式：SCL周期 = (系统时钟频率 / (分频系数 + 1)) / 2
    wire [31:0] scl_period = (CLK_FREQ / (clk_div_reg + 1)) >> 1;

    // FIFO状态标志
    wire tx_fifo_empty = (tx_count == 0);
    wire tx_fifo_full  = (tx_count == FIFO_DEPTH);
    wire rx_fifo_empty = (rx_count == 0);
    wire rx_fifo_full  = (rx_count == FIFO_DEPTH);

    // =========================================================================
    // I2C核心状态机
    // =========================================================================
    always @(posedge clk) begin: I2C_CORE
        if (!resetn) begin
            // 复位所有状态
            i2c_state <= 3'b000;             // 进入IDLE状态
            bit_counter <= 0;                // 位计数器清零
            shift_reg <= 0;                  // 移位寄存器清零
            scl_out <= 1'b1;                 // SCL释放（高电平）
            sda_out <= 1'b1;                 // SDA释放（高电平）
            i2c_sda_oe <= 1'b0;              // SDA输出禁用（输入模式）
            clk_counter <= 0;                // 时钟计数器清零
            status_reg <= 0;                 // 状态寄存器清零
            irq_stat_reg <= 0;               // 中断状态寄存器清零
        end else begin
            // 处理中断结束确认
            if (eoi) begin
                irq <= 0;                    // 清除中断请求
            end

            // 更新状态寄存器
            status_reg[0] <= tx_fifo_empty;  // TX FIFO空状态
            status_reg[1] <= tx_fifo_full;   // TX FIFO满状态
            status_reg[2] <= rx_fifo_empty;  // RX FIFO空状态
            status_reg[3] <= rx_fifo_full;   // RX FIFO满状态
            status_reg[4] <= (i2c_state != 3'b000);  // 总线忙状态（非IDLE状态）
            status_reg[5] <= status_arb_lost;  // 保持仲裁丢失状态
            status_reg[6] <= status_ack_recv;  // 保持ACK接收状态
            status_reg[7] <= status_bus_err;   // 保持总线错误状态

            // 如果有使能的中断且当前无中断，则产生中断
            if (|(irq_stat_reg & irq_mask_reg) && !irq) begin
                irq <= 1'b1;
            end

            // =================================================================
            // I2C状态机实现
            // =================================================================
            case (i2c_state)
                3'b000: begin // IDLE状态
                    // 在IDLE状态，释放总线（SCL和SDA都为高）
                    scl_out <= 1'b1;
                    sda_out <= 1'b1;
                    i2c_sda_oe <= 1'b0;      // SDA为输入模式

                    // 如果使能且收到START命令，进入START状态
                    if (ctrl_start && ctrl_enable) begin
                        i2c_state <= 3'b001; // 转移到START状态
                    end
                end

                // 其他状态说明：
                // 3'b001: START状态 - 产生START条件（SDA在SCL高时从高变低）
                // 3'b010: ADDR状态 - 发送7位地址+1位读写位
                // 3'b011: DATA_TX状态 - 发送数据字节
                // 3'b100: DATA_RX状态 - 接收数据字节
                // 3'b101: ACK状态 - 处理ACK/NACK
                // 3'b110: STOP状态 - 产生STOP条件（SDA在SCL高时从低变高）

                default: begin
                    // 完整的状态机实现应包括：
                    // 1. START条件生成
                    // 2. 地址发送和ACK检测
                    // 3. 数据发送/接收（8位）
                    // 4. ACK/NACK处理
                    // 5. STOP条件生成
                    // 6. 仲裁丢失检测
                    // 7. 总线错误处理
                end
            endcase
        end
    end

    // =========================================================================
    // FIFO管理逻辑
    // =========================================================================
    always @(posedge clk) begin: FIFO_MGMT
        if (!resetn) begin
            // 复位FIFO指针和计数器
            tx_wptr <= 0;
            tx_rptr <= 0;
            tx_count <= 0;
            rx_wptr <= 0;
            rx_rptr <= 0;
            rx_count <= 0;
        end else begin
            // TX FIFO写操作（通过内存映射接口）
            if (mem_valid && !mem_instr && |mem_wstrb && (mem_addr == RW_REG_DATA)) begin
                if (!tx_fifo_full) begin
                    tx_fifo[tx_wptr] <= mem_wdata[7:0];  // 写入数据到TX FIFO
                    tx_wptr <= tx_wptr + 1;              // 写指针递增
                    tx_count <= tx_count + 1;            // 数据计数递增
                end
            end

            // TX FIFO读操作（由I2C状态机读取）
            // 当I2C状态机需要发送数据时从此处读取
            if (/* I2C状态机从TX FIFO读取的条件 */ 1'b0) begin
                if (!tx_fifo_empty) begin
                    data_reg <= tx_fifo[tx_rptr];        // 从TX FIFO读取数据
                    tx_rptr <= tx_rptr + 1;              // 读指针递增
                    tx_count <= tx_count - 1;            // 数据计数递减
                end
            end

            // RX FIFO写操作（由I2C状态机写入）
            // 当I2C状态机接收到数据时写入此处
            if (/* I2C状态机写入RX FIFO的条件 */ 1'b0) begin
                if (!rx_fifo_full) begin
                    rx_fifo[rx_wptr] <= data_reg;        // 写入数据到RX FIFO
                    rx_wptr <= rx_wptr + 1;              // 写指针递增
                    rx_count <= rx_count + 1;            // 数据计数递增
                    // 设置RX FIFO满中断
                    if (rx_count == FIFO_DEPTH-1) begin
                        irq_stat_reg[1] <= 1'b1;         // 触发RX满中断
                    end
                end
            end

            // RX FIFO读操作（通过内存映射接口）
            if (mem_valid && !mem_instr && (mem_wstrb == 0) && (mem_addr == RW_REG_DATA)) begin
                if (!rx_fifo_empty) begin
                    data_reg <= rx_fifo[rx_rptr];        // 从RX FIFO读取数据
                    rx_rptr <= rx_rptr + 1;              // 读指针递增
                    rx_count <= rx_count - 1;            // 数据计数递减
                    // 清除RX FIFO满中断
                    if (rx_count == FIFO_DEPTH) begin
                        irq_stat_reg[1] <= 1'b0;         // 清除RX满中断
                    end
                end
            end
        end
    end

    // =========================================================================
    // SCL时钟生成逻辑
    // =========================================================================

    // SCL时钟生成标志：当时钟计数器达到SCL半周期时置位
    assign scl_gen = (clk_counter >= scl_period);

    always @(posedge clk) begin: SCL_GEN
        if (!resetn) begin
            clk_counter <= 0;
            i2c_scl <= 1'b1;                  // 复位时SCL为高电平
        end else if (ctrl_enable) begin
            // I2C使能时的SCL生成
            if (clk_counter >= (scl_period << 1) - 1) begin
                clk_counter <= 0;              // 计数器归零
                i2c_scl <= ~i2c_scl;           // 翻转SCL（产生时钟）
            end else begin
                clk_counter <= clk_counter + 1; // 计数器递增
            end
        end else begin
            // I2C禁用时
            clk_counter <= 0;
            i2c_scl <= 1'b1;                  // SCL保持高电平
        end
    end

    // =========================================================================
    // SDA控制逻辑
    // =========================================================================
    always @(*) begin
        i2c_sda_oe = sda_out;  // 当发送数据时使能输出
        // i2c_sda是双向信号，由外部的三态缓冲器处理
        // 当i2c_sda_oe=1时，sda_out驱动i2c_sda
        // 当i2c_sda_oe=0时，i2c_sda由外部设备驱动，通过i2c_sda_in读取
    end

    // =========================================================================
    // 内存映射读接口
    // =========================================================================
    always @(posedge clk) begin: MMIO_READ
        if (!resetn) begin
            mem_rdata <= 0;
            mem_ready <= 0;
        end else begin
            // 处理读请求（mem_wstrb=0表示读操作）
            if (mem_valid && (!mem_instr) && mem_wstrb == 0) begin
                mem_ready <= 1;  // 确认读操作完成
                case (mem_addr)
                    RW_REG_CTRL:     mem_rdata <= ctrl_reg;      // 读取控制寄存器
                    RO_REG_STATUS:   mem_rdata <= status_reg;    // 读取状态寄存器
                    RW_REG_DATA:     mem_rdata <= {24'b0, data_reg}; // 读取数据寄存器
                    RW_REG_CLK_DIV:  mem_rdata <= clk_div_reg;   // 读取时钟分频寄存器
                    RW_REG_ADDR:     mem_rdata <= {24'b0, addr_reg}; // 读取地址寄存器
                    RW_REG_IRQ_STAT: mem_rdata <= irq_stat_reg;  // 读取中断状态寄存器
                    RW_REG_IRQ_MASK: mem_rdata <= irq_mask_reg;  // 读取中断掩码寄存器
                    default:         mem_rdata <= 32'h0;         // 默认返回0
                endcase
            end else begin
                mem_rdata <= 0;
                mem_ready <= 0;
            end
        end
    end

    // =========================================================================
    // 内存映射写接口
    // =========================================================================
    integer j;

    always @(posedge clk) begin: MMIO_WRITE
        if (!resetn) begin
            // 复位寄存器值
            ctrl_reg <= 0;                                     // 控制寄存器清零
            clk_div_reg <= (CLK_FREQ / DEFAULT_FREQ) - 1;      // 设置默认时钟分频
            addr_reg <= 0;                                     // 地址寄存器清零
            irq_mask_reg <= 0;                                 // 中断掩码清零
            data_reg <= 0;                                     // 数据寄存器清零
        end else begin
            // 处理写请求（mem_wstrb≠0表示写操作）
            if (mem_valid && (!mem_instr) && mem_wstrb != 0) begin
                mem_ready <= 1;  // 确认写操作完成
                case(mem_addr)
                    RW_REG_CTRL:     ctrl_reg <= mem_wdata;     // 写入控制寄存器
                    RW_REG_CLK_DIV:  clk_div_reg <= mem_wdata;  // 写入时钟分频寄存器
                    RW_REG_ADDR:     addr_reg <= mem_wdata[7:0]; // 写入地址寄存器
                    RW_REG_IRQ_MASK: irq_mask_reg <= mem_wdata; // 写入中断掩码寄存器
                    RW_REG_IRQ_STAT: irq_stat_reg <= mem_wdata; // 写入1清除中断位
                    default: ; // 其他寄存器为只读或在其他地方处理
                endcase
            end else begin
                mem_ready <= 0;
            end
        end
    end

endmodule

`endif
