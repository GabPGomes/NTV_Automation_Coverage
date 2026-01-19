`timescale 1ns / 1ps

`ifndef SIMULATION
`include "processor_ci_defines.vh"
`endif

// Bus type: AHB


module processorci_top (
    input wire sys_clk, // Clock de sistema
    input wire rst_n,   // Reset do sistema

    `ifndef SIMULATION
    // UART pins
    input wire rx,
    output wire tx,

    // SPI pins
    input wire sck,
    input wire cs,
    input wire mosi,
    output wire miso,

    //SPI control pins
    input wire rw,
    output wire intr

    `else
    output wire        core_cyc,      // Indica uma transação ativa
    output wire        core_stb,      // Indica uma solicitação ativa
    output wire        core_we,       // 1 = Write, 0 = Read

    output wire [3:0]  core_sel,      // Seletores de byte
    output wire [31:0] core_addr,     // Endereço
    output wire [31:0] core_data_out, // Dados de entrada (para escrita)
    input wire [31:0] core_data_in,  // Dados de saída (para leitura)

    input wire        core_ack       // Confirmação da transação

    `ifdef ENABLE_SECOND_MEMORY
,
    output wire        data_mem_cyc,
    output wire        data_mem_stb,
    output wire        data_mem_we,
    output wire [3:0]  data_mem_sel,
    output wire [31:0] data_mem_addr,
    output wire [31:0] data_mem_data_out,
    input wire [31:0] data_mem_data_in,
    input wire        data_mem_ack
    `endif

    `endif
);
wire clk_core, rst_core;
`ifdef SIMULATION
assign clk_core = sys_clk;
assign rst_core = ~rst_n;

`else

// Fios do barramento entre Controller e Processor
wire        core_cyc;
wire        core_stb;
wire        core_we;
wire [3:0]  core_sel;
wire [31:0] core_addr;
wire [31:0] core_data_out;
wire [31:0] core_data_in;
wire        core_ack;

`ifdef ENABLE_SECOND_MEMORY
wire        data_mem_cyc;
wire        data_mem_stb;
wire        data_mem_we;
wire [3:0]  data_mem_sel;
wire [31:0] data_mem_addr;
wire [31:0] data_mem_data_out;
wire [31:0] data_mem_data_in;
wire        data_mem_ack;
`endif
`endif

`ifndef SIMULATION
Controller #(
    .CLK_FREQ           (`CLOCK_FREQ),
    .BIT_RATE           (`BIT_RATE),
    .PAYLOAD_BITS       (`PAYLOAD_BITS),
    .BUFFER_SIZE        (`BUFFER_SIZE),
    .PULSE_CONTROL_BITS (`PULSE_CONTROL_BITS),
    .BUS_WIDTH          (`BUS_WIDTH),
    .WORD_SIZE_BY       (`WORD_SIZE_BY),
    .ID                 (`ID),
    .RESET_CLK_CYCLES   (`RESET_CLK_CYCLES),
    .MEMORY_FILE        (`MEMORY_FILE),
    .MEMORY_SIZE        (`MEMORY_SIZE)
) u_Controller (
    .clk                (sys_clk),

    .rst_n              (rst_n),
    
    // SPI signals
    .sck_i              (sck),
    .cs_i               (cs),
    .mosi_i             (mosi),
    .miso_o             (miso),
    
    // SPI callback signals
    .rw_i               (rw),
    .intr_o             (intr),
    
    // UART signals
    .rx                 (rx),
    .tx                 (tx),
    
    // Clock, reset, and bus signals
    .clk_core_o         (clk_core),
    .rst_core_o         (rst_core),
    
    // Barramento padrão (Wishbone)
    .core_cyc_i         (core_cyc),
    .core_stb_i         (core_stb),
    .core_we_i          (core_we),
    .core_addr_i        (core_addr),
    .core_data_i        (core_data_out),
    .core_data_o        (core_data_in),
    .core_ack_o         (core_ack)

    `ifdef ENABLE_SECOND_MEMORY
    ,
    .data_mem_cyc_i     (data_mem_cyc),
    .data_mem_stb_i     (data_mem_stb),
    .data_mem_we_i      (data_mem_we),
    .data_mem_addr_i    (data_mem_addr),
    .data_mem_data_i    (data_mem_data_out),
    .data_mem_data_o    (data_mem_data_in),
    .data_mem_ack_o     (data_mem_ack)
    `endif
);
`endif

wire [31:0] _core_data_in;
wire        _core_ack;
`ifdef ENABLE_SECOND_MEMORY
wire [31:0] _data_mem_data_in;
wire        _data_mem_ack;
`endif

`ifdef PIPELINED_WISHBONE
always @(posedge clk_core) begin
    if (rst_core) begin
        _core_ack <= 1'b0;
        _core_data_in <= 32'b0;
    end else begin
        _core_ack <= core_ack;
        _core_data_in <= core_data_in;
    end
end
`else
assign _core_ack = core_ack;
assign _core_data_in = core_data_in;
`endif

`ifdef ENABLE_SECOND_MEMORY
`ifdef PIPELINED_WISHBONE
always @(posedge clk_core) begin
    if (rst_core) begin
        _data_mem_ack <= 1'b0;
        _data_mem_data_in <= 32'b0;
    end else begin
        _data_mem_ack <= data_mem_ack;
        _data_mem_data_in <= data_mem_data_in;
    end
end
`else
assign _data_mem_ack = data_mem_ack;
assign _data_mem_data_in = data_mem_data_in;
`endif
`endif

// Core space




// AHB - Instruction bus
wire [31:0] HADDR;
wire        HWRITE;
wire [2:0]  HSIZE;
wire [2:0]  HBURST;
wire        HMASTLOCK;
wire [3:0]  HPROT;
wire [1:0]  HTRANS;
wire [31:0] HWDATA;
wire [31:0] HRDATA;
wire        HREADY;
wire        HRESP;

ahb_to_wishbone #( // bus adapter
    .ADDR_WIDTH(32),
    .DATA_WIDTH(32)
) ahb2wb_inst (
    // Clock & Reset
    .HCLK       (clk_core),
    .HRESETn    (~rst_core),

    // AHB interface
    .HADDR      (HADDR),
    .HTRANS     (HTRANS),
    .HWRITE     (HWRITE),
    .HSIZE      (HSIZE),
    .HBURST     (HBURST),
    .HPROT      (HPROT),
    .HLOCK      (HMASTLOCK),
    .HWDATA     (HWDATA),
    .HREADY     (HREADY),
    .HRDATA     (HRDATA),
    .HREADYOUT  (HREADY), // normalmente igual a HREADY em designs simples
    .HRESP      (HRESP),

    // Wishbone interface
    .wb_cyc     (core_cyc),
    .wb_stb     (core_stb),
    .wb_we      (core_we),
    .wb_wstrb   (core_sel),
    .wb_adr     (core_addr),
    .wb_dat_w   (core_data_out),
    .wb_dat_r   (core_data_in),
    .wb_ack     (core_ack)
);


hazard3_cpu_1port #(
    .RESET_VECTOR       (32'h00000000),
    .PMP_HARDWIRED_ADDR (0),
    .PMP_HARDWIRED_CFG  (0),
    .DEBUG_SUPPORT      (0),
    .MVENDORID_VAL      (32'h0)
) Processor (
    .clk                        (clk_core),
    .clk_always_on              (clk_core),
    .rst_n                      (~rst_core),
    .pwrup_req                  (),
    .pwrup_ack                  (1),
    .clk_en                     (),
    .unblock_out                (),
    .unblock_in                 (1),
    .haddr                      (HADDR),
    .hwrite                     (HWRITE),
    .htrans                     (HTRANS),
    .hsize                      (HSIZE),
    .hburst                     (HBURST),
    .hprot                      (HPROT),
    .hmastlock                  (HLOCK),
    .hmaster                    (),
    .hexcl                      (),
    .hready                     (HREADY),
    .hresp                      (HRESP),
    .hexokay                    (0),
    .hwdata                     (HWDATA),
    .hrdata                     (HRDATA),
    .dbg_req_halt               (0),
    .dbg_req_halt_on_reset      (0),
    .dbg_req_resume             (0),
    .dbg_halted                 (),
    .dbg_running                (),
    .dbg_data0_rdata            (0),
    .dbg_data0_wdata            (),
    .dbg_data0_wen              (),
    .dbg_instr_data             (0),
    .dbg_instr_data_vld         (0),
    .dbg_instr_data_rdy         (),
    .dbg_instr_caught_exception (),
    .dbg_instr_caught_ebreak    (),
    .dbg_sbus_addr              (0),
    .dbg_sbus_write             (0),
    .dbg_sbus_size              (0),
    .dbg_sbus_vld               (0),
    .dbg_sbus_rdy               (),
    .dbg_sbus_err               (),
    .dbg_sbus_wdata             (0),
    .dbg_sbus_rdata             (),
    .irq                        (0),
    .soft_irq                   (0),
    .timer_irq                  (0)
);

// assign mappings



endmodule