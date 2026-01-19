`timescale 1ns / 1ps

`ifndef SIMULATION
`include "processor_ci_defines.vh"
`endif

// Bus type: AHB


module processorci_top (
    input logic sys_clk, // Clock de sistema
    input logic rst_n,   // Reset do sistema

    `ifndef SIMULATION
    // UART pins
    input  logic rx,
    output logic tx,

    // SPI pins
    input  logic sck,
    input  logic cs,
    input  logic mosi,
    output logic miso,

    //SPI control pins
    input  logic rw,
    output logic intr

    `else
    output logic        core_cyc,      // Indica uma transação ativa
    output logic        core_stb,      // Indica uma solicitação ativa
    output logic        core_we,       // 1 = Write, 0 = Read

    output logic [3:0]  core_sel,      // Seletores de byte
    output logic [31:0] core_addr,     // Endereço
    output logic [31:0] core_data_out, // Dados de entrada (para escrita)
    input  logic [31:0] core_data_in,  // Dados de saída (para leitura)

    input  logic        core_ack       // Confirmação da transação

    `ifdef ENABLE_SECOND_MEMORY
,
    output logic        data_mem_cyc,
    output logic        data_mem_stb,
    output logic        data_mem_we,
    output logic [3:0]  data_mem_sel,
    output logic [31:0] data_mem_addr,
    output logic [31:0] data_mem_data_out,
    input  logic [31:0] data_mem_data_in,
    input  logic        data_mem_ack
    `endif

    `endif
);
logic clk_core, rst_core;
`ifdef SIMULATION
assign clk_core = sys_clk;
assign rst_core = ~rst_n;

`else

// Fios do barramento entre Controller e Processor
logic        core_cyc;
logic        core_stb;
logic        core_we;
logic [3:0]  core_sel;
logic [31:0] core_addr;
logic [31:0] core_data_out;
logic [31:0] core_data_in;
logic        core_ack;

`ifdef ENABLE_SECOND_MEMORY
logic        data_mem_cyc;
logic        data_mem_stb;
logic        data_mem_we;
logic [3:0]  data_mem_sel;
logic [31:0] data_mem_addr;
logic [31:0] data_mem_data_out;
logic [31:0] data_mem_data_in;
logic        data_mem_ack;
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

logic [31:0] _core_data_in;
logic        _core_ack;
`ifdef ENABLE_SECOND_MEMORY
logic [31:0] _data_mem_data_in;
logic        _data_mem_ack;
`endif

`ifdef PIPELINED_WISHBONE
always_ff @(posedge clk_core) begin
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
always_ff @(posedge clk_core) begin
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
logic [31:0] HADDR;
logic        HWRITE;
logic [2:0]  HSIZE;
logic [2:0]  HBURST;
logic        HMASTLOCK;
logic [3:0]  HPROT;
logic [1:0]  HTRANS;
logic [31:0] HWDATA;
logic [31:0] HRDATA;
logic        HREADY;
logic        HRESP;

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
    .clk                 (),
    .clk_always_on       (),
    .rst_n               (),
    .pwrup_req           (),
    .pwrup_ack           (),
    .clk_en              (),
    .unblock_out         (),
    .unblock_in          (),
    .haddr               (),
    .hwrite              (),
    .htrans              (),
    .hsize               (),
    .hburst              (),
    .hprot               (),
    .hmastlock           (),
    .hmaster             (),
    .hexcl               (),
    .hready              (),
    .hresp               (),
    .hexokay             (),
    .hwdata              (),
    .hrdata              (),
    .dbg_req_halt        (),
    .dbg_req_halt_on_reset(),
    .dbg_req_resume      (),
    .dbg_halted          (),
    .dbg_running         (),
    .dbg_data0_rdata     (),
    .dbg_data0_wdata     (),
    .dbg_data0_wen       (),
    .dbg_instr_data      (),
    .dbg_instr_data_vld  (),
    .dbg_instr_data_rdy  (),
    .dbg_instr_caught_exception(),
    .dbg_instr_caught_ebreak(),
    .dbg_sbus_addr       (),
    .dbg_sbus_write      (),
    .dbg_sbus_size       (),
    .dbg_sbus_vld        (),
    .dbg_sbus_rdy        (),
    .dbg_sbus_err        (),
    .dbg_sbus_wdata      (),
    .dbg_sbus_rdata      (),
    .irq                 (),
    .soft_irq            (),
    .timer_irq           ()
);

// assign mappings


endmodule