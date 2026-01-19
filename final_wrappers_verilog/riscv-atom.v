`timescale 1ns / 1ps

`ifndef SIMULATION
`include "processor_ci_defines.vh"
`endif

`define ENABLE_SECOND_MEMORY 1
`define EN_EXCEPT
`define EN_RVZICSR

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

    output wire [3:0]  core_sel,
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

// Core space

AtomRV_wb Processor (
    .wb_clk_i           (clk_core),
    .wb_rst_i           (rst_core),
    .reset_vector_i     (32'h0),

    // === IBUS Wishbone Master Interface ===
    .iport_wb_adr_o     (core_addr),
    .iport_wb_dat_i     (core_data_in),
    .iport_wb_cyc_o     (core_cyc),
    .iport_wb_stb_o     (core_stb),
    .iport_wb_ack_i     (core_ack),

    // === DBUS Wishbone Master Interface === 
    .dport_wb_adr_o     (data_mem_addr),
    .dport_wb_dat_o     (data_mem_data_out),
    .dport_wb_dat_i     (data_mem_data_in),
    .dport_wb_cyc_o     (data_mem_cyc),
    .dport_wb_stb_o     (data_mem_stb),
    .dport_wb_we_o      (data_mem_we),
    .dport_wb_sel_o     (data_mem_sel),
    .dport_wb_ack_i     (data_mem_ack),

    // Interrupt signals (always present when EN_EXCEPT is defined in core)
    .irq_i              (1'b0),
    .timer_int_i        (1'b0)
);

assign core_we = 1'b0;
assign core_data_out = 32'h0;
assign core_sel = 4'b1111;


endmodule
