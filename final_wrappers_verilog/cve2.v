`timescale 1ns / 1ps

`ifndef SIMULATION
`include "processor_ci_defines.vh"
`endif

// Bus type: Custom

`define ENABLE_SECOND_MEMORY
`define PIPELINED_WISHBONE


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

wire instr_gnt_i;
wire instr_rvalid_i;

wire data_gnt_i;
wire data_rvalid_i;




cve2_core Processor (
    .clk_i               (clk_core),
    .rst_ni              (~rst_core),
    .test_en_i           (0),
    .hart_id_i           (0),
    .boot_addr_i         (0),
    .instr_req_o         (core_stb),
    .instr_gnt_i         (instr_gnt_i),
    .instr_rvalid_i      (instr_rvalid_i),
    .instr_addr_o        (core_addr),
    .instr_rdata_i       (_core_data_in),
    .instr_err_i         (0),
    .data_req_o          (data_mem_stb),
    .data_gnt_i          (data_gnt_i),
    .data_rvalid_i       (data_rvalid_i),
    .data_we_o           (data_mem_we),
    .data_be_o           (data_mem_sel),
    .data_addr_o         (data_mem_addr),
    .data_wdata_o        (data_mem_data_out),
    .data_rdata_i        (_data_mem_data_in),
    .data_err_i          (0),
    .x_issue_valid_o     (),
    .x_issue_ready_i     (0),
    .x_issue_req_o       (),
    .x_issue_resp_i      (0),
    .x_register_o        (),
    .x_commit_valid_o    (),
    .x_commit_o          (),
    .x_result_valid_i    (0),
    .x_result_ready_o    (),
    .x_result_i          (0),
    .irq_software_i      (0),
    .irq_timer_i         (0),
    .irq_external_i      (0),
    .irq_fast_i          (0),
    .irq_nm_i            (0),
    .irq_pending_o       (),
    .debug_req_i         (0),
    .debug_halted_o      (),
    .dm_halt_addr_i      (0),
    .dm_exception_addr_i (0),
    .crash_dump_o        (),
    .fetch_enable_i      (1),
    .core_busy_o         ()
);

// assign mappings

assign core_data_out = 0;
assign core_we = 0;
assign data_mem_cyc = 1;
assign core_cyc = 1;
assign core_sel = 4'b1111;
assign instr_gnt_i = core_ack;
assign instr_rvalid_i = _core_ack;
assign data_gnt_i = data_mem_ack;
assign data_rvalid_i = _data_mem_ack;

endmodule