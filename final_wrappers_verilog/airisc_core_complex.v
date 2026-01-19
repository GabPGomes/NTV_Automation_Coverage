`timescale 1ns / 1ps

`ifndef SIMULATION
`include "processor_ci_defines.vh"
`endif

// Bus type: AHB

`define ENABLE_SECOND_MEMORY


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


// AHB - Data bus
// AHB - Instruction bus
wire [31:0] DATA_HADDR;
wire        DATA_HWRITE;
wire [2:0]  DATA_HSIZE;
wire [2:0]  DATA_HBURST;
wire        DATA_HMASTLOCK;
wire [3:0]  DATA_HPROT;
wire [1:0]  DATA_HTRANS;
wire [31:0] DATA_HWDATA;
wire [31:0] DATA_HRDATA;
wire        DATA_HREADY;
wire        DATA_HRESP;

ahb_to_wishbone #( // bus adapter
    .ADDR_WIDTH(32),
    .DATA_WIDTH(32)
) ahb2wb_data (
    // Clock & Reset
    .HCLK       (clk_core),
    .HRESETn    (~rst_core),

    // AHB interface
    .HADDR      (DATA_HADDR),
    .HTRANS     (DATA_HTRANS),
    .HWRITE     (DATA_HWRITE),
    .HSIZE      (DATA_HSIZE),
    .HBURST     (DATA_HBURST),
    .HPROT      (DATA_HPROT),
    .HLOCK      (DATA_HMASTLOCK),
    .HWDATA     (DATA_HWDATA),
    .HREADY     (DATA_HREADY),
    .HRDATA     (DATA_HRDATA),
    .HREADYOUT  (DATA_HREADY), // normalmente igual a HREADY em designs simples
    .HRESP      (DATA_HRESP),

    // Wishbone interface
    .wb_cyc     (data_mem_cyc),
    .wb_stb     (data_mem_stb),
    .wb_we      (data_mem_we),
    .wb_wstrb   (data_mem_sel),
    .wb_adr     (data_mem_addr),
    .wb_dat_w   (data_mem_data_out),
    .wb_dat_r   (data_mem_data_in),
    .wb_ack     (data_mem_ack)
);


airi5c_core Processor (
    .rst_ni              (~rst_core),
    .clk_i               (clk_core),
    .ndmreset_o          (),
    .testmode_i          (0),
    .ext_interrupts_i    (0),
    .system_timer_tick_i (0),
    .imem_haddr_o        (HADDR),
    .imem_hwrite_o       (HWRITE),
    .imem_hsize_o        (HSIZE),
    .imem_hburst_o       (HBURST),
    .imem_hmastlock_o    (HLOCK),
    .imem_hprot_o        (HPROT),
    .imem_htrans_o       (HTRANS),
    .imem_hwdata_o       (HWDATA),
    .imem_hrdata_i       (HRDATA),
    .imem_hready_i       (HREADY),
    .imem_hresp_i        (HRESP),
    .dmem_haddr_o        (DATA_HADDR),
    .dmem_hwrite_o       (DATA_HWRITE),
    .dmem_hsize_o        (DATA_HSIZE),
    .dmem_hburst_o       (DATA_HBURST),
    .dmem_hmastlock_o    (DATA_HLOCK),
    .dmem_hprot_o        (DATA_HPROT),
    .dmem_htrans_o       (DATA_HTRANS),
    .dmem_hwdata_o       (DATA_HWDATA),
    .dmem_hrdata_i       (DATA_HRDATA),
    .dmem_hready_i       (DATA_HREADY),
    .dmem_hresp_i        (DATA_HRESP),
    .lock_custom_i       (0),
    .dmi_addr_i          (0),
    .dmi_en_i            (0),
    .dmi_rdata_o         (),
    .dmi_wdata_i         (0),
    .dmi_wen_i           (0),
    .dmi_error_o         (),
    .dmi_dm_busy_o       ()
);

// assign mappings



endmodule