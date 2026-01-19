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

wire [31:0] dmem_addr;
wire [1:0]  dmem_width;
wire [127:0] instr_data;
wire instr_resp;
wire instr_req;
wire [31:0] instr_addr;

ssrv_top Processor(
    .clk(clk_core),
    .rst(rst_core),

    .imem_req(instr_req),
    .imem_addr(instr_addr),
    .imem_rdata(instr_data),
    .imem_resp(instr_resp),
    .imem_err(0),

    .dmem_req(data_mem_stb),
    .dmem_cmd(data_mem_we),
    .dmem_width(dmem_width),
    .dmem_addr(dmem_addr),
    .dmem_wdata(data_mem_data_out),
    .dmem_rdata(_data_mem_data_in),
    .dmem_resp(_data_mem_ack),
    .dmem_err(0)

);

// assign mappings

assign core_data_out = 0;
assign core_we = 0;
assign core_cyc = 1;
assign core_sel = 4'b1111;
assign data_mem_cyc = 1;
assign data_mem_addr = dmem_addr;

localparam [3:0] IDLE              = 4'd0;
localparam [3:0] READ_WB_1         = 4'd1;
localparam [3:0] READ_NEXT_INSTR   = 4'd2;
localparam [3:0] READ_WB_2         = 4'd3;
localparam [3:0] READ_NEXT_INSTR_2 = 4'd4;
localparam [3:0] READ_WB_3         = 4'd5;
localparam [3:0] READ_NEXT_INSTR_3 = 4'd6;
localparam [3:0] READ_WB_4         = 4'd7;
localparam [3:0] WB                = 4'd8;

reg [3:0] state;

always @(posedge clk_core) begin
    if(rst_core) begin
        instr_resp <=0;
        instr_data <= 0;
    end else begin
        case (state)
            IDLE: begin
                instr_resp <= 0;
                if(instr_req) begin
                    core_addr <= instr_addr;
                    core_stb  <= 1;
                    state     <= READ_WB_1;
                end
            end

            READ_WB_1: begin
                if(core_ack) begin
                    core_stb         <= 0;
                    instr_data[31:0] <= core_data_in;
                    core_addr        <= core_addr + 4;
                    state            <= READ_NEXT_INSTR; 
                end
            end

            READ_NEXT_INSTR: begin
                core_stb <= 1;
                state    <= READ_WB_2;
            end

            READ_WB_2: begin
                if(core_ack) begin
                    core_stb          <= 0;
                    instr_data[63:32] <= core_data_in;
                    core_addr         <= core_addr + 4;
                    state             <= READ_NEXT_INSTR_2; 
                end
            end

            READ_NEXT_INSTR_2: begin
                core_stb <= 1;
                state    <= READ_WB_3;
            end

            READ_WB_3: begin
                if(core_ack) begin
                    core_stb          <= 0;
                    instr_data[95:64] <= core_data_in;
                    core_addr         <= core_addr + 4;
                    state             <= READ_NEXT_INSTR_3; 
                end
            end

            READ_NEXT_INSTR_3: begin
                core_stb <= 1;
                state    <= READ_WB_4;
            end

            READ_WB_4: begin
                if(core_ack) begin
                    core_stb           <= 0;
                    instr_data[127:96] <= core_data_in;
                    state              <= WB; 
                end
            end

            WB: begin
                instr_resp <= 1;
                state      <= IDLE;
            end

            default: state <= IDLE;
        endcase
    end
end

// Write strobe translation
always @(*) begin
    case (dmem_width)
        2'b00: begin  // 1 byte
            case (dmem_addr[1:0])
                2'b00: data_mem_sel = 4'b0001;
                2'b01: data_mem_sel = 4'b0010;
                2'b10: data_mem_sel = 4'b0100;
                2'b11: data_mem_sel = 4'b1000;
                default: data_mem_sel = 4'b0000;
            endcase
        end
        2'b01: begin  // 2 bytes (halfword)
            case (dmem_addr[1:0])
                2'b00: data_mem_sel = 4'b0011;
                2'b10: data_mem_sel = 4'b1100;
                default: data_mem_sel = 4'b0000; // invalid
            endcase
        end
        2'b10: begin  // 4 bytes (word)
            case (dmem_addr[1:0])
                2'b00: data_mem_sel = 4'b1111;
                default: data_mem_sel = 4'b0000; // invalid
            endcase
        end
        default: begin
            data_mem_sel = 4'b0000;
        end
    endcase
end

endmodule