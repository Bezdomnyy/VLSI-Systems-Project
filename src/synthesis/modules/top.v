module top #(
    parameter DIVISOR = 50000000,
    parameter FILE_NAME = "mem_init.mif",
    parameter ADDR_WIDTH = 6,
    parameter DATA_WIDTH = 16
) (
    input clk,
    input rst_n,
    input [2:0] btn,
    input [9:0] sw,
    output [9:0] led,
    output [27:0] hex
);

    wire slow_clk_out;
    clk_div slow_clk(
        .clk(clk),
        .rst_n(rst_n),
        .out(slow_clk_out)
    );

    wire cpu_mem_we;
    wire [ADDR_WIDTH-1 : 0] cpu_mem_addr;
    wire [DATA_WIDTH-1 : 0] cpu_mem_data;
    wire [DATA_WIDTH-1 : 0] cpu_mem_in;
    memory #(
        .FILE_NAME(FILE_NAME),
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
        ) memory (
            .clk(slow_clk_out),
            .we(cpu_mem_we),
            .addr(cpu_mem_addr),
            .data(cpu_mem_data),
            .out(cpu_mem_in)
        );

    wire [DATA_WIDTH-1 : 0] cpu_out;
    wire [DATA_WIDTH-1 : 0] cpu_in;
    assign cpu_in = {{(DATA_WIDTH-4){1'b0}}, sw[3:0]};
    wire [ADDR_WIDTH-1 : 0] cpu_pc;
    wire [ADDR_WIDTH-1 : 0] cpu_sp;
    cpu #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) cpu (
        .clk(slow_clk_out),
        .rst_n(sw[9]),
        .mem_in(cpu_mem_in),
        .in(cpu_in),
        .mem_we(cpu_mem_we),
        .mem_addr(cpu_mem_addr),
        .mem_data(cpu_mem_data),
        .out(cpu_out),
        .pc(cpu_pc),
        .sp(cpu_sp)
    );

    assign led[4:0] = cpu_out[4:0];

    wire [3:0] sp_tens, sp_ones;
    wire [3:0] pc_tens, pc_ones;

    bcd bcd_sp (.in(cpu_sp), .tens(sp_tens), .ones(sp_ones));
    bcd bcd_pc (.in(cpu_pc), .tens(pc_tens), .ones(pc_ones));

    ssd ssd_sp_tens (.in(sp_tens), .out(hex[27:21]));
    ssd ssd_sp_ones (.in(sp_ones), .out(hex[20:14]));
    ssd ssd_pc_tens (.in(pc_tens), .out(hex[13:7]));
    ssd ssd_pc_ones (.in(pc_ones), .out(hex[6:0]));
    
endmodule
