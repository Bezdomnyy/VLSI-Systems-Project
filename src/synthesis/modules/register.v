module register #(
    parameter DATA_WIDTH = 16,
    parameter HIGH = DATA_WIDTH - 1

    //parameter RESET_VAL = {DATA_WIDTH{1'b0}}
) (
    input clk,
    input rst_n,
    input cl,
    input ld,
    input [HIGH:0] in,
    input inc,
    input dec,
    input sr,
    input ir,
    input sl,
    input il,
    output [HIGH:0] out
);

    reg [HIGH:0] out_reg, out_next;
    assign out = out_reg;

    always @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            out_reg <= {DATA_WIDTH{1'b0}};
        else
            out_reg <= out_next;
    end

    always @(*) begin
        out_next = out_reg;

        if      (cl)    out_next = {DATA_WIDTH{1'b0}};
        else if (ld)    out_next = in;
        else if (inc)   out_next = out_reg + {{HIGH{1'b0}}, 1'b1};
        else if (dec)   out_next = out_reg - {{HIGH{1'b0}}, 1'b1};
        else if (sr)    out_next = {ir, out_reg[HIGH:1]};
        else if (sl)    out_next = {out_reg[HIGH-1:0], il};
        
    end
    
endmodule