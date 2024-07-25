module clk_div #(
    parameter DIVISOR = 50000000
) (
    input clk,
    input rst_n,
    output reg out
);

    integer counter = 0;

    always @(posedge clk)
    begin
        counter <= counter + 1;
        if(counter>=(DIVISOR-1))
            counter <= 0;

        out <= (counter<DIVISOR/2)?1'b1:1'b0;

    end
endmodule

