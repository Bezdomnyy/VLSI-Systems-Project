module alu #(
    parameter DATA_WIDTH = 16,
    parameter HIGH = DATA_WIDTH - 1
) (
    input [2:0] oc,
    input [HIGH:0] a,
    input [HIGH:0] b,
    output reg [HIGH:0] f
);
   
    always @(*) begin
        case (oc)
            3'b000: f = a + b;  // ADD
            3'b001: f = a - b;  // SUB
            3'b010: f = a * b;  // MUL
            3'b011: f = a / b;  // DIV
            3'b100: f = ~a;     // NOT
            3'b101: f = a ^ b;  // XOR
            3'b110: f = a | b;  // OR
            3'b111: f = a & b;  // AND
        endcase
    end
    
endmodule
