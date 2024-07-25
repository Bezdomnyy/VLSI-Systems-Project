module bcd (
    input [5:0] in,
    output reg [3:0] ones,
    output reg [3:0] tens
);
    
    always @(*) begin
        ones = in % 6'd10;
        tens = (in / 6'd10) % 6'd10;
    end


endmodule