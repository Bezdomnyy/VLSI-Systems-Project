module top;

    // ALU params
    reg [2:0] alu_oc;
    reg [3:0] alu_a, alu_b;
    wire [3:0] alu_f;

    // REG params
    reg clk, reg_rst_n, reg_cl, reg_ld, reg_inc, reg_dec, reg_sr, reg_sl;
    reg reg_ir, reg_il;
    reg [3:0] reg_in;
    wire [3:0] reg_out;

    // helper vars
    integer i;

    // ALU instance
    alu alu_i(
        .oc(alu_oc),
        .a(alu_a),
        .b(alu_b),
        .f(alu_f)
    );

    // REG instance
    register reg_i(
        .clk(clk),
        .rst_n(reg_rst_n),
        .cl(reg_cl),
        .ld(reg_ld),
        .in(reg_in),
        .inc(reg_inc),
        .dec(reg_dec),
        .sr(reg_sr),
        .ir(reg_ir),
        .sl(reg_sl),
        .il(reg_il),
        .out(reg_out)
    );

    initial begin
        // INIT clk
        clk = 1'b0;

        // ALU test
        $monitor("Vreme: %2d, OC: %3b, A: %4b, B: %4b", $time, alu_oc, alu_a, alu_b);
        for (i = 0; i < 2 ** (3+4+4); i = i + 1) begin
            {alu_oc, alu_a, alu_b} = i;
            #10;
        end
        $stop;

        // REG test
        reg_rst_n = 1'b0;
        {clk, reg_cl, reg_ld, reg_inc, reg_dec, reg_sr, reg_sl} = 7'h00;
        {reg_ir, reg_il} = 2'b00;
        reg_in = 4'h0;

        #2 reg_rst_n = 1'b1;

        $monitor("Vreme: %2d, in: %4b, out: %4b, [cl: %1b  ld: %1b  inc: %1b  dec: %1b  sr: %1b ir: %1b  sl: %1b il: %1b]",
            $time, reg_in, reg_out, reg_cl, reg_ld, reg_inc, reg_dec, reg_sr, reg_ir, reg_sl, reg_il);

        repeat(1000) begin
            reg_in = $urandom_range(2**4 - 1) ;
            {reg_cl, reg_ld, reg_inc, reg_dec, reg_sr, reg_sl} = $urandom_range(2**6 - 1);
            {reg_ir, reg_il} = $urandom_range(2**2 - 1);
            #10;
        end

        $finish;
    end

    // clk ticks
    always #5 clk = ~clk;

endmodule
