module cpu #(
    parameter ADDR_WIDTH = 6,
    parameter DATA_WIDTH = 16,

    parameter ADDR_HIGH = ADDR_WIDTH - 1,
    parameter DATA_HIGH = DATA_WIDTH - 1
) (
    input clk,
    input rst_n,
    input [DATA_HIGH : 0] mem_in,
    input [DATA_HIGH : 0] in,
    output reg mem_we,
    output reg [ADDR_HIGH : 0] mem_addr,
    output reg [DATA_HIGH : 0] mem_data,
    output [DATA_HIGH : 0] out,
    output [ADDR_HIGH : 0] pc,
    output [ADDR_HIGH : 0] sp
);

    /* CPU REGISTER INSTANTIATION */


    // [PC] Program Counter
    // - Width:            ADDR_WIDTH (6b)
    // - Initial Value:    0x08
    reg pc_ld, pc_inc;
    reg [ADDR_HIGH:0] pc_in;
    wire [ADDR_HIGH : 0] pc_out;
    assign pc = pc_out;
    localparam PC_INIT_VAL = {{(ADDR_WIDTH-4){1'b0}},4'h8};
    register #(.DATA_WIDTH(ADDR_WIDTH)) pc_reg(
        .clk(clk),
        .rst_n(rst_n),
        .ld(pc_ld),
        .in(pc_in),
        .inc(pc_inc),
        .out(pc_out)
    );


    // [SP] Stack Pointer
    // - Width:            ADDR_WIDTH (6b)
    // - Initial Value:    0x3F (63)
    // - Stack Direction:  Downward (From Higher to Lower Addresses)
    // - Pointer Location: First Available Location
    reg sp_ld;
    reg [ADDR_HIGH: 0] sp_in;
    wire [ADDR_HIGH : 0] sp_out;
    assign sp = sp_out;
    localparam SP_INIT_VAL = {ADDR_WIDTH{1'b1}};
    register #(.DATA_WIDTH(ADDR_WIDTH)) sp_reg(
        .clk(clk),
        .rst_n(rst_n),
        .ld(sp_ld),
        .in(sp_in),
        .inc(sp_inc),
        .dec(sp_dec),
        .out(sp_out)
    );


    // [IRH] Instruction Register HIGH
    // - Width:            DATA_WIDTH (16b)
    reg irh_ld;
    reg [DATA_HIGH: 0] irh_in;
    wire [DATA_HIGH : 0] irh_out;
    register #(.DATA_WIDTH(DATA_WIDTH)) irh_reg(
        .clk(clk),
        .rst_n(rst_n),
        .ld(irh_ld),
        .in(irh_in),
        .out(irh_out)
    );
    wire [3:0] opcode, dst, src1, src2;
    assign {opcode, dst, src1, src2} = irh_out;
    wire [6:0] dst_addr, src1_addr, src2_addr;
    assign dst_addr = {{(ADDR_WIDTH-3){1'b0}}, dst[2:0]}; // {{(ADDR_WIDTH-3){1'b0}}, dst[2:0]};
    assign src1_addr = {{(ADDR_WIDTH-3){1'b0}}, src1[2:0]};
    assign src2_addr = {{(ADDR_WIDTH-3){1'b0}}, src2[2:0]};


    // [IRL] Instruction Register LOW
    // - Width:            DATA_WIDTH (16b)
    reg irl_ld;
    reg [DATA_HIGH : 0] irl_in;
    wire [DATA_HIGH : 0 ] irl_out;
    register #(.DATA_WIDTH(DATA_WIDTH)) irl_reg(
        .clk(clk),
        .rst_n(rst_n),
        .ld(irl_ld),
        .in(irl_in),
        .out(irl_out)
    );
    wire [DATA_HIGH : 0 ] ir_data;


    // [A] Accumulator
    // - Width:            DATA_WIDTH (16b)
    reg a_ld;
    reg [DATA_HIGH : 0] a_in;
    wire [DATA_HIGH : 0] a_out;
    register #(.DATA_WIDTH(DATA_WIDTH)) a_reg(
        .clk(clk),
        .rst_n(rst_n),
        .ld(a_ld),
        .in(a_in),
        .out(a_out)
    );


    // [B] Auxiliary register
    // - Width:            DATA_WIDTH (16b)
    reg b_ld;
    reg [DATA_HIGH : 0] b_in;
    wire [DATA_HIGH : 0] b_out;
    register #(.DATA_WIDTH(DATA_WIDTH)) b_reg(
        .clk(clk),
        .rst_n(rst_n),
        .ld(b_ld),
        .in(b_in),
        .out(b_out)
    );


    // [ALU] Arithmetic Logic Unit
    // - Width            DATA_WIDTH (16b)
    // - Operation Code   (3b)       
    reg [DATA_HIGH : 0] alu_a, alu_b;
    reg [2:0] alu_oc;
    wire [DATA_HIGH : 0] alu_out;
    alu #(.DATA_WIDTH(DATA_WIDTH)) alu_i(
        .oc(alu_oc),
        .a(alu_a),
        .b(alu_b),
        .f(alu_out)
    );
    localparam ALU_ADD = 3'b000;
    localparam ALU_SUB = 3'b001;
    localparam ALU_MUL = 3'b010;



    /* CPU INSTRUCTION PHASE CODES */

    // Instruction Phase Information
    reg [7:0] iphase_reg, iphase_next;
    wire [3:0] iphase;
    assign iphase = iphase_reg[7:4];

    // [0] INITIAL Phase
    localparam INIT = 4'h0;
    localparam initial0 = {INIT, 4'h0};

    // [1] Instruction FETCH Phase
    localparam FETCH = 4'h1;
    localparam fetch0 = {FETCH, 4'h0};
    localparam fetch1 = {FETCH, 4'h1};

    // [2] Instruction DECODE Phase
    localparam DECODE = 4'h2;
    localparam decode0 = {DECODE, 4'h0};
    localparam decode1 = {DECODE, 4'h1};
    localparam decode2 = {DECODE, 4'h2};
    localparam decode3 = {DECODE, 4'h3};
    localparam decode4 = {DECODE, 4'h4};
    localparam decode5 = {DECODE, 4'h5};
    localparam decode6 = {DECODE, 4'h6};
    localparam decode7 = {DECODE, 4'h7};
    localparam decode8 = {DECODE, 4'h8};
    localparam decode9 = {DECODE, 4'h9};

    // [3] Instructin EXECUTE Phase
    localparam EXECUTE = 4'h3;
    localparam execute0 = {EXECUTE, 4'h0};
    localparam execute1 = {EXECUTE, 4'h1};

    // [4] Instruction WRITE-BACK Phase
    localparam WRITEBACK = 4'h4;
    localparam writeback0 = {WRITEBACK, 4'h0};
    localparam writeback1 = {WRITEBACK, 4'h1};
    localparam writeback2 = {WRITEBACK, 4'h2};

    // [5] FINISH Phase
    localparam FINISH = 4'h5;
    localparam finishloop = {FINISH, 4'h0};



    /* CPU INSTRUCTION OPERATION CODES */
    localparam MOV = 4'b0000;
    localparam IN = 4'b0111;
    localparam OUT = 4'b1000;

    localparam ADD = 4'b0001;
    localparam SUB = 4'b0010;
    localparam MUL = 4'b0011;
    localparam DIV = 4'b0100;

    localparam STOP = 4'b1111;



    /* CPU IMPLEMENTATION */
    // initial begin
    //     $monitor("out: %16b", out);
    // end
    // Output Logic
    reg [DATA_HIGH : 0] out_reg, out_next;
    assign out = out_reg;

    // Sequential Logic
    always @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            iphase_reg <= initial0;
            out_reg <= {DATA_WIDTH{1'b0}};
        end
        else begin
            iphase_reg <= iphase_next;
            out_reg <= out_next;
        end
    end

    // Combinational Logic
    always @(*) begin
        iphase_next = iphase_reg;
        out_next = out_reg;

        // Reset PC Inputs
        {pc_ld, pc_inc} = 2'b00;
        pc_in = {ADDR_WIDTH{1'b0}};

        // Reset SP Inputs
        sp_ld = 1'b0;
        sp_in = {ADDR_WIDTH{1'b0}};

        // Reset IRH Inputs
        irh_ld = 1'b0;
        irh_in = {DATA_WIDTH{1'b0}};

        // Reset IRL Inputs
        irl_ld = 1'b0;
        irl_in = {DATA_WIDTH{1'b0}};

        // Reset A Inputs
        a_ld = 1'b0;
        a_in = {DATA_WIDTH{1'b0}};

        // Reset B Inputs
        b_ld = 1'b0;
        b_in = {DATA_WIDTH{1'b0}};

        // Reset ALU Inputs
        alu_a = {DATA_WIDTH{1'b0}};
        alu_b = {DATA_WIDTH{1'b0}};
        alu_oc = 3'b000;

        // Reset MEM signals
        mem_we = 1'b0;
        mem_addr = {ADDR_WIDTH{1'b0}};
        mem_data = {DATA_WIDTH{1'b0}};






        case (iphase)

            INIT : begin
                case(iphase_reg)

                    initial0 : begin
                        // $display("init");
                        //$display("time: %4d phase: %8b pc: %6pc sp %6b", $time, iphase_reg, pc, sp);
                        pc_in = PC_INIT_VAL;
                        pc_ld = 1'b1;
                        sp_in = SP_INIT_VAL;
                        sp_ld = 1'b1;

                        iphase_next = fetch0;
                    end

                endcase
            end





            FETCH : begin
                case (iphase_reg)

                    fetch0 : begin
                        // $display("fetch0");
                        //$display("time: %4d phase: %8b pc: %6b sp: %6b", $time, iphase_reg, pc, sp);

                        mem_we = 1'b0;
                        mem_addr = pc_out;
                        pc_inc = 1'b1;

                        iphase_next = fetch1;
                    end

                    fetch1 : begin
                        // $display("fetch1");
                        //$display("mem_in: %16b", mem_in);
                        irh_in = mem_in;
                        irh_ld = 1'b1;

                        iphase_next = decode0;
                    end

                endcase
            end





            DECODE : begin
                case (opcode)
            
                    MOV : begin
                        case (iphase_reg)

                        decode0 : begin
                            // $display("mov");
                            // Load Second Operand if Third Operand is b0000
                            if (src2 == 4'b0000) begin
                                mem_we = 1'b0;
                                mem_addr = src1_addr;
                                
                                iphase_next = decode2;
                            end
                            // Load Second Instruction Word if Third Operand is b1000
                            else if (src2 == 4'b1000) begin
                                mem_we = 1'b0;
                                mem_addr = src1_addr;
                                pc_inc = 1'b1;

                                iphase_next = decode1;
                            end
                            else
                                iphase_next = finishloop;
                        end

                        decode1 : begin
                            irl_in = mem_in;
                            irl_ld = 1'b1;

                            iphase_next = execute0;
                        end

                        decode2 : begin
                            a_ld = 1'b1;
                            a_in = mem_in;

                            iphase_next = (src1[3] ? decode3 : writeback0);
                        end

                        decode3 : begin
                            mem_addr = a_out;
                            mem_we = 1'b0;

                            iphase_next = decode4;
                        end

                        decode4 : begin
                            a_ld = 1'b1;
                            a_in = mem_in;

                            iphase_next = writeback0;
                        end

                        endcase
                    end



                    OUT : begin
                        case (iphase_reg)

                            decode0 : begin
                                // $display("out");
                                // $display("opcode: %4b dst: %4b src1: %4b src2: %4b", opcode, dst, src1, src2);
                                // $display("time: %4d phase: %8b pc: %6b sp: %6b", $time, iphase_reg, pc, sp);
                                mem_we = 1'b0;
                                mem_addr = dst_addr;
                                // $display("dst_addr: %6b", dst_addr);

                                iphase_next = decode1;
                            end

                            decode1 : begin
                                // $display("out1");
                                //$display("time: %4d phase: %8b pc: %6b sp: %6b", $time, iphase_reg, pc, sp);
                                a_ld = 1'b1;
                                a_in = mem_in;
                                // $display("mem_data: %16b", mem_in);

                                iphase_next = (dst[3] ? decode2 : execute0);
                            end

                            decode2 : begin
                                // $display("out2");
                                mem_we = 1'b0;
                                mem_addr = a_out;
                                // $display("addr: %6b", a_out);

                                iphase_next = decode3;
                            end

                            decode3 : begin
                                // $display("out3");
                                a_ld = 1'b1;
                                a_in = mem_in;
                                // $display("mem_data: %16b", mem_in);
                                iphase_next = execute0;
                            end

                        endcase
                    end



                    IN : begin
                        case (iphase_reg)

                            decode0 : begin
                                // $display("in");
                                a_ld = 1'b1;
                                a_in = in;
                                // $display("in: %16b", in);

                                iphase_next = writeback0;
                            end

                        endcase
                    end



                    ADD, SUB, MUL : begin
                        case (iphase_reg)

                            decode0 : begin
                                mem_we = 1'b0;
                                mem_addr = src1_addr;

                                iphase_next = decode1;
                            end

                            decode1 : begin
                                a_ld = 1'b1;
                                a_in = mem_in;

                                iphase_next = (src1[3] ? decode2 : decode4);
                            end

                            decode2 : begin
                                mem_we = 1'b0;
                                mem_addr = a_out[ADDR_HIGH : 0];

                                iphase_next = decode3;
                            end

                            decode3 : begin
                                a_ld = 1'b1;
                                a_in = mem_in;

                                iphase_next = decode4;
                            end

                            decode4 : begin
                                mem_we = 1'b0;
                                mem_addr = src2_addr;

                                iphase_next = decode5;
                            end

                            decode5: begin
                                b_ld = 1'b1;
                                b_in = mem_in;

                                iphase_next = (src2[3] ? decode6 : execute0);
                            end

                            decode6 : begin
                                mem_we = 1'b0;
                                mem_addr = b_out[ADDR_HIGH : 0];

                                iphase_next = decode7;
                            end

                            decode7 : begin
                                b_ld = 1'b1;
                                b_in = mem_in;

                                iphase_next = execute0;
                            end 

                        endcase
                    end



                    DIV : begin
                        iphase_next = fetch0;
                    end



                    STOP : begin
                        case (iphase_reg)

                            decode0 : begin
                                // $display("stop0");
                                if (dst) begin
                                    mem_we = 1'b0;
                                    mem_addr = dst_addr;
                                    iphase_next = decode1;
                                end
                                else if (src1) begin
                                    mem_we = 1'b0;
                                    mem_addr = src1_addr;
                                    iphase_next = decode4; // TODO
                                end
                                else if (src2) begin
                                    mem_we = 1'b0;
                                    mem_addr = src2_addr;
                                    iphase_next = decode7; // TODO
                                end
                                else
                                    iphase_next = finishloop;
                            end

                            decode1 : begin
                                // $display("stop1");
                                if (dst[3]) begin
                                    a_ld = 1'b1;
                                    a_in = mem_in;

                                    iphase_next = decode2;
                                end
                                else begin
                                    out_next = mem_in;

                                if (src1) begin
                                    mem_we = 1'b0;
                                    mem_addr = src1_addr;
                                    iphase_next = decode4; // TODO
                                end
                                else if (src2) begin
                                    mem_we = 1'b0;
                                    mem_addr = src2_addr;
                                    iphase_next = decode7; // TODO
                                end
                                else iphase_next = finishloop;
                                end
                            end

                            decode2 : begin
                                // $display("stop2");
                                mem_we = 1'b0;
                                mem_addr = a_out[ADDR_HIGH : 0];

                                iphase_next = decode3;
                            end

                            decode3 : begin
                                // $display("stop3");
                                out_next = mem_in;

                                if (src1) begin
                                    mem_we = 1'b0;
                                    mem_addr = src1_addr;

                                    iphase_next = decode4;
                                end
                                else if (src2) begin
                                    mem_we = 1'b0;
                                    mem_addr = src2_addr;

                                    iphase_next = decode7;
                                end
                                else iphase_next = finishloop;
                            end

                            decode4 : begin
                                // $display("stop4");
                                if (src1[3]) begin
                                    a_ld = 1'b1;
                                    a_in = mem_in;

                                    iphase_next = decode5;
                                end
                                else begin
                                    out_next = mem_in;

                                    if (src2) begin
                                    mem_we = 1'b0;
                                    mem_addr = src2_addr;

                                    iphase_next = decode7;
                                    end
                                    else iphase_next = finishloop;
                                end
                            end

                            decode5 : begin
                                // $display("stop5");
                                mem_we = 1'b0;
                                mem_addr = a_out[ADDR_HIGH : 0];

                                iphase_next = decode6;
                            end

                            decode6 : begin
                                // $display("stop6");
                                out_next = mem_in;

                                if (src2) begin
                                    mem_we = 1'b0;
                                    mem_addr = src2_addr;

                                    iphase_next = decode7;
                                end
                                else iphase_next = finishloop;
                            end

                            decode7 : begin
                                // $display("stop7");
                                if (src2[3]) begin
                                    a_ld = 1'b1;
                                    a_in = mem_in;

                                    iphase_next = decode8;
                                end
                                else begin
                                    out_next = mem_in;

                                    iphase_next= finishloop;
                                end
                            end

                            decode8 : begin
                                // $display("stop8");
                                mem_we = 1'b0;
                                mem_addr = a_out[ADDR_HIGH : 0];

                                iphase_next = decode9;
                            end

                            decode9 : begin
                                // $display("stop9");
                                out_next = mem_in;

                                iphase_next = finishloop;
                            end
                            
                        endcase
                    end



                    default: begin
                        // $display("Invalid OP Code: %4b", opcode);
                        iphase_next = finishloop;
                    end

                endcase
            end





            EXECUTE : begin
                case (opcode)

                    MOV : begin
                        a_in = irl_out;
                        a_ld = 1'b1;

                        iphase_next = writeback0;
                    end



                    OUT : begin
                        // $display("out_exec");
                        out_next = a_out;
                        // $display("acc: %16b", a_out);

                        iphase_next = fetch0;
                    end



                    ADD, SUB, MUL : begin
                        case (opcode)
                            ADD : alu_oc = ALU_ADD;
                            SUB : alu_oc = ALU_SUB;
                            MUL : alu_oc = ALU_MUL;
                        endcase
                        alu_a = a_out;
                        alu_b = b_out;
                        a_ld = 1'b1;
                        a_in = alu_out;

                        iphase_next = writeback0;
                    end



                endcase
            end





            WRITEBACK : begin
                case (opcode)

                    MOV, IN, ADD, SUB, MUL : begin
                        case (iphase_reg)

                            writeback0 : begin
                                // $display("writeback0");
                                if (dst[3]) begin
                                    mem_addr = dst_addr;
                                    mem_we = 1'b0;

                                    iphase_next = writeback1;
                                end
                                else begin
                                    mem_addr = dst_addr;
                                    mem_we = 1'b1;
                                    mem_data = a_out;
                                    // $display("in wb");
                                    // $display("acc: %16b", a_out);
                                    // $display("dst_addr: %16b", dst_addr);

                                    iphase_next = fetch0;
                                end
                            end

                            writeback1 : begin
                                // $display("writeback1");
                                b_ld = 1'b1;
                                b_in = mem_in;

                                iphase_next = writeback2;
                            end

                            writeback2 : begin
                                // $display("writeback2");
                                mem_addr = b_out;
                                mem_we = 1'b1;
                                mem_data = a_out;

                                iphase_next = fetch0;
                            end

                        endcase
                    end
                endcase

            end

            FINISH : begin
                case (iphase_reg)

                finishloop : begin
                    // $display("finish");
                    // #2 $finish;
                end

                endcase 
            end
        endcase

    end
endmodule
