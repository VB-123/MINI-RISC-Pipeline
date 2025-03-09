`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Vasanthi 
// 
// Create Date: 13.02.2025 11:47:45
// Design Name: 
// Module Name: ALU_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module ALU_tb();
    `include "parameters.v"
    // Inputs
    reg clk;                    // Add clock
    reg reset;
    reg [4:0] opcode;
    reg [15:0] operand_1;
    reg [15:0] operand_2;
    reg [3:0] bit_position;
    
    // Outputs
    wire [15:0] result_0;
    wire [15:0] result_1;
    wire [15:0] current_flags;  // Change to wire since it's now an output from Flag_Register
    wire [15:0] next_flags;
   
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 100MHz clock
    end

    // Instantiate the ALU
    ALU alu (
        .opcode(opcode),
        .operand_1(operand_1),
        .operand_2(operand_2),
        .bit_position(bit_position),
        .current_flags(current_flags),
        .result_0(result_0),
        .result_1(result_1),
        .next_flags(next_flags)
    );

    // Instantiate the Flag Register
    Flag_Register flag_reg (
        .clk(clk),
        .reset(reset),
        .next_flags(next_flags),
        .current_flags(current_flags)
    );

    integer num_failures;
    initial begin
        num_failures = 0;
    end
    
    // Test stimulus
    initial begin
        // Initialize inputs
        clk = 0;
        reset = 1;
        opcode = 0;
        operand_1 = 0;
        operand_2 = 0;
        bit_position = 0;

        #20 reset = 0;
        // Wait for 100 ns for global reset
        #100;
        

	/* Syntax: test_case (input [64:0] test_name; input [4:0] test_opcode; 
			      input [15:0] test_op1; input [15:0] test_op2; input [3:0] test_bit_pos;) */
        // Test ADD operation
        // Normal addition
        test_case("ADD - Normal", ADD, 16'd10, 16'd20, 4'd0);
        // Addition with carry
        test_case("ADD - Carry", ADD, 16'hFFFF, 16'd1, 4'd0);
        // Addition with overflow
        test_case("ADD - Overflow", ADD, 16'h7FFF, 16'h7FFF, 4'd0);
        
        // Test MUL operation
        test_case("MUL - Normal", MUL, 16'd10, 16'd20, 4'd0);
        test_case("MUL - Large", MUL, 16'hFFFF, 16'h2, 4'd0);
        
        // Test SUB operation
        test_case("SUB - Normal", SUB, 16'd20, 16'd10, 4'd0);
        test_case("SUB - Negative", SUB, 16'd40, 16'd29, 4'd0);
        
        // Test DIV operation
        test_case("DIV - Normal", DIV, 16'd20, 16'd5, 4'd0);
        test_case("DIV - By Zero", DIV, 16'd20, 16'd0, 4'd0);
        
        // Test logical operations
        test_case("NOT", NOT, 16'hAAAA, 16'd0, 4'd0);
        test_case("AND", AND, 16'hAAAA, 16'h5555, 4'd0);
        test_case("OR", OR, 16'hAAAA, 16'h5555, 4'd0);
        test_case("XOR", XOR, 16'hAAAA, 16'h5555, 4'd0);
        
        // Test INC operation
        test_case("INC - Normal", INC, 16'd10, 16'd0, 4'd0);
        test_case("INC - Overflow", INC, 16'hFFFF, 16'd0, 4'd0);
        
        // Test CMP operation
        test_case("CMP - Equal", CMP, 16'd10, 16'd10, 4'd0);
        test_case("CMP - Greater", CMP, 16'd20, 16'd10, 4'd0);
        test_case("CMP - Less", CMP, 16'd10, 16'd20, 4'd0);
        
        // Test rotation operations
        test_case("RR", RR, 16'hAAAA, 16'd0, 4'd0);
        test_case("RL", RL, 16'hAAAA, 16'd0, 4'd0);
        
        // Test bit operations
        test_case("SETB", SETB, 16'h0000, 16'd0, 4'd8);
        test_case("CLRB", CLRB, 16'hFFFF, 16'd0, 4'd8);
        test_case("CPLB", CPLB, 16'hFFFF, 16'd0, 4'd8);
        // Test setting different flag bits
        test_case("SETF - Carry Flag", SETF, 16'h0000, 16'h0000, 4'd0);
        test_case("SETF - Zero Flag", SETF, 16'h0000, 16'h0000, 4'd7);
        test_case("SETF - Negative Flag", SETF, 16'h0000, 16'h0000, 4'd6);
        test_case("SETF - General Purpose Flag", SETF, 16'h0000, 16'h0000, 4'd4);
        
        test_case("CLRF - Carry Flag", CLRF, 16'h0000, 16'h0000, 4'd0);
        test_case("CPLF - Carry Flag",  CPLF, 16'h0000, 16'h0000, 4'd0);

        $display("\n=== Test Summary ===");
        if (num_failures == 0)
            $display("All tests passed successfully!");
        else
            $display("Some tests failed.");
        // End simulation
        #100 $finish;
    end
    
    // Task to run test cases
    task test_case;
        input [255:0] test_name;
        input [4:0] test_opcode;
        input [15:0] test_op1;
        input [15:0] test_op2;
        input [3:0] test_bit_pos;
        reg failed;
        reg [31:0] expected_mul;
        reg [15:0] expected_div, expected_mod;
        
        begin
            failed = 0;
            $display("\nRunning test: %s", test_name);
            opcode = test_opcode;
            operand_1 = test_op1;
            operand_2 = test_op2;
            bit_position = test_bit_pos;
            @(posedge clk);  // Wait for flag register to update
            case(test_opcode)
                ADD: begin // ADD
                    if(result_0 !== test_op1 + test_op2) begin
                        failed = 1;
                        $display("\nFAILURE in %s:", test_name);
                        $display("ADD operation failed:");
                        $display("Expected: %h, Got: %h", test_op1 + test_op2, result_0);
                        $display("Carry flag: %b", next_flags[0]);
                    end
                    $display("Flag register: %h", next_flags);
                end
                
                MUL: begin // MUL
                    expected_mul = test_op1 * test_op2;
                    if(result_0 !== expected_mul[15:0] || result_1 !== expected_mul[31:16]) begin
                        failed = 1;
                        $display("\nFAILURE in %s:", test_name);
                        $display("MUL operation failed:");
                        $display("Expected: %h%h, Got: %h%h", expected_mul[31:16], expected_mul[15:0], result_1, result_0);
                    end
                    $display("Flag register: %h", next_flags);
                end
                
                SUB: begin // SUB
                    if(result_0 !== test_op1 - test_op2) begin
                        failed = 1;
                        $display("\nFAILURE in %s:", test_name);
                        $display("SUB operation failed:");
                        $display("Expected: %h, Got: %h", test_op1 - test_op2, result_0);
                        $display("Borrow flag: %b", next_flags[0]);
                    end
                    $display("Flag register: %h", next_flags);
                end
                
                DIV: begin // DIV
                    if(test_op2 != 0) begin
                        expected_div = test_op1 / test_op2;
                        expected_mod = test_op1 % test_op2;
                        if(result_0 !== expected_div || result_1 !== expected_mod) begin
                            failed = 1;
                            $display("\nFAILURE in %s:", test_name);
                            $display("DIV operation failed:");
                            $display("Expected quotient: %h, Got: %h", expected_div, result_0);
                            $display("Expected remainder: %h, Got: %h", expected_mod, result_1);
                        end
                    end else if(result_0 !== 16'hFFFF || !next_flags[1]) begin
                        failed = 1;
                        $display("\nFAILURE in %s:", test_name);
                        $display("DIV by zero handling failed");
                    end
                    $display("Flag register: %h", next_flags);
                end
                
                NOT: begin // NOT
                    if(result_0 !== ~test_op1) begin
                        failed = 1;
                        $display("\nFAILURE in %s:", test_name);
                        $display("NOT operation failed:");
                        $display("Expected: %h, Got: %h", ~test_op1, result_0);
                    end
                    $display("Flag register: %h", next_flags);
                end
                
                AND: begin // AND
                    if(result_0 !== (test_op1 & test_op2)) begin
                        failed = 1;
                        $display("\nFAILURE in %s:", test_name);
                        $display("AND operation failed:");
                        $display("Expected: %h, Got: %h", test_op1 & test_op2, result_0);
                    end
                    $display("Flag register: %h", next_flags);
                end
                
                OR: begin // OR
                    if(result_0 !== (test_op1 | test_op2)) begin
                        failed = 1;
                        $display("\nFAILURE in %s:", test_name);
                        $display("OR operation failed:");
                        $display("Expected: %h, Got: %h", test_op1 | test_op2, result_0);
                    end
                    $display("Flag register: %h", next_flags);
                end
                
                XOR: begin // XOR
                    if(result_0 !== (test_op1 ^ test_op2)) begin
                        failed = 1;
                        $display("\nFAILURE in %s:", test_name);
                        $display("XOR operation failed:");
                        $display("Expected: %h, Got: %h", test_op1 ^ test_op2, result_0);
                    end
                    $display("Flag register: %h", next_flags);
                end
                
                INC: begin // INC
                    if(result_0 !== test_op1 + 16'h0001) begin
                        failed = 1;
                        $display("\nFAILURE in %s:", test_name);
                        $display("INC operation failed:");
                        $display("Expected: %h, Got: %h", test_op1 + 16'h0001, result_0);
                        $display("Overflow flag: %b", next_flags[1]);
                    end
                    $display("Flag register: %h", next_flags);
                end
                
                CMP: begin // CMP
                    if((test_op1 == test_op2 && !next_flags[3]) ||
                       (test_op1 > test_op2 && !next_flags[2]) ||
                       (test_op1 < test_op2 && !next_flags[0])) begin
                        failed = 1;
                        $display("\nFAILURE in %s:", test_name);
                        $display("CMP flags incorrect:");
                        $display("Expected: Equal=%b, Greater=%b, Less=%b",
                               test_op1 == test_op2,
                               test_op1 > test_op2,
                               test_op1 < test_op2);
                        $display("Got flags: %b", next_flags);
                    end
                    $display("Flag register: %h", next_flags);
                end
                
                RR: begin // RR
                    if(result_0 !== {test_op1[0], test_op1[15:1]}) begin
                        failed = 1;
                        $display("\nFAILURE in %s:", test_name);
                        $display("RR operation failed:");
                        $display("Expected: %h, Got: %h", {test_op1[0], test_op1[15:1]}, result_0);
                    end
                    $display("Flag register: %h", next_flags);
                end
                
                RL: begin // RL
                    if(result_0 !== {test_op1[14:0], test_op1[15]}) begin
                        failed = 1;
                        $display("\nFAILURE in %s:", test_name);
                        $display("RL operation failed:");
                        $display("Expected: %h, Got: %h", {test_op1[14:0], test_op1[15]}, result_0);
                    end
                    $display("Flag register: %h", next_flags);
                end
                
                SETB: begin // SETB
                    if(result_0 !== (test_op1 | (16'b1 << test_bit_pos))) begin
                        failed = 1;
                        $display("\nFAILURE in %s:", test_name);
                        $display("SETB operation failed:");
                        $display("Expected: %h, Got: %h", test_op1 | (16'b1 << test_bit_pos), result_0);
                    end
                    $display("Flag register: %h", next_flags);
                end
                
                CLRB: begin // CLRB
                    if(result_0 !== (test_op1 & ~(16'b1 << test_bit_pos))) begin
                        failed = 1;
                        $display("\nFAILURE in %s:", test_name);
                        $display("CLRB operation failed:");
                        $display("Expected: %h, Got: %h", test_op1 & ~(16'b1 << test_bit_pos), result_0);
                    end
                    $display("Flag register: %h", next_flags);
                end
                
                CPLB: begin // CPLB
                    if(result_0 !== (test_op1 ^ (16'b1 << test_bit_pos))) begin
                      failed = 1;
                      $display("\nFAILURE in %s:", test_name);
                      $display("CPLB operation failed:");
                      $display("Expected: %h, Got: %h", test_op1 ^ (16'b1 << test_bit_pos), result_0);
                    end
                    $display("Flag register: %h", next_flags);
                end

                SETF: begin // SETF
                    if(!next_flags[test_bit_pos]) begin
                        failed = 1;
                        $display("\nFAILURE in %s:", test_name);
                        $display("SETF operation failed:");
                        $display("Flag bit %d not set", test_bit_pos);
                    end
                    $display("Flag register: %h", next_flags);
                end
                
                CLRF: begin
                  if(next_flags[test_bit_pos]) begin
                    failed = 1;
                    $display("\nFAILURE in %s:", test_name);
                    $display("CLRF operation failed:");
                    $display("Flag bit %d not cleared", test_bit_pos);
                  end
                  $display("Flag register: %h", next_flags);
                end

                CPLF: begin
                    if(next_flags[test_bit_pos] === current_flags[test_bit_pos]) begin
                        failed = 1;
                        $display("\nFAILURE in %s:", test_name);
                        $display("CPLF operation failed:");
                        $display("Flag bit %d not complemented", test_bit_pos);
                    end
                end
            endcase
            
                if(failed) begin
                  num_failures = num_failures + 1;
                end
        end
    endtask
    
    // Wave dump
    initial begin
        $dumpfile("alu_test.vcd");
        $dumpvars(0, ALU_tb);
    end
    
endmodule
