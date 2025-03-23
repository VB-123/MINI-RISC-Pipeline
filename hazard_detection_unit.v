module HAZARD_Unit_very_old (
    input wire [4:0] opcode_D,       // Opcode in Decode stage
    input wire [4:0] opcode_E,       // Opcode in Execute stage
    input wire [2:0] rs1_D,          // Source register 1 in Decode 
    input wire [2:0] rs2_D,          // Source register 2 in Decode
    input wire [2:0] rd_E,           // Destination register in Execute
    input wire [2:0] rd_W,           // Destination register in Writeback
    input wire mem_read_E,           // Memory read in Execute stage
    input wire branch_D,             // Branch instruction in Decode
    input wire jump_D,               // Jump instruction in Decode
    input wire reg_write_E,          // Register write in Execute
    input wire reg_write_W,          // Register write in Writeback
    
    output reg stall_F,              // Stall Fetch stage
    output reg stall_D,              // Stall Decode stage
    output reg flush_F,              // Flush Fetch stage
    output reg flush_D,              // Flush Decode stage
    output reg [1:0] forward_A,      // Forward control for ALU input A
    output reg [1:0] forward_B       // Forward control for ALU input B
);

    // Data hazard detection (Load-Use hazard)
    always @(*) begin
        // Default: no stall
        stall_F = 1'b0;
        stall_D = 1'b0;
        
        // Load-use hazard: When a load instruction is followed by an instruction that uses the loaded data
        if (mem_read_E && ((rd_E == rs1_D) || (rd_E == rs2_D))) begin
            stall_F = 1'b1;
            stall_D = 1'b1;
        end
    end
    
    // Control hazard detection (Branch/Jump)
    always @(*) begin
        // Default: no flush
        flush_F = 1'b0;
        flush_D = 1'b0;
        
        // When branch or jump instruction is detected, flush the pipeline
        if (branch_D || jump_D) begin
            flush_F = 1'b1;
            flush_D = 1'b1;
        end
    end
    
    // Forwarding logic for data hazards
    always @(*) begin
        // Default: no forwarding
        forward_A = 2'b00;  // No forwarding
        forward_B = 2'b00;  // No forwarding
        
        // Forward from Execute/Memory stage (EX hazard)
        if (reg_write_E && (rd_E != 0) && (rd_E == rs1_D))
            forward_A = 2'b01;
            
        if (reg_write_E && (rd_E != 0) && (rd_E == rs2_D))
            forward_B = 2'b01;
            
        // Forward from Writeback stage (MEM hazard)
        if (reg_write_W && (rd_W != 0) && !(reg_write_E && (rd_E != 0) && (rd_E == rs1_D)) && (rd_W == rs1_D))
            forward_A = 2'b10;
            
        if (reg_write_W && (rd_W != 0) && !(reg_write_E && (rd_E != 0) && (rd_E == rs2_D)) && (rd_W == rs2_D))
            forward_B = 2'b10;
    end
    
endmodule
