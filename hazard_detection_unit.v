// F/D Stage Register
module FD_Register (
    input wire clk,
    input wire reset,
    input wire stall_F,
    input wire flush_F,
    
    // Inputs from Fetch stage
    input wire [15:0] instruction_in,
    input wire [10:0] pc_in,
    
    // Outputs to Decode stage
    output reg [15:0] instruction_out,
    output reg [10:0] pc_out
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            instruction_out <= 16'b0;
            pc_out <= 11'b0;
        end
        else if (flush_F) begin
            instruction_out <= 16'b0;  // NOP instruction
            pc_out <= 11'b0;
        end
        else if (!stall_F) begin
            instruction_out <= instruction_in;
            pc_out <= pc_in;
        end
    end
endmodule

// D/E Stage Register
module DE_Register (
    input wire clk,
    input wire reset,
    input wire stall_D,
    input wire flush_D,
    
    // Inputs from Decode stage
    input wire [4:0] opcode_in,
    input wire [2:0] reg_write_addr_in,
    input wire [15:0] reg_data_1_in,
    input wire [15:0] reg_data_2_in,
    input wire [7:0] immediate_in,
    input wire [3:0] bit_position_in,
    input wire [10:0] pc_in,
    
    // Control signals from Decode
    input wire alu_src_in,
    input wire reg_write_in,
    input wire mem_read_in,
    input wire mem_write_in,
    input wire [1:0] write_mode_in,
    
    // Outputs to Execute stage
    output reg [4:0] opcode_out,
    output reg [2:0] reg_write_addr_out,
    output reg [15:0] reg_data_1_out,
    output reg [15:0] reg_data_2_out,
    output reg [7:0] immediate_out,
    output reg [3:0] bit_position_out,
    output reg [10:0] pc_out,
    
    // Control signals to Execute
    output reg alu_src_out,
    output reg reg_write_out,
    output reg mem_read_out,
    output reg mem_write_out,
    output reg [1:0] write_mode_out
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            opcode_out <= 5'b0;
            reg_write_addr_out <= 3'b0;
            reg_data_1_out <= 16'b0;
            reg_data_2_out <= 16'b0;
            immediate_out <= 8'b0;
            bit_position_out <= 4'b0;
            pc_out <= 11'b0;
            
            // Control signals
            alu_src_out <= 1'b0;
            reg_write_out <= 1'b0;
            mem_read_out <= 1'b0;
            mem_write_out <= 1'b0;
            write_mode_out <= 2'b0;
        end
        else if (flush_D) begin
            opcode_out <= 5'b0;
            reg_write_addr_out <= 3'b0;
            reg_data_1_out <= 16'b0;
            reg_data_2_out <= 16'b0;
            immediate_out <= 8'b0;
            bit_position_out <= 4'b0;
            pc_out <= 11'b0;
            
            // Control signals
            alu_src_out <= 1'b0;
            reg_write_out <= 1'b0;
            mem_read_out <= 1'b0;
            mem_write_out <= 1'b0;
            write_mode_out <= 2'b0;
        end
        else if (!stall_D) begin
            opcode_out <= opcode_in;
            reg_write_addr_out <= reg_write_addr_in;
            reg_data_1_out <= reg_data_1_in;
            reg_data_2_out <= reg_data_2_in;
            immediate_out <= immediate_in;
            bit_position_out <= bit_position_in;
            pc_out <= pc_in;
            
            // Control signals
            alu_src_out <= alu_src_in;
            reg_write_out <= reg_write_in;
            mem_read_out <= mem_read_in;
            mem_write_out <= mem_write_in;
            write_mode_out <= write_mode_in;
        end
    end
endmodule

// E/W Stage Register
module EW_Register (
    input wire clk,
    input wire reset,
    
    // Inputs from Execute stage
    input wire [4:0] opcode_in,
    input wire [2:0] reg_write_addr_in,
    input wire [15:0] alu_result_0_in,
    input wire [15:0] alu_result_1_in,
    input wire [15:0] mem_data_in,
    
    // Control signals from Execute
    input wire reg_write_in,
    input wire mem_to_reg_in,
    input wire [1:0] write_mode_in,
    
    // Outputs to Writeback stage
    output reg [4:0] opcode_out,
    output reg [2:0] reg_write_addr_out,
    output reg [15:0] alu_result_0_out,
    output reg [15:0] alu_result_1_out,
    output reg [15:0] mem_data_out,
    
    // Control signals to Writeback
    output reg reg_write_out,
    output reg mem_to_reg_out,
    output reg [1:0] write_mode_out
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            opcode_out <= 5'b0;
            reg_write_addr_out <= 3'b0;
            alu_result_0_out <= 16'b0;
            alu_result_1_out <= 16'b0;
            mem_data_out <= 16'b0;
            
            // Control signals
            reg_write_out <= 1'b0;
            mem_to_reg_out <= 1'b0;
            write_mode_out <= 2'b0;
        end
        else begin
            opcode_out <= opcode_in;
            reg_write_addr_out <= reg_write_addr_in;
            alu_result_0_out <= alu_result_0_in;
            alu_result_1_out <= alu_result_1_in;
            mem_data_out <= mem_data_in;
            
            // Control signals
            reg_write_out <= reg_write_in;
            mem_to_reg_out <= mem_to_reg_in;
            write_mode_out <= write_mode_in;
        end
    end
endmodule
