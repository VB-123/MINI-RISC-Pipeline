// FD Register
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
            instruction_out <= 16'b0;
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
    input wire [2:0] reg_write_addr_in, // rd_D
    input wire [2:0] source_reg1_in, // rs1_D
    input wire [2:0] source_reg2_in, // rs2_D
    input wire [15:0] reg_data_1_in,
    input wire [15:0] reg_data_2_in,
    input wire [7:0] immediate_in,
    input wire [3:0] bit_position_in,
    input wire [10:0] pc_in,
    input wire [15:0] flags_in,
    input wire [10:0] branch_addr_in,
    
    // Control signals 
    input wire alu_src_in,
    input wire [1:0] reg_write_in,
    input wire mem_write_in,
    input wire mem_to_reg_in,
    input wire mem_read_in,
    input wire read_write_in,
    input wire alu_op_in,  // ALU enable signal input
    input wire branch_en_in,
    input wire io_op_in,    // IO operation enable input

    // Outputs to Execute stage
    output reg [4:0] opcode_out,
    output reg [2:0] reg_write_addr_out, //rd_E
    output reg [2:0] source_reg1_out, //rs1_E
    output reg [2:0] source_reg2_out, //rs2_E
    output reg [15:0] reg_data_1_out,
    output reg [15:0] reg_data_2_out,
    output reg [7:0] immediate_out,
    output reg [3:0] bit_position_out,
    output reg [10:0] pc_out,
    output reg [15:0] flags_out,
    output reg [10:0] branch_addr_out,
    output reg [10:0] mem_read_addr_out,
    
    // Control signals to Execute
    output reg alu_src_out,
    output reg read_write_out,
    output reg mem_write_out,
    output reg mem_to_reg_out,
    output reg [1:0] write_mode_out,
    output reg mem_read_out,
    output reg alu_op_out,  // ALU enable signal output
    output reg io_op_out,
    output reg branch_en_out
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            opcode_out <= 5'b0;
            reg_write_addr_out <= 3'b0;
            source_reg1_out <= 3'b0;
            source_reg2_out <= 3'b0;
            reg_data_1_out <= 16'b0;
            reg_data_2_out <= 16'b0;
            immediate_out <= 8'b0;
            bit_position_out <= 4'b0;
            pc_out <= 11'b0;
            flags_out <= 16'b0;
            branch_addr_out <= 11'b0;
            mem_read_addr_out <= 11'b0;
            
            // Control signals
            alu_src_out <= 1'b0;
            read_write_out <= 1'b0;
            mem_read_out <= 1'b0;
            mem_write_out <= 1'b0;
            mem_to_reg_out <= 1'b0;
            write_mode_out <= 2'b00;
            alu_op_out <= 1'b0;  // Reset ALU enable signal
            io_op_out <= 1'b0;
            branch_en_out <= 1'b0;
        end
        else if (flush_D) begin
            opcode_out <= 5'b0;
            reg_write_addr_out <= 3'b0;
            source_reg1_out <= 3'b0;
            source_reg2_out <= 3'b0;
            reg_data_1_out <= 16'b0;
            reg_data_2_out <= 16'b0;
            immediate_out <= 8'b0;
            bit_position_out <= 4'b0;
            pc_out <= 11'b0;
            flags_out <= 16'b0;
            branch_addr_out <= 11'b0;
            mem_read_addr_out <= 11'b0;
            
            // Control signals
            alu_src_out <= 1'b0;
            read_write_out <= 1'b0;
            mem_read_out <= 1'b0;
            mem_write_out <= 1'b0;
            mem_to_reg_out <= 1'b0;
            write_mode_out <= 2'b00;
            alu_op_out <= 1'b0;
            io_op_out <= 1'b0;
            branch_en_out <= 1'b0;
        end
        else if (!stall_D) begin
            opcode_out <= opcode_in;
            reg_write_addr_out <= reg_write_addr_in;
            source_reg1_out <= source_reg1_in;
            source_reg2_out <= source_reg2_in;
            reg_data_1_out <= reg_data_1_in;
            reg_data_2_out <= reg_data_2_in;
            immediate_out <= immediate_in;
            bit_position_out <= bit_position_in;
            pc_out <= pc_in;
            flags_out <= flags_in;
            branch_addr_out <= branch_addr_in;
            mem_read_addr_out <= reg_data_1_in [10:0];
            
            // Control signals
            alu_src_out <= alu_src_in;
            mem_read_out <= mem_read_in;
            mem_write_out <= mem_write_in;
            mem_to_reg_out <= mem_to_reg_in;
            write_mode_out <= reg_write_in;
            read_write_out <= read_write_in;
            alu_op_out <= alu_op_in;
            io_op_out <= io_op_in;
            branch_en_out <= branch_en_in;
        end
    end
endmodule

// E/W Stage Register
module EW_Register (
    input wire clk,
    input wire reset,
    
    // Inputs from Execute stage
    input wire [4:0] opcode_in,
    input wire [2:0] reg_write_addr_in, // rd_E
    input wire [2:0] source_reg1_in, // rs1_E
    input wire [2:0] source_reg2_in, // rs2_E
    input wire [15:0] alu_result_0_in,
    input wire [15:0] alu_result_1_in,
    input wire [15:0] prev_alu_result_0,
    input wire [15:0] mem_data_in,
    input wire [15:0] flags_in,
    input wire [10:0] branch_addr_in,
    input wire [15:0] input_port_in,
    
    // Control signals
    input wire read_write_in,
    input wire [1:0] write_mode_in,
    input wire flag_reg_en_in,
    input wire mem_to_reg_in,
    input wire mem_write_in,
    input wire io_op_in,
    input wire flush_E,
    input wire stall_E,
    
    // Outputs to Writeback stage
    output reg [4:0] opcode_out,
    output reg [2:0] reg_write_addr_out, // rd_W
    output reg [2:0] source_reg1_out, // rs1_W
    output reg [2:0] source_reg2_out, // rs2_W
    output reg [15:0] reg_write_data_0_out,
    output reg [15:0] reg_write_data_1_out,
    output reg [15:0] flags_out,
    output reg [10:0] branch_addr_out,
    
    // Added missing signals (this is my debbuging ability at it's best)
    output reg [10:0] mem_addr_out,
    output reg [15:0] mem_write_data_out,
    output reg mem_write_out,
    
    // Control signals to Writeback
    output reg read_write_out,
    output reg [1:0] write_mode_out,
    output reg flag_reg_en_out,
    output reg io_op_out,
    output reg mem_to_reg_out
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            opcode_out <= 5'b0;
            reg_write_addr_out <= 3'b0;
            source_reg1_out <= 3'b0;
            source_reg2_out <= 3'b0;
            reg_write_data_0_out <= 16'b0;
            reg_write_data_1_out <= 16'b0;
            flags_out <= 16'b0;
            mem_addr_out <= 11'b0;
            mem_write_data_out <= 16'b0;
            mem_write_out <= 1'b0;
            branch_addr_out <= 11'b0;
            
            // Control signals
            read_write_out <= 1'b0;
            write_mode_out <= 2'b00;
            flag_reg_en_out <= 1'b0;
            io_op_out <= 1'b0;
            mem_to_reg_out <= 1'b0;
        end
        else if (flush_E) begin
            opcode_out <= 5'b0;
            reg_write_addr_out <= 3'b0;
            source_reg1_out <= 3'b0;
            source_reg2_out <= 3'b0;
            reg_write_data_0_out <= 16'b0;
            reg_write_data_1_out <= 16'b0;
            flags_out <= 16'b0;
            mem_addr_out <= 11'b0;
            mem_write_data_out <= 16'b0;
            mem_write_out <= 1'b0;
            branch_addr_out <= 11'b0;
            
            // Control signals
            read_write_out <= 1'b0;
            write_mode_out <= 2'b00;
            flag_reg_en_out <= 1'b0;
            io_op_out <= 1'b0;
            mem_to_reg_out <= 1'b0;
        end
        else if (!stall_E) begin
            opcode_out <= opcode_in;
            reg_write_addr_out <= io_op_in ? source_reg1_in : reg_write_addr_in;
            source_reg1_out <= source_reg1_in;
            source_reg2_out <= source_reg2_in;
            reg_write_data_0_out <= mem_to_reg_in ? mem_data_in : io_op_in ? input_port_in:  alu_result_0_in;
            reg_write_data_1_out <= alu_result_1_in;
            flags_out <= flags_in;
            branch_addr_out <= branch_addr_in;
            
            // Memory signals
            mem_addr_out <= alu_result_0_in[10:0]; // Assuming address comes from ALU
            mem_write_data_out <= alu_result_1_in; // Assuming data comes from ALU
            mem_write_out <= mem_write_in;
            
            // Control signals
            read_write_out <= read_write_in;
            write_mode_out <= write_mode_in;
            flag_reg_en_out <= flag_reg_en_in;
            io_op_out <= io_op_in;
            mem_to_reg_out <= mem_to_reg_in;
        end
    end
endmodule