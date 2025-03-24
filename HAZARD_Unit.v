module HAZARD_Unit(
  // Inputs
  input wire alu_en,                      // From Control Unit
  input wire [4:0] opcode_D,              // From Decode stage
  input wire [4:0] opcode_E,              // From Execute stage
  input wire [2:0] rd_D,                  // From Decode stage
  input wire [2:0] source_reg1_D,         // From Decode stage
  input wire [2:0] source_reg2_D,         // From Decode stage
  input wire [2:0] rd_E,                  // From Execute stage
  input wire [2:0] source_reg1_E,         // From Execute stage
  input wire [2:0] source_reg2_E,         // From Execute stage
  input wire [2:0] rd_W,                  // From Writeback stage
  input wire [2:0] source_reg1_W,         // From Writeback stage
  input wire [2:0] source_reg2_W,         // From Writeback stage
  input wire mem_read_E,                  // From Control Unit via DE Register
  input wire branch,                      // From Control Unit
  input wire jump_hzd,                    // Connect to control_unit
  input wire reg_write_E,                 // From Control Unit via DE Register
  input wire reg_write_W,                 // From Control Unit via EW Register
  
  // Outputs
  output reg stall_F,                     // To FD Register
  output reg stall_D,                     // To DE Register
  output reg flush_F,                     // To FD Register
  output reg flush_D,                     // To DE Register
  output reg [1:0] forward_A,             // To forwarding logic
  output reg [1:0] forward_B              // To forwarding logic
);
  
  // Forwarding logic
  always @(*) begin
    // Default: no forwarding
    forward_A = 2'b00;
    forward_B = 2'b00;
    $display("\nHazard unit DEBUG: reg_write_W=%b, rd_W=%d, source_reg1_E=%d, source_reg2_E=%d \n", 
            reg_write_W, rd_W, source_reg1_E, source_reg2_E);
    
    // Forwarding for first operand (A)
    if (reg_write_W) begin
      // Special handling for byte load instructions
      if ((opcode_E == `LBH || opcode_E == `LBL) && (rd_W == source_reg1_E)) begin
        forward_A = 2'b10;  // Forward from Writeback stage
      end
      // Normal forwarding
      else if (rd_W == source_reg1_E) begin
        forward_A = 2'b10;  // Forward from Writeback stage
      end
    end
    
    // Forwarding for second operand (B)
    if (reg_write_W) begin
      // for byte load instructions
      if ((opcode_E == `LBH || opcode_E == `LBL) && (rd_W == source_reg2_E)) begin
        forward_B = 2'b10;  // Forward from Writeback stage
      end
      // Normal forwarding
      else if (rd_W == source_reg2_E) begin
        forward_B = 2'b10;  // Forward from Writeback stage
      end
    end
  end
  
  // RAW hazard detection
  wire raw_hazard;
  wire lbh_lbl_hazard;              
  assign lbh_lbl_hazard = (
    ((opcode_E == `LBH || opcode_E == `LBL)) && 
    (reg_write_E) && 
    ((rd_E == source_reg1_D) || (rd_E == source_reg2_D))
  );
  
  assign raw_hazard = reg_write_E && 
                  ((rd_E == source_reg1_D) || (rd_E == source_reg2_D)) &&
                  !lbh_lbl_hazard;
  // Control hazard detection                       
  wire control_hazard;
  assign control_hazard = branch || jump_hzd;
  
  // Pipeline control logic
  always @(*) begin
    // Default values
    stall_F = 1'b0;
    stall_D = 1'b0;
    flush_F = 1'b0;
    flush_D = 1'b0;

    if (raw_hazard) begin
      stall_F = 1'b1;  // Stall fetch stage
      stall_D = 1'b1;  // Stall decode stage
      flush_D = 1'b1;  // Insert bubble in execute stage
    end

    // Control hazard: flush the pipeline
    if (control_hazard) begin
      flush_F = 1'b1;  // Flush fetch stage
      flush_D = 1'b1;  // Flush decode stage
    end
  end
endmodule