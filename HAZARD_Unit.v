`include "parameters.v"
module HAZARD_Unit(
  // Inputs
  input wire clk,                         // Clock signal
  input wire alu_en_in,                      // From Control Unit (not needed rn)
  input wire [4:0] opcode_D,              // From Decode stage
  input wire [4:0] opcode_E,              // From Execute stage
  input wire [4:0] opcode_W,
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
  input wire branch_en,                      // From Control Unit
  input wire jump_hzd,                    // Connect to control_unit
  input wire reg_write_D, 
  input wire reg_write_E,                 // From Control Unit via DE Register
  input wire reg_write_W,                 // From Control Unit via EW Register
  
  // Outputs
  output reg stall_F,                     // To FD Register
  output reg stall_D,                     // To DE Register
  output reg stall_E,                     // To EW Register
  output reg flush_F,                     // To FD Register
  output reg flush_D,                     // To DE Register
  output reg flush_E,                     // To EW Register
  output reg [1:0] forward_A,             // To forwarding logic
  output reg [1:0] forward_B,              // To forwarding logic
  output reg forward_decode_A,  // Forward to Decode stage for source_reg1
  output reg forward_decode_B,  // Forward to Decode stage for source_reg2
  output reg alu_en_out
);
  // New registers to track pipeline state
  /* reg [2:0] preserved_source_reg1;
  reg [2:0] preserved_source_reg2;
  reg [2:0] preserved_rd;
  reg preserved_reg_valid;
  reg preserve_fwd_B; */

  // Forwarding logic
  always @(*) begin
    // Default: no forwarding
    forward_A = 2'b00;
    forward_B = 2'b00;
    forward_decode_A = 1'b0;
    forward_decode_B = 1'b0;

    // Preserve source registers when a hazard is detected
    /* if (raw_hazard || control_hazard) begin
      preserved_source_reg1 <= source_reg1_D;
      preserved_source_reg2 <= source_reg2_D;
      preserved_rd <= rd_E;
      preserved_reg_valid <= 1'b1;
    end else begin
      preserved_reg_valid <= 1'b0;
    end */

    // Forwarding logic
    if (reg_write_W) begin
      if (rd_W == source_reg1_E) begin
        forward_A = 2'b10;
      end
      
      if (rd_W == source_reg2_E) begin
        forward_B = 2'b10;
      end

      if (rd_W == source_reg1_D) begin
        forward_decode_A = 1'b1; // Forward from writeback to decode
      end
      
      if (rd_W == source_reg2_D) begin
        forward_decode_B = 1'b1; // Forward from writeback to decode
      end

      if (reg_write_W && (opcode_D == `LOAD) && (rd_W == source_reg1_D)) begin
        forward_decode_A = 1'b1;  // Forward from WB to Decode
      end
    end
    //$display("Hazard DEBUG: reg_write_W=%b, rd_W=%d, source_reg1_E=%d, source_reg2_E=%d, forward_A=%b, forward_B = %b, rd_E=%d, source_reg1_D=%d, source_reg2_D=%d Stall_D = %b, Flush_D = %b, Stall_E = %b, Flush_E = %b", 
                            //reg_write_W, rd_W, source_reg1_E, source_reg2_E, forward_A, forward_B, rd_E, source_reg1_D, source_reg2_D, stall_D, flush_D, stall_E, flush_E);
  end

  // RAW hazard detection (unchanged)
  wire raw_hazard;
  wire lbh_lbl_hazard;              
  assign lbh_lbl_hazard = (
    ((opcode_E == `LBH || opcode_E == `LBL)) && 
    (reg_write_E) && 
    ((rd_E == source_reg1_D) || (rd_E == source_reg2_D))
  );
  
  assign raw_hazard = reg_write_E && reg_write_W &&
                  ((rd_E == source_reg1_D) || (rd_E == source_reg2_D));

  // Control hazard detection                       
  wire control_hazard;
  assign control_hazard = branch_en && ((opcode_W == `SETF) || (opcode_W == `CPLF));
  
  // Pipeline control logic
  always @(*) begin
    // Default values
    stall_F = 1'b0;
    stall_D = 1'b0;
    stall_E = 1'b0;
    flush_F = 1'b0;
    flush_D = 1'b0;
    flush_E = 1'b0;
    alu_en_out = 1'b0;  // Always disable ALU during branch

    if (branch_en) begin
        flush_F = 1'b1;  // Always flush fetch stage
        flush_D = 1'b1;  // Always flush decode stage
        stall_F = 1'b1;  // Stall fetch
        stall_D = 1'b1;  // Stall decode
        stall_E = 1'b1;  // Stall execute
        alu_en_out = 1'b0;  // Explicitly disable ALU
    end
  end
endmodule