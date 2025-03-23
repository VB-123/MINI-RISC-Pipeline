`include "parameters.v"
module control_unit (
    input wire [4:0] opcode,      // from instruction register
    
    // Control signals
    output reg alu_src,           // Select between register or immediate for ALU
    output reg read_write,        // Register file read/write select (00: read, 01: write low byte, 11: write high byte, 10: unused)
    output reg mem_write,         // Memory write enable
    output reg mem_to_reg,        // Select between ALU result or memory data for register write
    output reg branch,            // Branch instruction
    output reg inc_pc,            // Increment PC
    output reg jump,              // Jump instruction (added for hazard unit)
    output reg mem_read,          // Memory read (added for hazard unit)
    output reg alu_op,            // ALU operation control (added as it was used internally)
    output reg [1:0] write_mode   // Write mode for register (added as it was used internally)
);


  always @(*) begin
    // Default control signals
    alu_src = 1'b0;      // Use register value by default
    read_write = 1'b0;  // Register read by default
    mem_write = 1'b0;    // No memory write by default
    mem_to_reg = 1'b0;   // Use ALU result by default
    branch = 1'b0;       // Not a branch instruction by default
    alu_op = 1'b0;       // ALU operation (simplified)
    inc_pc = 1'b1;       // Default to incrementing PC unless explicitly disabled
    jump = 1'b0;         // Not a jump instruction by default
    mem_read = 1'b0;     // No memory read by default
    write_mode = 2'b00;  // Default write mode
    
    case (opcode)
      // ALU operations
      `ADD,`SUB, `DIV, `NOT, `AND, `OR, `XOR, `INC, `CMP, `RR, `RL, `SETB, `CLRB, `CPLB: begin
        read_write = 1'b1;  // Write low byte
        alu_src = 1'b0;
        alu_op = 1'b1;
        write_mode = 2'b01;
      end

      `MUL: begin
        read_write = 1'b1;
        alu_src = 1'b0;
        alu_op = 1'b1;
        write_mode = 2'b11;
      end
      
      // Flag operations
      `SETF, `CLRF, `CPLF: begin
        alu_op = 1'b1;
      end
      
      // Memory operations
      `LOAD: begin
        read_write = 1'b1;
        mem_to_reg = 1'b1;
        mem_read = 1'b1;   // Set memory read signal
        write_mode = 2'b11; 
        alu_op = 1'b0;
      end
      
      `STORE: begin
        read_write = 1'b0;
        mem_write = 1'b1;
        alu_op = 1'b0;
      end
      
      // Immediate load operations
      `LBL: begin
        read_write = 1'b1;       // Enable register write
        alu_src = 1'b1;
        write_mode = 2'b01;
        alu_op = 1'b0;
      end
      
      `LBH: begin
        read_write = 1'b1;
        alu_src = 1'b1;
        write_mode = 2'b01;
        alu_op = 1'b0;
      end
      
      // Register transfer operations
      `MOV: begin
        read_write = 1'b1;
        alu_src = 1'b0;
        write_mode = 2'b01;
        alu_op = 1'b0;
      end
      
      // Branch/Jump operations
      `JF: begin
        branch = 1'b1;
        jump = 1'b1;
      end
      
      `LOADBR: begin
        read_write = 1'b1;
        jump = 1'b1;        // Set jump for LOADBR
        write_mode = 2'b11;
        alu_op = 1'b0;
      end
      
      // I/O operations
      `MOVOUT, `MOVIN, `MOVB: begin
        read_write = 1'b1;
      end
      
      `HALT: begin
        inc_pc = 1'b0;  // Do not increment PC
      end
      
      default: begin
        // No operation
      end
    endcase
  end
endmodule