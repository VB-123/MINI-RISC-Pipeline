`include "parameters.v"
module control_unit (
    input wire [4:0] opcode,      // from instruction register
    
    // Control signals
    output reg alu_src,           // Select between register or immediate for ALU
    output reg reg_write,         // Register file write enable
    output reg mem_read,          // Memory read enable
    output reg mem_write,         // Memory write enable
    output reg mem_to_reg,        // Select between ALU result or memory data for register write
    output reg branch,            // Branch instruction
    output reg jump,              // Jump instruction
    output reg [1:0] write_mode,  // 00: No write, 01: low byte, 10: high byte, 11: full word
    output reg alu_op             // ALU operation control (simplified for example)
);


  always @(*) begin
    // Default control signals
    alu_src = 1'b0;      // Use register value by default
    reg_write = 1'b0;    // No register write by default
    mem_read = 1'b0;     // No memory read by default
    mem_write = 1'b0;    // No memory write by default
    mem_to_reg = 1'b0;   // Use ALU result by default
    branch = 1'b0;       // Not a branch instruction by default
    jump = 1'b0;         // Not a jump instruction by default
    write_mode = 2'b00;  // No write by default
    alu_op = 1'b0;       // ALU operation (simplified)
    
    case (opcode)
      // ALU operations
      `ADD, `MUL, `SUB, `DIV, `NOT, `AND, `OR, `XOR, `INC, `CMP, `RR, `RL, `SETB, `CLRB, `CPLB: begin
        reg_write = 1'b1;
        write_mode = 2'b11;  // Full word
        alu_op = 1'b1;
      end
      
      // Flag operations
      `SETF, `CLRF, `CPLF: begin
        alu_op = 1'b1;  // Use ALU for flag operations
      end
      
      // Memory operations
      `LOAD: begin
        reg_write = 1'b1;
        mem_read = 1'b1;
        mem_to_reg = 1'b1;
        write_mode = 2'b11;  // Full word
      end
      
      `STORE: begin
        mem_write = 1'b1;
      end
      
      // Immediate load operations
      `LBL: begin
        reg_write = 1'b1;
        alu_src = 1'b1;    // Use immediate
        write_mode = 2'b01;  // Low byte
      end
      
      `LBH: begin
        reg_write = 1'b1;
        alu_src = 1'b1;    // Use immediate
        write_mode = 2'b10;  // High byte
      end
      
      // Register transfer operations
      `MOV: begin
        reg_write = 1'b1;
        write_mode = 2'b11;  // Full word
      end
      
      // Branch/Jump operations
      `JF: begin
        branch = 1'b1;
      end
      
      `LOADBR: begin
        jump = 1'b1;
      end
      
      // I/O operations
      `MOVOUT, `MOVIN, `MOVB: begin
        reg_write = 1'b1;
        write_mode = 2'b11;  // Full word
      end
      
      `HALT: begin
        // No operation
      end
      
      default: begin
        // No operation
      end
    endcase
  end
endmodule

