`timescale 1ps/1ps
`include "parameters.v"

module processor_top (
    input  wire        CLOCK_50,
    input  wire [9:0]  SW,
    input  wire [3:0]  KEY,
    output wire [9:0]  LEDR,
    output wire [6:0]  HEX0,
    output wire [6:0]  HEX1,
    output wire [6:0]  HEX2,
    output wire [6:0]  HEX3,
    output wire [6:0]  HEX4,
    output wire [6:0]  HEX5
);

    reg  [15:0] proc_input_reg;
    wire [15:0] proc_output_reg;

    // 1. Synchronize and edge-detect active-low push buttons (KEY[3:0])
    reg [2:0] key1_sync, key2_sync;
    always @(posedge CLOCK_50) begin
        key1_sync <= {key1_sync[1:0], KEY[1]}; // Load button
        key2_sync <= {key2_sync[1:0], KEY[2]}; // Single-step clock button
    end

    wire load_pulse = (key1_sync[2:1] == 2'b10); // Falling edge of KEY
    wire step_pulse = (key2_sync[2:1] == 2'b10); // Falling edge of KEY

    // 2. Active-high reset from KEY (KEY is active-low on DE1-SoC)
    wire proc_rst = ~KEY[0];

    // 3. Choose processor clock:
    //    SW == 0 -> Single-step manually with KEY
    //    SW == 1 -> Run continuously at 50 MHz (CLOCK_50)
    wire proc_clk = SW[9] ? CLOCK_50 : step_pulse;

    // 4. Two-step 16-bit input_reg loader using SW[7:0] and KEY
    always @(posedge CLOCK_50) begin
        if (proc_rst) begin
            proc_input_reg <= 16'h0000;
        end else if (load_pulse) begin
            if (SW[8] == 1'b0)
                proc_input_reg[7:0]  <= SW[7:0]; // Load lower byte [7:0]
            else
                proc_input_reg[15:8] <= SW[7:0]; // Load upper byte [15:8]
        end
    end

    // 5. Instantiate your pipelined_processor unchanged
    pipelined_processor u_cpu (
        .clk        (proc_clk),
        .rst        (proc_rst),
        .input_reg  (proc_input_reg),
        .output_reg (proc_output_reg)
    );

    // 6. Display output_reg (SW=0) or input_reg (SW=1) on HEX3..HEX0
    wire [15:0] hex_data = SW[9] ? proc_input_reg : proc_output_reg;

    decoder_hex u_h0 (.in(hex_data[3:0]),   .seg(HEX0));
    decoder_hex u_h1 (.in(hex_data[7:4]),   .seg(HEX1));
    decoder_hex u_h2 (.in(hex_data[11:8]),  .seg(HEX2));
    decoder_hex u_h3 (.in(hex_data[15:12]), .seg(HEX3));

    // Show "Ot" (Output) when SW=0 or "In" (Input) when SW=1 on HEX5..HEX4
    assign HEX5 = SW[9] ? 7'b1111001 : 7'b1000000; // 'I' or 'O'
    assign HEX4 = SW[9] ? 7'b0101011 : 7'b0000111; // 'n' or 't'

    assign LEDR = SW;

endmodule


module decoder_hex (
    input  wire [3:0] in,
    output reg  [6:0] seg
);
    always @(*) begin
        case (in)
            4'b0000: seg = 7'b1000000; // 0
            4'b0001: seg = 7'b1111001; // 1
            4'b0010: seg = 7'b0100100; // 2
            4'b0011: seg = 7'b0110000; // 3
            4'b0100: seg = 7'b0011001; // 4
            4'b0101: seg = 7'b0010010; // 5
            4'b0110: seg = 7'b0000010; // 6
            4'b0111: seg = 7'b1111000; // 7
            4'b1000: seg = 7'b0000000; // 8
            4'b1001: seg = 7'b0010000; // 9
            4'b1010: seg = 7'b0001000; // A
            4'b1011: seg = 7'b0000011; // b
            4'b1100: seg = 7'b1000110; // C
            4'b1101: seg = 7'b0100001; // d
            4'b1110: seg = 7'b0000110; // E
            4'b1111: seg = 7'b0001110; // F
            default: seg = 7'b1111111;
        endcase
    end
endmodule
