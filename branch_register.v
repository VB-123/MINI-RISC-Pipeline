module branch_register(
    input wire reset,
    input wire [10:0] branch_addr_in,
    input wire branch_en,
    output reg [10:0] branch_addr_out
);
    reg [10:0] stored_addr;
    always @(*) begin
        if (reset) begin
            branch_addr_out = 11'b0;
            stored_addr = 11'b0;
        end 
        else if (branch_en) begin
            branch_addr_out = branch_addr_in;
            stored_addr = branch_addr_in;
        end 
        else begin
            branch_addr_out = stored_addr;
        end
    end
endmodule
