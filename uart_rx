`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 28.02.2026 16:25:50
// Design Name: 
// Module Name: uart_rx
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

// ============================================================
// UART RECEIVER — 8N1
// ============================================================
// Oversamples at 16× baud rate for robust bit sampling
// ============================================================
module uart_rx #(
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 9600
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       rx,
    output reg  [7:0] data_out,
    output reg        valid
);

    // Baud rate tick count (no oversampling — sample at mid-bit)
    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;  // 10417 for 9600 @ 100MHz

    localparam S_IDLE  = 3'd0;
    localparam S_START = 3'd1;
    localparam S_DATA  = 3'd2;
    localparam S_STOP  = 3'd3;

    reg [2:0]  state;
    reg [15:0] clk_count;
    reg [2:0]  bit_index;
    reg [7:0]  shift_reg;

    // Synchronize RX input (2-stage FF) — WITH initial values
    reg rx_sync1 = 1'b1;
    reg rx_sync2 = 1'b1;
    always @(posedge clk) begin
        if (rst) begin
            rx_sync1 <= 1'b1;
            rx_sync2 <= 1'b1;
        end else begin
            rx_sync1 <= rx;
            rx_sync2 <= rx_sync1;
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state     <= S_IDLE;
            clk_count <= 16'd0;
            bit_index <= 3'd0;
            shift_reg <= 8'd0;
            data_out  <= 8'd0;
            valid     <= 1'b0;
        end else begin
            valid <= 1'b0;

            case (state)
                // Wait for start bit (falling edge: line goes LOW)
                S_IDLE: begin
                    clk_count <= 16'd0;
                    bit_index <= 3'd0;
                    if (rx_sync2 == 1'b0) begin
                        state <= S_START;
                    end
                end

                // Wait until middle of start bit to confirm it
                S_START: begin
                    if (clk_count < (CLKS_PER_BIT / 2) - 1) begin
                        clk_count <= clk_count + 1;
                    end else begin
                        clk_count <= 16'd0;
                        if (rx_sync2 == 1'b0) begin
                            // Start bit confirmed at midpoint
                            state <= S_DATA;
                        end else begin
                            // False start — go back
                            state <= S_IDLE;
                        end
                    end
                end

                // Sample each data bit at its midpoint
                S_DATA: begin
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1;
                    end else begin
                        clk_count <= 16'd0;
                        // Sample bit — UART sends LSB first
                        shift_reg <= {rx_sync2, shift_reg[7:1]};

                        if (bit_index == 3'd7) begin
                            state <= S_STOP;
                        end else begin
                            bit_index <= bit_index + 1;
                        end
                    end
                end

                // Wait for stop bit
                S_STOP: begin
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1;
                    end else begin
                        clk_count <= 16'd0;
                        if (rx_sync2 == 1'b1) begin
                            // Valid stop bit — output the byte
                            data_out <= shift_reg;
                            valid    <= 1'b1;
                        end
                        // Always return to idle
                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
