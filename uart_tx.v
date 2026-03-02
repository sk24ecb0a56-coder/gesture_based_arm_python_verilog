`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 28.02.2026 16:27:25
// Design Name: 
// Module Name: uart_tx
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

module uart_tx #(
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 9600
)(
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] data_in,     // Byte to send
    input  wire       start,       // Pulse to begin transmission
    output reg        tx,          // UART TX pin
    output reg        busy,        // High while transmitting
    output reg        done         // Pulse high when byte sent
);

    localparam TICK_COUNT = CLK_FREQ / BAUD_RATE;

    localparam S_IDLE  = 3'd0;
    localparam S_START = 3'd1;
    localparam S_DATA  = 3'd2;
    localparam S_STOP  = 3'd3;

    reg [2:0]  state;
    reg [15:0] tick_counter;
    reg [2:0]  bit_index;
    reg [7:0]  shift_reg;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state        <= S_IDLE;
            tx           <= 1'b1;  // UART idle = high
            busy         <= 1'b0;
            done         <= 1'b0;
            tick_counter <= 16'd0;
            bit_index    <= 3'd0;
            shift_reg    <= 8'd0;
        end else begin
            done <= 1'b0;

            case (state)
                S_IDLE: begin
                    tx   <= 1'b1;
                    busy <= 1'b0;
                    if (start) begin
                        shift_reg    <= data_in;
                        busy         <= 1'b1;
                        state        <= S_START;
                        tick_counter <= 16'd0;
                    end
                end

                S_START: begin
                    tx <= 1'b0;  // Start bit = low
                    if (tick_counter < TICK_COUNT - 1) begin
                        tick_counter <= tick_counter + 1;
                    end else begin
                        tick_counter <= 16'd0;
                        bit_index    <= 3'd0;
                        state        <= S_DATA;
                    end
                end

                S_DATA: begin
                    tx <= shift_reg[0];  // LSB first
                    if (tick_counter < TICK_COUNT - 1) begin
                        tick_counter <= tick_counter + 1;
                    end else begin
                        tick_counter <= 16'd0;
                        shift_reg    <= {1'b0, shift_reg[7:1]};
                        if (bit_index == 3'd7) begin
                            state <= S_STOP;
                        end else begin
                            bit_index <= bit_index + 1;
                        end
                    end
                end

                S_STOP: begin
                    tx <= 1'b1;  // Stop bit = high
                    if (tick_counter < TICK_COUNT - 1) begin
                        tick_counter <= tick_counter + 1;
                    end else begin
                        done  <= 1'b1;
                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

