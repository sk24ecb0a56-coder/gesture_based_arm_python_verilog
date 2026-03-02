`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 28.02.2026 16:31:25
// Design Name: 
// Module Name: tb_gesture_display
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


module tb_gesture_display;
    reg         clk;
    reg         resetn;
    reg         uart_rx_pin;
    wire        uart_tx_pin;
    wire [6:0]  seg;
    wire        dp;
    wire [7:0]  an;
    wire [15:0] led;
    wire        led16_r, led16_g, led16_b;
    wire        led17_r, led17_g, led17_b;

    gesture_display_top dut (
        .CLK100MHZ   (clk),
        .CPU_RESETN  (resetn),
        .UART_TXD_IN (uart_rx_pin),
        .UART_RXD_OUT(uart_tx_pin),
        .SEG         (seg),
        .DP          (dp),
        .AN          (an),
        .LED         (led),
        .LED16_R     (led16_r),
        .LED16_G     (led16_g),
        .LED16_B     (led16_b),
        .LED17_R     (led17_r),
        .LED17_G     (led17_g),
        .LED17_B     (led17_b)
    );

    // ── 100 MHz clock (10ns period) ──
    initial clk = 0;
    always #5 clk = ~clk;

    // ── UART bit period for 9600 baud ──
    localparam BAUD_PERIOD = 1_000_000_000 / 9600;  // ~104166 ns

    // ── Task: Send one byte via UART (8N1, LSB first) ──
    task uart_send_byte;
        input [7:0] data;
        integer i;
        begin
            $display("[%0t] UART TX: 0x%02X", $time, data);

            // Start bit (low)
            uart_rx_pin = 1'b0;
            #(BAUD_PERIOD);

            // 8 data bits (LSB first)
            for (i = 0; i < 8; i = i + 1) begin
                uart_rx_pin = data[i];
                #(BAUD_PERIOD);
            end

            // Stop bit (high)
            uart_rx_pin = 1'b1;
            #(BAUD_PERIOD);

            // Inter-byte gap
            #(BAUD_PERIOD * 2);
        end
    endtask

    // ── Main test sequence ──
    initial begin
        // Initialize
        uart_rx_pin = 1'b1;  // UART idle = high
        resetn = 1'b0;       // Assert reset

        #1000;
        resetn = 1'b1;       // Release reset
        #1000;

        $display("========================================");
        $display("  GESTURE DISPLAY TESTBENCH START");
        $display("========================================");

        // Test 1: Send FIST (0 fingers) — 0xA0
        $display("\n--- Test 1: FIST (0 fingers) ---");
        uart_send_byte(8'hA0);
        #(BAUD_PERIOD * 5);
        $display("  LED[4:0] = %b (expect 00000)", led[4:0]);
        $display("  LED[5]   = %b (expect 1 = hand detected)", led[5]);
        $display("  LED[15:8]= %b (raw byte debug)", led[15:8]);

        // Test 2: Send ONE (1 finger) — 0xA1
        $display("\n--- Test 2: ONE (1 finger) ---");
        uart_send_byte(8'hA1);
        #(BAUD_PERIOD * 5);
        $display("  LED[4:0] = %b (expect 00001)", led[4:0]);

        // Test 3: Send TWO — 0xA2
        $display("\n--- Test 3: TWO (2 fingers) ---");
        uart_send_byte(8'hA2);
        #(BAUD_PERIOD * 5);
        $display("  LED[4:0] = %b (expect 00011)", led[4:0]);

        // Test 4: Send THREE — 0xA3
        $display("\n--- Test 4: THREE (3 fingers) ---");
        uart_send_byte(8'hA3);
        #(BAUD_PERIOD * 5);
        $display("  LED[4:0] = %b (expect 00111)", led[4:0]);

        // Test 5: Send FOUR — 0xA4
        $display("\n--- Test 5: FOUR (4 fingers) ---");
        uart_send_byte(8'hA4);
        #(BAUD_PERIOD * 5);
        $display("  LED[4:0] = %b (expect 01111)", led[4:0]);

        // Test 6: Send FIVE — 0xA5
        $display("\n--- Test 6: FIVE (5 fingers) ---");
        uart_send_byte(8'hA5);
        #(BAUD_PERIOD * 5);
        $display("  LED[4:0] = %b (expect 11111)", led[4:0]);

        // Test 7: Send NO HAND — 0xAF
        $display("\n--- Test 7: NO HAND ---");
        uart_send_byte(8'hAF);
        #(BAUD_PERIOD * 5);
        $display("  LED[4:0] = %b (expect 00000)", led[4:0]);
        $display("  LED[5]   = %b (expect 0 = no hand)", led[5]);

        // Test 8: Invalid packet (wrong header) — should be ignored
        $display("\n--- Test 8: INVALID (0xB3) — should ignore ---");
        uart_send_byte(8'hB3);
        #(BAUD_PERIOD * 5);
        $display("  LED[5]   = %b (should still be 0 from prev)", led[5]);

        // Test 9: Back to valid — 0xA3
        $display("\n--- Test 9: Valid again (3 fingers) ---");
        uart_send_byte(8'hA3);
        #(BAUD_PERIOD * 5);
        $display("  LED[4:0] = %b (expect 00111)", led[4:0]);
        $display("  LED[5]   = %b (expect 1)", led[5]);

        $display("\n========================================");
        $display("  TESTBENCH COMPLETE");
        $display("========================================");

        #100000;
        $finish;
    end

    // ── Waveform dump (for GTKWave / Vivado) ──
    initial begin
        $dumpfile("gesture_display.vcd");
        $dumpvars(0, tb_gesture_display);
    end

endmodule
