// ============================================================
// GESTURE DISPLAY TOP MODULE
// Nexys 4 DDR (XC7A100T-1CSG324C)
// ============================================================
// Receives finger count via UART from PC
// Displays on: 7-segment displays, LEDs
//
// UART Protocol:
//   Byte = 0xA<n>  where n = 0-5 (finger count)
//   0xAF = no hand detected
//   Upper nibble 0xA = sync marker
//   Lower nibble  = data
//
// Pin assignments match Nexys 4 DDR master XDC
// ============================================================
`timescale 1ns / 1ps
`timescale 1ns / 1ps
module gesture_display_top (
    input  wire        CLK100MHZ,
    input  wire        CPU_RESETN,
    input  wire        UART_TXD_IN,
    output wire        UART_RXD_OUT,
    output wire [6:0]  SEG,
    output wire        DP,
    output wire [7:0]  AN,
    output wire [15:0] LED,
    output wire        LED16_R,
    output wire        LED16_G,
    output wire        LED16_B,
    output wire        LED17_R,
    output wire        LED17_G,
    output wire        LED17_B
);

    // ──────────────────────────────────────────────────
    // INTERNAL SIGNALS
    // ──────────────────────────────────────────────────
    wire        rst = ~CPU_RESETN;
    wire [7:0]  rx_data;
    wire        rx_valid;

    reg  [3:0]  finger_count;
    reg         hand_detected;
    reg  [3:0]  gesture_latched;

    reg  [7:0]  tx_data;
    reg         tx_start;
    wire        tx_busy;
    wire        tx_done;

    // Debug: latch raw received byte for LED display
    reg  [7:0]  rx_data_latched;

    // ──────────────────────────────────────────────────
    // LED[7] blinks at ~0.75 Hz
    // ──────────────────────────────────────────────────
    reg [26:0] heartbeat_counter;
    always @(posedge CLK100MHZ or posedge rst) begin
        if (rst)
            heartbeat_counter <= 27'd0;
        else
            heartbeat_counter <= heartbeat_counter + 1;
    end

    // ──────────────────────────────────────────────────
    // UART RECEIVER (9600 baud, 8N1)
    // ──────────────────────────────────────────────────
    uart_rx #(
        .CLK_FREQ(100_000_000),
        .BAUD_RATE(9600)
    ) u_uart_rx (
        .clk     (CLK100MHZ),
        .rst     (rst),
        .rx      (UART_TXD_IN),
        .data_out(rx_data),
        .valid   (rx_valid)
    );

    // ──────────────────────────────────────────────────
    // UART TRANSMITTER (echo back for ack)
    // ──────────────────────────────────────────────────
    uart_tx #(
        .CLK_FREQ(100_000_000),
        .BAUD_RATE(9600)
    ) u_uart_tx (
        .clk     (CLK100MHZ),
        .rst     (rst),
        .data_in (tx_data),
        .start   (tx_start),
        .tx      (UART_RXD_OUT),
        .busy    (tx_busy),
        .done    (tx_done)
    );

    // ──────────────────────────────────────────────────
    // PROTOCOL DECODER
    // ──────────────────────────────────────────────────
    always @(posedge CLK100MHZ or posedge rst) begin
        if (rst) begin
            finger_count    <= 4'd0;
            hand_detected   <= 1'b0;
            gesture_latched <= 4'd0;
            tx_start        <= 1'b0;
            tx_data         <= 8'd0;
            rx_data_latched <= 8'd0;
        end else begin
            tx_start <= 1'b0;

            if (rx_valid) begin
                // Latch raw byte for debug LEDs
                rx_data_latched <= rx_data;

                // Check sync marker (upper nibble = 0xA)
                if (rx_data[7:4] == 4'hA) begin
                    if (rx_data[3:0] == 4'hF) begin
                        hand_detected <= 1'b0;
                        finger_count  <= 4'd0;
                    end else if (rx_data[3:0] <= 4'd5) begin
                        hand_detected   <= 1'b1;
                        finger_count    <= rx_data[3:0];
                        gesture_latched <= rx_data[3:0];
                    end

                    // Echo back as acknowledgment
                    if (!tx_busy) begin
                        tx_data  <= rx_data;
                        tx_start <= 1'b1;
                    end
                end
            end
        end
    end

    // ──────────────────────────────────────────────────
    // 7-SEGMENT DISPLAY
    // ──────────────────────────────────────────────────
    seven_seg_controller u_seven_seg (
        .clk           (CLK100MHZ),
        .rst           (rst),
        .finger_count  (gesture_latched),
        .hand_detected (hand_detected),
        .seg           (SEG),
        .dp            (DP),
        .an            (AN)
    );

    // ──────────────────────────────────────────────────
    // LED DISPLAY
    // ──────────────────────────────────────────────────
    reg [4:0] finger_leds;
    always @(*) begin
        case (gesture_latched)
            4'd0: finger_leds = 5'b00000;
            4'd1: finger_leds = 5'b00001;
            4'd2: finger_leds = 5'b00011;
            4'd3: finger_leds = 5'b00111;
            4'd4: finger_leds = 5'b01111;
            4'd5: finger_leds = 5'b11111;
            default: finger_leds = 5'b00000;
        endcase
    end

    assign LED[4:0]  = hand_detected ? finger_leds : 5'b00000;
    assign LED[5]    = hand_detected;
    assign LED[6]    = rx_valid;                    // Blinks when ANY byte received
    assign LED[7]    = heartbeat_counter[26];       // Heartbeat — proves FPGA is alive
    assign LED[15:8] = rx_data_latched;             // Latched raw UART byte (persists)

    // ──────────────────────────────────────────────────
    // RGB LEDs
    // ──────────────────────────────────────────────────
    assign LED16_R = ~hand_detected;
    assign LED16_G = hand_detected;
    assign LED16_B = 1'b0;

    reg [23:0] rx_flash_counter;
    always @(posedge CLK100MHZ or posedge rst) begin
        if (rst)
            rx_flash_counter <= 24'd0;
        else if (rx_valid)
            rx_flash_counter <= 24'd5_000_000;
        else if (rx_flash_counter > 0)
            rx_flash_counter <= rx_flash_counter - 1;
    end

    assign LED17_R = 1'b0;
    assign LED17_G = 1'b0;
    assign LED17_B = (rx_flash_counter > 0);

endmodule
