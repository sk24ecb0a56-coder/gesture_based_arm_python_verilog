
module seven_seg_controller (
    input  wire       clk,
    input  wire       rst,
    input  wire [3:0] finger_count,   // 0-5
    input  wire       hand_detected,
    output reg  [6:0] seg,            // Active-low segments {g,f,e,d,c,b,a}
    output reg        dp,             // Active-low decimal point
    output reg  [7:0] an              // Active-low digit select
);

    // ── Refresh counter ──
    // 100 MHz / 2^18 ≈ 381 Hz per digit, ~48 Hz total refresh (8 digits)
    reg [17:0] refresh_counter;
    wire [2:0] digit_select;

    always @(posedge clk or posedge rst) begin
        if (rst)
            refresh_counter <= 18'd0;
        else
            refresh_counter <= refresh_counter + 1;
    end

    assign digit_select = refresh_counter[17:15];

    // ── Digit data multiplexer ──
    reg [3:0] digit_data;
    reg       digit_blank;

    // Encoding for special characters:
    //  0-9: normal digits
    //  10 = 'G'
    //  11 = 'E'
    //  12 = 'S'
    //  13 = 't'
    //  14 = 'H'
    //  15 = '-' (dash)

    always @(*) begin
        digit_blank = 1'b0;
        case (digit_select)
            3'd0: digit_data = finger_count;                    // Rightmost: finger count
            3'd1: digit_data = hand_detected ? 4'd14 : 4'd15;  // H or -
            3'd2: begin digit_data = 4'd0; digit_blank = 1'b1; end  // blank
            3'd3: begin digit_data = 4'd0; digit_blank = 1'b1; end  // blank
            3'd4: digit_data = 4'd13;                           // t
            3'd5: digit_data = 4'd12;                           // S
            3'd6: digit_data = 4'd11;                           // E
            3'd7: digit_data = 4'd10;                           // G
            default: begin digit_data = 4'd0; digit_blank = 1'b1; end
        endcase
    end

    // ── Anode driver (active low) ──
    always @(*) begin
        an = 8'b11111111;  // All off
        if (!digit_blank)
            an[digit_select] = 1'b0;  // Enable selected digit
    end

    // ── 7-segment decoder ──
    //   Segment mapping:    a
    //                      ───
    //                   f │   │ b
    //                      ─g─
    //                   e │   │ c
    //                      ───
    //                       d
    //
    //   seg = {g, f, e, d, c, b, a}  (active LOW)

    always @(*) begin
        dp = 1'b1;  // Decimal point off (active low)
        if (digit_blank) begin
            seg = 7'b1111111;  // All segments off
        end else begin
            case (digit_data)
                //              gfedcba
                4'd0:  seg = 7'b1000000;  // 0
                4'd1:  seg = 7'b1111001;  // 1
                4'd2:  seg = 7'b0100100;  // 2
                4'd3:  seg = 7'b0110000;  // 3
                4'd4:  seg = 7'b0011001;  // 4
                4'd5:  seg = 7'b0010010;  // 5
                4'd6:  seg = 7'b0000010;  // 6
                4'd7:  seg = 7'b1111000;  // 7
                4'd8:  seg = 7'b0000000;  // 8
                4'd9:  seg = 7'b0010000;  // 9
                4'd10: seg = 7'b0000010;  // G (same as 6, looks like G)
                4'd11: seg = 7'b0000110;  // E
                4'd12: seg = 7'b0010010;  // S (same as 5)
                4'd13: seg = 7'b0000111;  // t
                4'd14: seg = 7'b0001001;  // H
                4'd15: seg = 7'b0111111;  // - (dash)
                default: seg = 7'b1111111;
            endcase
        end
    end

endmodule
