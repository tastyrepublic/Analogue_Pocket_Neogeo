// NeoGeo logic definition
// MiSTer RTC to uPD4990
//
// Copyright (C) 2019 Sean Gonsalves
// Modified for Analogue Pocket (Autonomous RTC + Precision Timing)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

module uPD4990(
	input clk_74a,
	input nRESET,
	input CLK,			// 12MHz please
	input [64:0] rtc,   // Pocket APF Bridge Date/Time
	input rtc_valid,
	input CS, OE,		// Both always low
	input DATA_CLK,
	input DATA_IN,
	input STROBE,
	output TP,
	output DATA_OUT
);

	// --- Internal Registers for Autonomous RTC ---
	reg [7:0] r_sec, r_min, r_hour, r_day, r_month, r_year;
	reg [3:0] r_weekday;
	reg init_done;

	wire [47:0] TIME_DATA;
	wire [3:0] MONTH_HEX;
	reg [47:0] SHIFT_REG;
	reg [3:0] CMD_REG;
	reg OUT_HOLD;				// Prevents CLK from shifting the 48 lower bits of SHIFT_REG
	
	// Precision Timing Accumulator (Replaces DIV9 for 32.768kHz accuracy)
	reg [23:0] acc_32k;

	reg [2:0] TP_SEL;
	reg [5:0] TP_HSECONDS;	// Half seconds (incremented at 2Hz)
	reg TP_SEC_RUN;
	reg INTERVAL_FLAG;
	
	reg [14:0] DIV15;
	reg [5:0] DIV6;
	
	reg [1:0] DATA_CLK_G_SR;
	reg [1:0] STROBE_G_SR;
	
	
	// "rtc" format:
	// 64 63        55        47        39        31        23        15        7
	// x  01000000 00000WWW YYYYYYYY 000MMMMM 00DDDDDD 0PHHHHHH 0MMMMMMM 0SSSSSSS
	// Values are in BCD
	// P: 1=PM 0=AM
	// Weekday: 1 to 7
	// Month: 1 to 12

	// uPD4990 datasheet:
	// Data is represented in BCD notation. Only months are represented in hexadecimal notation.
	// Format:
	// CCCC YYYYYYYY MMMM WWWW DDDDDDDD HHHHHHHH MMMMMMMM SSSSSSSS
	
	// --- Helper: Sakamoto's Algorithm for Weekday ---
	function [3:0] calc_weekday;
		input [7:0] y, m, d;
		reg [10:0] total_days;
		integer yy, mm, dd;
		begin
			// Convert BCD to Integer
			yy = (y[7:4] * 10) + y[3:0];
			mm = (m[7:4] * 10) + m[3:0];
			dd = (d[7:4] * 10) + d[3:0];
			
			// Adjust for Sakamoto's algorithm
			if (mm < 3) begin
				mm = mm + 12;
				yy = yy - 1;
			end
			
			// Formula: (d + 2m + 3(m+1)/5 + y + y/4 - y/100 + y/400) % 7
			// Simplified for 2000-2099:
			total_days = dd + (13 * (mm + 1)) / 5 + yy + (yy / 4) + 6;
			calc_weekday = (total_days % 7);
		end
	endfunction

	// BCD to hex
	assign MONTH_HEX = r_month[7:4] * 10 + r_month[3:0]; 
	
	// Use internal registers for Time Data instead of static inputs
	assign TIME_DATA = {r_year, MONTH_HEX, r_weekday, r_day, r_hour, r_min, r_sec};
	
	wire [14:0] DIV15_INC = DIV15 + 1'b1;	// Look-ahead for edge detection
	wire [5:0] DIV6_INC = DIV6 + 1'b1;		// Look-ahead for edge detection
	
	wire INTERVAL_TRIG = (((TP_SEL == 3'd4) & (TP_HSECONDS >= 6'd1-1)) |
								((TP_SEL == 3'd5) & (TP_HSECONDS >= 6'd10-1)) |
								((TP_SEL == 3'd6) & (TP_HSECONDS >= 6'd30-1)) |
								((TP_SEL == 3'd7) & (TP_HSECONDS >= 6'd60-1)));
	
	wire DATA_CLK_G = CS & DATA_CLK;
	wire STROBE_G = CS & STROBE;
	
	always @(posedge CLK)
	begin
		if (!nRESET)
		begin
			OUT_HOLD <= 1;
			TP_SEL <= 0;
			CMD_REG <= 0;
			TP_HSECONDS <= 0;
			TP_SEC_RUN <= 1;
			INTERVAL_FLAG <= 1;
			acc_32k <= 0;
			DIV15 <= 15'd0;
			DIV6 <= 6'd0;
			DATA_CLK_G_SR <= 0;
			STROBE_G_SR <= 0;
			
			init_done <= 0;
			r_sec <= 0; r_min <= 0; r_hour <= 0;
			r_day <= 8'h01; r_month <= 8'h01; r_year <= 8'h00;
			r_weekday <= 0;
		end
		else
		begin
			// --- Initialization: Sync with Pocket Time ONCE on boot ---
			if (!init_done && rtc_valid) begin
				// CORRECTED BIT MAPPING for 64-bit Bus:
				// [31:0]  = Time (Sec, Min, Hour, Reserved)
				// [63:32] = Date (Day, Month, Year, Reserved)
				
				r_sec   <= rtc[7:0];
				r_min   <= rtc[15:8];
				r_hour  <= rtc[23:16];
				// rtc[31:24] is skipped (Reserved/00)
				
				r_day   <= rtc[39:32]; // Fixed: Was grabbing 00 padding before
				r_month <= rtc[47:40];
				r_year  <= rtc[55:48];
				
				init_done <= 1;
			end
			
			// Continuous Weekday Calculation
			if (init_done) begin
				r_weekday <= calc_weekday(r_year, r_month, r_day);
			end

			// --- Precision Timing Logic (32.768kHz) ---
			// 12MHz / 32768Hz accumulator
			if (acc_32k >= 24'd12000000 - 24'd32768)
			begin
				acc_32k <= acc_32k - 24'd12000000 + 24'd32768; // Reset with carry
				
				DIV15 <= DIV15_INC;
				
				if ({DIV15[8], DIV15_INC[8]} == 2'b01)
				begin
					// 64Hz from 32768Hz
					if (TP_SEC_RUN)
					begin
						DIV6 <= DIV6_INC;
						if ({DIV6[4], DIV6_INC[4]} == 2'b01)
						begin
							// 2Hz from 64Hz
							// INTERVAL_FLAG is toggled in the middle of the interval (50% duty cycle)
							// It can be forced back high with command b1100
							if (INTERVAL_TRIG)
							begin
								TP_HSECONDS <= 6'd0;
								INTERVAL_FLAG <= ~INTERVAL_FLAG;
							end
							else
								TP_HSECONDS <= TP_HSECONDS + 1'b1;

							// --- Autonomous Clock Increment (1.0 Hz) ---
							if (TP_HSECONDS[0] == 1'b1) begin
								r_sec[3:0] <= r_sec[3:0] + 1;
								if (r_sec[3:0] == 9) begin
									r_sec[3:0] <= 0;
									r_sec[7:4] <= r_sec[7:4] + 1;
									if (r_sec[7:4] == 5) begin
										r_sec <= 0;
										r_min[3:0] <= r_min[3:0] + 1;
										if (r_min[3:0] == 9) begin
											r_min[3:0] <= 0;
											r_min[7:4] <= r_min[7:4] + 1;
											if (r_min[7:4] == 5) begin
												r_min <= 0;
												r_hour[3:0] <= r_hour[3:0] + 1;
												if (r_hour[3:0] == 9) begin
													r_hour[3:0] <= 0;
													r_hour[7:4] <= r_hour[7:4] + 1;
												end
												if (r_hour == 8'h23 && r_min == 8'h59 && r_sec == 8'h59) begin
													 r_hour <= 0;
													 // Day rollover omitted for FPGA simplicity
												end
											end
										end
									end
								end
							end
						end
					end
				end
			end
			else
				acc_32k <= acc_32k + 24'd32768; // Accumulate
			
			
			DATA_CLK_G_SR <= {DATA_CLK_G_SR[0], DATA_CLK_G};
			STROBE_G_SR <= {STROBE_G_SR[0], STROBE_G};
			
			// Rising edge of DATA_CLK_G
			if (DATA_CLK_G_SR == 2'b01)
			begin
				if (!OUT_HOLD)
					SHIFT_REG <= {CMD_REG[0], SHIFT_REG[47:1]};
				
				// CMD_REG always shifts regardless of OUT_HOLD
				CMD_REG <= {DATA_IN, CMD_REG[3:1]};
			end
			
			// Rising edge of STROBE_G
			if (STROBE_G_SR == 2'b01)
			begin
				casez(CMD_REG)
					4'b0000: OUT_HOLD <= 1'b1;		// Hold
					4'b0001: OUT_HOLD <= 1'b0;		// Allow shift
					4'b0010: OUT_HOLD <= 1'b1;		// Setting time isn't supported
					4'b0011:								// Read time
					begin
						SHIFT_REG <= TIME_DATA;
						OUT_HOLD <= 1'b1;
					end
					4'b01zz, 4'b10zz: TP_SEL <= {CMD_REG[3], CMD_REG[1:0]};
					4'b1100: INTERVAL_FLAG <= 1'b1;	// Reset
					4'b1101: TP_SEC_RUN <= 1'b1;
					4'b111z: TP_SEC_RUN <= 1'b0;
				endcase
			end
		end
	end
	
	assign TP = (TP_SEL == 3'd0) ? DIV15[8] :
					(TP_SEL == 3'd1) ? DIV15[6] :
					(TP_SEL == 3'd2) ? DIV15[3] :
					(TP_SEL == 3'd3) ? DIV15[2] :
					INTERVAL_FLAG;
	
	assign DATA_OUT = OUT_HOLD ? rtc[0] : SHIFT_REG[0];
	
endmodule