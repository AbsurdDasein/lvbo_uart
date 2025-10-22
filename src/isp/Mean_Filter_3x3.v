/*
 * 3x3 Mean Filter Module with Line Buffer
 * 
 * Description:
 *   - Uses 9 SRAM modules as line buffers (3 rows × RGB channels)
 *   - Implements 3×3 mean filter for noise reduction
 *   - Handles frame sync and timing alignment
 *   
 * Parameters:
 *   - IMG_WIDTH: Image width (default 1280)
 *   - IMG_HEIGHT: Image height (default 720)
 *   
 * Latency: 
 *   - 2 rows + 3 pixels delay for filter window establishment
 *   - Frame sync signals delayed accordingly
 */

module Mean_Filter_3x3 
#(
    parameter IMG_WIDTH  = 1280,
    parameter IMG_HEIGHT = 720
)
(
    // Clock and Reset
    input  wire         clk,
    input  wire         rst_n,
    
    // Input Video Stream
    input  wire         i_vsync,
    input  wire         i_hsync,
    input  wire         i_de,
    input  wire [7:0]   i_r,
    input  wire [7:0]   i_g,
    input  wire [7:0]   i_b,
    
    // Output Video Stream (filtered)
    output reg          o_vsync,
    output reg          o_hsync,
    output reg          o_de,
    output reg  [7:0]   o_r,
    output reg  [7:0]   o_g,
    output reg  [7:0]   o_b,
    
    // SRAM Row0 Interface (Red channel)
    output wire         sram_r0_r_we,
    output wire         sram_r0_r_re,
    output wire         sram_r0_r_waddren,
    output wire         sram_r0_r_raddren,
    output wire [10:0]  sram_r0_r_waddr,
    output wire [10:0]  sram_r0_r_raddr,
    output wire [7:0]   sram_r0_r_wdata,
    input  wire [7:0]   sram_r0_r_rdata,
    
    // SRAM Row0 Interface (Green channel)
    output wire         sram_r0_g_we,
    output wire         sram_r0_g_re,
    output wire         sram_r0_g_waddren,
    output wire         sram_r0_g_raddren,
    output wire [10:0]  sram_r0_g_waddr,
    output wire [10:0]  sram_r0_g_raddr,
    output wire [7:0]   sram_r0_g_wdata,
    input  wire [7:0]   sram_r0_g_rdata,
    
    // SRAM Row0 Interface (Blue channel)
    output wire         sram_r0_b_we,
    output wire         sram_r0_b_re,
    output wire         sram_r0_b_waddren,
    output wire         sram_r0_b_raddren,
    output wire [10:0]  sram_r0_b_waddr,
    output wire [10:0]  sram_r0_b_raddr,
    output wire [7:0]   sram_r0_b_wdata,
    input  wire [7:0]   sram_r0_b_rdata,
    
    // SRAM Row1 Interface (Red channel)
    output wire         sram_r1_r_we,
    output wire         sram_r1_r_re,
    output wire         sram_r1_r_waddren,
    output wire         sram_r1_r_raddren,
    output wire [10:0]  sram_r1_r_waddr,
    output wire [10:0]  sram_r1_r_raddr,
    output wire [7:0]   sram_r1_r_wdata,
    input  wire [7:0]   sram_r1_r_rdata,
    
    // SRAM Row1 Interface (Green channel)
    output wire         sram_r1_g_we,
    output wire         sram_r1_g_re,
    output wire         sram_r1_g_waddren,
    output wire         sram_r1_g_raddren,
    output wire [10:0]  sram_r1_g_waddr,
    output wire [10:0]  sram_r1_g_raddr,
    output wire [7:0]   sram_r1_g_wdata,
    input  wire [7:0]   sram_r1_g_rdata,
    
    // SRAM Row1 Interface (Blue channel)
    output wire         sram_r1_b_we,
    output wire         sram_r1_b_re,
    output wire         sram_r1_b_waddren,
    output wire         sram_r1_b_raddren,
    output wire [10:0]  sram_r1_b_waddr,
    output wire [10:0]  sram_r1_b_raddr,
    output wire [7:0]   sram_r1_b_wdata,
    input  wire [7:0]   sram_r1_b_rdata,
    
    // SRAM Row2 Interface (Red channel)
    output wire         sram_r2_r_we,
    output wire         sram_r2_r_re,
    output wire         sram_r2_r_waddren,
    output wire         sram_r2_r_raddren,
    output wire [10:0]  sram_r2_r_waddr,
    output wire [10:0]  sram_r2_r_raddr,
    output wire [7:0]   sram_r2_r_wdata,
    input  wire [7:0]   sram_r2_r_rdata,
    
    // SRAM Row2 Interface (Green channel)
    output wire         sram_r2_g_we,
    output wire         sram_r2_g_re,
    output wire         sram_r2_g_waddren,
    output wire         sram_r2_g_raddren,
    output wire [10:0]  sram_r2_g_waddr,
    output wire [10:0]  sram_r2_g_raddr,
    output wire [7:0]   sram_r2_g_wdata,
    input  wire [7:0]   sram_r2_g_rdata,
    
    // SRAM Row2 Interface (Blue channel)
    output wire         sram_r2_b_we,
    output wire         sram_r2_b_re,
    output wire         sram_r2_b_waddren,
    output wire         sram_r2_b_raddren,
    output wire [10:0]  sram_r2_b_waddr,
    output wire [10:0]  sram_r2_b_raddr,
    output wire [7:0]   sram_r2_b_wdata,
    input  wire [7:0]   sram_r2_b_rdata
);

    // =========================================================================
    // Internal Signals
    // =========================================================================
    
    // Row and column counters
    reg [11:0] col_cnt;
    reg [11:0] row_cnt;
    
    // Line buffer management
    reg [1:0] wr_row_sel;  // Which SRAM row to write: 0=Row0, 1=Row1, 2=Row2
    
    // Pipeline stages for sync signals
    reg [2:0] vsync_pipe;
    reg [2:0] hsync_pipe;
    reg [2:0] de_pipe;
    
    // Shift registers for current row (3-pixel window)
    reg [7:0] row2_r_sr[2:0];  // Current row Red
    reg [7:0] row2_g_sr[2:0];  // Current row Green
    reg [7:0] row2_b_sr[2:0];  // Current row Blue
    
    // SRAM read data pipeline (1 cycle delay)
    reg [7:0] row0_r_data, row1_r_data;
    reg [7:0] row0_g_data, row1_g_data;
    reg [7:0] row0_b_data, row1_b_data;
    
    // Shift registers for Row0 and Row1 (from SRAM)
    reg [7:0] row0_r_sr[2:0];
    reg [7:0] row0_g_sr[2:0];
    reg [7:0] row0_b_sr[2:0];
    
    reg [7:0] row1_r_sr[2:0];
    reg [7:0] row1_g_sr[2:0];
    reg [7:0] row1_b_sr[2:0];
    
    // Frame start detection
    reg vsync_d;
    wire frame_start = i_vsync & ~vsync_d;
    
    // Read/Write address
    reg [10:0] rw_addr;
    
    // Valid window flag (exclude borders)
    reg filter_valid;
    reg [11:0] valid_col_cnt;
    reg [11:0] valid_row_cnt;
    
    // Border handling flags
    reg is_top_border;    // First 2 rows
    reg is_left_border;   // First 2 columns
    
    // =========================================================================
    // Row and Column Counter
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_cnt <= 0;
            row_cnt <= 0;
            vsync_d <= 0;
        end else begin
            vsync_d <= i_vsync;
            
            if (frame_start) begin
                row_cnt <= 0;
                col_cnt <= 0;
                wr_row_sel <= 0;
            end else if (i_de) begin
                if (col_cnt == IMG_WIDTH - 1) begin
                    col_cnt <= 0;
                    row_cnt <= row_cnt + 1;
                    // Rotate write row: 0->1->2->0
                    wr_row_sel <= (wr_row_sel == 2) ? 0 : wr_row_sel + 1;
                end else begin
                    col_cnt <= col_cnt + 1;
                end
            end
        end
    end
    
    // =========================================================================
    // SRAM Address Management
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rw_addr <= 0;
        end else if (frame_start) begin
            rw_addr <= 0;
        end else if (i_de) begin
            if (col_cnt == IMG_WIDTH - 1)
                rw_addr <= 0;
            else
                rw_addr <= rw_addr + 1;
        end
    end
    
    // =========================================================================
    // SRAM Write Control
    // =========================================================================
    // Write to current row based on wr_row_sel
    
    // Row0 Write
    assign sram_r0_r_we = (wr_row_sel == 0) && i_de;
    assign sram_r0_r_waddren = 1'b1;
    assign sram_r0_r_waddr = rw_addr;
    assign sram_r0_r_wdata = i_r;
    
    assign sram_r0_g_we = (wr_row_sel == 0) && i_de;
    assign sram_r0_g_waddren = 1'b1;
    assign sram_r0_g_waddr = rw_addr;
    assign sram_r0_g_wdata = i_g;
    
    assign sram_r0_b_we = (wr_row_sel == 0) && i_de;
    assign sram_r0_b_waddren = 1'b1;
    assign sram_r0_b_waddr = rw_addr;
    assign sram_r0_b_wdata = i_b;
    
    // Row1 Write
    assign sram_r1_r_we = (wr_row_sel == 1) && i_de;
    assign sram_r1_r_waddren = 1'b1;
    assign sram_r1_r_waddr = rw_addr;
    assign sram_r1_r_wdata = i_r;
    
    assign sram_r1_g_we = (wr_row_sel == 1) && i_de;
    assign sram_r1_g_waddren = 1'b1;
    assign sram_r1_g_waddr = rw_addr;
    assign sram_r1_g_wdata = i_g;
    
    assign sram_r1_b_we = (wr_row_sel == 1) && i_de;
    assign sram_r1_b_waddren = 1'b1;
    assign sram_r1_b_waddr = rw_addr;
    assign sram_r1_b_wdata = i_b;
    
    // Row2 Write
    assign sram_r2_r_we = (wr_row_sel == 2) && i_de;
    assign sram_r2_r_waddren = 1'b1;
    assign sram_r2_r_waddr = rw_addr;
    assign sram_r2_r_wdata = i_r;
    
    assign sram_r2_g_we = (wr_row_sel == 2) && i_de;
    assign sram_r2_g_waddren = 1'b1;
    assign sram_r2_g_waddr = rw_addr;
    assign sram_r2_g_wdata = i_g;
    
    assign sram_r2_b_we = (wr_row_sel == 2) && i_de;
    assign sram_r2_b_waddren = 1'b1;
    assign sram_r2_b_waddr = rw_addr;
    assign sram_r2_b_wdata = i_b;
    
    // =========================================================================
    // SRAM Read Control
    // =========================================================================
    // Read from two previous rows based on wr_row_sel
    // If writing to Row0, read from Row1 and Row2
    // If writing to Row1, read from Row2 and Row0
    // If writing to Row2, read from Row0 and Row1
    
    wire [1:0] rd_row0_sel = (wr_row_sel == 0) ? 2'd1 : (wr_row_sel == 1) ? 2'd2 : 2'd0;
    wire [1:0] rd_row1_sel = (wr_row_sel == 0) ? 2'd2 : (wr_row_sel == 1) ? 2'd0 : 2'd1;
    
    // All SRAMs read enable when data valid
    assign sram_r0_r_re = i_de;
    assign sram_r0_r_raddren = 1'b1;
    assign sram_r0_r_raddr = rw_addr;
    
    assign sram_r0_g_re = i_de;
    assign sram_r0_g_raddren = 1'b1;
    assign sram_r0_g_raddr = rw_addr;
    
    assign sram_r0_b_re = i_de;
    assign sram_r0_b_raddren = 1'b1;
    assign sram_r0_b_raddr = rw_addr;
    
    assign sram_r1_r_re = i_de;
    assign sram_r1_r_raddren = 1'b1;
    assign sram_r1_r_raddr = rw_addr;
    
    assign sram_r1_g_re = i_de;
    assign sram_r1_g_raddren = 1'b1;
    assign sram_r1_g_raddr = rw_addr;
    
    assign sram_r1_b_re = i_de;
    assign sram_r1_b_raddren = 1'b1;
    assign sram_r1_b_raddr = rw_addr;
    
    assign sram_r2_r_re = i_de;
    assign sram_r2_r_raddren = 1'b1;
    assign sram_r2_r_raddr = rw_addr;
    
    assign sram_r2_g_re = i_de;
    assign sram_r2_g_raddren = 1'b1;
    assign sram_r2_g_raddr = rw_addr;
    
    assign sram_r2_b_re = i_de;
    assign sram_r2_b_raddren = 1'b1;
    assign sram_r2_b_raddr = rw_addr;
    
    // =========================================================================
    // Pipeline Stage 1: SRAM Read Data + Input Data
    // =========================================================================
    // Route SRAM outputs based on read row selection
    wire [7:0] sram_rd_row0_r = (rd_row0_sel == 0) ? sram_r0_r_rdata : 
                                 (rd_row0_sel == 1) ? sram_r1_r_rdata : sram_r2_r_rdata;
    wire [7:0] sram_rd_row0_g = (rd_row0_sel == 0) ? sram_r0_g_rdata : 
                                 (rd_row0_sel == 1) ? sram_r1_g_rdata : sram_r2_g_rdata;
    wire [7:0] sram_rd_row0_b = (rd_row0_sel == 0) ? sram_r0_b_rdata : 
                                 (rd_row0_sel == 1) ? sram_r1_b_rdata : sram_r2_b_rdata;
    
    wire [7:0] sram_rd_row1_r = (rd_row1_sel == 0) ? sram_r0_r_rdata : 
                                 (rd_row1_sel == 1) ? sram_r1_r_rdata : sram_r2_r_rdata;
    wire [7:0] sram_rd_row1_g = (rd_row1_sel == 0) ? sram_r0_g_rdata : 
                                 (rd_row1_sel == 1) ? sram_r1_g_rdata : sram_r2_g_rdata;
    wire [7:0] sram_rd_row1_b = (rd_row1_sel == 0) ? sram_r0_b_rdata : 
                                 (rd_row1_sel == 1) ? sram_r1_b_rdata : sram_r2_b_rdata;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row0_r_data <= 0; row0_g_data <= 0; row0_b_data <= 0;
            row1_r_data <= 0; row1_g_data <= 0; row1_b_data <= 0;
            
            row2_r_sr[0] <= 0; row2_g_sr[0] <= 0; row2_b_sr[0] <= 0;
            
            vsync_pipe[0] <= 0;
            hsync_pipe[0] <= 0;
            de_pipe[0] <= 0;
        end else begin
            // Capture SRAM read data (1 cycle after read enable)
            row0_r_data <= sram_rd_row0_r;
            row0_g_data <= sram_rd_row0_g;
            row0_b_data <= sram_rd_row0_b;
            
            row1_r_data <= sram_rd_row1_r;
            row1_g_data <= sram_rd_row1_g;
            row1_b_data <= sram_rd_row1_b;
            
            // Current row data (direct from input)
            row2_r_sr[0] <= i_r;
            row2_g_sr[0] <= i_g;
            row2_b_sr[0] <= i_b;
            
            // Sync signal pipeline
            vsync_pipe[0] <= i_vsync;
            hsync_pipe[0] <= i_hsync;
            de_pipe[0] <= i_de;
        end
    end
    
    // =========================================================================
    // Pipeline Stage 2: Build 3-pixel shift registers
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row0_r_sr[0] <= 0; row0_r_sr[1] <= 0; row0_r_sr[2] <= 0;
            row0_g_sr[0] <= 0; row0_g_sr[1] <= 0; row0_g_sr[2] <= 0;
            row0_b_sr[0] <= 0; row0_b_sr[1] <= 0; row0_b_sr[2] <= 0;
            
            row1_r_sr[0] <= 0; row1_r_sr[1] <= 0; row1_r_sr[2] <= 0;
            row1_g_sr[0] <= 0; row1_g_sr[1] <= 0; row1_g_sr[2] <= 0;
            row1_b_sr[0] <= 0; row1_b_sr[1] <= 0; row1_b_sr[2] <= 0;
            
            row2_r_sr[1] <= 0; row2_r_sr[2] <= 0;
            row2_g_sr[1] <= 0; row2_g_sr[2] <= 0;
            row2_b_sr[1] <= 0; row2_b_sr[2] <= 0;
            
            vsync_pipe[1] <= 0;
            hsync_pipe[1] <= 0;
            de_pipe[1] <= 0;
        end else begin
            // Shift Row0 data
            row0_r_sr[0] <= row0_r_data;
            row0_r_sr[1] <= row0_r_sr[0];
            row0_r_sr[2] <= row0_r_sr[1];
            
            row0_g_sr[0] <= row0_g_data;
            row0_g_sr[1] <= row0_g_sr[0];
            row0_g_sr[2] <= row0_g_sr[1];
            
            row0_b_sr[0] <= row0_b_data;
            row0_b_sr[1] <= row0_b_sr[0];
            row0_b_sr[2] <= row0_b_sr[1];
            
            // Shift Row1 data
            row1_r_sr[0] <= row1_r_data;
            row1_r_sr[1] <= row1_r_sr[0];
            row1_r_sr[2] <= row1_r_sr[1];
            
            row1_g_sr[0] <= row1_g_data;
            row1_g_sr[1] <= row1_g_sr[0];
            row1_g_sr[2] <= row1_g_sr[1];
            
            row1_b_sr[0] <= row1_b_data;
            row1_b_sr[1] <= row1_b_sr[0];
            row1_b_sr[2] <= row1_b_sr[1];
            
            // Shift Row2 data
            row2_r_sr[1] <= row2_r_sr[0];
            row2_r_sr[2] <= row2_r_sr[1];
            
            row2_g_sr[1] <= row2_g_sr[0];
            row2_g_sr[2] <= row2_g_sr[1];
            
            row2_b_sr[1] <= row2_b_sr[0];
            row2_b_sr[2] <= row2_b_sr[1];
            
            // Sync signal pipeline
            vsync_pipe[1] <= vsync_pipe[0];
            hsync_pipe[1] <= hsync_pipe[0];
            de_pipe[1] <= de_pipe[0];
        end
    end
    
    // =========================================================================
    // Valid Window Detection
    // =========================================================================
    // Always output data, but mark border regions for special handling
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_row_cnt <= 0;
            valid_col_cnt <= 0;
            filter_valid <= 0;
            is_top_border <= 0;
            is_left_border <= 0;
        end else begin
            if (frame_start) begin
                valid_row_cnt <= 0;
                valid_col_cnt <= 0;
            end else if (de_pipe[1]) begin
                if (valid_col_cnt == IMG_WIDTH - 1) begin
                    valid_col_cnt <= 0;
                    if (valid_row_cnt < IMG_HEIGHT - 1)
                        valid_row_cnt <= valid_row_cnt + 1;
                end else begin
                    valid_col_cnt <= valid_col_cnt + 1;
                end
            end
            
            // Filter is valid when we have enough data (row >= 2, col >= 2)
            filter_valid <= de_pipe[1] && (valid_row_cnt >= 2) && (valid_col_cnt >= 2);
            
            // Mark border regions
            is_top_border <= de_pipe[1] && (valid_row_cnt < 2);
            is_left_border <= de_pipe[1] && (valid_col_cnt < 2);
        end
    end
    
    // =========================================================================
    // Pipeline Stage 3: 3x3 Mean Filter Computation
    // =========================================================================
    // Sum all 9 pixels and divide by 9 (approximate with >>3 + >>4)
    // Actual division by 9 can use: (sum * 455) >> 12 ≈ sum / 9
    
    reg [13:0] sum_r, sum_g, sum_b;
    
    // Border handling flags pipeline
    reg is_top_border_d;
    reg is_left_border_d;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum_r <= 0; sum_g <= 0; sum_b <= 0;
            vsync_pipe[2] <= 0;
            hsync_pipe[2] <= 0;
            de_pipe[2] <= 0;
            is_top_border_d <= 0;
            is_left_border_d <= 0;
        end else begin
            // Sum all 9 pixels (each 8-bit, max sum = 9*255 = 2295, fits in 12 bits)
            sum_r <= row0_r_sr[0] + row0_r_sr[1] + row0_r_sr[2] +
                     row1_r_sr[0] + row1_r_sr[1] + row1_r_sr[2] +
                     row2_r_sr[0] + row2_r_sr[1] + row2_r_sr[2];
                     
            sum_g <= row0_g_sr[0] + row0_g_sr[1] + row0_g_sr[2] +
                     row1_g_sr[0] + row1_g_sr[1] + row1_g_sr[2] +
                     row2_g_sr[0] + row2_g_sr[1] + row2_g_sr[2];
                     
            sum_b <= row0_b_sr[0] + row0_b_sr[1] + row0_b_sr[2] +
                     row1_b_sr[0] + row1_b_sr[1] + row1_b_sr[2] +
                     row2_b_sr[0] + row2_b_sr[1] + row2_b_sr[2];
            
            // Sync signal pipeline
            vsync_pipe[2] <= vsync_pipe[1];
            hsync_pipe[2] <= hsync_pipe[1];
            // Always output de, not just when filter is valid
            de_pipe[2] <= de_pipe[1];
            
            // Pipeline border flags
            is_top_border_d <= is_top_border;
            is_left_border_d <= is_left_border;
        end
    end
    
    // =========================================================================
    // Output Stage: Divide by 9 and output
    // =========================================================================
    // Divide by 9 using: (sum * 455) >> 12
    // For simplicity, we can use (sum * 7) >> 6 ≈ sum / 9.14 (close approximation)
    // Or even simpler: sum / 9 using combinational divider (not recommended for speed)
    // Best approach: (sum * 57) >> 9 = sum * 0.111 ≈ sum / 9
    
    wire [20:0] mult_r = sum_r * 57;  // 14-bit * 6-bit = 20-bit
    wire [20:0] mult_g = sum_g * 57;
    wire [20:0] mult_b = sum_b * 57;
    
    // Border pixel data (use center pixel from current input)
    reg [7:0] border_r, border_g, border_b;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            border_r <= 0;
            border_g <= 0;
            border_b <= 0;
        end else begin
            // Store center pixel for border filling
            border_r <= row2_r_sr[1];
            border_g <= row2_g_sr[1];
            border_b <= row2_b_sr[1];
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_vsync <= 0;
            o_hsync <= 0;
            o_de <= 0;
            o_r <= 0;
            o_g <= 0;
            o_b <= 0;
        end else begin
            o_vsync <= vsync_pipe[2];
            o_hsync <= hsync_pipe[2];
            o_de <= de_pipe[2];
            
            // Output selection: filtered data or border fill
            if (is_top_border_d || is_left_border_d) begin
                // Border region: output the border pixel (replicate edge)
                o_r <= border_r;
                o_g <= border_g;
                o_b <= border_b;
            end else begin
                // Normal region: output filtered data
                // Divide by 9: multiply by 57 and shift right by 9
                o_r <= mult_r[16:9];
                o_g <= mult_g[16:9];
                o_b <= mult_b[16:9];
            end
        end
    end

endmodule

