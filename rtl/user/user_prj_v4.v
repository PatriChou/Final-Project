// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * user_project_wrapper
 *
 * This wrapper enumerates all of the pins available to the
 * user for the user project.
 *
 * An example user project is provided in this wrapper.  The
 * example should be removed and replaced with the actual
 * user project.
 *
 *-------------------------------------------------------------
 */

module user_project_wrapper #(
    parameter BITS = 32
) (
`ifdef USE_POWER_PINS
    inout vdda1,	// User area 1 3.3V supply
    inout vdda2,	// User area 2 3.3V supply
    inout vssa1,	// User area 1 analog ground
    inout vssa2,	// User area 2 analog ground
    inout vccd1,	// User area 1 1.8V supply
    inout vccd2,	// User area 2 1.8v supply
    inout vssd1,	// User area 1 digital ground
    inout vssd2,	// User area 2 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // Analog (direct connection to GPIO pad---use with caution)
    // Note that analog I/O is not available on the 7 lowest-numbered
    // GPIO pads, and so the analog_io indexing is offset from the
    // GPIO indexing by 7 (also upper 2 GPIOs do not have analog_io).
    inout [`MPRJ_IO_PADS-10:0] analog_io,

    // Independent clock (on independent integer divider)
    input   user_clock2,

    // User maskable interrupt signals
    output [2:0] user_irq
);


wire wbs_cyc_exdata_read, wbs_cyc_exinst_read;
wire wbs_cyc_exdata_write, wbs_cyc_exinst_write;
wire wbs_we_exmem, wbs_ack_o_exmem, wbs_ack_o_wbaxi;
wire [31:0] wbs_dat_i_exmem, wbs_adr_i_exmem, wbs_dat_o_exmem;
wire wbs_cyc_exdata, wbs_cyc_exinst, wbs_cyc_wbaxi, wbs_cyc_exmem;
wire [31:0]wbs_dat_o_wbaxi;
reg [31:0] wbs_dat_o_d;
reg [2:0] curr_state, next_state;
reg [3:0] load_cnt;
reg [31:0]  RAM_data  [0:7];
reg [6:0]   Tag_data;
reg [31:0]  RAM_inst  [0:3][0:7];
reg [6:0]   Tag_inst  [0:3];
reg [2:0] flu_cnt;
wire inst_hit, data_hit;

localparam INIT_ST  = 3'd0;
localparam IDLE_ST  = 3'd1;
localparam MISS_ST  = 3'd2;
localparam HIT_ST   = 3'd3;
localparam LOAD_ST  = 3'd4;

assign inst_hit = ( curr_state != LOAD_ST && wbs_cyc_exinst_read && ( (Tag_inst[0] == wbs_adr_i[11:5]) ||  
                                                                      (Tag_inst[1] == wbs_adr_i[11:5]) || 
                                                                      (Tag_inst[2] == wbs_adr_i[11:5]) || 
                                                                      (Tag_inst[3] == wbs_adr_i[11:5]) ) )? 1'b1:1'b0;

assign data_hit = ( curr_state != LOAD_ST && wbs_cyc_exdata_read && ( (Tag_data == wbs_adr_i[11:5]) ) )? 1'b1:1'b0;

always@(posedge wb_clk_i or negedge wb_rst_i) begin
    if(wb_rst_i)
        curr_state <= INIT_ST;
    else 
        curr_state <= next_state;
end

always@(*) begin
    case(curr_state)
        INIT_ST: next_state = (wbs_cyc_exmem)? LOAD_ST:INIT_ST;

        IDLE_ST: next_state = (!(wbs_cyc_exdata || wbs_cyc_exinst) || wbs_we_i)? IDLE_ST:
                              (data_hit || inst_hit)?     HIT_ST:
                                                          LOAD_ST;
        
        HIT_ST:  next_state = (data_hit || inst_hit)? HIT_ST:IDLE_ST;
        
        LOAD_ST: next_state = (load_cnt == 4'd7 && wbs_ack_o_exmem)? IDLE_ST : LOAD_ST; 
        
        default: next_state = INIT_ST;
    endcase
end
/*--------------------------------------*/
/* Prefetch Cache   */
/*--------------------------------------*/

always@(posedge wb_clk_i or negedge wb_rst_i) begin
    if(wb_rst_i) begin
        Tag_data <= 7'd0;
        Tag_inst[0] <= 7'd0;
        Tag_inst[1] <= 7'd0;
        Tag_inst[2] <= 7'd0;
        Tag_inst[3] <= 7'd0;
    end
    else if(curr_state == LOAD_ST) begin
        Tag_data <= (wbs_cyc_exdata_read)? wbs_adr_i[11:5] : Tag_data;
        Tag_inst[0] <= (wbs_cyc_exinst_read && (flu_cnt == 3'd0) )? wbs_adr_i[11:5] : Tag_inst[0];
        Tag_inst[1] <= (wbs_cyc_exinst_read && (flu_cnt == 3'd1) )? wbs_adr_i[11:5] : Tag_inst[1];
        Tag_inst[2] <= (wbs_cyc_exinst_read && (flu_cnt == 3'd2) )? wbs_adr_i[11:5] : Tag_inst[2];
        Tag_inst[3] <= (wbs_cyc_exinst_read && (flu_cnt == 3'd3) )? wbs_adr_i[11:5] : Tag_inst[3];
    end
    else begin
        Tag_data <= Tag_data;
        Tag_inst[0] <= Tag_inst[0];
        Tag_inst[1] <= Tag_inst[1];
        Tag_inst[2] <= Tag_inst[2];
        Tag_inst[3] <= Tag_inst[3];
    end
end

always@(posedge wb_clk_i) begin
    if(curr_state == INIT_ST) 
        flu_cnt <= 3'd0;
    else if(curr_state == LOAD_ST && next_state != LOAD_ST && wbs_cyc_exinst_read)
        flu_cnt <= (flu_cnt == 3'd3)? 3'd0 : flu_cnt + 3'd1;
    else
        flu_cnt <= flu_cnt;
end

always@(posedge wb_clk_i) begin
    if(curr_state != LOAD_ST && next_state == LOAD_ST)
        load_cnt <= 4'd0;
    else if(curr_state == LOAD_ST && wbs_ack_o_exmem && load_cnt != 4'hf)
        load_cnt <= load_cnt + 4'h1;
    else
        load_cnt <= load_cnt;
end

// read or write
always@(posedge wb_clk_i) begin
    if(wbs_cyc_exdata_write && (Tag_data == wbs_adr_i[11:5])) begin
        RAM_data[wbs_adr_i[4:2]] <= wbs_dat_i;
    end
    else if(curr_state == LOAD_ST && wbs_cyc_exdata && wbs_ack_o_exmem) begin
         RAM_data[load_cnt[2:0]] <= wbs_dat_o_exmem;
    end
    else begin
        RAM_data[0] <= RAM_data[0];
        RAM_data[1] <= RAM_data[1];
        RAM_data[2] <= RAM_data[2];
        RAM_data[3] <= RAM_data[3];
        RAM_data[4] <= RAM_data[4];
        RAM_data[5] <= RAM_data[5];
        RAM_data[6] <= RAM_data[6];
        RAM_data[7] <= RAM_data[7];
    end
end

always@(posedge wb_clk_i) begin
    if(wbs_cyc_exinst_write) begin
        if(wbs_adr_i[11:5] == Tag_inst[0])        RAM_inst[0][wbs_adr_i[4:2]] <= wbs_dat_i;
        else if (wbs_adr_i[11:5] == Tag_inst[1])  RAM_inst[1][wbs_adr_i[4:2]] <= wbs_dat_i;
        else if (wbs_adr_i[11:5] == Tag_inst[2])  RAM_inst[2][wbs_adr_i[4:2]] <= wbs_dat_i;
        else if (wbs_adr_i[11:5] == Tag_inst[3])  RAM_inst[3][wbs_adr_i[4:2]] <= wbs_dat_i;
    end
    else if(curr_state == LOAD_ST && wbs_cyc_exinst_read && wbs_ack_o_exmem) begin
        if(wbs_adr_i[11:5] == Tag_inst[0])        RAM_inst[0][load_cnt[2:0]] <= wbs_dat_o_exmem;
        else if (wbs_adr_i[11:5] == Tag_inst[1])  RAM_inst[1][load_cnt[2:0]] <= wbs_dat_o_exmem;
        else if (wbs_adr_i[11:5] == Tag_inst[2])  RAM_inst[2][load_cnt[2:0]] <= wbs_dat_o_exmem;
        else if (wbs_adr_i[11:5] == Tag_inst[3])  RAM_inst[3][load_cnt[2:0]] <= wbs_dat_o_exmem;
    end
    else begin
        RAM_inst[0][0] <= RAM_inst[0][0];RAM_inst[0][1] <= RAM_inst[0][1];RAM_inst[0][2] <= RAM_inst[0][2];RAM_inst[0][3] <= RAM_inst[0][3];
        RAM_inst[0][4] <= RAM_inst[0][4];RAM_inst[0][5] <= RAM_inst[0][5];RAM_inst[0][6] <= RAM_inst[0][6];RAM_inst[0][7] <= RAM_inst[0][7];
        RAM_inst[1][0] <= RAM_inst[1][0];RAM_inst[1][1] <= RAM_inst[1][1];RAM_inst[1][2] <= RAM_inst[1][2];RAM_inst[1][3] <= RAM_inst[1][3];
        RAM_inst[1][4] <= RAM_inst[1][4];RAM_inst[1][5] <= RAM_inst[1][5];RAM_inst[1][6] <= RAM_inst[1][6];RAM_inst[1][7] <= RAM_inst[1][7];
        RAM_inst[2][0] <= RAM_inst[2][0];RAM_inst[2][1] <= RAM_inst[2][1];RAM_inst[2][2] <= RAM_inst[2][2];RAM_inst[2][3] <= RAM_inst[2][3];
        RAM_inst[2][4] <= RAM_inst[2][4];RAM_inst[2][5] <= RAM_inst[2][5];RAM_inst[2][6] <= RAM_inst[2][6];RAM_inst[2][7] <= RAM_inst[2][7];        
        RAM_inst[3][0] <= RAM_inst[3][0];RAM_inst[3][1] <= RAM_inst[3][1];RAM_inst[3][2] <= RAM_inst[3][2];RAM_inst[3][3] <= RAM_inst[3][3];
        RAM_inst[3][4] <= RAM_inst[3][4];RAM_inst[3][5] <= RAM_inst[3][5];RAM_inst[3][6] <= RAM_inst[3][6];RAM_inst[3][7] <= RAM_inst[3][7];
    end
end

/*--------------------------------------*/
/* Wishbone Decoder   */
/*--------------------------------------*/

assign wbs_cyc_exdata_read = wbs_cyc_exdata & ~wbs_we_i;
assign wbs_cyc_exinst_read = wbs_cyc_exinst & ~wbs_we_i;
assign wbs_cyc_exdata_write = wbs_cyc_exdata & wbs_we_i;
assign wbs_cyc_exinst_write = wbs_cyc_exinst & wbs_we_i;
assign wbs_cyc_exdata = (wbs_cyc_i && (wbs_adr_i[31:24] == 8'h38) && (wbs_adr_i[11:8] >= 4'h6))? 1'b1 : 1'b0;
assign wbs_cyc_exinst = (wbs_cyc_i && (wbs_adr_i[31:24] == 8'h38) && (wbs_adr_i[11:8] < 4'h6))? 1'b1 : 1'b0;
assign wbs_cyc_wbaxi = (wbs_cyc_i && (wbs_adr_i[31:24] == 8'h30) )? 1'b1 : 1'b0;

assign wbs_dat_o = wbs_dat_o_d;
always@(*) begin
    wbs_dat_o_d = wbs_dat_o_exmem;
    if(wbs_cyc_wbaxi) 
        wbs_dat_o_d = wbs_dat_o_wbaxi;
    else if(curr_state == HIT_ST && next_state == HIT_ST) begin
        if(wbs_cyc_exdata)
            wbs_dat_o_d = RAM_data[wbs_adr_i[4:2]];
        else begin
            if(wbs_adr_i[11:5] == Tag_inst[0])        wbs_dat_o_d = RAM_inst[0][wbs_adr_i[4:2]];
            else if (wbs_adr_i[11:5] == Tag_inst[1])  wbs_dat_o_d = RAM_inst[1][wbs_adr_i[4:2]];
            else if (wbs_adr_i[11:5] == Tag_inst[2])  wbs_dat_o_d = RAM_inst[2][wbs_adr_i[4:2]];
            else if (wbs_adr_i[11:5] == Tag_inst[3])  wbs_dat_o_d = RAM_inst[3][wbs_adr_i[4:2]];
        end
    end
end

assign wbs_ack_o = (wbs_cyc_wbaxi)?         wbs_ack_o_wbaxi:
                   (wbs_cyc_exinst_read && curr_state == LOAD_ST)? wbs_ack_o_exmem:
                   (curr_state == HIT_ST && next_state == HIT_ST)? 1'b1:
                   (wbs_we_i)?              wbs_ack_o_exmem:
                                            1'b0;


assign wbs_cyc_exmem =  (wbs_we_i)? wbs_cyc_i:
                        (curr_state == LOAD_ST)? 1'b1:
                                                 1'b0;
                                                
assign wbs_we_exmem = wbs_we_i;

assign wbs_dat_i_exmem = wbs_dat_i;

assign wbs_adr_i_exmem = (curr_state == LOAD_ST)? {wbs_adr_i[31:5], load_cnt[2:0], 2'b00}:
                                                  wbs_adr_i;

/*--------------------------------------*/
/* User project is instantiated  here   */
/*--------------------------------------*/

user_proj_example mprj_sdr (
`ifdef USE_POWER_PINS
	.vccd1(vccd1),	// User area 1 1.8V power
	.vssd1(vssd1),	// User area 1 digital ground
`endif

    .wb_clk_i(wb_clk_i),
    .wb_rst_i(wb_rst_i),

    // MGMT SoC Wishbone Slave

    .wbs_cyc_i(wbs_cyc_exmem),
    .wbs_stb_i(wbs_stb_i),
    .wbs_we_i(wbs_we_exmem),
    .wbs_sel_i(wbs_sel_i),
    .wbs_adr_i(wbs_adr_i_exmem),
    .wbs_dat_i(wbs_dat_i_exmem),
    .wbs_ack_o(wbs_ack_o_exmem),
    .wbs_dat_o(wbs_dat_o_exmem),

    // Logic Analyzer

    .la_data_in(la_data_in),
    .la_data_out(la_data_out),
    .la_oenb (la_oenb),

    // IO Pads

    .io_in (io_in),
    .io_out(io_out),
    .io_oeb(io_oeb),

    // IRQ
    .irq(user_irq)
);

// =================== WB AXI ======================
wb_axi wb_axi(
    .wb_clk_i(wb_clk_i),
    .wb_rst_i(~wb_rst_i),
    .wbs_stb_i(wbs_stb_i),
    .wbs_cyc_i(wbs_cyc_wbaxi),      // only change here 
    .wbs_we_i(wbs_we_i),
    .wbs_sel_i(wbs_sel_i),
    .wbs_adr_i(wbs_adr_i),
    .wbs_dat_i(wbs_dat_i),
    .wbs_ack_o(wbs_ack_o_wbaxi),
    .wbs_dat_o(wbs_dat_o_wbaxi)
);

endmodule	// user_project_wrapper

`default_nettype wire

