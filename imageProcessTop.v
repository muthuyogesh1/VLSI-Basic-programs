`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 29.04.2025 15:38:21
// Design Name: 
// Module Name: imageProcessTop
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


module imageProcessTop(
input   axi_clk,
input   axi_reset_n,
//slave interface
input   i_data_valid,
input [7:0] i_data,
output  o_data_ready,
//master interface
output  o_data_valid,
output [7:0] o_data,
input   i_data_ready,
//interrupt
output  o_intr

    );

wire [71:0] pixel_data;
wire pixel_data_valid;
wire axis_prog_full;
wire [7:0] convolved_data;
wire convolved_data_valid;

assign o_data_ready = !axis_prog_full;
    
imageControl IC(
    .i_clk(axi_clk),
    .i_rst(!axi_reset_n),
    .i_pixel_data(i_data),
    .i_pixel_data_valid(i_data_valid),
    .o_pixel_data(pixel_data),
    .o_pixel_data_valid(pixel_data_valid),
    .o_intr(o_intr)
  );    
  
 conv conv(
     .i_clk(axi_clk),
     .i_pixel_data(pixel_data),
     .i_pixel_data_valid(pixel_data_valid),
     .o_convolved_data(convolved_data),
     .o_convolved_data_valid(convolved_data_valid)
 ); 
 
 outputBuffer OB (
   .wr_rst_busy(),        // output wire wr_rst_busy
   .rd_rst_busy(),        // output wire rd_rst_busy
   .s_aclk(axi_clk),                  // input wire s_aclk
   .s_aresetn(axi_reset_n),            // input wire s_aresetn
   .s_axis_tvalid(convolved_data_valid),    // input wire s_axis_tvalid
   .s_axis_tready(),    // output wire s_axis_tready
   .s_axis_tdata(convolved_data),      // input wire [7 : 0] s_axis_tdata
   .m_axis_tvalid(o_data_valid),    // output wire m_axis_tvalid
   .m_axis_tready(i_data_ready),    // input wire m_axis_tready
   .m_axis_tdata(o_data),      // output wire [7 : 0] m_axis_tdata
   .axis_prog_full(axis_prog_full)  // output wire axis_prog_full
 );
  
    
    
endmodule


module imageControl(
input                    i_clk,
input                    i_rst,
input [7:0]              i_pixel_data,
input                    i_pixel_data_valid,
output reg [71:0]        o_pixel_data,
output                   o_pixel_data_valid,
output reg               o_intr
);

reg [8:0] pixelCounter;
reg [1:0] currentWrLineBuffer;
reg [3:0] lineBuffDataValid;
reg [3:0] lineBuffRdData;
reg [1:0] currentRdLineBuffer;
wire [23:0] lb0data;
wire [23:0] lb1data;
wire [23:0] lb2data;
wire [23:0] lb3data;
reg [8:0] rdCounter;
reg rd_line_buffer;
reg [11:0] totalPixelCounter;
reg rdState;

localparam IDLE = 'b0,
           RD_BUFFER = 'b1;

assign o_pixel_data_valid = rd_line_buffer;

always @(posedge i_clk)
begin
    if(i_rst)
        totalPixelCounter <= 0;
    else
    begin
        if(i_pixel_data_valid & !rd_line_buffer)
            totalPixelCounter <= totalPixelCounter + 1;
        else if(!i_pixel_data_valid & rd_line_buffer)
            totalPixelCounter <= totalPixelCounter - 1;
    end
end

always @(posedge i_clk)
begin
    if(i_rst)
    begin
        rdState <= IDLE;
        rd_line_buffer <= 1'b0;
        o_intr <= 1'b0;
    end
    else
    begin
        case(rdState)
            IDLE:begin
                o_intr <= 1'b0;
                if(totalPixelCounter >= 1536)
                begin
                    rd_line_buffer <= 1'b1;
                    rdState <= RD_BUFFER;
                end
            end
            RD_BUFFER:begin
                if(rdCounter == 511)
                begin
                    rdState <= IDLE;
                    rd_line_buffer <= 1'b0;
                    o_intr <= 1'b1;
                end
            end
        endcase
    end
end
    
always @(posedge i_clk)
begin
    if(i_rst)
        pixelCounter <= 0;
    else 
    begin
        if(i_pixel_data_valid)
            pixelCounter <= pixelCounter + 1;
    end
end


always @(posedge i_clk)
begin
    if(i_rst)
        currentWrLineBuffer <= 0;
    else
    begin
        if(pixelCounter == 511 & i_pixel_data_valid)
            currentWrLineBuffer <= currentWrLineBuffer+1;
    end
end


always @(*)
begin
    lineBuffDataValid = 4'h0;
    lineBuffDataValid[currentWrLineBuffer] = i_pixel_data_valid;
end

always @(posedge i_clk)
begin
    if(i_rst)
        rdCounter <= 0;
    else 
    begin
        if(rd_line_buffer)
            rdCounter <= rdCounter + 1;
    end
end

always @(posedge i_clk)
begin
    if(i_rst)
    begin
        currentRdLineBuffer <= 0;
    end
    else
    begin
        if(rdCounter == 511 & rd_line_buffer)
            currentRdLineBuffer <= currentRdLineBuffer + 1;
    end
end


always @(*)
begin
    case(currentRdLineBuffer)
        0:begin
            o_pixel_data = {lb2data,lb1data,lb0data};
        end
        1:begin
            o_pixel_data = {lb3data,lb2data,lb1data};
        end
        2:begin
            o_pixel_data = {lb0data,lb3data,lb2data};
        end
        3:begin
            o_pixel_data = {lb1data,lb0data,lb3data};
        end
    endcase
end

always @(*)
begin
    case(currentRdLineBuffer)
        0:begin
            lineBuffRdData[0] = rd_line_buffer;
            lineBuffRdData[1] = rd_line_buffer;
            lineBuffRdData[2] = rd_line_buffer;
            lineBuffRdData[3] = 1'b0;
        end
       1:begin
            lineBuffRdData[0] = 1'b0;
            lineBuffRdData[1] = rd_line_buffer;
            lineBuffRdData[2] = rd_line_buffer;
            lineBuffRdData[3] = rd_line_buffer;
        end
       2:begin
             lineBuffRdData[0] = rd_line_buffer;
             lineBuffRdData[1] = 1'b0;
             lineBuffRdData[2] = rd_line_buffer;
             lineBuffRdData[3] = rd_line_buffer;
       end  
      3:begin
             lineBuffRdData[0] = rd_line_buffer;
             lineBuffRdData[1] = rd_line_buffer;
             lineBuffRdData[2] = 1'b0;
             lineBuffRdData[3] = rd_line_buffer;
       end        
    endcase
end
    
lineBuffer lB0(
    .i_clk(i_clk),
    .i_rst(i_rst),
    .i_data(i_pixel_data),
    .i_data_valid(lineBuffDataValid[0]),
    .o_data(lb0data),
    .i_rd_data(lineBuffRdData[0])
 ); 
 
 lineBuffer lB1(
     .i_clk(i_clk),
     .i_rst(i_rst),
     .i_data(i_pixel_data),
     .i_data_valid(lineBuffDataValid[1]),
     .o_data(lb1data),
     .i_rd_data(lineBuffRdData[1])
  ); 
  
  lineBuffer lB2(
      .i_clk(i_clk),
      .i_rst(i_rst),
      .i_data(i_pixel_data),
      .i_data_valid(lineBuffDataValid[2]),
      .o_data(lb2data),
      .i_rd_data(lineBuffRdData[2])
   ); 
   
   lineBuffer lB3(
       .i_clk(i_clk),
       .i_rst(i_rst),
       .i_data(i_pixel_data),
       .i_data_valid(lineBuffDataValid[3]),
       .o_data(lb3data),
       .i_rd_data(lineBuffRdData[3])
    );    
    
endmodule




module conv(
input        i_clk,
input [71:0] i_pixel_data,
input        i_pixel_data_valid,
output reg [7:0] o_convolved_data,
output reg   o_convolved_data_valid
    );
    
integer i; 
reg [7:0] kernel1 [8:0];
reg [7:0] kernel2 [8:0];
reg [10:0] multData1[8:0];
reg [10:0] multData2[8:0];
reg [10:0] sumDataInt1;
reg [10:0] sumDataInt2;
reg [10:0] sumData1;
reg [10:0] sumData2;
reg multDataValid;
reg sumDataValid;
reg convolved_data_valid;
reg [20:0] convolved_data_int1;
reg [20:0] convolved_data_int2;
wire [21:0] convolved_data_int;
reg convolved_data_int_valid;

initial
begin
    kernel1[0] =  1;
    kernel1[1] =  0;
    kernel1[2] = -1;
    kernel1[3] =  2;
    kernel1[4] =  0;
    kernel1[5] = -2;
    kernel1[6] =  1;
    kernel1[7] =  0;
    kernel1[8] = -1;
    
    kernel2[0] =  1;
    kernel2[1] =  2;
    kernel2[2] =  1;
    kernel2[3] =  0;
    kernel2[4] =  0;
    kernel2[5] =  0;
    kernel2[6] = -1;
    kernel2[7] = -2;
    kernel2[8] = -1;
end    
    
always @(posedge i_clk)
begin
    for(i=0;i<9;i=i+1)
    begin
        multData1[i] <= $signed(kernel1[i])*$signed({1'b0,i_pixel_data[i*8+:8]});
        multData2[i] <= $signed(kernel2[i])*$signed({1'b0,i_pixel_data[i*8+:8]});
    end
    multDataValid <= i_pixel_data_valid;
end


always @(*)
begin
    sumDataInt1 = 0;
    sumDataInt2 = 0;
    for(i=0;i<9;i=i+1)
    begin
        sumDataInt1 = $signed(sumDataInt1) + $signed(multData1[i]);
        sumDataInt2 = $signed(sumDataInt2) + $signed(multData2[i]);
    end
end

always @(posedge i_clk)
begin
    sumData1 <= sumDataInt1;
    sumData2 <= sumDataInt2;
    sumDataValid <= multDataValid;
end

always @(posedge i_clk)
begin
    convolved_data_int1 <= $signed(sumData1)*$signed(sumData1);
    convolved_data_int2 <= $signed(sumData2)*$signed(sumData2);
    convolved_data_int_valid <= sumDataValid;
end

assign convolved_data_int = convolved_data_int1+convolved_data_int2;

    
always @(posedge i_clk)
begin
    if(convolved_data_int > 4000)
        o_convolved_data <= 8'hff;
    else
        o_convolved_data <= 8'h00;
    o_convolved_data_valid <= convolved_data_int_valid;
end
    
endmodule




module lineBuffer(
input   i_clk,
input   i_rst,
input [7:0] i_data,
input   i_data_valid,
output [23:0] o_data,
input i_rd_data
);

reg [7:0] line [511:0]; //line buffer
reg [8:0] wrPntr;
reg [8:0] rdPntr;

always @(posedge i_clk)
begin
    if(i_data_valid)
        line[wrPntr] <= i_data;
end

always @(posedge i_clk)
begin
    if(i_rst)
        wrPntr <= 'd0;
    else if(i_data_valid)
        wrPntr <= wrPntr + 'd1;
end

assign o_data = {line[rdPntr],line[rdPntr+1],line[rdPntr+2]};

always @(posedge i_clk)
begin
    if(i_rst)
        rdPntr <= 'd0;
    else if(i_rd_data)
        rdPntr <= rdPntr + 'd1;
end


endmodule


module outputBuffer #(
    parameter WIDTH = 8,
    parameter DEPTH = 18,
    parameter ADDR_WIDTH = $clog2(DEPTH)
)(
    input  wire                  s_aclk,
    input  wire                  s_aresetn,

    // Slave AXIS interface
    input  wire                  s_axis_tvalid,
    output wire                  s_axis_tready,
    input  wire [WIDTH-1:0]      s_axis_tdata,

    // Master AXIS interface
    output wire                  m_axis_tvalid,
    input  wire                  m_axis_tready,
    output wire [WIDTH-1:0]      m_axis_tdata,

    // Control signals
    output wire                  axis_prog_full,
    output wire                  wr_rst_busy,
    output wire                  rd_rst_busy
);

    // FIFO storage
    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_WIDTH-1:0] wr_ptr = 0;
    reg [ADDR_WIDTH-1:0] rd_ptr = 0;
    reg [$clog2(DEPTH+1)-1:0] count = 0;

    // Pipelined output
    reg [WIDTH-1:0] pipe1 = 0;
    reg [WIDTH-1:0] pipe2 = 0;
    reg             valid_pipe1 = 0;
    reg             valid_pipe2 = 0;

    // FIFO full/empty
    wire full  = (count == DEPTH);
    wire empty = (count == 0);

    assign s_axis_tready = ~full;
    assign m_axis_tvalid = valid_pipe2;
    assign m_axis_tdata  = pipe2;
    assign axis_prog_full = (count >= (DEPTH - 2));

    assign wr_rst_busy = ~s_aresetn;
    assign rd_rst_busy = ~s_aresetn;

    // Write logic
    always @(posedge s_aclk) begin
        if (!s_aresetn) begin
            wr_ptr <= 0;
        end else if (s_axis_tvalid && s_axis_tready) begin
            mem[wr_ptr] <= s_axis_tdata;
            wr_ptr <= wr_ptr + 1;
        end
    end

    // Read logic into pipe1
    wire read_en = (!empty && (!valid_pipe1 || (valid_pipe1 && !valid_pipe2)));

    always @(posedge s_aclk) begin
        if (!s_aresetn) begin
            rd_ptr <= 0;
        end else if (read_en) begin
            pipe1 <= mem[rd_ptr];
            valid_pipe1 <= 1'b1;
            rd_ptr <= rd_ptr + 1;
        end else if (!valid_pipe1) begin
            valid_pipe1 <= 0;
        end
    end

    // Pipe1 → Pipe2 (Latency = 2)
    always @(posedge s_aclk) begin
        if (!s_aresetn) begin
            pipe2 <= 0;
            valid_pipe2 <= 0;
        end else if (valid_pipe1 && (!valid_pipe2 || (valid_pipe2 && m_axis_tready))) begin
            pipe2 <= pipe1;
            valid_pipe2 <= 1'b1;
            valid_pipe1 <= 1'b0;
        end else if (valid_pipe2 && m_axis_tready) begin
            valid_pipe2 <= 0;
        end
    end

    // FIFO counter
    always @(posedge s_aclk) begin
        if (!s_aresetn) begin
            count <= 0;
        end else begin
            case ({s_axis_tvalid && s_axis_tready, read_en})
                2'b10: count <= count + 1;
                2'b01: count <= count - 1;
                default: count <= count;
            endcase
        end
    end

endmodule

