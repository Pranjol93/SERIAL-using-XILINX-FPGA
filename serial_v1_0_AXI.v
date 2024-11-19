// AXI4-lite GPIO IP implementation
// (gpio_v1_0_AXI.v)
// Jason Losh based on Xilinx IP tool auto-generated file
//
// Contains:
// AXI4-lite interface
// GPIO memory-mapped interface
// GPIO port interface and implemention
// GPIO interrupt generation

`timescale 1 ns / 1 ps

    module serial_v1_0_AXI #
    (
        // Bit width of S_AXI address bus
        parameter integer C_S_AXI_ADDR_WIDTH = 4
    )
    (
        // Ports to top level module (what makes this the GPIO IP module)
//        input clk, 
//        input reset, 
//        input [8:0] wr_data, 
//        input wr_request,
//        output reg out, // for baudrate
        output reg IDLE_led,
        output reg START_led,
        output reg D0_led,
        output reg STOP1_led,
        
        output wire Tx_out,
        output wire CLK_OUT, //for baudrate
        
        output [8:0] rd_data, 
//        input rd_request, 
        output empty, 
        output full, 
        output reg overflow, 
//        input clear_overflow_request, 
        output reg [4:0] wr_index, 
        output reg[4:0] rd_index, 
        output wire [4:0] watermark,

        // AXI clock and reset        
        input wire S_AXI_ACLK,
        input wire S_AXI_ARESETN,

        // AXI write channel
        // address:  add, protection, valid, ready
        // data:     data, byte enable strobes, valid, ready
        // response: response, valid, ready 
        input wire [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_AWADDR,
        input wire [2:0] S_AXI_AWPROT,
        input wire S_AXI_AWVALID,
        output wire S_AXI_AWREADY,
        
        input wire [31:0] S_AXI_WDATA,
        input wire [3:0] S_AXI_WSTRB,
        input wire S_AXI_WVALID,
        output wire  S_AXI_WREADY,
        
        output wire [1:0] S_AXI_BRESP,
        output wire S_AXI_BVALID,
        input wire S_AXI_BREADY,
        
        // AXI read channel
        // address: add, protection, valid, ready
        // data:    data, resp, valid, ready
        input wire [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_ARADDR,
        input wire [2:0] S_AXI_ARPROT,
        input wire S_AXI_ARVALID,
        output wire S_AXI_ARREADY,
        
        output wire [31:0] S_AXI_RDATA,
        output wire [1:0] S_AXI_RRESP,
        output wire S_AXI_RVALID,
        input wire S_AXI_RREADY
    );

    // Internal registers
    reg [31:0] latch_data;
    reg [31:0] status;
    reg [31:0] control;
    reg [31:0] brd;
//    reg [31:0] int_positive;
//    reg [31:0] int_negative;
//    reg [31:0] int_edge_mode;
//    reg [31:0] int_status;
//    reg [31:0] int_clear_request;
    
    // Register map
    // ofs  fn
    //   0  data (r/w)
    //   4  out (r/w)
    //   8  od (r/w)
    //  12  int_enable (r/w)
    //  16  int_positive (r/w)
    //  20  int_negative (r/w)
    //  24  int_edge_mode (r/w)
    //  28  int_status_clear (r/w1c)
    
    // Register numbers
    localparam integer DATA_REG             = 2'b00;
    localparam integer STATUS_REG              = 2'b01;
    localparam integer CONTROL_REG              = 2'b10;
    localparam integer BRD_REG                  = 2'b011;
//    localparam integer INT_POSITIVE_REG     = 3'b100;
//    localparam integer INT_NEGATIVE_REG     = 3'b101;
//    localparam integer INT_EDGE_MODE_REG    = 3'b110;
//    localparam integer INT_STATUS_CLEAR_REG = 3'b111;
    
    // AXI4-lite signals
    reg axi_awready;
    reg axi_wready;
    reg [1:0] axi_bresp;
    reg axi_bvalid;
    reg axi_arready;
    reg [31:0] axi_rdata;
    reg [1:0] axi_rresp;
    reg axi_rvalid;
    
    // friendly clock, reset, and bus signals from master
    wire axi_clk           = S_AXI_ACLK;
    wire axi_resetn        = S_AXI_ARESETN;
    wire [31:0] axi_awaddr = S_AXI_AWADDR;
    wire axi_awvalid       = S_AXI_AWVALID;
    wire axi_wvalid        = S_AXI_WVALID;
    wire [3:0] axi_wstrb   = S_AXI_WSTRB;
    wire axi_bready        = S_AXI_BREADY;
    wire [31:0] axi_araddr = S_AXI_ARADDR;
    wire axi_arvalid       = S_AXI_ARVALID;
    wire axi_rready        = S_AXI_RREADY;    
    
    // assign bus signals to master to internal reg names
    assign S_AXI_AWREADY = axi_awready;
    assign S_AXI_WREADY  = axi_wready;
    assign S_AXI_BRESP   = axi_bresp;
    assign S_AXI_BVALID  = axi_bvalid;
    assign S_AXI_ARREADY = axi_arready;
    assign S_AXI_RDATA   = axi_rdata;
    assign S_AXI_RRESP   = axi_rresp;
    assign S_AXI_RVALID  = axi_rvalid;
    
    // Handle gpio input metastability safely
//    reg [31:0] read_port_data;
//    reg [31:0] pre_read_port_data;
//    always_ff @ (posedge(axi_clk))
//    begin
//        pre_read_port_data <= gpio_data_in;
//        read_port_data <= pre_read_port_data;
//    end

    // Assert address ready handshake (axi_awready) 
    // - after address is valid (axi_awvalid)
    // - after data is valid (axi_wvalid)
    // - while configured to receive a write (aw_en)
    // De-assert ready (axi_awready)
    // - after write response channel ready handshake received (axi_bready)
    // - after this module sends write response channel valid (axi_bvalid) 
    wire wr_add_data_valid = axi_awvalid && axi_wvalid;
    reg aw_en;
    always_ff @ (posedge axi_clk)
    begin
        if (axi_resetn == 1'b0)
        begin
            axi_awready <= 1'b0;
            aw_en <= 1'b1;
        end
        else
        begin
            if (wr_add_data_valid && ~axi_awready && aw_en)
            begin
                axi_awready <= 1'b1;
                aw_en <= 1'b0;
            end
            else if (axi_bready && axi_bvalid)
                begin
                    aw_en <= 1'b1;
                    axi_awready <= 1'b0;
                end
            else           
                axi_awready <= 1'b0;
        end 
    end

    // Capture the write address (axi_awaddr) in the first clock (~axi_awready)
    // - after write address is valid (axi_awvalid)
    // - after write data is valid (axi_wvalid)
    // - while configured to receive a write (aw_en)
    reg [C_S_AXI_ADDR_WIDTH-1:0] waddr;
    always_ff @ (posedge axi_clk)
    begin
        if (axi_resetn == 1'b0)
            waddr <= 0;
        else if (wr_add_data_valid && ~axi_awready && aw_en)
            waddr <= axi_awaddr;
    end

    // Output write data ready handshake (axi_wready) generation for one clock
    // - after address is valid (axi_awvalid)
    // - after data is valid (axi_wvalid)
    // - while configured to receive a write (aw_en)
    always_ff @ (posedge axi_clk)
    begin
        if (axi_resetn == 1'b0)
            axi_wready <= 1'b0;
        else
            axi_wready <= (wr_add_data_valid && ~axi_wready && aw_en);
    end      
    
    
    
    
    

    // Write data to internal registers
    // - after address is valid (axi_awvalid)
    // - after write data is valid (axi_wvalid)
    // - after this module asserts ready for address handshake (axi_awready)
    // - after this module asserts ready for data handshake (axi_wready)
    // write correct bytes in 32-bit word based on byte enables (axi_wstrb)
    // int_clear_request write is only active for one clock
    wire wr = wr_add_data_valid && axi_awready && axi_wready;
    integer byte_index;
    always_ff @ (posedge axi_clk)
    begin
        if (axi_resetn == 1'b0)
        begin
            latch_data[31:0] <= 32'b0;
            status <= 32'b0;
            control <= 32'b0;
            brd <= 32'b0;
//            int_positive <= 32'b0;
//            int_negative <= 32'b0;
//            int_edge_mode <= 32'b0;
//            int_clear_request <= 32'b0;
        end 
        else 
        begin
            if (wr)
            begin
                case (axi_awaddr[3:2])
                    DATA_REG:
                        for (byte_index = 0; byte_index <= 3; byte_index = byte_index+1)
                            if ( axi_wstrb[byte_index] == 1) 
                                latch_data[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                    STATUS_REG:
                        for (byte_index = 0; byte_index <= 3; byte_index = byte_index+1)
                            if (axi_wstrb[byte_index] == 1)
                                status[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                    CONTROL_REG: 
                        for (byte_index = 0; byte_index <= 3; byte_index = byte_index+1)
                            if (axi_wstrb[byte_index] == 1)
                                control[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                    BRD_REG:
                        for (byte_index = 0; byte_index <= 3; byte_index = byte_index+1)
                            if (axi_wstrb[byte_index] == 1)
                                brd[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                        
//                    INT_POSITIVE_REG:
//                        for (byte_index = 0; byte_index <= 3; byte_index = byte_index+1)
//                            if (axi_wstrb[byte_index] == 1) 
//                                int_positive[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
//                    INT_NEGATIVE_REG:
//                        for (byte_index = 0; byte_index <= 3; byte_index = byte_index+1)
//                            if (axi_wstrb[byte_index] == 1)
//                                int_negative[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
//                    INT_EDGE_MODE_REG:
//                        for (byte_index = 0; byte_index <= 3; byte_index = byte_index+1)
//                            if (axi_wstrb[byte_index] == 1)
//                                int_edge_mode[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
//                    INT_STATUS_CLEAR_REG:
//                        for (byte_index = 0; byte_index <= 3; byte_index = byte_index+1)
//                            if (axi_wstrb[byte_index] == 1)
//                                int_clear_request[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                endcase
            end
//            else
//                int_clear_request <= 32'b0;
        end
    end 
    reg [23:0] ibrd;   
    reg [7:0] fbrd;    
    reg enable;
    reg out;
    reg brd_out;
      
    always @(*) begin
        ibrd = brd[31:8];     // Capture upper 24 bits for integer part
        fbrd = brd[7:0];      // Capture lower 8 bits for fractional part
        enable = control[4];  // Capture ENABLE from CONTROL register (bit 5)
        out=(control[5] & brd_out);
    end
    assign CLK_OUT=out;
    // Instantiate the brd module
    brd baud_rate_generator (
        .clk(axi_clk),
        .enable(enable),
        .ibrd(ibrd),
        .fbrd(fbrd),
        .out(brd_out));
    
    // Send write response (axi_bvalid, axi_bresp)
    // - after address is valid (axi_awvalid)
    // - after write data is valid (axi_wvalid)
    // - after this module asserts ready for address handshake (axi_awready)
    // - after this module asserts ready for data handshake (axi_wready)
    // Clear write response valid (axi_bvalid) after one clock
    wire wr_add_data_ready = axi_awready && axi_wready;
    always_ff @ (posedge axi_clk)
    begin
        if (axi_resetn == 1'b0)
        begin
            axi_bvalid  <= 0;
            axi_bresp   <= 2'b0;
        end 
        else
        begin    
            if (wr_add_data_valid && wr_add_data_ready && ~axi_bvalid)
            begin
                axi_bvalid <= 1'b1;
                axi_bresp  <= 2'b0;
            end
            else if (S_AXI_BREADY && axi_bvalid) 
                axi_bvalid <= 1'b0; 
        end
    end   

    // In the first clock (~axi_arready) that the read address is valid
    // - capture the address (axi_araddr)
    // - output ready (axi_arready) for one clock
    reg [C_S_AXI_ADDR_WIDTH-1:0] raddr;
    always_ff @ (posedge axi_clk)
    begin
        if (axi_resetn == 1'b0)
        begin
            axi_arready <= 1'b0;
            raddr <= 32'b0;
        end 
        else
        begin    
            // if valid, pulse ready (axi_rready) for one clock and save address
            if (axi_arvalid && ~axi_arready)
            begin
                axi_arready <= 1'b1;
                raddr  <= axi_araddr;
            end
            else
                axi_arready <= 1'b0;
        end 
    end       
        
    // Update register read data
    // - after this module receives a valid address (axi_arvalid)
    // - after this module asserts ready for address handshake (axi_arready)
    // - before the module asserts the data is valid (~axi_rvalid)
    //   (don't change the data while asserting read data is valid)
    wire rd = axi_arvalid && axi_arready && ~axi_rvalid;
    always_ff @ (posedge axi_clk)
    begin
        if (axi_resetn == 1'b0)
        begin
            axi_rdata <= 32'b0;
        end 
        else
        begin    
            if (rd)
            begin
		// Address decoding for reading registers
		case (raddr[3:2])
		    DATA_REG: 
		        axi_rdata <= {23'b0,rd_data};
		    STATUS_REG:
		        axi_rdata <= status;
		    CONTROL_REG: 
		        axi_rdata <= control;
		    BRD_REG: 
			    axi_rdata <= brd;
//		    INT_POSITIVE_REG:
//			axi_rdata <= int_positive;
//		    INT_NEGATIVE_REG:
//			axi_rdata <= int_negative;
//		    INT_EDGE_MODE_REG:
//			axi_rdata <= int_edge_mode;
//		    INT_STATUS_CLEAR_REG:
//		        axi_rdata <= int_status;
		endcase
            end   
        end
    end    

    // Assert data is valid for reading (axi_rvalid)
    // - after address is valid (axi_arvalid)
    // - after this module asserts ready for address handshake (axi_arready)
    // De-assert data valid (axi_rvalid) 
    // - after master ready handshake is received (axi_rready)
    always_ff @ (posedge axi_clk)
    begin
        if (axi_resetn == 1'b0)
            axi_rvalid <= 1'b0;
        else
        begin
            if (axi_arvalid && axi_arready && ~axi_rvalid)
            begin
                axi_rvalid <= 1'b1;
                axi_rresp <= 2'b0;
            end   
            else if (axi_rvalid && axi_rready)
                axi_rvalid <= 1'b0;
        end
    end    
    reg clear_overflow_request;
    reg write_single_pulse;
    
    edge_detector write_edge_detector (
    .clk(axi_clk),
    .reset(~axi_resetn),
    .req(wr && axi_awaddr[3:2]==DATA_REG ),
    .single_pulse(write_single_pulse)
    
);
    reg data_request; // for output data request
    //read_clear_pulse
    reg read_single_pulse;
    
    edge_detector read_edge_detector (
    .clk(axi_clk),
    .reset(~axi_resetn),
    .req(data_request),
    .single_pulse(read_single_pulse)
    
);
    
    Fifo fifo16x9 (
    .clk(axi_clk),
    .reset(~axi_resetn),
    .wr_data(latch_data[9:0]),
    .wr_request(write_single_pulse),
    .rd_data(rd_data),
    .rd_request(read_single_pulse),
    .empty(empty),
    .full(full),
    .overflow(overflow),
    .clear_overflow_request(clear_overflow_request),
    .wr_index(wr_index),
    .rd_index(rd_index),
    .watermark(watermark)
);



    reg transmit_out;  
//    reg [8:0] rd_data_for_transmitter;
//    assign rd_data_for_transmitter=rd_data;
    
    transmitter Tx (
    .clk(axi_clk),
    .brgen(CLK_OUT), 
    .enable(control[4]) , 
    .reset(~axi_resetn),
    .empty(empty), 
    .size(control[1:0]),
    .parity(control[3:2]), 
    .stop2(control[8]), //if 0 stop1, 1 stop2   
    .data(rd_data),  
    .data_request(data_request), 
    .out_transmitter(transmit_out),
    .IDLE_led(IDLE_led),
    .START_led(START_led),
    .D0_led(D0_led),
    .STOP1_led(STOP1_led)
    
);  
    assign Tx_out=transmit_out;  


  
endmodule

module Fifo(
    input clk,
    input reset,
    input reg [8:0] wr_data,
    input wr_request,
    output reg [8:0] rd_data,
    input rd_request,
    output empty,
    output full,
    output reg overflow,
    input clear_overflow_request,
    output reg [4:0] wr_index,
    output reg [4:0] rd_index,
    output wire [4:0] watermark
);

    // Declare the FIFO memory
    reg [8:0] fifo_mem [15:0]; // 16 x 9-bit FIFO
//    reg [4:0] num_elements; // Number of elements in the FIFO
    
    
    
    

    // Set empty and full flags based on number of elements
    assign empty = (wr_index == rd_index) ? 1'b1 : 1'b0; 
    assign full  = (wr_index[4] ^ rd_index[4]) && (wr_index[3:0] == rd_index[3:0]);

    assign watermark = (16+(wr_index%16) - (rd_index%16))%16;



    
    
    always @(posedge clk) 
    begin 
        if(reset)
        begin
            wr_index<= 5'b0;
            rd_index<= 5'b0;
//            watermark<= 5'b0;
            overflow<= 1'b0;
        end
        else 
        begin
            if (clear_overflow_request)
            begin
                
                overflow<=1'b0;
            
            end
            
            else if ( wr_request && !full)
            
            begin
                
                fifo_mem[wr_index[3:0]]<=wr_data;
                wr_index <= (wr_index + 1) % 32;
            
            end
            
            else if ( wr_request && full)
            
            begin
                overflow<=1'b1;
            
            end
            
            
            else if ( rd_request && !empty)
            
            begin
                rd_data<=fifo_mem[rd_index[3:0]];
                rd_index <= (rd_index + 1) % 32;
            
            end
          end
        end
   
endmodule

module edge_detector (
    input clk,
    input reset,
    input req,

    output reg single_pulse
    
);
    reg previous_req;
    always @(posedge clk) 
    begin 
        if(reset)
        begin
            single_pulse<= 1'b0;
            previous_req<= 1'b0;
        end
        else 
        begin
            previous_req<=req;
            single_pulse<=req&&(~previous_req);
        end
    end
endmodule 
module brd (
    input wire clk,               
    input wire enable,            
    input wire [23:0] ibrd,       
    input wire [7:0] fbrd,       
    output wire out                
);

    // Internal 32-bit values in Q24.8 format
//    reg [31:0] ibrd;              
//    reg [31:0] fbrd;            
    reg [23:0] counter = 0;       
    reg [31:0] main_ibrd_fbrd={ibrd,fbrd};      
    reg [31:0] current_ibrd_fbrd=main_ibrd_fbrd;    
    reg temp_out=clk;  
    

    // Convert integer inputs to fixed-point Q24.8 format
//    initial begin
       
//        ibrd = {ibrd_int[23:0], 8'b0};        
//        fbrd = {24'b0, (fbrd_int * 256) / 100};  
//        main_ibrd_fbrd = ibrd + fbrd;    
//    main_ibrd_fbrd={ibrd,fbrd};    
//    current_ibrd_fbrd=main_ibrd_fbrd;
//    end

    always @(posedge clk) begin
        if (enable) begin
            // Increment counter by 1 in fixed-point
            counter <= counter + 1;

            // Check if counter reaches or exceeds current integer + fractional divisor
            if (counter >= current_ibrd_fbrd[31:8]) begin
                // Toggle the output signal
                temp_out <= ~temp_out;

                // Update the integer divisor with the fractional component
                current_ibrd_fbrd <= current_ibrd_fbrd + main_ibrd_fbrd; // Adjust divisor by adding fractional part
            end
        end else begin
            // If not enabled, reset the counter and current_ibrd
//            counter <= 0;
            temp_out=0;
//            current_ibrd_fbrd <= main_ibrd_fbrd;
        end
    end
    assign out=temp_out;
endmodule


// Transmitter

module transmitter(
    input wire clk,               // Main clock
    input wire brgen,             // Baudrate generator clock
    input wire enable,            // Enable signal
    input wire reset,             // Reset signal
    input wire empty,             // Signal to indicate data availability
    input wire [1:0] size,        // Data size: 00 = 5 bits, 01 = 6 bits, 10 = 7 bits, 11 = 8 bits
    input wire [1:0] parity,      // 2-bit parity field: 00 = off, 01 = even, 10 = odd
    input wire stop2,             // Stop bit control (0 for 1 stop bit, 1 for 2 stop bits)
    input wire [8:0] data,        // Input data (9 bits, MSB ignored if size < 9)
    output reg data_request,      // Request signal for data
    output reg out_transmitter ,
    output reg IDLE_led,
    output reg START_led,
    output reg D0_led,
    output reg STOP1_led               // Serial output
);

    // State encoding
    typedef enum logic [3:0] {
        IDLE = 4'd0,
        START = 4'd1,
        D0 = 4'd2,
        D1 = 4'd3,
        D2 = 4'd4,
        D3 = 4'd5,
        D4 = 4'd6,
        D5 = 4'd7,
        D6 = 4'd8,
        D7 = 4'd9,
        PARITY = 4'd10,
        STOP1 = 4'd11,
        STOP2 = 4'd12
    } state_t;

    state_t state, next_state=IDLE;
    reg [8:0] latched_data;
    reg [2:0] bit_counter;        // Counter for tracking data bits
    reg parity_bit;               // Calculated parity bit
    reg brgen_curr, brgen_prev;   // Registers for edge detection on brgen
    reg brgen_rising;             // Indicates rising edge of brgen
    reg [3:0] counter;            // 4-bit counter to count up to 16
    reg output_16;                // Internal register for 1/16 clock signal
    reg output_16_prev;           // Register to hold the previous value of output_16
    reg output_16_rising;         // Register to indicate rising edge of output_16

    // Calculate parity based on data and parity type
    always @(*) begin
        case (parity)
            2'b00: parity_bit = 1'b0;                             // Parity off
            2'b01: // Even parity
                case (size)
                    2'b00: parity_bit = ^data[4:0];           // 5-bit data (00)
                    2'b01: parity_bit = ^data[5:0];            // 6-bit data (01)
                    2'b10: parity_bit = ^data[6:0];           // 7-bit data (10)
                    2'b11: parity_bit = ^data[7:0];          // 8-bit data (11)
                    default: parity_bit = 1'b0;
                endcase
            2'b10: // Odd parity
                case (size)
                    2'b00: parity_bit = ~(^data[4:0]);              // 5-bit data (00)
                    2'b01: parity_bit = ~(^data[5:0]);             // 6-bit data (01)
                    2'b10: parity_bit = ~(^data[6:0]);              // 7-bit data (10)
                    2'b11: parity_bit = ~(^data[7:0]);              // 8-bit data (11)
                    default: parity_bit = 1'b0;
                endcase
            2'b11: parity_bit = data[8]; 
            default: parity_bit = 1'b0;                           // Default case, unused
        endcase
    end

    // Baudrate counter logic to generate internal output_16
    always @(posedge clk) begin
        brgen_curr <= brgen;                 // Capture current value
        brgen_prev <= brgen_curr;            // Update previous value

        // Detect rising edge: current is 1, previous is 0
        brgen_rising <= (brgen_curr & ~brgen_prev);

        // Increment counter if rising edge is detected
        if (brgen_rising) begin
            if (counter == 4'd15) begin
                output_16 <= 1;              // Set output_16 high when counter reaches 16
                counter <= 4'd0;             // Reset counter
            end else begin
                output_16 <= 0;              // Keep output_16 low if counter is not 16
                counter <= counter + 1;
            end
        end else begin
            output_16 <= 0;                  // Ensure output_16 is low when no rising edge
        end
    end

    // Detect rising edge of output_16
    always @(posedge clk) begin
        output_16_prev <= output_16;                  // Store previous value of output_16
        output_16_rising <= output_16 & ~output_16_prev; // Rising edge detection
    end

    // State transition and output logic, triggered on rising edge of output_16
always @(posedge clk) begin
    if (enable && output_16_rising) begin
        state <= next_state; // Update state on the rising edge of output_16
    end
end

// State transition logic based on current state and conditions
always @(*) begin
    case (state)
        IDLE: begin
            if (!empty) begin
                next_state = START;
            end else begin
                next_state = IDLE;
            end
        end

        START: begin
            next_state = D0;
        end

        D0: begin
            next_state = D1;
        end

        D1: begin
            next_state = D2;
        end

        D2: begin
            next_state = D3;
        end

        D3: begin
            next_state = D4;
        end

        D4: begin
            if (size > 2'b00) next_state = D5;
            else if (parity != 2'b00) next_state = PARITY;
            else next_state = STOP1;
        end

        D5: begin
            if (size > 2'b01) next_state = D6;
            else if (parity != 2'b00) next_state = PARITY;
            else next_state = STOP1;
        end

        D6: begin
            if (size > 2'b10) next_state = D7;
            else if (parity != 2'b00) next_state = PARITY;
            else next_state = STOP1;
        end

        D7: begin
            if (parity != 2'b00) next_state = PARITY;
            else next_state = STOP1;
        end

        PARITY: begin
            next_state = STOP1;
        end

        STOP1: begin
            if (stop2) next_state = STOP2;
            else if (!empty) next_state = START;
            else next_state = IDLE;
        end

        STOP2: begin
            if (!empty) next_state = START;
            else next_state = IDLE;
        end

//        default: begin
//            next_state = IDLE;
//        end
    endcase
end

// Output logic based on the current state
always @(*) begin
    case (state)
        IDLE: begin
            out_transmitter = 1'b1;
            data_request = 1'b0;
            IDLE_led = 1'b1;
//            START_led = 1'b0;
//            D0_led = 1'b0;
//            STOP1_led = 1'b0;
        end

        START: begin
            out_transmitter = 1'b0;
            data_request = 1'b1;
//            IDLE_led = 1'b0;
            START_led = 1'b1;
        end

        D0: begin
//            out_transmitter = 1'b1;
            out_transmitter = data[0];
            D0_led = 1'b1;
        end

        D1: out_transmitter = data[1];
        D2: out_transmitter = data[2];
        D3: out_transmitter = data[3];
        D4: out_transmitter = data[4];
        D5: out_transmitter = data[5];
        D6: out_transmitter = data[6];
        D7: out_transmitter = data[7];

        PARITY: begin
            out_transmitter = parity_bit;
            STOP1_led = 1'b1;
        end

        STOP1: out_transmitter = 1'b1;
        STOP2: out_transmitter = 1'b1;

//        default: out_transmitter = 1'b1;
    endcase
end

endmodule
