// Top level file for GPIO example for Xilinx XUP Blackboard rev D 
// (gpio_system_top.sv)
// Jason Losh
//
// 32-bit GPIO port:
//   GPIO port for IP module tied to GPIO [23:0]
//   INTR debug output for IP module tied to RGB0[1]
//   AXI4-lite aperature memory offset is 0x00000000

module gpio_system_top (
    input CLK100,
    output [9:0] LED,       // RGB1, RGB0, LED 9..0 placed from left to right
    output [2:0] RGB0,      
    output [2:0] RGB1,
    output [3:0] SS_ANODE,   // Anodes 3..0 placed from left to right
    output [7:0] SS_CATHODE, // Bit order: DP, G, F, E, D, C, B, A
    input [11:0] SW,         // SWs 11..0 placed from left to right
    input [3:0] PB,          // PBs 3..0 placed from left to right
    inout [23:0] GPIO,       // PMODA-C 1P, 1N, ... 3P, 3N order
    output [3:0] SERVO,      // Servo outputs
    output PDM_SPEAKER,      // PDM signals for mic and speaker
    input PDM_MIC_DATA,      
    output PDM_MIC_CLK,
    output ESP32_UART1_TXD,  // WiFi/Bluetooth serial interface 1
    input ESP32_UART1_RXD,
    output IMU_SCLK,         // IMU spi clk
    output IMU_SDI,          // IMU spi data input
    input IMU_SDO_AG,        // IMU spi data output (accel/gyro)
    input IMU_SDO_M,         // IMU spi data output (mag)
    output IMU_CS_AG,        // IMU cs (accel/gyro) 
    output IMU_CS_M,         // IMU cs (mag)
    input IMU_DRDY_M,        // IMU data ready (mag)
    input IMU_INT1_AG,       // IMU interrupt (accel/gyro)
    input IMU_INT_M,         // IMU interrupt (mag)
    output IMU_DEN_AG,       // IMU data enable (accel/gyro)
    inout [14:0]DDR_addr,
    inout [2:0]DDR_ba,
    inout DDR_cas_n,
    inout DDR_ck_n,
    inout DDR_ck_p,
    inout DDR_cke,
    inout DDR_cs_n,
    inout [3:0]DDR_dm,
    inout [31:0]DDR_dq,
    inout [3:0]DDR_dqs_n,
    inout [3:0]DDR_dqs_p,
    inout DDR_odt,
    inout DDR_ras_n,
    inout DDR_reset_n,
    inout DDR_we_n,
    inout FIXED_IO_ddr_vrn,
    inout FIXED_IO_ddr_vrp,
    inout [53:0]FIXED_IO_mio,
    inout FIXED_IO_ps_clk,
    inout FIXED_IO_ps_porb,
    inout FIXED_IO_ps_srstb
    );
     
    // Terminate all of the unused outputs or i/o's
    // assign LED = 10'b0000000000;
//    assign RGB0 = 3'b000;
//    assign RGB1 = 3'b000;
    assign SS_ANODE = 4'b0000;
    assign SS_CATHODE = 8'b11100111;
//    assign GPIO = 24'bzzzzzzzzzzzzzzzzzzzzzzzz;
//    assign GPIO[15:0] = 16'bzzzzzzzzzzzzzzzz;
//    assign GPIO[23:20] = 4'bzzzzz;
//    assign GPIO[18] = 1'bz;
    assign SERVO = 4'b0000;
    assign PDM_SPEAKER = 1'b0;
    assign PDM_MIC_CLK = 1'b0;
    assign ESP32_UART1_TXD = 1'b0;
    assign IMU_SCLK = 1'b0;
    assign IMU_SDI = 1'b0;
    assign IMU_CS_AG = 1'b1;
    assign IMU_CS_M = 1'b1;
    assign IMU_DEN_AG = 1'b0;

    // Tie gpio to PMOD connectors
    wire [31:0] gpio_data_in;
    wire [31:0] gpio_data_out;
    wire [31:0] gpio_data_oe;
    
    
    wire CLK_OUT;
    wire Tx_out;
    // pin control
    genvar j;
    for (j = 2; j < 22; j = j + 1)
        assign GPIO[j] = gpio_data_oe[j] ? gpio_data_out[j] : 1'bz;
    assign gpio_data_in = {8'b0, GPIO[23:2]};
    
        
    // Tie intr output to RGB0 green LED
    wire intr;
//    assign RGB0 = {1'b0, intr, 1'b0};

    // Instantiate system wrapper
//    reg CLK_OUT;
    assign GPIO[1]=CLK_OUT; //baudrate
    assign GPIO[0]=Tx_out; //transmit
    
    wire clk = CLK100;
    reg [8:0] rd_data;
    reg [4:0] wr_index;
    reg [4:0] rd_index;
    wire [4:0] watermark;
    wire empty;
    wire full;
    reg overflow;
    wire IDLE_led;
    wire START_led;
    wire D0_led;
    wire STOP1_led;
    
    wire [1:0] mode = SW[11:10];
    reg [9:0] led_driver1;
    always @ (mode)
    begin
         case (mode)
              2'b00: led_driver1 ={rd_index,wr_index} ;
              2'b01: led_driver1 = {full,empty,overflow,2'b0,watermark};
              2'b10: led_driver1 = {1'b0,rd_data};
              2'b11: led_driver1 = {6'b0,STOP1_led,D0_led,START_led,IDLE_led};

          endcase
    end   
    assign LED=led_driver1; 
//    parameter RED = 3'b001;
//    parameter GREEN = 3'b010;
//    assign RGB0 = full ? RED : 3'b000;
//    assign RGB1 = empty ? GREEN : 3'b000;
    
    
    system_wrapper system (
        .DDR_addr(DDR_addr),
        .DDR_ba(DDR_ba),
        .DDR_cas_n(DDR_cas_n),
        .DDR_ck_n(DDR_ck_n),
        .DDR_ck_p(DDR_ck_p),
        .DDR_cke(DDR_cke),
        .DDR_cs_n(DDR_cs_n),
        .DDR_dm(DDR_dm),
        .DDR_dq(DDR_dq),
        .DDR_dqs_n(DDR_dqs_n),
        .DDR_dqs_p(DDR_dqs_p),
        .DDR_odt(DDR_odt),
        .DDR_ras_n(DDR_ras_n),
        .DDR_reset_n(DDR_reset_n),
        .DDR_we_n(DDR_we_n),
        .FIXED_IO_ddr_vrn(FIXED_IO_ddr_vrn),
        .FIXED_IO_ddr_vrp(FIXED_IO_ddr_vrp),
        .FIXED_IO_mio(FIXED_IO_mio),
        .FIXED_IO_ps_clk(FIXED_IO_ps_clk),
        .FIXED_IO_ps_porb(FIXED_IO_ps_porb),
        .FIXED_IO_ps_srstb(FIXED_IO_ps_srstb),
        .gpio_data_in(gpio_data_in),
        .gpio_data_out(gpio_data_out),
        .gpio_data_oe(gpio_data_oe),
        .IDLE_led(IDLE_led),
        .START_led(START_led),
        .D0_led(D0_led),
        .STOP1_led(STOP1_led),
        .Tx_out(Tx_out),
        .CLK_OUT(CLK_OUT),
        .intr(intr),
        .empty(empty),
        .full(full),
        .overflow(overflow),
        .rd_data(rd_data),
        .rd_index(rd_index),
        .watermark(watermark),
        .wr_index(wr_index));
        

endmodule
