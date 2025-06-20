//Designed: Sanidhya Saxena
//Guide: Kuruvilla Varghese

(* keep_hierarchy = "yes" *)
module UART_TOP #(
    parameter CLOCK_RATE = 100000000, // board internal clock
    parameter BAUD_RATE = 9600
)
(
  input clk,
  input rst,

  input RX,
  output TX,

  output [7:0] lcd_reg_uart,

  input [31:0] addr_i,
  input req_i,
  input [1:0] burst_i,
  input write_i,
  input [31:0] wdata_i,
  input [3:0] bstrobe_i,

  output [31:0] rdata_o,
  output reg ready_o,
  output stall_o,
  output reg tlast_o
);

wire rxDone, rxBusy, rxErr;

wire RX_DONE;

assign RX_DONE = rxDone && ~rxBusy && ~rxErr;

//=============UART Register==================
//UART_DVSR     - 0x5C00_0000 (RW)
//UART_TX_DATA  - 0x5C00_0004 (W)
//UART_TX_BUSY  - 0x5C00_0008 (R)
//UART_RX_DATA  - 0x5C00_0010 (R)
//UART_RX_EMPTY  - 0x5C00_0014 (R)


wire [7:0] rdata_rx;
wire rd_uart;
wire tx_wr;
wire rd_dvsr;
wire rd_tx_full, rd_rx_empty;
wire tx_full, rx_empty;

reg [11:0] dvsr;

//==============LEDs=========================
//LED[0] = TX
//LED[1] = RX
//LED[2] = tx_full
//LED[3] = rx_empty
//LED[4] = 0
//LED[5] = 0
//LED[6] = 0
//LED[7] = 0
//

assign lcd_reg_uart = {4'b0000,rx_empty,tx_full,RX,TX};

always @(posedge clk or posedge rst) begin
  if(rst) begin
    dvsr <= 12'd0;
  end
  else if(req_i & write_i) begin
    if(addr_i[7:0] == 8'h00) dvsr <= wdata_i[11:0];
  end
end

assign rdata_o =  rd_dvsr     ? dvsr              :
                  rd_uart     ? rdata_rx          : 
                  rd_tx_full  ? {31'd0,tx_full}   :
                  rd_rx_empty ? {31'd0,rx_empty}  : 32'd0;


assign rd_dvsr      = (req_i & ~write_i) && (addr_i[7:0] == 8'h00);                 //Read DVSR Register
assign tx_wr        = (req_i & write_i & bstrobe_i[0]) && (addr_i[7:0] == 8'h04);   //Write on UART_TX_DATA
assign rd_tx_full   = (req_i & ~write_i) && (addr_i[7:0] == 8'h08);                 //Read TX_FULL Flag
assign rd_uart      = (req_i & ~write_i) && (addr_i[7:0] == 8'h10);                 //Read on UART_RX_DATA
assign rd_rx_empty  = (req_i & ~write_i) && (addr_i[7:0] == 8'h14);                 //Read RD_EMPTY Flag

always @(posedge clk or posedge rst) begin
  if(rst) begin
    ready_o <= 1'b0;
    tlast_o <= 1'b0;
  end
  else if(req_i & ~ready_o) begin
    ready_o <= 1'b1;
    tlast_o <= 1'b1;
  end
  else if(ready_o) begin
    ready_o <= 1'b0;
    tlast_o <= 1'b0;
  end
end

assign stall_o = 1'b0;

(* keep_hierarchy = "yes" *)
Uart #(.CLOCK_RATE(CLOCK_RATE), .BAUD_RATE(BAUD_RATE)) UART (
  .clk(clk),
  .rst(rst),

  //Divisor Baud rate
  .dvsr(dvsr),
  
  //Rx interface
  .rx(RX),
  .rdata_rx(rdata_rx),
  .rd_uart(rd_uart),
  .rx_empty(rx_empty),

  //Tx interface
  .tx(TX),
  .wdata_tx(wdata_i),
  .tx_wr(tx_wr & ~ready_o),
  .tx_full(tx_full)
);

endmodule

