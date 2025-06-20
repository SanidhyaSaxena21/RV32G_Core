//Designed: Sanidhya Saxena
//Guide: Kuruvilla Varghese

(* keep_hierarchy = "yes" *)
module Uart  #(
    parameter CLOCK_RATE = 100000000, // board internal clock
    parameter BAUD_RATE = 9600
)(
    input wire clk,
    input rst,

    //Divisor 
    input [11:0] dvsr,

    // rx interface
    input wire rx,
    output wire [7:0] rdata_rx,
    input rd_uart,
    output rx_empty,

    // tx interface
    output wire tx,
    input wire [7:0] wdata_tx,
    input tx_wr,
    output tx_full
);

wire rxClk;
wire txClk;

wire rxDone, rxBusy, rxErr;
wire tx_fifo_empty, txEn, txDone, txBusy;

wire [7:0] out; //From RX to FIFO
wire [7:0] in;  //From FIFO to TX

(* keep_hierarchy = "yes" *)
BaudRateGenerator #(
    .CLOCK_RATE(CLOCK_RATE),
    .BAUD_RATE(BAUD_RATE)
) generatorInst (
    .clk(clk),
    .dvsr(dvsr),
    .rxClk(rxClk),
    .txClk(txClk)
);

(* keep_hierarchy = "yes" *)
Uart8Receiver rxInst (
    .clk(rxClk),
    .en(1'b1),
    .in(rx),
    .out(out),
    .done(rxDone),
    .busy(rxBusy),
    .err(rxErr)
);

(* keep_hierarchy = "yes" *)
Sync_FIFO RX_FIFO (
  .wr_clk(rxClk),
  .rd_clk(Clk),
  .rst(rst),
  .buf_in(out),
  .buf_out(rdata_rx),
  .wr_en(rxDone & ~rxBusy & ~rxErr),
  .rd_en(rd_uart),
  .buf_empty(rx_empty),
  .buf_full()
);

(* keep_hierarchy = "yes" *)
Uart8Transmitter txInst (
    .clk(txClk),
    .rst(rst),
    .en(1'b1),
    .start(~tx_fifo_empty && ~txDone),
    .in(in),
    .out(tx),
    .done(txDone),
    .busy(txBusy)
);

(* keep_hierarchy = "yes" *)
Sync_FIFO TX_FIFO (
  .wr_clk(clk),
  .rd_clk(txClk),
  .rst(rst),
  .buf_in(wdata_tx),
  .buf_out(in),
  .wr_en(tx_wr),
  .rd_en(txDone),
  .buf_empty(tx_fifo_empty),
  .buf_full(tx_full)
);

endmodule
