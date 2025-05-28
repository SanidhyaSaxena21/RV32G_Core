//Designer: Sanidhya Saxena
//Guide: Kuruvilla Varghese
//

(* keep_hierarchy = "yes" *)
module Interconnect_Wrapper (

  //RISC-V Core
  input         m_req,
  input [31:0]  m_addr,
  input [1:0]   m_burst,
  input         m_write,
  input [31:0]  m_write_data,
  input [3:0]   m_bstrobe,

  output [31:0] m_rdata,
  output        m_ready,
  output        m_stall,
  output        m_tlast,

  //Slave 0 (RAM)
  output reg         s0_req,
  output reg [31:0]  s0_addr,
  output reg [1:0]   s0_burst,
  output reg         s0_write,
  output reg [31:0]  s0_write_data,
  output reg [3:0]   s0_bstrobe,

  input [31:0] s0_rdata,
  input        s0_ready,
  input        s0_stall,
  input        s0_tlast,


  //Slave 1 (CLINT)
  output reg         s1_req,
  output reg [31:0]  s1_addr,
  output reg [1:0]   s1_burst,
  output reg         s1_write,
  output reg [31:0]  s1_write_data,
  output reg [3:0]   s1_bstrobe,

  input [31:0] s1_rdata,
  input        s1_ready,
  input        s1_stall,
  input        s1_tlast,

  //Slave 2 (UART)
  output reg         s2_req,
  output reg [31:0]  s2_addr,
  output reg [1:0]   s2_burst,
  output reg         s2_write,
  output reg [31:0]  s2_write_data,
  output reg [3:0]   s2_bstrobe,

  input [31:0] s2_rdata,
  input        s2_ready,
  input        s2_stall,
  input        s2_tlast,

  //Slave 3  (SPI)
  output reg         s3_req,
  output reg [31:0]  s3_addr,
  output reg [1:0]   s3_burst,
  output reg         s3_write,
  output reg [31:0]  s3_write_data,
  output reg [3:0]   s3_bstrobe,

  input [31:0] s3_rdata,
  input        s3_ready,
  input        s3_stall,
  input        s3_tlast
);

wire s0_grant,s1_grant, s2_grant, s3_grant;

(* keep_hierarchy = "yes" *)
arbiter slave_arbiter (
  .m_addr(m_addr),
  .s0_grant(s0_grant),
  .s1_grant(s1_grant),
  .s2_grant(s2_grant),
  .s3_grant(s3_grant)
);

always @(*) begin
  // Default: all slaves disabled
  s0_req        = 0;
  s0_addr       = 0;
  s0_burst      = 0;
  s0_write      = 0;
  s0_write_data = 0;
  s0_bstrobe    = 0;

  s1_req        = 0;
  s1_addr       = 0;
  s1_burst      = 0;
  s1_write      = 0;
  s1_write_data = 0;
  s1_bstrobe    = 0;

  s2_req        = 0;
  s2_addr       = 0;
  s2_burst      = 0;
  s2_write      = 0;
  s2_write_data = 0;
  s2_bstrobe    = 0;

  s3_req        = 0;
  s3_addr       = 0;
  s3_burst      = 0;
  s3_write      = 0;
  s3_write_data = 0;
  s3_bstrobe    = 0;

  // Master-to-slave routing based on grant
  if (s0_grant) begin
    s0_req        = m_req;
    s0_addr       = m_addr;
    s0_burst      = m_burst;
    s0_write      = m_write;
    s0_write_data = m_write_data;
    s0_bstrobe    = m_bstrobe;
  end
  else if (s1_grant) begin
    s1_req        = m_req;
    s1_addr       = m_addr;
    s1_burst      = m_burst;
    s1_write      = m_write;
    s1_write_data = m_write_data;
    s1_bstrobe    = m_bstrobe;
  end
  else if (s2_grant) begin
    s2_req        = m_req;
    s2_addr       = m_addr;
    s2_burst      = m_burst;
    s2_write      = m_write;
    s2_write_data = m_write_data;
    s2_bstrobe    = m_bstrobe;
  end
  else if (s3_grant) begin
    s3_req        = m_req;
    s3_addr       = m_addr;
    s3_burst      = m_burst;
    s3_write      = m_write;
    s3_write_data = m_write_data;
    s3_bstrobe    = m_bstrobe;
  end
end

assign m_rdata = s0_grant ? s0_rdata :
                 s1_grant ? s1_rdata :
                 s2_grant ? s2_rdata :
                 s3_grant ? s3_rdata : 32'b0;

assign m_ready = s0_grant ? s0_ready :
                 s1_grant ? s1_ready :
                 s2_grant ? s2_ready :
                 s3_grant ? s3_ready : 1'b0;

assign m_stall = s0_grant ? s0_stall :
                 s1_grant ? s1_stall :
                 s2_grant ? s2_stall :
                 s3_grant ? s3_stall : 1'b0;

assign m_tlast = s0_grant ? s0_tlast :
                 s1_grant ? s1_tlast :
                 s2_grant ? s2_tlast :
                 s3_grant ? s3_tlast : 1'b0;



endmodule

