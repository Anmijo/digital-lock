`default_nettype none
// Empty top module

module top (
  // I/O ports
  input  logic hz100, reset,
  input  logic [20:0] pb,
  output logic [7:0] left, right,
         ss7, ss6, ss5, ss4, ss3, ss2, ss1, ss0,
  output logic red, green, blue,

  // UART ports
  output logic [7:0] txdata,
  input  logic [7:0] rxdata,
  output logic txclk, rxclk,
  input  logic txready, rxready
);

  // Your code goes here...
  reg [7:0] strobeReg;

  logic strobe,en;
  logic [4:0] key;

  typedef enum logic [3:0] {
    LS0=0, LS1=1, LS2=2, LS3=3, LS4=4, LS5=5, LS6=6, LS7=7,
    OPEN=8, ALARM=9, INIT=10
  } state_t;


  state_t state;
  logic[31:0] out;

  synckey step2(.clk(hz100), .rst(reset), .in(pb[19:0]), .out(key), .strobe(strobe));
 
  fsm machine(.clk(strobe), .rst(reset), .keyout(key), .seq(out), .state(state));
  assign en = state == INIT ? 1'b1 : 1'b0;

  sequence_sr genSequence(.clk(strobe), .rst(reset),.en(en), .in(key), .out(out));

  display disp(.state(state), .seq(out),.ss({ss7, ss6, ss5, ss4, ss3, ss2, ss1, ss0}), .red(red), .green(green), .blue(blue));

endmodule

// Add more modules down here...
typedef enum logic [3:0] {
  LS0=0, LS1=1, LS2=2, LS3=3, LS4=4, LS5=5, LS6=6, LS7=7,
  OPEN=8, ALARM=9, INIT=10
} state_t;

module display(
    input logic[3:0] state,
    input logic [31:0] seq,
    output logic [63:0] ss,
    output logic red,green,blue
);
    logic en;

    // assign green = state == OPEN;
    // assign red = state == ALARM;

    // assign blue = ((state == LS0) | (state == LS1) | (state == LS2)| (state == LS3) | (state == LS4) | (state == LS5)| (state == LS6) | (state == LS7)) ? 1:0;
    // assign en = state == INIT;
    // assign ss[7] = state == LS7;
    // assign ss[15] = state == LS6;
    // assign ss[23] = state == LS5;
    // assign ss[31] = state == LS4;
    // assign ss[39] = state == LS3;
    // assign ss[47] = state == LS2;
    // assign ss[55] = state == LS1;
    // assign ss[63] = state == LS0;


    // ssdec s0(.enable(en), .in(seq[3:0]), .out(ss[6:0]));
    // ssdec s1(.enable(en), .in(seq[7:4]), .out(ss[14:8]));
    // ssdec s2(.enable(en), .in(seq[11:8]), .out(ss[22:16]));
    // ssdec s3(.enable(en), .in(seq[15:12]), .out(ss[30:24]));
    // ssdec s4(.enable(en), .in(seq[19:16]), .out(ss[38:32]));
    // ssdec s5(.enable(en), .in(seq[23:20]), .out(ss[46:40]));
    // ssdec s6(.enable(en), .in(seq[27:24]), .out(ss[54:48]));
    // ssdec s7(.enable(en), .in(seq[31:28]), .out(ss[62:56]));

    logic [63:0] ss2;
    ssdec s0(.enable(en), .in(seq[3:0]), .out(ss2[6:0]));
    ssdec s1(.enable(en), .in(seq[7:4]), .out(ss2[14:8]));
    ssdec s2(.enable(en), .in(seq[11:8]), .out(ss2[22:16]));
    ssdec s3(.enable(en), .in(seq[15:12]), .out(ss2[30:24]));
    ssdec s4(.enable(en), .in(seq[19:16]), .out(ss2[38:32]));
    ssdec s5(.enable(en), .in(seq[23:20]), .out(ss2[46:40]));
    ssdec s6(.enable(en), .in(seq[27:24]), .out(ss2[54:48]));
    ssdec s7(.enable(en), .in(seq[31:28]), .out(ss2[62:56]));


    always_comb begin
        case (state)
            INIT: begin
                en = 1;
                red = 0;
                green = 0;
                blue = 0;
                ss = ss2;
            end
            OPEN:begin
                en = 0;
                red = 0;
                green = 1;
                blue = 0;   
                ss[63:56] = '0; //0
                ss[55:48] = '0;//0
                ss[47:40] = '0; //0
                ss[39:32] = '0; //0
                ss[31:24] = 8'b00111111; //O
                ss[23:16] =  8'b01110011;//P
                ss[15:8] = 8'b01111001;//E
                ss[7:0] = 8'b01010100; //N
            end
            ALARM:begin
                en = 0;
                red = 1;
                green = 0;
                blue = 0;    
                ss[63:56] = 8'b00111001; //C
                ss[55:48] = 8'b01110111; //A
                ss[47:40] = 8'b00111000; //L
                ss[39:32] = 8'b00111000; //LALARM
                ss[31:24] = '0; //0
                ss[23:16] =  8'b01100111;//9
                ss[15:8] = 8'b00000110;//1
                ss[7:0] = 8'b00000110;//1
            end
            LS0: begin
                blue = 1;
                green = 0;
                red = 0;
                en = 0;
                ss[63:56] = 8'b10000000; //0
                ss[55:48] = '0;//0
                ss[47:40] = '0; //0
                ss[39:32] = '0; //0
                ss[31:24] = '0;  //O
                ss[23:16] =  '0; //0
                ss[15:8] =  '0;//0
                ss[7:0] = '0; //1
            end
            LS1: begin
                blue = 1;
                green = 0;
                red = 0;
                en = 0;
                ss[63:56] = '0; //0
                ss[55:48] = 8'b10000000;//0
                ss[47:40] = '0; //0
                ss[39:32] = '0; //0
                ss[31:24] = '0;  //O
                ss[23:16] =  '0; //0
                ss[15:8] =  '0;//0
                ss[7:0] = '0; //1
            end
            LS2: begin
                blue = 1;
                green = 0;
                red = 0;
                en = 0;          
                ss[63:56] = '0; //0
                ss[55:48] = '0;//0
                ss[47:40] = 8'b10000000; //0
                ss[39:32] = '0; //0
                ss[31:24] = '0;  //O
                ss[23:16] =  '0; //0
                ss[15:8] =  '0;//0
                ss[7:0] = '0; //1
            end
            LS3: begin
                blue = 1;
                green = 0;
                red = 0;
                en = 0;                
                ss[63:56] = '0; //0
                ss[55:48] = '0;//0
                ss[47:40] = '0; //0
                ss[39:32] = 8'b10000000; //1
                ss[31:24] = '0;  //O
                ss[23:16] =  '0; //0
                ss[15:8] =  '0;//0
                ss[7:0] = '0; //0  
            end
            LS4: begin
                blue = 1;
                green = 0;
                red = 0;
                en = 0;           
                ss[63:56] = '0; //0
                ss[55:48] = '0;//0
                ss[47:40] = '0; //0
                ss[39:32] = '0; //0
                ss[31:24] = 8'b10000000;  //1
                ss[23:16] =  '0; //0
                ss[15:8] =  '0;//0
                ss[7:0] = '0; //0  

            end
            LS5: begin
                blue = 1;
                green = 0;
                red = 0;
                en = 0;
                ss[63:56] = '0; //0
                ss[55:48] = '0;//0
                ss[47:40] = '0; //0
                ss[39:32] = '0; //0
                ss[31:24] = '0;  //O
                ss[23:16] =  8'b10000000; //1
                ss[15:8] =  '0;//0
                ss[7:0] = '0; //0 

            end
            LS6: begin
                blue = 1;
                green = 0;
                red = 0;
                en = 0;                
                ss[63:56] = '0; //0
                ss[55:48] = '0;//0
                ss[47:40] = '0; //0
                ss[39:32] = '0; //0
                ss[31:24] = '0;  //O
                ss[23:16] =  '0; //0
                ss[15:8] = 8'b10000000; //0
                ss[7:0] = '0; //1
            end
            LS7: begin
                blue = 1;
                green = 0;
                red = 0;
                en = 0;
                ss[63:56] = '0; //0
                ss[55:48] = '0;//0
                ss[47:40] = '0; //0
                ss[39:32] = '0; //0
                ss[31:24] = '0;  //O
                ss[23:16] =  '0; //0
                ss[15:8] = '0; //0
                ss[7:0] = 8'b10000000; //1
            end
            default: begin
                ss = '0; 
                blue = 0;
                green = 0;
                red = 0;
                en = 0;
            end
        endcase
    end             
    
endmodule


module sequence_sr(
  input logic clk,
  input logic rst,
  input logic en,
  input logic [4:0] in,
  output logic[31:0] out
);

logic temp;

assign temp = in < 5'd16;

always_ff @(posedge clk, posedge rst) begin
    if (rst) begin
        out <= 0;
    end else if (en & temp) begin
        out <= {out[27:0],in[3:0]};
    end else begin
      out <= out;
    end
end

    
endmodule



module fsm(
input logic clk,
input logic rst,
input logic [4:0] keyout,
input logic [31:0] seq,
output state_t state
);

state_t current_state, next_state;

always_ff @(posedge clk, posedge rst)
    if (rst)
        current_state <= INIT;
    else
        current_state <= next_state;

always_comb begin
    case (current_state)
        INIT:
          if (5'b10000== keyout[4:0])
              next_state = LS0;
          else
              next_state = INIT;
        LS0:
            if (seq[31:28] == keyout[3:0])
                next_state = LS1;
            else
                next_state = ALARM;
        LS1:
            if (seq[27:24] == keyout[3:0])
                next_state = LS2;
            else
                next_state = ALARM;
        LS2:
            if (seq[23:20] == keyout[3:0])
                next_state = LS3;
            else
                next_state = ALARM;
        LS3:
            if (seq[19:16] == keyout[3:0])
                next_state = LS4;
            else
                next_state = ALARM;
        LS4:
            if (seq[15:12] == keyout[3:0])
                next_state = LS5;
            else
                next_state = ALARM;
        LS5:
            if (seq[11:8] == keyout[3:0])
                next_state = LS6;
            else
                next_state = ALARM;
        LS6:
            if (seq[7:4] == keyout[3:0])
                next_state = LS7;
            else
                next_state = ALARM;        
        LS7:
            if (seq[3:0] == keyout[3:0])
                next_state = OPEN;
            else
                next_state = ALARM;
        OPEN:
            if (5'b10000 == keyout[4:0])
                next_state = LS0;
            else
                next_state = OPEN;
        ALARM:
                next_state = ALARM;
        default:
            next_state = INIT;
    endcase
end

assign state = current_state;


endmodule





module synckey(
  input logic [19:0] in,
  input logic clk,
  input logic rst,
  output logic [4:0] out,
  output logic strobe
);
  logic orIn;
  logic synchronizer_ff1;
  logic delayedClock_ff2;
  
  logic [4:0]out1;

  assign out1 = in[19] ? 5'd19 :
                in[18] ? 5'd18 :
                in[17] ? 5'd17 :
                in[16] ? 5'd16 :
                in[15] ? 5'd15 :
                in[14] ? 5'd14 :
                in[13] ? 5'd13 :
                in[12] ? 5'd12 :
                in[11] ? 5'd11 :
                in[10] ? 5'd10 :
                in[9] ? 5'd9 :
                in[8] ? 5'd8 :
                in[7] ? 5'd7 :
                in[6] ? 5'd6 :
                in[5] ? 5'd5 :
                in[4] ? 5'd4 :
                in[3] ? 5'd3 :
                in[2] ? 5'd2 :
                in[1] ? 5'd1 :
                 5'd00;
    
  assign orIn = |in;

  always_ff @(posedge clk, posedge rst) begin
      if (rst) begin
        synchronizer_ff1  <= 0;
        delayedClock_ff2 <= 0;
        out<=0;
      end else begin
        synchronizer_ff1  <= orIn;
        delayedClock_ff2 <= synchronizer_ff1;
        out<= out1;
      end
  end

  assign strobe = delayedClock_ff2;

endmodule


module ssdec (
    input logic [3:0] in,
    input logic enable,
    output logic [6:0] out
);

always_comb begin
    if (enable == 1'b1) begin     
        case(in[3:0])
            4'b0000: begin out=7'b0111111; end //0
            4'b0001: begin out=7'b0000110; end //1
            4'b0010: begin out=7'b1011011; end //2
            4'b0011: begin out=7'b1001111; end //3
            4'b0100: begin out=7'b1100110; end //4
            4'b0101: begin out=7'b1101101; end //5
            4'b0110: begin out=7'b1111101; end //6
            4'b0111: begin out=7'b0000111; end //7
            4'b1000: begin out=7'b1111111; end //8
            4'b1001: begin out=7'b1100111; end //9
            4'b1010: begin out=7'b1110111; end //A
            4'b1011: begin out=7'b1111100; end //B
            4'b1100: begin out=7'b0111001; end //C
            4'b1101: begin out=7'b1011110; end //D
            4'b1110: begin out=7'b1111001; end //E
            4'b1111: begin out=7'b1110001; end //F

            default: begin out=7'b0000000; end
        endcase
    end else begin
        out = 7'b0;
    end
 end


endmodule
