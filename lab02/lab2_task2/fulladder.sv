 module lab2_task2(input  logic clk, reset,
                     input  logic n, s, e, w,
                     output logic win, die);
logic sword1, sword2;

roomfsm room(clk, reset, n, s, e, w, sword1, sword2, win, die);

swordfsm sword(clk, reset, sword2, sword1);
					
endmodule

module roomfsm(input logic clk, reset,
	       input logic n, s, e, w, sword_in,
               output logic sword_out, win, die);

  typedef enum logic [2:0] {CAVE, TUNNEL, RIVER, SWORD, DRAGON, GRAVE, WINROOM, TRASH} statetype;
  statetype state, nextstate;

  always @(posedge clk, posedge reset)
    begin
    	if (reset) 	state <= CAVE;
        else		state <= nextstate;
    end

  always_comb
    case (state)
      CAVE: if (e) nextstate = TUNNEL;
			 else if (n) nextstate = TRASH;
          else   nextstate = CAVE;
      TUNNEL: if (s) nextstate = RIVER;
          else if (w) nextstate = CAVE;
          else nextstate = TUNNEL;
      RIVER: if (w) nextstate = SWORD;
          else if (e) nextstate = DRAGON;
			 else nextstate = RIVER;
      SWORD: if (e) nextstate = RIVER;
          else nextstate = SWORD;
      DRAGON: if (sword_in) nextstate = WINROOM;
          else nextstate = GRAVE;
      GRAVE: nextstate = GRAVE;
      WINROOM: nextstate = WINROOM;
		TRASH: if (s) nextstate = WINROOM;
		       else nextstate = SWORD;
      default: nextstate = CAVE;
    endcase

  //changin state values
  assign sword_out = (state == SWORD);
  assign win = (state == WINROOM);
  assign die = (state == GRAVE);

endmodule


module swordfsm(input logic clk, reset,
		input logic sword_in,
		output logic sword_out);

  logic state, nextstate;

  always @(posedge clk, posedge reset)
    begin
    	if (reset) 	state <= 0;
        else		state <= nextstate;
    end


   always_comb
     case (state)
       0: if (sword_in) nextstate = 1;
          else nextstate = 0;
       1: nextstate = 1;
     endcase

   assign sword_out = (state == 1);
endmodule