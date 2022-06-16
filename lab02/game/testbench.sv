module testbench(); 
  logic        clk, reset;
  logic        n, s, e, w, win, die, winexpected, dieexpected;
  logic [31:0] vectornum, errors;
  logic [5:0]  testvectors[10000:0];
  logic [6:0]  hash;

  lab2_task2  dut(clk, reset, n, s, e, w, win, die); 

  always 
    begin
      clk=1; #5; clk=0; #5; 
    end 

  initial 
    begin
      $readmemb("adventuregame.tv", testvectors); 
      vectornum = 0; errors = 0; hash = 0; reset = 1; #22; reset = 0; #70; reset = 1; #10; reset = 0; #70; reset = 1; #10; reset = 0;
    end 

  
  always @(posedge clk) 
    begin
      #1; {n, s, e, w, winexpected, dieexpected} = testvectors[vectornum]; 
    end 

  always @(negedge clk) 
    if (~reset) begin  
      if (win !== winexpected || die !== dieexpected) begin
        $display("Error: inputs = %b", {n, s, e, w});
        $display(" state = %b", dut.room.state);
        $display(" outputs = %b %b (%b %b expected)", 
                 win, die, winexpected, dieexpected); 
        errors = errors + 1; 
      end
      hash = hash ^ {win, die};
      hash = {hash[5:0], hash[6]^hash[5]};
      vectornum = vectornum + 1;
      if (testvectors[vectornum] === 6'bx) begin 
        $display("%d tests completed with %d errors", vectornum, errors); 
        $display("hash: %h", hash);
        $stop; 
      end 
    end 
endmodule 