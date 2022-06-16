module testbench();
  logic        clk, reset;
  logic        left, right, lc, lb, la, ra, rb, rc;
  logic [5:0]  expected;
  logic [31:0] vectornum, errors;
  logic [7:0]  testvectors[10000:0];
// инстанцировать тестируемое устройство
lab2_task1 dut(clk, reset, left, right, lc, lb, la, ra, rb, rc);
// generate clock
always begin
    clk=1; #5; clk=0; #5;
  end
// на старте теста, загрузите вектора и запустите сброс
initial
  begin
    $readmemb("thunderbird.tv", testvectors);
    vectornum = 0; errors = 0; reset = 1; #22; reset = 0;
  end
// применение тестовых векторов по нарастающему фронту тактового сигнала
always @(posedge clk)
  begin
    #1; {left, right, expected} = testvectors[vectornum];
  end
always @(negedge clk)
  if (~reset) begin    // skip during reset
    if ({lc, lb, la, ra, rb, rc} !== expected) begin // check result
      $display("Error: inputs = %b", {left, right});
      $display(" outputs = %b %b %b %b %b %b (%b expected)",
        lc, lb, la, ra, rb, rc, expected);
      errors = errors + 1;
end
    vectornum = vectornum + 1;
    if (testvectors[vectornum] === 8'bx) begin
      $display("%d tests completed with %d errors", vectornum, errors);
$stop; end
  end
endmodule