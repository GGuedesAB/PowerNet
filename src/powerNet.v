module powerNet();

  // declarando variáveis
  reg clk, rst;
  reg [55:0] valor1, valor2;
  wire [55:0] soma;

  reg [55:0] pc_addr;
  reg [15:0] in_se_data;
  reg [55:0] mux21_data1, mux21_data2, mux31_data1, mux31_data2, mux31_data3;
  reg       sel_mux21;
  reg [1:0] sel_mux31;
  reg [7:0] addr, addr_dm;
  reg mem_write, mem_read;
  reg [55:0] write_dm_data;
  reg reg_write;

  reg[3:0] reg_dst, reg_src;
  reg [55:0] in_rb_data;

  wire [55:0] out_rb_data1, out_rb_data2;

  wire [55:0] out_se_data;

  wire      [55:0] out_dm_data;
  wire      [55:0] out_inst;
  wire      [55:0] out_mux21_data;
  wire      [55:0] out_mux31_data;
  wire      [55:0] out_pc;
  // Instancia modulos e unidades funcionais

  add somador(.a(valor1), .b(valor2), .sum(soma));

  regs_bank rbank(.clk(clk), .reg1(reg_dst[3:0]), .reg2(reg_src[3:0]), .in_data(in_rb_data[55:0]), .reg_write(reg_write), .data1(out_rb_data1[55:0]), .data2(out_rb_data2[55:0]));

  sign_extend sign_ext(.in(in_se_data), .out(out_se_data));

  data_memory data_mem(.clk(clk), .addr(addr_dm), .mem_write(mem_write), .mem_read(mem_read), .write_data(write_dm_data), .out_data(out_dm_data));

  inst_memory inst_mem(.clk(clk), .addr(addr), .out_inst(out_inst));

  mux21 mux_21(.mux21_data1(mux21_data1), .mux21_data2(mux21_data2), .sel_mux21(sel_mux21), .out_mux21_data(out_mux21_data));

  mux31 mux_31(.mux31_data1(mux31_data1), .mux31_data2(mux31_data2), .mux31_data3(mux31_data3), .sel_mux31(sel_mux31), .out_mux31_data(out_mux31_data));

  program_counter pc(.clk(clk), .rst(rst), .pc_addr(pc_addr), .out_pc(out_pc));

  // inicio simulaçao

  initial begin
    $dumpfile("powerNet.vcd");
    $dumpvars;

    // intervalos de tempo silaçao de cada unidade funcional
    #0 clk <= 0;
    #0 rst = 0;
    #0 pc_addr = 'h0;

    // Add
    #1 valor1 = 56'd3;
    #1 valor2 = 56'd5;

    // extensor de sinal
    #2 in_se_data = 16'hFFFA;

    // mux21
    #3 mux21_data1 = 56'd10;
    #3 sel_mux21 = 2'b00;
    #4 mux21_data2 = 56'd15;
    #4 sel_mux21 = 2'b01;

    // mux31
    #5 mux31_data1 = 56'd90;
    #5 sel_mux31 = 2'b00;
    #6 mux31_data2 = 56'd55;
    #6 sel_mux31 = 2'b01;
    #7 mux31_data3 = 56'd32;
    #7 sel_mux31 = 2'b10;

    // pc
    #8 pc_addr = 'hA;
    #9 rst = 'b1;

    // Memória de instruções
    #10 addr = 8'd100;

    // memória de dados - escrita
    #11 addr_dm = 8'd250;
    #11 mem_write = 1'b1;
    #11 write_dm_data = 56'd45;

    // memória de dados - leitura
    #12 mem_write = 1'b0;
    #12 mem_read = 1'b1;
    #13 mem_read = 1'b0;

    // banco de registradores - leitura
    #14 reg_dst = 4'b101;
    #14 reg_src = 4'b100;

    // banco de registradores - escrita
    #15 reg_write = 1'b1;
    #15 in_rb_data = 56'd45;

    // fim simulacao

    #17 $finish;

  end

  // comportamento clock
  always #1 clk = ~clk;

endmodule

module add(.a(valor1), .b(valor2), .sum(soma));
// declaracao parametros
  input       [55:0] valor1;
  input       [55:0] valor2;
  output wire [55:0] soma;

  assign soma = (valor1 + valor2);

endmodule

module regs_bank(.clk(clk), .reg1(reg_dst), .reg2(reg_src), .in_data(in_rb_data), .reg_write(reg_write), .data1(out_rb_data1), .data2(out_rb_data2));
  // declaracao parametros
  input               clk;
  input      [3:0]    reg_dst;
  input      [3:0]    reg_src;
  input      [55:0]   in_rb_data;
  input               reg_write;
  output reg [55:0]   out_rb_data1;
  output reg [55:0]   out_rb_data2;

  // declaracao dos registradores do banco
  reg [55:0] registers[0:10];

  // zerando os registradores por boa prática
  initial begin
    registers[0] <= 56'h0;
    registers[1] <= 56'h0;
    registers[2] <= 56'h0;
    registers[3] <= 56'h0;
    registers[4] <= 56'h0;
    registers[5] <= 56'h0;
    registers[6] <= 56'h0;
    registers[7] <= 56'h0;
    registers[8] <= 56'h0;
    registers[9] <= 56'h0;
    registers[10] <= 56'h0;
  end

  // caso seja para escrever no registrador
  always @ (posedge clk) begin
    if(reg_write) begin
      registers[reg_dst] <= in_rb_data;
    end
  end
  // feito na descida do clock para ser possível escrever e dar o output em um
  // mesmo ciclo.
  always @ (negedge clk) begin
    out_rb_data1 <= registers[reg_dst];
    out_rb_data2 <= registers[reg_src];
  end

endmodule

module sign_extend(.in(in_se_data), .out(out_se_data));
  // declaracao parametros
  input    [15:0]   in_se_data;
  output   [55:0]   out_se_data;

  assign out_se_data = { {40{in_se_data[15]}}, in_se_data};

endmodule

module data_memory(.clk(clk), .addr(addr_m), .mem_write(mem_write), .mem_read(mem_read), .write_data(write_dm_data), .out_data(out_dm_data));
  // declaracao parametros
  input wire            clk;
  input wire            mem_read;
  input wire            mem_write;
  input wire  [7:0]     addr_m;
  input wire  [55:0]    write_dm_data;
  output wire [55:0]    out_dm_data;
  // criando a memória de dados
  reg         [55:0]    d_mem[255:0];

  integer i;
//  initial begin
//    $readmemh("data_mem.txt", d_mem);
//  end
  initial begin
    $readmemh("data_mem.txt", d_mem);
  end

  always @ (posedge clk) begin
    if (mem_write) begin
      d_mem[addr_m] <= write_dm_data;
    end
    // explicitando o refresh para o compilador
    else begin
      d_mem[addr_m] <= d_mem[addr_m];
    end
  end

  // sempre dá output da memória no local desejado (addr_m)
  assign out_dm_data = d_mem[addr_m];

endmodule

module inst_memory(.clk(clk), .addr(addr), .out_inst(out_inst));
  // declarando parametros
  input wire          clk;
  input wire   [7:0]  addr;
  output wire  [55:0] out_inst;

  // criando a memória de instrucoes
  reg          [55:0] inst_mem[0:255];

  // fazendo a leitura do arquivo com as intruções
  initial begin
    $readmemh("inst_mem.txt",inst_mem);
  end

  // dando output da proxima instrucao
  assign out_inst =   inst_mem[addr];

endmodule

module mux21(.mux21_data1(mux21_data1), .mux21_data2(mux21_data2), .sel_mux21(sel_mux21), .out_mux21_data(out_mux21_data));
  // declarando parametros
  input        [55:0]       mux21_data1;
  input        [55:0]       mux21_data2;
  input                     sel_mux21;
  output wire  [55:0]       out_mux21_data;

  // output na linha selecionada
  assign out_mux21_data = (sel_mux21) ? mux21_data2 : mux21_data1;

endmodule

module mux31(.mux31_data1(mux31_data1), .mux31_data2(mux31_data2), .mux31_data3(mux31_data3), .sel_mux31(sel_mux31), .out_mux31_data(out_mux31_data));
  // declarando parametros
  input         [55:0]    mux31_data1;
  input         [55:0]    mux31_data2;
  input         [55:0]    mux31_data3;
  input         [1:0]     sel_mux31;
  output reg   [55:0]    out_mux31_data;

  // selecionando a linha desejada
  always @ (sel_mux31 or mux31_data1 or mux31_data2 or mux31_data3) begin
    case(sel_mux31)
      2'b00: out_mux31_data = mux31_data1;
      2'b01: out_mux31_data = mux31_data2;
      2'b10: out_mux31_data = mux31_data3;
    endcase
  end

endmodule

module program_counter(.clk(clk), .rst(rst), .pc_addr(pc_addr), .out_pc(out_pc));
  // declarando parametros
  input                   clk;
  input                   rst;
  input      [55:0]       pc_addr;
  output reg [55:0]       out_pc;

  // zerando pc inicialmente por boa pratica
  initial begin
    out_pc <= 56'h0;
  end

  // se resetar, zere o PC, senão some 8 a ele.
  always @ (posedge clk) begin
    if(rst == 1)
      out_pc <= 'h0;
    else // somente d'a output na entrada pois a soma 'e feita pelo somador

      out_pc <= pc_addr;
  end

endmodule
