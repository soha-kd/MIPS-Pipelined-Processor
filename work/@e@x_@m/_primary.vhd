library verilog;
use verilog.vl_types.all;
entity EX_M is
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        ALUResult_Ex    : in     vl_logic_vector(31 downto 0);
        ALUResult_M     : out    vl_logic_vector(31 downto 0);
        WriteData_Ex    : in     vl_logic_vector(31 downto 0);
        WriteData_M     : out    vl_logic_vector(31 downto 0);
        writereg_Ex     : in     vl_logic_vector(4 downto 0);
        writereg_M      : out    vl_logic_vector(4 downto 0);
        RegWrite_Ex     : in     vl_logic;
        RegWrite_M      : out    vl_logic;
        MemtoReg_Ex     : in     vl_logic;
        MemtoReg_M      : out    vl_logic;
        MemWrite_Ex     : in     vl_logic;
        MemWrite_M      : out    vl_logic
    );
end EX_M;
