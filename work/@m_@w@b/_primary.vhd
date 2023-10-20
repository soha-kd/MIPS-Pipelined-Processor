library verilog;
use verilog.vl_types.all;
entity M_WB is
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        ReadData_M      : in     vl_logic_vector(31 downto 0);
        ReadData_WB     : out    vl_logic_vector(31 downto 0);
        ALUResult_M     : in     vl_logic_vector(31 downto 0);
        ALUResult_WB    : out    vl_logic_vector(31 downto 0);
        writereg_M      : in     vl_logic_vector(4 downto 0);
        writereg_WB     : out    vl_logic_vector(4 downto 0);
        RegWrite_M      : in     vl_logic;
        RegWrite_WB     : out    vl_logic;
        MemtoReg_M      : in     vl_logic;
        MemtoReg_WB     : out    vl_logic
    );
end M_WB;
