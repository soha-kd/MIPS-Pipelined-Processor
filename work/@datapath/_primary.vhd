library verilog;
use verilog.vl_types.all;
entity Datapath is
    port(
        clk             : in     vl_logic;
        reset           : in     vl_logic;
        RegDst_ID       : in     vl_logic;
        RegWrite_ID     : in     vl_logic;
        ALUSrc_ID       : in     vl_logic;
        B               : in     vl_logic;
        MemtoReg_ID     : in     vl_logic;
        MemWrite_ID     : in     vl_logic;
        Branch_ID       : in     vl_logic;
        ALUControl_ID   : in     vl_logic_vector(3 downto 0);
        ReadData_M      : in     vl_logic_vector(31 downto 0);
        Instr_IF        : in     vl_logic_vector(31 downto 0);
        MemWrite_M      : out    vl_logic;
        Instr_ID        : out    vl_logic_vector(31 downto 0);
        PC_IF           : out    vl_logic_vector(31 downto 0);
        WriteData_M     : out    vl_logic_vector(31 downto 0);
        ALUResult_M     : out    vl_logic_vector(31 downto 0)
    );
end Datapath;
