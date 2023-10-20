library verilog;
use verilog.vl_types.all;
entity ID_EX is
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        dataone_ID      : in     vl_logic_vector(31 downto 0);
        dataone_Ex      : out    vl_logic_vector(31 downto 0);
        WriteData_ID    : in     vl_logic_vector(31 downto 0);
        WriteData_Ex    : out    vl_logic_vector(31 downto 0);
        extendedimm_ID  : in     vl_logic_vector(31 downto 0);
        extendedimm_Ex  : out    vl_logic_vector(31 downto 0);
        Instr_ID        : in     vl_logic_vector(31 downto 0);
        Instr_Ex        : out    vl_logic_vector(31 downto 0);
        RegWrite_ID     : in     vl_logic;
        RegWrite_Ex     : out    vl_logic;
        MemtoReg_ID     : in     vl_logic;
        MemtoReg_Ex     : out    vl_logic;
        MemWrite_ID     : in     vl_logic;
        MemWrite_Ex     : out    vl_logic;
        ALUControl_ID   : in     vl_logic_vector(3 downto 0);
        ALUControl_Ex   : out    vl_logic_vector(3 downto 0);
        ALUSrc_ID       : in     vl_logic;
        ALUSrc_Ex       : out    vl_logic;
        RegDst_ID       : in     vl_logic;
        RegDst_Ex       : out    vl_logic
    );
end ID_EX;
