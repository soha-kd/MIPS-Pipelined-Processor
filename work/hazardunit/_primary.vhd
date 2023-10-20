library verilog;
use verilog.vl_types.all;
entity hazardunit is
    port(
        Rt_EX           : in     vl_logic_vector(4 downto 0);
        Rs_D            : in     vl_logic_vector(4 downto 0);
        Rt_D            : in     vl_logic_vector(4 downto 0);
        writereg_M      : in     vl_logic_vector(4 downto 0);
        writereg_EX     : in     vl_logic_vector(4 downto 0);
        MemtoReg_E      : in     vl_logic;
        MemtoReg_M      : in     vl_logic;
        RegWrite_EX     : in     vl_logic;
        Branch_ID       : in     vl_logic;
        stall_IF_ID     : out    vl_logic;
        stall_ID_EX     : out    vl_logic;
        flush_EX_Mem    : out    vl_logic
    );
end hazardunit;
