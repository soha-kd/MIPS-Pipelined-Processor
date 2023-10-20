library verilog;
use verilog.vl_types.all;
entity IF_ID is
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        stall           : in     vl_logic;
        PCplus4_IF      : in     vl_logic_vector(31 downto 0);
        PCplus4_ID      : out    vl_logic_vector(31 downto 0);
        Instr_IF        : in     vl_logic_vector(31 downto 0);
        Instr_ID        : out    vl_logic_vector(31 downto 0)
    );
end IF_ID;
