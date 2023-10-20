library verilog;
use verilog.vl_types.all;
entity DataMem is
    port(
        clock           : in     vl_logic;
        MemWrite        : in     vl_logic;
        address         : in     vl_logic_vector(31 downto 0);
        WriteData       : in     vl_logic_vector(31 downto 0);
        ReadData        : out    vl_logic_vector(31 downto 0)
    );
end DataMem;
