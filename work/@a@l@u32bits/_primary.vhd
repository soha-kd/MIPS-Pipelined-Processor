library verilog;
use verilog.vl_types.all;
entity ALU32bits is
    port(
        a               : in     vl_logic_vector(31 downto 0);
        b               : in     vl_logic_vector(31 downto 0);
        f               : in     vl_logic_vector(3 downto 0);
        y               : out    vl_logic_vector(31 downto 0);
        zero            : out    vl_logic
    );
end ALU32bits;
