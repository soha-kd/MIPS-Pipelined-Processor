library verilog;
use verilog.vl_types.all;
entity InstMem is
    port(
        ReadAddress     : in     vl_logic_vector(31 downto 0);
        Instruction     : out    vl_logic_vector(31 downto 0)
    );
end InstMem;
