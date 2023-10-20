library verilog;
use verilog.vl_types.all;
entity RegisterFile32bits is
    port(
        clock           : in     vl_logic;
        RegWrite        : in     vl_logic;
        reset           : in     vl_logic;
        ReadRegister1   : in     vl_logic_vector(4 downto 0);
        ReadRegister2   : in     vl_logic_vector(4 downto 0);
        WriteRegister   : in     vl_logic_vector(4 downto 0);
        WriteData       : in     vl_logic_vector(31 downto 0);
        ReadData1       : out    vl_logic_vector(31 downto 0);
        ReadData2       : out    vl_logic_vector(31 downto 0)
    );
end RegisterFile32bits;
