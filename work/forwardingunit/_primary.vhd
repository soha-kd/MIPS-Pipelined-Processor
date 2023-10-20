library verilog;
use verilog.vl_types.all;
entity forwardingunit is
    port(
        Rs_EX           : in     vl_logic_vector(4 downto 0);
        Rt_EX           : in     vl_logic_vector(4 downto 0);
        Rs_ID           : in     vl_logic_vector(4 downto 0);
        Rt_ID           : in     vl_logic_vector(4 downto 0);
        writereg_M      : in     vl_logic_vector(4 downto 0);
        writereg_WB     : in     vl_logic_vector(4 downto 0);
        RegWrite_M      : in     vl_logic;
        RegWrite_WB     : in     vl_logic;
        ForwardAE       : out    vl_logic_vector(1 downto 0);
        ForwardBE       : out    vl_logic_vector(1 downto 0);
        ForwardAD       : out    vl_logic;
        ForwardBD       : out    vl_logic
    );
end forwardingunit;
