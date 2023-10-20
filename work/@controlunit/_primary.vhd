library verilog;
use verilog.vl_types.all;
entity Controlunit is
    port(
        Opcode          : in     vl_logic_vector(5 downto 0);
        Func            : in     vl_logic_vector(5 downto 0);
        MemtoReg        : out    vl_logic;
        MemWrite        : out    vl_logic;
        ALUSrc          : out    vl_logic;
        RegDst          : out    vl_logic;
        RegWrite        : out    vl_logic;
        Jump            : out    vl_logic;
        Branch          : out    vl_logic;
        B               : out    vl_logic;
        ALUControl      : out    vl_logic_vector(3 downto 0)
    );
end Controlunit;
