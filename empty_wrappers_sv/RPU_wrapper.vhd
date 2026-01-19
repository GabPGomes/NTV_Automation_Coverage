library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity processorci_top is
    generic (
        ENABLE_SECOND_MEMORY : boolean := false
    );
    port (
        sys_clk        : in  std_logic;
        rst_n          : in  std_logic;

        core_cyc       : out std_logic;
        core_stb       : out std_logic;
        core_we        : out std_logic;
        core_sel       : out std_logic_vector(3 downto 0);
        core_addr      : out std_logic_vector(31 downto 0);
        core_data_out  : out std_logic_vector(31 downto 0);
        core_data_in   : in  std_logic_vector(31 downto 0);
        core_ack       : in  std_logic;

        data_mem_cyc       : out std_logic := '0';
        data_mem_stb       : out std_logic := '0';
        data_mem_we        : out std_logic := '0';
        data_mem_sel       : out std_logic_vector(3 downto 0) := (others => '0');
        data_mem_addr      : out std_logic_vector(31 downto 0) := (others => '0');
        data_mem_data_out  : out std_logic_vector(31 downto 0) := (others => '0');
        data_mem_data_in   : in  std_logic_vector(31 downto 0) := (others => '0');
        data_mem_ack       : in  std_logic := '0'
    );
end entity;

architecture Behavioral of processorci_top is

    signal clk_core       : std_logic;
    signal rst_core       : std_logic;

    signal mem_o_addr     : std_logic_vector(31 downto 0);
    signal byte_enable    : std_logic_vector(1 downto 0);

    -- core_sel é atribuído no processo abaixo
    signal sel_internal : std_logic_vector(3 downto 0);

    component core
        port (
            I_clk            : in  std_logic;
            I_reset          : in  std_logic;
            I_halt           : in  std_logic;

            I_int_data       : in  std_logic_vector(31 downto 0);
            I_int            : in  std_logic;
            O_int_ack        : out std_logic;

            MEM_I_ready      : in  std_logic;
            MEM_O_cmd        : out std_logic;
            MEM_O_we         : out std_logic;
            MEM_O_byteEnable : out std_logic_vector(1 downto 0);
            MEM_O_addr       : out std_logic_vector(31 downto 0);
            MEM_O_data       : out std_logic_vector(31 downto 0);
            MEM_I_data       : in  std_logic_vector(31 downto 0);
            MEM_I_dataReady  : in  std_logic;

            O_halted         : out std_logic;
            O_DBG            : out std_logic_vector(63 downto 0)
        );
    end component;

begin

    clk_core <= sys_clk;
    rst_core <= not rst_n;

    -- Atribuição direta
    core_stb   <= core_cyc;
    core_addr  <= mem_o_addr;
    core_sel <= sel_internal;

    Processor : core
        port map (
            I_clk             => clk_core,
            I_reset           => rst_core,
            I_halt            => '0',

            I_int_data        => (others => '0'),
            I_int             => '0',
            O_int_ack         => open,

            MEM_I_ready       => '1',
            MEM_O_cmd         => core_cyc,
            MEM_O_we          => core_we,
            MEM_O_byteEnable  => byte_enable,
            MEM_O_addr        => mem_o_addr,
            MEM_O_data        => core_data_out,
            MEM_I_data        => core_data_in,
            MEM_I_dataReady   => core_ack,

            O_halted          => open,
            O_DBG             => open
        );

    -- Conversão da lógica combinacional de geração do core_sel
    process (byte_enable, mem_o_addr)
    begin
        sel_internal <= "0000";
        case byte_enable is
            when "00" =>  -- 1 byte
                case mem_o_addr(1 downto 0) is
                    when "00" => sel_internal <= "0001";
                    when "01" => sel_internal <= "0010";
                    when "10" => sel_internal <= "0100";
                    when "11" => sel_internal <= "1000";
                    when others => sel_internal <= "0000";
                end case;

            when "01" =>  -- 2 bytes (halfword)
                case mem_o_addr(1 downto 0) is
                    when "00" => sel_internal <= "0011";
                    when "10" => sel_internal <= "1100";
                    when others => sel_internal <= "0000"; -- inválido
                end case;

            when "10" =>  -- 4 bytes (word)
                sel_internal <= "1111";
            when others =>
                sel_internal <= "0000";
        end case;
    end process;

end architecture;
