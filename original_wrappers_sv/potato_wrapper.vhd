-- Bus type: Wishbone

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity processorci_top is
    generic (
        SIMULATION : boolean := false;
        ENABLE_SECOND_MEMORY : boolean := false;
        PIPELINED_WISHBONE : boolean := false
    );
    port (
        sys_clk : in std_logic; -- Clock de sistema
        rst_n   : in std_logic; -- Reset do sistema

        -- UART pins (when not in SIMULATION)
        rx : in  std_logic;
        tx : out std_logic;

        -- SPI pins (when not in SIMULATION)
        sck  : in  std_logic;
        cs   : in  std_logic;
        mosi : in  std_logic;
        miso : out std_logic;

        -- SPI control pins (when not in SIMULATION)
        rw   : in  std_logic;
        intr : out std_logic;

        -- Wishbone signals (when in SIMULATION)
        core_cyc      : out std_logic;                     -- Indica uma transação ativa
        core_stb      : out std_logic;                     -- Indica uma solicitação ativa
        core_we       : out std_logic;                     -- 1 = Write, 0 = Read
        core_sel      : out std_logic_vector(3 downto 0);  -- Seletores de byte
        core_addr     : out std_logic_vector(31 downto 0); -- Endereço
        core_data_out : out std_logic_vector(31 downto 0); -- Dados de entrada (para escrita)
        core_data_in  : in  std_logic_vector(31 downto 0); -- Dados de saída (para leitura)
        core_ack      : in  std_logic;                     -- Confirmação da transação

        -- Second memory signals (when ENABLE_SECOND_MEMORY)
        data_mem_cyc      : out std_logic;
        data_mem_stb      : out std_logic;
        data_mem_we       : out std_logic;
        data_mem_sel      : out std_logic_vector(3 downto 0);
        data_mem_addr     : out std_logic_vector(31 downto 0);
        data_mem_data_out : out std_logic_vector(31 downto 0);
        data_mem_data_in  : in  std_logic_vector(31 downto 0);
        data_mem_ack      : in  std_logic
    );
end entity processorci_top;

architecture rtl of processorci_top is
    

    component pp_potato is
    	generic(
		    PROCESSOR_ID           : std_logic_vector(31 downto 0) := x"00000000"; --! Processor ID.
		    RESET_ADDRESS          : std_logic_vector(31 downto 0) := x"00000000"; --! Address of the first instruction to execute.
		    MTIME_DIVIDER          : positive                      := 5;           --! Divider for the clock driving the MTIME counter.
		    ICACHE_ENABLE          : boolean                       := true;        --! Whether to enable the instruction cache.
		    ICACHE_LINE_SIZE       : natural                       := 4;           --! Number of words per instruction cache line.
		    ICACHE_NUM_LINES       : natural                       := 128          --! Number of cache lines in the instruction cache.
	    );
        port (
            clk                     : in  std_logic;
            reset                   : in  std_logic;
            irq                     : in  std_logic_vector(7 downto 0);
            wb_dat_in               : in  std_logic_vector(31 downto 0);
            wb_ack_in               : in  std_logic;
            test_context_out_state  : out std_logic_vector(2 downto 0);
            test_context_out_number : out std_logic_vector(31 downto 0);
            wb_adr_out              : out std_logic_vector(31 downto 0);
            wb_sel_out              : out std_logic_vector(3 downto 0);
            wb_cyc_out              : out std_logic;
            wb_stb_out              : out std_logic;
            wb_we_out               : out std_logic;
            wb_dat_out              : out std_logic_vector(31 downto 0)
        );
    end component;

    -- Internal signals
    signal clk_core  : std_logic;
    signal rst_core  : std_logic;


begin

    clk_core <= sys_clk;
    rst_core <= not rst_n;

    -- Core instantiation
    Processor: pp_potato
	    generic map(
            PROCESSOR_ID    => x"00000000",
            RESET_ADDRESS   => x"00000000",
            MTIME_DIVIDER   => 5,
            ICACHE_ENABLE   => false,
            ICACHE_LINE_SIZE=> 4,
            ICACHE_NUM_LINES=> 128
	    )
        port map (
            clk                     => clk_core,
            reset                   => rst_core,
            irq                     => (others => '0'),
            wb_dat_in               => core_data_in,
            wb_ack_in               => core_ack,
            test_context_out_state  => open,
            test_context_out_number => open,
            wb_adr_out              => core_addr,
            wb_sel_out              => core_sel,
            wb_cyc_out              => core_cyc,
            wb_stb_out              => core_stb,
            wb_we_out               => core_we,
            wb_dat_out              => core_data_out
        );

end architecture rtl;