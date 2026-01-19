-- ================================================================================ --
-- NEORV32 CPU Wishbone Wrapper (single-file, no extra modules)                      --
-- -------------------------------------------------------------------------------- --
-- Instantiates `neorv32_cpu` and exposes a single Wishbone-like bus:                --
--   cyc, stb, we, sel[3:0], addr[31:0], data_out[31:0], data_in[31:0], ack         --
-- Arbitrates the CPU's instruction/data internal buses and translates handshakes.   --
-- No other modules are instantiated; all logic is implemented here.                 --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Licensed under the BSD-3-Clause license.                                         --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity processorci_top is
  generic (
    -- Pass-through CPU generics (same as `neorv32_cpu`)
    -- General --
    HART_ID             : natural range 0 to 1023 := 0;
    BOOT_ADDR           : std_ulogic_vector(31 downto 0) := (others => '0');
    DEBUG_PARK_ADDR     : std_ulogic_vector(31 downto 0) := (others => '0');
    DEBUG_EXC_ADDR      : std_ulogic_vector(31 downto 0) := (others => '0');
    -- RISC-V ISA Extensions --
    RISCV_ISA_C         : boolean := true;
    RISCV_ISA_E         : boolean := false;
    RISCV_ISA_M         : boolean := true;
    RISCV_ISA_U         : boolean := true;
    RISCV_ISA_Zaamo     : boolean := true;
    RISCV_ISA_Zalrsc    : boolean := true;
    RISCV_ISA_Zcb       : boolean := false;
    RISCV_ISA_Zba       : boolean := true;
    RISCV_ISA_Zbb       : boolean := true;
    RISCV_ISA_Zbkb      : boolean := false;
    RISCV_ISA_Zbkc      : boolean := false;
    RISCV_ISA_Zbkx      : boolean := false;
    RISCV_ISA_Zbs       : boolean := true;
    RISCV_ISA_Zfinx     : boolean := false;
    RISCV_ISA_Zicntr    : boolean := true;
    RISCV_ISA_Zicond    : boolean := true;
    RISCV_ISA_Zihpm     : boolean := true;
    RISCV_ISA_Zknd      : boolean := false;
    RISCV_ISA_Zkne      : boolean := false;
    RISCV_ISA_Zknh      : boolean := false;
    RISCV_ISA_Zksed     : boolean := false;
    RISCV_ISA_Zksh      : boolean := false;
    RISCV_ISA_Zmmul     : boolean := false;
    RISCV_ISA_Zxcfu     : boolean := false;
    RISCV_ISA_Sdext     : boolean := false;
    RISCV_ISA_Sdtrig    : boolean := false;
    RISCV_ISA_Smpmp     : boolean := false;
    -- Tuning Options --
    CPU_CONSTT_BR_EN    : boolean := false;
    CPU_FAST_MUL_EN     : boolean := true;
    CPU_FAST_SHIFT_EN   : boolean := true;
    CPU_RF_HW_RST_EN    : boolean := false;
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS     : natural range 0 to 16 := 0;
    PMP_MIN_GRANULARITY : natural := 4;
    PMP_TOR_MODE_EN     : boolean := false;
    PMP_NAP_MODE_EN     : boolean := false;
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS        : natural range 0 to 13 := 0;
    HPM_CNT_WIDTH       : natural range 0 to 64 := 32;
    -- Trigger Module (TM) --
    NUM_HW_TRIGGERS     : natural range 0 to 16 := 0
  );
  port (
    -- Global control --
    sys_clk     : in  std_ulogic; -- system clock
    rst_n       : in  std_ulogic; -- system reset, low-active

    -- External Wishbone-like instruction bus --
    core_cyc      : out std_ulogic;
    core_stb      : out std_ulogic;
    core_we       : out std_ulogic;
    core_sel      : out std_ulogic_vector(3 downto 0);
    core_addr     : out std_ulogic_vector(31 downto 0);
    core_data_out : out std_ulogic_vector(31 downto 0);
    core_data_in  : in  std_ulogic_vector(31 downto 0);
    core_ack      : in  std_ulogic;

    -- External Wishbone-like data bus --
    data_mem_cyc      : out std_ulogic;
    data_mem_stb      : out std_ulogic;
    data_mem_we       : out std_ulogic;
    data_mem_sel      : out std_ulogic_vector(3 downto 0);
    data_mem_addr     : out std_ulogic_vector(31 downto 0);
    data_mem_data_out : out std_ulogic_vector(31 downto 0);
    data_mem_data_in  : in  std_ulogic_vector(31 downto 0);
    data_mem_ack      : in  std_ulogic
  );
end processorci_top;

architecture rtl of processorci_top is

  -- CPU bus interfaces --
  signal ibus_req : bus_req_t;
  signal dbus_req : bus_req_t;
  signal ibus_rsp : bus_rsp_t := rsp_terminate_c;
  signal dbus_rsp : bus_rsp_t := rsp_terminate_c;

  -- interrupts tied low (no external interrupts) --
  constant zero16 : std_ulogic_vector(15 downto 0) := (others => '0');

  -- Pipelined Wishbone ---------
  signal r_core_ack       : std_ulogic;
  signal r_core_data_in   : std_ulogic_vector(31 downto 0);
  signal r_data_mem_ack   : std_ulogic;
  signal r_data_mem_data_in : std_ulogic_vector(31 downto 0); 

begin

  -- CPU instance -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------
  Processor: entity neorv32.neorv32_cpu
    generic map (
      -- General --
      HART_ID             => HART_ID,
      BOOT_ADDR           => BOOT_ADDR,
      DEBUG_PARK_ADDR     => DEBUG_PARK_ADDR,
      DEBUG_EXC_ADDR      => DEBUG_EXC_ADDR,
      -- RISC-V ISA Extensions --
      RISCV_ISA_C         => RISCV_ISA_C,
      RISCV_ISA_E         => RISCV_ISA_E,
      RISCV_ISA_M         => RISCV_ISA_M,
      RISCV_ISA_U         => RISCV_ISA_U,
      RISCV_ISA_Zaamo     => RISCV_ISA_Zaamo,
      RISCV_ISA_Zalrsc    => RISCV_ISA_Zalrsc,
      RISCV_ISA_Zcb       => RISCV_ISA_Zcb,
      RISCV_ISA_Zba       => RISCV_ISA_Zba,
      RISCV_ISA_Zbb       => RISCV_ISA_Zbb,
      RISCV_ISA_Zbkb      => RISCV_ISA_Zbkb,
      RISCV_ISA_Zbkc      => RISCV_ISA_Zbkc,
      RISCV_ISA_Zbkx      => RISCV_ISA_Zbkx,
      RISCV_ISA_Zbs       => RISCV_ISA_Zbs,
      RISCV_ISA_Zfinx     => RISCV_ISA_Zfinx,
      RISCV_ISA_Zicntr    => RISCV_ISA_Zicntr,
      RISCV_ISA_Zicond    => RISCV_ISA_Zicond,
      RISCV_ISA_Zihpm     => RISCV_ISA_Zihpm,
      RISCV_ISA_Zknd      => RISCV_ISA_Zknd,
      RISCV_ISA_Zkne      => RISCV_ISA_Zkne,
      RISCV_ISA_Zknh      => RISCV_ISA_Zknh,
      RISCV_ISA_Zksed     => RISCV_ISA_Zksed,
      RISCV_ISA_Zksh      => RISCV_ISA_Zksh,
      RISCV_ISA_Zmmul     => RISCV_ISA_Zmmul,
      RISCV_ISA_Zxcfu     => RISCV_ISA_Zxcfu,
      RISCV_ISA_Sdext     => RISCV_ISA_Sdext,
      RISCV_ISA_Sdtrig    => RISCV_ISA_Sdtrig,
      RISCV_ISA_Smpmp     => RISCV_ISA_Smpmp,
      -- Tuning Options --
      CPU_CONSTT_BR_EN    => CPU_CONSTT_BR_EN,
      CPU_FAST_MUL_EN     => CPU_FAST_MUL_EN,
      CPU_FAST_SHIFT_EN   => CPU_FAST_SHIFT_EN,
      CPU_RF_HW_RST_EN    => CPU_RF_HW_RST_EN,
      -- Physical Memory Protection (PMP) --
      PMP_NUM_REGIONS     => PMP_NUM_REGIONS,
      PMP_MIN_GRANULARITY => PMP_MIN_GRANULARITY,
      PMP_TOR_MODE_EN     => PMP_TOR_MODE_EN,
      PMP_NAP_MODE_EN     => PMP_NAP_MODE_EN,
      -- Hardware Performance Monitors (HPM) --
      HPM_NUM_CNTS        => HPM_NUM_CNTS,
      HPM_CNT_WIDTH       => HPM_CNT_WIDTH,
      -- Trigger Module (TM) --
      NUM_HW_TRIGGERS     => NUM_HW_TRIGGERS
    )
    port map (
      -- global control --
      clk_i      => sys_clk,
      rstn_i     => rst_n,
      -- status --
      trace_o    => open,
      sleep_o    => open,
      -- interrupts --
      msi_i      => '0',
      mei_i      => '0',
      mti_i      => '0',
      firq_i     => zero16,
      dbi_i      => '0',
      -- instruction bus interface --
      ibus_req_o => ibus_req,
      ibus_rsp_i => ibus_rsp,
      -- data bus interface --
      dbus_req_o => dbus_req,
      dbus_rsp_i => dbus_rsp
    );

  -- Pipelined Wishbone  --------------------------------------------
  delay_inst: process(sys_clk)
  begin
    if rising_edge(sys_clk) then
      if (rst_n = '0') then
        r_core_ack <= '0';
        r_core_data_in <= (others => '0');
      else
        r_core_ack <= core_ack;
        r_core_data_in <= core_data_in;
      end if;
    end if;
  end process delay_inst;

  delay_data: process(sys_clk)
  begin
    if rising_edge(sys_clk) then
      if (rst_n = '0') then
        r_data_mem_ack <= '0';
        r_data_mem_data_in <= (others => '0');
      else
        r_data_mem_ack <= data_mem_ack;
        r_data_mem_data_in <= data_mem_data_in;
      end if;
    end if;
  end process delay_data;

  -- Expanding instruction interface signals -------------------------------------------------------
  core_addr     <= ibus_req.addr;
  core_data_out <= ibus_req.data;
  core_we       <= ibus_req.rw;
  core_sel      <= ibus_req.ben;
  core_stb      <= ibus_req.stb;
  core_cyc      <= '1';

  ibus_rsp.data   <= r_core_data_in;
  ibus_rsp.ack    <= r_core_ack;
  ibus_rsp.err    <= '0';

-- Expanding Data interface signals -------------------------------------------------------
  data_mem_addr     <= dbus_req.addr;
  data_mem_data_out <= dbus_req.data;
  data_mem_we       <= dbus_req.rw;
  data_mem_sel      <= dbus_req.ben;
  data_mem_stb      <= dbus_req.stb;
  data_mem_cyc      <= '1';

  dbus_rsp.data   <= r_data_mem_data_in;
  dbus_rsp.ack    <= r_data_mem_ack;
  dbus_rsp.err    <= '0';

end rtl;
