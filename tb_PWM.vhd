library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity tb_PWM is
end tb_PWM;

architecture test of tb_PWM is

	constant HALF_PERIOD : time := 5 ns;

	constant ALLDUTY :std_logic_vector(7 downto 0) := x"05";
	constant DUTY :std_logic_vector(7 downto 0) := x"02";
	constant POLARIRY: std_logic_vector(7 downto 0) := x"01";
	
	constant addrDivider : std_logic_vector(3 downto 0) := x"0";
	constant addrAllDuty : std_logic_vector(3 downto 0) := x"1";
	constant addrDuty : std_logic_vector(3 downto 0) := x"2";
	constant addrPolarity : std_logic_vector(3 downto 0) := x"3";

	signal  clk 	: std_logic := '0';
	signal  nReset 	: std_logic := '0';
	signal  enable 	: std_logic;
		
	signal  address 	: std_logic_vector(3 downto 0);
	signal  write 		: std_logic := '1';
	signal  read 		: std_logic;
	signal  writedata 	: std_logic_vector(15 downto 0);
	signal  readdata 	: std_logic_vector(15 downto 0);
		
	signal  pwm_out 	: std_logic;

begin
  
  inst_cd: entity work.ClockDivider
  port map (
    clk => clk,
    nReset => nReset,
    address => address,
    write => write,
    read => read,
    writedata => writedata,
    readdata => readdata,
    enable_out => enable);

  inst_pwm: entity work.PulseWidthMod
  port map (
    clk => clk,
    nReset => nReset,
    enable => enable,
    address => address,
    write => write,
    read => read,
    writedata => writedata(7 downto 0),
    readdata => readdata(7 downto 0),
    pwm_out => pwm_out);
    
  
  nReset <= '1' after 10 ns;
  
  process begin
    clk <= '1'; wait for HALF_PERIOD;
    clk <= '0'; wait for HALF_PERIOD;
  end process;
  
  process begin
    wait for 5 ns;
    address <= addrDivider;
    writedata <= x"0005";
    wait for 60 ns;
    address <= addrAllDuty;
    writedata <= x"0005";
    wait for 50 ns;
    address <= addrDuty;
    writedata <= x"0003";
    wait for 50 ns;     
    address <= addrPolarity;
    writedata <= x"0001";
    wait for 50 ns;    
    write <= '0';
    wait for 800 ns;
    write <= '1';
    wait for 5 ns;
    address <= addrAllDuty;
    writedata <= x"0007";
    wait for 100 ns;
    write <= '0';
    wait;
  end process;
    
  
  

end architecture test;