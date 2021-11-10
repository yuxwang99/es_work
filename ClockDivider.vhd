library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ClockDivider is 
	port (
		clk 	: in std_logic;
		nReset 	: in std_logic;
		
		address 	: in std_logic_vector(3 downto 0);
		write 		: in std_logic;
		read 		: in std_logic;
		writedata 	: in std_logic_vector(15 downto 0);
		readdata 	: out std_logic_vector(15 downto 0);
		
		enable_out 	: out std_logic
	);
end ClockDivider;

architecture divide of ClockDivider is 
	signal iRegDivider 	: std_logic_vector(15 downto 0) := (others => '0');
	-- signal counter		: unsigned(15 downto 0) := (others => '0');
begin

	-- write period to register
	w_period: process (clk, nReset)
	begin
		if nReset = '0' then
			iRegDivider <= (others => '0');
		elsif rising_edge(clk) then
			if write = '1' then
				if address = "0000" then
					iRegDivider <= writedata;
				end if;
			end if;
		end if;
	end process w_period;
	
	-- read period from register
	r_period: process (clk, nReset)
	begin
		if nReset = '0' then
			readdata <= (others => '0');
		elsif rising_edge(clk) then
			if read = '1' then
				if address = "0000" then
					readdata <= iRegDivider;
				end if;
			end if;
		end if;
	end process r_period;
	
	
	enable: process (clk, nReset, iRegDivider)
	  variable counter : unsigned(15 downto 0) := (others => '0');
	begin
		if nReset = '0' then
			counter := (others => '0');
		elsif iRegDivider'event then
		  counter := (others => '0');
		elsif rising_edge(clk) then
		  if iRegDivider /= x"0000" then
			  if iRegDivider = std_logic_vector(counter) then
				  enable_out <= '1';
				  counter := (others => '0');
			  else
				  enable_out <= '0';
			  end if;
			end if;
			counter := counter + 1;
		end if;		
	end process enable;
	

end divide;