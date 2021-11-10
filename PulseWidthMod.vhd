library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity PulseWidthMod is 
	port (
		clk 	: in std_logic;
		nReset 	: in std_logic;
		enable 	: in std_logic;
		
		address 	: in std_logic_vector(3 downto 0);
		write 		: in std_logic;
		read 		: in std_logic;
		writedata 	: in std_logic_vector(7 downto 0);
		readdata 	: out std_logic_vector(7 downto 0);
		
		pwm_out 	: out std_logic
	);
end PulseWidthMod;

architecture PWM of PulseWidthMod is 

	signal allDuty 		: std_logic_vector(7 downto 0) := (others => '0');
	signal duty			: std_logic_vector(7 downto 0) := (others => '0');
	signal polarity 	: std_logic_vector(7 downto 0) := x"01";
	
	-- signal counter	: unsigned(7 downto 0) := (others => '0');
begin

	-- write period to register
	w_period: process (clk, nReset)
	begin
		if nReset = '0' then
			allDuty <= (others => '0');
			duty <= (others => '0');
			polarity <= x"01";
		elsif rising_edge(clk) then
			if enable = '1' then
				if write = '1' then
					case address is 
						when "0001" => allDuty <= writedata;
						when "0010" => duty <= writedata;
						when "0011" => polarity <= writedata;
						when others => null;
					end case;
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
			if enable = '1' then
				if read = '1' then
					case address is 
						when "0001" => readdata <= allDuty;
						when "0010" => readdata <= duty;
						when "0011" => readdata <= polarity;
						when others => null;
					end case;
				end if;
			end if;
		end if;
	end process r_period;
	
	
	pwm: process (clk, enable, nReset, allDuty, duty, polarity) is 
	   variable counter	: unsigned(7 downto 0) := (others => '0');
	begin
	  if allDuty'event or duty'event or polarity'event then
	    counter := (others => '0');
		elsif rising_edge(clk) then
			if enable = '1' then
			  if duty /= x"00" and allDuty /= x"00" then 
				  if allDuty = std_logic_vector(counter) then
					 pwm_out <= '1' xnor polarity(0);
					 counter := (others => '0');
				  elsif duty = std_logic_vector(counter) then
					 pwm_out <= '0' xnor polarity(0);
				  end if;
				  counter := counter + 1;
				end if;
			end if;
		end if;
	end process pwm;

end PWM;