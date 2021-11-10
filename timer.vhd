library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.all;


entity timer is
    port(
        -- bus interface
        clk     : in  std_logic;
        reset_n : in  std_logic;
        cs      : in  std_logic;
        read    : in  std_logic;
        write   : in  std_logic;
        address : in  std_logic_vector(1 downto 0);
        wrdata  : in  std_logic_vector(31 downto 0);

        irq     : out std_logic;
        rddata  : out std_logic_vector(31 downto 0)
    );
end timer;

architecture synth of timer is

    constant CLK_PERIOD : time    := 40 ns;
    signal counter, period : std_logic_vector(31 downto 0);
    signal status_to, status_run : std_logic;
    signal ctrl_ito, ctrl_cont, ctrl_start, ctrl_stop : std_logic ;
    signal addr_next : std_logic_vector(1 downto 0);
    signal read_next, cs_next : std_logic;
    
begin
    rddata <= (others => 'Z'); 
   

    wt : process (write, clk,reset_n, counter,wrdata,status_run)
    begin
	rddata <= (others => 'Z'); 
        if reset_n = '0' then
    	        ctrl_cont <= '0' ;
	        ctrl_ito <= '0';
	        status_to <= '0';
	        status_run <= '0';
	        period <= (others => '0');
	        counter <= (others => '0');
	        rddata <= (others => 'Z');

	else
            if rising_edge(clk) then
	        if write = '1' then
		    if address = "01" and cs = '1' then
		        rddata <= (others => 'Z'); 
		        period <= wrdata;
			counter <= wrdata;
		    end if;
		    if address = "10" and cs = '1' then
		        rddata <= (others => 'Z'); 
			if wrdata(3) = '1' then
		            ctrl_start <= '1';
			    ctrl_stop <= '0';
			end if;
			if wrdata(2) = '1' then
		            ctrl_start <= '0';
			    ctrl_stop <= '1';
			end if;
			ctrl_ito <= wrdata(1);
			ctrl_cont <= wrdata(0);
		    end if;
		    if address = "11" and cs = '1' then
		        status_to <= status_to and wrdata(1); 
		    end if;

	        end if;
	    	if not(write='1' and address="01" and cs='1') or not(write='1' and address="10" and cs='1')then
	    	    if counter = x"00000000" then
			counter <= period;
		        status_to <= '1' and status_run;
			ctrl_start <= ctrl_cont ;
			--ctrl_stop <= (not ctrl_cont) and status_run;
	    	        status_run <= '0';
	    	    else 
		    	if ctrl_start = '1' and ctrl_stop = '0' then
		            counter <= counter - 1;
			    status_run <= ctrl_start ;
		    	end if;
	    	    end if;
               end if;
            end if;
	end if;
    end process wt;

    rd_delay : process(clk)
    begin 
	rddata <= (others => 'Z'); 
	if rising_edge(clk) then
	    read_next <= read ;
	    cs_next <= cs;
	    addr_next <= address;
	end if;
    end process rd_delay;

    rd : process (read_next, cs_next, counter, addr_next,status_run)

    begin
	rddata <= (others => 'Z'); 
	if read_next = '1' and cs_next = '1' then
		rddata <= (others => '0'); 
		if addr_next  = "00" then
		    rddata <= counter;
		end if;
		if addr_next = "01" then
		    rddata <= period ;
		end if;
		if addr_next  = "10" then
		    rddata <= (0 => ctrl_cont, 1=>ctrl_ito, others=>'0') ;	    
		end if;
		if addr_next  = "11" then -- read status
		    rddata <= (0 => status_run, 1=>status_to, others=>'0');
		end if;
	 end if; 
    end process rd;

    irq_bv : process(status_to,clk)
    begin
	if rising_edge(clk) then
            irq <= status_to and ctrl_ito;
	end if;
    end process irq_bv;

end synth;
