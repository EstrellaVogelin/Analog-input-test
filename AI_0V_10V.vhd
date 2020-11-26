--------------------------------------------------------------------------------
------------------------------------- ENTITY -----------------------------------
--------------------------------------------------------------------------------

library ieee;
   use ieee.std_logic_1164.all;
   use ieee.numeric_std.all;
   use IEEE.MATH_REAL.ALL;
   
library std;
   use std.textio.all;

library work;
  use work.SPI_Slave_BFM_PKG.all;
  use work.OctoSPI_Slave_BFM_PKG.all;
  use work.ADS7953_BFM_PKG.all;
  use work.AI_0V_10V_tb_PKG.all; 
  use work.AI_SIM_PKG.all; 


entity AI_0V_10V is
    generic(SimType            : integer := 1;  -- 0->Post Layout; 1->functional
            SimCond            : integer := 1;  -- 0->bestcase; 1->typcase; 2->worstcase
            g_CLK_PERIOD       : time    := 20 ns
    );


  port (
    
  
    reset_n                 : out    std_logic := '0';
    clk                     : in     std_logic;
    S2P_SYNC                : inout  std_logic_vector(0 to 4);

    -- Command/Readback SPIs
    S2P_CR                  : inout std_logic_vector(0 to 17) := (others => 'Z');  -- OctoSPI
    -- Config/Events SPIs
    S2P_CE                  : inout std_logic_vector(0 to 3);-- := (others => 'Z');    -- SingleSPI
    
    ----------------------------------------------------------------------
                            
    -- SPI slave BFM Interface Signals
    SPIx_rx_rdy             : in  std_logic;
    SPIx_rx_data            : in  t_OctoSPI_matrix;
    SPIx_tx_data            : out t_OctoSPI_matrix ;--:= (others => (others => '0'));
    SPIx_tx_ack             : in  std_logic;
    SPIx_busy               : in  std_logic;

    BITE_SPI_rx_rdy         : in  std_logic;
    BITE_SPI_rx_data        : in  std_logic_vector(0 to 15);
    BITE_SPI_tx_data        : out std_logic_vector(0 to 15) := (others => '0');
    BITE_SPI_tx_ack         : in  std_logic;
    BITE_SPI_busy           : in  std_logic;

    -- ADS7953 SPI Signals
    ADS7953_CH0             : out real_vector(0 to 2):= (others => 2.048);
    ADS7953_CH1             : out real_vector(0 to 2):= (others => 0.0);
    ADS7953_CH2             : out real_vector(0 to 2):= (others => 0.0);
    ADS7953_CH3             : out real_vector(0 to 2):= (others => 0.0);
    ADS7953_CH4             : out real_vector(0 to 2):= (others => 0.0);
    ADS7953_CH5             : out real_vector(0 to 2):= (others => 0.0);  
    ADS7953_CH6             : out real_vector(0 to 2):= (others => 0.0);  
    ADS7953_CH7             : out real_vector(0 to 2):= (others => 0.0);  
    ADS7953_CH8             : out real_vector(0 to 2):= (others => 0.0);  
    ADS7953_CH9             : out real_vector(0 to 2):= (others => 0.0);  
    ADS7953_CH10            : out real_vector(0 to 2):= (others => 0.0);  
    ADS7953_CH11            : out real_vector(0 to 2):= (others => 0.0);  
    ADS7953_CH12            : out real_vector(0 to 2):= (others => 0.0);
    ADS7953_CH13            : out real_vector(0 to 2):= (others => 0.0);
    ADS7953_CH14            : out real_vector(0 to 2):= (others => 2.048);
    ADS7953_CH15            : out real_vector(0 to 2):= (others => 0.0);
    GPIO0                   : inout std_logic :='Z';
    GPIO1                   : inout std_logic :='Z';
    GPIO2                   : inout std_logic :='Z';
    GPIO3                   : inout std_logic :='Z'
									 
  
  );    
  
  
end AI_0V_10V;



-------------------------------------------------------------------------------
------------------------------- ARCHITECTURE ----------------------------------
-------------------------------------------------------------------------------
  architecture stimuli of AI_0V_10V is
  
  -----------------------------------------------------------------------------
  -------------------------------TYPES-----------------------------------------
  -----------------------------------------------------------------------------
    type t_AI_0_10_VDC  is array (0 to c_NUM_of_AI_0_10-1)   of std_logic_vector(0 to 15);
    
  -----------------------------------------------------------------------------
  ------------------------------CONSTANTS--------------------------------------
  -----------------------------------------------------------------------------  

    constant c_CLK_PERIOD           : time  := g_CLK_PERIOD;
                                    
                                    
    constant c_FT                   : std_logic_vector(1 downto 0):= "11";
    constant c_ND                   : std_logic_vector(1 downto 0):= "00";
    constant c_NO                   : std_logic_vector(1 downto 0):= "01";
    constant c_NCD                  : std_logic_vector(1 downto 0):= "10";
  
    -----------------------------------------------------
    -------------------- SIGNALS ------------------------
    -----------------------------------------------------
    signal OctoSPI_Message_array    : t_OctoSPI_array;
                                    
    signal testNumb                 : integer   := 0;
    signal testResult               : std_logic := '1';
    signal GeneralResult            : std_logic := '1';
    signal raw_config_msg           : std_logic_vector (8159 downto 0):= (others => '0');
                                    
    signal BITE_message_buffer      : t_Buff    := c_BUFF_RST;
    signal BITE_message_buffer_clr  : std_logic := '0'; 
    
    signal FSI_temp                 : std_logic_vector (0 to 31);
    signal FSI_temp_ID              : std_logic_vector (0 to 31);
    signal FSI_SUM                  : std_logic_vector (0 to 31);
    
    signal AI_0_10_Voltage_Input    : real_vector(0 to c_NUM_of_AI_0_10-1)  := (others => 0.0);
                                              


    
   --/*******************************************************************************
   --*
   --* Procedure name :  p_BIT_SPI_refresh
   --*
   --* Description    :  This procedure will:
   --*                   1) Perform the SPI exchange
   --*                   2) De-serialize the BIT data   
   --*
   --* Note(s)        :  None
   --*
   --* Input Data     :  None
   --*
   --* Output Data    :  data  : BIT data records
   --*
   --* Signals        :  None
   --*******************************************************************************/
   procedure p_BIT_refresh(data : out t_BIT_MSG_array; counter : out integer; BITE_message_buffer : in t_Buff) is

   begin    
     counter := 0;
     data:= (others => c_Event_STAT_RST);
     for i in 0 to (BITE_message_buffer.index-1)/16 loop
       data(i).FSI      := BITE_message_buffer.Data(i*16)& BITE_message_buffer.data(i*16+1);  
       data(i).SPARE_0  := BITE_message_buffer.Data(i*16+2);       
       data(i).State    := BITE_message_buffer.Data(i*16+3);
       data(i).SPARE_1  := BITE_message_buffer.Data(i*16+4);
       data(i).Sanction := BITE_message_buffer.Data(i*16+5);
       data(i).EDL      := BITE_message_buffer.Data(i*16+6);
       data(i).ED_0     := BITE_message_buffer.Data(i*16+7)(0 to 7);
       data(i).ED_1     := BITE_message_buffer.Data(i*16+7)(8 to 15);
       data(i).ED_2     := BITE_message_buffer.Data(i*16+8)(0 to 7);
       data(i).ED_3     := BITE_message_buffer.Data(i*16+8)(8 to 15);
       data(i).ED_4     := BITE_message_buffer.Data(i*16+9)(0 to 7);
       data(i).ED_5     := BITE_message_buffer.Data(i*16+9)(8 to 15);
       data(i).ED_6     := BITE_message_buffer.Data(i*16+10)(0 to 7);
       data(i).ED_7     := BITE_message_buffer.Data(i*16+10)(8 to 15);
       data(i).ED_8     := BITE_message_buffer.Data(i*16+11)(0 to 7);
       data(i).ED_9     := BITE_message_buffer.Data(i*16+11)(8 to 15);
       data(i).SPARE_2  := BITE_message_buffer.Data(i*16+12);
       data(i).SPARE_3  := BITE_message_buffer.Data(i*16+13);
       data(i).SPARE_4  := BITE_message_buffer.Data(i*16+14);
       data(i).CRC      := BITE_message_buffer.Data(i*16+15);
       counter := counter +1;
     end loop;
     counter := counter -1;
     -- Wait between SPI transmissions
     wait for 100 ns;
   end p_BIT_refresh;
   
   
   
   --/*******************************************************************************
   --*
   --* Procedure name :  p_find_BIT_with_State
   --*
   --* Description    :  This procedure will find if the BIT data records contain an FSI message,
   --*                   with a certain state
   --*
   --* Note(s)        :  Overloaded
   --*
   --* Input Data     :  fsi              - minimum position in the BIT data records
   --*                   state            - state to be checked
   --*                   BIT_data         - BIT data records
   --*
   --* Output Data    :  found            - 1: a message with the required FSI and state was found; 0: otherwise
   --*                   position         - position of the message in the BIT data records, -1 if not found
   --*
   --* Signals        :  testResult <out>  - will be set to '1' if an error is detected,
   --*                                       otherwise it will not be driven
   --*******************************************************************************/
   procedure p_find_BIT_with_State(
                              fsi                 : in  std_logic_vector(0 to 31);
                              -- fsi                 : in  integer;
                              state               : in  std_logic_vector(0 to 15);
                              BIT_data            : inout  t_BIT_MSG_array;
                              found               : inout integer;
                              index               : out integer;
                              BITE_message_buffer : in t_Buff) is
     variable v_index   : integer := 0;
   begin
         found := 0;
         index := 0;
      -- If not found, read the non-visible queue until the message appears or the limit of the queue is reached.
         p_BIT_refresh(BIT_data, v_index, BITE_message_buffer);
         for i in 0 to v_index loop
            if ((fsi = BIT_data(i).FSI) and (BIT_data(i).STATE = state)) then
               found := 1;
               index := i;
            end if;
            
         end loop;        
   end p_find_BIT_with_State;
   
   
   
    -- AI_0_10 mapping -- 
---------------------------------------------------------------------------------------
    ADS7953_CH5(1)  <= AI_0_10_Voltage_Input(0);
    ADS7953_CH7(1)  <= AI_0_10_Voltage_Input(1);
    ADS7953_CH4(1)  <= AI_0_10_Voltage_Input(2);
    ADS7953_CH6(1)  <= AI_0_10_Voltage_Input(3);
    ADS7953_CH1(1)  <= AI_0_10_Voltage_Input(4);
    ADS7953_CH11(1) <= AI_0_10_Voltage_Input(5);
    ADS7953_CH10(1) <= AI_0_10_Voltage_Input(6);
    ADS7953_CH9(1)  <= AI_0_10_Voltage_Input(7);
    
    
    
   --/*******************************************************************************
   --*
   --* Process name   :  p_remap_SPI
   --*
   --* Description    :  This process will remap the readback message from SPI
   --*
   --* Note(s)        :  None
   --*
   --*******************************************************************************/

   p_remap_SPI : process
    variable v_message_cnt : natural;
    variable v_word_cnt    : natural;
   begin
     wait until rising_edge(SPIx_rx_rdy);
     for idx in SPIx_rx_data'RANGE loop
       v_message_cnt := 0;
       while v_message_cnt < 8 loop
         v_word_cnt  := 0;
         while v_word_cnt < 8 loop
           OctoSPI_Message_array(idx)(v_message_cnt)(v_word_cnt) <= SPIx_rx_data(7-idx)((7-v_message_cnt)*256+((7-v_word_cnt)*32+31) downto (7-v_message_cnt)*256+((7-v_word_cnt)*32));
           v_word_cnt := v_word_cnt + 1;
         end loop;
         v_message_cnt := v_message_cnt + 1;
       end loop;
     end loop;
   end process p_remap_SPI;


    --/*******************************************************************************
   --*
   --* Process name   :  p_BITE_Rx_Buffer
   --*
   --* Description    :  This process will remap the readback message from EVENT SPI
   --*
   --* Note(s)        :  None
   --*
   --*******************************************************************************/ 
    
   p_BITE_Rx_Buffer : process
   begin
      wait on BITE_SPI_rx_rdy, BITE_message_buffer_clr;

      if(rising_edge(BITE_SPI_rx_rdy) AND (BITE_message_buffer.index < c_BUFF_DEPTH)) then
        BITE_message_buffer.data(BITE_message_buffer.index) <= BITE_SPI_rx_data;
        BITE_message_buffer.index                         <= BITE_message_buffer.index + 1;
      end if;
      if(rising_edge(BITE_message_buffer_clr)) then
        BITE_message_buffer.index                         <= 0;
      end if;
   end process p_BITE_Rx_Buffer;
   
   
   
   --------------------------------------------------
   --******************* Menu ******************--
   --------------------------------------------------
   menu : process
      variable rdLine: line;
      variable val: integer;
   begin
       p_SubTitle("");
       p_SubTitle("----------------------------------------------------------------------------");
       p_SubTitle("---                       Board Test Menu                         ---");
       p_SubTitle("----------------------------------------------------------------------------");
       p_SubTitle("");
       p_SubTitle("0 .- Test ALL ");
       p_SubTitle("");
       p_SubTitle("1 .- AI_0V_10V_001 ");
       p_SubTitle("");
       p_SubTitle("2 .- AI_0V_10V_002 ");
       p_SubTitle("");
       
       p_SubTitle("Select the test to perform: ");
       readline(input, rdLine);
       read(rdLine, val);
       
       testNumb <= val;

       
       wait;
       
   end process menu;
   
  --------------------------------------------------
  --******************* Stimuli ******************--
  --------------------------------------------------
   
  p_Stimuli : process 
    
    
    ----------------------------------------------------------------------------------------------------------------------------------------------
    --/*******************************************************************************
   --*
   --* Function name   :  f_to_real
   --*
   --* Description    :  This function return a real from a float
   --*
   --* Note(s)        :  None
   --*
   --*******************************************************************************/ 
    function f_to_real(float : in  std_logic_vector) return real is
      variable v_real         : real;
      variable v_float        : unsigned(31 downto 0) := unsigned(float);
      constant c_FP32_EXP     : std_logic_vector(30 downto 23) := (others => '0');
      constant c_FP32_MANTISA : std_logic_vector(22 downto  0) := (others => '0');
    begin
      if(float = (float'RANGE => '0')) then
        v_real := 0.0;
      else
        v_real := 2.0**(to_integer(v_float(c_FP32_EXP'RANGE))-127) * (1.0+real(to_integer(v_float(c_FP32_MANTISA'RANGE)))/2.0**23);
        if(float(float'LEFT) = '1') then
          v_real := -v_real;
        end if;
      end if;
      return v_real;
    end f_to_real;
    ---------------------------------------------------------------------------------------------------------------------------------------------




   ---------------------------------------------------------------------
   ---------------------------- VARIABLES ------------------------------
   ---------------------------------------------------------------------   
    variable test_nb                 : integer   := 0;
                                                     
    variable v_AI_VDC_CNF            : t_AI_VDC  := (x"1BBC", x"2BBC", x"3BBC", x"4BBC", x"5BBC", x"6BBC", x"7BBC", x"8BBC");     
    
    variable v_CRC                   : std_logic_vector(31 downto 0);
    variable v_CRC_Data              : std_logic_vector (8159 downto 0);
    variable v_count                 : integer;
    variable v_count2                : integer;
    variable V_CRC_16_cal            : std_logic_vector(15 downto 0);
    
    variable v_start_time            : time;
    variable v_start_time2           : time;
                                      
    variable v_AI_data               : t_AI_RECORD_array(0 to 7);
                                          
    variable v_tmp_AI_st             : std_logic_vector(0 to 7);

    variable v_CRC_test              : std_logic_vector(31 downto 0);
    
    variable v_AIs_val                : t_real_matrix (0 to 2)(0 to c_NUM_of_AI-1):= ((10.0, 9.25, 8.75, 9.75, 6.54, 8.5, 7.98, 7.15),
                                                                                    (1.0, 2.15, 3.18, 1.65, 4.7, 2.28, 5.23, 1.75),
																				                                            (9.98, 9.5, 10.0, 9.6, 9.7, 9.43, 9.8, 9.65));

                                     
    variable v_tmp_Event             : std_logic_vector(0 to 31);  
    variable v_BIT_data              : t_BIT_MSG_array; 
    variable v_tmp_b                 : integer := 0; 
    variable v_fsi_temp              : integer := 0; 
    variable v_FSI_SUM               : std_logic_vector (0 to 31);
    variable v_FSI_index             : integer := 0;


    begin
    
      SPIx_tx_data <=(others =>(others => '0'));
    
      S2P_SYNC             <= "0ZZZ0";
      S2P_CE               <= "ZZZZ";
      SPIx_tx_data         <= (others => (others => '0'));
      ADS7953_CH0          <= (others => 2.048);
      ADS7953_CH3(0)       <= (0.0);
      ADS7953_CH6(0)       <= (0.0);
      ADS7953_CH9(0)       <= (0.0);
      ADS7953_CH9(2)       <= (0.0);
      ADS7953_CH10(2)      <= (0.0);
      
      ADS7953_CH11(2)      <= (0.0); 
      ADS7953_CH12(0)      <= (0.0);  
      ADS7953_CH12(2)      <= (0.0);      
      ADS7953_CH13(0)      <= (0.0); 
      ADS7953_CH13(2)      <= (0.0); 
      ADS7953_CH14         <= (others => 2.048);
      ADS7953_CH15         <= (others => 0.0);
                      
      reset_n       <= '1';
      wait for 1600 ns;
      wait until rising_edge(clk);
      wait until rising_edge(clk);        
         
          
          
          
      p_SubTitle("=============================================================================");
      p_SubTitle("===                    Board Test Bench                    ===");
      p_SubTitle("=============================================================================");
      ASSERT FALSE REPORT "===       Board Test Bench        ===" SEVERITY Note;
      
      
    ------------------------------------------------
                 -- <AI_0V-10V_1001> --
    ------------------------------------------------  
    --**** This test purpose is to test different values from 0.0V to 10V in the 8 AI's
    --**** and check if CRC 16 is correct


    if((testNumb = 1001) or (testNumb = 0) or (testNumb = 100)) then 
      --------------------------------------------------------
      ------------- SET ALL AIs TO 5V FOR DEFAULT------------- 
      --------------------------------------------------------
      test_nb:= 0; 
	    testResult    <= '1';
	    for i in 0 to c_NUM_of_AI -1 loop
	      v_AI_data(i).CMD.VOLTAGE_CMD := 5.0;
        p_select_AI_cmd_voltage(0, AI_Voltage_Input, i, v_AI_data);
	    end loop;  
    
      --------------------------------------------------------
      --------------------- APPLY RESET ---------------------- 
      --------------------------------------------------------
      S2P_SYNC(0) <= '0';
      wait for 50 us;     
      reset_n <='0';
      wait for 50 us;
      reset_n <='1';       
      wait for 100 us;
      wait until rising_edge(clk);
    
      --------------------------------------------------------
      ----             SET AI X TO VALUE Y            ----
      --------------------------------------------------------	
	    for val_idx in v_AIs_val'RANGE loop -- 0 TO 2
	      for i in v_AIs_val(0)'RANGE loop  -- 0 TO 7
	        v_AI_data(i).CMD.VOLTAGE_CMD := v_AIs_val(val_idx)(i);
          p_select_AI_cmd_voltage(0, AI_Voltage_Input, i, v_AI_data);
	    	  wait for 3000 us;
            --------------------------------------------------------
            ----       CHECK VOLTAGE, VOLTAGE FS AND CRC        ----
            --------------------------------------------------------
	        p_SubTitle ("");
            test_nb:= test_nb+1;	
            p_select_AI_readback (0, OctoSPI_Message_array, v_AI_data);	  
            p_check(test_nb,"Check AI_"&integer'image(i+1)& " value", f_to_real(v_AI_data(i).STAT.VOL_RB), v_AIs_val(val_idx)(i), 0.1, testResult);
            p_check(test_nb,"Check for AI_"&integer'image(i+1)&" FS value", v_AI_data(i).STAT.FS_VOL_RB, c_NO, testResult);
	        p_SubTitle ("");	  
	        V_CRC_16_cal := Calculate_CRC16(OctoSPI_Message_array(4)(i)(0)& OctoSPI_Message_array(4)(i)(1)&OctoSPI_Message_array(4)(i)(2)&OctoSPI_Message_array(4)(i)(3)&OctoSPI_Message_array(4)(i)(4)&OctoSPI_Message_array(4)(i)(5)&OctoSPI_Message_array(4)(i)(6));
	        p_check(test_nb,"Check for AI "&integer'image(i+1)&" CRC value", OctoSPI_Message_array(4)(i)(7)(31 downto 16), V_CRC_16_cal, testResult);
	      end loop;
	    end loop;
    
    
      wait until rising_edge(clk);
      p_SubTitle("====================================");
      if(testResult = '1') then
        p_SubTitle("=== <AI_0V_10V_1001> : PASS ===");
      else
        p_SubTitle("=== <AI_0V_10V_1001> : FAIL ===");
        GeneralResult <= '0';
      end if;
      p_SubTitle("====================================");
      p_SubTitle("");
      p_SubTitle("");
    end if;
    
    
    
    
      ------------------------------------------------
                     -- <AI_0V_10V_1002> --
      ------------------------------------------------  
      --**** This test purpose is to test the reference monitor, FD computation and monitor event reporting
    
    if((testNumb = 1002) or (testNumb = 0) or (testNumb = 100)) then
      testResult    <= '1';
      --------------------------------------------------------
      ------------------ SET ALL AIs TO 5V -------------------
      --------------------------------------------------------
      for i in 0 to c_NUM_of_AI -1 loop
	      v_AI_data(i).CMD.VOLTAGE_CMD := 5.0;
        p_select_AI_cmd_voltage(0, AI_Voltage_Input, i, v_AI_data);
	    end loop;  
    
      --------------------------------------------------------
      --------------------- APPLY RESET ---------------------- 
      --------------------------------------------------------
      S2P_SYNC(0) <= '0';
      wait for 50 us; 
      reset_n <='0';
      wait for 50 us;
      reset_n <='1';      
      
      wait for 100 us;
      wait until rising_edge(clk);
    
    
      --------------------------------------------------------
      ------------------ CLEAR EVENT FIFO -------------------- 
      --------------------------------------------------------
	    BITE_message_buffer_clr <= '1';
	    wait for 10 ns;
	    BITE_message_buffer_clr <= '0'; 
    
      wait for 350 us;
      --------------------------------------------------------
      ------------ SET REF_VOLT TO INVALID VALUE -------------
      --------------------------------------------------------
      ADS7953_CH0(0) <= 4.0;
      ADS7953_CH14(0) <= 4.0;
    
      --------------------------------------------------------
      ---------- WAIT FOR THE VOLTAGE TO DECREASE ------------
      --------------------------------------------------------
    
      wait for 2820 us;
      
      wait until falling_edge (S2P_CR(0));
      wait until rising_edge (S2P_CR(0));
      
      --------------------------------------------------------
      ------- THE VOLTAGE READBACK OF ALL AIs IS 0.0V --------
      --------------------------------------------------------
      --------------------------------------------------------
      --------------- THE FD COMPUTATION IS ND ---------------
      --------------------------------------------------------
      p_select_AI_readback (0, OctoSPI_Message_array, v_AI_data);
    
      for j in 0 to c_NUM_of_AI -1 loop
        p_check(test_nb,"Check for AI  " &integer'image(j+1)& " value", f_to_real(v_AI_data(j).STAT.VOL_RB), 0.0, testResult);
        p_check(test_nb,"Check for AI FS  " &integer'image(j+1)&"  value", v_AI_data(j).STAT.FS_VOL_RB, c_ND, testResult);
      end loop;
      
      -------------------------------------------------------
      ------------------ CHECK ALL EVENTS -------------------
      -------------------------------------------------------
	    wait for 1000 us;
      FSI_temp <= x"4C031232";
	    wait until rising_edge(clk);
      for i in 0 to c_NUM_of_AI-1 loop
	      v_fsi_temp := to_integer(unsigned(FSI_temp))+i;
	      v_FSI_SUM := std_logic_vector(to_unsigned(v_fsi_temp, v_FSI_SUM'length));
          p_find_BIT_with_State(v_FSI_SUM, c_Event_appear,v_BIT_data, v_tmp_b, v_FSI_index);
	      p_check(test_nb, to_hex(v_FSI_SUM)& " appear", v_tmp_b, 1, testResult);
	      p_check(test_nb,"Check for AI " &integer'image(i+1)&"  Eng Data Lenght", v_BIT_data(v_FSI_index).EDL, x"0004", testResult);
	      p_check(test_nb,"Check for AI " &integer'image(i+1)&"  Sanction", v_BIT_data(v_FSI_index).Sanction, x"0000", testResult);
	      
	      V_CRC_16_cal := Calculate_EVENT_CRC16(v_BIT_data(v_FSI_index).FSI& v_BIT_data(v_FSI_index).SPARE_0&v_BIT_data(v_FSI_index).STATE&
	      v_BIT_data(v_FSI_index).SPARE_1&v_BIT_data(v_FSI_index).SANCTION&v_BIT_data(v_FSI_index).EDL&v_BIT_data(v_FSI_index).ED_0&
	      v_BIT_data(v_FSI_index).ED_1&v_BIT_data(v_FSI_index).ED_2&v_BIT_data(v_FSI_index).ED_3&v_BIT_data(v_FSI_index).ED_4&
	      v_BIT_data(v_FSI_index).ED_5&v_BIT_data(v_FSI_index).ED_6&v_BIT_data(v_FSI_index).ED_7&v_BIT_data(v_FSI_index).ED_8&
	      v_BIT_data(v_FSI_index).ED_9&v_BIT_data(v_FSI_index).SPARE_2&v_BIT_data(v_FSI_index).SPARE_3&v_BIT_data(v_FSI_index).SPARE_4);
	      p_check(test_nb,"Check for EVENT CRC value", v_BIT_data(v_FSI_index).CRC, V_CRC_16_cal, testResult);
	    end loop;	
    
      --------------------------------------------------------
      ------------------ CLEAR EVENT FIFO -------------------- 
      --------------------------------------------------------
	    BITE_message_buffer_clr <= '1';
	    wait for 10 ns;
	    BITE_message_buffer_clr <= '0';  	
    
      --------------------------------------------------------
      ----------- SET REF_VOLT TO A VALID VALUE --------------
      --------------------------------------------------------    
      ADS7953_CH0(0)  <= (2.048);
      ADS7953_CH14(0) <= (2.048);
      
      --------------------------------------------------------
      ------------------ SET ALL AIs TO 5V -------------------
      --------------------------------------------------------
	    for i in 0 to c_NUM_of_AI -1 loop
	      v_AI_data(i).CMD.VOLTAGE_CMD := 5.0;
        p_select_AI_cmd_voltage(0, AI_Voltage_Input, i, v_AI_data);
	    end loop;  
    
      --------------------------------------------------------
      -- WAIT FOR MORE THAN 16 MONITORING CYCLES + TIME TO ---
      --------------------------------------------------------
      ----------- INCREASE THE VOLTAGE FROM 0 TO 5 -----------
      --------------------------------------------------------
      wait for 18000 us;
      wait until falling_edge (S2P_CR(0));
      wait until rising_edge (S2P_CR(0));   
      --------------------------------------------------------
      ---- CHECK THE VOLTAGE READBACK OF ALL AIs IS 5.0V -----
      --------------------------------------------------------
      p_select_AI_readback (0, OctoSPI_Message_array, v_AI_data);
    
      for j in 0 to c_NUM_of_AI -1 loop
        p_check(test_nb,"Check for AI  " &integer'image(j+1)& " value", f_to_real(v_AI_data(j).STAT.VOL_RB), 5.0, 0.1, testResult);
        p_check(test_nb,"Check for AI FS  " &integer'image(j+1)&"  value", v_AI_data(j).STAT.FS_VOL_RB, c_NO, testResult);
      end loop;
      
      -------------------------------------------------------
      ------------------ CHECK ALL EVENTS -------------------
      ------------------------------------------------------- 
	    wait for 1000 us;
      FSI_temp <= x"4C031232";
	    wait until rising_edge(clk);
      for i in 0 to c_NUM_of_AI-1 loop
	      v_fsi_temp := to_integer(unsigned(FSI_temp))+i;
	      v_FSI_SUM := std_logic_vector(to_unsigned(v_fsi_temp, v_FSI_SUM'length));
          p_find_BIT_with_State(v_FSI_SUM, c_Event_disappear,v_BIT_data, v_tmp_b, v_FSI_index);
	      p_check(test_nb, to_hex(v_FSI_SUM)& " disappear", v_tmp_b, 1, testResult);
	      p_check(test_nb,"Check for AI " &integer'image(i+1)&"  Eng Data Lenght", v_BIT_data(v_FSI_index).EDL, x"0004", testResult);
	      p_check(test_nb,"Check for AI " &integer'image(i+1)&"  Sanction", v_BIT_data(v_FSI_index).Sanction, x"0000", testResult);
	      
	      V_CRC_16_cal := Calculate_EVENT_CRC16(v_BIT_data(v_FSI_index).FSI& v_BIT_data(v_FSI_index).SPARE_0&v_BIT_data(v_FSI_index).STATE&
	      v_BIT_data(v_FSI_index).SPARE_1&v_BIT_data(v_FSI_index).SANCTION&v_BIT_data(v_FSI_index).EDL&v_BIT_data(v_FSI_index).ED_0&
	      v_BIT_data(v_FSI_index).ED_1&v_BIT_data(v_FSI_index).ED_2&v_BIT_data(v_FSI_index).ED_3&v_BIT_data(v_FSI_index).ED_4&
	      v_BIT_data(v_FSI_index).ED_5&v_BIT_data(v_FSI_index).ED_6&v_BIT_data(v_FSI_index).ED_7&v_BIT_data(v_FSI_index).ED_8&
	      v_BIT_data(v_FSI_index).ED_9&v_BIT_data(v_FSI_index).SPARE_2&v_BIT_data(v_FSI_index).SPARE_3&v_BIT_data(v_FSI_index).SPARE_4);
	      p_check(test_nb,"Check for EVENT CRC value", v_BIT_data(v_FSI_index).CRC, V_CRC_16_cal, testResult);
	    end loop;	
      
      --------------------------------------------------------
      ------------------ CLEAR EVENT FIFO -------------------- 
      --------------------------------------------------------
	    BITE_message_buffer_clr <= '1';
	    wait for 10 ns;
	    BITE_message_buffer_clr <= '0';    
    
     
      wait until rising_edge(clk);
      p_SubTitle("====================================");
      if(testResult = '1') then
        p_SubTitle("=== <AI_0V_10V_1002> : PASS ===");
      else
        p_SubTitle("=== <AI_0V_10V_1002> : FAIL ===");
        GeneralResult <= '0';
      end if;
      p_SubTitle("====================================");
      p_SubTitle("");
      p_SubTitle("");
    end if;
  
  end process p_Stimuli;
  
end stimuli;