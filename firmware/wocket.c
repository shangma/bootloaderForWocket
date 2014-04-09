
#include <avr/eeprom.h> 
#include "mcu_atmega.h"
#include "wocket.h"
#include "crc.h"
#include <util/delay.h>

//buffer to store commands from the phone
unsigned char aBuffer[MAX_COMMAND_SIZE];
unsigned char rBuffer[MAX_COMMAND_SIZE];

unsigned char  command_counter = 0;	//counts how many bytes received for a command
unsigned char  command_length = 0;	//stores the length of a command
unsigned char  opcode;
unsigned int   command_timer = 0;
unsigned char  processed_counter = 0;
unsigned short address=0xffff;
unsigned char  response_length = 0;
unsigned char  paused = 0;
unsigned int   alive_timer = 0;
unsigned short word=0;

unsigned short x = 0;
unsigned short y = 0;
unsigned short z = 0;

unsigned tester = 0;
#define _WOCKET_INITIALIZED 0x25

uint8_t EEMEM _NV_INITIALIZED;
uint8_t EEMEM _NV_STATUS_BYTE;
uint8_t EEMEM _NV_SAMPLING_RATE;
uint8_t EEMEM _NV_TM;
uint8_t EEMEM _NV_WTM;
uint8_t EEMEM _NV_SENS;
uint8_t EEMEM _NV_TCT;
uint8_t EEMEM _NV_TCTREPS;
uint8_t EEMEM _NV_TCTLAST;

#define _DEFAULTBTCAL100 725   // Default Battery Calibration values
#define _DEFAULTBTCAL80  680
#define _DEFAULTBTCAL60  640
#define _DEFAULTBTCAL40  600
#define _DEFAULTBTCAL20  560
#define _DEFAULTBTCAL10  540

uint16_t EEMEM _NV_BTCAL100;
uint16_t EEMEM _NV_BTCAL80;
uint16_t EEMEM _NV_BTCAL60;
uint16_t EEMEM _NV_BTCAL40;
uint16_t EEMEM _NV_BTCAL20;
uint16_t EEMEM _NV_BTCAL10;

unsigned int _wBTCAL100;	  // Battery Calibration values
unsigned int _wBTCAL80;
unsigned int _wBTCAL60;
unsigned int _wBTCAL40;
unsigned int _wBTCAL20;
unsigned int _wBTCAL10;

#define _DEFAULT_X1G_CAL  500 // Default calibration values for each axis of the accelerometer 
#define _DEFAULT_XN1G_CAL 501
#define _DEFAULT_Y1G_CAL  502
#define _DEFAULT_YN1G_CAL 503
#define _DEFAULT_Z1G_CAL  504
#define _DEFAULT_ZN1G_CAL 505

uint16_t EEMEM _NV_X1G_CAL;
uint16_t EEMEM _NV_XN1G_CAL;
uint16_t EEMEM _NV_Y1G_CAL;
uint16_t EEMEM _NV_YN1G_CAL;
uint16_t EEMEM _NV_Z1G_CAL;
uint16_t EEMEM _NV_ZN1G_CAL;

unsigned int _wX1G_CAL;		  //  Calibration values for each axis of the accelerometer
unsigned int _wXN1G_CAL;
unsigned int _wY1G_CAL;
unsigned int _wYN1G_CAL;
unsigned int _wZ1G_CAL;
unsigned int _wZN1G_CAL;

unsigned char _INITIALIZED = 0;
unsigned char _STATUS_BYTE = 0;
unsigned char _SAMPLING_RATE = 40;	/*Current Sampling Rate is 40Hz and is set in _wocket_initialize() function*/
unsigned char _wTCNT2_reps = 1;
unsigned char _wTCNT2 = 0;
unsigned char _wTCNT2_last = 0;

unsigned char _TM;
unsigned char _wTM = _WTM_Continuous;//_WTM_Burst_60;
unsigned char _wSENS = _4_G;

unsigned long _wPC = 0;
unsigned long _wLastPC = 0;

unsigned char _LED_TIME = 0x3f;
unsigned char _LED_COLOR = 2;

uint8_t EEMEM _NV_PDT;
#define _DEFAULT_PDT 127
unsigned char _wPDT;

uint8_t EEMEM _NV_ALT;
unsigned char _wALT;

uint32_t _wShutdownTimer = 0;
uint32_t _DEFAULT_SHUTDOWN = 0;

unsigned char _MAX_SAMPLING_RATE = 0;
unsigned char _MIN_SAMPLING_RATE = 5;

//------------------------------------------------------Functions-----------------------------------------------------------------
/* 
	 The sampling rate is much slower than the MCU. MCU is configured to run at 8MHz (theoretically), in practice that is an 
	 approximate number. 
	 There are only 8/16 bit timers/counters in the wocket and for maximum power savings the 8 bit counter is used for setting the sampling
	 rate of the wocket accelerometer at a specified _SAMPLING_RATE value. But,the MCU provides a feature that would allow the user to 
	 increase the counter on each cycle with each power of 2 up to 2^10. Using the 2^10 mode, it is needed toincrement the counter is needed
	 to increase every 1024 cycles: (8000000/1024= 7812.5). So, the counter need to overflow with ticks=7812.5/_SAMPLING_RATE. When the counter overflows, an overflow interrupt gets invoked to show the approximate sampling time
	 for the accelerometer.	
*/
void _wocket_initialize_timer2_interrupt(void)
{
	unsigned short ticks = (unsigned short) ((F_CPU / 1024) / _SAMPLING_RATE); 	

	if (ticks > 256)
	{
		_wTCNT2 = 0;
		_wTCNT2_reps = (ticks / 256);
		_wTCNT2_last = 255 - (ticks % 256);
	}else
	{
		_wTCNT2= 255 - ticks;
		_wTCNT2_reps = 0;
		_wTCNT2_last = 255;
	}
}

//-------------------------------------------	

void _wocket_initialize(void) //This function initializes the wocket
{
	// Disable the watchdog timer. It has to be done at the beginning of the program.
	_atmega_disable_watchdog();
	_atmega_initialize(CPU_CLK_PRESCALAR_1024);
	num_skipped_timer_interrupts = 10;//(F_CPU/1024)/PERFECT_SAMPLING_FREQUENCY;
	
	unsigned short battery = _atmega_a2dConvert10bit(IN_VSENSE_BAT);
	if (battery < 700)
	{// Blink yellow 3times for 5 seconds if the battery is not fully charged 
		for (int i = 0; (i < 3); i++){
			_yellowled_turn_on();		
			for (int j = 0; (j < 200); j++)
				_delay_ms(5);
			_yellowled_turn_off();
			for (int j = 0; (j < 200); j++)
				_delay_ms(5);
		}
	}
	
	if (battery > 100)
	{//Load the status byte from the EEPROM  
		_INITIALIZED = eeprom_read_byte(&_NV_INITIALIZED);		
	}
	else
	{ //turn on the yellow led for 5 seconds then shutdown 
		_yellowled_turn_on();		
		for(int  i = 0; (i < 1000); i++)
			_delay_ms(5);
		_yellowled_turn_off();
		_delay_ms(500);
		_atmega_finalize();
		return;
	}
	
	// If the wocket has been initialized before, read the parameters from EEPROM and blinks green once	
	if (_INITIALIZED == _WOCKET_INITIALIZED)
	{		
		if (battery > 300)
		{
			_SAMPLING_RATE = eeprom_read_byte(&_NV_SAMPLING_RATE);
			_wTM = eeprom_read_byte(&_NV_WTM);

			_wTCNT2 = eeprom_read_byte(&_NV_TCT);
			_wTCNT2_reps = eeprom_read_byte(&_NV_TCTREPS);
			_wTCNT2_last = eeprom_read_byte(&_NV_TCTLAST);

			_wBTCAL100 = eeprom_read_word(&_NV_BTCAL100);
			_wBTCAL80  = eeprom_read_word(&_NV_BTCAL80);
			_wBTCAL60  = eeprom_read_word(&_NV_BTCAL60);
			_wBTCAL40  = eeprom_read_word(&_NV_BTCAL40);
			_wBTCAL20  = eeprom_read_word(&_NV_BTCAL20);
			_wBTCAL10  = eeprom_read_word(&_NV_BTCAL10);

			_wX1G_CAL  = eeprom_read_word(&_NV_X1G_CAL);
			_wXN1G_CAL = eeprom_read_word(&_NV_XN1G_CAL);
			_wY1G_CAL  = eeprom_read_word(&_NV_Y1G_CAL);
			_wYN1G_CAL = eeprom_read_word(&_NV_YN1G_CAL);
			_wZ1G_CAL  = eeprom_read_word(&_NV_Z1G_CAL);
			_wZN1G_CAL = eeprom_read_word(&_NV_ZN1G_CAL);

			_wPDT = eeprom_read_byte(&_NV_PDT);

			_greenled_turn_on();		
			for(int i = 0; (i < 200); i++)
				_delay_ms(10);
			_greenled_turn_off();
		}
	}

		
	// If the wocket has never been initialized, write the default settings and blink green 3 times 
	else
	{ 
		_SAMPLING_RATE = 40; 
		_wTM = _WTM_Continuous;
		//_wTM = _WTM_Burst_60;

		// Calculate the timer variables used to sample at the right frequency
		_wocket_initialize_timer2_interrupt();
		
		// Set the overflow interrupt timer 		
		switch(_wTM)
		{
			case _WTM_Continuous:	
				_MAX_SAMPLING_RATE = 126; //This limitation is due to the definition of SEND_SR  and GET_SR commands
				break;					  // The MCU provided in wocket able to sample at higher rates 								
			case _WTM_Burst_30:    	//a transfer mode that send the burst every 30 secs
				_MAX_SAMPLING_RATE = 80;		
				break;
			case _WTM_Burst_60:		//a transfer mode that send the burst every 60 secs
				_MAX_SAMPLING_RATE = 40;		
				break;
			case _WTM_Burst_90:		//a transfer mode that send the burst every 90 secs
				_MAX_SAMPLING_RATE = 30;		
				break;
			case _WTM_Burst_120:		//a transfer mode that send the burst every 120 secs
				_MAX_SAMPLING_RATE = 20;		
				break;
			default:
				break;
		}
		
		if (_SAMPLING_RATE > _MAX_SAMPLING_RATE)
		{
			_SAMPLING_RATE = _MAX_SAMPLING_RATE;
		
		}
		if (_SAMPLING_RATE < _MIN_SAMPLING_RATE)
		{
			_SAMPLING_RATE = _MIN_SAMPLING_RATE;		
		}
			
		
		if (battery > 300)
		{	
			// Write the parameters to the EEPROM
			eeprom_write_byte(&_NV_SAMPLING_RATE,_SAMPLING_RATE);
				
			eeprom_write_byte(&_NV_TCT,_wTCNT2);
			eeprom_write_byte(&_NV_TCTREPS,_wTCNT2_reps);
			eeprom_write_byte(&_NV_TCTLAST,_wTCNT2_last);			

			eeprom_write_byte(&_NV_WTM,_wTM);
			eeprom_write_byte(&_NV_STATUS_BYTE,0x00);
			eeprom_write_byte(&_NV_SENS,_wSENS);

			//Set default battery calibration values
			eeprom_write_word(&_NV_BTCAL100,_DEFAULTBTCAL100);
			eeprom_write_word(&_NV_BTCAL80, _DEFAULTBTCAL80);
			eeprom_write_word(&_NV_BTCAL60, _DEFAULTBTCAL60);
			eeprom_write_word(&_NV_BTCAL40, _DEFAULTBTCAL40);
			eeprom_write_word(&_NV_BTCAL20, _DEFAULTBTCAL20);
			eeprom_write_word(&_NV_BTCAL10, _DEFAULTBTCAL10);

			_wBTCAL100 = _DEFAULTBTCAL100;
			_wBTCAL80  = _DEFAULTBTCAL80;
			_wBTCAL60  = _DEFAULTBTCAL60;
			_wBTCAL40  = _DEFAULTBTCAL40;
			_wBTCAL20  = _DEFAULTBTCAL20;
			_wBTCAL10  = _DEFAULTBTCAL10;

			//Set default Accelerometer calibration values
			eeprom_write_word(&_NV_X1G_CAL, _DEFAULT_X1G_CAL);
			eeprom_write_word(&_NV_XN1G_CAL,_DEFAULT_XN1G_CAL);
			eeprom_write_word(&_NV_Y1G_CAL, _DEFAULT_Y1G_CAL);
			eeprom_write_word(&_NV_YN1G_CAL,_DEFAULT_YN1G_CAL);
			eeprom_write_word(&_NV_Z1G_CAL, _DEFAULT_Z1G_CAL);
			eeprom_write_word(&_NV_ZN1G_CAL,_DEFAULT_ZN1G_CAL);

			_wX1G_CAL  = _DEFAULT_X1G_CAL;
			_wXN1G_CAL = _DEFAULT_XN1G_CAL;
			_wY1G_CAL  = _DEFAULT_Y1G_CAL;
			_wYN1G_CAL = _DEFAULT_YN1G_CAL;
			_wZ1G_CAL  = _DEFAULT_Z1G_CAL;
			_wZN1G_CAL = _DEFAULT_ZN1G_CAL;

			_wPDT = _DEFAULT_PDT;
			eeprom_write_byte(&_NV_PDT, _wPDT);
		}

		// Set the initialized flag in the status byte
		_INITIALIZED = _WOCKET_INITIALIZED;

		// Write the status byte to the EEPROM		
		eeprom_write_byte(&_NV_INITIALIZED,_INITIALIZED);
				
		// Blink green for 5 seconds	
		for (int i = 0; (i < 3); i++){
			_greenled_turn_on();		
			for(int j = 0;(j < 200); j++)
				_delay_ms(5);
			_greenled_turn_off();
			for(int j = 0; (j < 200); j++)
				_delay_ms(5);
		}		
	}

	_DEFAULT_SHUTDOWN = (unsigned long)_wPDT * (unsigned long)_SAMPLING_RATE * (unsigned long)60;
	_wShutdownTimer = _DEFAULT_SHUTDOWN;
	
    // Enable Timer 2 
    _atmega_enable_timer2(CPU_CLK_PRESCALAR_1024); 
}

//-------------------------------------------
void _wocket_set_flag(unsigned char flag)
{
	sbi(_STATUS_BYTE, flag);
}

//-------------------------------------------
void _wocket_reset_flag(unsigned char flag)
{
	cbi(_STATUS_BYTE, flag);
}

//-------------------------------------------
unsigned char _wocket_is_flag_set(unsigned char flag)
{	
	return ((_STATUS_BYTE >> flag) & 0x01);
}

//-------------------------------------------
// sends uncompressed data via bluetooth (in both Continuous and Burst modes)
void _send_uncompressed_pdu(unsigned short x, unsigned short y, unsigned short z)
{
	aBuffer[0] = 0x80 | ((x >> 8) & 0x03);
	_bluetooth_transmit_uart0_byte(aBuffer[0]);
	aBuffer[1] = ((unsigned char) ((x >> 1) & 0x7f));
	_bluetooth_transmit_uart0_byte(aBuffer[1]);
	aBuffer[2] = ((unsigned char) ((x << 6) & 0x40)) | ((unsigned char) ((y >> 4) & 0x3f));
	_bluetooth_transmit_uart0_byte(aBuffer[2]);
	aBuffer[3] = ((unsigned char) ((y << 3) & 0x78)) | ((unsigned char) ((z >> 7) & 0x07));
	_bluetooth_transmit_uart0_byte(aBuffer[3]);
	aBuffer[4] = ((unsigned char) (z & 0x7f));
	_bluetooth_transmit_uart0_byte(aBuffer[4]);
}

//-------------------------------------------
// 	sends compressed data via bluetooth(in both Continuous and Burst modes)
void _send_compressed_pdu(unsigned char x, unsigned char y, unsigned char z)
{
	aBuffer[0] = 0xe0| ((x >> 1) & 0x1f);
	_bluetooth_transmit_uart0_byte(aBuffer[0]);
	aBuffer[1] = ((x & 0x01) << 6) | (y & 0x3f);
	_bluetooth_transmit_uart0_byte(aBuffer[1]);
	aBuffer[2] = (z << 1) & 0x7e;
	_bluetooth_transmit_uart0_byte(aBuffer[2]);	
}

//-------------------------------------------
//	sends the number of stored raw data 
void _send_batch_count(unsigned short count)
{
    aBuffer[0] = m_BC_RSP_BYTE0;
    aBuffer[1] = m_BC_RSP_BYTE1(count);
    aBuffer[2] = m_BC_RSP_BYTE2(count);
	aBuffer[3] = m_BC_RSP_BYTE3(count);
	for (int i = 0; (i < 4); i++)                                                                                       
       	_bluetooth_transmit_uart0_byte(aBuffer[i]);  
}

//-------------------------------------------
// 	sends the number of activity counts 
void _send_ac_count(unsigned short count)
{ 
    aBuffer[0] = m_ACC_RSP_BYTE0;
    aBuffer[1] = m_ACC_RSP_BYTE1(count);
    aBuffer[2] = m_ACC_RSP_BYTE2(count);
	for (int i = 0; (i < 3); i++)                                                                                       
       	_bluetooth_transmit_uart0_byte(aBuffer[i]);  
}

//-------------------------------------------
//	Activity count offset 
void _send_ac_offset(unsigned short offset)
{
    aBuffer[0] = m_OFT_RSP_BYTE0;
    aBuffer[1] = m_OFT_RSP_BYTE1(offset);
    aBuffer[2] = m_OFT_RSP_BYTE2(offset);
	for (int i = 0; (i < 3); i++)                                                                                       
       	_bluetooth_transmit_uart0_byte(aBuffer[i]); 
}

//-------------------------------------------
//	sends the firmware version 
void _send_fv()
{
    aBuffer[0] = m_FV_RSP_BYTE0;
    aBuffer[1] = m_FV_RSP_BYTE1(_FVERSION);    
	for (int i = 0; (i < 2); i++)                                                                                       
       	_bluetooth_transmit_uart0_byte(aBuffer[i]); 
}

//-------------------------------------------
// 	sends the hardware version
void _send_hv()
{
    aBuffer[0] = m_HV_RSP_BYTE0;
    aBuffer[1] = m_HV_RSP_BYTE1(_VERSION);    
	for (int i = 0; (i < 2); i++)                                                                                       
       	_bluetooth_transmit_uart0_byte(aBuffer[i]); 
}

//-------------------------------------------
// 	sends the battery level
void _send_bl(unsigned short level)
{
   aBuffer[0] = m_BL_RSP_BYTE0;
   aBuffer[1] = m_BL_RSP_BYTE1(level);
   aBuffer[2] = m_BL_RSP_BYTE2(level);
   for (int i = 0; (i < 3); i++)
       _bluetooth_transmit_uart0_byte(aBuffer[i]);
}

//-------------------------------------------
// sends the Activity counts
void _send_acs()
{
	unsigned short count   = 0;
	unsigned short seq_num = sseq;
	unsigned short num_acs = 0;
	//unsigned char  counter = 0;
    //Determine how many new acs need to be sent
	if (ci > si)
		num_acs = ci - si;
	else
		num_acs = ci + (AC_BUFFER_SIZE - si);
	
	//send AC offset accumulated within the minute & the overall AC sequence number
	_send_ac_offset(AC_NUMS - summary_count); 
	_send_ac_count(cseq);		

    //send the acs from start to current ac index
	for (int i = si;(i != ci);)
	{		
		count = acount[i];
		aBuffer[0] = m_AC_RSP_BYTE0;
    	aBuffer[1] = m_AC_RSP_BYTE1(seq_num);
    	aBuffer[2] = m_AC_RSP_BYTE2(seq_num);
		aBuffer[3] = m_AC_RSP_BYTE3(seq_num,count);
		aBuffer[4] = m_AC_RSP_BYTE4(count);
		aBuffer[5] = m_AC_RSP_BYTE5(count);

		for (int j = 0; (j < 6); j++)                                                                                       
       		_bluetooth_transmit_uart0_byte(aBuffer[j]);
			 
		i++;
		if (i == AC_BUFFER_SIZE)
			i = 0;
		seq_num++;


		/*// SAM TODO: check that this if is not needed
		if (num_acs < 60)
		{
			counter++;
			if (counter == 10)
				return;
		}*/
	}
	_receive_data();
}

//-------------------------------------------
// sends confirmation of end of batch data for bursty mode
void _send_end_batch()
{
	aBuffer[0] = m_END_BATCH_BYTE0;    	
    _bluetooth_transmit_uart0_byte(aBuffer[0]); 
}

//-------------------------------------------
// sends sampling rate
void _send_sr()
{
	aBuffer[0] = m_SR_RSP_BYTE0;
    aBuffer[1] = m_SR_RSP_BYTE1(_SAMPLING_RATE);
	for (int i = 0; (i < 2); i++)                                                                                       
       	_bluetooth_transmit_uart0_byte(aBuffer[i]); 
}

//-------------------------------------------
//	sends wocket transmission mode 
void _send_wtm()
{ 
	aBuffer[0] = m_WTM_RSP_BYTE0;
    aBuffer[1] = m_WTM_RSP_BYTE1(_wTM);
	for (int i = 0; (i < 2); i++)                                                                                       
       	_bluetooth_transmit_uart0_byte(aBuffer[i]); 
}

//-------------------------------------------
// 	Receive commands from the phone	
void _receive_data(void)
{
	unsigned char aByte;

	// Attempt to receive a byte only if no command is being received or a partial command has been received
	// This line was replaced with results from Harshit experiments
	// if ( ((command_counter==0)||(command_counter<command_length))  && (_bluetooth_receive_uart0_byte(&aByte)) )
    
	if(_bluetooth_receive_uart0_byte(&aByte))
	{
		rBuffer[command_counter++] = aByte;
				
		if ((aByte >> 5) == COMMAND_PREFIX)
    	{
        	opcode = aByte & 0x1f;                                              
        	switch (opcode)
			{
            	case (unsigned char)GetBatteryLevel:
                case (unsigned char)GetBatteryPercent:
                case (unsigned char)GetPacketCount:
                case (unsigned char)ResetBluetooth:
                case (unsigned char)GetSensorSentivity:
                case (unsigned char)GetCalibrationValues:
                case (unsigned char)GetSamplingRate:
                case (unsigned char)GetAliveTimer:
                case (unsigned char)GetPowerDownTimer:
                case (unsigned char)ResetWocket:                      
                case (unsigned char)Alive:
                case (unsigned char)PausePacket:
				case (unsigned char)ResumePacket:
				case (unsigned char)GetRadioTransmissionMode:
				case (unsigned char)GetWocketTransmissionMode:
				case (unsigned char)GetBatteryCalibration:
				case (unsigned char)GetHardwareVersion:
				case (unsigned char)GetFirmwareVersion:				
				case (unsigned char)GetTCT:
				case (unsigned char)ShutdownWocket:
                	command_length = 1;
                    break;
                case (unsigned char)SetSensorSentivity:
                case (unsigned char)SetSamplingRate:            
                case (unsigned char)SetAliveTimer:
                case (unsigned char)SetPowerDownTimer:
                case (unsigned char)SetRadioTransmissionMode:                
				case (unsigned char)SetWocketTransmissionMode:  
				case (unsigned char)SetLED:
                     command_length = 2;
                     break;
     			case (unsigned char)ACK:
					 command_length = 4;
                     break;
				case (unsigned char)SetTCT:                
                     command_length = 5;
                     break;
                case (unsigned char)SetCalibrationValues:
				case (unsigned char)SetBatteryCalibration:
                      command_length = 10;                                                              
                      break;                                                          
    		}
    		command_counter = 1;
    		command_timer = 0;
    		processed_counter = 0;                                            
    		address = 0xffff;
    		response_length = 0;
		}
	}

	// increment timer as long as the command is still being received
    if (command_counter > 0)
		command_timer++;

 	//if all command is received, start processing it
    if ((command_counter > 0) && (command_counter == command_length))
    {                                       
            switch (opcode)
            {
				case (unsigned char) ACK:																
					kseq = rBuffer[1] & 0x7f;
					kseq = kseq << 7 | (rBuffer[2] & 0x7f);
					kseq = kseq << 2 | ((rBuffer[3] >> 5) & 0x03);
					kseq ++;
					if ( (kseq <= cseq) && ((kseq - sseq) < AC_BUFFER_SIZE) && ((kseq - sseq) > 0) ) {					
						sseq = kseq;						
						dseq = cseq - kseq;
						if (dseq >= 0)
							si = ci - dseq;
						else
							si = AC_BUFFER_SIZE - (dseq - ci);
					}
					processed_counter = command_counter;
					break;	

		        //This command calibrates the wocket sampling rate by determining how much off it is from the 
				//_SAMPLING_RATE by counting timer interrupts and using that value to adjust the sampling rate.
			    case (unsigned char) SetTCT:  
			   		_wTCNT2 = m_SET_TCT (rBuffer[1], rBuffer[2]);
					_wTCNT2_reps = m_SET_TCTREPS (rBuffer[2], rBuffer[3]);
					_wTCNT2_last = m_SET_TCTLAST (rBuffer[3], rBuffer[4]);
					eeprom_write_byte (&_NV_TCT, _wTCNT2);
					eeprom_write_byte (&_NV_TCTREPS, _wTCNT2_reps);
					eeprom_write_byte (&_NV_TCTLAST, _wTCNT2_last);
					processed_counter = command_counter;
					_yellowled_turn_on();	
					_delay_ms(500);						
					_yellowled_turn_off(); 
					_delay_ms(500);
					break;

                case (unsigned char)PausePacket:                                                      
                    paused = 1;
                    processed_counter = command_counter;                                                      
                    break;

                case (unsigned char)ResumePacket:                                                     
                    paused = 0;
                    processed_counter = command_counter;                                                      
                    break;

                //reset alive timer if it is alive
                case (unsigned char)Alive:                                                      
                    alive_timer = 0;
                    processed_counter = command_counter;		
                    break;

            	case (unsigned char) GetBatteryLevel: 
                    word = _atmega_a2dConvert10bit(ADC7);
                    aBuffer[0] = m_BL_RSP_BYTE0;
                    aBuffer[1] = m_BL_RSP_BYTE1(word);
                    aBuffer[2] = m_BL_RSP_BYTE2(word);
                    processed_counter = command_counter;
                    response_length = 3;		                                                                          
                    break;	
						
            	case (unsigned char) GetSensorSentivity:           			  
                    aBuffer[0] = m_SENS_RSP_BYTE0;
                    aBuffer[1] = m_SENS_RSP_BYTE1(_wSENS);                       
                    processed_counter = command_counter;
                    response_length = 2;		                                                                          
                    break;
						
				case (unsigned char) SetSensorSentivity:           			  
                    _wSENS = m_SET_SEN(rBuffer[1]);
					eeprom_write_byte(&_NV_SENS, _wSENS);                       
                    processed_counter = command_counter;                                                                 
                    break;

            	case (unsigned char) GetBatteryPercent: 				
                    word = _atmega_a2dConvert10bit(ADC7);								  
					if (word > _wBTCAL100) // Calculate the battery percent
						word = 100;
					else if (word > _wBTCAL80)
						word = 80 + ((word - _wBTCAL80) * 20) / (_wBTCAL100 - _wBTCAL80);
					else if (word > _wBTCAL60)
						word = 60 + ((word - _wBTCAL60) * 20) / (_wBTCAL80  - _wBTCAL60);
					else if (word > _wBTCAL40)
						word = 40 + ((word - _wBTCAL40) * 20) / (_wBTCAL60  - _wBTCAL40);
					else if (word > _wBTCAL20)
						word = 20 + ((word - _wBTCAL20) * 20) / (_wBTCAL40  - _wBTCAL20);
					else if (word > _wBTCAL10)
						word = 10 + ((word - _wBTCAL10) * 10) / (_wBTCAL20  - _wBTCAL10);
					else
						word = 0;

                    aBuffer[0] = m_BP_RSP_BYTE0;
                    aBuffer[1] = m_BP_RSP_BYTE1(word);
                    processed_counter = command_counter;
                    response_length = 2;		                                                                          
                    break;
			   
			   case (unsigned char) GetPowerDownTimer:  
			   		aBuffer[0] = m_PDT_RSP_BYTE0;
                    aBuffer[1] = m_PDT_RSP_BYTE1(_wPDT);
					processed_counter = command_counter;
					response_length = 2;
					break;	
							
               case (unsigned char) SetPowerDownTimer:  
			   		_wPDT = m_SET_PDT(rBuffer[1]);
					eeprom_write_byte(&_NV_PDT, _wPDT);
					processed_counter = command_counter;
					break;
																									
               case (unsigned char) GetSamplingRate:  
			   		aBuffer[0] = m_SR_RSP_BYTE0;
                    aBuffer[1] = m_SR_RSP_BYTE1(_SAMPLING_RATE);
					processed_counter = command_counter;
					response_length = 2;
					break;

               case (unsigned char) GetPacketCount:  
			   		aBuffer[0] = m_PC_RSP_BYTE0;
                    aBuffer[1] = m_PC_RSP_BYTE1(_wPC);
					aBuffer[2] = m_PC_RSP_BYTE2(_wPC);
                    aBuffer[3] = m_PC_RSP_BYTE3(_wPC);
					aBuffer[4] = m_PC_RSP_BYTE4(_wPC);
                    aBuffer[5] = m_PC_RSP_BYTE5(_wPC);
					processed_counter = command_counter;
					response_length = 6;
					break;

               case (unsigned char) SetSamplingRate:  
			   		_SAMPLING_RATE = m_SET_SR(rBuffer[1]);
					if (_SAMPLING_RATE > _MAX_SAMPLING_RATE)
					{
						_SAMPLING_RATE = _MAX_SAMPLING_RATE;		
					}
					if (_SAMPLING_RATE < _MIN_SAMPLING_RATE)
					{
						_SAMPLING_RATE = _MIN_SAMPLING_RATE;		
					}
					_wocket_initialize_timer2_interrupt();
					eeprom_write_byte(&_NV_SAMPLING_RATE, _SAMPLING_RATE);
					eeprom_write_byte(&_NV_TCT, _wTCNT2);
					eeprom_write_byte(&_NV_TCTREPS, _wTCNT2_reps);
					eeprom_write_byte(&_NV_TCTLAST, _wTCNT2_last);
					processed_counter = command_counter;
					break;

 			   case (unsigned char) GetRadioTransmissionMode:  
			   		aBuffer[0] = m_TM_RSP_BYTE0;
                    aBuffer[1] = m_TM_RSP_BYTE1(_TM);
					processed_counter = command_counter;
					response_length = 2;
					break;

               case (unsigned char) SetRadioTransmissionMode:  
			   		_TM = m_SET_TM(rBuffer[1]);
					eeprom_write_byte(&_NV_TM, _TM);
					processed_counter = command_counter;
					break;

			   case (unsigned char) GetWocketTransmissionMode:  
			   		aBuffer[0] = m_WTM_RSP_BYTE0;
                    aBuffer[1] = m_WTM_RSP_BYTE1(_wTM);
					processed_counter = command_counter;
					response_length = 2;
					break;

               case (unsigned char) SetWocketTransmissionMode:  
			   		_wTM = m_SET_WTM(rBuffer[1]);
					eeprom_write_byte(&_NV_WTM, _wTM);
					switch(_wTM)
					{
						case _WTM_Continuous:	
							_MAX_SAMPLING_RATE = 126; //This limitation is due to the definition of SEND_SR  and GET_SR commands
							break;					  // The MCU provided in wocket able to sample at higher rates 							
						case _WTM_Burst_30:    	//a transfer mode that send the burst every 30 secs
							_MAX_SAMPLING_RATE = 80;		
							break;
						case _WTM_Burst_60:		//a transfer mode that send the burst every 60 secs
							_MAX_SAMPLING_RATE = 40;		
							break;
						case _WTM_Burst_90:		//a transfer mode that send the burst every 90 secs
							_MAX_SAMPLING_RATE = 30;		
							break;
						case _WTM_Burst_120:	//a transfer mode that send the burst every 120 secs
							_MAX_SAMPLING_RATE = 20;		
							break;
						default:
							break;
					}						
					processed_counter = command_counter;
					break;

               case (unsigned char) SetCalibrationValues:                                                                    
                    if (eeprom_is_ready())
                    {                                    
                        if (_atmega_a2dConvert10bit(ADC7) < 600)
                                break;
                        else
                        {   
							_wX1G_CAL  = m_SET_CAL_x1g(rBuffer[1], rBuffer[2]);
							eeprom_write_word(&_NV_X1G_CAL,_wX1G_CAL);
							_wXN1G_CAL = m_SET_CAL_xn1g(rBuffer[2], rBuffer[3]);
							eeprom_write_word(&_NV_XN1G_CAL,_wXN1G_CAL);
							_wY1G_CAL  = m_SET_CAL_y1g(rBuffer[3], rBuffer[4], rBuffer[5]);
							eeprom_write_word(&_NV_Y1G_CAL,_wY1G_CAL);
							_wYN1G_CAL = m_SET_CAL_yn1g(rBuffer[5], rBuffer[6]);
							eeprom_write_word(&_NV_YN1G_CAL, _wYN1G_CAL);
							_wZ1G_CAL  = m_SET_CAL_z1g(rBuffer[6], rBuffer[7], rBuffer[8]);
							eeprom_write_word(&_NV_Z1G_CAL, _wZ1G_CAL);
							_wZN1G_CAL = m_SET_CAL_zn1g(rBuffer[8], rBuffer[8]);
							eeprom_write_word(&_NV_ZN1G_CAL, _wZN1G_CAL);
							processed_counter = command_counter;
                        }                                                                                                                                  
                    }                                                                                                                       
                    //enable global interrupts
                    break;

                case (unsigned char) GetCalibrationValues:							                                                              
					aBuffer[0] = m_CAL_RSP_BYTE0;
                    aBuffer[1] = m_CAL_RSP_BYTE1_x1g(_wX1G_CAL);                                                                   
                    aBuffer[2] = m_CAL_RSP_BYTE2_x1g(_wX1G_CAL);
					aBuffer[2]|= m_CAL_RSP_BYTE2_xn1g(_wXN1G_CAL);
                    aBuffer[3] = m_CAL_RSP_BYTE3_xn1g(_wXN1G_CAL);
					aBuffer[3]|= m_CAL_RSP_BYTE3_y1g(_wY1G_CAL);
                    aBuffer[4] = m_CAL_RSP_BYTE4_y1g(_wY1G_CAL);
                    aBuffer[5] = m_CAL_RSP_BYTE5_y1g(_wY1G_CAL);
                    aBuffer[5]|= m_CAL_RSP_BYTE5_yn1g(_wYN1G_CAL);
                    aBuffer[6] = m_CAL_RSP_BYTE6_yn1g(_wYN1G_CAL);
					aBuffer[6]|= m_CAL_RSP_BYTE6_z1g(_wZ1G_CAL);
                    aBuffer[7] = m_CAL_RSP_BYTE7_z1g(_wZ1G_CAL);
                    aBuffer[8] = m_CAL_RSP_BYTE8_z1g(_wZ1G_CAL);
					aBuffer[8]|= m_CAL_RSP_BYTE8_zn1g(_wZN1G_CAL);
                    aBuffer[9] = m_CAL_RSP_BYTE9_zn1g(_wZN1G_CAL);
					processed_counter = command_counter;
                    response_length = 10;                                                                               
                    break;    

                case (unsigned char) GetBatteryCalibration:							                                                              
					aBuffer[0] = m_BTCAL_RSP_BYTE0;
                    aBuffer[1] = m_BTCAL_RSP_BYTE1_100(_wBTCAL100);                                                                   
                    aBuffer[2] = m_BTCAL_RSP_BYTE2_100(_wBTCAL100);
					aBuffer[2]|= m_BTCAL_RSP_BYTE2_80(_wBTCAL80);
                    aBuffer[3] = m_BTCAL_RSP_BYTE3_80(_wBTCAL80);
					aBuffer[3]|= m_BTCAL_RSP_BYTE3_60(_wBTCAL60);
                    aBuffer[4] = m_BTCAL_RSP_BYTE4_60(_wBTCAL60);
                    aBuffer[5] = m_BTCAL_RSP_BYTE5_60(_wBTCAL60);
                    aBuffer[5]|= m_BTCAL_RSP_BYTE5_40(_wBTCAL40);
                    aBuffer[6] = m_BTCAL_RSP_BYTE6_40(_wBTCAL40);
					aBuffer[6]|= m_BTCAL_RSP_BYTE6_20(_wBTCAL20);
                    aBuffer[7] = m_BTCAL_RSP_BYTE7_20(_wBTCAL20);
                    aBuffer[8] = m_BTCAL_RSP_BYTE8_20(_wBTCAL20);
					aBuffer[8]|= m_BTCAL_RSP_BYTE8_10(_wBTCAL10);
                    rBuffer[9] = m_BTCAL_RSP_BYTE9_10(_wBTCAL10);
					processed_counter = command_counter;
                    response_length = 10;                                                                               
                    break;  

				case (unsigned char) SetBatteryCalibration:
				/*All of the read/write functions first make sure the EEPROM is ready to be accessed. 
				Since this may cause long delays if a write operation is still pending, timecritical 
				applications should first poll the EEPROM e. g. using eeprom_is_ready()	*/
                    if (eeprom_is_ready())
                    {                                    
                        if (_atmega_a2dConvert10bit(ADC7) < 600) 
                	        break;
                        else
                        {   
							_wBTCAL100 = m_SET_BTCAL_100(rBuffer[1], rBuffer[2]);
							eeprom_write_word(&_NV_BTCAL100, _wBTCAL100);
							_wBTCAL80  = m_SET_BTCAL_80(rBuffer[2], rBuffer[3]);
							eeprom_write_word(&_NV_BTCAL80, _wBTCAL80);
							_wBTCAL60  = m_SET_BTCAL_60(rBuffer[3], rBuffer[4], rBuffer[5]);
							eeprom_write_word(&_NV_BTCAL60, _wBTCAL60);
							_wBTCAL40  = m_SET_BTCAL_40(rBuffer[5], rBuffer[6]);
							eeprom_write_word(&_NV_BTCAL40, _wBTCAL40);
							_wBTCAL20  = m_SET_BTCAL_20(rBuffer[6], rBuffer[7], rBuffer[8]);
							eeprom_write_word(&_NV_BTCAL20, _wBTCAL20);
							_wBTCAL10  = m_SET_BTCAL_10(rBuffer[8], rBuffer[8]);
							eeprom_write_word(&_NV_BTCAL10, _wBTCAL10);
							processed_counter = command_counter;
                        }                                                                                               
                    }
                    break;
						
   				case (unsigned char) GetHardwareVersion:  
			   		aBuffer[0] = m_HV_RSP_BYTE0;
                    aBuffer[1] = m_HV_RSP_BYTE1(_VERSION);
					processed_counter = command_counter;		
					response_length = 2;
					break;				
			
				case (unsigned char) GetFirmwareVersion:  
			   		aBuffer[0] = m_FV_RSP_BYTE0;
                    aBuffer[1] = m_FV_RSP_BYTE1(_FVERSION);
					processed_counter = command_counter;
					response_length=2;
					break;	
			
				case (unsigned char) GetTCT:  
			      	aBuffer[0] = m_TCT_RSP_BYTE0;
					aBuffer[1] = m_TCT_RSP_BYTE1(_wTCNT2);
					aBuffer[2] = m_TCT_RSP_BYTE2(_wTCNT2, _wTCNT2_reps);
					aBuffer[3] = m_TCT_RSP_BYTE3(_wTCNT2_reps, _wTCNT2_last);
					aBuffer[4] = m_TCT_RSP_BYTE4(_wTCNT2_last);
					processed_counter = command_counter;				
					response_length = 5;
					break;
			
				case (unsigned char) SetLED:  
			      	_LED_COLOR = m_SET_LED_COLOR(rBuffer[1]);
					_LED_TIME = m_SET_LED_TIME(rBuffer[1]);
					if (_LED_COLOR == 0){						
						for (int j=0; j<(_LED_TIME); j++){
							_yellowled_turn_on();	
							_delay_ms(500);						
							_yellowled_turn_off(); 
							_delay_ms(500);
						}
					} if (_LED_COLOR == 1){
						for (int j=0; j<(_LED_TIME); j++){
							_greenled_turn_on();	
							_delay_ms(500);						
							_greenled_turn_off(); 
							_delay_ms(500);
						}
					} 
					processed_counter = command_counter;
					break;
			
				case (unsigned char) ShutdownWocket:  
					shutdown_flag = 1;
					_bluetooth_turn_off();
					_greenled_turn_off();
					_yellowled_turn_off();			      	
					processed_counter = command_counter;
					break;
				
				case (unsigned char) ResetBluetooth:  
			      	_bluetooth_reset();
					processed_counter = command_counter;
					break;

				case (unsigned char) ResetWocket:  
			      	//_wocket_reset();
					/*_atmega_finalize();
					for (int j = 0; (j < 1000); j++)
						_delay_ms(5);*/
					_wocket_initialize();
					processed_counter = command_counter;
					break;

				case (unsigned char) GetAliveTimer:  
			   		aBuffer[0] = m_ALT_RSP_BYTE0;
                    aBuffer[1] = m_ALT_RSP_BYTE1(_wALT);
					processed_counter = command_counter;
					response_length = 2;
					break;	
							
                case (unsigned char) SetAliveTimer:  
			   		_wALT = m_SET_ALT(rBuffer[1]);
					eeprom_write_byte(&_NV_ALT, _wALT);
					processed_counter = command_counter;
					break;	
																					 							                              
                default:        
                    break;
            }

            if (processed_counter == command_counter)
			{                              
                    for (int i = 0; (i < response_length); i++)                                                                                       
                     	_bluetooth_transmit_uart0_byte(aBuffer[i]);                                                                                                                                                   
                    command_length = 0;
                    command_counter = 0;
                    command_timer = 0;
                    processed_counter = 0;                                            
                    address = 0xffff;
                    response_length = 0;                    
            }
    } //if command timed out
    else if ((command_timer >= MAX_COMMAND_TIMER))
    {                            
        command_length = 0;
        command_counter = 0;
        command_timer = 0;
        processed_counter = 0;                            
        address=0xffff;
        response_length = 0;
    }
}
