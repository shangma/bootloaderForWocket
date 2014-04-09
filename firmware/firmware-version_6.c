//*****************************************************************************
//
// File Name    : 'firmware-version_6.c'
// Title        : Interrupt driven code for the wockets
// Author       : Fahd Albinali, Aida Ehyaei
// Created      : 12/10/2008
// Modified     : 08/01/2012
// 
//  Description : This file includes the main loop for the code that runs on the wocket and
//  any interrupt service routines (ISRs). The code supports a variety of configuration modes
//  for the wockets including Master/Slave modes and supports low and high power configurations                 
//
// This code is distributed under the MIT License
//     
//
//*****************************************************************************

#include <stdlib.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h> 

#include <avr/power.h>
#include "mcu_atmega.h"
#include "wocket.h"
#include <util/delay.h>

//-----------------------------------Variables------------------------------------------------------------------------------

/*We have a maximum of 16K on the microcontroller. The storage capacity of depends on the sampling rate and that can vary.
With the sampling rate of 40Hz, the memory is allocated as follows:
- approximately 1K is allocated for summary data over 16 hours... specifically 960 bytes.

- 11K for raw data. 750 data units allows us to store over 1.25 Minute of raw data without overflowing.  
(750 x 4)/(60sec x 40sample) = 1.25 min 

12K and the rest goes for the heap and stack.*/

data_unit      data[DATA_SIZE];	/* This structure is used to store raw data. Each data unit (15bytes) consists of 4 samples. 
								Because the accelerometer is sampled using a 10 bit ADC. So, a sample consists of 30 bits for
								the 3 axes. 30x4= 120 bits i.e. 15 bytes.*/
unsigned short batch_counter = 0;// Counter of rows of raw data in bursty mode; if the number of rows increase DATA_SIZE, the batch_counter remains on DATA_SIZE.
unsigned short dataIndex = 0;	 // Used for data buffer; Limit is DATA_SIZE 
unsigned char  dataSubindex = 0; // Used for sub data buffer and has a max value of 4; Each row of matrix data[] saves 4 raw data.

short rawHead = 0;
short rawTail = 0;

unsigned short acount[AC_BUFFER_SIZE]; /* A buffer  with capacity of 960 bytes for storing activity counts up to 16 hours in
									   case the wocket losses the Blutooth connection (with phone)*/ 
unsigned short AC_NUMS = 0;
unsigned short summary_count;
unsigned short AC_SCALING = 24; /* The scaling factor is used to prevent AC to overflow. Based on experimentation with a 4g 
								accelerometer  with a 10-bit ADC, 24 was determined to be a reasonable scaling factor */
unsigned int AC_CEILING = 65535;

/*  ci, si, cseq and sseq: A circular buffer for the activity counts.  
In some older hardware versions, the wockets used to reset easily, which caused the sseq to reset and therefore 
the receiver would get out of sync seeing small seq numbers that were already ACKed. 
In the code, there is a test done to deal with that situation. */
unsigned short cseq = 0;		// the sequence numbers of the last AC
unsigned short sseq = 0;		// the next sequence number that need to be sent to phone
unsigned short kseq = 0;		// the acknowledged sequence number 
unsigned short dseq = 0;		// the difference between the last AC and the new Acknowledged seq: cseq - kseq
unsigned short ci   = 0;		// the pointer in the circular buffer to the last AC. It corresponds to cseq
unsigned short si   = 0;		// the pointer in the circular buffer to the next element to be sent, it corresponds to sseq(i.e. the next value to be sent
								// out or was sent out but no acknowledgment received from the phone).

unsigned short battery; 		// Contains battery level after sampling the battery 

unsigned short x;				// Acceleration values of x, y and z axis
unsigned short y;
unsigned short z;

unsigned int i = 0;

unsigned short prevx;			// Previous acceleration values 
unsigned short prevy;				
unsigned short prevz;	

unsigned short diffx;			// difference of the current and previous acceleration values 
unsigned short diffy;				
unsigned short diffz;

unsigned char  deltasign;		/* Different bits of this variable is set to 1 or 0 to show the positive or negative sign 
								(respectively) of the difference of current and previous accelerometer values for each axis.*/
unsigned char  compress=0;		/* Flag that decides if the data that is to be sent is compressed or not;Initialized int 
								the firmware to send at least first batch of bytes or data */

unsigned char  sampleFlag = 0;			// Flag to sample the ADC; gets set by the timer ISR	
unsigned int   seconds_passed = 0;		// Counter to keep sampling accelerometer
unsigned char  connected = 0;			// To indicate if the wocket is connected to the phone
unsigned int   seconds_disconnected = 0;// To incur delay in the Timer ISR
unsigned char  interrupts_passed = 0;
unsigned char  interrupt_reps = 0;		// Used to skip sampling depending on the sampling rate variables/timers in the timer ISR 
unsigned short xv[3][MAX_SR];   		// The vector of acceleration values for one second 
unsigned long  vmag;					// Sum of the filtered value from 3 axis
unsigned char  justconnected = 0;		// Flag to identify when the wocket got connected
unsigned short blink_counter;	/* Is used in the timer2 ISR. The wockets blink green once every 5 seconds when they are in 
								either the continuous or bursty mode. blink_counter is a simple counter that is used to 
								implement this */

unsigned char  isdocked = 0;		// Flag to indicate while wocket is  connected to charger or programmer
unsigned int   dockcounter = 0;	/* Counter to prevent resetting the wocket, if someone plug in the wocket for a short time 
								accidentally. This would lead to loose of the activity count data. The counter is used to 
								count up to 2400 and then shutdown the radio and reset variables to allow for faster charging.*/

char shutdown_flag = 0;

//------------------------------------------------------Functions-----------------------------------------------------------------

//---------------------------------_send_pdu------------------------------------------------
/* This function sends the raw data to the phone.
Sending a PDU, in an uncompressed mode, requires 10bits per axis. In typical scenarios, the accelerometer 
on the body is not moving or is moving slightly, it is therefore redundant to send the 10bits. Instead, if 
the difference between consecutive values, the differential data, is less than 32 (2^5), the differential
value is sent in compress mode within 5 bits*/
static __inline__ void _send_pdu(unsigned short x, unsigned short y, unsigned short z) {	
	if(compress) {
		deltasign = 0x00;
		if (x > prevx) {
			deltasign |= 0x01; //The first bit from right: sign of difference value in x axis
			diffx = x - prevx;
		}
		else
			diffx = prevx - x;
		
		if (y > prevy) {
			deltasign |= 0x02; //The Second bit from right: sign of difference value in y axis
			diffy = y - prevy;
		}
		else
			diffy = prevy - y;   
		
		if (z > prevz) {
			deltasign |= 0x04; //The third bit from right: sign of difference value in z axis
			diffz = z - prevz;
		}
		else
			diffz = prevz - z;
		
		if ((diffx < 32) && (diffy < 32) && (diffz < 32))			
			_send_compressed_pdu((diffx | ((deltasign &0x01)<<5)), (diffy | ((deltasign &0x02)<<4)), 
			(diffz | ((deltasign &0x04)<<3)));
		else
			_send_uncompressed_pdu(x, y, z);
	}
	else {
	 	_send_uncompressed_pdu(x, y, z);
		compress = 1;
	}
	prevx = x;
	prevy = y;
	prevz = z;
}
//--------------------------------------Filter-------------------------------------

// Averaging the accelerometer values of each axis in one miniute for producing the activity count 
unsigned short Filter(unsigned short data,int axis) {
	 unsigned short mean = 0;
	 int j=0;                
     for (; (j < _SAMPLING_RATE); j++) {
	 	  mean += xv[axis][j];	//Initializing the xv is not required because, the activity count is 
		  						//calculated and saved only once per minute when xv is filled with 
								//valid accelerometer data
          xv[axis][j] = xv[axis][j + 1];		  		  
	 }
	 mean = mean / _SAMPLING_RATE;
     xv[axis][j] = data;
     				 
	 if (data > mean)
	 	return (data - mean);
	 else
	 	return (mean - data);      
}

//-----------------------------------do_sampling----------------------------------------
/* Sampling the accelerometer 
Sampling procedure is done in various parts of the code to ensure that the interups are 
acknowledeged close enough to their occurances*/
void do_sampling(){
	
	sampleFlag = 0;
    
	//sample the accelerometer
	x = _atmega_a2dConvert10bit(ADC0);		
	y = _atmega_a2dConvert10bit(ADC1);
	z = _atmega_a2dConvert10bit(ADC2);
	//------test-----
	/*if (i == 1024) 
		i=0;
	x = y = z = i;
	i++;*/
	//---------------

	//Filter the raw accelerometer data and compute the vector of magnitude (Activity count)
	vmag += Filter(x, 0) + Filter(y, 1) + Filter(z, 2);
		
	//for calculating the activity count, skip the first samples to make sure the buffer is clean	
	if (_wPC > _SAMPLING_RATE) {							
		if (summary_count == 0) {			// calculate the activity count only once per miniute
			vmag = vmag / AC_SCALING;	// vmag is scaled in order to prevent the overflow 			
			if (vmag > AC_CEILING)
				acount[ci] = AC_CEILING;// the maximum possible value of activity counts (size: two bytes)	
			else
				acount[ci] = (unsigned short) vmag; 
	 		
			vmag = 0;
			++ci;

			if (ci == AC_BUFFER_SIZE)
				ci = 0;
			cseq++;	// if the activity counts for the first 16 hours are full then increment the c sequence	
			 
			if (ci == si) {
				si++;
				if (si == AC_BUFFER_SIZE)
					si = 0;
				sseq++;
			}
			acount[ci] = 0;
			summary_count = AC_NUMS;	// 1 minute Summary counter is associated with activity counts 
		}
		else
			summary_count--;
	}
	else if (_wPC == 40)				// discard the first 40 samples for the vmag  				
		vmag = 0;

	//Save the raw data in the RAM 
	m_SET_X(data[dataIndex], x, dataSubindex);
	m_SET_Y(data[dataIndex], y, dataSubindex);
	m_SET_Z(data[dataIndex], z, dataSubindex);

	dataSubindex++;
	if (dataSubindex >= 4) {	
	 	dataSubindex = 0;
		if (_wTM != _WTM_Continuous){ 
			dataIndex++;
			if (dataIndex >= DATA_SIZE)
				dataIndex = 0;

			if (batch_counter < (DATA_SIZE - 1))
				batch_counter++;
		}
	}
}	


//----------------------------------- Main function ----------------------------------------------
int main() {

   // If the wocket is docked, waits for 10 seconds
	if (_is_docked()) {
		for(int j = 0;(j < 10);j++)			
			for(int i = 0;(i < 200);i++)
				_delay_ms(5);
	}

	// Blink green for 5 seconds	
	_wocket_initialize();

	summary_count = _SAMPLING_RATE * 60;
	AC_NUMS = _SAMPLING_RATE * 60;

	/*Functions used for optimizing power by shutting down different peripherals. 
	These functions are part of the external dependencies provided by AVR software power.h*/
	power_adc_disable();
  	power_spi_disable();
  	power_timer0_disable();
  	power_timer1_disable();
  	power_twi_disable();

	//Loop indefinitely
	while(1){

		if(sampleFlag){		// Sample flag is turned ON in the timer 2 ISR (OVR)\
		    
			//turn on adc and micro controller on board	
			power_adc_enable();
			_atmega_adc_turn_on();
			
			do_sampling(); // To prevent data loss, sampling is done in the "main loop" instead of the 
			               //"Interrupt Service Routine"				
		
			if (_wTM == _WTM_Continuous) { //Countinuous Mode
				//Fetching raw data	
				switch(dataSubindex) {
					case 1:
							m_GET_X(x, data[dataIndex].byte1,  data[dataIndex].byte2,  0);
							m_GET_Y(y, data[dataIndex].byte2,  data[dataIndex].byte3,  0);
							m_GET_Z(z, data[dataIndex].byte3,  data[dataIndex].byte4,  0);
							break;
					case 2:
							m_GET_X(x, data[dataIndex].byte4,  data[dataIndex].byte5,  1);
							m_GET_Y(y, data[dataIndex].byte6,  data[dataIndex].byte7,  1);
							m_GET_Z(z, data[dataIndex].byte7,  data[dataIndex].byte8,  1);
							break;
					case 3:
							m_GET_X(x, data[dataIndex].byte8,  data[dataIndex].byte9,  2);
							m_GET_Y(y, data[dataIndex].byte9,  data[dataIndex].byte10, 2);
							m_GET_Z(z, data[dataIndex].byte11, data[dataIndex].byte12, 2);
							break;
					case 0:
							m_GET_X(x, data[dataIndex].byte12, data[dataIndex].byte13, 3);
							m_GET_Y(y, data[dataIndex].byte13, data[dataIndex].byte14, 3);
							m_GET_Z(z, data[dataIndex].byte14, data[dataIndex].byte15, 3);
							break;
				}											
		
		        // if the wocket was just connected, confirm the wocket transmission mode 
				if (justconnected == 1)	{
					_send_wtm();
					justconnected = 2;
				}		

				//Sending raw data: In continous mode all raw data are sent without compressing
				_send_uncompressed_pdu(x, y, z);

				//sample and send the battery level
				battery = _atmega_a2dConvert10bit(ADC7); 
				//_send_bl(battery);
				_receive_data();
			}
			
			else { //Bursty modes			
				if (connected) {			// check for the BT connection
					if (shutdown_flag == 0)	
						_greenled_turn_on();

					if (_wTM == _WTM_Continuous) // Skips sending the recent batch of raw data if transfer mode is changed 
						continue;                   					
                   
				    _delay_ms(5);
					_receive_data();
					
					/* Once the wocket is connected to a phone, it sends data immediately. In some of 
					the phones (devices) we may lose initial packets of the data stream, transmitted 
					by the wockets. The loss of the data is probably due to some problem that occurs 
					on the receiving end (phone) during the connection setup. The current design of 
					the wockets does not have the capability to allow flow control using RTS/CTS. 
					Instead, we delay the transmission of the actual data (activity counts or raw data) 
					by sending padding bytes. The decoder software on the phone ignores the (0xff) bytes
					received by the phone.*/

					for (int ixz = 0; (ixz < 100); ixz++) {                                                                                
       					_bluetooth_transmit_uart0_byte(0xff); // The decoder on the phone ignores these packets 
							
						if (sampleFlag)
							do_sampling();
					} 
				    //------------------------------------------------------------------										
					//_send_sr();			// Send the sampling rate to the phone 				 
					//_send_wtm();		    // Send the wocket transmission mode to the phone					
					battery = _atmega_a2dConvert10bit(ADC7); 
					_send_bl(battery);		//sample and send the battery level										
					_send_acs();			// Send the Activity counts information 
					
					rawTail = dataIndex;
					if (batch_counter == (DATA_SIZE - 1)) //in case of memory overflow
						rawHead = dataIndex + 1;
					/*if (rawTail - rawHead >= 0)
						batch_counter = rawTail - rawHead;
					else 
						batch_counter = DATA_SIZE - rawHead - 1 + rawTail;*/
					_send_batch_count(batch_counter * 4);	// Send the number of raw data packets that are going to be sent 
					batch_counter = 0;
					
					while(rawHead != rawTail) { //Send raw data from Circular buffer
					
						m_GET_X(x, data[rawHead].byte1,  data[rawHead].byte2,  0);
						m_GET_Y(y, data[rawHead].byte2,  data[rawHead].byte3,  0);
						m_GET_Z(z, data[rawHead].byte3,  data[rawHead].byte4,  0);
						_send_pdu(x, y, z);

						m_GET_X(x, data[rawHead].byte4,  data[rawHead].byte5,  1);
						m_GET_Y(y, data[rawHead].byte6,  data[rawHead].byte7,  1);
						m_GET_Z(z, data[rawHead].byte7,  data[rawHead].byte8,  1);
						_send_pdu(x, y, z);

						m_GET_X(x, data[rawHead].byte8,  data[rawHead].byte9,  2);
						m_GET_Y(y, data[rawHead].byte9,  data[rawHead].byte10, 2);
						m_GET_Z(z, data[rawHead].byte11, data[rawHead].byte12, 2);
						_send_pdu(x, y, z);

						m_GET_X(x, data[rawHead].byte12, data[rawHead].byte13, 3);
						m_GET_Y(y, data[rawHead].byte13, data[rawHead].byte14, 3);
						m_GET_Z(z, data[rawHead].byte14, data[rawHead].byte15, 3);
						_send_pdu(x, y, z);

						rawHead++;

						if (rawHead == DATA_SIZE)
							rawHead = 0;

						//_receive_data();

						if (sampleFlag)
							do_sampling();							
					}					
					
					_send_end_batch();	//tell phone/PC that all data for this minute has been sent 
					//------------------------------------------------------------------

					seconds_passed = 0;
					while (seconds_passed < 400) {//The delay provided here helps to not lose the sent data from the phone
						_delay_ms(5);
						seconds_passed++;
						_receive_data();

						if (sampleFlag)
							do_sampling();
					} 

					//Don't turn off the radio if a request to switch mode has been received
					if ((_wTM == _WTM_Continuous) && (shutdown_flag == 0))
						_bluetooth_turn_on();	
					else
						_bluetooth_turn_off();		
					
					command_counter = 0;
					seconds_disconnected = 0;
					_greenled_turn_off();

				} // End if (connected)
				if (sampleFlag)
					do_sampling();

			} // End else (_wTM==_WTM_Continuous) => _wTM is bursty 

			_atmega_adc_turn_off();
			power_adc_disable();

			if(_wTM==_WTM_Continuous){
				if ((dataSubindex == 0) && (!connected))
					dataIndex++;
				if (dataIndex == DATA_SIZE)
					dataIndex = 0;
			}

			connected = 0;			
		}// Endof the First if (sampleFlag)	
		
		   
			cli();				// Clear interruptions and set the system to sleep mode
			set_sleep_mode(SLEEP_MODE_IDLE);
			// Built in functionality to enable interrupts and shutdown of the cpu to save power 
    		sleep_enable();		// sleep.h 
    		sei();				// interrupt.h
    		sleep_cpu();		// sleep.h 
    		sleep_disable();	// sleep.h 	

	} // End While(1)

	return 0;
} // End main




//------------------------ Interrupt service routine for Timer 2------------------------------------
ISR(TIMER2_OVF_vect)
{	
	if (_is_docked()) // Has the wocket been connected to the charger/programmer more than one minitue?		
	{
		dockcounter++;		 
		if ((!isdocked)&& (dockcounter > (_SAMPLING_RATE * 60))){				
			ci   = 0;
			si   = 0;
			cseq = 0; 
			sseq = 0;		
			_bluetooth_turn_off();
			isdocked = 1;
			if (shutdown_flag == 1){
				//_wocket_initialize();
				_bluetooth_turn_on();
				shutdown_flag = 0;
				_yellowled_turn_on();
				for(int i = 0;(i < 200);i++)
					_delay_ms(10);
				_yellowled_turn_off();
			}
							
		}
		return;
	} else {
		dockcounter = 0;
		if (isdocked) {
			_bluetooth_turn_on();
			isdocked = 0;			
		}
	}

	if ((connected == 0)&& (shutdown_flag == 0)){
		blink_counter++;
		if (blink_counter == (_SAMPLING_RATE * 5))		// ON period
			_greenled_turn_on();
		else if (blink_counter == ((_SAMPLING_RATE * 5) + 10)) { 	// OFF period
			_greenled_turn_off();
			blink_counter = 0;
		}
	}

	// Adjusting the counter for the sampling rate 
	//REFER to _wocket_initialize_timer2_interrupt in the wocket.c	
 	if (interrupt_reps == 0) {	
		interrupt_reps = _wTCNT2_reps;
		TCNT2 = _wTCNT2;
	}
	else { //otherwise wait
		if (interrupt_reps == 1)	
			TCNT2 = _wTCNT2_last;	
		else		
			TCNT2 = _wTCNT2;					
		interrupt_reps--;
		return;
	}
	
	// When this flag is set, accelerometer should be sampled
	sampleFlag = 1;

	if (_wTM == _WTM_Continuous) {
		_wPC++;
		// Section of the code to indicate that the wocket got connected
		if (!_bluetooth_is_connected()){
			justconnected = 0;
			compress = 0;
			return;		
		}
		else if (justconnected == 0)
			justconnected = 1;

		if (_wShutdownTimer != _DEFAULT_SHUTDOWN)
			_wShutdownTimer  = _DEFAULT_SHUTDOWN;

		_receive_data();
	}

	else if (_wTM == _WTM_Burst_60)	{
		//This only works for Timer1,doesn't have any effect for this timer (Timer2)
		if (_wPDT != 0)
			_wShutdownTimer--;

		_wPC++;
		
		 // Turns the Bluetooth ON approximately every 45 seconds after the previous transmission 
		if (!_bluetooth_is_connected()) {			
			compress = 0; 

			if (seconds_disconnected < (_SAMPLING_RATE * 45)) // 45 Sec
				seconds_disconnected++;
			else if (seconds_disconnected == (_SAMPLING_RATE * 45))	{
				//before turning on the bluetooth make sure the receive buffer is flushed
				_receive_uart0_flush();
				if (shutdown_flag == 0)
					_bluetooth_turn_on();		
				seconds_disconnected = (_SAMPLING_RATE * 45) + 1;			
				_delay_ms(10);
			}

			return;	
		}

		//reset shutdown timer if connected
		if ((_wPDT != 0) && (_wShutdownTimer != _DEFAULT_SHUTDOWN))
			_wShutdownTimer = _DEFAULT_SHUTDOWN;

		_receive_data();		
		connected = 1;		
	}
}




