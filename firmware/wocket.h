
/*Refer to the document WOCKETS INTERFACE SPECIFICATION_PDU_Description.rtf for the wockets 
commands and description */

#define AC_BUFFER_SIZE 960

typedef struct{	      //Each data_unit has 15 bytes of data
	unsigned char byte1;
	unsigned char byte2;
	unsigned char byte3;
	unsigned char byte4;
	unsigned char byte5;
	unsigned char byte6;
	unsigned char byte7;
	unsigned char byte8;
	unsigned char byte9;
	unsigned char byte10;
	unsigned char byte11;
	unsigned char byte12;
	unsigned char byte13;
	unsigned char byte14;
	unsigned char byte15;
} data_unit;


#define DATA_SIZE 750 //Size of data array, used in wocket firmware 
#define MAX_SR 150 	  //Maximum of sampling rate, used in wocket firmware for the size of xv matrix

#define MAX_COMMAND_SIZE  10
#define MAX_COMMAND_TIMER 255

/* Wockets commands constants */

/* baud rates */
#define BAUD_9600 	1
#define BAUD_19200 	2
#define BAUD_38400 	0
#define BAUD_57600	3

/* Reserved Wockets EEPROM Locations */
#define X1G_ADDRESS  0x00
#define X1NG_ADDRESS 0x02
#define Y1G_ADDRESS  0x04
#define Y1NG_ADDRESS 0x06
#define Z1G_ADDRESS  0x08
#define Z1NG_ADDRESS 0x0A


/* Wockets Sensitivities */
#define _1_5_G 		0b000
#define _2_G 		0b001
#define _4_G		0b010 //currently
#define _6_G		0b011
#define _8_G		0b100
#define _12_G		0b111 	

/* Radio Transmission Modes */
#define _TM_1	0b000
#define _TM_2	0b001			

/* Wockets Transmission Modes */
#define _WTM_Continuous	0b000
#define _WTM_Burst_30	0b001		// Burst after 30 secs
#define _WTM_Burst_60	0b010       // Burst after 60 secs [Currently 60 is being used]
#define _WTM_Burst_90	0b011		// Burst after 90 secs
#define	_WTM_Burst_120	0b100		// Burst after 120 secs

/* Wockets Status Bits */
#define _STATUS_INITIALIZED 0

#define PERFECT_SAMPLING_FREQUENCY 90 // Current Sampling rate or frequncy is 40Hz

/**  WOCKETS PDU FORMAT **/

/* PDU Types */
#define UNCOMPRESSED_PDU 0b00
#define COMMAND_PDU 	 0b01
#define RESPONSE_PDU 	 0b10
#define COMPRESSED_PDU 	 0b11

/* Macro for PDU Header */
#define m_HEADER(type) (0x80|(type<<5))

/* Macros for Uncompressed PDU */
#define m_UNCOMPRESSED_PDU_BYTE1(x)   (m_HEADER(UNCOMPRESSED_PDU)|(x >> 8))
#define m_UNCOMPRESSED_PDU_BYTE2(x)   (0x7f & (x >> 1))
#define m_UNCOMPRESSED_PDU_BYTE3(x,y) ((0x40 & (x << 6)) | (0x3f & (y >> 4)))
#define m_UNCOMPRESSED_PDU_BYTE4(y,z) ((0x78 & (y << 3)) | (0x07 & (z >> 7)))
#define m_UNCOMPRESSED_PDU_BYTE5(z)   (0x7f & z)

/* Macros for compressed PDU */
#define m_COMPRESSED_PDU_BYTE1(x) 	  (m_HEADER(COMPRESSED_PDU)|(( x>> 1) & 0x1f))
#define m_COMPRESSED_PDU_BYTE2(x,y)   (((x & 0x01) << 6) | (y & 0x3f))
#define m_COMPRESSED_PDU_BYTE3(z) 	  (0x7e & (z << 1))

/* Macros for Command PDUs */

#define RESPONSE_HEADER(opcode)       (0xc0|opcode)

#define HEADER_PACKET   0x80
#define COMMAND_PREFIX  0b101

/*Refer the WOCKETSINTERFACE SPECIFICATION_PDU_Description.rtf to know what each byte 
of information indicates: 8 bits: 1/0bit[H:first byte/L:not first byte] xxbits 
[00: Full uncompressed data packet/01: Command packet/ 10: Response packet 11: Reserved 
for Future Use and 5 bits of command or data*/

/* Reserved Wockets Commands Opcodes */
#define GetBatteryLevel				0b00000
#define GetBatteryPercent 			0b00001
#define GetPacketCount 				0b00010
#define ResetBluetooth 				0b00011
#define GetSensorSentivity			0b00100
#define SetSensorSentivity			0b00101
#define GetCalibrationValues 		0b00110
#define SetCalibrationValues		0b00111
#define GetSamplingRate				0b01000
#define SetSamplingRate				0b01001
#define GetAliveTimer				0b01010
#define SetAliveTimer				0b01011
#define GetPowerDownTimer			0b01100
#define SetPowerDownTimer			0b01101
#define ResetWocket					0b01110
#define Alive						0b01111
#define PausePacket					0b10000
#define ResumePacket				0b10001
#define GetRadioTransmissionMode	0b10010
#define	SetRadioTransmissionMode	0b10011
#define GetBatteryCalibration		0b10100
#define SetBatteryCalibration		0b10101
#define GetHardwareVersion			0b10110
#define GetFirmwareVersion			0b10111
#define GetTCT						0b11000
#define SetTCT						0b11001
#define SetWocketTransmissionMode	0b11010
#define ACK							0b11011
#define SetLED						0b11100
#define ShutdownWocket				0b11101
#define GetWocketTransmissionMode	0b11110


/* Macros for Wockets Commands */

/* SET_SEN Macros */			/* Sensitivity of the accelerometer */
#define m_SET_SEN(aByte2) 	((aByte2 >> 3) & 0x07)

/* SET_SR Macros */				/* sets the sampling rate that a wocket should deliver */
#define m_SET_SR(aByte2) 	(aByte2 & 0x7f)

/* SET_PDT Macros */	/* The time in minutes before the wocket powers down while being disconnected */
#define m_SET_PDT(aByte2) 	(aByte2 & 0x7f)

/* SET_TM Macros */		/* sets the transmission mode for the Bluetooth radio */
#define m_SET_TM(aByte2) 	((aByte2 >> 4) & 0x07)

/* SET_WTM Macros */		/* sets the WOCKET transmission mode  */
#define m_SET_WTM(aByte2) 	((aByte2 >> 4) & 0x07)

/* SET_ALT Macros */	/* gets the approximate time that a wocket waits before resetting itself, 
if no alive packets are received*/ 
#define m_SET_ALT(aByte2) 	(aByte2 & 0x7f)

/* SET_LED Macros */
#define m_SET_LED_COLOR(aByte2)	((aByte2 & 0x40) >> 6)
#define m_SET_LED_TIME(aByte2)	(aByte2 & 0x3f)


/* SET_CAL Macros */    /* sets the calibration values for each axis of the accelerometer on the wocket */
#define m_SET_CAL_x1g(aByte1, aByte2)  	      (	(((unsigned short)(aByte1 & 0x7f)) << 3) | 	(((unsigned short)(aByte2 & 0x70)) >> 4)	)
#define m_SET_CAL_xn1g(aByte2, aByte3)   	  ( (((unsigned short)(aByte2 & 0x0f)) << 6) | 	(((unsigned short)(aByte3 & 0x7e)) >> 1)	)
#define m_SET_CAL_y1g(aByte3, aByte4, aByte5) ( (((unsigned short)(aByte3 & 0x01)) << 9) |	(((unsigned short)(aByte4 & 0x7f)) << 2) |	(((unsigned short)(aByte5 & 0x60)) >> 5)	)
#define m_SET_CAL_yn1g(aByte5, aByte6)  	  ( (((unsigned short)(aByte5 & 0x1f)) << 5) | 	(((unsigned short)(aByte6 & 0x7c)) >> 2)	)
#define m_SET_CAL_z1g(aByte6, aByte7, aByte8) ( (((unsigned short)(aByte6 & 0x03)) << 8) | 	(((unsigned short)(aByte7 & 0x7f)) << 1) | 	(((unsigned short)(aByte8 & 0x40)) >> 6)	)
#define m_SET_CAL_zn1g(aByte8, aByte9)    	  ( (((unsigned short)(aByte8 & 0x3f)) << 4) | 	(((unsigned short)(aByte9 & 0x78)) >> 3) 	)


/* SET_BTCAL Macros */ /* Battery Calibration: This section captures the battery performance curve 
by sampling the battery and recording the values at the specified percentiles. The values will be 
recorded on the PC/phone and then stored on the firmware using the command on the phone that 
triggers the macros on the firmware */
#define m_SET_BTCAL_100(aByte1, aByte2)  	  (	(((unsigned short)(aByte1 & 0x7f)) << 3) | 	(((unsigned short)(aByte2 & 0x70)) >> 4)	)
#define m_SET_BTCAL_80(aByte2, aByte3)   	  ( (((unsigned short)(aByte2 & 0x0f)) << 6) | 	(((unsigned short)(aByte3 & 0x7e)) >> 1)	)
#define m_SET_BTCAL_60(aByte3, aByte4,aByte5) ( (((unsigned short)(aByte3 & 0x01)) << 9) |	(((unsigned short)(aByte4 & 0x7f)) << 2) |	(((unsigned short)(aByte5 & 0x60)) >> 5)	)
#define m_SET_BTCAL_40(aByte5, aByte6)  	  ( (((unsigned short)(aByte5 & 0x1f)) << 5) | 	(((unsigned short)(aByte6 & 0x7c)) >> 2)	)
#define m_SET_BTCAL_20(aByte6, aByte7,aByte8) ( (((unsigned short)(aByte6 & 0x03)) << 8) | 	(((unsigned short)(aByte7 & 0x7f)) << 1) | 	(((unsigned short)(aByte8 & 0x40)) >> 6)	)
#define m_SET_BTCAL_10(aByte8, aByte9)   	  ( (((unsigned short)(aByte8 & 0x3f)) << 4) | 	(((unsigned short)(aByte9 & 0x78)) >> 3) 	)


/* SET_TCT Macros */ /* TCT Command was recently being worked on [Aug 2011] to set the ticks*/
#define m_SET_TCT(aByte1, aByte2)    	((unsigned char)((aByte1 & 0x7f) << 1)| (unsigned char)((aByte2 >> 6) & 0x01) )
#define m_SET_TCTREPS(aByte2, aByte3) 	((unsigned char)((aByte2 & 0x3f) << 2)| (unsigned char)((aByte3 >> 5) & 0x03) )
#define m_SET_TCTLAST(aByte3, aByte4) 	((unsigned char)((aByte3 & 0x1f) << 3)| (unsigned char)((aByte4 >> 4) & 0x07) )


/* SET_ACK Macros */		/* Sets the transmission mode for the Bluetooth radio */
#define m_ACK(aByte2, aByte3, aByte4)  	( (((unsigned short)(aByte2 & 0x7f)) << 9) | (((unsigned short)(aByte3 & 0x7f)) << 2) | (((unsigned short)(aByte4 >> 5)) & 0x03))



/* Reserved Wockets Response Opcodes */
#define BL_RSP 		0b00000		// response sends back the battery level for the wocket 
#define BP_RSP 		0b00001		/* Battery percent command: the idea here is that once the battery is calibrated we wanted an adjusted value that takes care of
								battery lifetime, calibration etc... */
#define PC_RSP		0b00010		// sends back the number of packets that were sent since the last GET_PC */
#define SENS_RSP	0b00011		// Sends back sensitivity 
#define CAL_RSP		0b00100		// sends back the calibration data for the wocket
#define SR_RSP		0b00101		// Sampling Rate
#define ALT_RSP		0b00110		// a 7 bit timer as outlined by SET_ALT
#define PDT_RSP		0b00111
#define WTM_RSP		0b01000
#define BTCAL_RSP 	0b01001		// Battery Calibration 	
#define HV_RSP	 	0b01010		// Hardware version 
#define FV_RSP	 	0b01011		// Firmware version 
#define BC_RSP		0b01100		/* Batch Count, includes the number of samples in a batch in bursty mode. This is used to determine if the wockets sent a partial
								or full batch*/
#define AC_RSP 		0b01101		// Activity count Response packet 
#define TCT_RSP		0b01110		// Response packet for the ticks 
#define ACC_RSP		0b01111 	//activity count count
#define OFT_RSP		0b10000 	//offset AC count
#define TM_RSP		0b10001
#define END_BATCH	0b10010		//confirmation that all data has been sent for the minute in bursty mode

/* Macros for Wockets Responses */

/* END_BATCH Macros*/
#define m_END_BATCH_BYTE0		RESPONSE_HEADER(END_BATCH)

/* BL_RSP Macros */
#define m_BL_RSP_BYTE0			RESPONSE_HEADER(BL_RSP)
#define m_BL_RSP_BYTE1(level)	(level >> 3)
#define m_BL_RSP_BYTE2(level)	((level & 0x07) << 4)

/* BP_RSP Macros */
#define m_BP_RSP_BYTE0			RESPONSE_HEADER(BP_RSP)
#define m_BP_RSP_BYTE1(percent)	(percent & 0x7f)

/* PC_RSP Macros */
#define m_PC_RSP_BYTE0			RESPONSE_HEADER(PC_RSP)
#define m_PC_RSP_BYTE1(count)	( count >> 25)
#define m_PC_RSP_BYTE2(count)	((count >> 18) & 0x7f)
#define m_PC_RSP_BYTE3(count)	((count >> 11) & 0x7f)
#define m_PC_RSP_BYTE4(count)	((count >> 4)  & 0x7f)
#define m_PC_RSP_BYTE5(count)	((count & 0x0f) << 3)

/* SENS_RSP Macros */
#define m_SENS_RSP_BYTE0				RESPONSE_HEADER(SENS_RSP)
#define m_SENS_RSP_BYTE1(sensitivity)	((sensitivity & 0x07)<<4)

/* CAL_RSP Macros */
#define m_CAL_RSP_BYTE0					RESPONSE_HEADER(CAL_RSP)
#define m_CAL_RSP_BYTE1_x1g(x1g)		((x1g  >> 3) & 0x7f)
#define m_CAL_RSP_BYTE2_x1g(x1g)		((x1g  << 4) & 0x70)
#define m_CAL_RSP_BYTE2_xn1g(xn1g)		((xn1g >> 6) & 0x0f)
#define m_CAL_RSP_BYTE3_xn1g(xn1g)		((xn1g << 1) & 0x7e)
#define m_CAL_RSP_BYTE3_y1g(y1g)		((y1g  >> 9) & 0x01)
#define m_CAL_RSP_BYTE4_y1g(y1g)		((y1g  >> 2) & 0x7f)
#define m_CAL_RSP_BYTE5_y1g(y1g)		((y1g  << 5) & 0x60)
#define m_CAL_RSP_BYTE5_yn1g(yn1g)		((yn1g >> 5) & 0x1f)
#define m_CAL_RSP_BYTE6_yn1g(yn1g)		((yn1g << 2) & 0x7c)
#define m_CAL_RSP_BYTE6_z1g(z1g)		((z1g  >> 8) & 0x03)
#define m_CAL_RSP_BYTE7_z1g(z1g)		((z1g  >> 1) & 0x7f)
#define m_CAL_RSP_BYTE8_z1g(z1g)		((z1g  << 6) & 0x40)
#define m_CAL_RSP_BYTE8_zn1g(zn1g)		((zn1g >> 4) & 0x3f)
#define m_CAL_RSP_BYTE9_zn1g(zn1g)		((zn1g << 3) & 0x78)

/* BTCAL_RSP Macros */
#define m_BTCAL_RSP_BYTE0						RESPONSE_HEADER(BTCAL_RSP)
#define m_BTCAL_RSP_BYTE1_100(percentile100)	((percentile100 >> 3) & 0x7f)
#define m_BTCAL_RSP_BYTE2_100(percentile100)	((percentile100 << 4) & 0x70)
#define m_BTCAL_RSP_BYTE2_80(percentile80)		((percentile80  >> 6) & 0x0f)
#define m_BTCAL_RSP_BYTE3_80(percentile80)		((percentile80  << 1) & 0x7e)
#define m_BTCAL_RSP_BYTE3_60(percentile60)		((percentile60  >> 9) & 0x01)
#define m_BTCAL_RSP_BYTE4_60(percentile60)		((percentile60  >> 2) & 0x7f)
#define m_BTCAL_RSP_BYTE5_60(percentile60)		((percentile60  << 5) & 0x60)
#define m_BTCAL_RSP_BYTE5_40(percentile40)		((percentile40  >> 5) & 0x1f)
#define m_BTCAL_RSP_BYTE6_40(percentile40)		((percentile40  << 2) & 0x7c)
#define m_BTCAL_RSP_BYTE6_20(percentile20)		((percentile20  >> 8) & 0x03)
#define m_BTCAL_RSP_BYTE7_20(percentile20)		((percentile20  >> 1) & 0x7f)
#define m_BTCAL_RSP_BYTE8_20(percentile20)		((percentile20  << 6) & 0x40)
#define m_BTCAL_RSP_BYTE8_10(percentile10)		((percentile10  >> 4) & 0x3f)
#define m_BTCAL_RSP_BYTE9_10(percentile10)		((percentile10  << 3) & 0x78)

/* SR_RSP Macros */
#define m_SR_RSP_BYTE0				RESPONSE_HEADER(SR_RSP)
#define m_SR_RSP_BYTE1(sr)			(sr & 0x7f)

/* ALT_RSP Macros */
#define m_ALT_RSP_BYTE0				RESPONSE_HEADER(ALT_RSP)
#define m_ALT_RSP_BYTE1(timeout)	(timeout & 0x7f)

/* PDT_RSP Macros */
#define m_PDT_RSP_BYTE0				RESPONSE_HEADER(PDT_RSP)
#define m_PDT_RSP_BYTE1(timeout)	(timeout & 0x7f)

/* TM_RSP Macros */
#define m_TM_RSP_BYTE0				RESPONSE_HEADER(TM_RSP)
#define m_TM_RSP_BYTE1(mode)		((mode & 0x07) << 4)

/* Wocket Traansmission Mode Macros */
#define m_WTM_RSP_BYTE0				RESPONSE_HEADER(WTM_RSP)
#define m_WTM_RSP_BYTE1(mode)		((mode & 0x07) << 4)

/* HV_RSP Macros */
#define m_HV_RSP_BYTE0				RESPONSE_HEADER(HV_RSP)
#define m_HV_RSP_BYTE1(version)		(version & 0x7f)

/* FV_RSP Macros */
#define m_FV_RSP_BYTE0				RESPONSE_HEADER(FV_RSP)
#define m_FV_RSP_BYTE1(version)		(version & 0x7f)

/* BC_RSP Macros */
#define m_BC_RSP_BYTE0				RESPONSE_HEADER(BC_RSP)
#define m_BC_RSP_BYTE1(count)		((count >> 9) & 0x7f)
#define m_BC_RSP_BYTE2(count)		((count >> 2) & 0x7f)
#define m_BC_RSP_BYTE3(count)		(((count & 0x03) << 5) & 0x60)

/* AC_RSP Macros */
#define m_AC_RSP_BYTE0					RESPONSE_HEADER(AC_RSP)
#define m_AC_RSP_BYTE1(seq_num)			((seq_num >> 9) & 0x7f)
#define m_AC_RSP_BYTE2(seq_num)			((seq_num >> 2) & 0x7f)
#define m_AC_RSP_BYTE3(seq_num, count)	( ((seq_num & 0x03) << 5) | ((count >> 11) & 0x1f) )
#define m_AC_RSP_BYTE4(count)			((count >> 4) & 0x7f)
#define m_AC_RSP_BYTE5(count)			((count & 0x0f) << 2)

/* TCT_RSP Macros */
#define m_TCT_RSP_BYTE0				RESPONSE_HEADER(TCT_RSP)
#define m_TCT_RSP_BYTE1(tct)		((tct >> 1) & 0x7f)
#define m_TCT_RSP_BYTE2(tct,reps)	(((tct & 0x01) << 6) | (reps >> 2))
#define m_TCT_RSP_BYTE3(reps,last)	(((reps & 0x03) << 5)| (last >> 3))
#define m_TCT_RSP_BYTE4(last)		((last & 0x07) << 4)

/* ACC_RSP Macros */
#define m_ACC_RSP_BYTE0				RESPONSE_HEADER(ACC_RSP)
#define m_ACC_RSP_BYTE1(count)		((count >> 7) & 0x7f)
#define m_ACC_RSP_BYTE2(count)		(count & 0x7f)

/* OFT_RSP Macros */
#define m_OFT_RSP_BYTE0				RESPONSE_HEADER(OFT_RSP)
#define m_OFT_RSP_BYTE1(offset)		((offset >> 7) & 0x7f)
#define m_OFT_RSP_BYTE2(offset)		(offset & 0x7f)

#define m_SUCCESS_RESPONSE_BYTE1	RESPONSE_HEADER(SUCCESS_RESPONSE)

#define BIT0_MASTERSLAVE_STATUS 0
#define BIT1_BURSTY_STATUS 1
#define BIT2_BIT3_BAUD_RATE 2

extern uint8_t EEMEM _NV_INITIALIZED;
extern uint8_t EEMEM _NV_STATUS_BYTE;
extern uint8_t EEMEM _NV_SAMPLING_RATE;
extern uint8_t EEMEM _NV_DEBUG;

extern uint8_t EEMEM _NV_TCT;
extern uint8_t EEMEM _NV_TCTREPS;
extern uint8_t EEMEM _NV_TCTLAST;

extern unsigned char _INITIALIZED;
extern unsigned char _STATUS_BYTE;
extern unsigned char _SAMPLING_RATE;

extern unsigned char _wTCNT2_reps;
extern unsigned char _wTCNT2;
extern unsigned char _wTCNT2_last;

extern unsigned char _wTM;
extern unsigned long _wPC;
extern unsigned long _wShutdownTimer;
extern unsigned long _DEFAULT_SHUTDOWN;
extern unsigned char _wPDT;

extern unsigned short cseq;
extern unsigned short sseq;
extern unsigned short kseq;
extern unsigned short dseq;
extern unsigned short ci;
extern unsigned short si;
extern unsigned short AC_NUMS;
extern unsigned short summary_count;
extern unsigned char  command_counter;
extern unsigned long _wLastPC;
extern char shutdown_flag;
/*Packet or frame of 5 bytes*/
typedef struct{
	unsigned char byte1; // sync bit, 2 bits packet type, 3 bits sensitivity, 2 bits MSB X
	unsigned char byte2; // 0 bit, 7 X
	unsigned char byte3; // 0 bit, 1 LSB X, 6 MSB y
	unsigned char byte4; // 0 bit, 4 LSB y, 3 MSB z
	unsigned char byte5; // 0 bit, 7 LSB z
} wockets_uncompressed_packet;


unsigned char num_skipped_timer_interrupts;
unsigned char wocket_status;

unsigned short scounter;


extern data_unit data[DATA_SIZE];
extern unsigned  short dataIndex;
extern unsigned  char  dataSubindex;

extern unsigned  short *xsp;
extern unsigned  short *ysp;
extern unsigned  short *zsp;

extern unsigned  short acount[AC_BUFFER_SIZE];

/*Section that stores RAW data on the RAM*/
//Each axes value is saved in 10 bits
#define m_SET_X(pdata, x, index) switch(index){\
									case 0: pdata.byte1   = (x >> 2); pdata.byte2  = ((x << 6) & 0xc0); break;\
									case 1: pdata.byte4  |= (x >> 8); pdata.byte5  = ( x & 0xff); break;\
									case 2: pdata.byte8  |= (x >> 6); pdata.byte9  = ((x << 2) & 0xfc); break;\
									case 3: pdata.byte12 |= (x >> 4); pdata.byte13 = ((x << 4) & 0xf0); break;}
#define m_SET_Y(pdata, y, index) switch(index){\
									case 0: pdata.byte2  |= (y >>4);  pdata.byte3  = ((y << 4) & 0xf0); break;\
									case 1: pdata.byte6   = (y >> 2); pdata.byte7  = ((y << 6) & 0xc0); break;\
									case 2: pdata.byte9  |= (y >> 8); pdata.byte10 = ( y & 0xff); break;\
									case 3: pdata.byte13 |= (y >> 6); pdata.byte14 = ((y << 2) & 0xfc); break;}
#define m_SET_Z(pdata, z, index) switch(index){\
									case 0: pdata.byte3  |= (z >> 6); pdata.byte4  = ((z << 2) & 0xfc); break;\
									case 1: pdata.byte7  |= (z >> 4); pdata.byte8  = ((z << 4) & 0xf0); break;\
									case 2: pdata.byte11  = (z >> 2); pdata.byte12 = ((z << 6) & 0xc0); break;\
									case 3: pdata.byte14 |= (z >> 8); pdata.byte15 = ( z & 0xff); break;}

/*Section that retrieves RAW data from the RAM*/
#define m_GET_X(x, byte1, byte2, index) switch(index){\
									case 0: x = ((byte1 & 0xff) << 2)|((byte2 & 0xc0) >> 6); break;\
									case 1: x = ((byte1 & 0x03) << 8)|( byte2 & 0xff); break;\
									case 2: x = ((byte1 & 0x0f) << 6)|((byte2 & 0xfc) >> 2); break;\
									case 3: x = ((byte1 & 0x3f) << 4)|((byte2 & 0xf0) >> 4); break;}

#define m_GET_Y(y, byte1, byte2, index) switch(index){\
									case 0: y = ((byte1 & 0x3f) << 4)|(byte2 >> 4); break;\
									case 1: y = ( byte1 << 2)|(byte2>>6); break;\
									case 2: y = ((byte1 & 0x03) << 8)|byte2; break;\
									case 3: y = ((byte1 & 0x0f) << 6)|(byte2 >> 2); break;} 


#define m_GET_Z(z, byte1, byte2, index) switch(index){\
									case 0: z = ((byte1 & 0x0f) << 6) | (byte2 >> 2); break;\
									case 1: z = ((byte1 & 0x3f) << 4) | (byte2 >> 4); break;\
									case 2: z = ( byte1 << 2) | (byte2 >> 6); break;\
									case 3: z = ((byte1 & 0x03) << 8) | byte2; break;}

void _wocket_initialize(void);
void _wocket_set_master_mode(void);
void _wocket_set_slave_mode(void);
unsigned char _wocket_is_master(void);

unsigned char _wocket_is_flag_set(unsigned char flag);
void _wocket_reset_flag(unsigned char flag);
void _wocket_set_flag(unsigned char flag);

void _send_data(void);
void _receive_data(void);
void _send_batch_count(unsigned short count);
void _send_ac_count(unsigned short count);
//void _send_summary_count(unsigned short count);
void _send_acs(void);
void _send_ac_offset(unsigned short offset);
void _send_data_bufferred(void);
void _send_wtm(void);
void _send_sr(void);
void _send_fv(void);
void _send_hv(void);
void _send_bl(unsigned short level);
void _send_end_batch(void);

unsigned char _wocket_get_baudrate(void);
void _wocket_set_baudrate(unsigned char baudrate);
void wockets_uncompressed_packet_encode_packet(unsigned short x, unsigned short y, unsigned short z);
void _transmit_packet(wockets_uncompressed_packet packet);

void _send_uncompressed_pdu(unsigned short x, unsigned short y, unsigned short z);
void _send_compressed_pdu(unsigned char x, unsigned char y, unsigned char z);





