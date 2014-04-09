//*****************************************************************************
//
// File Name    : 'mcu_atmega.h'
// Title        : Definitions of ports, pins and function prototypes for all MCU related code
// Author       : Fahd Albinali
// Created      : 12/10/2008
// Modified     : 08/01/2012
// 
//  Description : This file includes all the definitions and constants that are used by the MCU unit such as
//  the different pins, USART supported baud rates etc.
//  
//
// This code is distributed under the MIT License
//     
//
//*****************************************************************************

/* Constants and preprocessor directives */

// CPU Frequency 8 MHz
#define _VERSION 3    //Hardware version
#define _FVERSION 6 //Firmware version
#define F_CPU 8000000L

#define CPU_CLK_PRESCALAR_NONE 0
#define CPU_CLK_PRESCALAR_8 1
#define CPU_CLK_PRESCALAR_32 2
#define CPU_CLK_PRESCALAR_64 3
#define CPU_CLK_PRESCALAR_128 4
#define CPU_CLK_PRESCALAR_256 5
#define CPU_CLK_PRESCALAR_1024 6

//max and min values held by unsigned and signed variables
#define MAX_U08 255
#define MAX_U16 65535
#define MAX_U32 4294967295
#define MIN_S08 -128
#define MAX_S08 127
#define MIN_S16 -32768
#define MAX_S16 32767
#define MIN_S32 -2147483648
#define MAX_S32 2147483647

//ADC Selectors
#define ADC0 0
#define ADC1 1
#define ADC2 2
#define ADC3 3
#define ADC4 4
#define ADC6 6
#define ADC7 7

//ADC Prescalars

#define ADC_PRESCALAR_2 0
#define ADC_PRESCALAR_4 1
#define ADC_PRESCALAR_8 2
#define ADC_PRESCALAR_16 3
#define ADC_PRESCALAR_32 4
#define ADC_PRESCALAR_64 5
#define ADC_PRESCALAR_128 6


//UART supported baud rates

#define ATMEGA_BAUD_2400 	207
#define ATMEGA_BAUD_4800 	103
#define ATMEGA_BAUD_9600 	51
#define ATMEGA_BAUD_19200 	25
#define ATMEGA_BAUD_28800	16
#define ATMEGA_BAUD_38400 	12
#define ATMEGA_BAUD_57600	8
#define ATMEGA_BAUD_115200	3
#define ATMEGA_BAUD_230000	1
#define ATMEGA_BAUD_460000	0

//UART Modes
#define TX_UART_MODE 0
#define RX_UART_MODE 1
#define TX_RX_UART_MODE 2



#ifdef _VERSION == 3
// Port Definitions
// Port A
#define IN_ACCEL_Z_FILT 0
#define IN_ACCEL_Y_FILT 1
#define IN_ACCEL_X_FILT 2
#define IN_DOCK_N 3
#define IN_BT_CONNECT 4
#define IN_USER_N 5
#define IN_BT_DISC 6
#define IN_VSENSE_BAT 7

// Port B
#define OUT_ACCEL_SEL1 0
#define OUT_ACCEL_SLEEP_N 3
#define OUT_BT_SW_N 4
#define IN_CPU_PROG_MOSI 5
#define OUT_CPU_PROG_MISO 6
#define IN_CPU_PROG_SCLK 7


// Port C
#define OUT_LED_YE 3


// Port D
#define IN_BT_RXD 0
#define OUT_BT_TXD 1
#define OUT_BT_RESET_N 2
#define OUT_LED_GN 3

#else
// Port Definitions
// Port A
#define IN_VSENSE_COMP 0
#define IN_ACCEL_Z_FILT 1
#define IN_ACCEL_Y_FILT 2
#define IN_ACCEL_X_FILT 3
#define IN_VSENSE_BAT 4
#define IN_USER_N 5
#define PA6 6
#define PA7 7

// Port B
#define OUT_ACCEL_SEL1 0
#define OUT_ACCEL_SEL2 1
#define IN_VSENSE_COMP 2
#define OUT_ACCEL_SLEEP_N 3
#define OUT_BT_SW_N 4
#define IN_CPU_PROG_MOSI 5
#define OUT_CPU_PROG_MISO 6
#define IN_CPU_PROG_SCLK 7


// Port C
#define PC0 0
#define OUT_LED_GN 1
#define OUT_LED_YE 2
#define PC3 3
#define PC4 4
#define PC5 5
#define PC6 6
#define PC7 7


// Port D
#define IN_BT_RXD 0
#define OUT_BT_TXD 1
#define OUT_BT_RESET_N 2
#define IN_VIB_SW_N 3
#define IN_BT_CONNECT 4
#define IN_BT_DISC 5
#define PD6 6
#define PD7 7

#endif

/*  Accelerometer Constants */

#ifdef _VERSION == 3
#define _4G 0
#define _12G 1
#else
#define _1_5G 0
#define _2G 1
#define _4G 2
#define _6G 3
#endif


/* Wocket Status Bits Constants */

#define BIT0_BLUETOOTH_STATUS  0
#define BIT1_ACCELEROMETER_STATUS 1
#define BIT2_GREENLED_STATUS 2
#define BIT3_YELLOWLED_STATUS 3


/* Exported Function Prototypes */

/* MCU Specific Functions */
void _atmega_initialize(unsigned char timer_prescalar);
void _atmega_disable_JTAG(void);
void _atmega_disable_watchdog(void);
void _atmega_enable_timer2(unsigned char timer_prescalar);
void _atmega_disable_timer2(void);
void _atmega_finalize(void);
void _atmega_initialize_uart0(unsigned int baud, unsigned char mode);
void _atmega_initialize_uart1(unsigned int baud, unsigned char mode);
unsigned short _atmega_a2dConvert10bit(unsigned char channel);
void _atmega_reset(void);
void _atmega_adc_turn_on(void);
void _atmega_adc_turn_off(void);
void _wocket_reset(void);

/* LED Specific Functions */
void _greenled_turn_on(void);
void _greenled_turn_off(void);
unsigned char _is_greenled_on(void);

void _yellowled_turn_on(void);
void _yellowled_turn_off(void);
unsigned char _is_yellowled_on(void);

/* Bluetooth Specific Functions */

unsigned char _bluetooth_initialize(unsigned baudrate);
unsigned char _bluetooth_enter_command_mode(void);
unsigned char _bluetooth_exit_command_mode(void);
void _bluetooth_reset(void);
unsigned char _bluetooth_get_baud_rate(void);
unsigned char _bluetooth_set_baud_rate(unsigned char baudrate);
void _bluetooth_turn_on(void);
void _bluetooth_turn_off(void);
unsigned char _is_bluetooth_on(void);
unsigned char _bluetooth_is_connected(void);
unsigned char _bluetooth_is_discoverable(void);
void _bluetooth_transmit_uart0_byte(unsigned char data);
unsigned char _bluetooth_receive_uart0_byte(unsigned char *data);
void _receive_uart0_flush(void);

/* Accelerometer Specific Functions */
unsigned char _accelerometer_set_sensitivity(unsigned char sensitivity);
void _accelerometer_turn_on(void);
void _accelerometer_turn_off(void);
unsigned char _is_accelerometer_on(void);
unsigned char _is_docked(void);


/* Variables */

//The variable stores the status for different wocket priephrals
unsigned char atmega_status;
unsigned char atmega_clock_prescalar;


/* Macros */
#define 	set(addr, data)   addr = (data)
#define 	get(addr)   (addr)
#define cbi(reg,bit)    reg &= ~(BV(bit)) // Macro that clears a bit
#define sbi(reg,bit)    reg |= (BV(bit)) // Macro that sets a bit
#define BV(bit)         (1<<(bit)) //Macro that shifts a 1 to  a particular bit

