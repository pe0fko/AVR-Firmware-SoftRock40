//************************************************************************
//**
//** Project......: Firmware USB AVR Si570 controler.
//**
//** Platform.....: ATtiny45
//**
//** Licence......: This software is freely available for non-commercial 
//**                use - i.e. for research and experimentation only!
//**                Copyright: (c) 2006 by OBJECTIVE DEVELOPMENT Software GmbH
//**                Based on ObDev's AVR USB driver by Christian Starkjohann
//**
//** Programmer...: F.W. Krom, PE0FKO and
//**                thanks to Tom Baier DG8SAQ for the initial program.
//** 
//** Description..: Control the Si570 Freq. PLL chip over the USB port.
//**
//** History......: V15.1 02/12/2008: First release of PE0FKO.
//**                Check the main.c file
//**
//**************************************************************************

#ifndef _PE0FKO_MAIN_H_
#define _PE0FKO_MAIN_H_ 1

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#if 1
#include "usbavrcmd.h"
#include "usbconfig.h"
#include "usbdrv.h"
#endif

#define	VERSION_MAJOR	15
#define	VERSION_MINOR	16

// Switch's to set the code needed
#define	INCLUDE_NOT_USED		1				// Compatibility old firmware, I/O functions
#define INCLUDE_TEMP			1				// Include the temperature code
#define INCLUDE_INTERRUPT		0				// Include the usb interrupt code

#define	DEVICE_SI549						// Code generation for the DPLL Si549 chip
//#define	DEVICE_SI570							// Code generation for the DPLL Si570 chip
//#define	DEVICE_AD9850						// Code generation for the DDS AD9850 chip

#if defined (__AVR_ATtiny45__) || defined (__AVR_ATtiny85__)

#define BIT_SDA			PB1
#define BIT_SCL 		PB3
#define	I2C_DDR			DDRB
#define	I2C_PIN			PINB

#define IO_P1			PB4			
#define IO_P2			PB5			// Also RESET

#define IO_PTT			IO_P1
#define IO_CW1			IO_P2
#define IO_CW2			BIT_SDA

#define	MAX_RX_BAND		(1<<2)		// Max of 4 band's

#elif defined (__AVR_ATmega328P__)

#define BIT_SDA			PB4
#define BIT_SCL 		PB1
#define	I2C_DDR			DDRB
#define	I2C_PIN			PINB

#define IO_P1			PB2
#define IO_P2			PB0

#define IO_PTT			IO_P1
#define IO_CW1			IO_P2
#define IO_CW2			BIT_SDA

#define	BPF_RX_NR_BITS	2			// Bits used by the RX Band pass filter
#define	MAX_RX_BAND		(1<<2)		// Max of 4 band's
#define BF_P0			

#else
#error Define correct CPU.
#endif

#define IO_DDR			DDRB
#define IO_PORT			PORTB
#define IO_PIN			PINB
//#define IO_BIT_START		PB4		// First I/O line used
//#define	IO_BIT_LENGTH		2		// Using 2 I/O line (PB4, PB5)
//#define	IO_BIT_MASK		( (1<<IO_BIT_LENGTH)-1 )
//#define IO_P1			( IO_BIT_START+0 )
//#define IO_P2			( IO_BIT_START+1 )

#define	IO_BIT_MASK		( _BV(IO_P1) | _BV(IO_P2) )


#define	true			1
#define	false			0

#define	_2(x)		((uint32_t)1<<(x))	// Take power of 2

#define	bit_1(port,bit)	port |= _BV(bit)	// Set bit to one
#define	bit_0(port,bit)	port &= ~_BV(bit)	// Set bit to zero

typedef union {
	uint16_t			w;
	struct { uint8_t	b0,b1;	};
} sint16_t;

typedef union {
	uint32_t			dw;
	struct { sint16_t	w0,w1;	};
} sint32_t;

typedef union {
	uint64_t			ll;
	struct { sint32_t	l0,l1;	};
} sint64_t;

// Config flags for the R.ConfigFlags
#define	CONFIG_ABPF				_BV(0)
#define	CONFIG_INTERRUPT		_BV(1)

// Interrupt commands
#define	INTR_CMD_IO_CHANGE		1
#define	INTR_CMD_FREQ_CHANGE	2

typedef struct 
{
		uint8_t		RC_OSCCAL;					// CPU osc tune value (must be addr 0)
		uint8_t		ConfigFlags;				// Configuration flags.
		uint32_t	FreqXtal;					// crystal frequency[MHz] ([8.24] or [9.23] Si549)
		uint32_t	Freq;						// Running frequency[MHz] (11.21bits)
		uint16_t	SmoothTunePPM;				// Max PPM value for the smooth tune
		sint16_t	Band2CrossOver[MAX_RX_BAND];// Filter cross over points [0..2] (11.5bits)
		uint8_t		Band2Filter[MAX_RX_BAND];	// Filter number for band 0..3
		uint32_t	Band2Subtract[MAX_RX_BAND];	// Freq subtract value[MHz] (11.21bits) for band 0..3
		uint32_t	Band2Multiply[MAX_RX_BAND];	// Freq multiply value (11.21bits) for band 0..3
		uint8_t		SerialNumber;				// Default serial number last char! ("PE0FKO-2.X")
		uint16_t	SiChipDCOMin;				// Si570 Minimal DCO value
		uint16_t	SiChipDCOMax;				// Si570 Maximal DCO value
		uint8_t		SiChipGrade;				// Si570 chip grade
		uint8_t		Si570RFREQIndex;			// [0..6bit]Index to be used for the RFFREQ registers (7-12, 13-18), [7bit] Use freeze RFREQ register
		uint8_t		IntrMaskIo;					// Bits that are valid to check.
		uint8_t		ChipCrtlData;				// I2C address, default 0x55 (85 dec)
		uint32_t	MinimalOutputFreqeuency;	// Minimal chip frequency
		uint32_t	MaximalOutputFreqeuency;	// Maximal chip frequency
} var_t;

extern			var_t	R;						// Variables in RAM
extern	EEMEM	var_t	E;						// Variables in EEPROM

//enum { chip_none, chip_Si570, chip_Si549, chip_AD9850 };
enum	{ CHIP_NONE, CHIP_SI570, CHIP_SI549xx, CHIP_AD9850, CHIP_SI549=0xc0	};
	
typedef struct __attribute__((__packed__)) {
	uint8_t		chipID;
	uint8_t		chipGrade;
	uint32_t	chipXTal;
} chip_t;

extern			chip_t	ChipInfo;				// Connected VFO chip

#if INCLUDE_INTERRUPT							// Include the usb interrupt code
typedef struct {
	uint8_t		cmd;
	union {
		struct { uint8_t data; } io;
		struct { uint32_t data; } freq;
	} x; 
} intrBuf_t;

extern	intrBuf_t	intrBufIO;				// Interrupt buffer cmd for IO changed
extern	intrBuf_t	intrBufFreq;			// Interrupt buffer cmd for Freq changed
#endif

extern	sint16_t	replyBuf[4];			// USB Reply buffer
extern	uint8_t		intrBuf[8];				// Buffer used for the interrupt data

extern	void		Si_CmdReg(uint8_t reg, uint8_t data);
extern	uint8_t		Si_ReadRegisters(uint8_t index);
extern	void		SetFreq(uint32_t freq, uint8_t freq_fine);
extern	void		SetFreqDevice(uint32_t freq, uint8_t );
extern	void		DeviceInit(void);
extern	void		DeviceOnline(void);
extern	uint16_t	GetTemperature(void);
extern	void		CalcFreqFromRegSi570(uint8_t* reg);


//-------------------------------------------------------------------------------------------------
//---- SiLabs SI570
//-------------------------------------------------------------------------------------------------
#if defined(DEVICE_SI570)

typedef union {
	uint8_t			bData[6];
	struct {								// Si570 (6 bytes)
		uint8_t		N1_HS_DIV;				// HS_DIV_2_0 << 5 | N1_6_2
		uint8_t		N1_RFREQ_37_32;			// N1[1:0] RFREQ[37:32]
		uint8_t		RFREQ_31_24;			// RFREQ[31:24]
		uint8_t		RFREQ_23_16;			// RFREQ[23:16]
		uint8_t		RFREQ_15_8;				// RFREQ[15:8]
		uint8_t		RFREQ_7_0;				// RFREQ[7:0]
	};
} Si_Reg_t;

#define Chip_Grade_Default		CHIP_GRADE_C			// Use rhe grade A chip by default
#define Chip_Freq_Xtal			0x7248F5C2				// crystal frequency[MHz] [8.24] 114.285MHz

// The divider restrictions for the 3 Si57x speed grades or frequency grades are as follows
// - Grade A covers 10 to 945 MHz, 970 to 1134 MHz, and 1213 to 1417.5 MHz. Speed grade A
//   device have no divider restrictions.
// - Grade B covers 10 to 810 MHz. Speed grade B devices disable the output in the following
//   N1*HS_DIV settings: 1*4, 1*5
// - Grade C covers 10 to 280 MHz. Speed grade C devices disable the output in the following
//   N1*HS_DIV settings: 1*4, 1*5, 1*6, 1*7, 1*11, 2*4, 2*5, 2*6, 2*7, 2*9, 4*4
#define	CHIP_GRADE_A			1			// Si570 Grade A device is used. (10 - 1417MHz)
#define	CHIP_GRADE_B			2			// Si570 Grade B device is used. (10 - 810MHz)
#define	CHIP_GRADE_C			3			// Si570 Grade C device is used. (10 - 280MHz)
// Removing the 4*4 is out of the spec of the C grade chip, it may work!
#define	CHIP_GRADE_D			4			// Si570 Grade C device is used. (10 - 354MHz)

#define CHIP_MinimalOutputFreqeuency		((uint32_t)(  10.0 * _2(21)))
#define	CHIP_MaximalOutputFreqeuency_A		((uint32_t)(1417.5 * _2(21)))
#define	CHIP_MaximalOutputFreqeuency_B		((uint32_t)( 810.0 * _2(21)))
#define	CHIP_MaximalOutputFreqeuency_C		((uint32_t)( 280.0 * _2(21)))

// Using register-bank auto (Check 'signature' 07h, C2h, C0h, 00h, 00h, 00h) , 7Index (50ppm, 20ppm), 13Index (7ppm)
#define	RFREQ_DEFAULT_INDEX		0			// 0 if AUTO index!
#define	RFREQ_7_INDEX			7
#define	RFREQ_13_INDEX			13
#define	RFREQ_INDEX				0x7F
#define	RFREQ_FREEZE			0x80

extern	Si_Reg_t				Si_Reg_Data;	// Registers 7..12 value for the Si570
extern	uint8_t					Chip_OffLine;	// Chip off-line

//-------------------------------------------------------------------------------------------------
//---- SiLabs SI549
//-------------------------------------------------------------------------------------------------
#elif defined(DEVICE_SI549)

typedef union {
	uint8_t			bData[11];
	struct {		// Si549 (11 bytes)
		uint8_t		HSDIV_7_0;
		uint8_t		LSDIV_2_0_HSDIV_10_8;
		uint8_t		FBDIV_7_0;
		uint8_t		FBDIV_15_8;
		uint8_t		FBDIV_23_16;
		uint8_t		FBDIV_31_24;
		uint8_t		FBDIV_39_32;
		uint8_t		FBDIV_42_40;
		uint8_t		ADPLL_DELTA_M_7_0;
		uint8_t		ADPLL_DELTA_M_15_8;
		uint8_t		ADPLL_DELTA_M_23_16;
	};
} Si_Reg_t;

#define	CHIP_GRADE_A			1			// Si Grade A device is used. (0,25 - 1500 MHz)
#define	CHIP_GRADE_B			2			// Si Grade B device is used. (0,25 - 800  MHz)
#define	CHIP_GRADE_C			3			// Si Grade C device is used. (0,25 - 325  MHz)
#define	CHIP_GRADE_D			4			// Si Grade D, not e real chip grade only no frequency check done.

#define CHIP_MinimalOutputFreqeuency		((uint32_t)(   0.2 * _2(21)))
#define	CHIP_MaximalOutputFreqeuency_A		((uint32_t)(1500.0 * _2(21)))
#define	CHIP_MaximalOutputFreqeuency_B		((uint32_t)( 800.0 * _2(21)))
#define	CHIP_MaximalOutputFreqeuency_C		((uint32_t)( 325.0 * _2(21)))

#define Chip_Grade_Default		CHIP_GRADE_A			// Use rhe grade A chip by default
#define Chip_Freq_Xtal			0x98999999				// Si549 Chip crystal frequency, 152.6MHz * [8.24](32)

extern	Si_Reg_t				Si_Reg_Data;			// Registers 7..12 value for the Si570
extern	uint8_t					Chip_OffLine;			// Chip off-line

//-------------------------------------------------------------------------------------------------
//---- Analog Devices AD9850
//-------------------------------------------------------------------------------------------------
#elif defined(DEVICE_AD9850)							// Code generation for the DDS AD9850 chip

#define	DEVICE_XTAL		( 100.0 * _2(24) )				// Clock of the DDS chip [8.24]
#define	DEVICE_I2C		( 0x00 )						// Used for DDS control / phase word
#define	DDS_PORT		PORTB
#define	DDS_DATA		PB1
#define	DDS_W_CLK		PB3
#define	DDS_FQ_UD		PB4

#else
#error Define one frequency device.
#endif

//-------------------------------------------------------------------------------------------------

//#define	I2C_KBITRATE	400.0			// I2C Bus speed in Kbs
#define	I2C_KBITRATE	200.0				// 400 was to high?!?

extern	uint8_t		I2CErrors;
extern	void		I2CSendStart(void);
extern	void		I2CSendStop(void);
extern	void		I2CSendByte(uint8_t b);
extern	void 		I2CSend0(void);
extern	void 		I2CSend1(void);
extern	uint8_t		I2CReceiveByte(void);

#if 0
#   define SWITCH_START(cmd)       switch(cmd){{
#   define SWITCH_CASE(value)      }break; case (value):{
#   define SWITCH_CASE2(v1,v2)     }break; case (v1): case(v2):{
#   define SWITCH_CASE3(v1,v2,v3)  }break; case (v1): case(v2): case(v3):{
#   define SWITCH_CASE6(v1,v2,v3,v4,v5,v6)  }break; case (v1): case(v2): case(v3): case(v4): case(v5): case(v6):{
#   define SWITCH_DEFAULT          }break; default:{
#   define SWITCH_END              }}
#else
#   define SWITCH_START(cmd)       {uchar _cmd = cmd; if(0){
#   define SWITCH_CASE(value)      }else if(_cmd == (value)){
#   define SWITCH_CASE2(v1,v2)     }else if(_cmd == (v1) || _cmd == (v2)){
#   define SWITCH_CASE3(v1,v2,v3)  }else if(_cmd == (v1) || _cmd == (v2) || (_cmd == v3)){
#   define SWITCH_CASE6(v1,v2,v3,v4,v5,v6)  }else if(_cmd == (v1) || _cmd == (v2) || _cmd == (v3) || _cmd == (v4) || _cmd == (v5) || _cmd == (v6)){
#   define SWITCH_DEFAULT          }else{
#   define SWITCH_END              }}
#endif


#endif
