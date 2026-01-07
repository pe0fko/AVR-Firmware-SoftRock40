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
//** Programmer...: F.W. Krom, PE0FKO
//**                I like to thank Francis Dupont, F6HSI for checking the
//**                algorithm and add some usefull comment!
//**                Thanks to Tom Baier DG8SAQ for the initial program.
//** 
//** Description..: Calculations for the Si570 chip and Si570 program algorithme.
//**                Changed faster and precise code to program the Si570 device.
//**
//** History......: V15.1 02/12/2008: First release of PE0FKO.
//**                Check the main.c file
//**
//**************************************************************************

#include "main.h"

#if defined(DEVICE_SI570)

EEMEM	var_t		E;									// Variables in eeprom
		var_t		R									// Variables in ram
					=									// Variables in flash rom
{		.RC_OSCCAL			= 0xFF						// CPU osc tune value
,		.ConfigFlags		= CONFIG_ABPF				// Only ABPF selected
,		.FreqXtal			= Chip_Freq_Xtal			// crystal frequency[MHz] [8.24] 114.285MHz 
,		.Freq				= 0x03866666				// Running frequency[MHz] [11.21] 28.2MHz / 4 = 7.050MHz
,		.SmoothTunePPM		= 3500						// SmoothTunePPM
,		.Band2CrossOver[0]	= {  4.0 * 4.0 * _2(5) }	// Default filter cross over
,		.Band2CrossOver[1]	= {  8.0 * 4.0 * _2(5) }	// frequnecy for softrock V9
,		.Band2CrossOver[2]	= { 16.0 * 4.0 * _2(5) }	// BPF. Four value array.
,		.Band2CrossOver[3]	= { 1 }						// ABPF is default enabled
,		.Band2Filter		= {	0,            1,            2,            3            }
,		.Band2Subtract		= {	0.0 * _2(21), 0.0 * _2(21), 0.0 * _2(21), 0.0 * _2(21) }
,		.Band2Multiply		= {	1.0 * _2(21), 1.0 * _2(21), 1.0 * _2(21), 1.0 * _2(21) }
,		.SerialNumber		= '0'						// Default USB SerialNumber ID.
,		.SiChipDCOMin		= 4850						// min VCO frequency 4850 MHz
,		.SiChipDCOMax		= 5670						// max VCO frequency 5670 MHz
,		.SiChipGrade		= Chip_Grade_Default				// Si570 chip grade C default (save)
,		.Si570RFREQIndex	= RFREQ_DEFAULT_INDEX		// Index for the RFFREQ registers
,		.IntrMaskIo			= IO_BIT_MASK				//
,		.ChipCrtlData		= 0x55						// I2C address or ChipCrtlData
,		.MinimalOutputFreqeuency	= 0					// 
,		.MaximalOutputFreqeuency	= 0					// 
};

chip_t	ChipInfo = 
{	.chipID					= CHIP_SI570
,	.chipGrade				= Chip_Grade_Default
,	.chipXTal				= Chip_Freq_Xtal			// Factory Xtal freq, not changed by calibrate!
};

		Si_Reg_t	Si_Reg_Data;						// Si570 register values
		uint8_t		Chip_OffLine;						// Si570 offline
static	uint32_t	FreqSmoothTune;						// The smooth tune center frequency
static	uint16_t	Si570_N;							// Total division (N1 * HS_DIV)
static	uint8_t		Si570_N1;							// The slow divider
static	uint8_t		Si570_HS_DIV;						// The high speed divider

static	void		Si570WriteSmallChange(void);
static	void		Si570WriteLargeChange(void);

#include "mul_div.h"


// Cost: 140us
// This function only works for the "C" & "B" grade of the Si570 chip.
// It will not check the frequency gaps for the "A" grade chip!!!
static uint8_t
Si570CalcDivider(uint32_t freq)
{
	// Register finding the lowest DCO frequenty
	uint8_t		xHS_DIV;
	sint16_t	xN1;
	uint16_t	xN;

	// Registers to save the found dividers
	uint8_t		sHS_DIV	= 0;
	uint8_t		sN1		= 0;
	uint16_t	sN		= 11*128;		// Total dividing
	uint16_t	N0;						// Total divider needed (N1 * HS_DIV)
	sint32_t	Freq;

	Freq.dw = freq;

	// Find the total division needed.
	// It is always one to low (not in the case reminder is zero, reminder not used here).
	// 16.0 bits = 13.3 bits / ( 11.5 bits >> 2)
	N0 = (R.SiChipDCOMin * (uint16_t)(_2(3))) / (Freq.w1.w >> 2);

	for(xHS_DIV = 11; xHS_DIV > 3; --xHS_DIV)
	{
		// Skip the unavailable divider's
		if (xHS_DIV == 8 || xHS_DIV == 10)
			continue;

		// Calculate the needed low speed divider
		xN1.w = N0 / xHS_DIV + 1;

		if (xN1.w > 128)
			continue;

		// Skip the unavailable N1 divider's
		if (xN1.b0 != 1 && (xN1.b0 & 1) == 1)
			xN1.b0 += 1;

		if (R.SiChipGrade == CHIP_GRADE_A)
		{
			// No divider restrictions!
		}
		else
		if (R.SiChipGrade == CHIP_GRADE_B)
		{
			if ((xN1.b0 == 1 && xHS_DIV == 4)
			||	(xN1.b0 == 1 && xHS_DIV == 5))
			{
				continue;
			}
		}
		else
		if (R.SiChipGrade == CHIP_GRADE_C)
		{
			if ((xN1.b0 == 1 && xHS_DIV == 4)
			||	(xN1.b0 == 1 && xHS_DIV == 5)
			||	(xN1.b0 == 1 && xHS_DIV == 6)
			||	(xN1.b0 == 1 && xHS_DIV == 7)
			||	(xN1.b0 == 1 && xHS_DIV == 11)
			||	(xN1.b0 == 2 && xHS_DIV == 4)
			||	(xN1.b0 == 2 && xHS_DIV == 5)
			||	(xN1.b0 == 2 && xHS_DIV == 6)
			||	(xN1.b0 == 2 && xHS_DIV == 7)
			||	(xN1.b0 == 2 && xHS_DIV == 9)
			||	(xN1.b0 == 4 && xHS_DIV == 4))
			{
				continue;
			}
		} 
		else
		if (R.SiChipGrade == CHIP_GRADE_D)
		{
			if ((xN1.b0 == 1 && xHS_DIV == 4)
			||	(xN1.b0 == 1 && xHS_DIV == 5)
			||	(xN1.b0 == 1 && xHS_DIV == 6)
			||	(xN1.b0 == 1 && xHS_DIV == 7)
			||	(xN1.b0 == 1 && xHS_DIV == 11)
			||	(xN1.b0 == 2 && xHS_DIV == 4)
			||	(xN1.b0 == 2 && xHS_DIV == 5)
			||	(xN1.b0 == 2 && xHS_DIV == 6)
			||	(xN1.b0 == 2 && xHS_DIV == 7)
			||	(xN1.b0 == 2 && xHS_DIV == 9))
			// Removing the 4*4 is out of the spec of the C grade chip, it may work!
//			||	(xN1.b0 == 4 && xHS_DIV == 4))
			{
				continue;
			}
		}
		else
		{
		}

		xN = xHS_DIV * xN1.b0;
		if (sN > xN)
		{
			sN		= xN;
			sN1		= xN1.b0;
			sHS_DIV	= xHS_DIV;
		}
	}

	if (sHS_DIV == 0)
		return false;

	Si570_N      = sN;
	Si570_N1     = sN1;
	Si570_HS_DIV = sHS_DIV;

	return true;
}

// frequency [MHz] * 2^21
static uint8_t
Si570CalcRFREQ(uint32_t freq, uint8_t index)
{
	sint64_t	RFREQ;

	//============================================================================
	// RFREQ = freq * sN * 8 / Xtal
	//============================================================================
	// freq  = F * 2^21 ==> [11.21] bits
	// xtal  = F * 2^24 ==> [8.24]  bits
	// sN    = 4..1408  ==> [11.0]  bits
	// RFREQ = F * 2^28 ==> [10.28] bits
	//
	// Calculation:
	// 1- DCO = freq * Si570_N	=> [11.21] * [11.0]	=> [22.21](43)
	// 2- DCO Freq between 4850...5670 MHz, change [22.21] to [13.21](34)
	// 3- RFREQ = DCO / xtal		=> [13.21] / [8.24]	=> [5.-3] R28+3 => [5.28]
	// 
	//============================================================================

	RFREQ.ll = umul_48_32_16(freq, Si570_N);

	// Check if DCO is lower than the Si570 max specified.
	if ( (RFREQ.ll >> 21) > R.SiChipDCOMax)
		return false;

	// 3- RFREQ = DCO / xtal		=> [13.21] / [8.24]	=> [5.-3] R28+3 => [5.28]
	RFREQ.ll = udiv_48_48_32_R(RFREQ.ll, R.FreqXtal, 3 + 28);

	// Convert divider ratio to SI570 register value
	Si_Reg_Data.N1_HS_DIV		= ( (Si570_HS_DIV - 4) << 5 ) | ( (Si570_N1 - 1) >> 2 );	// HS_DIV[2:0] << 5 | N1[6:2]
	Si_Reg_Data.N1_RFREQ_37_32	= ( (Si570_N1 - 1) & 0x03 ) << 6;							// N1[1:0] RFREQ[37:32](later)
	Si_Reg_Data.N1_RFREQ_37_32 |= RFREQ.l1.w0.b0;
	Si_Reg_Data.RFREQ_31_24		= RFREQ.l0.w1.b1;
	Si_Reg_Data.RFREQ_23_16		= RFREQ.l0.w1.b0;
	Si_Reg_Data.RFREQ_15_8		= RFREQ.l0.w0.b1;
	Si_Reg_Data.RFREQ_7_0		= RFREQ.l0.w0.b0;

//	161,1328125 MHz
//	0x01	0xC2	0xD1	0xE1	0x27	0xB2
//	148.35 MHz
//	0xA0	0xC2	0xEB	0xB0	0x4C	0xDC
//	100.0 MHz
//	0x22	0x42	0xBC	0x01	0x1E	0xBC

	return 1;
}

#if 1
uint64_t	A0;
uint64_t	A1;
uint64_t	A2;
long double	D1;
uint32_t	_freq;
uint64_t	_dF1;
uint64_t	_dF2;
uint32_t		_B0,_B1;
#endif

static uint8_t
Si570SmallChange(uint32_t frequency)
{
	sint64_t	dF;
//	uint32_t	delta_F, 
	uint32_t	delta_F_MAX;

/*	R.SmoothTunePPM: 0x0DAC
 *	frequency	FreqSmoothTune	dF.ll				*15					delta_F_MAX
 *	0x0c800000	0x00000000		0x000000000C800000	0x00000000BB800000	0x00000000	
 *	0x0c8a8f5c	0x0c800000		0x00000000000A8F5C	0x009e6664			0x00aae600		0x009e6664 <= 0x00aae600
*/

//	sint32_t previous_Frequency;

	// Get previous_Frequency   -> [11.21]
//	previous_Frequency.dw = FreqSmoothTune;

	// Delta_F (MHz) = |current_Frequency - FreqSmoothTune|  -> [11.21]
	dF.ll = (uint64_t)frequency - FreqSmoothTune;
	_dF1 = dF.ll;

	if (dF.l1.w1.b1 & 0x80)				// Check for negative value
		dF.ll = 0 - dF.ll;				// Make it always positive

//	delta_F = frequency - FreqSmoothTune;
//	if (delta_F >= _2(31)) 
//		delta_F = 0 - delta_F;

	// Delta_F (Hz) = (Delta_F (MHz) * 1_000_000) >> 16 not possible, overflow
	// replaced by:
	// Delta_F (Hz) = (Delta_F (MHz) * (1_000_000 >> 16)
	//              = Delta_F (MHz) * 15  (instead of 15.258xxxx)
	// Error        = (15 - 15.258) / 15.258 = 0.0169 < 1.7%

//	delta_F = delta_F * 15;          // [27.5] = [11.21] * [16.0]
	dF.l0.dw = dF.l0.dw * 15;          // [27.5] = [11.21] * [16.0]
	_dF2 = dF.ll;

	// Compute delta_F_MAX (Hz)= previous_Frequency(MHz) * 3500 ppm
//	delta_F_MAX = (uint32_t)previous_Frequency.w1.w * R.SmoothTunePPM;
	delta_F_MAX = (uint32_t)((sint32_t)FreqSmoothTune).w1.w * R.SmoothTunePPM;
	//   [27.5] =                          [11.5] * [16.0]
	_B0 = delta_F_MAX;
	_B1 = dF.l0.dw;
	
	// return TRUE if output changes less than ±3500 ppm from the previous_Frequency
//	return (delta_F <= delta_F_MAX) ? true : false;
	return (dF.l0.dw <= delta_F_MAX) ? true : false;
}


// Set the freq in the Si570.
// Use the possible smooth tuning.
void
SetFreqDevice(uint32_t freq, uint8_t index)
{
	// Check low / high frequency within range of the chip.
	if ((R.SiChipGrade == CHIP_GRADE_D)
	||  ((freq >= R.MinimalOutputFreqeuency) && (freq <= R.MaximalOutputFreqeuency)) )
	{
		if ((R.SmoothTunePPM != 0) && Si570SmallChange(freq))
		{
			Si570CalcRFREQ(freq, index);
			Si570WriteSmallChange();
//PORTB |= _BV(PB4);		// 1
		}
		else
		{
			if (!Si570CalcDivider(freq) || !Si570CalcRFREQ(freq, index))
				return;

			FreqSmoothTune = freq;
			Si570WriteLargeChange();
//PORTB &= ~_BV(PB4);		// 0
		}
	}
}


// Check Si570 old/new 'signature' 07h, C2h, C0h, 00h, 00h, 00h
static uint8_t
Check_Signature()
{
	static PROGMEM const uint8_t signature[] = { 0x07, 0xC2, 0xC0, 0x00, 0x00, 0x00 };
	uint8_t i;

	if (!Si_ReadRegisters(RFREQ_13_INDEX))
		return true;

	for(i = 0; i < sizeof(signature); ++i)
		if (pgm_read_byte(&signature[i]) != Si_Reg_Data.bData[i])
			break;

	return i == 6;	//sizeof(signature);
}

static void
Auto_index_detect_RFREQ(void)
{
	if ((R.Si570RFREQIndex & RFREQ_INDEX) == RFREQ_DEFAULT_INDEX)
	{
		// First RECALL the Si570 to default settings.
		Si_CmdReg(135, 0x01);
		_delay_us(100.0);

		// Check if signature found, then it is a old or new 50/20ppm chip
		// If not found it must be a *new* Si570 7ppm chip!
		if (Check_Signature())
		{
			R.Si570RFREQIndex &= RFREQ_FREEZE;
			R.Si570RFREQIndex |= RFREQ_7_INDEX;
		}
		else
		{
			R.Si570RFREQIndex &= RFREQ_FREEZE;
			R.Si570RFREQIndex |= RFREQ_13_INDEX;
		}
	}
}

void
DeviceInit(void)
{
	R.MinimalOutputFreqeuency = CHIP_MinimalOutputFreqeuency;

	switch (R.SiChipGrade) {
		case CHIP_GRADE_A: default:
			R.MaximalOutputFreqeuency = CHIP_MaximalOutputFreqeuency_A;
			break;
		case CHIP_GRADE_B:
			R.MaximalOutputFreqeuency = CHIP_MaximalOutputFreqeuency_B;
			break;
		case CHIP_GRADE_C:
			R.MaximalOutputFreqeuency = CHIP_MaximalOutputFreqeuency_C;
			break;
	}

//	DDRB  |= _BV(PB4);		// Output
//	PORTB &= ~_BV(PB4);		// 0

	Chip_OffLine = true;
}

void
DeviceOnline(void)
{
	// Check if Si570 is online and intialize if nessesary
	// SCL Low is now power on the SI570 chip in the Softrock V9
	if ((I2C_PIN & _BV(BIT_SCL)) != 0)
	{
		if (Chip_OffLine)
		{
			FreqSmoothTune = 0;				// Next SetFreq call no smoodtune

			Auto_index_detect_RFREQ();
			SetFreq(R.Freq, 0);

			Chip_OffLine = I2CErrors;
		}
	}
	else 
	{
		Chip_OffLine = true;
	}
}

static uint8_t
Si_CmdStart(uint8_t cmd)
{
	I2CSendStart();
	I2CSendByte((R.ChipCrtlData<<1)|0);	// send device address 
	if (I2CErrors == 0)
	{
		I2CSendByte(cmd);				// send Byte Command
		return true;
	}
	return false;
}

void
Si_CmdReg(uint8_t reg, uint8_t data)
{
	if (Si_CmdStart(reg))
	{
		I2CSendByte(data);
	}
	I2CSendStop();
}

// write all registers in one block from Si_Reg_Data
static void
Si570WriteRFREQ(void)
{
	if (Si_CmdStart(R.Si570RFREQIndex & RFREQ_INDEX))	// send Byte address 7/13
	{
		uint8_t i;
		for (i=0;i<6;i++)				// all 6 registers
			I2CSendByte(Si_Reg_Data.bData[i]);// send data 
	}
	I2CSendStop();
}

// read all registers in one block to Si_Reg_Data
uint8_t
Si_ReadRegisters(uint8_t index)
{
	if (Si_CmdStart(index & RFREQ_INDEX))	// send reg address 7 or 13
	{
		uint8_t i;
		I2CSendStart();
		I2CSendByte((R.ChipCrtlData<<1)|1);
		for (i=0; i<5; i++)
		{
			Si_Reg_Data.bData[i] = I2CReceiveByte();
			I2CSend0();					// 0 more bytes to follow
		}
		Si_Reg_Data.bData[5] = I2CReceiveByte();
		I2CSend1();						// 1 Last byte
	}
	I2CSendStop(); 

	return I2CErrors ? 0 : 6;
}

static void
Si570WriteSmallChange(void)
{
	if (R.Si570RFREQIndex & RFREQ_FREEZE)
	{
		// Prevents interim frequency changes when writing RFREQ registers.
		Si_CmdReg(135, 1<<5);		// Freeze M
		if (I2CErrors == 0)
		{
			Si570WriteRFREQ();
			Si_CmdReg(135, 0<<5);	// unFreeze M
		}
	}
	else
	{
		Si570WriteRFREQ();
	}
}

static void
Si570WriteLargeChange(void)
{
	Si_CmdReg(137, 1<<4);			// Freeze NCO
	if (I2CErrors == 0)
	{
		Si570WriteRFREQ();
		Si_CmdReg(137, 0<<4);		// unFreeze NCO
		Si_CmdReg(135, 1<<6);		// NewFreq set (auto clear)
	}
}

#endif

