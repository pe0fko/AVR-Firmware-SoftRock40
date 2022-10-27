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
//**
//** Description..: Calculations for the Si570 chip and Si570 program algorithme.
//**                Changed faster and precise code to program the Si570 device.
//**
//** History......: V15.16 24/03/2018: First release of PE0FKO.
//**                Check the main.c file
//**
//**************************************************************************

#include "main.h"

#if defined(DEVICE_SI549)

EEMEM	var_t		E;											// Variables in eeprom
		var_t		R											// Variables in ram
					=											// Variables in flash rom
{		.RC_OSCCAL					= 0xFF						// CPU osc tune value
,		.ConfigFlags				= 0							// No ABPF selected
,		.FreqXtal					= Chip_Freq_Xtal			// crystal frequency[MHz], 152.6MHz, [8.24](32), calibrated
,		.Freq						= 0x0C800000				// Running startup frequency, 100.0MHz, [11.21](32)
,		.SmoothTunePPM				= 950						// SmoothTunePPM Si549
,		.Band2CrossOver[0]			= {  4.0 * 4.0 * _2(5) }	// Default filter cross over
,		.Band2CrossOver[1]			= {  8.0 * 4.0 * _2(5) }	// frequnecy for softrock V9
,		.Band2CrossOver[2]			= { 16.0 * 4.0 * _2(5) }	// BPF. Four value array.
,		.Band2CrossOver[3]			= { 0 }						// ABPF is default disabled
,		.Band2Filter				= {	0,            1,            2,            3            }
,		.Band2Subtract				= {	0.0 * _2(21), 0.0 * _2(21), 0.0 * _2(21), 0.0 * _2(21) }
,		.Band2Multiply				= {	1.0 * _2(21), 1.0 * _2(21), 1.0 * _2(21), 1.0 * _2(21) }
,		.SerialNumber				= '0'						// Default USB SerialNumber ID.
,		.SiChipDCOMin				= 10800						// min VCO frequency 10.800,000000 MHz
,		.SiChipDCOMax				= 12511						// max VCO frequency 12.511,886114 MHz
,		.SiChipGrade				= Chip_Grade_Default		// Si570 chip grade A default (save)
,		.Si570RFREQIndex			= 23						// Not used in the Si549
,		.IntrMaskIo					= IO_BIT_MASK				//
,		.ChipCrtlData				= 0x55						// I2C address or ChipCrtlData
,		.MinimalOutputFreqeuency	= 0							//
,		.MaximalOutputFreqeuency	= 0							//
};

chip_t	ChipInfo = 
{	.chipID					= CHIP_SI549				// Firmware compiled for the Si549
,	.chipGrade				= Chip_Grade_Default		// The Si549 chip grade, used in this device
,	.chipXTal				= Chip_Freq_Xtal			// Factory Xtal freq, not changed by calibrate!
};

		Si_Reg_t	Si_Reg_Data;					// Si549 register values
		uint8_t		Chip_OffLine;					// Si549 offline
static	uint32_t	NonimalFreq;					// The smooth tune center frequency

static	void		Si549WriteNewFrequencyRegisters(void);
static	void		Si549WritePPMRegisters(void);

#include "mul_div.h"


static inline
uint8_t LOG2(uint8_t n)
{
	uint8_t i = 0;
	while(n) { n >>= 1; i++; };
	if (i > 5) i = 5;	// max 2^5
	return i;
}

static void
CalculateFrequencyRegisters(uint32_t freq)	// [11.21]
{
	sint32_t	dco;
	uint8_t		lsdiv;						// Low speed division register value (0..7)
	uint16_t	hsdiv;						// High speed division value (5..2046)

	// 0.2MHz	0x00066666	10800 / 0.1875 [.5] = 57600	=>	lsdiv = log2(57600 / 2046) = log2(ceil(28,152492)) = 5
	//								^^^^^^  ^^				hsdiv = 57600 / 2^5 = 1800
	// 0.2MHz	0x00066666	10800 / 0.2	        = 54000	=>	lsdiv = log2(54000 / 2046) = log2(ceil(26,392961)) = 5
	//														hsdiv = 54000 / 2^5 = 1687,5

	dco.w1.w = R.SiChipDCOMin;				// DCO [16.16]
	dco.w0.w = 0;							// DCO [16.16]

	dco.dw /= ((sint32_t)freq).w1.w;		// DCO [16.16] / [11.5] => [5.11]

	uint16_t div = dco.dw >> 11;			// Total divider value
	if (dco.w0.w & 0x07FF) div += 1;		// Ceiling of div

	lsdiv = LOG2( div / 2046 );				// Find the log2 of the low speed binary divider (2^N, N=0..5)
	hsdiv = div >> lsdiv;					// Find the high speed divider value
	if (lsdiv == 0 && hsdiv >= 34)			// If low speed is zero and high speed higher then 33
		if (hsdiv & 1)						//   only the even values may be used
			hsdiv += 1;

	Si_Reg_Data.HSDIV_7_0				= hsdiv & 0xFF;
	Si_Reg_Data.LSDIV_2_0_HSDIV_10_8	= ((hsdiv >> 8) & 0x07) | (lsdiv << 4);

	//	VCO = Freq * DIV	=> [11.21](32) * [16.0](16)	=> [27.21](48)
	sint64_t	vco;
	vco.ll = umul_48_32_16(freq, hsdiv << lsdiv);
	
	// Test on VCO Max with SiChipDCOMax?

	//	REG = VCO / Xtal	=> [25.21](46) / [8.24](32) => [17.-3](14+32Remainder) => R32 [14.32](46)	(LT 16GHz => 14bits)
	vco.ll = udiv_48_48_32_R(vco.ll, R.FreqXtal, 3 + 32);

	Si_Reg_Data.FBDIV_7_0		= vco.l0.w0.b0;
	Si_Reg_Data.FBDIV_15_8		= vco.l0.w0.b1;
	Si_Reg_Data.FBDIV_23_16		= vco.l0.w1.b0;
	Si_Reg_Data.FBDIV_31_24		= vco.l0.w1.b1;
	Si_Reg_Data.FBDIV_39_32		= vco.l1.w0.b0;
	Si_Reg_Data.FBDIV_42_40		= vco.l1.w0.b1;
}

static uint8_t
Si549SmallChange(uint32_t frequency)
{
	sint64_t	dF;
	uint8_t		negative;			// Keep track of negative dF

/*
 *	dF		= ABS( F1 - F2 )		=> [11.21] - [11.21] => [11.21](32)
 *	1500MHz => 1500 * 950	=> dF_max = 1.425.000
 *	Check (last bytes zero)		|dF| < 2.000.000	dF = [1.21]
 *
 *	1.000.000 => 0x000F4240
 *	The divide need minimal 0.5/1500 resolution, 12 bits. Take [10.14] to get [.24] (byte boundary).
 *	PPM = 1.000.000 * dF / F1		=> [20.0] * [1.21] / [11.21]	=> [21.21](42) / [11.21]	=> [10.0]	R14 [10.14](24)
 *
 *	1.000.000 = 0xF4240		=> 1.000.000 / 2^6 = 15625.0	=> 0x3D09 [14.0]
 *	PPM = (1000000/2^6) * dF / F1 * 2^6	=> [14.0] * [1.21] / [11.21]	=> [15.21](36) / [11.21]	=> [4.0]	R6+14 [10.14](24)
 *	PPM = 0x3D09 * dF / F1 * 2^6		=> [14.0] * [1.21] / [11.21]	=> [15.21](36) / [11.21]	=> [4.0]	R6+14 [10.14](24)
 *
 *	PPMreg = PPM / 0.0001164		=> PPM * 0x863C42	=> [10.14] * [14.10]	=> [24.24](44)
 *	PPMreg = PPM * 8591,064453125	=> PPM * 0x863C42	=> [10.14] * [14.10]	=> [24.24](44)
 *	PPMreg = PPM * 8591				=> PPM * 0x00218f	=> [10.14] * [14.0]		=> [24.14](38)
 *	PPMreg = PPM * 8591,0644 * 4	=> PPM * 34364.25	=> [10.14] * [14.4]		=> [24.18](42)
 *	PPMreg = PPM * 8591,0644 * 4	=> PPM * 34364.00	=> [10.14] * [14.2]		=> [24.16](40)	<<-- Best one
 *	
 *	Direct calc: no ppm check direct possible?
 *	PPMreg = 1.000.000 * dF / F1 / 0.0001164	=> 8591065292 * dF / F1	=> [34.0] * [1.21] / [11.21]	=> [35.21](56) / [11.21](32) => [24.0]
 */

	if (NonimalFreq == 0)						// Freq chip unknow for now,
		return false;							//   do the full sequence!

	// Calculate the frequency difference, [11.21] - [11.21] => [11.21](32)
	dF.ll = (uint64_t)frequency - NonimalFreq;

	negative = dF.l1.w1.b1 & 0x80;				// Check for negative value
	if (negative)								// Make it always positive
		dF.ll = 0 - dF.ll;

	// Check if dF is smaller then 2.0MHz 
	// dF = [1.21] => 0xFFC0.0000
	if ( dF.l0.w1.w & 0xFFC0 )					// If dF >= 2MHz
			return false;						//   then exit

	// Multiply dF by 1.000.000 / 2^6 (0xF4240 = 1.000.000, 0x3D09 = 15.625)
	// PPM = [1.21] * [14.0] => [15.21](36)
	dF.ll = umul_48_32_16(dF.l0.dw, 15625);

	//	PPM = dF / F1 * 2^6
	//	[15.21](36) / [11.21]	=> [4.0]	R6+14 [10.14](24)
	dF.ll = udiv_48_48_32_R(dF.ll, NonimalFreq, 6 + 14);

	// Check if the PPM value is below 950+1 (+1 we don't check the fraction)
	if ((dF.ll >> 14) >= R.SmoothTunePPM)		// [10.14](24)
		return false;  

	// Multiply by the magic number => 1 / 0.0001164 = 8591 [14.0]
	// 4 * 1/0.0001164 = 34364,2611
	// [10.14] * [14.2] => [24.16](40)
	dF.ll = umul_48_32_16(dF.l0.dw, 34364);

	if (negative)								// if negative,
		dF.ll = 0 - dF.ll;						//   change the sign of register

	// [24.16](40) => [24.0](24)
	Si_Reg_Data.ADPLL_DELTA_M_7_0	= dF.l0.w1.b0;
	Si_Reg_Data.ADPLL_DELTA_M_15_8	= dF.l0.w1.b1;
	Si_Reg_Data.ADPLL_DELTA_M_23_16	= dF.l1.w0.b0;

	return true;
}


// Set the freq in the Si549. Use the possible smooth tuning.
void
SetFreqDevice(uint32_t freq, uint8_t dummy)
{
	// Check low / high frequency within range of the chip.
	if ((R.SiChipGrade == CHIP_GRADE_D)
	||  ((freq >= R.MinimalOutputFreqeuency) && (freq <= R.MaximalOutputFreqeuency)) )
	{
		if ((R.SmoothTunePPM != 0) && Si549SmallChange(freq))
		{
			Si549WritePPMRegisters( );
		}
		else
		{
			NonimalFreq = freq;

			CalculateFrequencyRegisters( freq );

			Si549WriteNewFrequencyRegisters();
			
			if (( Si_Reg_Data.ADPLL_DELTA_M_7_0 != 0 )
			||	( Si_Reg_Data.ADPLL_DELTA_M_15_8 != 0 )
			||	( Si_Reg_Data.ADPLL_DELTA_M_23_16 != 0 ) ) 
			{
				Si_Reg_Data.ADPLL_DELTA_M_7_0 = 0;
				Si_Reg_Data.ADPLL_DELTA_M_15_8 = 0;
				Si_Reg_Data.ADPLL_DELTA_M_23_16 = 0;
				Si549WritePPMRegisters( );
			}
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

	Chip_OffLine = true;
}

void
DeviceOnline(void)
{
	// Check if Si549 is I2C on-line and initialize if necessary!
	if ((I2C_PIN & _BV(BIT_SCL)) != 0)
	{
		if (Chip_OffLine)
		{
			NonimalFreq = 0L;					// Next SetFreq call no smooth-tune
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

static void
Si549WriteNewFrequencyRegisters(void)
{
	Si_CmdReg(255, 0x00);							// CMD=255, Set page register to point to page 0
	if (I2CErrors == 0)
	{
		Si_CmdReg(69, 0x00);						// CMD=69, Disable FCAL overwrite
		Si_CmdReg(17, 0x00);						// CMD=17, Synchronously disable output

		if (Si_CmdStart(23))						// CMD=23 & 24
		{
			I2CSendByte( Si_Reg_Data.HSDIV_7_0 );	
			I2CSendByte( Si_Reg_Data.LSDIV_2_0_HSDIV_10_8 );
		}
		I2CSendStop();

		if (Si_CmdStart(26))						// CMD=26, 27 28, 29, 30, 31
		{
			I2CSendByte( Si_Reg_Data.FBDIV_7_0 );	// FBFRAC.w0.b0
			I2CSendByte( Si_Reg_Data.FBDIV_15_8 );	// FBFRAC.w0.b1
			I2CSendByte( Si_Reg_Data.FBDIV_23_16 );	// FBFRAC.w1.b0
			I2CSendByte( Si_Reg_Data.FBDIV_31_24 );	// FBFRAC.w1.b1
			I2CSendByte( Si_Reg_Data.FBDIV_39_32 );	// FBINT.b0
			I2CSendByte( Si_Reg_Data.FBDIV_42_40 );	// FBINT.b1
		}
		I2CSendStop();
		
		Si_CmdReg( 7, 0x08);						// CMD=7, Start FCAL
		Si_CmdReg(17, 0x01);						// CMD=17, Synchronously enable output
	}
}

static void
Si549WritePPMRegisters(void)
{
	if (Si_CmdStart(231))			// CMD=231, 232, 323
	{
		I2CSendByte( Si_Reg_Data.ADPLL_DELTA_M_7_0 );
		I2CSendByte( Si_Reg_Data.ADPLL_DELTA_M_15_8 );
		I2CSendByte( Si_Reg_Data.ADPLL_DELTA_M_23_16 );
	}
	I2CSendStop();
}

// read all registers in one block to Si_Reg_Data
uint8_t
Si_ReadRegisters(uint8_t index)
{
	uint8_t i;

	if (Si_CmdStart(23))							// Start at register 23
	{
		I2CSendStart();
		I2CSendByte((R.ChipCrtlData<<1)|1);
		Si_Reg_Data.bData[0] = I2CReceiveByte();	// Register 23
		I2CSend0();									// 0 more bytes to follow
		Si_Reg_Data.bData[1] = I2CReceiveByte();	// Register 24
		I2CSend1();									// 1 Last byte
	}
	I2CSendStop();

	if (Si_CmdStart(26))							// Start at register 26 until 31
	{
		I2CSendStart();
		I2CSendByte((R.ChipCrtlData<<1)|1);
		for (i=2; i < 7; i++) {						// Max 64 bytes
			Si_Reg_Data.bData[i] = I2CReceiveByte();
			I2CSend0();								// 0 more bytes to follow
		}
		Si_Reg_Data.bData[i] = I2CReceiveByte();
		I2CSend1();									// 1 Last byte
	}
	I2CSendStop();

	if (Si_CmdStart(231))							// Start at register 231 until 233
	{
		I2CSendStart();
		I2CSendByte((R.ChipCrtlData<<1)|1);
		Si_Reg_Data.bData[8] = I2CReceiveByte();
		I2CSend0();
		Si_Reg_Data.bData[9] = I2CReceiveByte();
		I2CSend0();
		Si_Reg_Data.bData[10] = I2CReceiveByte();
		I2CSend1();									// 1 Last byte
	}
	I2CSendStop();

	return I2CErrors ? 0 : 11;
}

#endif
 