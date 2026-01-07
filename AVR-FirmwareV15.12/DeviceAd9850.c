//************************************************************************
//**
//** Project......: Firmware USB AVR AD9850 controler.
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
//** Description..: Control the AD9850 with the Si570 register commands.
//**
//** History......: V15.1 02/12/2008: First release of PE0FKO.
//**                Check the main.c file
//**
//**************************************************************************

#include "main.h"

#if INCLUDE_AD9850

#include "CalcVFO.c"						// Include code is small size

#error "****** AD9850 Code not ready! ******"

static void
AD9850_OutputByte(uint8_t code)
{
	uint8_t i;

	for(i=0; i < 8; ++i)
	{
		if (code & 1)
			bit_1(DDS_PORT, DDS_DATA);
		else
			bit_0(DDS_PORT, DDS_DATA);

		bit_1(DDS_PORT, DDS_W_CLK);
		code >>= 1;
		bit_0(DDS_PORT, DDS_W_CLK);
	}
}

static void
AD9850_Load(uint32_t freq)
{
	sint32_t Freq;
	Freq.dw = freq;

	AD9850_OutputByte(Freq.w0.b0);
	AD9850_OutputByte(Freq.w0.b1);
	AD9850_OutputByte(Freq.w1.b0);
	AD9850_OutputByte(Freq.w1.b1);

	AD9850_OutputByte(R.ChipCrtlData);	// Phase / control word

	bit_1(DDS_PORT, DDS_FQ_UD);
	bit_0(DDS_PORT, DDS_FQ_UD);
}


void
AD9850_LoadFreq(uint32_t freq)
{
	uint8_t		cnt;
	uint32_t	RR;						// Division remainder
	uint32_t	Count;

	// DDS AD9850
	// Freq = Count * Xtal / (1<<32);
	// Count = Freq * 1(<<32) / Xtal
	// [43.21] = [11.21] * [32.0] / [8.24]
	// [43.21] = [43.21] / [8.24]
	// Count = Freq * 1(<<32) * 8 / Xtal
	// [43.21] = [40.24] / [8.24]

	//---------------------------------------------------------------------------
	// Quotient_32 = Dividend_32 / Divisor_32
	//---------------------------------------------------------------------------
	// Dividend_32: freq      b3      b2      b1      b0
	// Divisor_32 : FreqXtal
	// Quotient_32: Count     b4      b3      b2      b1      b0
	//---------------------------------------------------------------------------

	RR = 0;							// Clear Remainder_32
	cnt = 32+1+32+3;				// Init Loop_Counter
									// (32 = 0.32 bits, 3 = * 8)

	asm (
	"clc                 \n\t"		// Partial_result = carry = 0

"L_A_%=:                 \n\t"		// Repeat
	"rol %A0             \n\t"		//   Put last Partial_result in Quotient_32
	"rol %B0             \n\t"		//   and shift left Dividend_32 ...
	"rol %C0             \n\t"
	"rol %D0             \n\t"

	"rol %A2             \n\t"		//                      ... into Remainder_32
	"rol %B2             \n\t"
	"rol %C2             \n\t"
	"rol %D2             \n\t"

	"sub %A2,%A3         \n\t"		//   Remainder =  Remainder - Divisor_32
	"sbc %B2,%B3         \n\t"
	"sbc %C2,%C3         \n\t"
	"sbc %D2,%D3         \n\t"

	"brcc L_B_%=         \n\t"		//   If result negative
									//   Then
	"add %A2,%A3         \n\t"		//     Restore Remainder
	"adc %B2,%B3         \n\t"
	"adc %C2,%C3         \n\t"
	"adc %D2,%D3         \n\t"

	"clc                 \n\t"		//     Partial_result = 0
	"rjmp L_C_%=         \n\t"

"L_B_%=:                 \n\t"		//   Else
	"sec                 \n\t"		//     Partial_result = 1

"L_C_%=:                 \n\t"		//   End If
	"dec %1              \n\t"		// Until(--cnt == 0)
	"brne L_A_%=         \n\t"

	"adc %A0,__zero_reg__\n\t"		// Round by the last bit of Count
	"adc %B0,__zero_reg__\n\t"
	"adc %C0,__zero_reg__\n\t"
	"adc %D0,__zero_reg__\n\t"

"L_X_%=:                 \n\t"

	// Output operand list
	//--------------------
	: "=r" (Count)					// %0 -> Dividend_32

	// Input operand list
	//-------------------
	: "r" (cnt)                     // %1 -> Loop_Counter
	, "r" (RR)                      // %2 -> Remainder_32
	, "r" (R.FreqXtal)              // %3 -> Divisor_32
	, "0" (freq)
	);
	
	AD9850_Load(Count);
}


void
SetFreq(uint32_t freq)		// frequency [MHz] * 2^21
{
#if INCLUDE_FREQ_SM
	freq = CalcFreqMulAdd(freq);
#endif

	R.Freq = freq;

	AD9850_LoadFreq(freq);

#if INCLUDE_ABPF
	SetFilter();
#endif
}


void
DeviceInit(void)
{
	// Clock in parallel data
	bit_1(DDS_PORT, DDS_W_CLK);
	bit_0(DDS_PORT, DDS_W_CLK);

	// Enable serial mode
	bit_1(DDS_PORT, DDS_FQ_UD);
	bit_0(DDS_PORT, DDS_FQ_UD);

	// Set startup Freq
	SetFreq(R.Freq);
}


// Nodig voor Xtal calibratie!
// Dont read from Si570 device, calc them from the (saved) frequency.
uint8_t
GetRegFromSi570(void)
{
	uint8_t		cnt;
	uint32_t	RR;					// Division remainder
	sint32_t	Freq;

	Freq.dw = R.Freq;

	Si570_Data.HS_DIV = 0;			// HS_DIV = 0 (real divider = 4)
	Si570_Data.N1 = 15>>2;			// N1 = 15 (real divider = 16)

	// F = RFREQ * Xtal / (N1 * HS_DIV)
	// RFREQ = F * (N1 * HS_DIV) / Xtal
	// (N1 * HS_DIV) = 4 * 16 = 64
	// RFREQ = F * 64 / Xtal
	// [12.28] = [19.21] * 64 / [8.24]
	// [12.28] = [16.24] * 2^3 * 2^6 * 2^28 / [8.24]
	// RFREQ = 28.2 * 64 / 100 = 18,048

	RR = 0;							// Clear Remainder_32
	cnt = 40+1+28+3+6;				// Init Loop_Counter
									// (28 = 12.28 bits, 3 = * 8, 2 = *4)
	asm (
	"clc                 \n\t"		// Partial_result = carry = 0

"L_A_%=:                 \n\t"		// Repeat
	"rol %0              \n\t"		//   Put last Partial_result in Quotient_40
	"rol %1              \n\t"		//   and shift left Dividend_40 ...
	"rol %2              \n\t"
	"rol %3              \n\t"
	"rol %4              \n\t"

	"rol %A6             \n\t"		//                      ... into Remainder_32
	"rol %B6             \n\t"
	"rol %C6             \n\t"
	"rol %D6             \n\t"

	"sub %A6,%A7         \n\t"		//   Remainder =  Remainder - Divisor_32
	"sbc %B6,%B7         \n\t"
	"sbc %C6,%C7         \n\t"
	"sbc %D6,%D7         \n\t"

	"brcc L_B_%=         \n\t"		//   If result negative
									//   Then
	"add %A6,%A7         \n\t"		//     Restore Remainder
	"adc %B6,%B7         \n\t"
	"adc %C6,%C7         \n\t"
	"adc %D6,%D7         \n\t"

	"clc                 \n\t"		//     Partial_result = 0
	"rjmp L_C_%=         \n\t"

"L_B_%=:                 \n\t"		//   Else
	"sec                 \n\t"		//     Partial_result = 1

"L_C_%=:                 \n\t"		//   End If
	"dec %5              \n\t"		// Until(--cnt == 0)
	"brne L_A_%=         \n\t"

	"adc %0,__zero_reg__\n\t"		// Round by the last bit of Count
	"adc %1,__zero_reg__\n\t"
	"adc %2,__zero_reg__\n\t"
	"adc %3,__zero_reg__\n\t"
	"adc %4,__zero_reg__\n\t"

"L_X_%=:                 \n\t"

	// Output operand list
	//--------------------
#if 0 //DEBUG
	: "=r" (Si570_Data.RFREQ.w0.b0) // %0 -> Dividend_40 LSB
	, "=r" (Si570_Data.RFREQ.w0.b1) // %1        "
	, "=r" (Si570_Data.RFREQ.w1.b0) // %2        "
	, "=r" (Si570_Data.RFREQ.w1.b1) // %3        "     
	, "=r" (Si570_Data.RFREQ_b4)    // %4        "     MSB
#else
	: "=r" (Si570_Data.RFREQ.w1.b1) // %0 -> Dividend_40
	, "=r" (Si570_Data.RFREQ.w1.b0) // %1        "
	, "=r" (Si570_Data.RFREQ.w0.b1) // %2        "
	, "=r" (Si570_Data.RFREQ.w0.b0) // %3        "     LSB
	, "=r" (Si570_Data.RFREQ_b4)    // %4        "     MSB
#endif

	// Input operand list
	//-------------------
	: "r" (cnt)                     // %5 -> Loop_Counter
	, "r" (RR)                      // %6 -> Remainder_32
	, "r" (R.FreqXtal)              // %7 -> Divisor_32
	, "0" (Freq.w0.b0)              //  0 -> Dividend_40 LSB
	, "1" (Freq.w0.b1)              //  1 -> Dividend_40
	, "2" (Freq.w1.b0)              //  2 -> Dividend_40
	, "3" (Freq.w1.b1)              //  3 -> Dividend_40
	, "4" (0)                       //  4 -> Dividend_40 MSB
	);

	Si570_Data.RFREQ_b4 |= 15<<6;	// N1 = 15 (real divider = 16)

	return sizeof(Si570_t);
}

#endif
