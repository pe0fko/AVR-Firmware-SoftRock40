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
//** Description..: Calculations for 48bits multiply and divide
//**
//** History......: 2018/04/20: PE0FKO.
//**
//**************************************************************************
 

#ifndef MUL_DIV_H_
#define MUL_DIV_H_

inline uint64_t
umul_48_32_16(uint32_t A, uint16_t B)
{
	sint64_t	M;
	M.ll = 0;
	
	// D2:D1 = Ma * Mb, [48] = [32] * [32]
	asm volatile (
	"		clr		r26				\n"
	"		clr		r27				\n"
	"		rjmp	L3%=			\n"

	"L1%=:	add		%A[D1],%A[Ma]	\n"
	"		adc		%B[D1],%B[Ma]	\n"
	"		adc		%C[D1],%C[Ma]	\n"
	"		adc		%D[D1],%D[Ma]	\n"
	"		adc		%A[D2],r26		\n"
	"		adc		%B[D2],r27		\n"

	"L2%=:	lsl		%A[Ma]			\n"
	"		rol		%B[Ma]			\n"
	"		rol		%C[Ma]			\n"
	"		rol		%D[Ma]			\n"
	"		rol		r26				\n"
	"		rol		r27				\n"

	"L3%=:	lsr		%B[Mb]			\n"
	"		ror		%A[Mb]			\n"
	"		brcs	L1%=			\n"
	"		sbci	%B[Mb],0x00		\n"
	"		brne	L2%=			\n"

	//	"		sbiw	%B[M],0x00		\n"
	//	"		brne	L2%=			\n"

	// Output operand list
	//--------------------
	: [D1]	"=r" (M.l0.dw)			// %0	-> 32bits, 48bits, 6bytes
	, [D2]	"=r" (M.l1.dw)			// %1	-> 16bits

	// Input operand list
	//-------------------
	: [Ma]	"r" (A)					// %2	-> 32bits
	, [Mb]	"r" (B)					// %4	-> 16bits
	,		"0" (M.l0.dw)			// %0	-> 32bits, 48bits, 6bytes
	,		"1" (M.l1.dw)			// %1	-> 16bits

	: "r26", "r27"
	);

	return M.ll;
}

//	Quotient(48bits)  = Dividend(48bits) / Divisor(32bits)
inline uint64_t
udiv_48_48_32_R(uint64_t A, uint32_t B, uint8_t R)
{
	uint32_t	remainder;						// Division remainder
	sint64_t	X;

	X.ll = A;
	remainder = 0;

	R += 48;

	asm volatile (
	"L1%=:	lsl		%A[D0]		\n"
	"		rol		%B[D0]		\n"
	"		rol		%A[D1]		\n"
	"		rol		%B[D1]		\n"
	"		rol		%A[D2]		\n"
	"		rol		%B[D2]		\n"
	"		rol		%A[R]		\n"
	"		rol		%B[R]		\n"
	"		rol		%C[R]		\n"
	"		rol		%D[R]		\n"
	"		brcs	L2%=		\n"
	"		cp		%A[R],%A[X]	\n"
	"		cpc		%B[R],%B[X]	\n"
	"		cpc		%C[R],%C[X]	\n"
	"		cpc		%D[R],%D[X]	\n"
	"		brcs	L3%=		\n"
	"L2%=:	sub		%A[R],%A[X]	\n"
	"		sbc		%B[R],%B[X]	\n"
	"		sbc		%C[R],%C[X]	\n"
	"		sbc		%D[R],%D[X]	\n"
	"		inc		%A[D0]		\n"
	"L3%=:	dec		%[cnt]		\n"
	"		brne	L1%=		\n"


	// Output operand list
	//--------------------
	: [D0]	"=r" (X.l0.w0.w)		// %0 -> FBDIV_7_0  , word,	Dividend_48,	b0...b1	(uint16_t)
	, [D1]	"=r" (X.l0.w1.w)		// %1 -> FBDIV_23_16, word,	Dividend_48,	b2...b3	(uint16_t)
	, [D2]	"=r" (X.l1.w0.w)		// %2 -> FBDIV_39_32, byte,	Dividend_48,	b4	(uint8_t)
	, [R]	"=r" (remainder)			// %3 -> Remainder_32

	// Input operand list
	//-------------------
	: [X]	"r" (B)						// %4 -> Xtal freq, double, Divisor, b0...b3 (uint32_t)
	, [cnt] "r" (R)						// %5 -> Loop_Counter
	,		"0" (X.l0.w0.w)			// %0 -> Dividend_48	b0...b1	(uint32_t)
	,		"1" (X.l0.w1.w)			// %1 -> Dividend_48	b2...b3	(uint32_t)
	,		"2" (X.l1.w0.w)			// %2 -> Dividend_48	b4...b5	(uint16_t)
	,		"3" (remainder)				// %3 -> Remainder_32
	);

	return X.ll;
}



#endif /* MUL_DIV_H_ */