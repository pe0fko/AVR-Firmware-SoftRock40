//************************************************************************
//**
//** Project......: Firmware USB AVR Si570 controler.
//**
//** Platform.....: ATtiny45
//**
//** Licence......: This software is freely available for non-commercial 
//**
//** Programmer...: F.W. Krom, PE0FKO
//** 
//** Description..: Read the (internal ATTiny45 chip) temperature.
//**
//** History......: Check the main.c file
//**
//**************************************************************************

#include "main.h"

#if INCLUDE_TEMP

// Check: Datasheet AVR122

// temp count
// -40	230		datasheet
//   0	273 	AVR122
// +25	300 	datasheet
// +85	370 	datasheet

// return value 12.4bits. Resolution 0.0625oC

uint16_t
GetTemperature()
{
	uint16_t temp;

	// Ref 1.1V, MUX=ADC4 temperature
	ADMUX = (1<<REFS1)|15;
	ADCSRA = (1<<ADEN)|(1<<ADSC)|(7<<ADPS0);

	while(ADCSRA & _BV(ADSC)) {}

//	temp = ((ADC - 270) * (6 * (1<<4))) / 7;
	temp = ADC;	// V15.14 No data conversion anymore!

	ADCSRA = (0<<ADEN);

	// Scale to degree centigrade
//	temp -= 273;
//	temp *= 6 * (1<<4);
//	temp /= 7;
//	temp = ((temp - 270) * (6 * (1<<4))) / 7;

	return temp;
}

#endif

