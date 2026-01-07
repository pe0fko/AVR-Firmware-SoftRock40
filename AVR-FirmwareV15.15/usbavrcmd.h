//************************************************************************
//**
//** Project......: Firmware USB AVR Si570 controler.
//**
//** Platform.....: ATtiny[48]5 @ 16.5 MHz
//**
//** Licence......: This software is freely available for non-commercial 
//**                use - i.e. for research and experimentation only!
//**
//** Programmer...: F.W. Krom, PE0FKO
//** 
//** Description..: USB commando codes
//**
//** History......: V15.12:	First version
//**
//**************************************************************************
//

//#define CMD_ECHO_WORD			0x00	// V1.4: Function changed to get version.
#define	CMD_GET_VERSION			0x00	// V15.7: 
#define	CMD_SET_DDR				0x01	// V1.4: 
#define	CMD_GET_PIN				0x02	// V1.4: 
#define CMD_GET_PORT			0x03	// V1.4: 
#define	CMD_SET_PORT			0x04	// V1.4: 
//								0x05	// V1.4
//								0x06	// V1.4
//								0x07	// V1.4
//								0x08	// V1.4
//								0x09	// V1.4
//								0x0a	// V1.4
//								0x0b	// V1.4
//								0x0c	// V1.4
//								0x0d	// Free
//								0x0e	// Free
#define	CMD_REBOOT				0x0F	// V1.4: 
//								0x10	// V1.4: 
#define	CMD_READ_EEPROM			0x11	// V1.4: Not implemented
//								0x12	// Free
//								0x13	// Free
//								0x14	// Free
#define	CMD_SET_IO				0x15
#define	CMD_GET_IO				0x16
#define	CMD_SET_FILTER			0x17
#define	CMD_SET_RX_BAND_FILTER	0x18	// V15.12
#define	CMD_GET_RX_BAND_FILTER	0x19	// V15.12
#define	CMD_SET_TX_BAND_FILTER	0x1a	// MOBO Only
#define	CMD_GET_TX_BAND_FILTER	0x1b	// MOBO Only
//								0x1c	// Free
//								0x1d	// Free
//								0x1e	// Free
//								0x1f	// Free
#define	CMD_SET_SI570			0x20	// Write byte to Si570 register
//								0x21	// V1.4:
//								0x22	// V1.4:
//								0x23	// V1.4:

#define	CMD_SET_FREQ_REG		0x30
#define	CMD_SET_LO_SM			0x31
#define	CMD_SET_FREQ			0x32
#define	CMD_SET_XTAL			0x33
#define	CMD_SET_STARTUP			0x34
#define	CMD_SET_PPM				0x35
//								0x36	// V2.0: 
//								0x37	// V2.0: 
//								0x38	// V2.0: 
#define	CMD_GET_LO_SM			0x39
#define	CMD_GET_FREQ			0x3a
#define	CMD_GET_PPM				0x3b
#define	CMD_GET_STARTUP			0x3c
#define	CMD_GET_XTAL			0x3d
#define	CMD_GET_REGS			0x3e	// V1.4: Not implemented
#define	CMD_GET_SI570			0x3f
#define	CMD_GET_I2C_ERR			0x40
#define	CMD_SET_I2C_ADDR		0x41
#define	CMD_GET_CPU_TEMP		0x42	// V15.12: 
#define	CMD_GET_USB_ID			0x43	// V15.12: Get/Set the USB Serialnumber ID char (last char of string)
#define	CMD_SET_SI570_GRADE		0x44	// V15.14


#define	CMD_SET_USRP1			0x50
#define	CMD_GET_CW_KEY			0x51
//								0x52	// V2.0: 
//								0x53	// V2.0: 
//								0x54	// V2.0: 

// Mobo command's
#define	CMD_GET_FW_FEATURE		0x60	// Firmware Feature select
#define	CMD_GET_ADC_INPUTS		0x61	// Read analog inputs (Temp, PA current, P_out, P_ref, Vdd)
#define	CMD_RM_PA_HIGH_TEMP		0x64	// Read/Modify the PA High Temperature limit
#define	CMD_RM_PA_BIAS			0x65	// Read/Modify PA bias setting related values, 5 items
#define	CMD_RM_PA_SWR			0x66	// Read/Modify SWR measurement and SWR alarm related values 4 items
#define	CMD_SET_BYTE_GPIO		0x6e	// Write a Byte to (PCF8584) GPIO Extender
#define	CMD_GET_BYTE_GPIO		0x6f	// Read a Byte from (PCF8584) GPIO Extender

//								0xEE	// Used in old V2.0
//								0xEF	// Used in old V2.0
//								0xFF	// Used in old V2.0
