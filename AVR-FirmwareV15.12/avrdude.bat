@rem Programm ALL 
@rem PE0FKO 2009/02/01

@rem Fuse 1 = Disabled
@rem Fuse 0 = Enabled

@rem Fuse bit information:
@rem Fuse high byte:
@rem 0xdd = 1 1 0 1   1 1 0 1     RSTDISBL disabled (SPI programming can be done)
@rem 0x5d = 0 1 0 1   1 1 0 1     RSTDISBL enabled (PB5 can be used as I/O pin)
@rem        ^ ^ ^ ^   ^ \-+-/ 
@rem        | | | |   |   +------ BODLEVEL 2..0 (brownout trigger level -> 2.7V)
@rem        | | | |   +---------- EESAVE (preserve EEPROM on Chip Erase -> not preserved)
@rem        | | | +-------------- WDTON (watchdog timer always on -> disable)
@rem        | | +---------------- SPIEN (enable serial programming -> enabled)
@rem        | +------------------ DWEN (debug wire enable)
@rem        +-------------------- RSTDISBL (disable external reset -> disabled)
@rem
@rem Fuse low byte:
@rem 0xe1 = 1 1 1 0   0 0 0 1
@rem        ^ ^ \+/   \--+--/
@rem        | |  |       +------- CKSEL 3..0 (clock selection -> HF PLL)
@rem        | |  +--------------- SUT 1..0 (BOD enabled, fast rising power)
@rem        | +------------------ CKOUT (clock output on CKOUT pin -> disabled)
@rem        +-------------------- CKDIV8 (divide clock by 8 -> don't divide)

avrdude.exe -p t45 -c avr910 -P com4 -e -U flash:w:output/out.hex:i -U lfuse:w:0xE1:m -U hfuse:w:0xDD:m -V

pause
