ECHO OFF

PATH=C:\Users\fred.krom\AppData\Local\Arduino15\packages\arduino\tools\avrdude\6.3.0-arduino9/bin/;%PATH%
SET CONF="C:\Users\fred.krom\AppData\Local\Arduino15\packages\arduino\tools\avrdude\6.3.0-arduino9/etc/avrdude.conf"

avrdude.exe -C%CONF% -v -patmega328p -carduino -PCOM3 -b115200 -D -Uflash:w:%1%:i 
