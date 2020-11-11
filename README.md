# Netscale_72_SW_Networkbridge

*** What is it about? ***
This is a gateway between ethernet and rs485 developed for the STM32H743 discovery board + a customized addon board with rs485 transceiver. 

*** How does it work? ***
The firmware is lean and uses a dynamic list to move frames between the ethernet and uart interface. To have a good performance for handlig the frames, the firmware is baremetall. You can use the modules for your own hardware environment.  
