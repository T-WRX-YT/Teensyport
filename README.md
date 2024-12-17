# Teensyport Folders

## TEENSYPORT

Holds the final code for querying and reading Subaru CANBUS and SSM addresses to display on an ili9341 display.  Runs on a Teensy 4.0 or similar device.



## Subaru CANBUS Sniffing

Example code for reading CANBUS with CanHacker, and for reading directly in Arduino code.



## AEM Oil Temp and Pressure

Reads data from an AEM oil temperature sensor and pressure sensor, converts voltages to decimal and then sends the values via SW Serial to be picked up by far side.  I have no idea how to send 2 integers via serial and them read them properly on the far side, so it uses some weird string and then is interpreted and split up by the receiver.

I have to use this because the AEM sensors run at 5v, and the Teeny runs at 3v, so I do the 5v work on an Arduino and serial it over to the Teensy via a 5v to 3v logic converter.