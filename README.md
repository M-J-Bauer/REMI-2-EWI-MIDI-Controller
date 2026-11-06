# REMI 2 - EWI MIDI Controller
Electronic Wind Instrument (EWI) MIDI Controller using touch-pads for the "keys",
based on PIC18F45K22 (or PIC18F25K22) micro-controller, 
designed as a DIY project for electronics hobbyists with musical interests.

<img width="746" height="353" alt="REMI mk2 handset pic_w" src="https://github.com/user-attachments/assets/0bbea58a-74a4-46ec-9b1d-5ffb8d3e1e48" />

REMI 2 is a simple MIDI controller with standard MIDI OUT connector (5-pin DIN socket). 
It does not have a built-in sound synthesizer.
It can be used to play any MIDI synth or sound module with MIDI input. 
A low-cost ($10) MIDI-USB adapter allows the REMI to be used with a "virtual synthesizer"
(software app) running on a computer or mobile device.

Latest firmware supports a choice of recorder fingering schemes including close emulation of 
Baroque (German and English) fingering. The "standard" EWI fingering is designed to be easy to learn, 
for beginners with no previous experience of wind instruments.

Detailed construction notes and user manual can be found on the project web-page, here:
https://www.mjbauer.biz/Build_the_REMI_mk2.htm

_You are welcome to post comments or ask questions in the Discussions section._

__Firmware Installation__

A PIC programming tool, e.g. Microchip PICkit-3, is required to install the REMI application firmware.
Low-cost PICkit-3 clones are available from online suppliers via AliExpress, eBay, etc.

<img width="364" height="182" alt="PICkit3-pic" src="https://github.com/user-attachments/assets/b39bf920-88e0-4e0e-998b-f5d08be27e60" />

Ready-built firmware files are available here in the folder "hex". Download the hex file appropriate 
for your REMI hardware configuration. 
Download MPLAB.X IDE and IPE (PIC programmer app) from Microchip's website. (It's free.) Install the 'IPE' component only.
The full IDE is not needed. Follow the IPE embedded instructions to use the PIC programmer app. Be sure to specify 'PICkit-3'
as your connected programming tool.
