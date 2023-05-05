# REMI 2 - EWI MIDI Controller
Electronic Wind Instrument (EWI) - MIDI Controller using touch-pads for "keys",
based on PIC18F45K22 (or PIC18F25K22) micro-controller, designed as a DIY project for electronics hobbyists with musical interests.

REMI 2 is a simple MIDI controller with standard MIDI OUT connection (5-pin DIN socket). It does not have a built-in sound synthesizer.
A low-cost ($10) MIDI-USB adapter allows the REMI to be used with a "virtual synthesizer" (software app) running on a computer or mobile device.

Latest firmware supports a choice of recorder-like fingering schemes including emulation of Baroque (German and English) fingering. 
The "standard" REMI fingering is designed to be easy to learn, for beginners with no previous experience of wind instruments.

*** New firmware update (v1.4.50) supports PITCH BEND using a motion sensor MMA8451/2 (3-axis accelerometer). Pitch bend is activated by pressing on the Modulation Pad (force sensor), which acts as a touch-switch. MIDI modulation (CC1) messages can be disabled by a config option. A future firmware version will support other options to activate pitch bend, e.g. an extra touch-pad (at LH4 if not otherwise used) or a push-button switch operated by the RH thumb.

A full description of the project can be found in the doc folder. See also: http://www.mjbauer.biz/REMI_Intro.htm

_You are welcome to post comments or ask questions in the Discussions section._

REMI 2 firmware is built using Microchip's MPLAB.X IDE with XC8 compiler (free downloads from Microchip's website).
If you intend to modify or extend the firmware, you will need to install the full MPLAB.X IDE application. 
Otherwise, you just need to install the PIC programmer application (IPE) on your computer.
