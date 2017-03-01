openLRSngBeacon-GPS
===============

Lost plane beacon using openLRSng RX
Includes UBLOX GPS support for transmitting coordinates via CW/Morse Code

based mostly on:
https://github.com/openLRSng/openLRSngBeacon

GPS MORSE BEACON
USAGE NOTES:
- GPS coords are sent separately
- lat is sent 1st, indicated by high-pitched tone sequence ("bee-boop")
- lon is sent 2nd, indicated by low-pitched tone sequence ("boo-beep")
- fix tone (if enabled) is sent next, followed by morse code sequence
- transmissions can be manually initiated by tx'ing on beacon frequency within range of beacon
- GREEN LED will flash @ 1Hz (along w/ red LED) if GPS packets are being received & processed

See the beginning of the code for config options.

Currently supports receiver hardware types 3 & 5