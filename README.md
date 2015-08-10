# ECG_Discovery

ECG display with heart rate on the STM32F746G-DISCO board

# Description

This project used the STM32F746G-DISCO board to acquire an ECG signal from the OLIMEX SHIELD-EKG-EMG (https://www.olimex.com/Products/Duino/Shields/SHIELD-EKG-EMG/open-source-hardware) and displays it on the display.
It also detects the peaks in the signal and computes the heart rate in beats per minute (BPM)

# Pin mapping:

- AN0: ECG signal in

# Versions

- v1.0: First working version
- v1.1:
	- Removed debug data from the main display
	- Added more display buffers for a smoother display
	- Displays now a continuous waveform (not just the data points)

# Known issues

- Timer 5 trigger to ADC doesn't work (Timing done manually using Timer 2)

# Future work

- Full signal analysis
- More stats
- Touch screen