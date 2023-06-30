# Summary
CBM20 Geiger counter using PIC16F88 and 16x2 LCD


# Features
- Displays current CPM and dose (uS/hr) with rolling graph of last-seven time blocks
- Screens for session maximum and average, and all-time maximum CPM/dose
- Elapsed time display
- Rechargeable 18650 with charge level indicator
- Integration time of 60s or 10s
- Digital-loop boost converter with fault detection
- Audible and visual indicator


# Building
MPLABX v6.05 with XC8 compiler v2.36 optimisation 02s
3060 of 4096 bytes code space.

# Schematic

Drawing TBA, general overview: 18650 into eBay power bank module, whose 5V output goes into the system. Power switch is DPDT so that 5V to CPU and cell voltage (for measurement) gets cut. Boost converter is NFET driven by CPU, with 330uF polymer and ceramics at input, a few HV MLCCs at output. Feedback into CPU via big resistor divider. Two big resistors into CBM20 tube. The other side of the tube is connected to a sense circuit similar to the ones in the references.
LCD/Buzzer/LED/BoostFET all powered directly from power bank module's 5V output. The CPU is powered via 33R resistor and 1000uF electrolytic so that the dips due to the power bank module don't interfere much with the ADC performance.


# Pinning
Name | Description
-----|-----------
A0   | HV Feedback
A1   | Vbatt
A2   | Backlight
A3   | -	
A4   | -
A5   | -
A6	 | LCD-Reg
A7	 | LCD-Enable
B0	 | Boost (PWM)
B1	 | LCD-Data
B2	 | LCD-Data
B3	 | [PGM] LCD-Data
B4   | LCD-Data
B5   | Tube
B6   | PGC / Buzzer/LED
B7   | PGD / Button


# Detailed Operation

Upon power up the firmware version is shown and a lamp/buzzer test occurs. If the screen button is unpressed at power-up then the default integration time of 60 seconds is used. This means that each time block and CPM/does is calculated every 60s. If the button is held during power-up the startup screen will show Intg=10s meaning that each time block is only 10s. This gives a faster moving graph and more frequently updating readings, however it is less accurate.
The main screen at the top shows the CPM and dose in uS/hr, these values are updated every integration time. The lower left shows the total run time in H:MM:SS, and the far right shows a battery charge indicator with seven states. To the left of the battery indicator is a graph area with seven blocks which record the counts in each integration time. This graph fills from the top-down and always shows the counts for the last seven times. 
On the main screen, holding the button will reset the run time to zero, without affecting the current counts or the graph. 
A shorter press of the button will advance to the next screen which shows the maximum recording for this power-on session. Holding the button on this screen resets the session maximum to zero with affecting any other parameters.
A shorter press of the button will advance to the next screen which shows the average recording for this power-on session. Holding the button on this screen resets the session average to zero with affecting any other parameters.
A shorter press of the button will advance to the next screen which shows the maximum recording ever attained. This value is recorded in EEPROM and persists through power cycling. Holding the button on this screen resets the all-time maximum to zero with affecting any other parameters.
A shorter press of the button will return to the main screen again.
If the code restarts due to a watchdog timer reset, a 'w' is shown between the run time and graph to warn the user that the readings may not be complete.
The firmware detects over/under voltage regulation of the high voltage, as well as if the firmware attempts to drive the FET gate with too high a duty cycle for a continuous period. In these cases all operation will cease and the display shows "HV OVER ERROR", "HV UNDER ERROR" or "HV DUTY ERROR" respectively. 


# Known Issues / ToDo
- Produce schematic
