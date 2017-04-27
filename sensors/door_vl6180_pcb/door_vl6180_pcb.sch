EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:esp8266_sf_thing
LIBS:vl6180_tof_sf
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "VL6180 / ESP8266 Door and Deadbolt Sensor "
Date "2017-04-26"
Rev "1a"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ATTINY85-P IC1
U 1 1 59011136
P 5700 5150
F 0 "IC1" H 4550 5550 50  0000 C CNN
F 1 "ATTINY85-P" H 6700 4750 50  0000 C CNN
F 2 "DIP8" H 6700 5150 50  0000 C CIN
F 3 "" H 5700 5150 50  0000 C CNN
	1    5700 5150
	1    0    0    -1  
$EndComp
$Comp
L SPST SW1
U 1 1 5901117E
P 8100 2900
F 0 "SW1" H 8100 3000 50  0000 C CNN
F 1 "SPST" H 8100 2800 50  0000 C CNN
F 2 "" H 8100 2900 50  0000 C CNN
F 3 "" H 8100 2900 50  0000 C CNN
	1    8100 2900
	1    0    0    -1  
$EndComp
$Comp
L ESP8266_SF_THING U2
U 1 1 590116C1
P 6400 2950
F 0 "U2" H 6000 3550 60  0000 C CNN
F 1 "ESP8266_SF_THING" H 6400 2300 60  0000 C BNN
F 2 "" H 6400 2950 60  0001 C CNN
F 3 "" H 6400 2950 60  0001 C CNN
	1    6400 2950
	1    0    0    -1  
$EndComp
$Comp
L VL6180_TOF_SF U1
U 1 1 590119BD
P 3800 2750
F 0 "U1" H 3750 3150 60  0000 C CNN
F 1 "VL6180_TOF_SF" H 4300 2750 60  0000 C CNN
F 2 "" H 3800 2750 60  0001 C CNN
F 3 "" H 3800 2750 60  0001 C CNN
	1    3800 2750
	-1   0    0    -1  
$EndComp
Text GLabel 5700 2500 0    60   BiDi ~ 0
GND
Text GLabel 7100 2500 2    60   BiDi ~ 0
GND
Text GLabel 4550 2500 2    60   BiDi ~ 0
GND
Text GLabel 4300 2600 2    60   Input ~ 0
3V3
Wire Wire Line
	4200 2500 4550 2500
Wire Wire Line
	4200 2600 4300 2600
Wire Wire Line
	4200 2700 5700 2700
Wire Wire Line
	4200 2800 5700 2800
Text GLabel 7050 4900 2    60   Input ~ 0
3V3
Text GLabel 7050 5400 2    60   BiDi ~ 0
GND
Text GLabel 5400 2600 0    60   Output ~ 0
3V3
Wire Wire Line
	5400 2600 5700 2600
Text GLabel 8600 2900 2    60   BiDi ~ 0
GND
Wire Wire Line
	7100 2900 7400 2900
Wire Wire Line
	7400 2900 7600 2900
$Comp
L R R1
U 1 1 5901242E
P 3550 5100
F 0 "R1" V 3630 5100 50  0000 C CNN
F 1 "R1M" V 3550 5100 50  0000 C CNN
F 2 "" V 3480 5100 50  0000 C CNN
F 3 "" H 3550 5100 50  0000 C CNN
	1    3550 5100
	0    1    1    0   
$EndComp
$Comp
L R R2
U 1 1 59012475
P 7750 3200
F 0 "R2" V 7830 3200 50  0000 C CNN
F 1 "R5.6K" V 7750 3200 40  0000 C CNN
F 2 "" V 7680 3200 50  0000 C CNN
F 3 "" H 7750 3200 50  0000 C CNN
	1    7750 3200
	0    1    1    0   
$EndComp
Text GLabel 7900 3200 2    60   Input ~ 0
3V3
Wire Wire Line
	7400 2900 7400 3200
Wire Wire Line
	7400 3200 7400 4500
Wire Wire Line
	7400 3200 7600 3200
Connection ~ 7400 2900
Wire Wire Line
	7100 3200 7200 3200
Wire Wire Line
	7200 3200 7200 4600
Wire Wire Line
	7200 4600 4200 4600
Wire Wire Line
	4200 4600 4200 5100
Wire Wire Line
	3700 5100 4200 5100
Wire Wire Line
	4200 5100 4350 5100
Text GLabel 3400 5100 0    60   BiDi ~ 0
GND
Connection ~ 4200 5100
Wire Wire Line
	4350 5000 4100 5000
Wire Wire Line
	4100 5000 4100 3400
Wire Wire Line
	4100 3400 4400 3400
Wire Wire Line
	4400 3400 4400 2900
Wire Wire Line
	4400 2900 4200 2900
Wire Wire Line
	7400 4500 4300 4500
Wire Wire Line
	4300 4500 4300 4900
Wire Wire Line
	4300 4900 4350 4900
Connection ~ 7400 3200
Wire Wire Line
	4350 5200 4000 5200
Wire Wire Line
	4000 5200 4000 3300
Wire Wire Line
	4000 3300 5300 3300
Wire Wire Line
	5300 3300 5300 2900
Wire Wire Line
	5300 2900 5700 2900
$EndSCHEMATC
