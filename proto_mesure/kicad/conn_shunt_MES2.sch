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
LIBS:shunt
LIBS:max4377
LIBS:arduino_shieldsNCL
LIBS:carte_mesure-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 4
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Shunt Rsh2
U 1 1 5798E9CC
P 5750 2200
F 0 "Rsh2" H 5750 2350 60  0000 C CNN
F 1 "Shunt" H 5750 2050 60  0000 C CNN
F 2 "shunt:shunt" H 5750 2200 60  0001 C CNN
F 3 "" H 5750 2200 60  0000 C CNN
	1    5750 2200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5350 2100 4950 2100
Wire Wire Line
	4950 2300 5350 2300
Wire Wire Line
	5350 1650 5350 2100
Wire Wire Line
	5350 2300 5350 2750
Wire Wire Line
	5350 4650 4950 4650
Wire Wire Line
	4950 4850 5350 4850
Wire Wire Line
	5350 4200 5350 4650
Wire Wire Line
	5350 4850 5350 5250
$Comp
L MAX4377 U2
U 2 1 5798E9DB
P 7100 2600
F 0 "U2" H 7250 3300 60  0000 C CNN
F 1 "MAX4377" H 6550 3350 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 7100 2600 60  0001 C CNN
F 3 "" H 7100 2600 60  0000 C CNN
	2    7100 2600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR016
U 1 1 5798E9E9
P 6950 2850
F 0 "#PWR016" H 6950 2600 50  0001 C CNN
F 1 "GND" H 6950 2700 50  0000 C CNN
F 2 "" H 6950 2850 50  0000 C CNN
F 3 "" H 6950 2850 50  0000 C CNN
	1    6950 2850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 5798E9EF
P 6900 5450
F 0 "#PWR017" H 6900 5200 50  0001 C CNN
F 1 "GND" H 6900 5300 50  0000 C CNN
F 2 "" H 6900 5450 50  0000 C CNN
F 3 "" H 6900 5450 50  0000 C CNN
	1    6900 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 3900 6900 4200
Wire Wire Line
	6100 4200 6100 4700
Wire Wire Line
	6100 4700 6300 4700
Wire Wire Line
	5350 5250 6100 5250
$Comp
L Shunt Rsh3
U 1 1 5798EA05
P 5750 4750
F 0 "Rsh3" H 5750 4900 60  0000 C CNN
F 1 "Shunt" H 5750 4600 60  0000 C CNN
F 2 "shunt:shunt" H 5750 4750 60  0001 C CNN
F 3 "" H 5750 4750 60  0000 C CNN
	1    5750 4750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6100 4200 5350 4200
Wire Wire Line
	5750 4250 5750 4200
Connection ~ 5750 4200
Connection ~ 5750 5250
Wire Wire Line
	6300 4800 6100 4800
Wire Wire Line
	6100 4800 6100 5250
Wire Wire Line
	6900 5300 6900 5450
Wire Wire Line
	5350 1650 6100 1650
Wire Wire Line
	6100 1650 6100 2150
Wire Wire Line
	6100 2150 6350 2150
Wire Wire Line
	6350 2250 6100 2250
Wire Wire Line
	6100 2250 6100 2750
Wire Wire Line
	6100 2750 5350 2750
Wire Wire Line
	5750 2700 5750 2750
Connection ~ 5750 2750
Wire Wire Line
	5750 1700 5750 1650
Connection ~ 5750 1650
Wire Wire Line
	6950 2850 6950 2750
Wire Wire Line
	6950 1500 6950 1650
Wire Wire Line
	7500 4700 8350 4700
Wire Wire Line
	7550 2150 8200 2150
$Comp
L C C4
U 1 1 5798EA23
P 7200 1600
F 0 "C4" H 7225 1700 50  0000 L CNN
F 1 "100nF" H 7225 1500 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Rect_L7_W2.5_P5" H 7238 1450 50  0001 C CNN
F 3 "" H 7200 1600 50  0000 C CNN
	1    7200 1600
	0    1    1    0   
$EndComp
$Comp
L GND #PWR018
U 1 1 5798EA2A
P 7500 1600
F 0 "#PWR018" H 7500 1350 50  0001 C CNN
F 1 "GND" H 7500 1450 50  0000 C CNN
F 2 "" H 7500 1600 50  0000 C CNN
F 3 "" H 7500 1600 50  0000 C CNN
	1    7500 1600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6950 1600 7050 1600
Connection ~ 6950 1600
Wire Wire Line
	7350 1600 7500 1600
$Comp
L CONN_01X05 P5
U 1 1 5798EA33
P 1800 3250
F 0 "P5" H 1800 3550 50  0000 C CNN
F 1 "CONN_01X05" V 1900 3250 50  0000 C CNN
F 2 "Connect:bornier5" H 1800 3250 50  0001 C CNN
F 3 "" H 1800 3250 50  0000 C CNN
	1    1800 3250
	-1   0    0    1   
$EndComp
Wire Wire Line
	2000 3050 2750 3050
Wire Wire Line
	2000 3450 2750 3450
Wire Wire Line
	2000 3350 2750 3350
Wire Wire Line
	2000 3150 2750 3150
Wire Wire Line
	2000 3250 2800 3250
$Comp
L GND #PWR019
U 1 1 5798EA3F
P 2800 3250
F 0 "#PWR019" H 2800 3000 50  0001 C CNN
F 1 "GND" H 2800 3100 50  0000 C CNN
F 2 "" H 2800 3250 50  0000 C CNN
F 3 "" H 2800 3250 50  0000 C CNN
	1    2800 3250
	0    -1   -1   0   
$EndComp
Text Label 2150 3150 0    60   ~ 0
MP4+
Text Label 2150 3450 0    60   ~ 0
MP3+
Text Label 2150 3050 0    60   ~ 0
MP4-
Text Label 2150 3350 0    60   ~ 0
MP3-
Text Label 5000 4850 0    60   ~ 0
MP4+
Text Label 5000 4650 0    60   ~ 0
MP4-
Text Label 5050 2300 0    60   ~ 0
MP3+
Text Label 5050 2100 0    60   ~ 0
MP3-
$Comp
L MAX4377 U2
U 1 1 5799CE49
P 7050 5150
F 0 "U2" H 7200 5850 60  0000 C CNN
F 1 "MAX4377" H 6550 5900 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 7050 5150 60  0001 C CNN
F 3 "" H 7050 5150 60  0000 C CNN
	1    7050 5150
	1    0    0    -1  
$EndComp
Text HLabel 8200 2150 2    60   Input ~ 0
OUT_B1
Text HLabel 8350 4700 2    60   Input ~ 0
OUT_B2
$Comp
L +3V3 #PWR020
U 1 1 579B700F
P 6950 1500
F 0 "#PWR020" H 6950 1350 50  0001 C CNN
F 1 "+3V3" H 6950 1640 50  0000 C CNN
F 2 "" H 6950 1500 50  0000 C CNN
F 3 "" H 6950 1500 50  0000 C CNN
	1    6950 1500
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR021
U 1 1 579B7029
P 6900 3900
F 0 "#PWR021" H 6900 3750 50  0001 C CNN
F 1 "+3V3" H 6900 4040 50  0000 C CNN
F 2 "" H 6900 3900 50  0000 C CNN
F 3 "" H 6900 3900 50  0000 C CNN
	1    6900 3900
	1    0    0    -1  
$EndComp
$EndSCHEMATC
