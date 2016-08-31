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
Sheet 2 4
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
L Shunt Rsh??
U 1 1 5790CF45
P 5250 2350
AR Path="/578F869F/5790CF45" Ref="Rsh??"  Part="1" 
AR Path="/5790CF91/5790CF45" Ref="Rsh??"  Part="1" 
AR Path="/5798D1C3/5790CF45" Ref="Rsh??"  Part="1" 
AR Path="/5798DA86/5790CF45" Ref="Rsh??"  Part="1" 
F 0 "Rsh??" H 5250 2500 60  0000 C CNN
F 1 "Shunt" H 5250 2200 60  0000 C CNN
F 2 "" H 5250 2350 60  0000 C CNN
F 3 "" H 5250 2350 60  0000 C CNN
	1    5250 2350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4850 2250 4450 2250
Wire Wire Line
	4450 2450 4850 2450
Wire Wire Line
	4850 1800 4850 2250
Wire Wire Line
	4850 2450 4850 2900
Wire Wire Line
	4850 4800 4450 4800
Wire Wire Line
	4450 5000 4850 5000
Wire Wire Line
	4850 4350 4850 4800
Wire Wire Line
	4850 5000 4850 5400
$Comp
L MAX4377 U?
U 1 1 5790DCDB
P 6600 2750
AR Path="/578F869F/5790DCDB" Ref="U?"  Part="1" 
AR Path="/5790CF91/5790DCDB" Ref="U?"  Part="1" 
AR Path="/5798D1C3/5790DCDB" Ref="U?"  Part="1" 
AR Path="/5798DA86/5790DCDB" Ref="U?"  Part="1" 
F 0 "U?" H 6750 3450 60  0000 C CNN
F 1 "MAX4377" H 6050 3500 60  0000 C CNN
F 2 "" H 6600 2750 60  0000 C CNN
F 3 "" H 6600 2750 60  0000 C CNN
	1    6600 2750
	1    0    0    -1  
$EndComp
$Comp
L MAX4377 U?
U 2 1 5790DDE1
P 6550 5300
AR Path="/578F869F/5790DDE1" Ref="U?"  Part="2" 
AR Path="/5790CF91/5790DDE1" Ref="U?"  Part="2" 
AR Path="/5798D1C3/5790DDE1" Ref="U?"  Part="2" 
AR Path="/5798DA86/5790DDE1" Ref="U?"  Part="2" 
F 0 "U?" H 6700 6000 60  0000 C CNN
F 1 "MAX4377" H 6050 6050 60  0000 C CNN
F 2 "" H 6550 5300 60  0000 C CNN
F 3 "" H 6550 5300 60  0000 C CNN
	2    6550 5300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5790DEEE
P 6450 3000
AR Path="/578F869F/5790DEEE" Ref="#PWR?"  Part="1" 
AR Path="/5790CF91/5790DEEE" Ref="#PWR?"  Part="1" 
AR Path="/5798D1C3/5790DEEE" Ref="#PWR?"  Part="1" 
AR Path="/5798DA86/5790DEEE" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 6450 2750 50  0001 C CNN
F 1 "GND" H 6450 2850 50  0000 C CNN
F 2 "" H 6450 3000 50  0000 C CNN
F 3 "" H 6450 3000 50  0000 C CNN
	1    6450 3000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5790DF3E
P 6400 5600
AR Path="/578F869F/5790DF3E" Ref="#PWR?"  Part="1" 
AR Path="/5790CF91/5790DF3E" Ref="#PWR?"  Part="1" 
AR Path="/5798D1C3/5790DF3E" Ref="#PWR?"  Part="1" 
AR Path="/5798DA86/5790DF3E" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 6400 5350 50  0001 C CNN
F 1 "GND" H 6400 5450 50  0000 C CNN
F 2 "" H 6400 5600 50  0000 C CNN
F 3 "" H 6400 5600 50  0000 C CNN
	1    6400 5600
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 5790DF88
P 6450 1700
AR Path="/578F869F/5790DF88" Ref="#PWR?"  Part="1" 
AR Path="/5790CF91/5790DF88" Ref="#PWR?"  Part="1" 
AR Path="/5798D1C3/5790DF88" Ref="#PWR?"  Part="1" 
AR Path="/5798DA86/5790DF88" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 6450 1550 50  0001 C CNN
F 1 "VCC" H 6450 1850 50  0000 C CNN
F 2 "" H 6450 1700 50  0000 C CNN
F 3 "" H 6450 1700 50  0000 C CNN
	1    6450 1700
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 5790DFBA
P 6400 4150
AR Path="/578F869F/5790DFBA" Ref="#PWR?"  Part="1" 
AR Path="/5790CF91/5790DFBA" Ref="#PWR?"  Part="1" 
AR Path="/5798D1C3/5790DFBA" Ref="#PWR?"  Part="1" 
AR Path="/5798DA86/5790DFBA" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 6400 4000 50  0001 C CNN
F 1 "VCC" H 6400 4300 50  0000 C CNN
F 2 "" H 6400 4150 50  0000 C CNN
F 3 "" H 6400 4150 50  0000 C CNN
	1    6400 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 4150 6400 4350
Wire Wire Line
	5600 4350 5600 4850
Wire Wire Line
	5600 4850 5800 4850
Wire Wire Line
	4850 5400 5600 5400
$Comp
L Shunt Rsh?
U 1 1 5790D800
P 5250 4900
AR Path="/578F869F/5790D800" Ref="Rsh?"  Part="1" 
AR Path="/5790CF91/5790D800" Ref="Rsh?"  Part="1" 
AR Path="/5798D1C3/5790D800" Ref="Rsh?"  Part="1" 
AR Path="/5798DA86/5790D800" Ref="Rsh?"  Part="1" 
F 0 "Rsh?" H 5250 5050 60  0000 C CNN
F 1 "Shunt" H 5250 4750 60  0000 C CNN
F 2 "" H 5250 4900 60  0000 C CNN
F 3 "" H 5250 4900 60  0000 C CNN
	1    5250 4900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5600 4350 4850 4350
Wire Wire Line
	5250 4400 5250 4350
Connection ~ 5250 4350
Connection ~ 5250 5400
Wire Wire Line
	5800 4950 5600 4950
Wire Wire Line
	5600 4950 5600 5400
Wire Wire Line
	6400 5450 6400 5600
Wire Wire Line
	4850 1800 5600 1800
Wire Wire Line
	5600 1800 5600 2300
Wire Wire Line
	5600 2300 5850 2300
Wire Wire Line
	5850 2400 5600 2400
Wire Wire Line
	5600 2400 5600 2900
Wire Wire Line
	5600 2900 4850 2900
Wire Wire Line
	5250 2850 5250 2900
Connection ~ 5250 2900
Wire Wire Line
	5250 1850 5250 1800
Connection ~ 5250 1800
Wire Wire Line
	6450 3000 6450 2900
Wire Wire Line
	6450 1700 6450 1800
Text HLabel 7600 2300 2    60   Output ~ 0
OUT_A1
Text HLabel 7750 4850 2    60   Output ~ 0
OUT_A2
Wire Wire Line
	7000 4850 7750 4850
Wire Wire Line
	7050 2300 7600 2300
$Comp
L C C?
U 1 1 5798BDAD
P 6700 1750
AR Path="/578F869F/5798BDAD" Ref="C?"  Part="1" 
AR Path="/5790CF91/5798BDAD" Ref="C?"  Part="1" 
AR Path="/5798D1C3/5798BDAD" Ref="C?"  Part="1" 
AR Path="/5798DA86/5798BDAD" Ref="C?"  Part="1" 
F 0 "C?" H 6725 1850 50  0000 L CNN
F 1 "C" H 6725 1650 50  0000 L CNN
F 2 "" H 6738 1600 50  0000 C CNN
F 3 "" H 6700 1750 50  0000 C CNN
	1    6700 1750
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5798BF9E
P 7000 1750
AR Path="/578F869F/5798BF9E" Ref="#PWR?"  Part="1" 
AR Path="/5790CF91/5798BF9E" Ref="#PWR?"  Part="1" 
AR Path="/5798D1C3/5798BF9E" Ref="#PWR?"  Part="1" 
AR Path="/5798DA86/5798BF9E" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 7000 1500 50  0001 C CNN
F 1 "GND" H 7000 1600 50  0000 C CNN
F 2 "" H 7000 1750 50  0000 C CNN
F 3 "" H 7000 1750 50  0000 C CNN
	1    7000 1750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6450 1750 6550 1750
Connection ~ 6450 1750
Wire Wire Line
	6850 1750 7000 1750
$Comp
L CONN_01X05 P?
U 1 1 5798C44B
P 1300 3400
AR Path="/578F869F/5798C44B" Ref="P?"  Part="1" 
AR Path="/5790CF91/5798C44B" Ref="P?"  Part="1" 
AR Path="/5798D1C3/5798C44B" Ref="P?"  Part="1" 
AR Path="/5798DA86/5798C44B" Ref="P?"  Part="1" 
F 0 "P?" H 1300 3700 50  0000 C CNN
F 1 "CONN_01X05" V 1400 3400 50  0000 C CNN
F 2 "" H 1300 3400 50  0000 C CNN
F 3 "" H 1300 3400 50  0000 C CNN
	1    1300 3400
	-1   0    0    1   
$EndComp
Wire Wire Line
	1500 3200 2250 3200
Wire Wire Line
	1500 3600 2250 3600
Wire Wire Line
	1500 3500 2250 3500
Wire Wire Line
	1500 3300 2250 3300
Wire Wire Line
	1500 3400 2300 3400
$Comp
L GND #PWR?
U 1 1 5798C6EA
P 2300 3400
AR Path="/578F869F/5798C6EA" Ref="#PWR?"  Part="1" 
AR Path="/5790CF91/5798C6EA" Ref="#PWR?"  Part="1" 
AR Path="/5798D1C3/5798C6EA" Ref="#PWR?"  Part="1" 
AR Path="/5798DA86/5798C6EA" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 2300 3150 50  0001 C CNN
F 1 "GND" H 2300 3250 50  0000 C CNN
F 2 "" H 2300 3400 50  0000 C CNN
F 3 "" H 2300 3400 50  0000 C CNN
	1    2300 3400
	0    -1   -1   0   
$EndComp
Text Label 1650 3600 0    60   ~ 0
MP1+
Text Label 1650 3500 0    60   ~ 0
MP2+
Text Label 1650 3300 0    60   ~ 0
MP1-
Text Label 1650 3200 0    60   ~ 0
MP2-
Text Label 4500 5000 0    60   ~ 0
MP1+
Text Label 4500 4800 0    60   ~ 0
MP1-
Text Label 4550 2450 0    60   ~ 0
MP2+
Text Label 4550 2250 0    60   ~ 0
MP2-
$EndSCHEMATC
