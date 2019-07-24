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
LIBS:SirenSong-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L Q_NPN_EBC Q?
U 1 1 559F1348
P 6300 2900
F 0 "Q?" H 6600 2950 50  0000 R CNN
F 1 "Q_NPN_EBC" H 6900 2850 50  0000 R CNN
F 2 "Housings_TO-92:TO-92_Inline_Wide" H 6500 3000 29  0000 C CNN
F 3 "" H 6300 2900 60  0000 C CNN
	1    6300 2900
	1    0    0    -1  
$EndComp
$Comp
L Q_PNP_EBC Q?
U 1 1 559F1388
P 7100 2600
F 0 "Q?" H 7400 2650 50  0000 R CNN
F 1 "Q_PNP_EBC" H 7700 2550 50  0000 R CNN
F 2 "Housings_TO-92:TO-92_Inline_Wide" H 7300 2700 29  0000 C CNN
F 3 "" H 7100 2600 60  0000 C CNN
	1    7100 2600
	1    0    0    1   
$EndComp
$Comp
L Q_NMOS_GDS Q?
U 1 1 559F13E9
P 8200 2100
F 0 "Q?" H 8500 2150 50  0000 R CNN
F 1 "Q_NMOS_GDS" H 8850 2050 50  0000 R CNN
F 2 "Transistors_TO-220:TO-220_FET-GDS_Vertical" H 8400 2200 29  0000 C CNN
F 3 "" H 8200 2100 60  0000 C CNN
	1    8200 2100
	1    0    0    -1  
$EndComp
$Comp
L Q_PMOS_GDS Q?
U 1 1 559F1477
P 8200 1500
F 0 "Q?" H 8500 1550 50  0000 R CNN
F 1 "Q_PMOS_GDS" H 8850 1450 50  0000 R CNN
F 2 "Transistors_TO-220:TO-220_FET-GDS_Vertical" H 8400 1600 29  0000 C CNN
F 3 "" H 8200 1500 60  0000 C CNN
	1    8200 1500
	1    0    0    1   
$EndComp
$Comp
L D D?
U 1 1 559F14EB
P 9050 2100
F 0 "D?" H 9050 2200 50  0000 C CNN
F 1 "D" H 9050 2000 50  0000 C CNN
F 2 "Diodes_ThroughHole:Diode_DO-201AD_Horizontal_RM15" H 9050 2100 28  0000 C CNN
F 3 "" H 9050 2100 60  0000 C CNN
	1    9050 2100
	0    1    1    0   
$EndComp
$Comp
L CONN_01X02 P?
U 1 1 559F1541
P 9650 2100
F 0 "P?" H 9650 2250 50  0000 C CNN
F 1 "12V MOTOR" V 9750 2100 50  0000 C CNN
F 2 "" H 9650 2100 60  0000 C CNN
F 3 "" H 9650 2100 60  0000 C CNN
	1    9650 2100
	1    0    0    1   
$EndComp
$Comp
L +12V #PWR?
U 1 1 559F1818
P 8300 1300
F 0 "#PWR?" H 8300 1150 50  0001 C CNN
F 1 "+12V" H 8300 1440 50  0000 C CNN
F 2 "" H 8300 1300 60  0000 C CNN
F 3 "" H 8300 1300 60  0000 C CNN
	1    8300 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 1700 8300 1800
Wire Wire Line
	8300 1800 8300 1900
Wire Wire Line
	8300 1800 9350 1800
Wire Wire Line
	9050 1800 9050 1950
Connection ~ 8300 1800
Wire Wire Line
	9350 1800 9350 2050
Wire Wire Line
	9350 2050 9450 2050
$Comp
L GND #PWR?
U 1 1 559F19EE
P 8300 2500
F 0 "#PWR?" H 8300 2250 50  0001 C CNN
F 1 "GND" H 8300 2350 50  0000 C CNN
F 2 "" H 8300 2500 60  0000 C CNN
F 3 "" H 8300 2500 60  0000 C CNN
	1    8300 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 2300 8300 2400
Wire Wire Line
	8300 2400 8300 2500
Wire Wire Line
	8300 2400 9050 2400
Wire Wire Line
	9050 2400 9350 2400
Wire Wire Line
	9050 2400 9050 2250
Connection ~ 8300 2400
Wire Wire Line
	9350 2400 9350 2150
Wire Wire Line
	9350 2150 9450 2150
Connection ~ 9050 2400
$Comp
L R R?
U 1 1 559F1C6F
P 7200 1500
F 0 "R?" V 7280 1500 50  0000 C CNN
F 1 "470" V 7200 1500 50  0000 C CNN
F 2 "" V 7130 1500 30  0000 C CNN
F 3 "" H 7200 1500 30  0000 C CNN
	1    7200 1500
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 559F1CB4
P 7850 2100
F 0 "R?" V 7930 2100 50  0000 C CNN
F 1 "100" V 7850 2100 50  0000 C CNN
F 2 "" V 7780 2100 30  0000 C CNN
F 3 "" H 7850 2100 30  0000 C CNN
	1    7850 2100
	0    1    1    0   
$EndComp
$Comp
L D_Schottky D?
U 1 1 559F1D3C
P 7600 1800
F 0 "D?" H 7600 1900 50  0000 C CNN
F 1 "D_Schottky" H 7600 1700 50  0000 C CNN
F 2 "" H 7600 1800 60  0000 C CNN
F 3 "" H 7600 1800 60  0000 C CNN
	1    7600 1800
	0    1    1    0   
$EndComp
Wire Wire Line
	7350 1500 7600 1500
Wire Wire Line
	7600 1500 8000 1500
Wire Wire Line
	7600 1500 7600 1650
Connection ~ 7600 1500
Wire Wire Line
	7600 1950 7600 2100
Wire Wire Line
	7600 2100 7600 2900
Wire Wire Line
	7600 2100 7700 2100
$Comp
L Q_NPN_EBC Q?
U 1 1 559F1EC1
P 6700 1800
F 0 "Q?" H 7000 1850 50  0000 R CNN
F 1 "Q_NPN_EBC" H 7300 1750 50  0000 R CNN
F 2 "Housings_TO-92:TO-92_Inline_Wide" H 6900 1900 29  0000 C CNN
F 3 "" H 6700 1800 60  0000 C CNN
	1    6700 1800
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 559F1EF9
P 6800 1250
F 0 "R?" V 6880 1250 50  0000 C CNN
F 1 "10k" V 6800 1250 50  0000 C CNN
F 2 "" V 6730 1250 30  0000 C CNN
F 3 "" H 6800 1250 30  0000 C CNN
	1    6800 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 1400 6800 1500
Wire Wire Line
	6800 1500 6800 1600
Wire Wire Line
	7050 1500 6800 1500
Connection ~ 6800 1500
$Comp
L +12V #PWR?
U 1 1 559F1F99
P 6800 1100
F 0 "#PWR?" H 6800 950 50  0001 C CNN
F 1 "+12V" H 6800 1240 50  0000 C CNN
F 2 "" H 6800 1100 60  0000 C CNN
F 3 "" H 6800 1100 60  0000 C CNN
	1    6800 1100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 559F215E
P 6800 2000
F 0 "#PWR?" H 6800 1750 50  0001 C CNN
F 1 "GND" H 6800 1850 50  0000 C CNN
F 2 "" H 6800 2000 60  0000 C CNN
F 3 "" H 6800 2000 60  0000 C CNN
	1    6800 2000
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 559F21DC
P 6350 1800
F 0 "R?" V 6430 1800 50  0000 C CNN
F 1 "1k" V 6350 1800 50  0000 C CNN
F 2 "" V 6280 1800 30  0000 C CNN
F 3 "" H 6350 1800 30  0000 C CNN
	1    6350 1800
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 559F238A
P 7200 3150
F 0 "R?" V 7280 3150 50  0000 C CNN
F 1 "10k" V 7200 3150 50  0000 C CNN
F 2 "" V 7130 3150 30  0000 C CNN
F 3 "" H 7200 3150 30  0000 C CNN
	1    7200 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 2800 7200 2900
Wire Wire Line
	7200 2900 7200 3000
$Comp
L R R?
U 1 1 559F2779
P 6650 2600
F 0 "R?" V 6730 2600 50  0000 C CNN
F 1 "1k" V 6650 2600 50  0000 C CNN
F 2 "" V 6580 2600 30  0000 C CNN
F 3 "" H 6650 2600 30  0000 C CNN
	1    6650 2600
	0    1    1    0   
$EndComp
Wire Wire Line
	6800 2600 6900 2600
$Comp
L R R?
U 1 1 559F281A
P 6400 2350
F 0 "R?" V 6480 2350 50  0000 C CNN
F 1 "10k" V 6400 2350 50  0000 C CNN
F 2 "" V 6330 2350 30  0000 C CNN
F 3 "" H 6400 2350 30  0000 C CNN
	1    6400 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 2500 6400 2600
Wire Wire Line
	6400 2600 6400 2700
Wire Wire Line
	6500 2600 6400 2600
Connection ~ 6400 2600
$Comp
L GND #PWR?
U 1 1 559F28E3
P 6400 3100
F 0 "#PWR?" H 6400 2850 50  0001 C CNN
F 1 "GND" H 6400 2950 50  0000 C CNN
F 2 "" H 6400 3100 60  0000 C CNN
F 3 "" H 6400 3100 60  0000 C CNN
	1    6400 3100
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR?
U 1 1 559F29E1
P 7200 2400
F 0 "#PWR?" H 7200 2250 50  0001 C CNN
F 1 "+12V" H 7200 2540 50  0000 C CNN
F 2 "" H 7200 2400 60  0000 C CNN
F 3 "" H 7200 2400 60  0000 C CNN
	1    7200 2400
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 559F339B
P 5950 2900
F 0 "R?" V 6030 2900 50  0000 C CNN
F 1 "1k" V 5950 2900 50  0000 C CNN
F 2 "" V 5880 2900 30  0000 C CNN
F 3 "" H 5950 2900 30  0000 C CNN
	1    5950 2900
	0    1    1    0   
$EndComp
$Comp
L +12V #PWR?
U 1 1 559F3432
P 6400 2200
F 0 "#PWR?" H 6400 2050 50  0001 C CNN
F 1 "+12V" H 6400 2340 50  0000 C CNN
F 2 "" H 6400 2200 60  0000 C CNN
F 3 "" H 6400 2200 60  0000 C CNN
	1    6400 2200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 559F34FB
P 7200 3300
F 0 "#PWR?" H 7200 3050 50  0001 C CNN
F 1 "GND" H 7200 3150 50  0000 C CNN
F 2 "" H 7200 3300 60  0000 C CNN
F 3 "" H 7200 3300 60  0000 C CNN
	1    7200 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 2900 7200 2900
Connection ~ 7200 2900
Connection ~ 7600 2100
Text GLabel 6200 1800 0    50   Input ~ 0
MOTOR_PWM
Text GLabel 5800 2900 0    50   Input ~ 0
BRAKE
Text Notes 7900 3350 0    60   ~ 0
This fail-safe driver allows the\nbrake to override the PWM.\nThis prevents shoot-through\nin case both inputs are active.\nA typical half H-bridge can\nbe used in place of this circuit.
$Comp
L ATMEGA328P-P IC?
U 1 1 55A14CD2
P 3000 2300
F 0 "IC?" H 2250 3550 40  0000 L BNN
F 1 "ATMEGA328P-P" H 3400 900 40  0000 L BNN
F 2 "DIL28" H 3000 2300 30  0000 C CIN
F 3 "" H 3000 2300 60  0000 C CNN
	1    3000 2300
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5D37ADFC
P 8300 4500
F 0 "R?" V 8380 4500 50  0000 C CNN
F 1 "10k" V 8300 4500 50  0000 C CNN
F 2 "" V 8230 4500 30  0000 C CNN
F 3 "" H 8300 4500 30  0000 C CNN
	1    8300 4500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5D37B1C3
P 8300 4900
F 0 "C?" H 8325 5000 50  0000 L CNN
F 1 "10nF" H 8325 4800 50  0000 L CNN
F 2 "" H 8338 4750 30  0000 C CNN
F 3 "" H 8300 4900 60  0000 C CNN
	1    8300 4900
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X03 P?
U 1 1 5D37B395
P 9000 4200
F 0 "P?" H 9000 4400 50  0000 C CNN
F 1 "TACHOMETER" V 9100 4200 50  0000 C CNN
F 2 "" H 9000 4200 60  0000 C CNN
F 3 "" H 9000 4200 60  0000 C CNN
	1    9000 4200
	1    0    0    1   
$EndComp
$Comp
L GND #PWR?
U 1 1 5D37B4C9
P 8800 4300
F 0 "#PWR?" H 8800 4050 50  0001 C CNN
F 1 "GND" V 8800 4100 50  0000 C CNN
F 2 "" H 8800 4300 60  0000 C CNN
F 3 "" H 8800 4300 60  0000 C CNN
	1    8800 4300
	0    1    1    0   
$EndComp
Wire Wire Line
	7700 4200 8300 4200
Wire Wire Line
	8300 4200 8800 4200
Wire Wire Line
	8300 4200 8300 4350
Wire Wire Line
	8300 4650 8300 4700
Wire Wire Line
	8300 4700 8300 4750
$Comp
L GND #PWR?
U 1 1 5D37B5C6
P 8300 5050
F 0 "#PWR?" H 8300 4800 50  0001 C CNN
F 1 "GND" H 8300 4900 50  0000 C CNN
F 2 "" H 8300 5050 60  0000 C CNN
F 3 "" H 8300 5050 60  0000 C CNN
	1    8300 5050
	1    0    0    -1  
$EndComp
$Comp
L CP1 C?
U 1 1 5D37B731
P 7700 4900
F 0 "C?" H 7725 5000 50  0000 L CNN
F 1 "1uF" H 7725 4800 50  0000 L CNN
F 2 "" H 7700 4900 60  0000 C CNN
F 3 "" H 7700 4900 60  0000 C CNN
	1    7700 4900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5D37B78D
P 7700 5150
F 0 "#PWR?" H 7700 4900 50  0001 C CNN
F 1 "GND" H 7700 5000 50  0000 C CNN
F 2 "" H 7700 5150 60  0000 C CNN
F 3 "" H 7700 5150 60  0000 C CNN
	1    7700 5150
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5D37B801
P 7700 4500
F 0 "R?" V 7780 4500 50  0000 C CNN
F 1 "100k" V 7700 4500 50  0000 C CNN
F 2 "" V 7630 4500 30  0000 C CNN
F 3 "" H 7700 4500 30  0000 C CNN
	1    7700 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 4650 7700 4700
Wire Wire Line
	7700 4700 7700 4750
Connection ~ 8300 4200
Wire Wire Line
	7700 4200 7700 4350
$Comp
L R R?
U 1 1 5D37BABE
P 7400 4900
F 0 "R?" V 7480 4900 50  0000 C CNN
F 1 "100k" V 7400 4900 50  0000 C CNN
F 2 "" V 7330 4900 30  0000 C CNN
F 3 "" H 7400 4900 30  0000 C CNN
	1    7400 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 5050 7700 5100
Wire Wire Line
	7700 5100 7700 5150
Wire Wire Line
	7400 5050 7400 5100
Wire Wire Line
	7400 5100 7700 5100
Connection ~ 7700 5100
Wire Wire Line
	7400 4750 7400 4700
Wire Wire Line
	7300 4700 7400 4700
Wire Wire Line
	7400 4700 7700 4700
Connection ~ 7700 4700
Connection ~ 7400 4700
Wire Wire Line
	8200 4700 8300 4700
Connection ~ 8300 4700
Text GLabel 4000 3400 2    50   Input ~ 0
AIN0
Text GLabel 7300 4700 0    50   Output ~ 0
AIN0
Text GLabel 8200 4700 0    50   Output ~ 0
AIN1
Text GLabel 4000 3500 2    50   Input ~ 0
AIN1
Text GLabel 4000 1200 2    50   Output ~ 0
BRAKE
Text GLabel 4000 1300 2    50   Output ~ 0
MOTOR_PWM
$Comp
L LED D?
U 1 1 5D37D51C
P 5700 4900
F 0 "D?" H 5700 5000 50  0000 C CNN
F 1 "GREEN" H 5700 4800 50  0000 C CNN
F 2 "" H 5700 4900 60  0000 C CNN
F 3 "" H 5700 4900 60  0000 C CNN
	1    5700 4900
	0    -1   -1   0   
$EndComp
Text GLabel 4000 1700 2    50   Output ~ 0
LED_BUILTIN
$Comp
L LED D?
U 1 1 5D37D867
P 6000 4900
F 0 "D?" H 6000 5000 50  0000 C CNN
F 1 "RED" H 6000 4800 50  0000 C CNN
F 2 "" H 6000 4900 60  0000 C CNN
F 3 "" H 6000 4900 60  0000 C CNN
	1    6000 4900
	0    -1   -1   0   
$EndComp
$Comp
L LED D?
U 1 1 5D37D8F2
P 6300 4900
F 0 "D?" H 6300 5000 50  0000 C CNN
F 1 "YELLOW" H 6300 4800 50  0000 C CNN
F 2 "" H 6300 4900 60  0000 C CNN
F 3 "" H 6300 4900 60  0000 C CNN
	1    6300 4900
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5D37D977
P 6300 5400
F 0 "#PWR?" H 6300 5150 50  0001 C CNN
F 1 "GND" H 6300 5250 50  0000 C CNN
F 2 "" H 6300 5400 60  0000 C CNN
F 3 "" H 6300 5400 60  0000 C CNN
	1    6300 5400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5D37D9D0
P 6000 5400
F 0 "#PWR?" H 6000 5150 50  0001 C CNN
F 1 "GND" H 6000 5250 50  0000 C CNN
F 2 "" H 6000 5400 60  0000 C CNN
F 3 "" H 6000 5400 60  0000 C CNN
	1    6000 5400
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5D37DADA
P 6000 5250
F 0 "R?" V 6080 5250 50  0000 C CNN
F 1 "330" V 6000 5250 50  0000 C CNN
F 2 "" V 5930 5250 30  0000 C CNN
F 3 "" H 6000 5250 30  0000 C CNN
	1    6000 5250
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5D37DD05
P 6300 5250
F 0 "R?" V 6380 5250 50  0000 C CNN
F 1 "330" V 6300 5250 50  0000 C CNN
F 2 "" V 6230 5250 30  0000 C CNN
F 3 "" H 6300 5250 30  0000 C CNN
	1    6300 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 5100 6000 5100
Text GLabel 5700 4700 1    50   Input ~ 0
MOTOR_PWM
Text GLabel 6000 4700 1    50   Input ~ 0
BRAKE
Text GLabel 6300 4700 1    50   Input ~ 0
LED_BUILTIN
$Comp
L +5V #PWR?
U 1 1 5D37E6B1
P 8800 4100
F 0 "#PWR?" H 8800 3950 50  0001 C CNN
F 1 "+5V" V 8800 4300 50  0000 C CNN
F 2 "" H 8800 4100 60  0000 C CNN
F 3 "" H 8800 4100 60  0000 C CNN
	1    8800 4100
	0    -1   -1   0   
$EndComp
Text Notes 5750 4050 0    60   ~ 0
INDICATORS
Text Notes 7450 4050 0    60   ~ 0
TACHOMETER FILTERS
$Comp
L 7805 U?
U 1 1 5D37F5D4
P 4300 6750
F 0 "U?" H 4450 6554 60  0000 C CNN
F 1 "7805" H 4300 6950 60  0000 C CNN
F 2 "" H 4300 6750 60  0000 C CNN
F 3 "" H 4300 6750 60  0000 C CNN
	1    4300 6750
	1    0    0    -1  
$EndComp
$Comp
L CP1 C?
U 1 1 5D37FB69
P 4700 6850
F 0 "C?" H 4725 6950 50  0000 L CNN
F 1 "100uF" H 4725 6750 50  0000 L CNN
F 2 "" H 4700 6850 60  0000 C CNN
F 3 "" H 4700 6850 60  0000 C CNN
	1    4700 6850
	1    0    0    -1  
$EndComp
$Comp
L CP1 C?
U 1 1 5D37FD61
P 3900 6850
F 0 "C?" H 3925 6950 50  0000 L CNN
F 1 "2200uF" H 3925 6750 50  0000 L CNN
F 2 "" H 3900 6850 60  0000 C CNN
F 3 "" H 3900 6850 60  0000 C CNN
	1    3900 6850
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 5D37FF9E
P 6000 7200
F 0 "#PWR?" H 6000 7050 50  0001 C CNN
F 1 "+5V" V 6000 7400 50  0000 C CNN
F 2 "" H 6000 7200 60  0000 C CNN
F 3 "" H 6000 7200 60  0000 C CNN
	1    6000 7200
	0    1    1    0   
$EndComp
$Comp
L D_Schottky D?
U 1 1 5D38029E
P 5200 6850
F 0 "D?" H 5200 6950 50  0000 C CNN
F 1 "1N5817" H 5200 6750 50  0000 C CNN
F 2 "" H 5200 6850 60  0000 C CNN
F 3 "" H 5200 6850 60  0000 C CNN
	1    5200 6850
	0    -1   -1   0   
$EndComp
$Comp
L D_Schottky D?
U 1 1 5D38067D
P 5500 6850
F 0 "D?" H 5500 6950 50  0000 C CNN
F 1 "1N5817" H 5500 6750 50  0000 C CNN
F 2 "" H 5500 6850 60  0000 C CNN
F 3 "" H 5500 6850 60  0000 C CNN
	1    5500 6850
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X06 P?
U 1 1 5D381977
P 1300 5850
F 0 "P?" H 1300 6200 50  0000 C CNN
F 1 "FTDI PROGRAMMER" V 1400 5850 50  0000 C CNN
F 2 "" H 1300 5850 60  0000 C CNN
F 3 "" H 1300 5850 60  0000 C CNN
	1    1300 5850
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5D381B15
P 1500 5600
F 0 "#PWR?" H 1500 5350 50  0001 C CNN
F 1 "GND" V 1500 5400 50  0000 C CNN
F 2 "" H 1500 5600 60  0000 C CNN
F 3 "" H 1500 5600 60  0000 C CNN
	1    1500 5600
	0    -1   -1   0   
$EndComp
$Comp
L +5VP #PWR?
U 1 1 5D381DDB
P 1500 5800
F 0 "#PWR?" H 1500 5650 50  0001 C CNN
F 1 "+5VP" V 1500 6000 50  0000 C CNN
F 2 "" H 1500 5800 60  0000 C CNN
F 3 "" H 1500 5800 60  0000 C CNN
	1    1500 5800
	0    1    1    0   
$EndComp
$Comp
L SW_PUSH SW?
U 1 1 5D382327
P 1300 4900
F 0 "SW?" H 1450 5010 50  0000 C CNN
F 1 "RESET" H 1300 4820 50  0000 C CNN
F 2 "" H 1300 4900 60  0000 C CNN
F 3 "" H 1300 4900 60  0000 C CNN
	1    1300 4900
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5D3824DB
P 1300 5200
F 0 "#PWR?" H 1300 4950 50  0001 C CNN
F 1 "GND" V 1300 5000 50  0000 C CNN
F 2 "" H 1300 5200 60  0000 C CNN
F 3 "" H 1300 5200 60  0000 C CNN
	1    1300 5200
	0    -1   -1   0   
$EndComp
Text GLabel 4000 2650 2    50   Input ~ 0
~RESET
Text GLabel 1400 4600 2    50   Output ~ 0
~RESET
$Comp
L R R?
U 1 1 5D383024
P 1300 4450
F 0 "R?" V 1380 4450 50  0000 C CNN
F 1 "10k" V 1300 4450 50  0000 C CNN
F 2 "" V 1230 4450 30  0000 C CNN
F 3 "" H 1300 4450 30  0000 C CNN
	1    1300 4450
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 5D383532
P 1300 4300
F 0 "#PWR?" H 1300 4150 50  0001 C CNN
F 1 "+5V" H 1300 4440 50  0000 C CNN
F 2 "" H 1300 4300 60  0000 C CNN
F 3 "" H 1300 4300 60  0000 C CNN
	1    1300 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 4600 1300 4600
$Comp
L THERMISTOR TH?
U 1 1 5D383969
P 5750 7200
F 0 "TH?" V 5850 7250 50  0000 C CNN
F 1 "250mA PTC" V 5650 7200 50  0000 C BNN
F 2 "" H 5750 7200 60  0000 C CNN
F 3 "" H 5750 7200 60  0000 C CNN
	1    5750 7200
	0    1    1    0   
$EndComp
$Comp
L +5VP #PWR?
U 1 1 5D383B2C
P 5500 6700
F 0 "#PWR?" H 5500 6550 50  0001 C CNN
F 1 "+5VP" H 5500 6840 50  0000 C CNN
F 2 "" H 5500 6700 60  0000 C CNN
F 3 "" H 5500 6700 60  0000 C CNN
	1    5500 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 7000 5500 7200
Wire Wire Line
	5500 7200 5200 7200
Wire Wire Line
	5200 7200 5200 7000
Wire Wire Line
	5200 6700 4700 6700
$Comp
L BARREL_JACK CON?
U 1 1 5D38421E
P 2900 6800
F 0 "CON?" H 2900 7050 60  0000 C CNN
F 1 "12V 3.3A" H 2900 6600 60  0000 C CNN
F 2 "" H 2900 6800 60  0000 C CNN
F 3 "" H 2900 6800 60  0000 C CNN
	1    2900 6800
	1    0    0    -1  
$EndComp
$Comp
L CP1 C?
U 1 1 5D384EAB
P 3600 6850
F 0 "C?" H 3625 6950 50  0000 L CNN
F 1 "220uF" H 3625 6750 50  0000 L CNN
F 2 "" H 3600 6850 60  0000 C CNN
F 3 "" H 3600 6850 60  0000 C CNN
	1    3600 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 6700 3600 6700
Wire Wire Line
	3600 6700 3900 6700
Connection ~ 3600 6700
$Comp
L GND #PWR?
U 1 1 5D385132
P 4700 7000
F 0 "#PWR?" H 4700 6750 50  0001 C CNN
F 1 "GND" H 4700 6850 50  0000 C CNN
F 2 "" H 4700 7000 60  0000 C CNN
F 3 "" H 4700 7000 60  0000 C CNN
	1    4700 7000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5D3851B2
P 4300 7000
F 0 "#PWR?" H 4300 6750 50  0001 C CNN
F 1 "GND" H 4300 6850 50  0000 C CNN
F 2 "" H 4300 7000 60  0000 C CNN
F 3 "" H 4300 7000 60  0000 C CNN
	1    4300 7000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5D385232
P 3900 7000
F 0 "#PWR?" H 3900 6750 50  0001 C CNN
F 1 "GND" H 3900 6850 50  0000 C CNN
F 2 "" H 3900 7000 60  0000 C CNN
F 3 "" H 3900 7000 60  0000 C CNN
	1    3900 7000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5D3852B2
P 3600 7000
F 0 "#PWR?" H 3600 6750 50  0001 C CNN
F 1 "GND" H 3600 6850 50  0000 C CNN
F 2 "" H 3600 7000 60  0000 C CNN
F 3 "" H 3600 7000 60  0000 C CNN
	1    3600 7000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5D385332
P 3300 7000
F 0 "#PWR?" H 3300 6750 50  0001 C CNN
F 1 "GND" H 3300 6850 50  0000 C CNN
F 2 "" H 3300 7000 60  0000 C CNN
F 3 "" H 3300 7000 60  0000 C CNN
	1    3300 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 6800 3300 6900
Wire Wire Line
	3300 6900 3300 7000
Wire Wire Line
	3300 6800 3200 6800
Wire Wire Line
	3200 6900 3300 6900
Connection ~ 3300 6900
$Comp
L CP1 C?
U 1 1 5D385805
P 1400 1350
F 0 "C?" H 1425 1450 50  0000 L CNN
F 1 "100uF" H 1425 1250 50  0000 L CNN
F 2 "" H 1400 1350 60  0000 C CNN
F 3 "" H 1400 1350 60  0000 C CNN
	1    1400 1350
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5D385FB7
P 1700 1350
F 0 "C?" H 1725 1450 50  0000 L CNN
F 1 "0.1uF" H 1725 1250 50  0000 L CNN
F 2 "" H 1738 1200 30  0000 C CNN
F 3 "" H 1700 1350 60  0000 C CNN
	1    1700 1350
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 5D386056
P 1400 1200
F 0 "#PWR?" H 1400 1050 50  0001 C CNN
F 1 "+5V" H 1400 1340 50  0000 C CNN
F 2 "" H 1400 1200 60  0000 C CNN
F 3 "" H 1400 1200 60  0000 C CNN
	1    1400 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 1200 1700 1200
Wire Wire Line
	1700 1200 2100 1200
Connection ~ 1700 1200
$Comp
L GND #PWR?
U 1 1 5D386186
P 1400 1500
F 0 "#PWR?" H 1400 1250 50  0001 C CNN
F 1 "GND" H 1400 1350 50  0000 C CNN
F 2 "" H 1400 1500 60  0000 C CNN
F 3 "" H 1400 1500 60  0000 C CNN
	1    1400 1500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5D386213
P 1700 1500
F 0 "#PWR?" H 1700 1250 50  0001 C CNN
F 1 "GND" H 1700 1350 50  0000 C CNN
F 2 "" H 1700 1500 60  0000 C CNN
F 3 "" H 1700 1500 60  0000 C CNN
	1    1700 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 1200 2100 1500
Wire Wire Line
	2100 1500 2100 1800
Connection ~ 2100 1500
$Comp
L Crystal Y?
U 1 1 5D386B64
P 4750 2000
F 0 "Y?" H 4750 2150 50  0000 C CNN
F 1 "16MHz" H 4750 1850 50  0000 C CNN
F 2 "" H 4750 2000 60  0000 C CNN
F 3 "" H 4750 2000 60  0000 C CNN
	1    4750 2000
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5D386C9F
P 4500 2150
F 0 "C?" H 4525 2250 50  0000 L CNN
F 1 "22pF" H 4525 2050 50  0000 L CNN
F 2 "" H 4538 2000 30  0000 C CNN
F 3 "" H 4500 2150 60  0000 C CNN
	1    4500 2150
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5D386F3D
P 5000 2150
F 0 "C?" H 5025 2250 50  0000 L CNN
F 1 "22pF" H 5025 2050 50  0000 L CNN
F 2 "" H 5038 2000 30  0000 C CNN
F 3 "" H 5000 2150 60  0000 C CNN
	1    5000 2150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5D386FE1
P 4500 2300
F 0 "#PWR?" H 4500 2050 50  0001 C CNN
F 1 "GND" H 4500 2150 50  0000 C CNN
F 2 "" H 4500 2300 60  0000 C CNN
F 3 "" H 4500 2300 60  0000 C CNN
	1    4500 2300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5D387070
P 5000 2300
F 0 "#PWR?" H 5000 2050 50  0001 C CNN
F 1 "GND" H 5000 2150 50  0000 C CNN
F 2 "" H 5000 2300 60  0000 C CNN
F 3 "" H 5000 2300 60  0000 C CNN
	1    5000 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 1900 4500 1900
Wire Wire Line
	4500 1900 4500 2000
Wire Wire Line
	4500 2000 4600 2000
Wire Wire Line
	4000 1800 5000 1800
Wire Wire Line
	5000 1800 5000 2000
Wire Wire Line
	5000 2000 4900 2000
NoConn ~ 1500 6100
Text GLabel 2000 5900 2    50   Output ~ 0
RX
NoConn ~ 1500 5700
Text GLabel 4000 2800 2    50   Input ~ 0
RX
Text GLabel 4000 2900 2    50   Output ~ 0
TX
Text GLabel 2000 6000 2    50   Input ~ 0
TX
$Comp
L R R?
U 1 1 5D3892A4
P 1700 6150
F 0 "R?" V 1780 6150 50  0000 C CNN
F 1 "10k" V 1700 6150 50  0000 C CNN
F 2 "" V 1630 6150 30  0000 C CNN
F 3 "" H 1700 6150 30  0000 C CNN
	1    1700 6150
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5D389362
P 1850 6000
F 0 "R?" V 1930 6000 50  0000 C CNN
F 1 "6.8k" V 1850 6000 50  0000 C CNN
F 2 "" V 1780 6000 30  0000 C CNN
F 3 "" H 1850 6000 30  0000 C CNN
	1    1850 6000
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5D3894D4
P 1700 6300
F 0 "#PWR?" H 1700 6050 50  0001 C CNN
F 1 "GND" H 1700 6150 50  0000 C CNN
F 2 "" H 1700 6300 60  0000 C CNN
F 3 "" H 1700 6300 60  0000 C CNN
	1    1700 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 6000 1500 6000
Wire Wire Line
	2000 5900 1500 5900
Text Notes 4000 6400 0    60   ~ 0
POWER SUPPLY
Text Notes 1900 4800 0    60   ~ 0
PROGRAMMING INTERFACE
$Comp
L GND #PWR?
U 1 1 5D38BC7F
P 2100 3500
F 0 "#PWR?" H 2100 3250 50  0001 C CNN
F 1 "GND" H 2100 3350 50  0000 C CNN
F 2 "" H 2100 3500 60  0000 C CNN
F 3 "" H 2100 3500 60  0000 C CNN
	1    2100 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 3500 2100 3400
$Comp
L POT RV?
U 1 1 5D38D05D
P 10300 4150
F 0 "RV?" H 10300 4050 50  0000 C CNN
F 1 "10k" H 10300 4150 50  0000 C CNN
F 2 "" H 10300 4150 60  0000 C CNN
F 3 "" H 10300 4150 60  0000 C CNN
	1    10300 4150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9800 3900 10300 3900
Wire Wire Line
	10300 3900 10700 3900
$Comp
L LED D?
U 1 1 5D38D5F7
P 10300 4600
F 0 "D?" H 10300 4700 50  0000 C CNN
F 1 "LED" H 10300 4500 50  0000 C CNN
F 2 "" H 10300 4600 60  0000 C CNN
F 3 "" H 10300 4600 60  0000 C CNN
	1    10300 4600
	0    -1   -1   0   
$EndComp
Text Notes 9800 3700 0    60   ~ 0
OPTICAL TACHOMETER
$Comp
L CONN_01X03 P?
U 1 1 5D38E3D8
P 9300 4200
F 0 "P?" H 9300 4400 50  0000 C CNN
F 1 "TACHOMETER" V 9400 4200 50  0000 C CNN
F 2 "" H 9300 4200 60  0000 C CNN
F 3 "" H 9300 4200 60  0000 C CNN
	1    9300 4200
	-1   0    0    1   
$EndComp
$Comp
L OPTO_NPN Q?
U 1 1 5D38E6AA
P 10600 4800
F 0 "Q?" H 10750 4850 50  0000 L CNN
F 1 "OPTO_NPN" H 10750 4700 50  0000 L CNN
F 2 "" H 10600 4800 60  0000 C CNN
F 3 "" H 10600 4800 60  0000 C CNN
	1    10600 4800
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5D38E75F
P 10700 5150
F 0 "R?" V 10780 5150 50  0000 C CNN
F 1 "75k" V 10700 5150 50  0000 C CNN
F 2 "" V 10630 5150 30  0000 C CNN
F 3 "" H 10700 5150 30  0000 C CNN
	1    10700 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 4400 10150 4400
Wire Wire Line
	10150 4400 10150 4150
Wire Wire Line
	10700 3900 10700 4600
Connection ~ 10300 3900
Wire Wire Line
	9500 4100 9800 4100
Wire Wire Line
	9800 4100 9800 3900
Wire Wire Line
	10700 5000 9900 5000
Wire Wire Line
	9900 5000 9900 4200
Wire Wire Line
	9900 4200 9500 4200
Wire Wire Line
	9500 4300 9800 4300
Wire Wire Line
	9800 4300 9800 5300
Wire Wire Line
	9800 5300 10300 5300
Wire Wire Line
	10300 5300 10700 5300
Wire Wire Line
	10300 4800 10300 5300
Connection ~ 10300 5300
NoConn ~ 4000 1400
NoConn ~ 4000 1500
NoConn ~ 4000 1600
NoConn ~ 4000 2050
NoConn ~ 4000 2150
NoConn ~ 4000 2250
NoConn ~ 4000 2350
NoConn ~ 4000 2450
NoConn ~ 4000 2550
NoConn ~ 4000 3000
NoConn ~ 4000 3100
NoConn ~ 4000 3200
NoConn ~ 4000 3300
Text Notes 7250 1100 0    60   ~ 0
MOTOR DRIVER
Text Notes 9200 5800 0    60   ~ 0
The tachometer is a reflective sensor.\nIt uses a red LED to illuminate a black-\nand-white pinwheel pattern on the\nback of the rotor.
Text Notes 2500 900  0    60   ~ 0
MICROCONTROLLER
Text Notes 7250 6150 0    60   ~ 0
The filter on AIN0 outputs the\naverage tachometer voltage.\nThe filter on AIN1 output a\nsmooth sine or square wave.\nThe analog comparator detects\nwhen these signals cross each\nother to read the black and white\npatches on the rotor.
$Comp
L +12V #PWR?
U 1 1 5D396DEA
P 3600 6700
F 0 "#PWR?" H 3600 6550 50  0001 C CNN
F 1 "+12V" H 3600 6840 50  0000 C CNN
F 2 "" H 3600 6700 60  0000 C CNN
F 3 "" H 3600 6700 60  0000 C CNN
	1    3600 6700
	1    0    0    -1  
$EndComp
Text Notes 2600 7600 0    60   ~ 0
Large capacitors are necessary on the 12V power rail to prevent interference.\nIf the microcontroller resets whenever the motor turns on, add more caps.
$EndSCHEMATC
