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
LIBS:alvaroferran
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "2017-08-17"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Notes 7050 6800 0    138  ~ 28
Neo Air Shield
Text Notes 7050 7000 0    79   ~ 0
Alvaro Ferrán\n
Text Notes 650  2150 0    100  ~ 0
3.3V REG
$Comp
L AP2112K-3.3TRG1 U1
U 1 1 59957A1E
P 1900 1250
F 0 "U1" H 1550 1000 50  0000 C CNN
F 1 "AP2112K-3.3TRG1" H 1900 1600 50  0000 C CNN
F 2 "" H 1700 1450 60  0001 C CNN
F 3 "" H 1700 1450 60  0001 C CNN
	1    1900 1250
	1    0    0    -1  
$EndComp
NoConn ~ 2500 1250
$Comp
L C C6
U 1 1 59958A8B
P 2600 1500
F 0 "C6" H 2450 1600 50  0000 L CNN
F 1 "1uF" H 2625 1400 50  0000 L CNN
F 2 "" H 2638 1350 50  0001 C CNN
F 3 "" H 2600 1500 50  0001 C CNN
	1    2600 1500
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 59958ADE
P 1200 1500
F 0 "C5" H 1225 1600 50  0000 L CNN
F 1 "1uF" H 1050 1400 50  0000 L CNN
F 2 "" H 1238 1350 50  0001 C CNN
F 3 "" H 1200 1500 50  0001 C CNN
	1    1200 1500
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 59958B03
P 800 1300
F 0 "C1" H 825 1400 50  0000 L CNN
F 1 "10uF 10V" H 600 1200 50  0000 L CNN
F 2 "" H 838 1150 50  0001 C CNN
F 3 "" H 800 1300 50  0001 C CNN
	1    800  1300
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 59958B3A
P 3000 1300
F 0 "C2" H 3025 1400 50  0000 L CNN
F 1 "10uF 10V" H 2800 1200 50  0000 L CNN
F 2 "" H 3038 1150 50  0001 C CNN
F 3 "" H 3000 1300 50  0001 C CNN
	1    3000 1300
	1    0    0    -1  
$EndComp
$Comp
L 3.3V #PWR11
U 1 1 59959291
P 3000 950
F 0 "#PWR11" H 3000 800 50  0001 C CNN
F 1 "3.3V" H 3000 1100 50  0000 C CNN
F 2 "" H 3000 950 50  0001 C CNN
F 3 "" H 3000 950 50  0001 C CNN
	1    3000 950 
	1    0    0    -1  
$EndComp
$Comp
L 3.3V #PWR10
U 1 1 599592B2
P 2600 950
F 0 "#PWR10" H 2600 800 50  0001 C CNN
F 1 "3.3V" H 2600 1100 50  0000 C CNN
F 2 "" H 2600 950 50  0001 C CNN
F 3 "" H 2600 950 50  0001 C CNN
	1    2600 950 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR24
U 1 1 599597CE
P 3000 1750
F 0 "#PWR24" H 3000 1500 50  0001 C CNN
F 1 "GND" H 3000 1600 50  0000 C CNN
F 2 "" H 3000 1750 50  0001 C CNN
F 3 "" H 3000 1750 50  0001 C CNN
	1    3000 1750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR23
U 1 1 59959861
P 1900 1750
F 0 "#PWR23" H 1900 1500 50  0001 C CNN
F 1 "GND" H 1900 1600 50  0000 C CNN
F 2 "" H 1900 1750 50  0001 C CNN
F 3 "" H 1900 1750 50  0001 C CNN
	1    1900 1750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR22
U 1 1 5995A4E0
P 800 1750
F 0 "#PWR22" H 800 1500 50  0001 C CNN
F 1 "GND" H 800 1600 50  0000 C CNN
F 2 "" H 800 1750 50  0001 C CNN
F 3 "" H 800 1750 50  0001 C CNN
	1    800  1750
	1    0    0    -1  
$EndComp
$Comp
L MCP73831 U2
U 1 1 5995ADC8
P 6100 1250
F 0 "U2" H 5850 1500 50  0000 C CNN
F 1 "MCP73831" H 6000 1000 50  0000 C CNN
F 2 "" H 6100 1250 98  0001 C CNN
F 3 "" H 6100 1250 98  0001 C CNN
	1    6100 1250
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 5995AED6
P 5350 1450
F 0 "R3" V 5250 1400 50  0000 C CNN
F 1 "330" V 5450 1450 50  0000 C CNN
F 2 "" V 5280 1450 50  0001 C CNN
F 3 "" H 5350 1450 50  0001 C CNN
	1    5350 1450
	0    1    1    0   
$EndComp
$Comp
L R R4
U 1 1 5995B2E0
P 6750 1450
F 0 "R4" H 6850 1600 50  0000 C CNN
F 1 "2k" H 6850 1350 50  0000 C CNN
F 2 "" V 6680 1450 50  0001 C CNN
F 3 "" H 6750 1450 50  0001 C CNN
	1    6750 1450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR15
U 1 1 5995B360
P 6750 1700
F 0 "#PWR15" H 6750 1450 50  0001 C CNN
F 1 "GND" H 6750 1550 50  0000 C CNN
F 2 "" H 6750 1700 50  0001 C CNN
F 3 "" H 6750 1700 50  0001 C CNN
	1    6750 1700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR14
U 1 1 5995B0F7
P 6500 1700
F 0 "#PWR14" H 6500 1450 50  0001 C CNN
F 1 "GND" H 6500 1550 50  0000 C CNN
F 2 "" H 6500 1700 50  0001 C CNN
F 3 "" H 6500 1700 50  0001 C CNN
	1    6500 1700
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 5995B465
P 7000 1450
F 0 "C4" H 7025 1550 50  0000 L CNN
F 1 "4.7uF" H 7000 1350 50  0000 L CNN
F 2 "" H 7038 1300 50  0001 C CNN
F 3 "" H 7000 1450 50  0001 C CNN
	1    7000 1450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR16
U 1 1 5995B77F
P 7000 1700
F 0 "#PWR16" H 7000 1450 50  0001 C CNN
F 1 "GND" H 7000 1550 50  0000 C CNN
F 2 "" H 7000 1700 50  0001 C CNN
F 3 "" H 7000 1700 50  0001 C CNN
	1    7000 1700
	1    0    0    -1  
$EndComp
$Comp
L LED D2
U 1 1 5995BA46
P 5050 1300
F 0 "D2" H 5050 1400 50  0000 C CNN
F 1 "LED" H 5050 1200 50  0000 C CNN
F 2 "" H 5050 1300 50  0001 C CNN
F 3 "" H 5050 1300 50  0001 C CNN
	1    5050 1300
	0    1    -1   0   
$EndComp
$Comp
L C C3
U 1 1 5995BE66
P 4800 1450
F 0 "C3" H 4825 1550 50  0000 L CNN
F 1 "4.7uF" H 4800 1350 50  0000 L CNN
F 2 "" H 4838 1300 50  0001 C CNN
F 3 "" H 4800 1450 50  0001 C CNN
	1    4800 1450
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR13
U 1 1 5995BFC7
P 4800 1700
F 0 "#PWR13" H 4800 1450 50  0001 C CNN
F 1 "GND" H 4800 1550 50  0000 C CNN
F 2 "" H 4800 1700 50  0001 C CNN
F 3 "" H 4800 1700 50  0001 C CNN
	1    4800 1700
	1    0    0    -1  
$EndComp
$Comp
L μUSB J3
U 1 1 5995CAD4
P 3850 1250
F 0 "J3" H 3550 1600 50  0000 L CNN
F 1 "μUSB" H 3550 900 50  0000 L CNN
F 2 "" H 3850 1250 98  0001 C CNN
F 3 "" H 3850 1250 98  0001 C CNN
	1    3850 1250
	1    0    0    -1  
$EndComp
$Comp
L Polyfuse F1
U 1 1 5995C789
P 4500 1050
F 0 "F1" V 4400 1050 50  0000 C CNN
F 1 "Polyfuse" V 4600 1100 50  0001 C CNN
F 2 "" H 4550 850 50  0001 L CNN
F 3 "" H 4500 1050 50  0001 C CNN
	1    4500 1050
	0    1    1    0   
$EndComp
$Comp
L GND #PWR12
U 1 1 5995CD47
P 4150 1700
F 0 "#PWR12" H 4150 1450 50  0001 C CNN
F 1 "GND" H 4150 1550 50  0000 C CNN
F 2 "" H 4150 1700 50  0001 C CNN
F 3 "" H 4150 1700 50  0001 C CNN
	1    4150 1700
	1    0    0    -1  
$EndComp
Text Label 4050 1150 0    60   ~ 0
USBD+
Text Label 4050 1250 0    60   ~ 0
USBD-
$Comp
L CONN_01X02 J1
U 1 1 5995DBD0
P 7850 900
F 0 "J1" H 7850 1050 50  0000 C CNN
F 1 "SW" V 7950 850 50  0000 L CNN
F 2 "" H 7850 900 50  0001 C CNN
F 3 "" H 7850 900 50  0001 C CNN
	1    7850 900 
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 J2
U 1 1 5995DF78
P 7850 1200
F 0 "J2" H 7850 1050 50  0000 C CNN
F 1 "BATT" V 7950 1100 50  0000 L CNN
F 2 "" H 7850 1200 50  0001 C CNN
F 3 "" H 7850 1200 50  0001 C CNN
	1    7850 1200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR17
U 1 1 5995E10F
P 7550 1700
F 0 "#PWR17" H 7550 1450 50  0001 C CNN
F 1 "GND" H 7550 1550 50  0000 C CNN
F 2 "" H 7550 1700 50  0001 C CNN
F 3 "" H 7550 1700 50  0001 C CNN
	1    7550 1700
	1    0    0    -1  
$EndComp
Text Notes 3550 2150 0    100  ~ 0
BAT CHARGER
$Comp
L VBAT #PWR25
U 1 1 5996A946
P 850 2550
F 0 "#PWR25" H 850 2400 50  0001 C CNN
F 1 "VBAT" H 850 2690 50  0000 C CNN
F 2 "" H 850 2550 50  0001 C CNN
F 3 "" H 850 2550 50  0001 C CNN
	1    850  2550
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 5996AA94
P 1050 3400
F 0 "C10" H 1075 3500 50  0000 L CNN
F 1 "0.1uF" H 1000 3300 50  0000 L CNN
F 2 "" H 1088 3250 50  0001 C CNN
F 3 "" H 1050 3400 50  0001 C CNN
	1    1050 3400
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 5996AD14
P 850 3250
F 0 "C8" H 875 3350 50  0000 L CNN
F 1 "2.2uF" H 980 3150 50  0000 C CNN
F 2 "" H 888 3100 50  0001 C CNN
F 3 "" H 850 3250 50  0001 C CNN
	1    850  3250
	-1   0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 5996ADD0
P 2700 3350
F 0 "R9" V 2800 3200 50  0000 C CNN
F 1 "620" V 2800 3400 50  0000 C CNN
F 2 "" V 2630 3350 50  0001 C CNN
F 3 "" H 2700 3350 50  0001 C CNN
	1    2700 3350
	0    1    1    0   
$EndComp
$Comp
L MIC2253-06YML-TR U5
U 1 1 5996CB15
P 1850 3250
F 0 "U5" H 1450 2750 50  0000 L BNN
F 1 "MIC2253-06YML-TR" H 1450 3700 50  0000 L BNN
F 2 "" H 750 3800 50  0001 L BNN
F 3 "" H 750 3550 50  0001 L BNN
	1    1850 3250
	1    0    0    -1  
$EndComp
$Comp
L C C11
U 1 1 5996CC43
P 2950 3700
F 0 "C11" H 2975 3800 50  0000 L CNN
F 1 "10nF" H 3150 3600 50  0000 R CNN
F 2 "" H 2988 3550 50  0001 C CNN
F 3 "" H 2950 3700 50  0001 C CNN
	1    2950 3700
	-1   0    0    -1  
$EndComp
$Comp
L R R10
U 1 1 5996CEA1
P 3250 3700
F 0 "R10" V 3150 3650 50  0000 C CNN
F 1 "10k" V 3350 3700 50  0000 C CNN
F 2 "" V 3180 3700 50  0001 C CNN
F 3 "" H 3250 3700 50  0001 C CNN
	1    3250 3700
	1    0    0    -1  
$EndComp
$Comp
L L L1
U 1 1 5996D302
P 1450 2650
F 0 "L1" V 1400 2550 50  0000 C CNN
F 1 "MPI4040R3-2R2-R" V 1525 2650 50  0000 C CNN
F 2 "" H 1450 2650 50  0001 C CNN
F 3 "" H 1450 2650 50  0001 C CNN
	1    1450 2650
	0    1    -1   0   
$EndComp
$Comp
L D_Schottky D3
U 1 1 5996D5DD
P 2800 2650
F 0 "D3" H 2800 2750 50  0000 L CNN
F 1 "BA340A" H 2600 2550 50  0000 L CNN
F 2 "" H 2800 2650 50  0001 C CNN
F 3 "" H 2800 2650 50  0001 C CNN
	1    2800 2650
	-1   0    0    1   
$EndComp
$Comp
L R R8
U 1 1 5996DCE2
P 3250 2900
F 0 "R8" V 3150 2850 50  0000 C CNN
F 1 "30.9k" V 3350 2900 50  0000 C CNN
F 2 "" V 3180 2900 50  0001 C CNN
F 3 "" H 3250 2900 50  0001 C CNN
	1    3250 2900
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 5996E37E
P 3550 2900
F 0 "C7" H 3575 3000 50  0000 L CNN
F 1 "100pF" H 3570 2800 50  0000 L CNN
F 2 "" H 3588 2750 50  0001 C CNN
F 3 "" H 3550 2900 50  0001 C CNN
	1    3550 2900
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 5996E59B
P 3850 3250
F 0 "C9" H 3875 3350 50  0000 L CNN
F 1 "22uF" H 3950 3150 50  0000 C CNN
F 2 "" H 3888 3100 50  0001 C CNN
F 3 "" H 3850 3250 50  0001 C CNN
	1    3850 3250
	1    0    0    -1  
$EndComp
$Comp
L 5V #PWR26
U 1 1 5996EAF1
P 3850 2550
F 0 "#PWR26" H 3850 2400 50  0001 C CNN
F 1 "5V" H 3850 2690 50  0000 C CNN
F 2 "" H 3850 2550 50  0001 C CNN
F 3 "" H 3850 2550 50  0001 C CNN
	1    3850 2550
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR33
U 1 1 5996EFCF
P 1250 3950
F 0 "#PWR33" H 1250 3700 50  0001 C CNN
F 1 "GNDA" H 1250 3800 50  0000 C CNN
F 2 "" H 1250 3950 50  0001 C CNN
F 3 "" H 1250 3950 50  0001 C CNN
	1    1250 3950
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR35
U 1 1 5996F17B
P 2950 3950
F 0 "#PWR35" H 2950 3700 50  0001 C CNN
F 1 "GNDA" H 2950 3800 50  0000 C CNN
F 2 "" H 2950 3950 50  0001 C CNN
F 3 "" H 2950 3950 50  0001 C CNN
	1    2950 3950
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR36
U 1 1 5996F1D7
P 3250 3950
F 0 "#PWR36" H 3250 3700 50  0001 C CNN
F 1 "GNDA" H 3250 3800 50  0000 C CNN
F 2 "" H 3250 3950 50  0001 C CNN
F 3 "" H 3250 3950 50  0001 C CNN
	1    3250 3950
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR34
U 1 1 5996F2F5
P 1750 3950
F 0 "#PWR34" H 1750 3750 50  0001 C CNN
F 1 "GNDPWR" H 1750 3820 50  0000 C CNN
F 2 "" H 1750 3900 50  0001 C CNN
F 3 "" H 1750 3900 50  0001 C CNN
	1    1750 3950
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR32
U 1 1 5996F38E
P 850 3950
F 0 "#PWR32" H 850 3750 50  0001 C CNN
F 1 "GNDPWR" H 850 3820 50  0000 C CNN
F 2 "" H 850 3900 50  0001 C CNN
F 3 "" H 850 3900 50  0001 C CNN
	1    850  3950
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR37
U 1 1 5996F405
P 3850 3950
F 0 "#PWR37" H 3850 3750 50  0001 C CNN
F 1 "GNDPWR" H 3850 3820 50  0000 C CNN
F 2 "" H 3850 3900 50  0001 C CNN
F 3 "" H 3850 3900 50  0001 C CNN
	1    3850 3950
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR40
U 1 1 5996F779
P 2400 4300
F 0 "#PWR40" H 2400 4100 50  0001 C CNN
F 1 "GNDPWR" H 2400 4170 50  0000 C CNN
F 2 "" H 2400 4250 50  0001 C CNN
F 3 "" H 2400 4250 50  0001 C CNN
	1    2400 4300
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR39
U 1 1 5996F7D5
P 2200 4300
F 0 "#PWR39" H 2200 4050 50  0001 C CNN
F 1 "GNDA" H 2200 4150 50  0000 C CNN
F 2 "" H 2200 4300 50  0001 C CNN
F 3 "" H 2200 4300 50  0001 C CNN
	1    2200 4300
	0    1    1    0   
$EndComp
$Comp
L GND #PWR41
U 1 1 5996F89D
P 2600 4300
F 0 "#PWR41" H 2600 4050 50  0001 C CNN
F 1 "GND" H 2600 4150 50  0000 C CNN
F 2 "" H 2600 4300 50  0001 C CNN
F 3 "" H 2600 4300 50  0001 C CNN
	1    2600 4300
	0    -1   -1   0   
$EndComp
$Comp
L GND-CONNECT G1
U 1 1 5996FD52
P 2400 4200
F 0 "G1" H 2200 4350 60  0001 C CNN
F 1 "GND-CONNECT" H 2450 4700 60  0001 C CNN
F 2 "" H 2400 4200 60  0001 C CNN
F 3 "" H 2400 4200 60  0001 C CNN
	1    2400 4200
	1    0    0    -1  
$EndComp
Text Notes 700  4450 0    100  ~ 0
BOOST\n
$Comp
L DRV8837 U4
U 1 1 5996C453
P 5150 2800
F 0 "U4" H 4950 3050 60  0000 C CNN
F 1 "DRV8837" H 5100 2550 60  0000 C CNN
F 2 "" H 5150 2800 60  0001 C CNN
F 3 "" H 5150 2800 60  0001 C CNN
	1    5150 2800
	1    0    0    -1  
$EndComp
$Comp
L 5V #PWR28
U 1 1 5996CBD5
P 5700 2550
F 0 "#PWR28" H 5700 2400 50  0001 C CNN
F 1 "5V" H 5700 2690 50  0000 C CNN
F 2 "" H 5700 2550 50  0001 C CNN
F 3 "" H 5700 2550 50  0001 C CNN
	1    5700 2550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR29
U 1 1 5996D4E5
P 5700 3050
F 0 "#PWR29" H 5700 2800 50  0001 C CNN
F 1 "GND" H 5700 2900 50  0000 C CNN
F 2 "" H 5700 3050 50  0001 C CNN
F 3 "" H 5700 3050 50  0001 C CNN
	1    5700 3050
	1    0    0    -1  
$EndComp
Text Label 4400 2750 0    60   ~ 0
MOT1A
Text Label 4400 2850 0    60   ~ 0
MOT1B
Text Label 4400 2950 0    60   ~ 0
MOTEN
$Comp
L DRV8837 U6
U 1 1 5996DB50
P 5150 3750
F 0 "U6" H 4950 4000 60  0000 C CNN
F 1 "DRV8837" H 5100 3500 60  0000 C CNN
F 2 "" H 5150 3750 60  0001 C CNN
F 3 "" H 5150 3750 60  0001 C CNN
	1    5150 3750
	1    0    0    -1  
$EndComp
$Comp
L 5V #PWR31
U 1 1 5996DB5C
P 5700 3500
F 0 "#PWR31" H 5700 3350 50  0001 C CNN
F 1 "5V" H 5700 3640 50  0000 C CNN
F 2 "" H 5700 3500 50  0001 C CNN
F 3 "" H 5700 3500 50  0001 C CNN
	1    5700 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR38
U 1 1 5996DB66
P 5700 4000
F 0 "#PWR38" H 5700 3750 50  0001 C CNN
F 1 "GND" H 5700 3850 50  0000 C CNN
F 2 "" H 5700 4000 50  0001 C CNN
F 3 "" H 5700 4000 50  0001 C CNN
	1    5700 4000
	1    0    0    -1  
$EndComp
Text Label 4400 3700 0    60   ~ 0
MOT2A
Text Label 4400 3800 0    60   ~ 0
MOT2B
Text Label 4400 3900 0    60   ~ 0
MOTEN
Text Notes 4400 4450 0    100  ~ 0
MOTOR DRIVERS
$Comp
L CONN_01X02_FEMALE J4
U 1 1 5996E25F
P 5900 2800
F 0 "J4" H 5900 3000 50  0000 C CNN
F 1 "CONN_01X02_FEMALE" H 5975 2600 50  0001 C CNN
F 2 "" H 5900 2900 50  0001 C CNN
F 3 "" H 5900 2900 50  0001 C CNN
	1    5900 2800
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02_FEMALE J5
U 1 1 5996E9A1
P 5900 3750
F 0 "J5" H 5900 3950 50  0000 C CNN
F 1 "CONN_01X02_FEMALE" H 5975 3550 50  0001 C CNN
F 2 "" H 5900 3850 50  0001 C CNN
F 3 "" H 5900 3850 50  0001 C CNN
	1    5900 3750
	1    0    0    -1  
$EndComp
$Comp
L LMC7101AIM5 U3
U 1 1 5996FD92
P 9500 1400
F 0 "U3" H 9400 1800 60  0000 C CNN
F 1 "LMC7101AIM5" H 10000 1400 60  0000 C CNN
F 2 "" H 9500 1400 60  0001 C CNN
F 3 "" H 9500 1400 60  0001 C CNN
	1    9500 1400
	1    0    0    -1  
$EndComp
$Comp
L 5V #PWR6
U 1 1 59970006
P 9650 750
F 0 "#PWR6" H 9650 600 50  0001 C CNN
F 1 "5V" H 9650 890 50  0000 C CNN
F 2 "" H 9650 750 50  0001 C CNN
F 3 "" H 9650 750 50  0001 C CNN
	1    9650 750 
	1    0    0    -1  
$EndComp
$Comp
L 3.3V #PWR5
U 1 1 599701F1
P 8950 750
F 0 "#PWR5" H 8950 600 50  0001 C CNN
F 1 "3.3V" H 8950 900 50  0000 C CNN
F 2 "" H 8950 750 50  0001 C CNN
F 3 "" H 8950 750 50  0001 C CNN
	1    8950 750 
	1    0    0    -1  
$EndComp
$Comp
L VBAT #PWR4
U 1 1 5997063C
P 8650 750
F 0 "#PWR4" H 8650 600 50  0001 C CNN
F 1 "VBAT" H 8650 890 50  0000 C CNN
F 2 "" H 8650 750 50  0001 C CNN
F 3 "" H 8650 750 50  0001 C CNN
	1    8650 750 
	1    0    0    -1  
$EndComp
$Comp
L VBAT #PWR3
U 1 1 59970E27
P 7550 750
F 0 "#PWR3" H 7550 600 50  0001 C CNN
F 1 "VBAT" H 7550 890 50  0000 C CNN
F 2 "" H 7550 750 50  0001 C CNN
F 3 "" H 7550 750 50  0001 C CNN
	1    7550 750 
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 59971513
P 8650 1500
F 0 "R5" V 8550 1400 50  0000 C CNN
F 1 "4.7k" V 8550 1600 50  0000 C CNN
F 2 "" V 8580 1500 50  0001 C CNN
F 3 "" H 8650 1500 50  0001 C CNN
	1    8650 1500
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 59972022
P 8650 1000
F 0 "R1" V 8550 900 50  0000 C CNN
F 1 "1.5k" V 8550 1100 50  0000 C CNN
F 2 "" V 8580 1000 50  0001 C CNN
F 3 "" H 8650 1000 50  0001 C CNN
	1    8650 1000
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 599720AF
P 8950 1000
F 0 "R2" V 8850 900 50  0000 C CNN
F 1 "1.5k" V 8850 1100 50  0000 C CNN
F 2 "" V 8880 1000 50  0001 C CNN
F 3 "" H 8950 1000 50  0001 C CNN
	1    8950 1000
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 5997215C
P 8950 1500
F 0 "R6" V 8850 1400 50  0000 C CNN
F 1 "4.7k" V 8850 1600 50  0000 C CNN
F 2 "" V 8880 1500 50  0001 C CNN
F 3 "" H 8950 1500 50  0001 C CNN
	1    8950 1500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR18
U 1 1 59972EE3
P 8650 1700
F 0 "#PWR18" H 8650 1450 50  0001 C CNN
F 1 "GND" H 8650 1550 50  0000 C CNN
F 2 "" H 8650 1700 50  0001 C CNN
F 3 "" H 8650 1700 50  0001 C CNN
	1    8650 1700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR19
U 1 1 59972FA2
P 8950 1700
F 0 "#PWR19" H 8950 1450 50  0001 C CNN
F 1 "GND" H 8950 1550 50  0000 C CNN
F 2 "" H 8950 1700 50  0001 C CNN
F 3 "" H 8950 1700 50  0001 C CNN
	1    8950 1700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR20
U 1 1 59973019
P 9650 1700
F 0 "#PWR20" H 9650 1450 50  0001 C CNN
F 1 "GND" H 9650 1550 50  0000 C CNN
F 2 "" H 9650 1700 50  0001 C CNN
F 3 "" H 9650 1700 50  0001 C CNN
	1    9650 1700
	1    0    0    -1  
$EndComp
Text Notes 8350 2150 0    100  ~ 0
BAT COMPARATOR
$Comp
L R R7
U 1 1 59975674
P 10450 1500
F 0 "R7" V 10350 1400 50  0000 C CNN
F 1 "10k" V 10350 1600 50  0000 C CNN
F 2 "" V 10380 1500 50  0001 C CNN
F 3 "" H 10450 1500 50  0001 C CNN
	1    10450 1500
	1    0    0    -1  
$EndComp
$Comp
L 3.3V #PWR7
U 1 1 59975852
P 10450 750
F 0 "#PWR7" H 10450 600 50  0001 C CNN
F 1 "3.3V" H 10450 900 50  0000 C CNN
F 2 "" H 10450 750 50  0001 C CNN
F 3 "" H 10450 750 50  0001 C CNN
	1    10450 750 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR21
U 1 1 599763C1
P 10450 1700
F 0 "#PWR21" H 10450 1450 50  0001 C CNN
F 1 "GND" H 10450 1550 50  0000 C CNN
F 2 "" H 10450 1700 50  0001 C CNN
F 3 "" H 10450 1700 50  0001 C CNN
	1    10450 1700
	1    0    0    -1  
$EndComp
$Comp
L BSS84 Q1
U 1 1 59975499
P 10350 1050
F 0 "Q1" H 10550 1100 50  0000 L CNN
F 1 "BSS84" H 10550 1000 50  0000 L CNN
F 2 "" H 10550 1150 50  0001 C CNN
F 3 "" H 10350 1050 50  0001 C CNN
	1    10350 1050
	1    0    0    1   
$EndComp
Text Label 10600 1300 0    60   ~ 0
MOTEN
$Comp
L 5V #PWR?
U 1 1 5996F70C
P 1200 950
F 0 "#PWR?" H 1200 800 50  0001 C CNN
F 1 "5V" H 1200 1090 50  0000 C CNN
F 2 "" H 1200 950 50  0001 C CNN
F 3 "" H 1200 950 50  0001 C CNN
	1    1200 950 
	1    0    0    -1  
$EndComp
$Comp
L 5V #PWR?
U 1 1 5996F786
P 800 950
F 0 "#PWR?" H 800 800 50  0001 C CNN
F 1 "5V" H 800 1090 50  0000 C CNN
F 2 "" H 800 950 50  0001 C CNN
F 3 "" H 800 950 50  0001 C CNN
	1    800  950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 1050 1200 1050
Wire Wire Line
	1200 1250 1300 1250
Connection ~ 1200 1050
Connection ~ 1200 1250
Wire Wire Line
	2600 950  2600 1350
Wire Wire Line
	2600 1050 2500 1050
Connection ~ 2600 1050
Wire Wire Line
	1200 950  1200 1350
Wire Wire Line
	800  950  800  1150
Wire Wire Line
	3000 950  3000 1150
Wire Wire Line
	1200 1650 2600 1650
Connection ~ 1900 1650
Wire Wire Line
	1900 1750 1900 1650
Wire Notes Line
	600  2250 11150 2250
Wire Wire Line
	5700 1350 5500 1350
Wire Wire Line
	5500 1350 5500 1450
Wire Wire Line
	6500 1350 6500 1700
Wire Wire Line
	6750 1700 6750 1600
Wire Wire Line
	6500 1250 6750 1250
Wire Wire Line
	6750 1250 6750 1300
Wire Wire Line
	7000 1600 7000 1700
Wire Wire Line
	5050 1450 5200 1450
Wire Wire Line
	5700 1050 5700 1150
Connection ~ 5050 1050
Wire Wire Line
	4800 1700 4800 1600
Wire Wire Line
	4800 1300 4800 1050
Connection ~ 4800 1050
Wire Wire Line
	4350 1050 4050 1050
Wire Wire Line
	4350 1150 4050 1150
Wire Wire Line
	4350 1250 4050 1250
Wire Wire Line
	4050 1450 4150 1450
Wire Wire Line
	4150 1450 4150 1700
Wire Wire Line
	4650 1050 5700 1050
Connection ~ 7000 1150
Wire Notes Line
	8200 550  8200 2250
Wire Wire Line
	1050 3250 1250 3250
Wire Wire Line
	1250 3350 1250 3950
Wire Wire Line
	1250 3550 1050 3550
Connection ~ 1250 3550
Wire Wire Line
	850  3400 850  3950
Wire Wire Line
	1250 2950 850  2950
Connection ~ 850  2950
Wire Wire Line
	1250 3050 1150 3050
Wire Wire Line
	1150 3050 1150 2950
Connection ~ 1150 2950
Wire Wire Line
	2450 3350 2550 3350
Wire Wire Line
	2850 3350 2950 3350
Wire Wire Line
	2950 3350 2950 3550
Wire Wire Line
	3250 3050 3250 3550
Wire Wire Line
	2450 3250 3550 3250
Connection ~ 850  2650
Wire Wire Line
	1600 2650 2650 2650
Wire Wire Line
	850  2550 850  3100
Wire Wire Line
	1300 2650 850  2650
Wire Wire Line
	2450 2950 2550 2950
Wire Wire Line
	2550 2650 2550 3050
Wire Wire Line
	2550 3050 2450 3050
Connection ~ 2550 2950
Connection ~ 2550 2650
Wire Wire Line
	2450 3150 3050 3150
Wire Wire Line
	3050 3150 3050 2650
Wire Wire Line
	2950 2650 3850 2650
Wire Wire Line
	3250 2650 3250 2750
Connection ~ 3050 2650
Connection ~ 3250 3250
Wire Wire Line
	3550 2650 3550 2750
Connection ~ 3250 2650
Wire Wire Line
	3550 3250 3550 3050
Wire Wire Line
	3850 2550 3850 3100
Connection ~ 3550 2650
Connection ~ 3850 2650
Wire Wire Line
	3250 3950 3250 3850
Wire Wire Line
	2950 3850 2950 3950
Wire Wire Line
	3850 3950 3850 3400
Wire Wire Line
	1750 3850 1750 3950
Wire Wire Line
	1750 3900 1950 3900
Wire Wire Line
	1950 3900 1950 3850
Connection ~ 1750 3900
Wire Wire Line
	2300 4200 2300 4300
Wire Wire Line
	2300 4300 2200 4300
Wire Wire Line
	2400 4200 2400 4300
Wire Wire Line
	2500 4200 2500 4300
Wire Wire Line
	2500 4300 2600 4300
Wire Notes Line
	3400 550  3400 2250
Wire Notes Line
	550  4550 6200 4550
Wire Notes Line
	4250 4550 4250 2250
Wire Wire Line
	800  1450 800  1750
Wire Wire Line
	3000 1450 3000 1750
Wire Wire Line
	4600 2550 4600 2650
Wire Wire Line
	4600 2650 4700 2650
Wire Wire Line
	5700 2550 5700 2650
Wire Wire Line
	5700 2650 5600 2650
Wire Wire Line
	5700 3050 5700 2950
Wire Wire Line
	5700 2950 5600 2950
Wire Wire Line
	4700 2750 4400 2750
Wire Wire Line
	4400 2850 4700 2850
Wire Wire Line
	4400 2950 4700 2950
Wire Wire Line
	4600 3500 4600 3600
Wire Wire Line
	4600 3600 4700 3600
Wire Wire Line
	5700 3500 5700 3600
Wire Wire Line
	5700 3600 5600 3600
Wire Wire Line
	5700 4000 5700 3900
Wire Wire Line
	5700 3900 5600 3900
Wire Wire Line
	4700 3700 4400 3700
Wire Wire Line
	4400 3800 4700 3800
Wire Wire Line
	4400 3900 4700 3900
Wire Wire Line
	5800 2900 5800 2850
Wire Wire Line
	5800 2850 5600 2850
Wire Wire Line
	5800 2700 5800 2750
Wire Wire Line
	5800 2750 5600 2750
Wire Wire Line
	5800 3650 5800 3700
Wire Wire Line
	5800 3700 5600 3700
Wire Wire Line
	5800 3850 5800 3800
Wire Wire Line
	5800 3800 5600 3800
Wire Notes Line
	6200 4550 6200 2250
Wire Wire Line
	8650 1150 8650 1350
Wire Wire Line
	8950 1150 8950 1350
Connection ~ 8650 1200
Connection ~ 8950 1300
Wire Wire Line
	8650 750  8650 850 
Wire Wire Line
	8950 750  8950 850 
Wire Wire Line
	9650 1000 9650 750 
Wire Wire Line
	9250 1200 9250 1150
Wire Wire Line
	9250 1300 9250 1350
Wire Wire Line
	10450 850  10450 750 
Wire Wire Line
	10450 1350 10450 1250
Wire Wire Line
	9650 1700 9650 1500
Wire Wire Line
	8950 1650 8950 1700
Wire Wire Line
	8650 1700 8650 1650
Wire Wire Line
	10450 1700 10450 1650
Wire Wire Line
	10450 1300 10900 1300
Connection ~ 10450 1300
Wire Wire Line
	10050 1250 10150 1250
Wire Wire Line
	10150 1250 10150 1050
Wire Wire Line
	9250 1200 8650 1200
Wire Wire Line
	8950 1300 9250 1300
Wire Wire Line
	5050 1150 5050 1050
Wire Wire Line
	7000 1150 7000 1300
Wire Wire Line
	7650 1150 6500 1150
Connection ~ 7550 1150
Wire Wire Line
	7550 750  7550 850 
Wire Wire Line
	7550 850  7650 850 
Wire Wire Line
	7650 950  7550 950 
Wire Wire Line
	7550 950  7550 1150
Wire Wire Line
	7550 1700 7550 1250
Wire Wire Line
	7550 1250 7650 1250
$Comp
L 3.3V #PWR?
U 1 1 59971AD0
P 4600 2550
F 0 "#PWR?" H 4600 2400 50  0001 C CNN
F 1 "3.3V" H 4600 2700 50  0000 C CNN
F 2 "" H 4600 2550 50  0001 C CNN
F 3 "" H 4600 2550 50  0001 C CNN
	1    4600 2550
	1    0    0    -1  
$EndComp
$Comp
L 3.3V #PWR?
U 1 1 59971B4A
P 4600 3500
F 0 "#PWR?" H 4600 3350 50  0001 C CNN
F 1 "3.3V" H 4600 3650 50  0000 C CNN
F 2 "" H 4600 3500 50  0001 C CNN
F 3 "" H 4600 3500 50  0001 C CNN
	1    4600 3500
	1    0    0    -1  
$EndComp
$EndSCHEMATC
