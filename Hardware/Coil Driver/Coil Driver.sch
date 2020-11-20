EESchema Schematic File Version 4
EELAYER 30 0
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
L power:GND #PWR?
U 1 1 5FB7A8AB
P 1250 1900
F 0 "#PWR?" H 1250 1650 50  0001 C CNN
F 1 "GND" H 1255 1727 50  0000 C CNN
F 2 "" H 1250 1900 50  0001 C CNN
F 3 "" H 1250 1900 50  0001 C CNN
	1    1250 1900
	1    0    0    -1  
$EndComp
$Comp
L Connector:Barrel_Jack_Switch J?
U 1 1 5FB7AF2A
P 2050 1650
F 0 "J?" H 2107 1967 50  0000 C CNN
F 1 "Barrel_Jack_Switch" H 2107 1876 50  0000 C CNN
F 2 "" H 2100 1610 50  0001 C CNN
F 3 "~" H 2100 1610 50  0001 C CNN
	1    2050 1650
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 5FB7B79A
P 1250 1600
F 0 "#FLG?" H 1250 1675 50  0001 C CNN
F 1 "PWR_FLAG" H 1250 1773 50  0000 C CNN
F 2 "" H 1250 1600 50  0001 C CNN
F 3 "~" H 1250 1600 50  0001 C CNN
	1    1250 1600
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 5FB7BD03
P 1500 1900
F 0 "#FLG?" H 1500 1975 50  0001 C CNN
F 1 "PWR_FLAG" H 1500 2073 50  0000 C CNN
F 2 "" H 1500 1900 50  0001 C CNN
F 3 "~" H 1500 1900 50  0001 C CNN
	1    1500 1900
	-1   0    0    1   
$EndComp
Wire Wire Line
	1250 1600 1250 1900
Wire Wire Line
	1500 1900 1500 1600
$Comp
L power:VDD #PWR?
U 1 1 5FB7A1B5
P 1500 1600
F 0 "#PWR?" H 1500 1450 50  0001 C CNN
F 1 "VDD" H 1515 1773 50  0000 C CNN
F 2 "" H 1500 1600 50  0001 C CNN
F 3 "" H 1500 1600 50  0001 C CNN
	1    1500 1600
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR?
U 1 1 5FB7F671
P 2500 1400
F 0 "#PWR?" H 2500 1250 50  0001 C CNN
F 1 "VDD" H 2515 1573 50  0000 C CNN
F 2 "" H 2500 1400 50  0001 C CNN
F 3 "" H 2500 1400 50  0001 C CNN
	1    2500 1400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FB7FC02
P 2500 1900
F 0 "#PWR?" H 2500 1650 50  0001 C CNN
F 1 "GND" H 2505 1727 50  0000 C CNN
F 2 "" H 2500 1900 50  0001 C CNN
F 3 "" H 2500 1900 50  0001 C CNN
	1    2500 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 1550 2500 1550
Wire Wire Line
	2500 1550 2500 1500
Wire Wire Line
	2350 1750 2500 1750
Wire Wire Line
	2500 1750 2500 1800
$Comp
L Device:CP C?
U 1 1 5FB80E1E
P 2700 1650
F 0 "C?" H 2818 1696 50  0000 L CNN
F 1 "CP" H 2818 1605 50  0000 L CNN
F 2 "" H 2738 1500 50  0001 C CNN
F 3 "~" H 2700 1650 50  0001 C CNN
	1    2700 1650
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR?
U 1 1 5FB9E408
P 3650 1400
F 0 "#PWR?" H 3650 1250 50  0001 C CNN
F 1 "+12V" H 3665 1573 50  0000 C CNN
F 2 "" H 3650 1400 50  0001 C CNN
F 3 "" H 3650 1400 50  0001 C CNN
	1    3650 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 1500 2700 1500
Connection ~ 2500 1500
Wire Wire Line
	2500 1500 2500 1400
Wire Wire Line
	3650 1500 3650 1400
$Comp
L Device:Polyfuse_Small F?
U 1 1 5FBA1D43
P 3400 1500
F 0 "F?" V 3200 1450 50  0000 C CNN
F 1 "miniSMDC110F/16-2" V 3300 1300 50  0000 C CNN
F 2 "" H 3450 1300 50  0001 L CNN
F 3 "~" H 3400 1500 50  0001 C CNN
	1    3400 1500
	0    1    1    0   
$EndComp
Wire Wire Line
	3500 1500 3650 1500
$Comp
L Device:CP C?
U 1 1 5FBA6C8F
P 3050 1650
F 0 "C?" H 3168 1696 50  0000 L CNN
F 1 "CP" H 3168 1605 50  0000 L CNN
F 2 "" H 3088 1500 50  0001 C CNN
F 3 "~" H 3050 1650 50  0001 C CNN
	1    3050 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 1500 3050 1500
Connection ~ 2700 1500
Connection ~ 3050 1500
Wire Wire Line
	3050 1500 3300 1500
Wire Wire Line
	2500 1800 2700 1800
Connection ~ 2500 1800
Wire Wire Line
	2500 1800 2500 1900
Connection ~ 2700 1800
Wire Wire Line
	2700 1800 3050 1800
$Comp
L MyParts:DGD05473 U?
U 1 1 5FBAB121
P 1850 5500
F 0 "U?" H 1850 6167 50  0000 C CNN
F 1 "DGD05473" H 1850 6076 50  0000 C CNN
F 2 "MyParts:W-DFN3030-10" H 1950 4950 50  0001 C CNN
F 3 "https://www.diodes.com/assets/Datasheets/DGD05473.pdf" H 1850 5500 50  0001 C CNN
	1    1850 5500
	1    0    0    -1  
$EndComp
$Comp
L MyParts:DMN3023L Q?
U 1 1 5FBABB38
P 3750 5200
F 0 "Q?" H 3600 5400 50  0000 L CNN
F 1 "DMN3023L" H 3300 5300 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3950 5125 50  0001 L CIN
F 3 "https://www.diodes.com/assets/Datasheets/DMN3023L.pdf" H 3750 5200 50  0001 L CNN
	1    3750 5200
	1    0    0    -1  
$EndComp
$Comp
L MyParts:B2100A D?
U 1 1 5FBAC6F3
P 4050 5200
F 0 "D?" V 4004 5280 50  0000 L CNN
F 1 "B2100A" V 4095 5280 50  0000 L CNN
F 2 "Diode_SMD:D_SMA" H 4050 5025 50  0001 C CNN
F 3 "https://www.diodes.com/assets/Datasheets/B2100A.pdf" H 4050 5200 50  0001 C CNN
	1    4050 5200
	0    1    1    0   
$EndComp
$Comp
L power:+12V #PWR?
U 1 1 5FBBBA98
P 1000 6950
F 0 "#PWR?" H 1000 6800 50  0001 C CNN
F 1 "+12V" H 1015 7123 50  0000 C CNN
F 2 "" H 1000 6950 50  0001 C CNN
F 3 "" H 1000 6950 50  0001 C CNN
	1    1000 6950
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5FBB990E
P 5900 7100
F 0 "C?" H 5808 7146 50  0000 R CNN
F 1 "10uF" H 5808 7055 50  0000 R CNN
F 2 "" H 5900 7100 50  0001 C CNN
F 3 "~" H 5900 7100 50  0001 C CNN
	1    5900 7100
	-1   0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5FBB9904
P 5550 7100
F 0 "C?" H 5458 7146 50  0000 R CNN
F 1 "10uF" H 5458 7055 50  0000 R CNN
F 2 "" H 5550 7100 50  0001 C CNN
F 3 "~" H 5550 7100 50  0001 C CNN
	1    5550 7100
	-1   0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5FBB98FA
P 4500 7100
F 0 "C?" H 4408 7146 50  0000 R CNN
F 1 "10uF" H 4408 7055 50  0000 R CNN
F 2 "" H 4500 7100 50  0001 C CNN
F 3 "~" H 4500 7100 50  0001 C CNN
	1    4500 7100
	-1   0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5FBB98F0
P 4850 7100
F 0 "C?" H 4758 7146 50  0000 R CNN
F 1 "10uF" H 4758 7055 50  0000 R CNN
F 2 "" H 4850 7100 50  0001 C CNN
F 3 "~" H 4850 7100 50  0001 C CNN
	1    4850 7100
	-1   0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5FBB98E6
P 5200 7100
F 0 "C?" H 5108 7146 50  0000 R CNN
F 1 "10uF" H 5108 7055 50  0000 R CNN
F 2 "" H 5200 7100 50  0001 C CNN
F 3 "~" H 5200 7100 50  0001 C CNN
	1    5200 7100
	-1   0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5FBB6555
P 4150 7100
F 0 "C?" H 4058 7146 50  0000 R CNN
F 1 "10uF" H 4058 7055 50  0000 R CNN
F 2 "" H 4150 7100 50  0001 C CNN
F 3 "~" H 4150 7100 50  0001 C CNN
	1    4150 7100
	-1   0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5FBB654B
P 3800 7100
F 0 "C?" H 3708 7146 50  0000 R CNN
F 1 "10uF" H 3708 7055 50  0000 R CNN
F 2 "" H 3800 7100 50  0001 C CNN
F 3 "~" H 3800 7100 50  0001 C CNN
	1    3800 7100
	-1   0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5FBB6541
P 2750 7100
F 0 "C?" H 2658 7146 50  0000 R CNN
F 1 "10uF" H 2658 7055 50  0000 R CNN
F 2 "" H 2750 7100 50  0001 C CNN
F 3 "~" H 2750 7100 50  0001 C CNN
	1    2750 7100
	-1   0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5FBB6537
P 3100 7100
F 0 "C?" H 3008 7146 50  0000 R CNN
F 1 "10uF" H 3008 7055 50  0000 R CNN
F 2 "" H 3100 7100 50  0001 C CNN
F 3 "~" H 3100 7100 50  0001 C CNN
	1    3100 7100
	-1   0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5FBB652D
P 3450 7100
F 0 "C?" H 3358 7146 50  0000 R CNN
F 1 "10uF" H 3358 7055 50  0000 R CNN
F 2 "" H 3450 7100 50  0001 C CNN
F 3 "~" H 3450 7100 50  0001 C CNN
	1    3450 7100
	-1   0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5FBAFD22
P 2400 7100
F 0 "C?" H 2308 7146 50  0000 R CNN
F 1 "10uF" H 2308 7055 50  0000 R CNN
F 2 "" H 2400 7100 50  0001 C CNN
F 3 "~" H 2400 7100 50  0001 C CNN
	1    2400 7100
	-1   0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5FBAF586
P 2050 7100
F 0 "C?" H 1958 7146 50  0000 R CNN
F 1 "10uF" H 1958 7055 50  0000 R CNN
F 2 "" H 2050 7100 50  0001 C CNN
F 3 "~" H 2050 7100 50  0001 C CNN
	1    2050 7100
	-1   0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5FBAF0D7
P 1000 7100
F 0 "C?" H 908 7146 50  0000 R CNN
F 1 "10uF" H 908 7055 50  0000 R CNN
F 2 "" H 1000 7100 50  0001 C CNN
F 3 "~" H 1000 7100 50  0001 C CNN
	1    1000 7100
	-1   0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5FBAEBA7
P 1350 7100
F 0 "C?" H 1258 7146 50  0000 R CNN
F 1 "10uF" H 1258 7055 50  0000 R CNN
F 2 "" H 1350 7100 50  0001 C CNN
F 3 "~" H 1350 7100 50  0001 C CNN
	1    1350 7100
	-1   0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5FBAE12B
P 1700 7100
F 0 "C?" H 1608 7146 50  0000 R CNN
F 1 "10uF" H 1608 7055 50  0000 R CNN
F 2 "" H 1700 7100 50  0001 C CNN
F 3 "~" H 1700 7100 50  0001 C CNN
	1    1700 7100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1000 6950 1000 7000
Wire Wire Line
	1000 7000 1350 7000
Connection ~ 1000 7000
Connection ~ 1350 7000
Wire Wire Line
	1350 7000 1700 7000
Connection ~ 1700 7000
Wire Wire Line
	1700 7000 2050 7000
Connection ~ 2050 7000
Wire Wire Line
	2050 7000 2400 7000
Connection ~ 2400 7000
Wire Wire Line
	2400 7000 2750 7000
Connection ~ 2750 7000
Wire Wire Line
	2750 7000 3100 7000
Connection ~ 3100 7000
Wire Wire Line
	3100 7000 3450 7000
Connection ~ 3450 7000
Wire Wire Line
	3450 7000 3800 7000
Connection ~ 3800 7000
Wire Wire Line
	3800 7000 4150 7000
Connection ~ 4150 7000
Wire Wire Line
	4150 7000 4500 7000
Connection ~ 4500 7000
Wire Wire Line
	4500 7000 4850 7000
Connection ~ 4850 7000
Wire Wire Line
	4850 7000 5200 7000
Connection ~ 5200 7000
Wire Wire Line
	5200 7000 5550 7000
Connection ~ 5550 7000
Wire Wire Line
	5550 7000 5900 7000
$Comp
L power:GND #PWR?
U 1 1 5FBE348A
P 5900 7250
F 0 "#PWR?" H 5900 7000 50  0001 C CNN
F 1 "GND" H 5905 7077 50  0000 C CNN
F 2 "" H 5900 7250 50  0001 C CNN
F 3 "" H 5900 7250 50  0001 C CNN
	1    5900 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 7200 1350 7200
Wire Wire Line
	5900 7200 5900 7250
Connection ~ 5900 7200
Connection ~ 1350 7200
Wire Wire Line
	1350 7200 1700 7200
Connection ~ 1700 7200
Wire Wire Line
	1700 7200 2050 7200
Connection ~ 2050 7200
Wire Wire Line
	2050 7200 2400 7200
Connection ~ 2400 7200
Wire Wire Line
	2400 7200 2750 7200
Connection ~ 2750 7200
Wire Wire Line
	2750 7200 3100 7200
Connection ~ 3100 7200
Wire Wire Line
	3100 7200 3450 7200
Connection ~ 3450 7200
Wire Wire Line
	3450 7200 3800 7200
Connection ~ 3800 7200
Wire Wire Line
	3800 7200 4150 7200
Connection ~ 4150 7200
Wire Wire Line
	4150 7200 4500 7200
Connection ~ 4500 7200
Wire Wire Line
	4500 7200 4850 7200
Connection ~ 4850 7200
Wire Wire Line
	4850 7200 5200 7200
Connection ~ 5200 7200
Wire Wire Line
	5200 7200 5550 7200
Connection ~ 5550 7200
Wire Wire Line
	5550 7200 5900 7200
Text Notes 950  6650 0    50   ~ 0
Bulk caps for gate drivers (5 each) and h-bridge (5)
$Comp
L power:+12V #PWR?
U 1 1 5FBEBDA4
P 1300 5000
F 0 "#PWR?" H 1300 4850 50  0001 C CNN
F 1 "+12V" H 1315 5173 50  0000 C CNN
F 2 "" H 1300 5000 50  0001 C CNN
F 3 "" H 1300 5000 50  0001 C CNN
	1    1300 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 5500 1100 5500
Wire Wire Line
	1400 5700 1100 5700
$Comp
L power:GND #PWR?
U 1 1 5FBF3CB1
P 1300 6000
F 0 "#PWR?" H 1300 5750 50  0001 C CNN
F 1 "GND" H 1305 5827 50  0000 C CNN
F 2 "" H 1300 6000 50  0001 C CNN
F 3 "" H 1300 6000 50  0001 C CNN
	1    1300 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 6000 1300 5900
Wire Wire Line
	1300 5800 1400 5800
Wire Wire Line
	1400 5900 1300 5900
Connection ~ 1300 5900
Wire Wire Line
	1300 5900 1300 5800
Text Label 1100 5500 0    50   ~ 0
HIN_A
Text Label 1100 5700 0    50   ~ 0
LIN_A
Wire Wire Line
	1300 5000 1300 5100
Wire Wire Line
	1300 5100 1400 5100
$Comp
L Device:R_Small_US R?
U 1 1 5FC0276A
P 950 5450
F 0 "R?" H 882 5404 50  0000 R CNN
F 1 "10k" H 882 5495 50  0000 R CNN
F 2 "" H 950 5450 50  0001 C CNN
F 3 "~" H 950 5450 50  0001 C CNN
	1    950  5450
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FC031FC
P 950 5600
F 0 "#PWR?" H 950 5350 50  0001 C CNN
F 1 "GND" H 955 5427 50  0000 C CNN
F 2 "" H 950 5600 50  0001 C CNN
F 3 "" H 950 5600 50  0001 C CNN
	1    950  5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  5600 950  5550
Wire Wire Line
	950  5300 950  5350
Wire Wire Line
	950  5300 1400 5300
Wire Wire Line
	950  5300 700  5300
Connection ~ 950  5300
$Comp
L Device:R_Small_US R?
U 1 1 5FC0BE86
P 2450 5100
F 0 "R?" V 2245 5100 50  0000 C CNN
F 1 "10" V 2336 5100 50  0000 C CNN
F 2 "" H 2450 5100 50  0001 C CNN
F 3 "~" H 2450 5100 50  0001 C CNN
	1    2450 5100
	0    -1   1    0   
$EndComp
Wire Wire Line
	2300 5100 2350 5100
Wire Wire Line
	2550 5100 2650 5100
$Comp
L MyParts:DMN3023L Q?
U 1 1 5FC16F4C
P 3750 5800
F 0 "Q?" H 3600 6000 50  0000 L CNN
F 1 "DMN3023L" H 3300 5900 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3950 5725 50  0001 L CIN
F 3 "https://www.diodes.com/assets/Datasheets/DMN3023L.pdf" H 3750 5800 50  0001 L CNN
	1    3750 5800
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R?
U 1 1 5FC1A4CA
P 3150 5700
F 0 "R?" V 3050 5750 50  0000 C CNN
F 1 "0" V 3050 5600 50  0000 C CNN
F 2 "" H 3150 5700 50  0001 C CNN
F 3 "~" H 3150 5700 50  0001 C CNN
	1    3150 5700
	0    -1   1    0   
$EndComp
$Comp
L Device:R_Small_US R?
U 1 1 5FC212D0
P 3500 5350
F 0 "R?" H 3432 5396 50  0000 R CNN
F 1 "500k" H 3432 5305 50  0000 R CNN
F 2 "" H 3500 5350 50  0001 C CNN
F 3 "~" H 3500 5350 50  0001 C CNN
	1    3500 5350
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R?
U 1 1 5FC22107
P 3500 5950
F 0 "R?" H 3432 5996 50  0000 R CNN
F 1 "500k" H 3432 5905 50  0000 R CNN
F 2 "" H 3500 5950 50  0001 C CNN
F 3 "~" H 3500 5950 50  0001 C CNN
	1    3500 5950
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3500 5250 3500 5200
Wire Wire Line
	3500 5200 3550 5200
Wire Wire Line
	3500 5450 3850 5450
Wire Wire Line
	3850 5450 3850 5400
Connection ~ 3850 5450
Wire Wire Line
	3500 5800 3500 5850
Wire Wire Line
	3500 5800 3550 5800
$Comp
L power:GND #PWR?
U 1 1 5FC38C0F
P 3850 6100
F 0 "#PWR?" H 3850 5850 50  0001 C CNN
F 1 "GND" H 3855 5927 50  0000 C CNN
F 2 "" H 3850 6100 50  0001 C CNN
F 3 "" H 3850 6100 50  0001 C CNN
	1    3850 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 6050 3850 6050
Wire Wire Line
	3850 6050 3850 6000
Wire Wire Line
	3850 6100 3850 6050
Connection ~ 3850 6050
$Comp
L MyParts:B2100A D?
U 1 1 5FC5A1DE
P 4050 5800
F 0 "D?" V 4004 5880 50  0000 L CNN
F 1 "B2100A" V 4095 5880 50  0000 L CNN
F 2 "Diode_SMD:D_SMA" H 4050 5625 50  0001 C CNN
F 3 "https://www.diodes.com/assets/Datasheets/B2100A.pdf" H 4050 5800 50  0001 C CNN
	1    4050 5800
	0    1    1    0   
$EndComp
Wire Wire Line
	3850 6000 4050 6000
Wire Wire Line
	4050 6000 4050 5950
Wire Wire Line
	4050 5050 4050 4950
Wire Wire Line
	4050 4950 3850 4950
Wire Wire Line
	3850 4950 3850 5000
$Comp
L power:+12V #PWR?
U 1 1 5FC6C16C
P 3850 4900
F 0 "#PWR?" H 3850 4750 50  0001 C CNN
F 1 "+12V" H 3865 5073 50  0000 C CNN
F 2 "" H 3850 4900 50  0001 C CNN
F 3 "" H 3850 4900 50  0001 C CNN
	1    3850 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 4900 3850 4950
Connection ~ 3850 4950
Wire Wire Line
	3850 5450 3850 5500
Wire Wire Line
	3850 5550 4050 5550
Connection ~ 3850 5550
Connection ~ 4050 5550
Wire Wire Line
	4050 5550 4050 5650
Connection ~ 3850 6000
Wire Wire Line
	3850 5550 3850 5600
$Comp
L Device:C_Small C?
U 1 1 5FC0C5EE
P 2750 5100
F 0 "C?" V 2950 5150 50  0000 R CNN
F 1 "100nF" V 2850 5200 50  0000 R CNN
F 2 "" H 2750 5100 50  0001 C CNN
F 3 "~" H 2750 5100 50  0001 C CNN
	1    2750 5100
	0    1    -1   0   
$EndComp
Text Label 700  5300 0    50   ~ 0
GD_EN
Wire Wire Line
	4050 5350 4050 5500
$Comp
L MyParts:10nF C?
U 1 1 5FBAD0E5
P 4500 5500
F 0 "C?" V 4350 5400 50  0000 C CNN
F 1 "10nF" V 4350 5600 50  0000 C CNN
F 2 "Capacitor_THT:C_Rect_L7.2mm_W4.5mm_P5.00mm_FKS2_FKP2_MKS2_MKP2" H 4538 5350 50  0001 C CNN
F 3 "https://www.wima.de/wp-content/uploads/media/e_WIMA_MKP_2.pdf" H 4500 5500 50  0001 C CNN
	1    4500 5500
	0    1    1    0   
$EndComp
$Comp
L Device:L L?
U 1 1 5FCA83DF
P 5000 5500
F 0 "L?" V 5100 5500 50  0000 C CNN
F 1 "COIL" V 4950 5500 50  0000 C CNN
F 2 "" H 5000 5500 50  0001 C CNN
F 3 "~" H 5000 5500 50  0001 C CNN
	1    5000 5500
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small_US R?
U 1 1 5FC18993
P 3150 5300
F 0 "R?" V 3050 5350 50  0000 C CNN
F 1 "0" V 3050 5200 50  0000 C CNN
F 2 "" H 3150 5300 50  0001 C CNN
F 3 "~" H 3150 5300 50  0001 C CNN
	1    3150 5300
	0    -1   1    0   
$EndComp
Wire Wire Line
	3250 5300 3300 5300
Wire Wire Line
	3300 5300 3300 5200
Wire Wire Line
	3300 5200 3500 5200
Connection ~ 3500 5200
Wire Wire Line
	3250 5700 3300 5700
Wire Wire Line
	3300 5700 3300 5800
Wire Wire Line
	3300 5800 3500 5800
Connection ~ 3500 5800
Wire Wire Line
	2300 5300 3050 5300
Wire Wire Line
	2300 5700 3050 5700
Connection ~ 3850 5500
Wire Wire Line
	3850 5500 3850 5550
Wire Wire Line
	2300 5500 2950 5500
Wire Wire Line
	2850 5100 2950 5100
Wire Wire Line
	2950 5100 2950 5500
Connection ~ 2950 5500
Wire Wire Line
	2950 5500 3850 5500
$Comp
L MyParts:DGD05473 U?
U 1 1 5FD4EFA4
P 7650 5500
F 0 "U?" H 7650 6167 50  0000 C CNN
F 1 "DGD05473" H 7650 6076 50  0000 C CNN
F 2 "MyParts:W-DFN3030-10" H 7750 4950 50  0001 C CNN
F 3 "https://www.diodes.com/assets/Datasheets/DGD05473.pdf" H 7650 5500 50  0001 C CNN
	1    7650 5500
	-1   0    0    -1  
$EndComp
$Comp
L MyParts:DMN3023L Q?
U 1 1 5FD4EFAE
P 5750 5200
F 0 "Q?" H 5600 5400 50  0000 L CNN
F 1 "DMN3023L" H 5300 5300 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5950 5125 50  0001 L CIN
F 3 "https://www.diodes.com/assets/Datasheets/DMN3023L.pdf" H 5750 5200 50  0001 L CNN
	1    5750 5200
	-1   0    0    -1  
$EndComp
$Comp
L MyParts:B2100A D?
U 1 1 5FD4EFB8
P 5450 5200
F 0 "D?" V 5404 5280 50  0000 L CNN
F 1 "B2100A" V 5495 5280 50  0000 L CNN
F 2 "Diode_SMD:D_SMA" H 5450 5025 50  0001 C CNN
F 3 "https://www.diodes.com/assets/Datasheets/B2100A.pdf" H 5450 5200 50  0001 C CNN
	1    5450 5200
	0    -1   1    0   
$EndComp
$Comp
L power:+12V #PWR?
U 1 1 5FD4EFC2
P 8200 5000
F 0 "#PWR?" H 8200 4850 50  0001 C CNN
F 1 "+12V" H 8215 5173 50  0000 C CNN
F 2 "" H 8200 5000 50  0001 C CNN
F 3 "" H 8200 5000 50  0001 C CNN
	1    8200 5000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8100 5500 8400 5500
Wire Wire Line
	8100 5700 8400 5700
$Comp
L power:GND #PWR?
U 1 1 5FD4EFCE
P 8200 6000
F 0 "#PWR?" H 8200 5750 50  0001 C CNN
F 1 "GND" H 8205 5827 50  0000 C CNN
F 2 "" H 8200 6000 50  0001 C CNN
F 3 "" H 8200 6000 50  0001 C CNN
	1    8200 6000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8200 6000 8200 5900
Wire Wire Line
	8200 5800 8100 5800
Wire Wire Line
	8100 5900 8200 5900
Connection ~ 8200 5900
Wire Wire Line
	8200 5900 8200 5800
Text Label 8400 5500 2    50   ~ 0
HIN_B
Text Label 8400 5700 2    50   ~ 0
LIN_B
Wire Wire Line
	8200 5000 8200 5100
Wire Wire Line
	8200 5100 8100 5100
$Comp
L Device:R_Small_US R?
U 1 1 5FD4EFE1
P 8550 5450
F 0 "R?" H 8482 5404 50  0000 R CNN
F 1 "10k" H 8482 5495 50  0000 R CNN
F 2 "" H 8550 5450 50  0001 C CNN
F 3 "~" H 8550 5450 50  0001 C CNN
	1    8550 5450
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FD4EFEB
P 8550 5600
F 0 "#PWR?" H 8550 5350 50  0001 C CNN
F 1 "GND" H 8555 5427 50  0000 C CNN
F 2 "" H 8550 5600 50  0001 C CNN
F 3 "" H 8550 5600 50  0001 C CNN
	1    8550 5600
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8550 5600 8550 5550
Wire Wire Line
	8550 5300 8550 5350
Wire Wire Line
	8550 5300 8100 5300
Wire Wire Line
	8550 5300 8800 5300
Connection ~ 8550 5300
$Comp
L Device:R_Small_US R?
U 1 1 5FD4EFFA
P 7050 5100
F 0 "R?" V 6845 5100 50  0000 C CNN
F 1 "10" V 6936 5100 50  0000 C CNN
F 2 "" H 7050 5100 50  0001 C CNN
F 3 "~" H 7050 5100 50  0001 C CNN
	1    7050 5100
	0    1    1    0   
$EndComp
Wire Wire Line
	7200 5100 7150 5100
Wire Wire Line
	6950 5100 6850 5100
$Comp
L MyParts:DMN3023L Q?
U 1 1 5FD4F006
P 5750 5800
F 0 "Q?" H 5600 6000 50  0000 L CNN
F 1 "DMN3023L" H 5300 5900 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5950 5725 50  0001 L CIN
F 3 "https://www.diodes.com/assets/Datasheets/DMN3023L.pdf" H 5750 5800 50  0001 L CNN
	1    5750 5800
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R?
U 1 1 5FD4F010
P 6350 5700
F 0 "R?" V 6250 5750 50  0000 C CNN
F 1 "0" V 6250 5600 50  0000 C CNN
F 2 "" H 6350 5700 50  0001 C CNN
F 3 "~" H 6350 5700 50  0001 C CNN
	1    6350 5700
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small_US R?
U 1 1 5FD4F01A
P 6000 5350
F 0 "R?" H 5932 5396 50  0000 R CNN
F 1 "500k" H 5932 5305 50  0000 R CNN
F 2 "" H 6000 5350 50  0001 C CNN
F 3 "~" H 6000 5350 50  0001 C CNN
	1    6000 5350
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R?
U 1 1 5FD4F024
P 6000 5950
F 0 "R?" H 5932 5996 50  0000 R CNN
F 1 "500k" H 5932 5905 50  0000 R CNN
F 2 "" H 6000 5950 50  0001 C CNN
F 3 "~" H 6000 5950 50  0001 C CNN
	1    6000 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 5250 6000 5200
Wire Wire Line
	6000 5200 5950 5200
Wire Wire Line
	6000 5450 5650 5450
Wire Wire Line
	5650 5450 5650 5400
Connection ~ 5650 5450
Wire Wire Line
	6000 5800 6000 5850
Wire Wire Line
	6000 5800 5950 5800
$Comp
L power:GND #PWR?
U 1 1 5FD4F035
P 5650 6100
F 0 "#PWR?" H 5650 5850 50  0001 C CNN
F 1 "GND" H 5655 5927 50  0000 C CNN
F 2 "" H 5650 6100 50  0001 C CNN
F 3 "" H 5650 6100 50  0001 C CNN
	1    5650 6100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6000 6050 5650 6050
Wire Wire Line
	5650 6050 5650 6000
Wire Wire Line
	5650 6100 5650 6050
Connection ~ 5650 6050
$Comp
L MyParts:B2100A D?
U 1 1 5FD4F043
P 5450 5800
F 0 "D?" V 5404 5880 50  0000 L CNN
F 1 "B2100A" V 5495 5880 50  0000 L CNN
F 2 "Diode_SMD:D_SMA" H 5450 5625 50  0001 C CNN
F 3 "https://www.diodes.com/assets/Datasheets/B2100A.pdf" H 5450 5800 50  0001 C CNN
	1    5450 5800
	0    -1   1    0   
$EndComp
Wire Wire Line
	5650 6000 5450 6000
Wire Wire Line
	5450 6000 5450 5950
Wire Wire Line
	5450 5050 5450 4950
Wire Wire Line
	5450 4950 5650 4950
Wire Wire Line
	5650 4950 5650 5000
$Comp
L power:+12V #PWR?
U 1 1 5FD4F052
P 5650 4900
F 0 "#PWR?" H 5650 4750 50  0001 C CNN
F 1 "+12V" H 5665 5073 50  0000 C CNN
F 2 "" H 5650 4900 50  0001 C CNN
F 3 "" H 5650 4900 50  0001 C CNN
	1    5650 4900
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5650 4900 5650 4950
Connection ~ 5650 4950
Wire Wire Line
	5650 5450 5650 5500
Wire Wire Line
	5650 5550 5450 5550
Connection ~ 5650 5550
Connection ~ 5450 5550
Wire Wire Line
	5450 5550 5450 5650
Connection ~ 5650 6000
Wire Wire Line
	5650 5550 5650 5600
$Comp
L Device:C_Small C?
U 1 1 5FD4F065
P 6750 5100
F 0 "C?" V 6950 5150 50  0000 R CNN
F 1 "100nF" V 6850 5200 50  0000 R CNN
F 2 "" H 6750 5100 50  0001 C CNN
F 3 "~" H 6750 5100 50  0001 C CNN
	1    6750 5100
	0    -1   -1   0   
$EndComp
Text Label 8800 5300 2    50   ~ 0
GD_EN
Wire Wire Line
	5450 5350 5450 5500
$Comp
L Device:R_Small_US R?
U 1 1 5FD4F071
P 6350 5300
F 0 "R?" V 6250 5350 50  0000 C CNN
F 1 "0" V 6250 5200 50  0000 C CNN
F 2 "" H 6350 5300 50  0001 C CNN
F 3 "~" H 6350 5300 50  0001 C CNN
	1    6350 5300
	0    1    1    0   
$EndComp
Wire Wire Line
	6250 5300 6200 5300
Wire Wire Line
	6200 5300 6200 5200
Wire Wire Line
	6200 5200 6000 5200
Connection ~ 6000 5200
Wire Wire Line
	6250 5700 6200 5700
Wire Wire Line
	6200 5700 6200 5800
Wire Wire Line
	6200 5800 6000 5800
Connection ~ 6000 5800
Wire Wire Line
	7200 5300 6450 5300
Wire Wire Line
	7200 5700 6450 5700
Connection ~ 5650 5500
Wire Wire Line
	5650 5500 5650 5550
Wire Wire Line
	7200 5500 6550 5500
Wire Wire Line
	6650 5100 6550 5100
Wire Wire Line
	6550 5100 6550 5500
Connection ~ 6550 5500
Wire Wire Line
	6550 5500 5650 5500
Wire Wire Line
	4050 5500 4350 5500
Connection ~ 4050 5500
Wire Wire Line
	4050 5500 4050 5550
Wire Wire Line
	4650 5500 4850 5500
Wire Wire Line
	5150 5500 5450 5500
Connection ~ 5450 5500
Wire Wire Line
	5450 5500 5450 5550
Text Label 2500 5300 0    50   ~ 0
HO_A
Text Label 2500 5700 0    50   ~ 0
LO_A
Text Label 4100 5500 0    50   ~ 0
VSW+
Text Label 5200 5500 0    50   ~ 0
VSW-
Text Label 6850 5300 0    50   ~ 0
HO_B
Text Label 6850 5700 0    50   ~ 0
LO_B
Connection ~ 1000 3100
Wire Wire Line
	1000 3000 1000 3100
$Comp
L power:+12V #PWR?
U 1 1 5FBA432F
P 1000 3000
F 0 "#PWR?" H 1000 2850 50  0001 C CNN
F 1 "+12V" H 1015 3173 50  0000 C CNN
F 2 "" H 1000 3000 50  0001 C CNN
F 3 "" H 1000 3000 50  0001 C CNN
	1    1000 3000
	1    0    0    -1  
$EndComp
Connection ~ 4500 3300
Wire Wire Line
	4500 3250 4500 3300
$Comp
L power:+3.3V #PWR?
U 1 1 5FB96E34
P 4500 3250
F 0 "#PWR?" H 4500 3100 50  0001 C CNN
F 1 "+3.3V" H 4515 3423 50  0000 C CNN
F 2 "" H 4500 3250 50  0001 C CNN
F 3 "" H 4500 3250 50  0001 C CNN
	1    4500 3250
	1    0    0    -1  
$EndComp
Connection ~ 4500 3550
Wire Wire Line
	4500 3600 4500 3550
Wire Wire Line
	4500 3550 4500 3500
Wire Wire Line
	4100 3550 4500 3550
Wire Wire Line
	4100 3500 4100 3550
Wire Wire Line
	4100 3300 4500 3300
Connection ~ 4100 3300
Connection ~ 3750 3300
Wire Wire Line
	3750 3300 4100 3300
$Comp
L power:GND #PWR?
U 1 1 5FB94E21
P 4500 3600
F 0 "#PWR?" H 4500 3350 50  0001 C CNN
F 1 "GND" H 4505 3427 50  0000 C CNN
F 2 "" H 4500 3600 50  0001 C CNN
F 3 "" H 4500 3600 50  0001 C CNN
	1    4500 3600
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5FB942E6
P 4500 3400
F 0 "C?" H 4408 3446 50  0000 R CNN
F 1 "10uF" H 4408 3355 50  0000 R CNN
F 2 "" H 4500 3400 50  0001 C CNN
F 3 "~" H 4500 3400 50  0001 C CNN
	1    4500 3400
	-1   0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5FB93B10
P 4100 3400
F 0 "C?" H 4008 3446 50  0000 R CNN
F 1 "10uF" H 4008 3355 50  0000 R CNN
F 2 "" H 4100 3400 50  0001 C CNN
F 3 "~" H 4100 3400 50  0001 C CNN
	1    4100 3400
	-1   0    0    -1  
$EndComp
Connection ~ 1000 3350
Wire Wire Line
	1000 3400 1000 3350
Wire Wire Line
	1000 3350 1000 3300
Wire Wire Line
	1400 3350 1000 3350
Wire Wire Line
	1400 3300 1400 3350
$Comp
L power:GND #PWR?
U 1 1 5FB92A6C
P 1000 3400
F 0 "#PWR?" H 1000 3150 50  0001 C CNN
F 1 "GND" H 1005 3227 50  0000 C CNN
F 2 "" H 1000 3400 50  0001 C CNN
F 3 "" H 1000 3400 50  0001 C CNN
	1    1000 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 3100 1000 3100
Connection ~ 1400 3100
Connection ~ 1750 3100
Wire Wire Line
	1750 3100 1400 3100
$Comp
L Device:C_Small C?
U 1 1 5FB91F2D
P 1000 3200
F 0 "C?" H 908 3246 50  0000 R CNN
F 1 "10uF" H 908 3155 50  0000 R CNN
F 2 "" H 1000 3200 50  0001 C CNN
F 3 "~" H 1000 3200 50  0001 C CNN
	1    1000 3200
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1750 3800 1900 3800
Wire Wire Line
	1750 3950 1750 3800
$Comp
L power:GND #PWR?
U 1 1 5FB8BA3A
P 1750 3950
F 0 "#PWR?" H 1750 3700 50  0001 C CNN
F 1 "GND" H 1755 3777 50  0000 C CNN
F 2 "" H 1750 3950 50  0001 C CNN
F 3 "" H 1750 3950 50  0001 C CNN
	1    1750 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 3100 1900 3100
Wire Wire Line
	1750 3150 1750 3100
Wire Wire Line
	1750 3450 1750 3350
Wire Wire Line
	1900 3450 1750 3450
$Comp
L Device:R_Small_US R?
U 1 1 5FB89E6C
P 1750 3250
F 0 "R?" H 1818 3296 50  0000 L CNN
F 1 "10k" H 1818 3205 50  0000 L CNN
F 2 "" H 1750 3250 50  0001 C CNN
F 3 "~" H 1750 3250 50  0001 C CNN
	1    1750 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 3300 3650 3300
Connection ~ 3700 3300
Wire Wire Line
	3700 3100 3700 3300
Wire Wire Line
	3200 3100 3700 3100
Wire Wire Line
	3750 4000 3750 4050
$Comp
L power:GND #PWR?
U 1 1 5FB8940F
P 3750 4050
F 0 "#PWR?" H 3750 3800 50  0001 C CNN
F 1 "GND" H 3755 3877 50  0000 C CNN
F 2 "" H 3750 4050 50  0001 C CNN
F 3 "" H 3750 4050 50  0001 C CNN
	1    3750 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 3300 3700 3300
Wire Wire Line
	3750 3450 3750 3300
Connection ~ 3750 3800
Wire Wire Line
	3750 3650 3750 3800
Wire Wire Line
	2700 3800 3750 3800
$Comp
L Device:R_Small_US R?
U 1 1 5FB883A7
P 3750 3900
F 0 "R?" H 3818 3946 50  0000 L CNN
F 1 "10k" H 3818 3855 50  0000 L CNN
F 2 "" H 3750 3900 50  0001 C CNN
F 3 "~" H 3750 3900 50  0001 C CNN
	1    3750 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 3300 3350 3300
Connection ~ 3200 3300
Connection ~ 2800 3300
Wire Wire Line
	2800 3300 3200 3300
Wire Wire Line
	2800 3450 2800 3300
Wire Wire Line
	2700 3450 2800 3450
Wire Wire Line
	2800 3100 2900 3100
Connection ~ 2800 3100
Wire Wire Line
	2700 3100 2800 3100
$Comp
L power:GND #PWR?
U 1 1 5FB86D15
P 3200 3600
F 0 "#PWR?" H 3200 3350 50  0001 C CNN
F 1 "GND" H 3205 3427 50  0000 C CNN
F 2 "" H 3200 3600 50  0001 C CNN
F 3 "" H 3200 3600 50  0001 C CNN
	1    3200 3600
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5FB855C2
P 2800 3200
F 0 "C?" H 2892 3246 50  0000 L CNN
F 1 "100nF" H 2892 3155 50  0000 L CNN
F 2 "" H 2800 3200 50  0001 C CNN
F 3 "~" H 2800 3200 50  0001 C CNN
	1    2800 3200
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R?
U 1 1 5FB845B2
P 3750 3550
F 0 "R?" H 3818 3596 50  0000 L CNN
F 1 "31.6k" H 3818 3505 50  0000 L CNN
F 2 "" H 3750 3550 50  0001 C CNN
F 3 "~" H 3750 3550 50  0001 C CNN
	1    3750 3550
	1    0    0    -1  
$EndComp
$Comp
L MyParts:B2100A D?
U 1 1 5FB83E83
P 3200 3450
F 0 "D?" V 3154 3530 50  0000 L CNN
F 1 "B2100A" V 3245 3530 50  0000 L CNN
F 2 "Diode_SMD:D_SMA" H 3200 3275 50  0001 C CNN
F 3 "https://www.diodes.com/assets/Datasheets/B2100A.pdf" H 3200 3450 50  0001 C CNN
	1    3200 3450
	0    1    1    0   
$EndComp
$Comp
L MyParts:1N4148W-7-F D?
U 1 1 5FB836B7
P 3050 3100
F 0 "D?" H 3050 3317 50  0000 C CNN
F 1 "1N4148W-7-F" H 3050 3226 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-123" H 3050 2925 50  0001 C CNN
F 3 "https://www.diodes.com/assets/Datasheets/ds30086.pdf" H 3050 3100 50  0001 C CNN
	1    3050 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:L L?
U 1 1 5FB82AFC
P 3500 3300
F 0 "L?" V 3600 3300 50  0000 C CNN
F 1 "15uH" V 3450 3300 50  0000 C CNN
F 2 "" H 3500 3300 50  0001 C CNN
F 3 "~" H 3500 3300 50  0001 C CNN
	1    3500 3300
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5FB80443
P 1400 3200
F 0 "C?" H 1308 3246 50  0000 R CNN
F 1 "10uF" H 1308 3155 50  0000 R CNN
F 2 "" H 1400 3200 50  0001 C CNN
F 3 "~" H 1400 3200 50  0001 C CNN
	1    1400 3200
	-1   0    0    -1  
$EndComp
$Comp
L MyParts:MCP16331 U?
U 1 1 5FB7973D
P 2300 3300
F 0 "U?" H 2300 3767 50  0000 C CNN
F 1 "MCP16331" H 2300 3676 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 2000 2650 50  0001 L CIN
F 3 "https://ww1.microchip.com/downloads/en/DeviceDoc/20005308C.pdf" H 2050 3750 50  0001 C CNN
	1    2300 3300
	1    0    0    -1  
$EndComp
$EndSCHEMATC
