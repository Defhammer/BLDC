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
LIBS:DSPIC33FJ64MC802-I_SP
LIBS:LM2901NG
LIBS:switches
LIBS:BLDC_Logic-cache
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
L DSPIC33FJ64MC802-I/SP U2
U 1 1 58FE67F6
P 4700 2200
F 0 "U2" H 4526 3102 50  0000 L CNN
F 1 "DSPIC33FJ64MC802-I/SP" H 4402 920 50  0000 L CNN
F 2 "DSPIC33FJ64MC802-I_SP:DIP254P762X508-28" H 4700 2200 50  0001 L CNN
F 3 "4.76 USD" H 4700 2200 50  0001 L CNN
F 4 "Microchip" H 4700 2200 50  0001 L CNN "MF"
F 5 "Good" H 4700 2200 50  0001 L CNN "Availability"
F 6 "DSPIC33FJ64MC802-I/SP" H 4700 2200 50  0001 L CNN "MP"
F 7 "MCU 16-bit dsPIC33F dsPIC RISC 64KB Flash 3.3V 28-Pin SPDIP Tube" H 4700 2200 50  0001 L CNN "Description"
F 8 "SPDIP-28 Microchip" H 4700 2200 50  0001 L CNN "Package"
	1    4700 2200
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR01
U 1 1 58FE6A06
P 1700 1000
F 0 "#PWR01" H 1700 850 50  0001 C CNN
F 1 "+3.3V" H 1700 1140 50  0000 C CNN
F 2 "" H 1700 1000 50  0001 C CNN
F 3 "" H 1700 1000 50  0001 C CNN
	1    1700 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 2800 1600 2800
Wire Wire Line
	1600 2800 1600 3250
Wire Wire Line
	1800 2900 1600 2900
Connection ~ 1600 2900
Wire Wire Line
	1800 3000 1600 3000
Connection ~ 1600 3000
$Comp
L GND #PWR02
U 1 1 58FE6B08
P 1600 3250
F 0 "#PWR02" H 1600 3000 50  0001 C CNN
F 1 "GND" H 1600 3100 50  0000 C CNN
F 2 "" H 1600 3250 50  0001 C CNN
F 3 "" H 1600 3250 50  0001 C CNN
	1    1600 3250
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X03 J9
U 1 1 58FE6C54
P 9250 2300
F 0 "J9" H 9250 2500 50  0000 C CNN
F 1 "Encdr" V 9350 2300 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 9250 2300 50  0001 C CNN
F 3 "" H 9250 2300 50  0001 C CNN
	1    9250 2300
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X06 J8
U 1 1 58FE6F80
P 9250 1750
F 0 "J8" H 9250 2100 50  0000 C CNN
F 1 "Programer" V 9350 1750 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 9250 1750 50  0001 C CNN
F 3 "" H 9250 1750 50  0001 C CNN
	1    9250 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 1500 8800 1500
Text Label 8800 1500 0    60   ~ 0
MCLR
Text Label 1500 1900 0    60   ~ 0
MCLR
$Comp
L R R2
U 1 1 58FE7553
P 1450 1050
F 0 "R2" V 1530 1050 50  0000 C CNN
F 1 "10K" V 1450 1050 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 1380 1050 50  0001 C CNN
F 3 "" H 1450 1050 50  0001 C CNN
	1    1450 1050
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR03
U 1 1 58FE797B
P 8400 1600
F 0 "#PWR03" H 8400 1350 50  0001 C CNN
F 1 "GND" H 8400 1450 50  0000 C CNN
F 2 "" H 8400 1600 50  0001 C CNN
F 3 "" H 8400 1600 50  0001 C CNN
	1    8400 1600
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR04
U 1 1 58FE7BAF
P 8650 1550
F 0 "#PWR04" H 8650 1400 50  0001 C CNN
F 1 "+3.3V" H 8650 1690 50  0000 C CNN
F 2 "" H 8650 1550 50  0001 C CNN
F 3 "" H 8650 1550 50  0001 C CNN
	1    8650 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 1600 8650 1600
Wire Wire Line
	8650 1600 8650 1550
Wire Wire Line
	8400 1600 8550 1600
Wire Wire Line
	8550 1600 8550 1700
Wire Wire Line
	8550 1700 9050 1700
$Comp
L CP C3
U 1 1 58FE8C8D
P 950 2000
F 0 "C3" H 975 2100 50  0000 L CNN
F 1 "10uF" H 975 1900 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D4.0mm_P1.50mm" H 988 1850 50  0001 C CNN
F 3 "" H 950 2000 50  0001 C CNN
	1    950  2000
	0    1    1    0   
$EndComp
Wire Wire Line
	1800 2000 1100 2000
$Comp
L GND #PWR05
U 1 1 58FE8D2C
P 700 2900
F 0 "#PWR05" H 700 2650 50  0001 C CNN
F 1 "GND" H 700 2750 50  0000 C CNN
F 2 "" H 700 2900 50  0001 C CNN
F 3 "" H 700 2900 50  0001 C CNN
	1    700  2900
	1    0    0    -1  
$EndComp
$Comp
L Crystal Y1
U 1 1 58FEAC84
P 1200 2450
F 0 "Y1" H 1200 2600 50  0000 C CNN
F 1 "Crystal" H 1200 2300 50  0000 C CNN
F 2 "Crystals:Crystal_HC49-U_Vertical" H 1200 2450 50  0001 C CNN
F 3 "" H 1200 2450 50  0001 C CNN
	1    1200 2450
	0    1    1    0   
$EndComp
Wire Wire Line
	1800 2400 1400 2400
Wire Wire Line
	1400 2400 1400 2250
Wire Wire Line
	1400 2250 1050 2250
Wire Wire Line
	1200 2250 1200 2300
Wire Wire Line
	1800 2500 1400 2500
Wire Wire Line
	1400 2500 1400 2650
Wire Wire Line
	1400 2650 1050 2650
Wire Wire Line
	1200 2650 1200 2600
$Comp
L C C2
U 1 1 58FEAE5D
P 900 2650
F 0 "C2" H 925 2750 50  0000 L CNN
F 1 "22pF" H 925 2550 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D3.0mm_W1.6mm_P2.50mm" H 938 2500 50  0001 C CNN
F 3 "" H 900 2650 50  0001 C CNN
	1    900  2650
	0    1    1    0   
$EndComp
Connection ~ 1200 2650
$Comp
L C C1
U 1 1 58FEAF08
P 900 2250
F 0 "C1" H 925 2350 50  0000 L CNN
F 1 "22pF" H 925 2150 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D3.0mm_W1.6mm_P2.50mm" H 938 2100 50  0001 C CNN
F 3 "" H 900 2250 50  0001 C CNN
	1    900  2250
	0    1    1    0   
$EndComp
Connection ~ 1200 2250
Wire Wire Line
	750  2250 700  2250
Wire Wire Line
	700  2000 700  2900
Wire Wire Line
	750  2650 700  2650
Connection ~ 700  2650
Wire Wire Line
	800  2000 700  2000
Connection ~ 700  2250
$Comp
L R R3
U 1 1 58FECF78
P 1600 4050
F 0 "R3" V 1680 4050 50  0000 C CNN
F 1 "3.3k" V 1600 4050 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 1530 4050 50  0001 C CNN
F 3 "" H 1600 4050 50  0001 C CNN
	1    1600 4050
	0    1    1    0   
$EndComp
$Comp
L R R7
U 1 1 58FECF7E
P 1800 4250
F 0 "R7" V 1880 4250 50  0000 C CNN
F 1 "2.2k" V 1800 4250 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 1730 4250 50  0001 C CNN
F 3 "" H 1800 4250 50  0001 C CNN
	1    1800 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 4050 1800 4100
$Comp
L GND #PWR06
U 1 1 58FECF86
P 1800 4450
F 0 "#PWR06" H 1800 4200 50  0001 C CNN
F 1 "GND" H 1800 4300 50  0000 C CNN
F 2 "" H 1800 4450 50  0001 C CNN
F 3 "" H 1800 4450 50  0001 C CNN
	1    1800 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 4400 1800 4450
Wire Wire Line
	950  4050 1450 4050
Connection ~ 1800 4050
$Comp
L R R4
U 1 1 58FEDF18
P 1600 4700
F 0 "R4" V 1680 4700 50  0000 C CNN
F 1 "3.3k" V 1600 4700 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 1530 4700 50  0001 C CNN
F 3 "" H 1600 4700 50  0001 C CNN
	1    1600 4700
	0    1    1    0   
$EndComp
$Comp
L R R8
U 1 1 58FEDF1E
P 1800 4900
F 0 "R8" V 1880 4900 50  0000 C CNN
F 1 "2.2k" V 1800 4900 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 1730 4900 50  0001 C CNN
F 3 "" H 1800 4900 50  0001 C CNN
	1    1800 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 4700 1950 4700
Wire Wire Line
	1800 4700 1800 4750
$Comp
L GND #PWR07
U 1 1 58FEDF26
P 1800 5100
F 0 "#PWR07" H 1800 4850 50  0001 C CNN
F 1 "GND" H 1800 4950 50  0000 C CNN
F 2 "" H 1800 5100 50  0001 C CNN
F 3 "" H 1800 5100 50  0001 C CNN
	1    1800 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 5050 1800 5100
Wire Wire Line
	1050 4700 1450 4700
Connection ~ 1800 4700
$Comp
L R R5
U 1 1 58FEE007
P 1600 5450
F 0 "R5" V 1680 5450 50  0000 C CNN
F 1 "3.3k" V 1600 5450 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 1530 5450 50  0001 C CNN
F 3 "" H 1600 5450 50  0001 C CNN
	1    1600 5450
	0    1    1    0   
$EndComp
$Comp
L R R9
U 1 1 58FEE00D
P 1800 5650
F 0 "R9" V 1880 5650 50  0000 C CNN
F 1 "2.2k" V 1800 5650 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 1730 5650 50  0001 C CNN
F 3 "" H 1800 5650 50  0001 C CNN
	1    1800 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 5450 1800 5500
$Comp
L GND #PWR08
U 1 1 58FEE015
P 1800 5850
F 0 "#PWR08" H 1800 5600 50  0001 C CNN
F 1 "GND" H 1800 5700 50  0000 C CNN
F 2 "" H 1800 5850 50  0001 C CNN
F 3 "" H 1800 5850 50  0001 C CNN
	1    1800 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 5800 1800 5850
Wire Wire Line
	1050 5450 1450 5450
Connection ~ 1800 5450
$Comp
L R R6
U 1 1 58FEE11E
P 1600 6100
F 0 "R6" V 1680 6100 50  0000 C CNN
F 1 "3.3k" V 1600 6100 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 1530 6100 50  0001 C CNN
F 3 "" H 1600 6100 50  0001 C CNN
	1    1600 6100
	0    1    1    0   
$EndComp
$Comp
L R R10
U 1 1 58FEE124
P 1800 6300
F 0 "R10" V 1880 6300 50  0000 C CNN
F 1 "2.2k" V 1800 6300 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 1730 6300 50  0001 C CNN
F 3 "" H 1800 6300 50  0001 C CNN
	1    1800 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 6100 1800 6150
$Comp
L GND #PWR09
U 1 1 58FEE12C
P 1800 6500
F 0 "#PWR09" H 1800 6250 50  0001 C CNN
F 1 "GND" H 1800 6350 50  0000 C CNN
F 2 "" H 1800 6500 50  0001 C CNN
F 3 "" H 1800 6500 50  0001 C CNN
	1    1800 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 6450 1800 6500
Wire Wire Line
	1050 6100 1450 6100
Connection ~ 1800 6100
$Comp
L LM2901NG U1
U 1 1 58FEEC63
P 3450 5000
F 0 "U1" H 3239 5759 50  0000 L CNN
F 1 "LM2901NG" H 3295 3833 50  0000 L CNN
F 2 "Housings_DIP:DIP-14_W7.62mm" H 3450 5000 50  0001 L CNN
F 3 "ON Semiconductor" H 3450 5000 50  0001 L CNN
F 4 "Low Cost, 36V, Single Supply Quad Comparator (Industrial Grade) TA= -40° to +105°C" H 3450 5000 50  0001 L CNN "Description"
F 5 "0.27 USD" H 3450 5000 50  0001 L CNN "Price"
F 6 "Good" H 3450 5000 50  0001 L CNN "Availability"
F 7 "DIP-14 ON Semiconductor" H 3450 5000 50  0001 L CNN "Package"
F 8 "LM2901NG" H 3450 5000 50  0001 L CNN "MP"
	1    3450 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 4600 4250 4600
Wire Wire Line
	4150 4700 4250 4700
Wire Wire Line
	4150 4800 4250 4800
Wire Wire Line
	4150 4900 4250 4900
Text Label 2600 4800 0    60   ~ 0
O1
Text Label 2600 5100 0    60   ~ 0
O2
Text Label 2600 5200 0    60   ~ 0
O3
Text Label 2600 5400 0    60   ~ 0
O4
Text Label 4200 4600 0    60   ~ 0
O1
Text Label 4200 4700 0    60   ~ 0
O2
Text Label 4200 4800 0    60   ~ 0
O3
Text Label 4200 4900 0    60   ~ 0
O4
$Comp
L +3.3V #PWR010
U 1 1 58FEFCD7
P 2700 4400
F 0 "#PWR010" H 2700 4250 50  0001 C CNN
F 1 "+3.3V" H 2700 4540 50  0000 C CNN
F 2 "" H 2700 4400 50  0001 C CNN
F 3 "" H 2700 4400 50  0001 C CNN
	1    2700 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 4400 2700 4600
Wire Wire Line
	2700 4600 2750 4600
$Comp
L GND #PWR011
U 1 1 58FF0088
P 2700 5800
F 0 "#PWR011" H 2700 5550 50  0001 C CNN
F 1 "GND" H 2700 5650 50  0000 C CNN
F 2 "" H 2700 5800 50  0001 C CNN
F 3 "" H 2700 5800 50  0001 C CNN
	1    2700 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 5700 2700 5700
Wire Wire Line
	2700 5700 2700 5800
Wire Wire Line
	1950 5000 2750 5000
Wire Wire Line
	1950 5300 2750 5300
Wire Wire Line
	2050 5500 2750 5500
Wire Wire Line
	1950 4700 1950 5000
Wire Wire Line
	1750 4050 2050 4050
Wire Wire Line
	2050 4050 2050 4900
Wire Wire Line
	2050 4900 2750 4900
Wire Wire Line
	1750 5450 1950 5450
Wire Wire Line
	1950 5450 1950 5300
Wire Wire Line
	1750 6100 2050 6100
Wire Wire Line
	2050 6100 2050 5500
Wire Wire Line
	2600 4800 2750 4800
Wire Wire Line
	2600 5100 2750 5100
Wire Wire Line
	2600 5200 2750 5200
Wire Wire Line
	2600 5400 2750 5400
$Comp
L CONN_01X03 J2
U 1 1 58FF1CA0
P 850 5450
F 0 "J2" H 850 5650 50  0000 C CNN
F 1 "CMD inputs" V 950 5450 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 850 5450 50  0001 C CNN
F 3 "" H 850 5450 50  0001 C CNN
	1    850  5450
	-1   0    0    1   
$EndComp
Wire Wire Line
	1050 5550 1050 6100
Wire Wire Line
	1050 5350 1050 4700
$Comp
L GND #PWR012
U 1 1 58FF222E
P 1000 4200
F 0 "#PWR012" H 1000 3950 50  0001 C CNN
F 1 "GND" H 1000 4050 50  0000 C CNN
F 2 "" H 1000 4200 50  0001 C CNN
F 3 "" H 1000 4200 50  0001 C CNN
	1    1000 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  4150 1000 4150
Wire Wire Line
	1000 4150 1000 4200
Wire Wire Line
	1600 2200 1800 2200
Text Label 1600 2200 0    60   ~ 0
O1
Text Label 1600 2300 0    60   ~ 0
O2
Wire Wire Line
	1600 2300 1800 2300
Wire Wire Line
	7600 1600 7750 1600
Text Label 7750 1600 0    60   ~ 0
O3
Text Label 7750 1700 0    60   ~ 0
O4
Wire Wire Line
	7600 1700 7750 1700
$Comp
L CONN_01X03 J7
U 1 1 58FF4111
P 9250 1200
F 0 "J7" H 9250 1400 50  0000 C CNN
F 1 "SERIAL" V 9350 1200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 9250 1200 50  0001 C CNN
F 3 "" H 9250 1200 50  0001 C CNN
	1    9250 1200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 58FF4353
P 8400 1350
F 0 "#PWR013" H 8400 1100 50  0001 C CNN
F 1 "GND" H 8400 1200 50  0000 C CNN
F 2 "" H 8400 1350 50  0001 C CNN
F 3 "" H 8400 1350 50  0001 C CNN
	1    8400 1350
	1    0    0    -1  
$EndComp
$Comp
L AP111733 U3
U 1 1 58FF6FE4
P 5250 5050
F 0 "U3" H 5350 4800 50  0000 C CNN
F 1 "AP111733" H 5250 5300 50  0000 C CNN
F 2 "TO_SOT_Packages_THT:TO-220_Vertical" H 5250 4700 50  0001 C CNN
F 3 "" H 5350 4800 50  0001 C CNN
	1    5250 5050
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR014
U 1 1 58FF7455
P 4750 5000
F 0 "#PWR014" H 4750 4850 50  0001 C CNN
F 1 "+5V" H 4750 5140 50  0000 C CNN
F 2 "" H 4750 5000 50  0001 C CNN
F 3 "" H 4750 5000 50  0001 C CNN
	1    4750 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 5050 4750 5050
Wire Wire Line
	4750 5050 4750 5000
$Comp
L CP C7
U 1 1 58FF7B72
P 4850 5250
F 0 "C7" H 4875 5350 50  0000 L CNN
F 1 "100uF" H 4875 5150 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D6.3mm_P2.50mm" H 4888 5100 50  0001 C CNN
F 3 "" H 4850 5250 50  0001 C CNN
	1    4850 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 5100 4850 5050
Connection ~ 4850 5050
$Comp
L CP C8
U 1 1 58FF7E48
P 5750 5250
F 0 "C8" H 5775 5350 50  0000 L CNN
F 1 "10uF" H 5775 5150 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D4.0mm_P2.00mm" H 5788 5100 50  0001 C CNN
F 3 "" H 5750 5250 50  0001 C CNN
	1    5750 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 5050 5900 5050
Wire Wire Line
	5750 5100 5750 5050
Connection ~ 5750 5050
Wire Wire Line
	5750 5400 4850 5400
Wire Wire Line
	5250 5350 5250 5450
Connection ~ 5250 5400
$Comp
L GND #PWR015
U 1 1 58FF8103
P 5250 5450
F 0 "#PWR015" H 5250 5200 50  0001 C CNN
F 1 "GND" H 5250 5300 50  0000 C CNN
F 2 "" H 5250 5450 50  0001 C CNN
F 3 "" H 5250 5450 50  0001 C CNN
	1    5250 5450
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR016
U 1 1 58FF986D
P 1200 2900
F 0 "#PWR016" H 1200 2750 50  0001 C CNN
F 1 "+3.3V" H 1200 3040 50  0000 C CNN
F 2 "" H 1200 2900 50  0001 C CNN
F 3 "" H 1200 2900 50  0001 C CNN
	1    1200 2900
	1    0    0    -1  
$EndComp
$Comp
L CP C6
U 1 1 58FF98B1
P 1200 3150
F 0 "C6" H 1225 3250 50  0000 L CNN
F 1 "1uF" H 1225 3050 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D4.0mm_P2.00mm" H 1238 3000 50  0001 C CNN
F 3 "" H 1200 3150 50  0001 C CNN
	1    1200 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 2900 1200 3000
Wire Wire Line
	1200 3300 1450 3300
Wire Wire Line
	1450 3300 1450 3100
Wire Wire Line
	1450 3100 1600 3100
Connection ~ 1600 3100
$Comp
L C C4
U 1 1 58FF9A3F
P 950 3150
F 0 "C4" H 975 3250 50  0000 L CNN
F 1 "0.1uF" H 975 3050 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D3.0mm_W2.0mm_P2.50mm" H 988 3000 50  0001 C CNN
F 3 "" H 950 3150 50  0001 C CNN
	1    950  3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  3000 950  2950
Wire Wire Line
	950  2950 1200 2950
Connection ~ 1200 2950
Wire Wire Line
	950  3300 950  3400
Wire Wire Line
	950  3400 1300 3400
Wire Wire Line
	1300 3400 1300 3300
Connection ~ 1300 3300
$Comp
L C C5
U 1 1 58FFA83B
P 1000 1250
F 0 "C5" H 1025 1350 50  0000 L CNN
F 1 "0.1uF" H 1025 1150 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D3.0mm_W2.0mm_P2.50mm" H 1038 1100 50  0001 C CNN
F 3 "" H 1000 1250 50  0001 C CNN
	1    1000 1250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 58FFAA79
P 750 1800
F 0 "#PWR017" H 750 1550 50  0001 C CNN
F 1 "GND" H 750 1650 50  0000 C CNN
F 2 "" H 750 1800 50  0001 C CNN
F 3 "" H 750 1800 50  0001 C CNN
	1    750  1800
	1    0    0    -1  
$EndComp
$Comp
L SW_DIP_x01 SW1
U 1 1 58FFC1E7
P 750 1400
F 0 "SW1" H 750 1550 50  0000 C CNN
F 1 "SW_DIP_x01" H 750 1250 50  0000 C CNN
F 2 "Buttons_Switches_THT:SW_PUSH-12mm" H 750 1400 50  0001 C CNN
F 3 "" H 750 1400 50  0001 C CNN
	1    750  1400
	0    1    1    0   
$EndComp
Wire Wire Line
	750  1050 1300 1050
Wire Wire Line
	750  1050 750  1100
Wire Wire Line
	1000 1050 1000 1100
Connection ~ 1000 1050
Wire Wire Line
	1000 1400 1000 1750
Wire Wire Line
	750  1700 750  1800
Wire Wire Line
	1000 1750 750  1750
Connection ~ 750  1750
Wire Wire Line
	1800 1700 1700 1700
Wire Wire Line
	1700 1700 1700 1000
Wire Wire Line
	1800 1600 1700 1600
Connection ~ 1700 1600
Connection ~ 1700 1050
Wire Wire Line
	1300 1900 1800 1900
$Comp
L R R1
U 1 1 58FFD7A9
P 1300 1300
F 0 "R1" V 1380 1300 50  0000 C CNN
F 1 "10" V 1300 1300 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 1230 1300 50  0001 C CNN
F 3 "" H 1300 1300 50  0001 C CNN
	1    1300 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 1050 1700 1050
Wire Wire Line
	1300 1150 1200 1150
Wire Wire Line
	1200 1150 1200 1050
Connection ~ 1200 1050
Wire Wire Line
	1300 1450 1300 1900
$Comp
L +3.3V #PWR018
U 1 1 58FFE7B0
P 5900 5000
F 0 "#PWR018" H 5900 4850 50  0001 C CNN
F 1 "+3.3V" H 5900 5140 50  0000 C CNN
F 2 "" H 5900 5000 50  0001 C CNN
F 3 "" H 5900 5000 50  0001 C CNN
	1    5900 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 5050 5900 5000
$Comp
L CONN_01X03 J1
U 1 1 58FFFB01
P 750 4050
F 0 "J1" H 750 4250 50  0000 C CNN
F 1 "CONN_01X03" V 850 4050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 750 4050 50  0001 C CNN
F 3 "" H 750 4050 50  0001 C CNN
	1    750  4050
	-1   0    0    1   
$EndComp
$Comp
L +5V #PWR019
U 1 1 58FFFC13
P 1000 3850
F 0 "#PWR019" H 1000 3700 50  0001 C CNN
F 1 "+5V" H 1000 3990 50  0000 C CNN
F 2 "" H 1000 3850 50  0001 C CNN
F 3 "" H 1000 3850 50  0001 C CNN
	1    1000 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  3950 1000 3950
Wire Wire Line
	1000 3950 1000 3850
$Comp
L CONN_01X02 J10
U 1 1 58FF927E
P 9250 2650
F 0 "J10" H 9250 2800 50  0000 C CNN
F 1 "Encdr Pwr" V 9350 2650 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 9250 2650 50  0001 C CNN
F 3 "" H 9250 2650 50  0001 C CNN
	1    9250 2650
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR020
U 1 1 58FF945C
P 8950 2600
F 0 "#PWR020" H 8950 2450 50  0001 C CNN
F 1 "+5V" H 8950 2740 50  0000 C CNN
F 2 "" H 8950 2600 50  0001 C CNN
F 3 "" H 8950 2600 50  0001 C CNN
	1    8950 2600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR021
U 1 1 58FF9B34
P 8950 2700
F 0 "#PWR021" H 8950 2450 50  0001 C CNN
F 1 "GND" H 8950 2550 50  0000 C CNN
F 2 "" H 8950 2700 50  0001 C CNN
F 3 "" H 8950 2700 50  0001 C CNN
	1    8950 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 2700 9050 2700
$Comp
L CONN_01X03 J4
U 1 1 58FFAAF7
P 7000 5600
F 0 "J4" H 7000 5800 50  0000 C CNN
F 1 "CONN_01X03" V 7100 5600 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 7000 5600 50  0001 C CNN
F 3 "" H 7000 5600 50  0001 C CNN
	1    7000 5600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR022
U 1 1 58FFAD07
P 6650 5850
F 0 "#PWR022" H 6650 5600 50  0001 C CNN
F 1 "GND" H 6650 5700 50  0000 C CNN
F 2 "" H 6650 5850 50  0001 C CNN
F 3 "" H 6650 5850 50  0001 C CNN
	1    6650 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 5500 6650 5500
Wire Wire Line
	6650 5500 6650 5850
Wire Wire Line
	6800 5600 6650 5600
Connection ~ 6650 5600
Wire Wire Line
	6800 5700 6650 5700
Connection ~ 6650 5700
$Comp
L CONN_01X02 J12
U 1 1 58FFC2EC
P 8100 4600
F 0 "J12" H 8100 4750 50  0000 C CNN
F 1 "3v3 output" V 8200 4600 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 8100 4600 50  0001 C CNN
F 3 "" H 8100 4600 50  0001 C CNN
	1    8100 4600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR023
U 1 1 58FFC83D
P 7850 4700
F 0 "#PWR023" H 7850 4450 50  0001 C CNN
F 1 "GND" H 7850 4550 50  0000 C CNN
F 2 "" H 7850 4700 50  0001 C CNN
F 3 "" H 7850 4700 50  0001 C CNN
	1    7850 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 4650 7850 4650
Wire Wire Line
	7850 4650 7850 4700
Wire Wire Line
	7850 4500 7850 4550
Wire Wire Line
	7850 4550 7900 4550
Wire Wire Line
	8950 2600 9050 2600
$Comp
L +3.3V #PWR024
U 1 1 58FFD5B5
P 7850 4500
F 0 "#PWR024" H 7850 4350 50  0001 C CNN
F 1 "+3.3V" H 7850 4640 50  0000 C CNN
F 2 "" H 7850 4500 50  0001 C CNN
F 3 "" H 7850 4500 50  0001 C CNN
	1    7850 4500
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR025
U 1 1 58FFE172
P 6700 4500
F 0 "#PWR025" H 6700 4350 50  0001 C CNN
F 1 "+5V" H 6700 4640 50  0000 C CNN
F 2 "" H 6700 4500 50  0001 C CNN
F 3 "" H 6700 4500 50  0001 C CNN
	1    6700 4500
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 J5
U 1 1 58FFECEA
P 7600 4600
F 0 "J5" H 7600 4750 50  0000 C CNN
F 1 "5V output" V 7700 4600 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 7600 4600 50  0001 C CNN
F 3 "" H 7600 4600 50  0001 C CNN
	1    7600 4600
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR026
U 1 1 58FFECF0
P 7350 4500
F 0 "#PWR026" H 7350 4350 50  0001 C CNN
F 1 "+5V" H 7350 4640 50  0000 C CNN
F 2 "" H 7350 4500 50  0001 C CNN
F 3 "" H 7350 4500 50  0001 C CNN
	1    7350 4500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR027
U 1 1 58FFECF6
P 7350 4700
F 0 "#PWR027" H 7350 4450 50  0001 C CNN
F 1 "GND" H 7350 4550 50  0000 C CNN
F 2 "" H 7350 4700 50  0001 C CNN
F 3 "" H 7350 4700 50  0001 C CNN
	1    7350 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 4650 7350 4650
Wire Wire Line
	7350 4650 7350 4700
$Comp
L CONN_01X02 J13
U 1 1 58FFECFE
P 8100 5200
F 0 "J13" H 8100 5350 50  0000 C CNN
F 1 "3v3 output" V 8200 5200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 8100 5200 50  0001 C CNN
F 3 "" H 8100 5200 50  0001 C CNN
	1    8100 5200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR028
U 1 1 58FFED04
P 7850 5300
F 0 "#PWR028" H 7850 5050 50  0001 C CNN
F 1 "GND" H 7850 5150 50  0000 C CNN
F 2 "" H 7850 5300 50  0001 C CNN
F 3 "" H 7850 5300 50  0001 C CNN
	1    7850 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 5250 7850 5250
Wire Wire Line
	7850 5250 7850 5300
Wire Wire Line
	7850 5100 7850 5150
Wire Wire Line
	7850 5150 7900 5150
Wire Wire Line
	7350 4500 7350 4550
Wire Wire Line
	7350 4550 7400 4550
$Comp
L +3.3V #PWR029
U 1 1 58FFED10
P 7850 5100
F 0 "#PWR029" H 7850 4950 50  0001 C CNN
F 1 "+3.3V" H 7850 5240 50  0000 C CNN
F 2 "" H 7850 5100 50  0001 C CNN
F 3 "" H 7850 5100 50  0001 C CNN
	1    7850 5100
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 J6
U 1 1 58FFED8E
P 7600 5200
F 0 "J6" H 7600 5350 50  0000 C CNN
F 1 "5V output" V 7700 5200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 7600 5200 50  0001 C CNN
F 3 "" H 7600 5200 50  0001 C CNN
	1    7600 5200
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR030
U 1 1 58FFED94
P 7350 5100
F 0 "#PWR030" H 7350 4950 50  0001 C CNN
F 1 "+5V" H 7350 5240 50  0000 C CNN
F 2 "" H 7350 5100 50  0001 C CNN
F 3 "" H 7350 5100 50  0001 C CNN
	1    7350 5100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR031
U 1 1 58FFED9A
P 7350 5300
F 0 "#PWR031" H 7350 5050 50  0001 C CNN
F 1 "GND" H 7350 5150 50  0000 C CNN
F 2 "" H 7350 5300 50  0001 C CNN
F 3 "" H 7350 5300 50  0001 C CNN
	1    7350 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 5250 7350 5250
Wire Wire Line
	7350 5250 7350 5300
Wire Wire Line
	7350 5100 7350 5150
Wire Wire Line
	7350 5150 7400 5150
$Comp
L R R11
U 1 1 5900181F
P 7650 3700
F 0 "R11" V 7730 3700 50  0000 C CNN
F 1 "10K" V 7650 3700 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 7580 3700 50  0001 C CNN
F 3 "" H 7650 3700 50  0001 C CNN
	1    7650 3700
	1    0    0    -1  
$EndComp
$Comp
L R R12
U 1 1 5900193C
P 7850 3700
F 0 "R12" V 7930 3700 50  0000 C CNN
F 1 "10K" V 7850 3700 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 7780 3700 50  0001 C CNN
F 3 "" H 7850 3700 50  0001 C CNN
	1    7850 3700
	1    0    0    -1  
$EndComp
$Comp
L R R13
U 1 1 590019AB
P 8050 3700
F 0 "R13" V 8130 3700 50  0000 C CNN
F 1 "10K" V 8050 3700 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 7980 3700 50  0001 C CNN
F 3 "" H 8050 3700 50  0001 C CNN
	1    8050 3700
	1    0    0    -1  
$EndComp
$Comp
L R R14
U 1 1 59001A20
P 8250 3700
F 0 "R14" V 8330 3700 50  0000 C CNN
F 1 "10K" V 8250 3700 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 8180 3700 50  0001 C CNN
F 3 "" H 8250 3700 50  0001 C CNN
	1    8250 3700
	1    0    0    -1  
$EndComp
$Comp
L R R15
U 1 1 59001BC3
P 8450 3700
F 0 "R15" V 8530 3700 50  0000 C CNN
F 1 "10K" V 8450 3700 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 8380 3700 50  0001 C CNN
F 3 "" H 8450 3700 50  0001 C CNN
	1    8450 3700
	1    0    0    -1  
$EndComp
$Comp
L R R16
U 1 1 59001C38
P 8650 3700
F 0 "R16" V 8730 3700 50  0000 C CNN
F 1 "10K" V 8650 3700 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 8580 3700 50  0001 C CNN
F 3 "" H 8650 3700 50  0001 C CNN
	1    8650 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7650 3100 7650 3550
Wire Wire Line
	7850 3000 7850 3550
Wire Wire Line
	8050 2900 8050 3550
Wire Wire Line
	8250 2800 8250 3550
Wire Wire Line
	8450 2700 8450 3550
Wire Wire Line
	8650 2600 8650 3550
Wire Wire Line
	7650 3850 7650 3950
Wire Wire Line
	7650 3950 8650 3950
Wire Wire Line
	8650 3950 8650 3850
Wire Wire Line
	7850 3850 7850 3950
Connection ~ 7850 3950
Wire Wire Line
	8050 3850 8050 3950
Connection ~ 8050 3950
Wire Wire Line
	8250 3850 8250 3950
Connection ~ 8250 3950
Wire Wire Line
	8450 3850 8450 3950
Connection ~ 8450 3950
$Comp
L GND #PWR032
U 1 1 59002B5E
P 8150 4000
F 0 "#PWR032" H 8150 3750 50  0001 C CNN
F 1 "GND" H 8150 3850 50  0000 C CNN
F 2 "" H 8150 4000 50  0001 C CNN
F 3 "" H 8150 4000 50  0001 C CNN
	1    8150 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 3950 8150 4000
Connection ~ 8150 3950
NoConn ~ 9050 2000
$Comp
L PWR_FLAG #FLG033
U 1 1 590068F5
P 6850 4800
F 0 "#FLG033" H 6850 4875 50  0001 C CNN
F 1 "PWR_FLAG" H 6850 4950 50  0000 C CNN
F 2 "" H 6850 4800 50  0001 C CNN
F 3 "" H 6850 4800 50  0001 C CNN
	1    6850 4800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR034
U 1 1 5900695D
P 6850 4850
F 0 "#PWR034" H 6850 4600 50  0001 C CNN
F 1 "GND" H 6850 4700 50  0000 C CNN
F 2 "" H 6850 4850 50  0001 C CNN
F 3 "" H 6850 4850 50  0001 C CNN
	1    6850 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 4800 6850 4850
$Comp
L PWR_FLAG #FLG035
U 1 1 590072C0
P 7000 4500
F 0 "#FLG035" H 7000 4575 50  0001 C CNN
F 1 "PWR_FLAG" H 7000 4650 50  0000 C CNN
F 2 "" H 7000 4500 50  0001 C CNN
F 3 "" H 7000 4500 50  0001 C CNN
	1    7000 4500
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 J3
U 1 1 59008BE5
P 6800 5200
F 0 "J3" H 6800 5350 50  0000 C CNN
F 1 "IO" V 6900 5200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 6800 5200 50  0001 C CNN
F 3 "" H 6800 5200 50  0001 C CNN
	1    6800 5200
	-1   0    0    1   
$EndComp
Wire Wire Line
	1800 2600 1700 2600
Wire Wire Line
	7600 2000 7750 2000
Text Label 1700 2600 0    60   ~ 0
IO2
Text Label 7750 2000 0    60   ~ 0
IO1
Text Label 7000 5150 0    60   ~ 0
IO1
Text Label 7000 5250 0    60   ~ 0
IO2
$Comp
L CONN_01X08 J11
U 1 1 59027E78
P 9250 3250
F 0 "J11" H 9250 3700 50  0000 C CNN
F 1 "3ph board" V 9350 3250 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x08_Pitch2.54mm" H 9250 3250 50  0001 C CNN
F 3 "" H 9250 3250 50  0001 C CNN
	1    9250 3250
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR036
U 1 1 5902853C
P 9000 3650
F 0 "#PWR036" H 9000 3500 50  0001 C CNN
F 1 "+5V" H 9000 3790 50  0000 C CNN
F 2 "" H 9000 3650 50  0001 C CNN
F 3 "" H 9000 3650 50  0001 C CNN
	1    9000 3650
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR037
U 1 1 5902869A
P 8850 3650
F 0 "#PWR037" H 8850 3400 50  0001 C CNN
F 1 "GND" H 8850 3500 50  0000 C CNN
F 2 "" H 8850 3650 50  0001 C CNN
F 3 "" H 8850 3650 50  0001 C CNN
	1    8850 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 3600 9000 3600
Wire Wire Line
	9000 3600 9000 3650
Wire Wire Line
	9050 3500 8850 3500
Wire Wire Line
	8850 3500 8850 3650
Wire Wire Line
	6700 4500 6700 4550
Wire Wire Line
	6700 4550 7000 4550
Wire Wire Line
	7000 4550 7000 4500
Wire Wire Line
	8400 1900 9050 1900
Wire Wire Line
	8350 1800 9050 1800
Connection ~ 7650 3400
Connection ~ 7850 3300
Connection ~ 8050 3200
Connection ~ 8250 3100
Connection ~ 8450 3000
Connection ~ 8650 2900
Wire Wire Line
	7600 2500 8550 2500
Wire Wire Line
	8550 2500 8550 2400
Wire Wire Line
	8550 2400 9050 2400
Wire Wire Line
	7600 2400 8500 2400
Wire Wire Line
	8500 2400 8500 2300
Wire Wire Line
	8500 2300 9050 2300
Wire Wire Line
	9050 2200 8450 2200
Wire Wire Line
	8450 2200 8450 2300
Wire Wire Line
	8450 2300 7600 2300
Wire Wire Line
	7600 2200 8400 2200
Wire Wire Line
	8400 2200 8400 1900
Wire Wire Line
	7600 2100 8350 2100
Wire Wire Line
	8350 2100 8350 1800
Wire Wire Line
	8400 1350 8400 1300
Wire Wire Line
	8400 1300 9050 1300
Wire Wire Line
	9050 1200 8250 1200
Wire Wire Line
	8250 1200 8250 1900
Wire Wire Line
	8250 1900 7600 1900
Wire Wire Line
	9050 1100 8200 1100
Wire Wire Line
	8200 1100 8200 1800
Wire Wire Line
	8200 1800 7600 1800
Wire Wire Line
	9050 3400 7650 3400
Wire Wire Line
	7850 3300 9050 3300
Wire Wire Line
	8050 3200 9050 3200
Wire Wire Line
	8250 3100 9050 3100
Wire Wire Line
	8450 3000 9050 3000
Wire Wire Line
	8650 2900 9050 2900
Wire Wire Line
	7600 3100 7650 3100
Wire Wire Line
	7600 3000 7850 3000
Wire Wire Line
	7600 2900 8050 2900
Wire Wire Line
	7600 2800 8250 2800
Wire Wire Line
	7600 2700 8450 2700
Wire Wire Line
	7600 2600 8650 2600
$EndSCHEMATC
