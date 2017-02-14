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
LIBS:ss461r
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
L SS461R U1
U 1 1 55AC4F61
P 2800 2400
F 0 "U1" H 2800 2500 60  0000 C CNN
F 1 "SS461R" H 2800 2400 60  0000 C CNN
F 2 "Housings_TO-92:TO-92_Inline_Narrow_Oval" H 2800 2400 60  0001 C CNN
F 3 "" H 2800 2400 60  0000 C CNN
	1    2800 2400
	1    0    0    -1  
$EndComp
$Comp
L SS461R U2
U 1 1 55AC4F77
P 3800 2400
F 0 "U2" H 3800 2500 60  0000 C CNN
F 1 "SS461R" H 3800 2400 60  0000 C CNN
F 2 "Housings_TO-92:TO-92_Inline_Narrow_Oval" H 3800 2400 60  0001 C CNN
F 3 "" H 3800 2400 60  0000 C CNN
	1    3800 2400
	1    0    0    -1  
$EndComp
$Comp
L SS461R U3
U 1 1 55AC4FCD
P 4600 2450
F 0 "U3" H 4600 2550 60  0000 C CNN
F 1 "SS461R" H 4600 2450 60  0000 C CNN
F 2 "Housings_TO-92:TO-92_Inline_Narrow_Oval" H 4600 2450 60  0001 C CNN
F 3 "" H 4600 2450 60  0000 C CNN
	1    4600 2450
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X03 P1
U 1 1 55AC53C2
P 2800 3300
F 0 "P1" H 2800 3500 50  0000 C CNN
F 1 "CONN_01X03" V 2900 3300 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 2800 3300 60  0001 C CNN
F 3 "" H 2800 3300 60  0000 C CNN
	1    2800 3300
	0    1    1    0   
$EndComp
$Comp
L CONN_01X03 P2
U 1 1 55AC544B
P 3800 3300
F 0 "P2" H 3800 3500 50  0000 C CNN
F 1 "CONN_01X03" V 3900 3300 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 3800 3300 60  0001 C CNN
F 3 "" H 3800 3300 60  0000 C CNN
	1    3800 3300
	0    1    1    0   
$EndComp
$Comp
L CONN_01X03 P3
U 1 1 55AC548E
P 4600 3300
F 0 "P3" H 4600 3500 50  0000 C CNN
F 1 "CONN_01X03" V 4700 3300 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 4600 3300 60  0001 C CNN
F 3 "" H 4600 3300 60  0000 C CNN
	1    4600 3300
	0    1    1    0   
$EndComp
Wire Wire Line
	2700 2950 2700 3100
Wire Wire Line
	2800 2950 2800 3100
Wire Wire Line
	2900 2950 2900 3100
Wire Wire Line
	3700 2950 3700 3100
Wire Wire Line
	3800 2950 3800 3100
Wire Wire Line
	3900 2950 3900 3100
Wire Wire Line
	4500 3000 4500 3100
Wire Wire Line
	4600 3000 4600 3100
Wire Wire Line
	4700 3000 4700 3100
$Comp
L CONN_01X03 P6
U 1 1 55AC56F0
P 7100 4450
F 0 "P6" H 7100 4650 50  0000 C CNN
F 1 "CONN_01X03" V 7200 4450 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 7100 4450 60  0001 C CNN
F 3 "" H 7100 4450 60  0000 C CNN
	1    7100 4450
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X03 P5
U 1 1 55AC5727
P 7050 3750
F 0 "P5" H 7050 3950 50  0000 C CNN
F 1 "CONN_01X03" V 7150 3750 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 7050 3750 60  0001 C CNN
F 3 "" H 7050 3750 60  0000 C CNN
	1    7050 3750
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X03 P4
U 1 1 55AC5786
P 7050 3150
F 0 "P4" H 7050 3350 50  0000 C CNN
F 1 "CONN_01X03" V 7150 3150 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 7050 3150 60  0001 C CNN
F 3 "" H 7050 3150 60  0000 C CNN
	1    7050 3150
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X02 P7
U 1 1 55AC57CD
P 8300 3100
F 0 "P7" H 8300 3250 50  0000 C CNN
F 1 "CONN_01X02" V 8400 3100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 8300 3100 60  0001 C CNN
F 3 "" H 8300 3100 60  0000 C CNN
	1    8300 3100
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X03 P8
U 1 1 55AC5846
P 8300 3700
F 0 "P8" H 8300 3900 50  0000 C CNN
F 1 "CONN_01X03" V 8400 3700 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 8300 3700 60  0001 C CNN
F 3 "" H 8300 3700 60  0000 C CNN
	1    8300 3700
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 55AC58D6
P 8000 3350
F 0 "R3" V 8080 3350 50  0000 C CNN
F 1 "330" V 8000 3350 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM15mm" V 7930 3350 30  0001 C CNN
F 3 "" H 8000 3350 30  0000 C CNN
	1    8000 3350
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 55AC592F
P 7800 3350
F 0 "R2" V 7880 3350 50  0000 C CNN
F 1 "330" V 7800 3350 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM15mm" V 7730 3350 30  0001 C CNN
F 3 "" H 7800 3350 30  0000 C CNN
	1    7800 3350
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 55AC595E
P 7600 3350
F 0 "R1" V 7680 3350 50  0000 C CNN
F 1 "330" V 7600 3350 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM15mm" V 7530 3350 30  0001 C CNN
F 3 "" H 7600 3350 30  0000 C CNN
	1    7600 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 3150 8100 3150
Wire Wire Line
	7250 3750 7400 3750
Wire Wire Line
	7400 3150 7400 4450
Connection ~ 7400 3150
Wire Wire Line
	7400 4450 7300 4450
Connection ~ 7400 3750
Wire Wire Line
	7300 3050 7300 4350
Wire Wire Line
	7300 3650 7250 3650
Wire Wire Line
	7250 3050 8100 3050
Connection ~ 7300 3650
Connection ~ 7300 3050
Wire Wire Line
	7600 3200 7600 3050
Connection ~ 7600 3050
Wire Wire Line
	7800 3200 7800 3050
Connection ~ 7800 3050
Wire Wire Line
	8000 3200 8000 3050
Connection ~ 8000 3050
Wire Wire Line
	7250 3250 7500 3250
Wire Wire Line
	7500 3250 7500 3600
Wire Wire Line
	7500 3600 8100 3600
Wire Wire Line
	7250 3850 7500 3850
Wire Wire Line
	7500 3850 7500 3700
Wire Wire Line
	7500 3700 8100 3700
Wire Wire Line
	7300 4550 7600 4550
Wire Wire Line
	7600 4550 7600 3800
Wire Wire Line
	7600 3800 8100 3800
Wire Wire Line
	7600 3500 7600 3600
Connection ~ 7600 3600
Wire Wire Line
	7800 3500 7800 3700
Connection ~ 7800 3700
Wire Wire Line
	8000 3500 8000 3800
Connection ~ 8000 3800
$EndSCHEMATC
