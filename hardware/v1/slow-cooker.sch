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
LIBS:SSR_Crydom
LIBS:SMPS_RS
LIBS:te-connectivity
LIBS:wurth
LIBS:epcos
LIBS:slow-cooker-cache
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
L Earth_Protective #PWR12
U 1 1 594E8ED1
P 9200 1175
F 0 "#PWR12" H 9450 925 50  0001 C CNN
F 1 "Earth_Protective" H 9650 1025 50  0001 C CNN
F 2 "" H 9200 1075 50  0000 C CNN
F 3 "" H 9200 1075 50  0000 C CNN
	1    9200 1175
	-1   0    0    1   
$EndComp
$Comp
L CONN_02X02 P1
U 1 1 594E90E8
P 1125 1350
F 0 "P1" H 1125 1550 50  0000 C CNN
F 1 "Input" H 1125 1200 50  0000 C CNN
F 2 "Connectors_Molex:Molex_MiniFit-JR-5556-04A_2x02x4.20mm_Straight" H 1125 1350 50  0001 C CNN
F 3 "" H 1125 1350 50  0000 C CNN
	1    1125 1350
	-1   0    0    -1  
$EndComp
$Comp
L Earth_Protective #PWR4
U 1 1 594E97BB
P 2875 2100
F 0 "#PWR4" H 3125 1850 50  0001 C CNN
F 1 "Earth_Protective" H 3325 1950 50  0001 C CNN
F 2 "" H 2875 2000 50  0000 C CNN
F 3 "" H 2875 2000 50  0000 C CNN
	1    2875 2100
	1    0    0    -1  
$EndComp
Text Notes 8425 3875 0    60   ~ 0
2.8" TFT display with resistive touch\nhttp://www.ebay.co.uk/itm/2-8-240x320-SPI-TFT-LCD-Touch-Panel-Serial-Port-Module-PCB-ILI9341-5V-3-3V-Red-/401332638377\n\nDriver library: https://github.com/adafruit/Adafruit_ILI9341\n\nhttp://www.instructables.com/id/Cheap-TFT-22-inch-Display-on-Arduino-ILI9340C-or-I/
Text Label 1500 1200 0    60   ~ 0
Live
Text Label 1500 1400 0    60   ~ 0
Neutral
$Comp
L ATMEGA328-A IC1
U 1 1 594EA219
P 3350 5650
F 0 "IC1" H 2600 6900 50  0000 L BNN
F 1 "ATMEGA328-AN" H 3750 4250 50  0000 L BNN
F 2 "Housings_QFP:TQFP-32_7x7mm_Pitch0.8mm" H 3350 5650 50  0001 C CIN
F 3 "" H 3350 5650 50  0000 C CNN
F 4 "131-0270" H 3350 5650 60  0001 C CNN "RS"
	1    3350 5650
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X14 P15
U 1 1 594F92DD
P 10000 4900
F 0 "P15" H 10000 5650 50  0000 C CNN
F 1 "LCD 1" V 10100 4900 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x14" H 10000 4900 50  0001 C CNN
F 3 "" H 10000 4900 50  0000 C CNN
	1    10000 4900
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 P16
U 1 1 594F93F3
P 10000 6000
F 0 "P16" H 10000 6250 50  0000 C CNN
F 1 "LCD 2" V 10100 6000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04" H 10000 6000 50  0001 C CNN
F 3 "" H 10000 6000 50  0000 C CNN
	1    10000 6000
	1    0    0    -1  
$EndComp
Text Label 9600 5850 2    60   ~ 0
SD_CS
Text Label 9600 5950 2    60   ~ 0
SPI_MOSI
Text Label 9600 6050 2    60   ~ 0
SPI_MISO
Text Label 9600 6150 2    60   ~ 0
SPI_SCK
Text Label 9600 4350 2    60   ~ 0
GND
Text Label 9600 4450 2    60   ~ 0
TFT_CS
Text Label 9600 4550 2    60   ~ 0
~RESET
Text Label 9600 4650 2    60   ~ 0
TFT_D/C
Text Label 9600 4750 2    60   ~ 0
SPI_MOSI
Text Label 9600 4850 2    60   ~ 0
SPI_SCK
Text Label 9600 4950 2    60   ~ 0
LED
Text Label 9600 5050 2    60   ~ 0
SPI_MISO
Text Label 9600 5150 2    60   ~ 0
SPI_SCK
Text Label 9600 5250 2    60   ~ 0
T_CS
Text Label 9600 5550 2    60   ~ 0
T_IRQ
$Comp
L LS01-15B05S U2
U 1 1 594FAABE
P 5725 1550
F 0 "U2" H 5475 1200 60  0000 C CNN
F 1 "LS01-15B05S" H 5725 1900 60  0000 C CNN
F 2 "SMPS_RS:RS_Pro_LS01" H 5825 1550 60  0001 C CNN
F 3 "" H 5825 1550 60  0001 C CNN
F 4 "771-9354" H 5725 1550 60  0001 C CNN "RS"
	1    5725 1550
	1    0    0    -1  
$EndComp
$Comp
L CP_Small C10
U 1 1 594FB0BD
P 5125 1550
F 0 "C10" H 5135 1620 50  0000 L CNN
F 1 "10u/400V" V 5000 1375 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Radial_D12.5_L25_P5" H 5125 1550 50  0001 C CNN
F 3 "" H 5125 1550 50  0000 C CNN
	1    5125 1550
	1    0    0    -1  
$EndComp
$Comp
L C_Small C11
U 1 1 594FB379
P 5725 2075
F 0 "C11" V 5600 2050 50  0000 L CNN
F 1 "1n/400V/Y" V 5850 1875 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Rect_L18_W5_P15" H 5725 2075 50  0001 C CNN
F 3 "" H 5725 2075 50  0000 C CNN
F 4 "882-9528" V 5725 2075 60  0001 C CNN "RS"
	1    5725 2075
	0    1    1    0   
$EndComp
$Comp
L CP_Small C12
U 1 1 594FB66B
P 6325 1550
F 0 "C12" H 6335 1620 50  0000 L CNN
F 1 "150u" V 6200 1450 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Radial_D8_L11.5_P3.5" H 6325 1550 50  0001 C CNN
F 3 "" H 6325 1550 50  0000 C CNN
F 4 "315-0704" H 6325 1550 60  0001 C CNN "RS"
	1    6325 1550
	1    0    0    -1  
$EndComp
$Comp
L CP_Small C13
U 1 1 594FB8B2
P 6825 1550
F 0 "C13" H 6835 1620 50  0000 L CNN
F 1 "68u" V 6700 1400 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Radial_D6.3_L11.2_P2.5" H 6825 1550 50  0001 C CNN
F 3 "" H 6825 1550 50  0000 C CNN
F 4 "526-1907" H 6825 1550 60  0001 C CNN "RS"
	1    6825 1550
	1    0    0    -1  
$EndComp
$Comp
L L_Small L2
U 1 1 594FBC49
P 6575 1300
F 0 "L2" V 6650 1250 50  0000 L CNN
F 1 "2.2u" V 6525 1225 50  0000 L CNN
F 2 "Choke_Axial_ThroughHole:Choke_Horizontal_RM10mm" H 6575 1300 50  0001 C CNN
F 3 "" H 6575 1300 50  0000 C CNN
F 4 "191-0425" V 6575 1300 60  0001 C CNN "RS"
	1    6575 1300
	0    -1   -1   0   
$EndComp
$Comp
L D_Small D1
U 1 1 594FC376
P 7125 1550
F 0 "D1" V 7050 1575 50  0000 L CNN
F 1 "SMBJ7.0A" H 6950 1475 50  0000 L CNN
F 2 "Diodes_SMD:SMB_Handsoldering" V 7125 1550 50  0001 C CNN
F 3 "" V 7125 1550 50  0000 C CNN
F 4 "885-5643" V 7125 1550 60  0001 C CNN "RS"
	1    7125 1550
	0    1    1    0   
$EndComp
$Comp
L C_Small C16
U 1 1 594FC568
P 7425 1550
F 0 "C16" H 7435 1620 50  0000 L CNN
F 1 "100n" V 7300 1375 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 7425 1550 50  0001 C CNN
F 3 "" H 7425 1550 50  0000 C CNN
	1    7425 1550
	1    0    0    -1  
$EndComp
$Comp
L R_Small R7
U 1 1 594FCB47
P 7725 1550
F 0 "R7" H 7775 1625 50  0000 L CNN
F 1 "NP" H 7775 1475 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 7725 1550 50  0001 C CNN
F 3 "" H 7725 1550 50  0000 C CNN
	1    7725 1550
	1    0    0    -1  
$EndComp
$Comp
L C_Small C7
U 1 1 594FF674
P 4675 1375
F 0 "C7" V 4550 1350 50  0000 L CNN
F 1 "1n/400V/Y" V 4475 1075 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Rect_L18_W5_P15" H 4675 1375 50  0001 C CNN
F 3 "" H 4675 1375 50  0000 C CNN
F 4 "882-9528" V 4675 1375 60  0001 C CNN "RS"
	1    4675 1375
	-1   0    0    1   
$EndComp
$Comp
L C_Small C8
U 1 1 594FF9F4
P 4675 1725
F 0 "C8" V 4550 1700 50  0000 L CNN
F 1 "1n/400V/Y" V 4475 1575 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Rect_L18_W5_P15" H 4675 1725 50  0001 C CNN
F 3 "" H 4675 1725 50  0000 C CNN
F 4 "882-9528" V 4675 1725 60  0001 C CNN "RS"
	1    4675 1725
	-1   0    0    1   
$EndComp
Text Label 8125 1300 0    60   ~ 0
5V
$Comp
L GND #PWR11
U 1 1 59503E00
P 8125 1800
F 0 "#PWR11" H 8125 1550 50  0001 C CNN
F 1 "GND" H 8125 1650 50  0000 C CNN
F 2 "" H 8125 1800 50  0000 C CNN
F 3 "" H 8125 1800 50  0000 C CNN
	1    8125 1800
	1    0    0    -1  
$EndComp
Text Label 10775 1900 0    60   ~ 0
Live
Text Label 10775 1700 0    60   ~ 0
Neutral
Text Label 1300 2800 2    60   ~ 0
5V
$Comp
L GND #PWR3
U 1 1 5950785D
P 2275 3275
F 0 "#PWR3" H 2275 3025 50  0001 C CNN
F 1 "GND" H 2275 3125 50  0000 C CNN
F 2 "" H 2275 3275 50  0000 C CNN
F 3 "" H 2275 3275 50  0000 C CNN
	1    2275 3275
	1    0    0    -1  
$EndComp
$Comp
L LT1761-3.3 U1
U 1 1 59509723
P 2275 2900
F 0 "U1" H 2275 3225 50  0000 C CNN
F 1 "LT1761-3.3" H 2275 3150 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-5" H 2325 2650 50  0001 L CNN
F 3 "" H 2275 2900 50  0000 C CNN
F 4 "396-471" H 2275 2900 60  0001 C CNN "RS"
	1    2275 2900
	1    0    0    -1  
$EndComp
$Comp
L CP_Small C2
U 1 1 5950ADB5
P 1450 3000
F 0 "C2" H 1460 3070 50  0000 L CNN
F 1 "10u" H 1460 2920 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Radial_D5_L11_P2" H 1450 3000 50  0001 C CNN
F 3 "" H 1450 3000 50  0000 C CNN
	1    1450 3000
	1    0    0    -1  
$EndComp
$Comp
L C_Small C6
U 1 1 5950AFB7
P 2850 2900
F 0 "C6" H 2860 2970 50  0000 L CNN
F 1 "10n" H 2860 2820 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 2850 2900 50  0001 C CNN
F 3 "" H 2850 2900 50  0000 C CNN
	1    2850 2900
	0    1    1    0   
$EndComp
Text Label 3400 2800 0    60   ~ 0
3V3
Text Label 9600 4250 2    60   ~ 0
3V3
$Comp
L CONN_01X03 P17
U 1 1 59514B44
P 6850 3200
F 0 "P17" H 6850 3400 50  0000 C CNN
F 1 "Temperature sensor" V 6950 3200 50  0000 C CNN
F 2 "Terminal_Blocks:TerminalBlock_Pheonix_PT-3.5mm_3pol" H 6850 3200 50  0001 C CNN
F 3 "" H 6850 3200 50  0000 C CNN
	1    6850 3200
	1    0    0    -1  
$EndComp
Text Notes 3000 3950 0    60   ~ 0
Enclosure: Hammond 1455N1202\nRS http://uk.rs-online.com/web/p/general-purpose-enclosures/7733072/\nhttp://www.hammondmfg.com/pdf/1455N1202.pdf
Text Label 1250 4550 2    60   ~ 0
3V3
$Comp
L C_Small C3
U 1 1 5951B9FA
P 1625 4750
F 0 "C3" H 1635 4820 50  0000 L CNN
F 1 "10n" H 1635 4670 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 1625 4750 50  0001 C CNN
F 3 "" H 1625 4750 50  0000 C CNN
	1    1625 4750
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR1
U 1 1 5951C39C
P 1250 4950
F 0 "#PWR1" H 1250 4700 50  0001 C CNN
F 1 "GND" H 1250 4800 50  0000 C CNN
F 2 "" H 1250 4950 50  0000 C CNN
F 3 "" H 1250 4950 50  0000 C CNN
	1    1250 4950
	0    1    1    0   
$EndComp
$Comp
L GND #PWR2
U 1 1 5951CA4A
P 2250 6750
F 0 "#PWR2" H 2250 6500 50  0001 C CNN
F 1 "GND" H 2250 6600 50  0000 C CNN
F 2 "" H 2250 6750 50  0000 C CNN
F 3 "" H 2250 6750 50  0000 C CNN
	1    2250 6750
	0    1    1    0   
$EndComp
Text Label 4450 4850 0    60   ~ 0
SPI_MOSI
Text Label 4450 4950 0    60   ~ 0
SPI_MISO
Text Label 4450 5050 0    60   ~ 0
SPI_SCK
Text Label 6500 3100 2    60   ~ 0
3V3
$Comp
L GND #PWR7
U 1 1 5951F3AF
P 6500 3300
F 0 "#PWR7" H 6500 3050 50  0001 C CNN
F 1 "GND" H 6500 3150 50  0000 C CNN
F 2 "" H 6500 3300 50  0000 C CNN
F 3 "" H 6500 3300 50  0000 C CNN
	1    6500 3300
	0    1    1    0   
$EndComp
Text Label 6500 3200 2    60   ~ 0
DS18B20
Text Label 5575 6350 0    60   ~ 0
DS18B20
$Comp
L R_Small R6
U 1 1 59520135
P 6075 6200
F 0 "R6" H 6125 6275 50  0000 L CNN
F 1 "4k7" H 6125 6125 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 6075 6200 50  0001 C CNN
F 3 "" H 6075 6200 50  0000 C CNN
	1    6075 6200
	1    0    0    -1  
$EndComp
Text Label 6075 6050 2    60   ~ 0
3V3
Text Label 4450 6650 0    60   ~ 0
TFT_D/C
Text Label 5000 6750 0    60   ~ 0
TFT_CS
Text Notes 7275 6200 0    60   ~ 0
Driver for touchscreen:\nhttps://github.com/spapadim/XPT2046
Text Label 9600 5350 2    60   ~ 0
SPI_MOSI
Text Label 9600 5450 2    60   ~ 0
SPI_MISO
Text Label 4450 4550 0    60   ~ 0
T_IRQ
Text Label 5000 6850 0    60   ~ 0
T_CS
$Comp
L R_Small R1
U 1 1 59505A52
P 5100 5200
F 0 "R1" H 5150 5275 50  0000 L CNN
F 1 "1M" H 5150 5125 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 5100 5200 50  0001 C CNN
F 3 "" H 5100 5200 50  0000 C CNN
	1    5100 5200
	1    0    0    -1  
$EndComp
$Comp
L C_Small C14
U 1 1 59505CD2
P 5650 4950
F 0 "C14" H 5660 5020 50  0000 L CNN
F 1 "12p" H 5660 4870 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 5650 4950 50  0001 C CNN
F 3 "" H 5650 4950 50  0000 C CNN
	1    5650 4950
	0    -1   -1   0   
$EndComp
$Comp
L C_Small C15
U 1 1 59505DF8
P 5650 5450
F 0 "C15" H 5660 5520 50  0000 L CNN
F 1 "12p" H 5660 5370 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 5650 5450 50  0001 C CNN
F 3 "" H 5650 5450 50  0000 C CNN
	1    5650 5450
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR6
U 1 1 59506260
P 5950 5200
F 0 "#PWR6" H 5950 4950 50  0001 C CNN
F 1 "GND" H 5950 5050 50  0000 C CNN
F 2 "" H 5950 5200 50  0000 C CNN
F 3 "" H 5950 5200 50  0000 C CNN
	1    5950 5200
	0    -1   -1   0   
$EndComp
$Comp
L R_Small R5
U 1 1 59506C95
P 5675 5850
F 0 "R5" H 5725 5925 50  0000 L CNN
F 1 "10k" H 5725 5775 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 5675 5850 50  0001 C CNN
F 3 "" H 5675 5850 50  0000 C CNN
	1    5675 5850
	1    0    0    -1  
$EndComp
Text Label 5675 5700 2    60   ~ 0
3V3
Text Label 5275 6000 0    60   ~ 0
~RESET
$Comp
L CONN_02X03 P12
U 1 1 59507725
P 6700 4325
F 0 "P12" H 6700 4525 50  0000 C CNN
F 1 "ICSP" H 6700 4125 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03" H 6700 3125 50  0001 C CNN
F 3 "" H 6700 3125 50  0000 C CNN
	1    6700 4325
	1    0    0    -1  
$EndComp
Text Label 6400 4225 2    60   ~ 0
SPI_MISO
Text Label 6400 4325 2    60   ~ 0
SPI_SCK
Text Label 6400 4425 2    60   ~ 0
~RESET
Text Label 7000 4225 0    60   ~ 0
3V3
Text Label 7000 4325 0    60   ~ 0
SPI_MOSI
$Comp
L GND #PWR8
U 1 1 595085CB
P 7000 4425
F 0 "#PWR8" H 7000 4175 50  0001 C CNN
F 1 "GND" H 7000 4275 50  0000 C CNN
F 2 "" H 7000 4425 50  0000 C CNN
F 3 "" H 7000 4425 50  0000 C CNN
	1    7000 4425
	0    -1   -1   0   
$EndComp
Text Label 4850 4650 0    60   ~ 0
SD_CS
Text Label 4450 6150 0    60   ~ 0
USART_RXD
Text Label 4450 6250 0    60   ~ 0
USART_TXD
Text Label 4450 6550 0    60   ~ 0
USART_XCK
$Comp
L CONN_01X06 P14
U 1 1 5950D95F
P 8300 4700
F 0 "P14" H 8300 5050 50  0000 C CNN
F 1 "FTDI" V 8400 4700 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06" H 8300 4700 50  0001 C CNN
F 3 "" H 8300 4700 50  0000 C CNN
	1    8300 4700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR10
U 1 1 5950DF65
P 7775 4450
F 0 "#PWR10" H 7775 4200 50  0001 C CNN
F 1 "GND" H 7775 4300 50  0000 C CNN
F 2 "" H 7775 4450 50  0000 C CNN
F 3 "" H 7775 4450 50  0000 C CNN
	1    7775 4450
	0    1    1    0   
$EndComp
Text Label 8050 4550 2    60   ~ 0
3V3
NoConn ~ 8100 4650
Text Label 8050 4750 2    60   ~ 0
USART_TXD
Text Label 8050 4850 2    60   ~ 0
USART_RXD
$Comp
L C_Small C17
U 1 1 5950F851
P 7875 4950
F 0 "C17" V 7800 5050 50  0000 L CNN
F 1 "100n" V 7800 4725 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 7875 4950 50  0001 C CNN
F 3 "" H 7875 4950 50  0000 C CNN
	1    7875 4950
	0    -1   -1   0   
$EndComp
Text Label 7700 4950 2    60   ~ 0
~RESET
$Comp
L MMBT3904 Q1
U 1 1 5957BD7D
P 9650 2525
F 0 "Q1" H 9850 2600 50  0000 L CNN
F 1 "MMBT3904" H 9850 2525 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 9850 2450 50  0001 L CIN
F 3 "" H 9650 2525 50  0000 L CNN
	1    9650 2525
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR13
U 1 1 5957D518
P 9750 2925
F 0 "#PWR13" H 9750 2675 50  0001 C CNN
F 1 "GND" H 9750 2775 50  0000 C CNN
F 2 "" H 9750 2925 50  0000 C CNN
F 3 "" H 9750 2925 50  0000 C CNN
	1    9750 2925
	1    0    0    -1  
$EndComp
Wire Wire Line
	1375 1400 2425 1400
Wire Wire Line
	9800 4250 9600 4250
Wire Wire Line
	9800 4350 9600 4350
Wire Wire Line
	9800 4450 9600 4450
Wire Wire Line
	9800 4550 9600 4550
Wire Wire Line
	9800 4650 9600 4650
Wire Wire Line
	9800 4750 9600 4750
Wire Wire Line
	9800 4850 9600 4850
Wire Wire Line
	9800 4950 9600 4950
Wire Wire Line
	9800 5050 9600 5050
Wire Wire Line
	9800 5150 9600 5150
Wire Wire Line
	9800 5250 9600 5250
Wire Wire Line
	9800 5350 9600 5350
Wire Wire Line
	9800 5450 9600 5450
Wire Wire Line
	9800 5550 9600 5550
Wire Wire Line
	9800 5850 9600 5850
Wire Wire Line
	9800 5950 9600 5950
Wire Wire Line
	9800 6050 9600 6050
Wire Wire Line
	9800 6150 9600 6150
Wire Wire Line
	5125 1450 5125 1400
Wire Wire Line
	5125 1400 5225 1400
Wire Wire Line
	5125 1650 5125 2075
Wire Wire Line
	5125 1700 5225 1700
Wire Wire Line
	5125 2075 5625 2075
Connection ~ 5125 1700
Wire Wire Line
	6325 2075 5825 2075
Wire Wire Line
	6325 1650 6325 2075
Wire Wire Line
	6225 1800 8125 1800
Wire Wire Line
	6325 1450 6325 1300
Wire Wire Line
	6225 1300 6475 1300
Wire Wire Line
	6825 1300 6825 1450
Connection ~ 6325 1300
Connection ~ 6325 1800
Wire Wire Line
	6825 1800 6825 1650
Wire Wire Line
	6675 1300 8125 1300
Wire Wire Line
	7125 1300 7125 1450
Connection ~ 6825 1300
Wire Wire Line
	7125 1800 7125 1650
Connection ~ 6825 1800
Wire Wire Line
	7425 1300 7425 1450
Connection ~ 7125 1300
Wire Wire Line
	7425 1800 7425 1650
Connection ~ 7125 1800
Wire Wire Line
	7725 1300 7725 1450
Connection ~ 7425 1300
Wire Wire Line
	7725 1800 7725 1650
Connection ~ 7425 1800
Wire Wire Line
	2425 1400 2425 1900
Wire Wire Line
	4675 1475 4675 1625
Wire Wire Line
	4675 1550 4475 1550
Connection ~ 4675 1550
Wire Wire Line
	4675 1200 4675 1275
Connection ~ 4675 1200
Wire Wire Line
	4675 1900 4675 1825
Connection ~ 4675 1900
Connection ~ 7725 1300
Connection ~ 7725 1800
Wire Wire Line
	2275 3200 2275 3275
Wire Wire Line
	1300 2800 1875 2800
Wire Wire Line
	1875 3000 1825 3000
Wire Wire Line
	1825 3000 1825 2800
Connection ~ 1825 2800
Wire Wire Line
	2750 2900 2675 2900
Wire Wire Line
	2950 2900 3025 2900
Wire Wire Line
	3025 2900 3025 2800
Wire Wire Line
	2675 2800 3400 2800
Wire Wire Line
	3200 2800 3200 2900
Connection ~ 3025 2800
Wire Wire Line
	3200 3250 3200 3100
Wire Wire Line
	1450 3250 3200 3250
Connection ~ 2275 3250
Wire Wire Line
	1450 3250 1450 3100
Wire Wire Line
	1450 2900 1450 2800
Connection ~ 1450 2800
Connection ~ 3200 2800
Wire Wire Line
	2450 4650 2200 4650
Wire Wire Line
	2200 4550 2200 5150
Wire Wire Line
	1250 4550 2450 4550
Wire Wire Line
	2200 4850 2450 4850
Connection ~ 2200 4650
Wire Wire Line
	2200 5150 2450 5150
Connection ~ 2200 4850
Connection ~ 2200 4550
Wire Wire Line
	1975 4650 1975 4550
Connection ~ 1975 4550
Wire Wire Line
	1800 4650 1800 4550
Connection ~ 1800 4550
Wire Wire Line
	1625 4650 1625 4550
Connection ~ 1625 4550
Wire Wire Line
	1350 4650 1350 4550
Connection ~ 1350 4550
Wire Wire Line
	1975 4950 1975 4850
Wire Wire Line
	1250 4950 1975 4950
Wire Wire Line
	1800 4950 1800 4850
Wire Wire Line
	1625 4950 1625 4850
Connection ~ 1800 4950
Wire Wire Line
	1350 4950 1350 4850
Connection ~ 1625 4950
Connection ~ 1350 4950
Wire Wire Line
	2250 6750 2450 6750
Wire Wire Line
	2350 6650 2350 6850
Wire Wire Line
	2350 6650 2450 6650
Connection ~ 2350 6750
Wire Wire Line
	2350 6850 2450 6850
Wire Wire Line
	4350 4850 4450 4850
Wire Wire Line
	4350 4950 4450 4950
Wire Wire Line
	4350 5050 4450 5050
Wire Wire Line
	6650 3100 6500 3100
Wire Wire Line
	6650 3200 6500 3200
Wire Wire Line
	6650 3300 6500 3300
Wire Wire Line
	4350 6350 6075 6350
Wire Wire Line
	6075 6350 6075 6300
Wire Wire Line
	6075 6100 6075 6050
Wire Wire Line
	4450 6650 4350 6650
Wire Wire Line
	4350 6750 5350 6750
Wire Wire Line
	4450 4550 4350 4550
Wire Wire Line
	4350 5150 4975 5150
Wire Wire Line
	4975 5150 4975 4950
Wire Wire Line
	4975 4950 5550 4950
Wire Wire Line
	4350 5250 4975 5250
Wire Wire Line
	4975 5250 4975 5450
Wire Wire Line
	4975 5450 5550 5450
Wire Wire Line
	5100 4950 5100 5100
Connection ~ 5375 4950
Wire Wire Line
	5100 5450 5100 5300
Connection ~ 5375 5450
Connection ~ 5100 4950
Connection ~ 5100 5450
Wire Wire Line
	5750 4950 5850 4950
Wire Wire Line
	5850 4950 5850 5450
Wire Wire Line
	5850 5450 5750 5450
Wire Wire Line
	5475 5200 5950 5200
Connection ~ 5850 5200
Wire Wire Line
	4350 6000 5675 6000
Wire Wire Line
	5675 6000 5675 5950
Wire Wire Line
	5675 5700 5675 5750
Wire Wire Line
	6450 4225 6400 4225
Wire Wire Line
	6450 4325 6400 4325
Wire Wire Line
	6450 4425 6400 4425
Wire Wire Line
	6950 4225 7000 4225
Wire Wire Line
	6950 4325 7000 4325
Wire Wire Line
	6950 4425 7000 4425
Wire Wire Line
	4350 4650 5200 4650
Wire Wire Line
	4350 6550 4450 6550
Wire Wire Line
	4350 6250 4450 6250
Wire Wire Line
	4350 6150 4450 6150
Wire Wire Line
	8100 4450 7775 4450
Wire Wire Line
	8050 4550 8100 4550
Wire Wire Line
	8100 4750 8050 4750
Wire Wire Line
	8050 4850 8100 4850
Wire Wire Line
	7975 4950 8100 4950
Wire Wire Line
	7775 4950 7700 4950
Wire Wire Line
	9750 2725 9750 2925
Wire Wire Line
	9750 2225 9750 2325
Text Label 9450 2000 2    60   ~ 0
3V3
$Comp
L R_Small R8
U 1 1 5957E591
P 9250 2525
F 0 "R8" V 9175 2425 50  0000 L CNN
F 1 "10k" V 9175 2575 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 9250 2525 50  0001 C CNN
F 3 "" H 9250 2525 50  0000 C CNN
	1    9250 2525
	0    1    1    0   
$EndComp
Wire Wire Line
	9350 2525 9450 2525
Wire Wire Line
	9150 2525 9050 2525
Text Label 9050 2525 2    60   ~ 0
RELAY
Text Label 4450 6450 0    60   ~ 0
RELAY
Wire Wire Line
	4450 6450 4350 6450
$Comp
L R_Small R2
U 1 1 59583C45
P 5200 4500
F 0 "R2" H 5250 4575 50  0000 L CNN
F 1 "10k" H 5250 4425 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 5200 4500 50  0001 C CNN
F 3 "" H 5200 4500 50  0000 C CNN
	1    5200 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 4650 5200 4600
Wire Wire Line
	5200 4400 5200 4350
Text Label 5200 4350 2    60   ~ 0
3V3
$Comp
L R_Small R4
U 1 1 59584B20
P 5600 6600
F 0 "R4" H 5650 6675 50  0000 L CNN
F 1 "10k" H 5650 6525 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 5600 6600 50  0001 C CNN
F 3 "" H 5600 6600 50  0000 C CNN
	1    5600 6600
	1    0    0    -1  
$EndComp
$Comp
L R_Small R3
U 1 1 59584C4D
P 5350 6600
F 0 "R3" H 5400 6675 50  0000 L CNN
F 1 "10k" H 5400 6525 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 5350 6600 50  0001 C CNN
F 3 "" H 5350 6600 50  0000 C CNN
	1    5350 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 6500 5350 6450
Wire Wire Line
	5250 6450 5600 6450
Wire Wire Line
	5600 6450 5600 6500
Text Label 5250 6450 2    60   ~ 0
3V3
Connection ~ 5350 6450
Wire Wire Line
	5350 6750 5350 6700
Wire Wire Line
	5600 6700 5600 6850
Wire Wire Line
	5600 6850 4350 6850
Text Notes 2200 1000 0    60   ~ 0
IEC inlet to include fuse box,\nwith 1A type T fuse\n+ mains switch
Wire Wire Line
	4475 1550 4475 2025
Wire Wire Line
	4475 2025 825  2025
$Comp
L TVS TVS1
U 1 1 59738A17
P 2625 1550
F 0 "TVS1" H 2625 1700 50  0000 C CNN
F 1 "1.5KE300CA1" H 2625 1400 50  0000 C CNN
F 2 "Diodes_ThroughHole:Diode_DO-201AD_Horizontal_RM15" H 2625 1550 50  0001 C CNN
F 3 "" H 2625 1550 50  0000 C CNN
F 4 "486-0654" H 2625 1550 60  0001 C CNN "RS"
	1    2625 1550
	0    1    1    0   
$EndComp
Wire Wire Line
	2150 1200 2875 1200
Wire Wire Line
	2625 1200 2625 1250
Wire Wire Line
	2625 1900 2625 1850
Connection ~ 2625 1200
Connection ~ 2625 1900
Wire Wire Line
	5025 1200 5025 1300
Wire Wire Line
	5025 1300 5225 1300
Wire Wire Line
	5025 1900 5025 1800
Wire Wire Line
	5025 1800 5225 1800
$Comp
L DPDT SW2
U 1 1 5973AADD
P 10325 1750
F 0 "SW2" H 10200 2025 50  0000 C CNN
F 1 "TE D3223" H 10550 2025 50  0000 C CNN
F 2 "Relays_ThroughHole_TE_Connectivity:FX2 D3223" H 9700 1925 50  0001 C CNN
F 3 "" H 9700 1925 50  0000 C CNN
F 4 "909-8025" H 10325 1750 60  0001 C CNN "RS"
	1    10325 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9750 2075 10025 2075
Wire Wire Line
	9750 2225 10025 2225
$Comp
L D_Small D2
U 1 1 5973D652
P 9625 2150
F 0 "D2" H 9575 2230 50  0000 L CNN
F 1 "STTH212U" H 9475 2070 50  0000 L CNN
F 2 "Diodes_SMD:SMB_Handsoldering" V 9625 2150 50  0001 C CNN
F 3 "" V 9625 2150 50  0000 C CNN
F 4 "248-619" H 9625 2150 60  0001 C CNN "RS"
	1    9625 2150
	0    1    1    0   
$EndComp
Wire Wire Line
	9750 2000 9750 2075
Wire Wire Line
	9625 2250 9625 2300
Wire Wire Line
	9625 2300 9750 2300
Connection ~ 9750 2300
Wire Wire Line
	9625 2050 9625 2000
Wire Wire Line
	9450 2000 9750 2000
Connection ~ 9625 2000
Wire Wire Line
	9925 1850 10025 1850
Wire Wire Line
	9925 1600 9925 1850
Wire Wire Line
	10625 1700 10775 1700
Wire Wire Line
	10775 1900 10625 1900
NoConn ~ 10625 1600
NoConn ~ 10625 1800
Text Notes 9475 1350 0    60   ~ 0
Add drills for creepage if necessary
NoConn ~ 4350 4750
$Comp
L SW_PUSH_SMALL_H SW1
U 1 1 5975AC35
P 4625 3225
F 0 "SW1" H 4705 3335 50  0000 C CNN
F 1 "Reset" H 4625 3150 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_EVQP0" H 4625 3425 50  0001 C CNN
F 3 "" H 4625 3425 50  0000 C CNN
	1    4625 3225
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR5
U 1 1 5975B2B7
P 4900 3225
F 0 "#PWR5" H 4900 2975 50  0001 C CNN
F 1 "GND" H 4900 3075 50  0000 C CNN
F 2 "" H 4900 3225 50  0000 C CNN
F 3 "" H 4900 3225 50  0000 C CNN
	1    4900 3225
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4900 3225 4775 3225
Wire Wire Line
	4475 3225 4375 3225
Text Label 4375 3225 2    60   ~ 0
~RESET
$Comp
L CONN_02X02 P13
U 1 1 594E8E2E
P 8850 1750
F 0 "P13" H 8850 2000 50  0000 C CNN
F 1 "Output" H 8850 1550 50  0000 C CNN
F 2 "Connectors_Molex:Molex_MiniFit-JR-5556-04A_2x02x4.20mm_Straight" H 8850 1750 50  0001 C CNN
F 3 "" H 8850 1750 50  0000 C CNN
	1    8850 1750
	-1   0    0    -1  
$EndComp
Wire Wire Line
	9200 1175 9200 1900
Wire Wire Line
	2425 1900 2875 1900
Wire Wire Line
	3700 1900 5025 1900
Wire Wire Line
	3000 1400 2875 1400
Wire Wire Line
	2875 1400 2875 1200
Wire Wire Line
	3600 1400 3700 1400
Wire Wire Line
	3700 1200 3700 1450
Wire Wire Line
	3600 1700 3700 1700
Wire Wire Line
	3700 1650 3700 1900
Wire Wire Line
	3000 1700 2875 1700
Wire Wire Line
	2875 1700 2875 1900
Wire Wire Line
	2875 2100 2875 2025
Connection ~ 2875 2025
$Comp
L CP_Small C9
U 1 1 5975E598
P 3200 3000
F 0 "C9" H 3210 3070 50  0000 L CNN
F 1 "10u" H 3210 2920 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Radial_D5_L11_P2" H 3200 3000 50  0001 C CNN
F 3 "" H 3200 3000 50  0000 C CNN
	1    3200 3000
	1    0    0    -1  
$EndComp
$Comp
L CP_Small C1
U 1 1 5975FD88
P 1350 4750
F 0 "C1" H 1360 4820 50  0000 L CNN
F 1 "10u" H 1360 4670 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Radial_D5_L11_P2" H 1350 4750 50  0001 C CNN
F 3 "" H 1350 4750 50  0000 C CNN
	1    1350 4750
	1    0    0    -1  
$EndComp
$Comp
L C_Small C4
U 1 1 59760BC8
P 1800 4750
F 0 "C4" H 1810 4820 50  0000 L CNN
F 1 "10n" H 1810 4670 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 1800 4750 50  0001 C CNN
F 3 "" H 1800 4750 50  0000 C CNN
	1    1800 4750
	-1   0    0    1   
$EndComp
$Comp
L C_Small C5
U 1 1 59760C7F
P 1975 4750
F 0 "C5" H 1985 4820 50  0000 L CNN
F 1 "10n" H 1985 4670 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 1975 4750 50  0001 C CNN
F 3 "" H 1975 4750 50  0000 C CNN
	1    1975 4750
	-1   0    0    1   
$EndComp
Text Notes 5875 2750 0    60   ~ 0
Connect to jack socket feedthrough\nUse tag on jack for enclosure earth
$Comp
L TST P10
U 1 1 5976E69C
P 7950 1300
F 0 "P10" H 7950 1625 50  0000 C BNN
F 1 "TEST_5V" H 7950 1550 50  0000 C CNN
F 2 "Connect:PINTST" H 7950 1300 50  0001 C CNN
F 3 "" H 7950 1300 50  0000 C CNN
	1    7950 1300
	1    0    0    -1  
$EndComp
Connection ~ 7950 1300
$Comp
L TST P11
U 1 1 5976F3D3
P 7950 1800
F 0 "P11" H 7950 2150 50  0000 C BNN
F 1 "TEST_GND" H 7950 2075 50  0000 C CNN
F 2 "Connect:PINTST" H 7950 1800 50  0001 C CNN
F 3 "" H 7950 1800 50  0000 C CNN
	1    7950 1800
	-1   0    0    1   
$EndComp
Connection ~ 7950 1800
$Comp
L CRYSTAL_SMD X1
U 1 1 5976C0EA
P 5375 5200
F 0 "X1" H 5375 5290 50  0000 C CNN
F 1 "16MHz" V 5500 4950 50  0000 L CNN
F 2 "Crystals:crystal_FA238-TSX3225" H 5375 5200 50  0001 C CNN
F 3 "" H 5375 5200 50  0000 C CNN
F 4 "667-6209" H 5375 5200 60  0001 C CNN "RS"
	1    5375 5200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5375 5000 5375 4950
Wire Wire Line
	5375 5400 5375 5450
Wire Wire Line
	8550 1600 9925 1600
Wire Wire Line
	10025 1650 9975 1650
Wire Wire Line
	9975 1650 9975 1800
Wire Wire Line
	9975 1800 9100 1800
$Comp
L C_Small C18
U 1 1 5988EB31
P 3700 1550
F 0 "C18" H 3710 1620 50  0000 L CNN
F 1 "100n/400V/X" V 3825 1275 50  0000 L CNN
F 2 "Capacitors_ThroughHole_Kemet:C_Rect_L24_W7.6_P20.3" H 3700 1550 50  0001 C CNN
F 3 "" H 3700 1550 50  0000 C CNN
F 4 "878-7979" H 3700 1550 60  0001 C CNN "RS"
	1    3700 1550
	1    0    0    -1  
$EndComp
Connection ~ 3700 1700
Connection ~ 3700 1400
$Comp
L L_Small L3
U 1 1 59890772
P 4225 1200
F 0 "L3" V 4300 1150 50  0000 L CNN
F 1 "1m/400V" V 4175 1125 50  0000 L CNN
F 2 "Choke_Axial_ThroughHole:Choke_Horizontal_RM10mm" H 4225 1200 50  0001 C CNN
F 3 "" H 4225 1200 50  0000 C CNN
F 4 "860-4067" V 4225 1200 60  0001 C CNN "RS"
	1    4225 1200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4325 1200 5025 1200
Wire Wire Line
	4125 1200 3700 1200
$Comp
L F_Small F1
U 1 1 598A4C6D
P 2050 1200
F 0 "F1" H 2010 1260 50  0000 L CNN
F 1 "10mA, Q" H 1930 1140 50  0000 L CNN
F 2 "Fuse_Holders_and_Fuses:Fuseholder5x20_horiz_open_Schurter_0031_8201" H 2050 1200 50  0001 C CNN
F 3 "" H 2050 1200 50  0000 C CNN
F 4 "337-1951" H 2050 1200 60  0001 C CNN "RS"
	1    2050 1200
	1    0    0    -1  
$EndComp
$Comp
L EPCOSB82730U L1
U 1 1 598A6D38
P 3300 1550
F 0 "L1" H 3300 1825 60  0000 C CNN
F 1 "1m/400V" H 3300 1275 60  0000 C CNN
F 2 "Choke_Common-Mode_Epcos:CommonModeChoke_Epcos-B82730U" H 3300 1550 60  0001 C CNN
F 3 "" H 3300 1550 60  0001 C CNN
F 4 "871-1452" H 3300 1550 60  0001 C CNN "RS"
	1    3300 1550
	1    0    0    -1  
$EndComp
Text Notes 1225 1100 0    60   ~ 0
Fuse holder cover:\nRS 337-1327
Text Label 9275 1600 0    60   ~ 0
Relay_Live
Text Label 9275 1800 0    60   ~ 0
Relay_Neutral
Wire Wire Line
	825  1300 875  1300
Wire Wire Line
	875  1400 825  1400
Wire Wire Line
	825  1200 1950 1200
Wire Wire Line
	825  1300 825  1200
Wire Wire Line
	825  1400 825  2025
Wire Wire Line
	1375 1300 1450 1300
Wire Wire Line
	1450 1300 1450 2025
Connection ~ 1450 2025
Wire Wire Line
	8550 1600 8550 1700
Wire Wire Line
	8550 1700 8600 1700
Wire Wire Line
	9200 1700 9100 1700
Wire Wire Line
	8600 1800 8550 1800
Wire Wire Line
	8550 1800 8550 1900
Wire Wire Line
	8550 1900 9200 1900
Connection ~ 9200 1700
$Comp
L CONN_01X02 P18
U 1 1 59925599
P 8025 3175
F 0 "P18" H 8025 3325 50  0000 C CNN
F 1 "Enclosure earth" V 8125 3175 50  0000 C CNN
F 2 "Terminal_Blocks:TerminalBlock_Pheonix_MKDS1.5-2pol" H 8025 3175 50  0001 C CNN
F 3 "" H 8025 3175 50  0000 C CNN
	1    8025 3175
	1    0    0    -1  
$EndComp
$Comp
L Earth_Protective #PWR9
U 1 1 59925D40
P 7775 3075
F 0 "#PWR9" H 8025 2825 50  0001 C CNN
F 1 "Earth_Protective" H 8225 2925 50  0001 C CNN
F 2 "" H 7775 2975 50  0000 C CNN
F 3 "" H 7775 2975 50  0000 C CNN
	1    7775 3075
	-1   0    0    1   
$EndComp
Wire Wire Line
	7775 3075 7775 3225
Wire Wire Line
	7775 3125 7825 3125
Wire Wire Line
	7775 3225 7825 3225
Connection ~ 7775 3125
NoConn ~ 4350 5900
NoConn ~ 4350 5800
NoConn ~ 4350 5700
NoConn ~ 4350 5600
NoConn ~ 4350 5500
NoConn ~ 4350 5400
NoConn ~ 2450 6000
NoConn ~ 2450 5900
$EndSCHEMATC