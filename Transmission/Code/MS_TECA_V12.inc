;****************************************************************************
;
;         MS_TECA_V12.inc (header file for MS_TECA_V12.asm)  3/01/10
;
;           Semi automatic controller for the Ford E4OD transmission
;
;         By Robert Hiebert with technical assistance from Dan Williams
;           and all those who contributed to the Megasquirt projects
;
;****************************************************************************

*****************************************************************************
*****************************************************************************
**   M E G A S Q U I R T - 2 0 0 1 - V2.00
**
**   (C) 2002 - B. A. Bowling And A. C. Grippo
**
**   This header must appear on all derivatives of this code.
**
*****************************************************************************
*****************************************************************************

;****************************************************************************
;
; ------------------------- MS_TECA Hardware Wiring  -----------------------
;
;****************************************************************************
;
; ----- Power connections -----
;
;  12 Volt input   - Pin 19
;  Vref 5V output  - Pin 8
;  Common ground   - Pins 22,23,24,26,27,29,32,33,34,35
;
; ----- Inputs [Port Name - Function - Pin#] -----
;
;  IRQ       - Engine Speed Sensor,(invert to IRQ)         - Pin 21
;  PTB0/AD0  - Manifold Absolute Pressure                  - No Pin
;  PTB1/AD1  - Manual Lever Position Switch                - Pin 3
;  PTB2/AD2  - Transmisssion Oil Temperature               - Pin 4
;  PTB3/AD3  - Line Pressure                               - Pin 5
;  PTB4/AD4  - Battery Voltage/Boot Loader Entry           - No Pin
;  PTB5/AD5  - Idle Air Control Input                      - Pin 6
;  PTB6/AD6  - Electronic Pressure Control Trim            - Pin 25
;  PTB7/AD7  - Throttle Position Switch                    - Pin 7
;  PTA0/KBD0 - Vehicle Speed Sensor                        - Pin 20
;  PTA1      - Exhaust Pressure Switch                     - Pin 9
;  PTA2      - DFC/Disable/Exhaust Brake release contacts  - Pin 28
;  PTA3      - DFC Enable/Exhaust Brake apply contacts     - Pin 10
;  PTA4      - Torque Converter Clutch release contacts    - Pin 11
;  PTA5      - Torque Converter Clutch apply contacts      - Pin 30
;  PTA6      - Shift down contacts                         - Pin 12
;  PTA7      - Shift up contacts                           - Pin 31
;
; ----- Outputs [Port Name - Function - Pin#] -----
;
;  PTD2/MOSI  - Decel Fuel Cut Permissive for engine           - Pin 13
;  PTD3/SPSCK - Program Loop Counter LED                       - No Pin
;  PTD4/T1CH0 - Electronic Pressure Control Voltage           - Pin 37
;  PTD5/T1CH1 - Idle Air Control Solonoid                      - Pin 36
;  PTC0       - Shift Solonoid 2                               - Pin 17
;  PTC1       - Shift Solonoid 1                               - Pin 18
;  PTC2       - Torque Converter Clutch Solonoid               - Pin 16
;  PTC3       - Coast Clutch Solonoid                          - Pin 15
;  PTC4       - Exhaust Brake air and secondary air solonoids  - Pin 14
;
; ----- Spares [Port name/Pin# - Header Socket#] -----
;
;  PTD0/SS    - H1
;  PTD1/MISO  - H2
;  Pin 1      - H3
;  Pin 2      - H4
;
;****************************************************************************

;****************************************************************************
; ------------------------ Input Port Equates -------------------------------
;****************************************************************************

VehSpd      equ 0     ; Vehicle Speed input on Port A0
ExhPS       equ 1     ; Exhaust Pressure Switch on Port A1
DFCdis      equ 2     ; DFC/Exhaust Brake release contacts on Port A2
DFCen       equ 3     ; DFC/Exhaust Brake apply contacts on Port A3
TCCrel      equ 4     ; TCC release contacts on Port A4
TCCapp      equ 5     ; TCC apply contacts on Port A5
ShiftDn     equ 6     ; Shift Down contacts on Port A6
ShiftUp     equ 7     ; Shift Up contacts on Port A7

;****************************************************************************
; ------------------------ Output Port Equates ------------------------------
;****************************************************************************

SS2      equ 0     ; Shift Solonoid #2 on Port C0
SS1      equ 1     ; Shift Solonoid #1 on Port C1
TCC      equ 2     ; Torque Converter Clutch Solonoid on Port C2
CCS      equ 3     ; Coast Clutch Solonoid #1 on Port C3
ExhBrk   equ 4     ; Exhaust Brake air and secondary air solonoids Port C4
DFCper   equ 2     ; Decel Fuel Cut permissive for Engine Control on Port D2
LoopFrq  equ 3     ; Loop frequency counter on Port D3
EPC      equ 4     ; EPC PWM on Port D4
IACpwm   equ 5     ; IAC PWM on Port D5

;****************************************************************************
; ------------------ "mlpsp","mlpsp_cur" Status equates ---------------------
;****************************************************************************

P      equ 1     ; MLPS in "Park" position             0 = out of range
R      equ 2     ; MLPS in "Reverse" position          0 = out of range
N      equ 3     ; MLPS in "Neutral" position          0 = out of range
D      equ 4     ; MLPS in "Drive" position            0 = out of range
M2     equ 5     ; MLPS in "Manual Second" position    0 = out of range
M1     equ 6     ; MLPS in "Manual First" position     0 = out of range
SF     equ 7     ; MLPS Sensor Failure                 0 = out of range

;****************************************************************************
; ----- "gearcnt","gearcnt_prv","gear_cur","gear_com" Status equates --------
;****************************************************************************

first     equ 1     ; Drive 1
second    equ 2     ; Drive 2
third     equ 3     ; Drive 3
forth     equ 4     ; Drive 4

;****************************************************************************
; ---------------- "inputs" Operating Status bit field equates --------------
;****************************************************************************

tachrise    equ 0     ; 1 = tach rising edge flag    0 = "tachrise" cleared
clk100k     equ 1     ; 1 = 0.1S   clock tick flag   0 = "clk100k" cleared
adcc        equ 2     ; 1 = ADC complete flag        0 = "adcc" cleared
vssp        equ 3     ; 1 = Veh Spd rising edge flag 0 = "vssp" cleared
iacon       equ 4     ; 1 = IAC pulse width on flag  0 = "iacon" cleared
SUhi        equ 5     ; 1 = Shift Up last pass Hi    0 = Shift Up lst pss Lo
SDhi        equ 6     ; 1 = Shift Dn last pass Hi    0 = Shift Dn lst pss Lo

;****************************************************************************
; ---------------- "trans" Operating Status bit field equates ---------------
;****************************************************************************

Estop     equ 0     ; 1 = Engine stopped             0 = Engine not stopped
Vstop     equ 1     ; 1 = Vehicle stopped            0 = Vehicle not stopped
ClsThrt   equ 2     ; 1 = TPS at closed throttle     0 = TPS not Close Throt
RPMtcc    equ 3     ; 1 = RPM TCC app permissive     0 = No RPM TCC perm.
MPHstall  equ 4     ; 1 = MPH at stall condition     0 = MPH above stall
accel     equ 5     ; 1 = TPS DOT EPC mode           0 = No TPS DOT EPC mode
PSIbrk    equ 6     ; 1 = Exh Brk Exh prs permissive 0 = No ExhBrk prs perm.

;****************************************************************************
; --------------- "trans2" Operating Status bit field equates ---------------
;****************************************************************************

CCSon     equ 0     ; 1 = CCS on                     0 = CCS off
TCCon     equ 1     ; 1 = TCC on                     0 = TCC off
DFCon     equ 2     ; 1 = DFC permissive             0 = No DFC permissive
Brkon     equ 3     ; 1 = Exhaust Brake on           0 = Exh Brk off
SSprog    equ 4     ; 1 = Gear Change in progress    0 = No GC in progress
TCprog    equ 5     ; 1 = TCC apply in progress      0 = No TCC in progress
D1D2      equ 6     ; 1 = In Drive 1 or 2            0 = Not in Drive 1 or 2


;****************************************************************************
; --------------- "shift" Operating Status bit field equates ---------------
;****************************************************************************

EPCrTCC   equ 0     ; 1 = EPC rising for TCC app   0 = No EPC rise for TCC
EPChTCC   equ 1     ; 1 = EPC holding for TCC app  0 = No EPC hold for TCC
EPCrSS    equ 2     ; 1 = EPC rising for shift     0 = No EPC rise for shift
EPChSS    equ 3     ; 1 = EPC holding for shift    0 = No EPC hold for shift
SS1del    equ 4     ; 1 = SS1 apply delay          0 = No SS1 delay
CCSdel    equ 5     ; 1 = CCS apply delay          0 = No CCS delay
SSsdel    equ 6     ; 1 = SSs apply delay(both)    0 = No SSs delay
Brkdel    equ 7     ; 1 = ExhBrk apply delay       0 = No ExhBrk delay


;****************************************************************************
; ------------- "tconf" Tuning Configuration bit field equates --------------
;****************************************************************************

tt      equ 0     ; 1 = TOT cor enabled        0 = TOT cor disabled
tr      equ 1     ; 1 = Trim cor enabled       0 = Trim cor disabled
cyl     equ 2     ; 1 = 8 cylinder             0 = 6 cylinder

;****************************************************************************
; --------------------- "TuneConfig" Status equates -------------------------
;****************************************************************************

;TOTon    equ 0     ; 1 = TOT cor enabled        0 = TOT cor disabled
;Trimon   equ 1     ; 1 = Trim cor enabled       0 = Trim cor disabled
;Cylno    equ 2     ; 1 = 8 cylinder             0 = 6 cylinder


;****************************************************************************
; ------------------------ "dfSel" Status equates --------------------------
;****************************************************************************

selTO      equ 0     ; Select "TO" Duty Factor            0 = not selected
selStl     equ 1     ; Select "EPC_stall" Duty Factor     0 = not selected
sel12      equ 2     ; Select "EPC_12" Duty Factor        0 = not selected
sel23      equ 3     ; Select "EPC_23" Duty Factor        0 = not selected
sel34      equ 4     ; Select "EPC_34" Duty Factor        0 = not selected
selTCC     equ 5     ; Select "EPC_TCC" Duty Factor       0 = not selected
selDcl     equ 6     ; Select "EPC_Decel" Duty Factor     0 = not selected
selM1      equ 7     ; Select "EPC_M1" Duty Factor        0 = not selected


;****************************************************************************
; ----------------- RAM Variables (512 bytes available) ---------------------
; I/O registers from $0000 to $0039 (decimal 0 to 63)
; Direct page addressing from $0000 to $00FF (decimal 0 to 255)
; Ram end at $023F (decimal 575)
;****************************************************************************

;Memory Location $0040 (decimal 64)

ms_ram_start:

;****************************************************************************
; ------------ RS232 Real Time Download and Datalog Variables ---------------
;****************************************************************************

secH:           ds 1     ; Seconds counter, Hi byte
secL:           ds 1	 ; Seconds counter, Lo byte
RPM:            ds 1     ; Engine RPM/20 (0 to 5100 rpm in byte variable)
MPH:            ds 1     ; MPH*2 (0 to 128 MPH in byte variable)
trans:          ds 1     ; Transmission status bit field variable(1 of 2)
trans2:         ds 1     ; Transmission status bit field variable(2 of 2)
gearcnt:        ds 1     ; Current gear count(sequential gear selector)
MAP:            ds 1     ; Manifold Absolute Pressure 8 bit ADC reading
MLPS:           ds 1     ; Manual Lever Position Switch 8 bit ADC reading
TOT:            ds 1     ; Transmission Oil Temperature 8 bit ADC reading
Lprs:           ds 1     ; Line Pressure 8 bit ADC reading
BAT:            ds 1     ; Battery Voltage 8 bit ADC reading
IAC:            ds 1     ; Engine Idle Air Control Sensor 8 bit ADC reading
Trim:           ds 1     ; Electronic Pressure Control trim 8 bit ADC rdng
TPS:            ds 1     ; Throttle Position Sensor 8 bit ADC reading
KPA:            ds 1     ; Manifold Absolute Pressure in KPA

;Memory Location $0050 (decimal 80)

MLPSp:          ds 1     ; Manual Lever Position Switch position
TOTemp:         ds 1     ; Transmission Oil Temperature variable
                         ; (degrees F to fit range in byte variable)
Lpsi:           ds 1     ; Line pressure in PSI
Volts:          ds 1     ; Battery voltage to 0.1V resolution
IACpw:          ds 1     ; IAC pulse width variable(0-60, 100uS resolution)
TPSp:           ds 1     ; TPSscale * 100 / TPSspan = TPSp(TPS percent)
TOTempDif:      ds 1     ; Difference between "TOTemp" and 180(140F)
TOTempP:        ds 1     ; TOTemp percent calculation value (TOTemp/180)
TOTAdd:         ds 1     ; Trans Oil Temp correction Add/Subtract value
TrimDif:        ds 1     ; Difference between "Trim" and 128(mid point)
TrimP:          ds 1     ; Trim percent calculation value (TrimDif/128)
TrimAdd:        ds 1     ; EPC Trim correction Add/Subtract value
df:             ds 1     ; EPC Duty Factor from "TO" table, stall or shift
                         ; tables, or, absolute values "EPC_TCC", or
                         ; "EPC_decel" (scaled to 256)
df1:            ds 1     ; "df" after TOT cor, before Trim cor
dff:            ds 1     ; "df1" after Trim cor(Final EPC Duty Factor)
EPCpwH:         ds 1     ; EPC Pulse Width Hi byte

;Memory Location $0060 (decimal 96)

EPCpwL:         ds 1     ; EPC Pulse Width Lo byte


;****************************************************************************
; This marks the end of the real time download variables, 33 in total
;****************************************************************************
;****************************************************************************
; --------------------- Status Bit Field Variables --------------------------
; - NOTE! ";*"  denotes that this variable is listed with RS232 variables
;****************************************************************************

;*trans:          ds 1     ; Transmission status bit field variable(1 of 2)
;*trans2:         ds 1     ; Transmission status bit field variable(2 of 2)
shift:          ds 1     ; Gear change status bit field variable
tconf:          ds 1     ; Tuning Configuration status bit field variable
inputs:         ds 1     ; Input status bit field variable(1 of 2)

;****************************************************************************
; --------------------- TPS Calibration Variables ---------------------------
; - NOTE! ";*"  denotes that this variable is listed with RS232 variables
;****************************************************************************

TPSscale    ds 1     ; TPS - CT_cnt = TPSscale
;*TPSp        ds 1     ; TPSscale * 100 / TPSspan = TPSp(TPS %)

;****************************************************************************
; ----------------------- Time Clock Variables ------------------------------
; - NOTE! ";*"  denotes that this variable is listed with RS232 variables
;****************************************************************************

uSx100:         ds 1	 ; 100 Microseconds counter
mS:             ds 1	 ; Milliseconds counter
mSx5:           ds 1     ; 5 Milliseconds counter
mSx20:          ds 1     ; 20 Milliseconds counter
mSx100:         ds 1     ; 100 Milliseconds counter
;*secL:           ds 1     ; Seconds counter, Lo byte
;*secH:           ds 1     ; Seconds counter, Hi byte


;****************************************************************************
; --------------------- 8 bit ADC input Variables --------------------------
; ----------------- (ordered list from "MAP" to "TPS") --------------------
; - NOTE! ";*"  denotes that this variable is listed with RS232 variables
;****************************************************************************

;*MAP:            ds 1     ; Manifold Absolute Pressure 8 bit ADC reading
;*MLPS:           ds 1     ; Manual Lever Position Switch 8 bit ADC reading
;*TOT:            ds 1     ; Transmission Oil Temperature 8 bit ADC reading
;*Lprs:           ds 1     ; Line Pressure 8 bit ADC reading
;*BAT:            ds 1     ; Battery Voltage 8 bit ADC reading
;*IAC:            ds 1     ; Engine Idle Air Control Sensor 8 bit ADC reading
;*Trim:           ds 1     ; Electronic Pressure Control trim 8 bit ADC rdng
;*TPS:            ds 1     ; Throttle Position Sensor 8 bit ADC reading
adsel:          ds 1     ; ADC Selector Variable

;****************************************************************************
; --------------------- ADC Conversion Variables --------------------------
; - NOTE! ";*"  denotes that this variable is listed with RS232 variables
;****************************************************************************

;*KPA:            ds 1     ; Manifold Absolute Pressure in KPA
;*MLPSp:          ds 1     ; Manual Lever Position Switch position
;*TOTemp:         ds 1     ; Transmission Oil Temperature variable
                           ;(degrees F - 40 to fit range in byte variable)
;*Lpsi:           ds 1     ; Line pressure in PSI
;*Volts:          ds 1     ; Battery voltage to 0.1V resolution
;*IACpw:          ds 1     ; IAC pulse width variable(0-60, 100uS resolution)

;****************************************************************************
; --------------------- Gear Selection Variables --------------------------
; - NOTE! ";*"  denotes that this variable is listed with RS232 variables
;****************************************************************************

mlpsp_cur:      ds 1     ; MLPS position current
;*gearcnt:        ds 1     ; Current gear count(sequential gear selector)
gearcnt_prv:    ds 1     ; Previous gear count
gear_cur:       ds 1     ; Current gear
gear_com:       ds 1     ; Commanded gear

;***************************************************************************
; -------------------- Engine RPM Calculation Variables -------------------
; - NOTE! ";*"  denotes that this variable is listed with RS232 variables
;***************************************************************************

RPMcH:          ds 1     ; RPM period counter Hi byte (100uS resolution)

 ;Memory Location $0070 (decimal 112)

RPMcL:          ds 1     ; RPM period counter Lo byte (100uS resolution)
RPMpH:          ds 1     ; RPM period Hi byte (100 uSresolution)
RPMpL:          ds 1     ; RPM period Lo byte (100 uSresolution)
;*RPM:            ds 1     ; Engine RPM/20 (0 to 5100 rpm in byte variable)

;****************************************************************************
; ----------------- Vehicle Speed Calculation Variables --------------------
; - NOTE! ";*"  denotes that this variable is listed with RS232 variables
;****************************************************************************

MPHcH:          ds 1     ; MPH period counter Hi byte (100uS resolution)
MPHcL:          ds 1     ; MPH period counter Lo byte (100uS resolution)
MPHpH:          ds 1     ; MPH period Hi byte (100uS resolution)
MPHpL:          ds 1     ; MPH period Lo byte (100uS resolution)
;*MPH:            ds 1     ; MPH/2 (0 to 128 MPH in byte variable)

;****************************************************************************
; --------------------- EPC PW Calculation Variables ------------------------
; - NOTE! ";*"  denotes that this variable is listed with RS232 variables
;****************************************************************************

dfSel:         ds 1     ; Duty Factor Select variable bit field
;*TOTempDif:      ds 1     ; Difference between "TOTemp" and 180(140F)
;*TOTempP:        ds 1     ; TOTemp percent calculation value (TOTemp/180)
;*TOTAdd:         ds 1     ; Trans Oil Temp correction Add/Subtract value
;*TrimDif:        ds 1     ; Difference between "Trim" and 128(mid point)
;*TrimP:          ds 1     ; Trim percent calculation value (TrimDif/128)
;*TrimAdd:        ds 1     ; EPC Trim correction Add/Subtract value
;*df:             ds 1     ; EPC Duty Factor from "TO" table, stall or shift
                           ; tables, or, absolute values "EPC_TCC", or
                           ; "EPC_decel" (scaled to 256)
;*df1:            ds 1     ; "df" after TOT cor, before Trim cor
;*dff:            ds 1     ; "df1" after Trim cor(Final EPC Duty Factor)
;*EPCpwH:         ds 1     ; EPC Pulse Width Hi byte
;*EPCpwL:         ds 1     ; EPC Pulse Width Lo byte

;****************************************************************************
; ----------------------- Variables for integer math ------------------------
;****************************************************************************

INTACC1:      rmb 4
INTACC2:      rmb 4

;Memory Location $0080 (decimal 128)(at INTACC1+1)

;****************************************************************************
; ------------------------ Temporary variables ------------------------------
;****************************************************************************

tmp1          ds     1
tmp2          ds     1
tmp3          ds     1
tmp4          ds     1
tmp5          ds     1
tmp6          ds     1
tmp7          ds     1
tmp8          ds     1
tmp9          ds     1
tmp10         ds     1
tmp11         ds     1
tmp12         ds     1
tmp13         ds     1
tmp14         ds     1
tmp15         ds     1
tmp16         ds     1

;Memory Location $0090 (decimal 144)

tmp17         ds     1
tmp18         ds     1
tmp19         ds     1
tmp20         ds     1
tmp21         ds     1
local_tmp     ds     1

;****************************************************************************
; ------- Serial Communications Interface Parameters and Variables ---------
;****************************************************************************

txcnt:        ds 1     ; SCI transmitter count (incremented)
txgoal:       ds 1     ; SCI number of bytes to transmit
txmode:       ds 1     ; Transmit mode flag
rxoffset:     ds 1     ; Offset placeholder when receiving VE/constants
                       ; vis. SCI

;****************************************************************************
; --------------------- Flash Burner Routine Variables ----------------------
;****************************************************************************

burnSrc:      ds 2T    ; Burn routine variable
burnDst:      ds 2T    ; Burn routine variable
burnCount:    ds 1T    ; Burn routine variable

;****************************************************************************
; ----------------------------- Misc. Variables -----------------------------
;****************************************************************************

LoopCntr:      ds 1     ; Loop counter for main loop frequency check

;Memory Location $00A0 (decimal 160)

IACcnt:        ds 1     ; IAC counter for IACpw (100uS resolution)
TIMcnt:        ds 1     ; 20mS timer counter
TPS_prv:       ds 1     ; TPS rdg previous(update every 0.1S for TPS DOT)
ShftUpDB:      ds 1     ; Shift Up contacts de-bounce 1mS counter variable
ShftDnDB:      ds 1     ; Shift Dn contacts de-bounce 1mS counter variable
AIACcnt:       ds 1     ; Auto IAC duration counter value(100mS res)
Spare1:        ds 1     ; Blank place holder for 16 byte increments
Spare2:        ds 1     ; Blank place holder for 16 byte increments
Spare3:        ds 1     ; Blank place holder for 16 byte increments
Spare4:        ds 1     ; Blank place holder for 16 byte increments
Spare5:        ds 1     ; Blank place holder for 16 byte increments
Spare6:        ds 1     ; Blank place holder for 16 byte increments
Spare7:        ds 1     ; Blank place holder for 16 byte increments
Spare8:        ds 1     ; Blank place holder for 16 byte increments
Spare9:        ds 1     ; Blank place holder for 16 byte increments
Spare10:       ds 1     ; Blank place holder for 16 byte increments

;Memory Location $00B0 (decimal 176)

ms_ram_end:

;****************************************************************************
; -------------------- Flash Configuration constants ------------------------
;  EPC tables and constants, "TO_table" is entry point, everything is offset
;  from this point.
;****************************************************************************

;****************************************************************************
; - First group of 64 bytes
;****************************************************************************

ms_rf_start:

TO_table      rmb $40     ; 64 bytes for "TO" Table values

;Memory Location $00F0 (decimal 240)

;****************************************************************************
; - Second group of 64 bytes
;****************************************************************************

RPM_range     rmb $08     ; RPM bins for 2D interpolation of "TO" table
KPA_range     rmb $08     ; KPA bins for 2D interpolation of "TO" table

;Memory Location $0100 (decimal 256)(end of direct page)

TPS_range     rmb $08     ; TPS bins for 2D interpolation of EPC tables
EPC_stall     rmb $08     ; EPC stall bins (function of TPS)

;Memory Location $0110 (decimal 272)

EPC_12        rmb $08     ; EPC 1-2 shift bins (function of TPS)
EPC_23        rmb $08     ; EPC 2-3 shift bins (function of TPS)

;Memory Location $0120 (decimal 288)

EPC_34        rmb $08     ; EPC 3-4 shift bins (function of TPS)
EPC_M1        rmb $08     ; EPC M1 bins (function of TPS)

;Memory Location $0130 (decimal 304)

;****************************************************************************
; - Third group of 64 bytes
;****************************************************************************

EPC_TCC       rmb $01     ; EPC pulse width for TCC application
EPC_decel     rmb $01     ; EPC pulse width for decel conditions
EPC_rise      rmb $01     ; EPC rise time delay(20mS resolution)
EPC_hold      rmb $01     ; EPC hold time delay(20mS resolution)
SS1_del       rmb $01     ; SS1 apply time delay(20mS res)(M2-D2 shift)
CCS_del       rmb $01     ; CCS apply time delay(20mS res)(D4-D3 shift)
SSs_del       rmb $01     ; SSs release time delay(20mS res)(D3-D4 shift)
ExBrk_del     rmb $01     ; Exhaust Brake apply time delay(20mS res)
TCC_min_RPM   rmb $01     ; TCC apply minimum RPM permissive
MPH_stall     rmb $01     ; MPH maximum for stall EPC
TPSrate       rmb $01     ; TPS DOT rate threshold for EPC stall settings
CT_cnt        rmb $01     ; Closed throttle position ADC count
WOT_cnt       rmb $01     ; Wide Open throttle position ADC count
TPSspan       rmb $01     ; WOT_cnt - CT_cnt = TPSspan
CT_min        rmb $01     ; Closed throttle position minimum %
TrimFac       rmb $01     ; EPC Trim adjustment max value

;Memory Location $0140 (decimal 320)

TuneConfig    rmb $01     ; Tuning configuration variable
RPMk          rmb $02     ; RPM calculation constant
TOTempFac     rmb $01     ; Trans Oil Temp adjustment max value
TOThi         rmb $01     ; TOT correction Hi Limit (degreesF)
TOTlo         rmb $01     ; TOT correction Lo Limit (degreesF)
DBup          rmb $01     ; Upshift debounce counter (ms)
DBdn          rmb $01     ; Downshift debounce counter (ms)
EPC_M1_decel  rmb $01     ; EPC pulse width for M1 decel conditions



Blank_2       rmb $27     ; 39 Blank place holders for 64 byte increments

;Memory Location $0170 (decimal 368)

ms_rf_end:

;****************************************************************************
; ---------------------------- Misc. Equates --------------------------------
;****************************************************************************

ms_ram_size       equ {ms_ram_end-ms_ram_start}
ms_rf_size        equ {ms_rf_end-ms_rf_start}
ms_total_ram_size equ {ms_rf_end-ms_ram_start}




