;***********************************************************************************************
;
;                    MV460.inc (header file for MV460.asm)
;
;        Real Time Variable Display and "digital dashboard" for MSnS460
;
;                         By Robert Hiebert 2010
;
; - Rev 1: 4/16/12 Add configurable bank fuel flow constants for fuel burn calculation
;          fine tuning
;
;***********************************************************************************************
;***********************************************************************************************
;
; ------------------------------------------- Operation ----------------------------------------
;
; On power up, the unit defaults to display screen #0.
; The user has a choice of 17,(0-16), screens which display the variables name
; abreviation on the top line, and their corresponding real time values on
; the bottom line. The lower line is updated every 250 miliseconds.
; The earlier version, MV_ECU also had a constant configuration mode which
; this version does not. This is a "display only" unit.
;
; Screen 0 displays "ERPM MAP AFR CLT MAT" ; Default screen
; "ERPM" is Engine RPM, uses calculated "rpmH:rpmL"
; "MAP"  is Manifold Absolute Pressure in KPA, uses "map"
; "AFR"  is Air Fuel Ratio x 10, uses "afr"
; "CLT"  is Engine Coolant Temp in degrees F+40, uses "clt"
; "MAT"  is Manifold Air Temperature in degrees F+40, uses "mat"
;
; Screen 1 displays " ERPM KPH LTHR KMLT " ; Fuel burn screen
; "ERPM" is Engine RPM, uses calculated "rpmH:rpmL"
; "KPH"  is Vehicle Speed in KPH (Lo Res Fast Update), uses "kph"
; "LTHR" is Current fuel burn in L/Hr x 10, uses "ltrHrH:ltrHrL"
; "KMLT" is Current fuel burn in KM/L x 10, over a 1 second period, uses "kmLtrH:kmLtrL"
;
; Screen 2 displays "ERPM MAP AFR FTC PLW" ; Fuel tune screen
; "ERPM" is Engine RPM, uses calculated "rpmhrH:rpmhrL"
; "MAP"  is Manifold Absolute Pressure in KPA, uses "map"
; "AFR"  is Air Fuel Ratio x 10, uses "afr"
; "FTC"  is Fuel Trim Correction in percent, uses "Ftrimcor"
; "PLW"  is Pulse Width Lo Res in mmS, uses "pw"
;
; Screen 3 displays "ERPM MAP TRA CTA MTA" ; Ignition tune screen
; "ERPM" is Engine RPM, uses calculated "rpmhrH:rpmhrL"
; "MAP"  is Manifold Absolute Pressure in KPA, uses "map"
; "TRA"  is Ignition Trim Angle in degrees BTDC, uses "trmAng"
; "CTA"  is Calculated Timing Angle in degrees BTDC, uses "timAng"
; "MTA"  is Monitored Ignition Timing Angle in degrees BTDC, uses "monTimAng"
;
; Screen 4 displays " GAM WCR ACR BCR TCR" ; Corrections screen
; "GAM" is Gamma Correction in percent, uses gammae
; "WCR" is Engine Coolant Correction in percent, uses "warmcor
; "ACR" is Manifold Air Temperature Correction in percent, uses "aircor"
; "BCR" is Barometric Pressure Correction in percent, uses "barocor"
; "TCR" is Acceleration Correction in percent, uses "tpsaccel"
;
; Screen 5 displays " ego EGV LMD AFR FTC" ; EGO screen
; "ego" is Exhaust Gas Oxygen ADC, uses "egoADC"
; "EGV" is Exhaust Gas Oxygen voltage x 10, uses "egoV"
; "LMD" is Exhaust Gas Oxygen Lambda x 10, uses "lambda"
; "AFR"  is Air Fuel Ratio x 10, uses "afr"
; "FTC"  is Fuel Trim Correction in percent, uses "Ftrimcor"
;
; Screen 6 displays " map MAP bar BAR BCR" ; ADC screen 1
; "map" is Manifold Pressure ADC, uses "mapADC"
; "MAP" is Manifold Absolute Pressure in KPA, uses "map"
; "bar" is Barometric Pressure ADC, uses "baroADC"
; "BAR" is Barometric Pressure in KPA, uses "barometer"
; "BCR" is Barometric Pressure Correction in percent, uses "barocor"
;
; Screen 7 displays " vlt VLT clt CLT WCR" ; ADC screen 2
; "vlt" is Battery Voltage ADC, uses "battADC"
; "VLT" is Battery Voltage x 10, uses "volts"
; "clt" is Engine Coolant Temperature ADC, uses "cltADC"
; "CLT" is Engine Coolant Temperature in degrees F+40, uses "clt"
; "WCR" is Engine Coolant Correction in percent, uses "warmcor"
;
; Screen 8 displays " vlt VLT mat MAT ACR" ; ADC screen 3
; "vlt" is Battery Voltage ADC, uses "battADC"
; "VLT" is Battery Voltage x 10, uses "volts"
; "mat" is Manifold Air Temperature ADC, uses "matADC"
; "MAT" is Manifold Air Temperature in degrees F+40, uses "mat"
; "ACR" is Manifold Air Temperature Correction in percent, uses "aircor"
;
; Screen 9 displays " tps TPP trm FTC TAF" ; ADC screen 4
; "tps" is Throttle Position ADC, uses "tps"
; "TPP" is Throttle Positioin in percent, uses "tpsp"
; "trm" is Fuel/Ign Trim ADC, uses "trimADC"
; "FTC"  is Fuel Trim Correction in percent, uses "Ftrimcor"
; "TAF"  is Ignition Trim Angle Factor, uses "trmAngFac"
;
; Screen 10 displays "PWHR PLW FLD VEC DTY" ; Pulsewidth screen
; "PWHR" is Pulse Width Hi Res in mmmS, uses "pwcalcH:pwcalcL"
; "PLW"  is Pulse Width Lo Res in mmS, uses "pw"
; "FLD"  is Fuel Deleivery Pulse Width Lo Res in mmS, uses "fd"
; "VEC   is Volumetric Efficiency(Current VE table value in percent), uses "vecurr"
; "DTY"  is Injector Duty Cycle in percent, uses "duty"
;
; Screen 11 displays " ERPM PIPD MNPD VSPD" ; Period screen
; "ERPM" is Engine RPM, uses calculated "rpmhrH:rpmhrL"
; "PIPD" is PIP Period Predicted in mmmS, uses "pippprdH:pippprdL"
; "MNPD" is Ignition Monitor Period in mmmS, uses "monpH:monpL"
; "VSPD" is Vehicle Speed Period in mmS, uses "vspH:vspL"
;
; Screen 12 displays "ERPM MAP SAF DAF TAF" ; Ignition factor screen
; "ERPM" is Engine RPM, uses calculated "rpmH:rpmL"
; "MAP"  is Manifold Absolute Pressure in KPA, uses "map"
; "SAF"  is Spark Angle Factor(Current ST table value), uses "spkAngFac"
; "DAF"  is Delay Angle Factor, uses "dlyAngFac"
; "TAF"  is Ignition Trim Angle Factor, uses "trmAngFac"
;
; Screen 13 displays "ERPM TRA DLA CTA MTA" ; Ignition angle screen
; "ERPM" is Engine RPM, uses calculated "rpmH:rpmL"
; "TRA"  is Ignition Trim Angle in degrees BTDC, uses "trmAng"
; "DLA"  is Ignition Delay Angle in degrees BTDC, uses "dlyAng"
; "CTA"  is Calculated Timing Angle in degrees BTDC, uses "timAng"
; "MTA"  is Monitored Ignition Timing Angle in degrees BTDC, uses "monTimAng"
;
; Screen 14 displays "KPH kph ODS FDSC SEC" ; Fuel burn variables screen
; "KPH"  is Vehicle Speed in KPH (Lo Res Fast Update), uses "kph"
; "kph"  is Vehicle Speed in KPH (Hi Res Slow Update), uses "kph1"
; "ODS"  is Odometer Counts over a 1 second period, uses "odoSec"
; "FDSC" is Fuel Delivery Injector On Time over a 1 second period in mmS, uses "fdSecH:fdSecL"
; "SEC"  is Seconds Counter Lo byte, uses "secL"
;
; Screen 15 displays " RL FC  FP FT  AL WL" ; "Alarmbits", "portAbits", "portCbits" screen
; "RL" is Rev Limiter(alarmbits 5)
; "FC" is Flood Clear(alarmbits 6)
; "FP" is Fuel Pump(portAbits 0)
; "FT" is Fuel Trim Enable(portAbits 5)
; "AL" is Accel LED(portCbits 1)
; "WL" is Warmup LED(portCbits 2)
;
; Screen 16 displays " RN CR SW  RW AC DC " ; "Engine" bit field screen
; "RN" is Engine Running(engine 0)
; "CR" is Engine Cranking(engine 1)
; "SW" is Start Warmup(engine 2)
; "RW" is Run Warmup(engine 3)
; "AC" is Accellerating(engine 4)
; "DC" is Decelerating(engine 5)
;
;***********************************************************************************************

;****************************************************************************
;
; -------------------------- MV460 Hardware Wiring  ------------------------
;
;****************************************************************************
;
; ----- Inputs [port name - function] -----
;
;  PTA0      - Display Screen next
;  PTA1      - Display Screen previous
;  PTA2      - not used
;  PTA3      - not used
;
; ----- Outputs [port name - function] -----
;
;  PTB4       - VFD Display Enable
;  PTB5       - VFD Display R/W
;  PTB6       - VFD Display RS
;  PTC0       - VFD Display data DB4
;  PTC1       - VFD Display data DB5
;  PTC2       - VFD Display data DB6
;  PTC3       - VFD Display data DB7
;
;****************************************************************************

;****************************************************************************
; ------------------------ Input Port Equates -------------------------------
;****************************************************************************

Sw0     equ 0     ; PTA0 - Display Screen next
Sw1     equ 1     ; PTA1 - Display Screen previous
Sw2     equ 2     ; PTA2 - not used
Sw3     equ 3     ; PTA3 - not used

;****************************************************************************
; ------------------------ Output Port Equates ------------------------------
;****************************************************************************

Enable     equ 4     ; PTB4 - VFD Display Enable
Rd_Wrt     equ 5     ; PTB5 - VFD Display R/W
Reg_Sel    equ 6     ; PTB6 - VFD Display RS
DB0        equ 0     ; PTC0 - VFD Display data DB0 (4 bit mode)
DB1        equ 1     ; PTC1 - VFD Display data DB1 (4 bit mode)
DB2        equ 2     ; PTC2 - VFD Display data DB2 (4 bit mode)
DB3        equ 3     ; PTC3 - VFD Display data DB3 (4 bit mode)
DB4        equ 0     ; PTC0 - VFD Display data DB4
DB5        equ 1     ; PTC1 - VFD Display data DB5
DB6        equ 2     ; PTC2 - VFD Display data DB6
DB7        equ 3     ; PTC3 - VFD Display data DB7

;****************************************************************************
; ------------------------ Output Mask Equates ------------------------------
;****************************************************************************

DB0m     equ $01     ; %00000001 - PTC0 - VFD Display data DB0 (4 bit mode)
DB1m     equ $02     ; %00000010 - PTC1 - VFD Display data DB1 (4 bit mode)
DB2m     equ $04     ; %00000100 - PTC2 - VFD Display data DB2 (4 bit mode)
DB3m     equ $08     ; %00001000 - PTC3 - VFD Display data DB3 (4 bit mode)
DB4m     equ $10     ; %00010000 - PTC0 - VFD Display data DB4
DB5m     equ $20     ; %00100000 - PTC1 - VFD Display data DB5
DB6m     equ $40     ; %01000000 - PTC2 - VFD Display data DB6
DB7m     equ $80     ; %10000000 - PTC3 - VFD Display data DB7


;****************************************************************************
; -------------- "Flags" Operating Status bit field equates -----------------
;****************************************************************************

mde      equ 0  ; 1 = Configure mode             0 = Display mode
frz      equ 1  ; 1 = Dsply Vars frozen          0 = Dsply vars 250mS update
sel      equ 2  ; 1 = Current Constant Selected  0 = No Constant selected
RcvVE    equ 3  ; 1 = Receiving VE group         0 = Not receiving VE group
RcvST    equ 4  ; 1 = Receiving ST group         0 = Not receiving ST group
LinChng  equ 5  ; 1 = Bot Line changed           0 = Bot Line not changed

;****************************************************************************
; ----------------- "LPflags","ARCflags","ARflags" equates ------------------
;****************************************************************************

Sw0LP     equ 0     ; 1 = last pass closed(Lo)   0 = last pass open(Hi)
Sw1LP     equ 1     ; 1 = last pass closed(Lo)   0 = last pass open(Hi)
Sw2LP     equ 2     ; 1 = last pass closed(Lo)   0 = last pass open(Hi)
Sw3LP     equ 3     ; 1 = last pass closed(Lo)   0 = last pass open(Hi)

;****************************************************************************
; --------------------------- "Swflags" equates -----------------------------
;****************************************************************************

Sw0cls     equ 0     ; 1 = contacts closed(Lo)     0 = contacts open(Hi)
Sw1cls     equ 1     ; 1 = contacts closed(Lo)     0 = contacts open(Hi)
Sw2cls     equ 2     ; 1 = contacts closed(Lo)     0 = contacts open(Hi)
Sw3cls     equ 3     ; 1 = contacts closed(Lo)     0 = contacts open(Hi)

;***********************************************************************************************
; ----------------------- "Engine" Operating Status bit field equates --------------------------
;***********************************************************************************************

running:    equ 0    ; 1 = Engine running      0 = Engine not running
crank:      equ 1    ; 1 = Engine cranking     0 = Engine not cranking
startw:     equ 2    ; 1 = Warmup enrich       0 = Not in startup warmup
warmup:     equ 3    ; 1 = In warmup           0 = Not in warmup
tpsaen:     equ 4    ; 1 = TPS accel mode      0 = Not in TPS accel mode
tpsden:     equ 5    ; 1 = Decel mode          0 = Not in decel mode

;***********************************************************************************************
; ----------------------- "alarmbits" Operating Status bit field equates -----------------------
;***********************************************************************************************

REVL:    equ 5     ; 1 = Engine Rev Limit         0 = No Engine Rev Limit
fldClr:  equ 6     ; 1 = Engine Flood Clear       0 = No Engine Flood Clear

;***********************************************************************************************
; ----------------------- "portAbits" Operating Status bit field equates -----------------------
;***********************************************************************************************

FPon:     equ 0    ; 1 = Fuel Pump On            0 = Fuel Pump Not On
FTen:     equ 5    ; 1 = Fuel Trim Enabled       0 = Ignition Trim Enabled

;***********************************************************************************************
; ----------------------- "portCbits" Operating Status bit field equates -----------------------
;***********************************************************************************************

acclLED:     equ 1    ; 1 = Accel LED on          0 = Accel LED not on
wmLED:       equ 2    ; 1 = Warmup LED on         0 = Warmup LED not on


;****************************************************************************
; -------------------------- RAM Variables ---------------------------------
; - I/O Registers from $000 to $003F(0 to 63)(64 bytes)
; - RAM from $0040 to $023F(64 to 575)(512 bytes)
;****************************************************************************

;***************************************************************************
; - Misc. Variables
;***************************************************************************

;Memory Location $0040 (decimal 64)

Sw0DB:          ds 1     ; Switch #0 de-bounce timer counter variable
Sw0ARC:         ds 1     ; Switch #0 auto-repeat command timer counter var
Sw0AR:          ds 1     ; Switch #0 auto-repeat timer counter variable
Sw1DB:          ds 1     ; Switch #1 de-bounce timer counter variable
Sw1ARC:         ds 1     ; Switch #1 auto-repeat command timer counter var
Sw1AR:          ds 1     ; Switch #1 auto-repeat timer counter variable
Sw2DB:          ds 1     ; Switch #2 de-bounce timer counter variable
Sw2ARC:         ds 1     ; Switch #2 auto-repeat command timer counter var
Sw2AR:          ds 1     ; Switch #2 auto-repeat timer counter variable
Sw3DB:          ds 1     ; Switch #3 de-bounce timer counter variable
Sw3ARC:         ds 1     ; Switch #3 auto-repeat command timer counter var
Sw3AR:          ds 1     ; Switch #3 auto-repeat timer counter variable
LPflags:        ds 1     ; Switch last pass status bit field variable
ARCflags:       ds 1     ; Switch auto-repeat command status bit field
ARflags:        ds 1     ; Switch auto-repeat status bit field variable
Swflags:        ds 1     ; Switch status bit field variable

;Memory Location $0050 (decimal 80)

ModeCntr:       ds 1     ; Counter for determining "mode" bit status
FrzCntr:        ds 1     ; Counter for determining "frz" bit status
SelCntr:        ds 1     ; Counter for determining "sel" bit status
flags:          ds 1     ; Bit field for operating status flags (1 of 2)
ScrnCnt:        ds 1     ; Counter for display screen numbers
ScrnCnt_prv:    ds 1     ; Screen count number previous
ScrnCnt_Lst:    ds 1     ; Screen count number last
ConCnt:         ds 1     ; Counter for Constant numbers
ConCnt_prv:     ds 1     ; Constant number previous
ConCnt_Lst:     ds 1     ; Constant number last
CurCon:         ds 1     ; Value of current selected constant
ConVal:         ds 1     ; Value for constant data


;***************************************************************************
; - Serial Communications Interface Variables
;***************************************************************************

ByteCnt:        ds 1     ; Count of bytes to receive via SCI
ByteGoal:       ds 1     ; Desired number of bytes to receive via SCI
readbuf:        ds 1     ; Buffer for temporary storage of received byte

;****************************************************************************
; ----------------------- Time Clock Variables ------------------------------
;****************************************************************************

mS:             ds 1	 ; Milliseconds counter

;Memory Location $0060 (decimal 96)

mSx5:           ds 1     ; 5 Milliseconds counter

;****************************************************************************
; - Variables for integer math
;****************************************************************************

INTACC1:      rmb 4
INTACC2:      rmb 4


;****************************************************************************
; - Temporary  variables
;****************************************************************************

tmp1:        ds 1     ; used in Umul32, Linterp, Ord_table_find
tmp2:        ds 1     ; used in Umul32, Linterp, Ord_table_find
tmp3:        ds 1     ; used in Umul32, Linterp, Ord_table_find
tmp4:        ds 1     ; used in Umul32, Linterp, Ord_table_find
tmp5:        ds 1     ; used in Umul32, Linterp, Ord_table_find
tmp6:        ds 1     ; used in Umul32, Linterp
tmp7:        ds 1     ; used in Umul32, Linterp

;Memory Location $0070 (decimal 112)

tmp8:        ds 1     ; used in Umul32
tmp9:        ds 1
tmp10:       ds 1
tmp11:       ds 1
tmp12:       ds 1
tmp13:       ds 1
tmp14:       ds 1
tmp15:       ds 1
tmp16:       ds 1
tmp17:       ds 1
tmp18:       ds 1
tmp19:       ds 1
tmp20:       ds 1
local_tmp:   ds 1     ; used in "div_round"


;***************************************************************************
; - VFD Variables
;***************************************************************************

value:          ds 1     ; Value sent to VFD(instruction or data)
LineNum:        ds 1     ; Line number for VFD(for instruction)

;Memory Location $0080 (decimal 128)

ColNum:         ds 1     ; Column number for VFD(for instruction)
DatVal:         ds 1     ; Data value for VFD
ComVal:         ds 1     ; Value for VFD command data
TopVal:         ds 1     ; Value for VFD top line data
DisVal:         ds 1     ; Value for VFD bottom line variable data

;***************************************************************************
; - Variables to display VFD bottom line
;***************************************************************************

BotLin0:      ds 1     ; Bottom Line Column 0
BotLin1:      ds 1     ; Bottom Line Column 1
BotLin2:      ds 1     ; Bottom Line Column 2
BotLin3:      ds 1     ; Bottom Line Column 3
BotLin4:      ds 1     ; Bottom Line Column 4
BotLin5:      ds 1     ; Bottom Line Column 5
BotLin6:      ds 1     ; Bottom Line Column 6
BotLin7:      ds 1     ; Bottom Line Column 7
BotLin8:      ds 1     ; Bottom Line Column 8
BotLin9:      ds 1     ; Bottom Line Column 9
BotLin10:     ds 1     ; Bottom Line Column 10

;Memory Location $0090 (decimal 144)

BotLin11:     ds 1     ; Bottom Line Column 11
BotLin12:     ds 1     ; Bottom Line Column 12
BotLin13:     ds 1     ; Bottom Line Column 13
BotLin14:     ds 1     ; Bottom Line Column 14
BotLin15:     ds 1     ; Bottom Line Column 15
BotLin16:     ds 1     ; Bottom Line Column 16
BotLin17:     ds 1     ; Bottom Line Column 17
BotLin18:     ds 1     ; Bottom Line Column 18
BotLin19:     ds 1     ; Bottom Line Column 19

;***************************************************************************
; - Variables to display VFD bottom line, last pass through main loop
;***************************************************************************

BotLin0L:      ds 1     ; Bottom Line Column 0, last pass
BotLin1L:      ds 1     ; Bottom Line Column 1, last pass
BotLin2L:      ds 1     ; Bottom Line Column 2, last pass
BotLin3L:      ds 1     ; Bottom Line Column 3, last pass
BotLin4L:      ds 1     ; Bottom Line Column 4, last pass
BotLin5L:      ds 1     ; Bottom Line Column 5, last pass
BotLin6L:      ds 1     ; Bottom Line Column 6, last pass

;Memory Location $00A0 (decimal 160)

BotLin7L:      ds 1     ; Bottom Line Column 7, last pass
BotLin8L:      ds 1     ; Bottom Line Column 8, last pass
BotLin9L:      ds 1     ; Bottom Line Column 9, last pass
BotLin10L:     ds 1     ; Bottom Line Column 10, last pass
BotLin11L:     ds 1     ; Bottom Line Column 11, last pass
BotLin12L:     ds 1     ; Bottom Line Column 12, last pass
BotLin13L:     ds 1     ; Bottom Line Column 13, last pass
BotLin14L:     ds 1     ; Bottom Line Column 14, last pass
BotLin15L:     ds 1     ; Bottom Line Column 15, last pass
BotLin16L:     ds 1     ; Bottom Line Column 16, last pass
BotLin17L:     ds 1     ; Bottom Line Column 17, last pass
BotLin18L:     ds 1     ; Bottom Line Column 18, last pass
BotLin19L:     ds 1     ; Bottom Line Column 19, last pass

;***************************************************************************
; - Variables to hold ASCII conversions
;***************************************************************************

AC_100:     ds 1     ; 8 bit ASCII conversion 100s column
AC_10:      ds 1     ; 8 bit ASCII conversion 10s column
AC_1:       ds 1     ; 8 bit ASCII conversion 1s column

;Memory Location $00B0 (decimal 176)

thousands:  ds 1     ; 16 bit ASCII conversion 1000s digit
hundreds:   ds 1     ; 16 bit ASCII conversion 100s digit
tens:       ds 1     ; 16 bit ASCII conversion 10s digit
ones:       ds 1     ; 16 bit ASCII conversion 1s digit

;****************************************************************************
; - RS232 Real Time Variables
;****************************************************************************

secl:           ds 1  ; low seconds - from 0 to 255, then rollover
mapADC:         ds 1  ; Manifold Absolute Pressure ADC
matADC:         ds 1  ; Manifold Air Temp ADC
cltADC:         ds 1  ; Coolant Temperature ADC
tpsADC:         ds 1  ; Throttle Position Sensor ADC
battADC:        ds 1  ; Battery Voltage ADC
egoADC:         ds 1  ; Exhaust Gas Oxygen ADC
trimADC:        ds 1  ; Fuel/Ignition Trim ADC
baroADC:        ds 1  ; Barometric Pressure ADC
barometer:      ds 1  ; Barometer Reading - KPa (0 - 255)
barocor:        ds 1  ; Barometer Lookup Correction - percent
warmcor:        ds 1  ; Total Warmup Correction - percent

;Memory Location $00C0 (decimal 192)

aircor:         ds 1  ; Air Density Correction lookup - percent
Ftrimcor:       ds 1  ; Fuel Trim Correction Factor (85% - 115%)
gammae:         ds 1  ; Total Gamma Enrichments - percent
tpsaccel:       ds 1  ; Acceleration enrichment - percent
rpm20:          ds 1  ; Computed engine RPM - rpm/100
vecurr:         ds 1  ; Current VE value from lookup table - percent
pwcalcH:        ds 1  ; Calculated puslewidth Hi Res Hi byte
pwcalcL:        ds 1  ; Calculated puslewidth Hi Res Lo byte
pw:             ds 1  ; Calculated pulsewidth Lo Res
fd:             ds 1  ; Fuel Delivery pulsewidth Lo Res
fdSecH:         ds 1  ; Fuel Delivery pulsewidth Lo res over 1 second Hi byte
fdSecL:         ds 1  ; Fuel Delivery pulsewidth Lo res over 1 second Lo byte
;*pippprdH:       ds 1  ; PIP period predicted Hi byte
;*pippprdL:       ds 1  ; PIP period predicted Lo byte
tachH:          ds 1  ; Tachometer Period(PIP period) Hi byte
tachL:          ds 1  ; Tachometer Period(PIP period) Lo byte
spkAngFac:      ds 1  ; Spark Angle Factor(from ST table)
trmAngFac:      ds 1  ; Trim Angle FActor (calculated in MSnS300)

;Memory Location $00D0 (decimal 208)

dlyAngFac:      ds 1  ; Delay Angle Factor
odoSec:         ds 1  ; Odometer counts over 1 second
vspH:           ds 1  ; Vehicle speed period Hi byte
vspL:           ds 1  ; Vehicle speed period Lo byte
monpH:          ds 1  ; Ignition monitor period Hi byte
monpL:          ds 1  ; Ignition monitor period Lo byte
tpsp:           ds 1  ; Throttle position percent
engine:         ds 1  ; Variable bit-field to hold engine current status
alarmbits:      ds 1  ; Engine alarm status bit field
portAbits:      ds 1  ; Port A status bit field
portCbits:      ds 1  ; Port C status bit field
BnkflowHmv:     ds 1  ; Injector bank flow rate L/hr x 10 Hi byte for MV
BnkflowLmv:     ds 1  ; Injector bank flow rate L/hr x 10 Lo byte for MV

;***********************************************************************************************
; This marks the end of the real time download variables, 41 in total
;***********************************************************************************************

;****************************************************************************
; Calculated variables
;****************************************************************************

map:            ds 1  ; Manifold Absolute Pressure in KPA
mat:            ds 1  ; Manifold Temperature in degrees F + 40
clt:            ds 1  ; Engine Coolant Temperature in degrees F + 40

; - Memory Location $00E0 (decimal 224)

volts:          ds 1  ; Battery (system) voltage
egoV:           ds 1  ; Exhaust Gas Oxygen sensor voltage
lambda:         ds 1  ; Exhaust Gas Oxygen lambda
afr:            ds 1  ; Exhaust Gas Oxygen Air/Fuel Ratio
duty:           ds 1  ; Injector duty cycle
trmAng:         ds 1  ; Ignition Trim Angle
dlyAng:         ds 1  ; Ignition Delay Angle
timAng:         ds 1  ; Calculated Ignition Timing Angle
monTimAng:      ds 1  ; Monitored Ignition Timing Angle
ltrHrH:         ds 1  ; Fuel burn in Litres per Hour x 10 Hi byte
ltrHrL:         ds 1  ; Fuel burn in Litres per Hour x 10 Lo byte
kph:            ds 1  ; Vehicle speed in KPH(lo res fast update)
kph1:           ds 1  ; Vehicle speed in KPH(hi res slow update)
kmLtrH:         ds 1  ; Fuel burn over 1 second time period in kM per litre x 10
kmLtrL:         ds 1  ; Fuel burn over 1 second time period in kM per litre x 10


;****************************************************************************
; Variables used in calculations
;****************************************************************************

mmH:            ds 1  ; Millimeters travelled over 1 second Hi byte

; - Memory Location $00F0 (decimal 240)

mmL:            ds 1  ; Millimeters travelled over 1 second Lo byte
mccH:           ds 1  ; CC/1000 fuel used over 1 second Hi byte
mccL:           ds 1  ; CC/1000 fuel used over 1 second Lo byte
monDlyAngFac:   ds 1  ; Monitored Delay Angle Factor
monDlyAng:      ds 1  ; Monitored Delay Angle
monFrngAng:     ds 1  ; Monitored Firing Angle
rpmH:           ds 1  ; Engine RPM Lo Res Hi Byte
rpmL:           ds 1  ; Engine RPM Lo Res Lo byte
rpmhrH:         ds 1  ; Engine RPM Hi Res Hi Byte
rpmhrL:         ds 1  ; Engine RPM Hi Res Lo byte
BnkFloH:        ds 1  ; Injector bank flow rate in cc/sec x 1,000,000 Hi byte
BnkFloL:        ds 1  ; Injector bank flow rate in cc/sec x 1,000,000 Lo byte
blank1:         ds 1  ; Place holder
blank2:         ds 1  ; Place holder
blank3:         ds 1  ; Place holder
blank4:         ds 1  ; Place holder

; - Memory Location $0100 (decimal 256)



