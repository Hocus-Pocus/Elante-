************************************************************************************************
************************************************************************************************
;   MSnS460.inc (include file for MSnS460.asm)
;   
;   Based on the work of Bruce Bowling and Al Grippo for the original B&G and Hi Res code.
;   Also Magnus Bjelk for his megasquirtnspark and the msnsextra team as well as all the 
;   unknown contributors to the MS project.
;   Special thanks to Eric Fahlgren with the early help integrating with Megatune, and 
;   Phil Tobin for assisting in integrating with Tuner Studio
;
;   Robert Hiebert 2010
;
; Rev 1: 4/10/11 - KPA bins changed to include 17 KPA. AFR target tables converted to Lambda.
;                  ADC averaging put back in.
;
; Rev 2: 12/24/11 - Knock detector code added.
;
; Rev 3: 4/15/12 - ASE counter and bank flow rate variables and code changes
;
; Rev 4: 7/04/12 - Modify code for tachH:tachL for Hi Res RPM calcs. Average 16 readings to
;                  to smooth things out.
;
************************************************************************************************

************************************************************************************************
;***********************************************************************************************
; This variant of the Megasquirt code was developed as part of "Megacontrol", a complete 
; powertrain management system for fuel, ignition, and transmission control replacing the EECIV 
; on a relatively stock, 1992 Ford Super Duty motorhome chassis. The Winnebago Elante 33 RQ is 
; equipped with the 7.5L V8 engine, 24 #/Hr injectors and E4OD transmission.  Fall back position 
; in case of "Megacontrol" failure is to revert to EECIV control, so it is as close as 
; practical to "Plug and Play". The code is a compilation of Megasquirtnspark, MegasquirtHR, 
; Megaspark, msns-extra and some additions and revisions of my own to suit my own needs. The HR 
; version of  fuel control was chosen for best resolution running the small stock injectors in 
; "8 squirts alternating" mode. Added features are a modified version of Megabaro, Fuel and 
; Ignition Trim control, and the vehicle speed and ignition monitor inputs. A manual control for 
; the PWM bypass idle air is part of the transmission controller. A Manual Decel Fuel Cut Enable  
; and exhaust brake is also part of the transmission controller.
;***********************************************************************************************

;***********************************************************************************************
;
; ----------------------------------- MSnS460 Hardware Wiring ----------------------------------
;
;***********************************************************************************************
;
; ----- Inputs [port name - function] -----
;
;  IRQ	- PIP (Profile Ignition Pickup)Input
;             (Rising edge at 10 degrees BTDC on Pin 24,inverted to IRQ)
;  PTB0/AD0 - Manifold Absolute Pressure
;  PTB1/AD1 - Manifold Air Temperature
;  PTB2/AD2 - Coolant Temperature
;  PTB3/AD3 - Throttle Position Sensor
;  PTB4/AD4 - Battery Voltage
;  PTB5/AD5 - Exhaust Gas Oxygen
;  PTB6/AD6 - Fuel or Ignition Trim
;  PTB7/AD7 - Barometric Pressure
;  PTA2     - Decel Fuel Cut Enable
;  PTA3     - Vehicle Speed
;  PTA4     - Ignition Monitor
;  PTA5     - Fuel/Ignition Trim Select(Hi(set) = Fuel Trim, Lo(clr) = Ign Trim)
;  PTA6     - Knock Detector Input
;
; ----- Outputs [port name - function] -----
;
;  PTD4/T1CH0 - Injector 1
;  PTD5/T1CH1 - Injector 2
;  PTA0       - Fuel Pump Relay driver transistor, sinking
;  PTA1	  - SPOUT (Spark Output) driver transistor, sinking, with pullup,
;               for TFI module, 50% duty cycle, fires on rising edge
;  PTA7       - Program Loop Frequency Pin
;  PTC0	  - Accumulated Injector On Time(open collector)
;  PTC1	  - Accel LED
;  PTC2	  - Warmup LED
;
;***********************************************************************************************
;***********************************************************************************************
;
; -----Assembler Directives -----
;
; EQU    - Equate -Define label equal to operand
; RMB    - Reserve Memory Byte -  Reserve number of bytes in memory
;          as specified by operand
; DS#n   - Define Storage - Reserve n number of bytes in memory
;          as specified by operand
;
;***********************************************************************************************

;***********************************************************************************************
; --------------------------------- Input Port Equates -----------------------------------------
;***********************************************************************************************

DFC_en      equ 2     ; Decel Fuel Cut Enable on Port A2
Veh_Spd     equ 3     ; Vehicle Speed Input on Port A3
Ign_Mon     equ 4     ; Ignition Monitor on Port A4
Trm_Sel     equ 5     ; Fuel/ Ignition Trim Select on Port A5
Knk_Det:    equ 6     ; Knock detected on Port A6

;***********************************************************************************************
; ----------------------------------- Output Port Equates --------------------------------------
;***********************************************************************************************

inject1     equ 4     ; Injector 1 on Port D4
inject2     equ 5     ; Injector 2 on Port D5
fuelp       equ 0     ; Fuel Pump Relay on Port A0
spark       equ 1     ; Spark Output on Port A1
Loopchk     equ 7     ; Program loop frequency pin on Port A7
aiot        equ 0     ; 25.5ms injector on time pulse on Port C0
aled        equ 1     ; "Accelleration Enrichments" on Port C1
wled        equ 2     ; "Warmup enrichments" on Port C2

;***********************************************************************************************
; ---------------------- "Squirt" Event Scheduling  bit field equates  -------------------------
;***********************************************************************************************

inj1:       equ 0     ; 1 = INJ1 fire          0 = INJ1 no fire
inj2:       equ 1     ; 1 = INJ2 fire          0 = INJ2 no fire
sched1:     equ 2     ; 1 = INJ1 scheduled     0 = INJ1 not scheduled
firing1:    equ 3     ; 1 = INJ1 firing        0 = INJ1 not firing
sched2:     equ 4     ; 1 = INJ2 scheduled     0 = INJ2 not scheduled
firing2:    equ 5     ; 1 = INJ2 firing        0 = INJ2 not firing

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
; ------------------------ "Engine2" Operating Status bit field equates ------------------------
;***********************************************************************************************

run         equ 0     ; 1 = in Run mode              0 = not in Run mode
schedspk    equ 1     ; 1 = Spark scheduled flag     0 = "schedspk" cleared
sparking    equ 2     ; 1 = Spark in progress flag   0 = "sparking" cleared
started     equ 3     ; 1 = Engine has started       0 = Engine not started
cant_crank  equ 4     ; 1 = Crank mode inhibit       0 = Crank mode OK
cant_delay  equ 5     ; 1 = Crank mode delay         0 = Crank mode not delay


;***********************************************************************************************
; -------------------------- "inputs" Operating Status bit field equates -----------------------
;***********************************************************************************************

piprise     equ 0     ; 1 = PIP rising edge flag       0 = "piprise" cleared
adcc        equ 1     ; 1 = ADC complete flag          0 = "adcc" cleared
MONen       equ 2     ; 1 = Ign Mon KBI enabled        0 = MON KBI disabled
VSen        equ 3     ; 1 = Veh Spd KBI enabled        0 = VS KBI disabled
clock100    equ 4     ; 1 = 100mS clock tick           0 = "clock100" cleared
clock250    equ 5     ; 1 = 250mS clock tick           0 = "clock250" cleared
clock500    equ 6     ; 1 = 500mS clock tick           0 = "clock500" cleared

;***********************************************************************************************
; ----------------------- "alarmbits" Operating Status bit field equates -----------------------
;***********************************************************************************************

knocking:   equ 0     ; 1 = Engine Knock               0 = No Engine Engine Knock
REVL:       equ 1     ; 1 = Engine Rev Limit           0 = No Engine Rev Limit 
fldClr:     equ 2     ; 1 = Engine Flood Clear         0 = No Engine Flood Clear

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

;***********************************************************************************************
; ---------------------------------------- Misc. equates ---------------------------------------
;***********************************************************************************************

Stat_Tim_Ang   equ $0A     ; Static Timing Angle 10 degrees BTDC
PAGESIZE       equ $A8     ;(168)Size of the data page, used by the P and C commands

ms_ram_start:

;***********************************************************************************************
; --------------------------- RAM Variables (512 bytes available) ------------------------------
; I/O registers from $0000 to $0039 (decimal 0 to 63)
; Direct page addressing from $0000 to $00FF (decimal 0 to 255)
; Ram end at $023F (decimal 575)
;***********************************************************************************************
;***********************************************************************************************
; ---------------------- RS232 Real Time Download and Datalog Variables ------------------------
;***********************************************************************************************

; - Memory Location $0040 (decimal 64)

secl:           ds 1 ; Low seconds - from 0 to 255, then rollover
map:	          ds 1 ; Manifold Absolute Pressure ADC Raw Reading - KPa (0 - 255)(AD0)
mat:	          ds 1 ; Manifold Air Temp ADC Raw Reading - counts (0 - 255)(AD1)
clt:	          ds 1 ; Coolant Temperature ADC Raw Reading - counts (0 - 255)(AD2)
tps:	          ds 1 ; Throttle Position Sensor ADC Raw Reading - counts (0 - 255)(AD3)
batt:	          ds 1 ; Battery Voltage ADC Raw Reading - counts (0 - 255)(AD4)
ego:            ds 1 ; Exhaust Gas Oxygen ADC Raw Reading - counts (0 - 255)(AD5)
trim:           ds 1 ; Fuel/Ignition Trim ADC Raw Reading - counts (0 - 255)(AD6)
baro:           ds 1 ; Barometric Pressure ADC Raw Reading - counts (0 - 255)(AD7)
barometer:      ds 1 ; Current barometer reading in KPA (for MV)
barocor:        ds 1 ; Barometer Lookup Correction - percent
warmcor:        ds 1 ; Total Warmup Correction - percent
aircor:         ds 1 ; Air Density Correction lookup - percent
Ftrimcor:       ds 1 ; Fuel Trim Correction Factor (85% - 115%)
gammae:         ds 1 ; Total Gamma Enrichments - percent
tpsaccel:       ds 1 ; Acceleration enrichment - percent

; - Memory Location $0050 (decimal 80)

rpm20:          ds 1 ; Computed engine RPM - rpm/20
vecurr:         ds 1 ; Current VE value from lookup table - percent
pwcalcH:        ds 1 ; high order of calculated pulsewith (16 bits)
pwcalcL:        ds 1 ; low order of calculated pulsewidth (16 bits)
pw:             ds 1 ; injector squirt time in 1/10 millesec (0 to 25.5 millisec) - applied
fd:             ds 1 ; Fuel Delivery PW Lo Res
fdSecH:         ds 1 ; Fuel Delivery PW Lo Res over 1 second Hi byte
fdSecL:         ds 1 ; Fuel Delivery PW Lo Res over 1 second Lo byte
tachH:          ds 1 ; Tachometer period averaged over 250 mS Hi byte 
tachL:          ds 1 ; Tachometer period averaged over 250 mS Lo byte 
Spk_Ang_Fac:    ds 1 ; Spark Angle Factor (from ST table)
Trm_Ang_Fac:    ds 1 ; Trim Angle Factor (from IgnTrimcor table)
Dly_Ang_Fac:    ds 1 ; Delay Angle Factor (Spk_Ang_Fac + Trm_Ang_Fac)
odoSec:         ds 1 ; Odometer counts over 1 second
VS_pH:          ds 1 ; Vehicle speed period Hi byte
VS_pL:          ds 1 ; Vehicle speed period Lo byte

; - Memory Location $0060 (decimal 96)

MON_pH:         ds 1 ; MON period Hi byte (MON_tsH - PIP_tsH)
MON_pL:         ds 1 ; MON period Lo byte (MON_tsL - PIP_tsL)
tpsp:           ds 1 ; Throttle position percent
engine:         ds 1 ; Variable bit-field to hold engine current status
alarmbits:      ds 1 ; Engine Alarm Status Bit Field
portAbits:      ds 1 ; Port A status Bit field
portCbits:      ds 1 ; Port C Status Bit Field
BnkflowHmv:     ds 1 ; Injector bank flow rate L/hr x 10 Hi byte for MV
BnkflowLmv:     ds 1 ; Injector bank flow rate L/hr x 10 Lo byte for MV

;***********************************************************************************************
; This marks the end of the real time download variables, 41 in total
;***********************************************************************************************

;***********************************************************************************************
; Bit Field Variables
;***********************************************************************************************

engine2:        ds 1 ; Variable bit-field to hold engine current status #2
squirt:         ds 1 ; Event variable bit field for Injector Firing
inputs:         ds 1 ; Bit field variable for input status flags

;***********************************************************************************************
; ADC Conversion Variables
;***********************************************************************************************

kpa:            ds 1 ; MAP value in units of KPa
coolant:        ds 1 ; Coolant temperature in Degrees F + 40 (allows -40 degress to fit in int)

;***********************************************************************************************
; ADC Calculation Variables
;***********************************************************************************************

adsel:      ds 1 ; ADC Selector Variable

;***********************************************************************************************
; Fuel Calculation Variables
;***********************************************************************************************

pwnadH:        ds 1     ; Calculated PW without Accel and deadband Hi byte

; - Memory Location $0070 (decimal 112)

pwnadL:        ds 1     ; Calculated PW without Accel and deadband Lo byte
fdhrH:         ds 1     ; Calc PW without deadband Hi byte (fuel delivery)
fdhrL:         ds 1     ; Calc PW without deadband Lo byte (fuel delivery)
deadband:      ds 1     ; Injector deadband in 100uS resolution
tpsfuelcut:    ds 1     ; TPS Fuel Cut (percent)

;***********************************************************************************************
; Engine RPM -> RPM = 12000/(ncyl * (rpmph - rpmpl))
;***********************************************************************************************

rpmph:      ds 1 ; High part of RPM Period
rpmpl:      ds 1 ; Low part of RPM Period
rpmch:      ds 1 ; Counter for high part of RPM
rpmcl:      ds 1 ; Counter for low part of RPM

;***********************************************************************************************
; Timer Variables
;***********************************************************************************************

mSx250:     ds 1 ; 250 Milliseconds counter
mSx100:     ds 1 ; 100 Milliseconds counter
mS:         ds 1 ; Milliseconds counter
uSx100:     ds 1 ; 100 Microseconds counter
sech:       ds 1 ; high seconds - rollover at 65536 secs (1110.933 minutes, 18.51 hours)
tpsaclk:    ds 1 ; TPS enrichment timer clock in 0.1 second resolution
asecount:   ds 1 ; Counter value for after-start enrichment counter - every ignition pulse

; - Memory Location $0080 (decimal 128)

last_tps:   ds 1 ; TPS reading updated every 0.1 seconds
igncount:   ds 1 ; Ignition pulse counter
altcount:   ds 1 ; Alternate count selector
tpsaclkcmp: ds 1 ; Comparison value for TPS acceleration time - from lookup table

;***********************************************************************************************
; SCI parameters/variables
;***********************************************************************************************

txcnt:      ds 1 ; SCI transmitter count (incremented)
txgoal:     ds 1 ; SCI number of bytes to transmit
txmode:     ds 1 ; Transmit mode flag
rxoffset:   ds 1 ; offset placeholder when receiving VE/constants vis. SCI

;***********************************************************************************************
; Routines for integer math - storage variables
;***********************************************************************************************

INTACC1       rmb 4
INTACC2       rmb 4


; - Memory Location $0090 (decimal 144)(at INTACC2+2)

;***********************************************************************************************
; Temporary variables
;***********************************************************************************************

tmp1    ds 1
tmp2    ds 1
tmp3    ds 1
tmp4    ds 1
tmp5    ds 1
tmp6    ds 1
tmp7    ds 1
tmp8    ds 1
tmp9    ds 1
tmp10   ds 1
tmp11   ds 1
tmp12   ds 1
tmp13   ds 1
tmp14   ds 1
tmp15   ds 1
tmp16   ds 1

; - Memory location $00A0 (decimal 160)

tmp17   ds 1
tmp18   ds 1
tmp19   ds 1
tmp20   ds 1
tmp21   ds 1
tmp22   ds 1


;***********************************************************************************************
; ----------------------------- Spark Timing  Control Variables --------------------------------
;***********************************************************************************************

PIP_tsH:        ds 1     ; PIP timestamp current reading Hi byte
PIP_tsL:        ds 1     ; PIP timestamp current reading Lo byte
PIP_tsH_prv:    ds 1     ; PIP timestamp previous reading Hi byte
PIP_tsL_prv:    ds 1     ; PIP timestamp previous reading Lo byte
PIP_pH_prv:     ds 1     ; PIP period Hi byte previous
PIP_pL_prv:     ds 1     ; PIP period Lo byte previous
PIP_pH_dif:     ds 1     ; PIP period difference Hi byte(PIP_pH - PIP_pH_prv)
PIP_pL_dif:     ds 1     ; PIP period difference Lo byte(PIP_pL - PIP_pL_prv)
PIP_pH:         ds 1     ; PIP period Hi byte (PIP_tsH - PIP_tsH_prv)
PIP_pL:         ds 1     ; PIP period Lo byte (PIP_tsL - PIP_tsL_prv)

; - Memory location $00B0 (decimal 176)

PIP_pH_pred:    ds 1     ; Predicted PIP period Hi byte (PIP_pH + PIP_pH_dif)
PIP_pL_pred:    ds 1     ; Predicted PIP period Lo byte (PIP_pL + PIP_pL_dif)
Delay_pH:       ds 1     ; Delay period from PIP to SPOUT, Hi byte
Delay_pL:       ds 1     ; Delay period from PIP to SPOUT, Lo byte
                         ;(16 bit result of Mul_Hi:Mul_Mid:Mul_Lo / $FF)
CH1_onH:        ds 1     ; T2CH1 output compare for SPOUT rising, Hi byte
CH1_onL:        ds 1     ; T2CH1 output compare for SPOUT rising, Lo byte
                         ;(PIP_tsH:PIP_tsL + Delay_pH:Delay_pL)
SPOUT_tsH:      ds 1     ; SPOUT timestamp current reading Hi byte
SPOUT_tsL:      ds 1     ; SPOUT timestamp current reading Lo byte
PIP_periodH:    ds 1     ; PIP period Hi byte (for SPOUT off calcs)
PIP_periodL:    ds 1     ; PIP period Lo byte (for SPOUT off calcs)
SPOUTon_pH:     ds 1     ; SPOUT on period Hi byte
SPOUTon_pL:     ds 1     ; SPOUT on period Lo byte
                         ;(50% duty cycle, half of PIP period)
CH1_offH:       ds 1     ; T2CH1 output compare for SPOUT falling, Hi byte
CH1_offL:       ds 1     ; T2CH1 output compare for SPOUT falling, Lo byte
                         ;(SPOUT_tsH:SPOUT_tsL + SPOUTon_pH:SPOUTon_pL)


;***********************************************************************************************
; ------------------------- Ignition Monitor Calculation Variables -----------------------------
;***********************************************************************************************

MON_tsH:          ds 1     ; MON timestamp Hi byte current reading
MON_tsL:          ds 1     ; MON timestamp Lo byte current reading

; - Memory location $00C0  (decimal 192)

;***********************************************************************************************
; ------------------------------------ Misc. Variables -----------------------------------------
;***********************************************************************************************

LoopCounter:    ds 1     ; Loop counter for main loop frequency check
MinPIP:         ds 1     ; Minimum PIP signals required for period calcs
burnSrc:        ds 2     ; Burn routine variable
page:           ds 1     ; Page selection variable
flocker:        ds 1     ; Burner locking semaphor

;***********************************************************************************************
; ------------------------------------- Burner variables ---------------------------------------
;***********************************************************************************************

burnDst:        ds 2     ; Burn routine variable
burnCount:      ds 1     ; Burn routine variable

;***********************************************************************************************
; ------------------------------ Fuel burn calculation variables -------------------------------
;***********************************************************************************************

fdcntr:         ds 1     ; Counter for AIOT trigger on
aiotcntr:       ds 1     ; Counter for AIOT trigger off
fdtH:           ds 1     ; Fuel deleivery Lo Res total Hi byte
fdtL:           ds 1     ; Fuel deleivery Lo Res total Lo byte

;***********************************************************************************************
; ---------------------------- Vehicle speed calculation variables -----------------------------
;***********************************************************************************************

VScH:          ds 1      ; 100uS counter for vehicle speed Hi byte
VScL:          ds 1      ; 100uS counter for MPH Lo byte
odo:           ds 1      ; Odometer pulse counter

; - Memory location $00CA (decimal 202)

;***********************************************************************************************
; Tachometer calculation variables
;***********************************************************************************************

tachcurH:       ds 1    ; Tachometer period current Hi byte 
tachcurL:       ds 1    ; Tachometer period current Lo byte 
tachTH:         ds 1    ; Tachometer period total Hi byte 
tachTM:         ds 1    ; Tachometer period total Mid byte 
tachTL:         ds 1    ; Tachometer period total Lo byte
tachcnt:        ds 1    ; Tachometer counter for tachometer period averaging time     
blank1:         ds 1    ; Placeholder     
blank2:         ds 1    ; Placeholder     
blank3:         ds 1    ; Placeholder     
blank4:         ds 1    ; Placeholder     
blank5:         ds 1    ; Placeholder     
blank6:         ds 1    ; Placeholder     
blank7:         ds 1    ; Placeholder     
blank8:         ds 1    ; Placeholder     
blank9:         ds 1    ; Placeholder     
blank10:        ds 1    ; Placeholder     

; - Memory location $00DA (decimal 202)

ms_ram_end:


;***********************************************************************************************
; - Page 1(TS page 1) VE table (table copied into RAM on demand)
;***********************************************************************************************

ms_rf_start:

VE_r               rmb $90     ; 144 bytes for VE Table

; - Memory location $016A (decimal 362)
 
RPMRANGEVE_r       rmb $0C     ; 12 VE Table RPM Bins for 2-D interpolation
KPARANGEVE_r       rmb $0C     ; 12 VE Table MAP Pressure Bins for 2_D interpolation

; - Memory location $0182 (decimal 386)

ms_rf_end:

;***********************************************************************************************
; - Page 2(TS page 2) ST table (table copied into RAM on demand)
;***********************************************************************************************

ST_r             equ  ms_rf_start         ; Spark timing table
RPMRANGEST_r     equ {ms_rf_start + $90}  ; Spark timing RPM bins for 2-D interpolation
KPARANGEST_r     equ {ms_rf_start + $9C}  ; Spark timing MAP pressure bins for 2-D interpolation

;***********************************************************************************************
; - Page 3(TS page 3) AFR1 table (table copied into RAM on demand)(Used by MLV only)
;***********************************************************************************************

AFR1_r             equ  ms_rf_start         ; AFR1 table
RPMRANGEAFR1_r     equ {ms_rf_start + $90}  ; AFR1 RPM bins for 2-D interpolation
KPARANGEAFR1_r     equ {ms_rf_start + $9C}  ; AFR1 MAP pressure bins for 2-D interpolation

;***********************************************************************************************
; - Page 4(TS page 4) AFR2 table (table copied into RAM on demand)(Used by MLV only)
;***********************************************************************************************

AFR2_r             equ  ms_rf_start         ; AFR2 table
RPMRANGEAFR2_r     equ {ms_rf_start + $90}  ; AFR2 RPM bins for 2-D interpolation
KPARANGEAFR2_r     equ {ms_rf_start + $9C}  ; AFR2 MAP pressure bins for 2-D interpolation

;***********************************************************************************************
; - Page 5(TS page 5) Constants always run from Flash so no need to copy to RAM
;***********************************************************************************************

;*WWURANGE         rmb $0A     ; WWU Table temperature Bins for 2_D interp.
;*WWU              rmb $0A     ; Warmup bins(fn temp)
;*TPSDOTRATE       rmb $04     ; TPSAQ table TPS count DOT for 2_D interp.
;*TPSAQ            rmb $04     ; TPS accel amount (fn TPSDOT)(0.1mS res)
;*cwu              rmb $01     ; Crank Pulse Width at -40 F)(0.1mS res)
;*cwh              rmb $01     ; Crank Pulse Width at 170 F)(0.1mS res)
;*awev             rmb $01     ; After-start Warmup % enrichment add-on
;*awc              rmb $01     ; After-start number of cycles
;*tpsacold         rmb $01     ; Cold accel amount (at -40F)(0.1ms res)
;*tpsthresh        rmb $01     ; Accel TPS DOT threshold
;*tpsasync         rmb $01     ; TPS Acceleration clock value
;*req_fuel         rmb $01     ; Fuel Constant
;*divider          rmb $01     ; IRQ divide factor for pulse
;*Alternate        rmb $01     ; Alternate injector drivers
;*InjOpen          rmb $01     ; Injector Open Time
;*battfac          rmb $01     ; Battery Gamma Factor
;*floodClear       rmb $01     ; tpsADC value for flood clear
;*acmult           rmb $01     ; Accel cold mult factor (%)
;*pru              rmb $01     ; Primer Pulse Width at -40 F)(0.1mS res)
;*prh              rmb $01     ; Primer Pulse Width at 170 F)(0.1mS res)
;*PIP_Angle        rmb $01     ; PIP Angle in crank degrees(4cyl=180. 6cyl=120, 8cyl=90
;*CT_cnt           rmb $01     ; Closed throttle position ADC count
;*WOT_cnt          rmb $01     ; Wide Open throttle position ADC count
;*ASEhi            rmb $01     ; Set point above which ASE is active
;*ASElo            rmb $01     ; Set point below which ASE is active
;*tpsdq            rmb $01     ; Deacceleration fuel cut
;*DFC_RPM          rmb $01     ; Decel Fuel Cut min RPM(rpm20)
;*DFC_TPS          rmb $01     ; Decel Fuel Cut min TPS ADC counts
;*dbCor            rmb $01     ; Deadband correction added to "fd"
;*BnkflowH         rmb $01     ; BnkflowH        Injector bank flow rate L/hr x 10 Hi byte 
                               ;                 configuration variable
;*BnkflowL         rmb $01     ; BnkflowL        Injector bank flow rate L/hr x 10 Lo byte 
                               ;                 configuration variable
;*ASEtype          rmb $01     ; ASEtype         Configuration variable for ASE counter type
                               ;                 0 = ignition cycles, 1 = 100 ms counter, 
                               ;                 2 = 250 ms counter, 3 = 500 ms counter 
                               ;                 4 = 1 second counter


;***********************************************************************************************
; - RAM copy equates
;***********************************************************************************************

ms_ram_size       equ {ms_ram_end-ms_ram_start}
ms_rf_size        equ {ms_rf_end-ms_rf_start}
ms_total_ram_size equ {ms_rf_end-ms_ram_start}

