;****************************************************************************
;
;                     MS_TECA_V12_E150_03_02_10a.asm 3/02/10
;                              MT V2.25 compatable
;      External Axiomatic proportional solonoid driver EPC control version
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
;  _V11:
;  This version of code utilizes the Axiomatic model 4000174 proportional
;  solenoid driver for EPC control. It has current regulation for varying
;  supply voltages and solenoid resistance. It also has adjustable dither
;  from 0-10% max current amplitude and 70-350HZ frequency.
;  Full output is 1.2 amps, but can be trimmed for the desired 1 amp max.
;  Control input is 0-5 volts, with full input resulting in full output.
;
;  Attempts to provide current control with voltage and temperature
;  compensation of EPC PWM on V10 proved to be unsatisfactory. All reference
;  to EPC PWM in this version is to be taken as PWM to produce the analog
;  0-5 volts for the EPC controller.
;
;  This version also changes the shift up logic so that it is no longer
;  necessary to wait until the EPC pressure rise sequence has timed out
;  before the next up shift can be made. TCC apply is also modified in this
;  way.
;****************************************************************************
;****************************************************************************
;  _V12:
;  This version adds an EPC table for Man1, and a decel EPC variable for Man1
;  to keep the pressures within stock limits. The shift up/down debounce
;  timer values are now flash configurable to try eliminate skipped shifts.
;****************************************************************************
;****************************************************************************
;  _V12_E150_03_02_10a:
;  This version just has latest configuration constants coded.
;****************************************************************************

;****************************************************************************
; - E4OD Information:
;
;  MLPS is a special segmented variable resistor mounted on the
;  Manual Gear Selector Lever Shaft. It is supplied with regulated
;  5 volts and is in series with a 560 ohm resistor. The ADC
;  reads the voltage drop across the variable resistor, values as follows:
;
;  Park     4607 ohms  4.458 volts  228 ADC counts
;           3770 ohms  4.353 volts  223 ADC counts  P (226)
;  Reverse  1593 ohms  3.7 volts    189 ADC counts
;           1304 ohms  3.498 volts  179 ADC counts  R (184)
;  Neutral  807 ohms   2.952 volts  151 ADC counts
;           660 ohms   2.705 volts  138 ADC counts  N (144)
;  Drive    442 ohms   2.206 volts  113 ADC counts
;           361 ohms   1.96 volts   100 ADC counts  D (106)
;  Man2     232 ohms   1.465 volts   75 ADC counts
;           190 ohms   1.267 volts   65 ADC counts  M2 (70)
;  Man1     95 ohms    0.725 volts   37 ADC counts
;           78 ohms    0.612 volts   31 ADC counts  M1 (34)
;
;  TPS input 0-5 volts potentiometer equivilent to
;  560 ohm plus 3k pot plus 560 ohm
:
;  Throttle Plate Angle Chart:
;  TPS ANGLE	Volts	A/D counts (.004887 volt per count)
;    0'     0.601    31
;    13'    1.201    61
;    26'    1.820    93
;    39'    2.429    124
;    52'    3.039    156
;    65'    3.648    187
;    78'    4.257    218
;    84'    4.538    232
;    91'    4.866    249
;
;  4.9L Engine:
;   Closed Throttle = 0.83V = 42 counts
;   WOT             = 4.45V = 228 counts
;
;  7.5L Engine:
;   Closed Throttle = 0.92V = 47 counts
;   WOT             = 4.53V = 232 counts
;
;  Line Pressure chart:
;  Gear IDLEpsi     WOT     STALLpsi
;  p     55-65                ---
;  R     75-99    240-265    (100%)
;  N     55-65                ---
;  D     55-65    156-174    (66%)
;  M2    55-65    156-174    (66%)
;  M1    75-99    157-186    (70%)
;  TCC regulated at 125 max  (48%)
;
;  Solonoid Application Chart:
;  MLPS   GEAR   SS1   SS2   CCS   TCC
;  P,R,N  1      on    off   off   off
;  D      1      on    off   on    on/off
;  D      2      on    on    on    on/off
;  D      3      off   on    on    on/off
;  D      4      off   off   off   on/off
;  Man2   2      off   off   on    on/off
;  Man1   1      on    off   on    off
;
;  Solonoid resistance(depending on temperature and production differences)
;  SS1,SS2,TCC,CCS, 20-30 ohms
;  EPC, 4.0-6.5 ohms
;
;  The Electronic Pressure Control Solonoid is a "variable force" solonoid
;  that operates on a current control of between 0 and 1 amp. Maximum current
;  of 1 amp produces a line pressure of ~0PSI. No current will produce maximum
;  line pressure depending on engine speed and gear selection. Loss of EPC
;  signal will result in maximum pressure.
;
;  Observations of OEM EPC PWM proved very difficult to determine. Best
;  estimate is a frequency of between ~1.4KHZ at max current, to ~20KHZ.
;  at min current With amplitude dither to prevent solonoid "stiction",
;  of between 160HZ and 210HZ. Dither amplitude proved unable to determine.
;  The first model of this controller successfully used a commercial
;  proportioning solonoid driver that had a frequency of 10KHZ with dither
;  frequency set at ~200HZ and amplitude set at max value of 10%
;
;  Generally speaking, Line pressure is proportional to throttle position,
;  the exception being steady throttle state above stall. Under these
;  conditions, line pressure is determined from a "torque table", to taylor
;  pressure to engine torque output.
;  Separate "curves" are provided for the following conditions:
;  - Stall speed (below ~ 4MPH)
;  - Above stall speed
;  - 1-2 shift
;  - 2-3 shift
;  - 3-4 shift
;  A constant line pressure of ~125 PSI (regulated maximum) is used for
;  Torque Converter Clutch applications and Decel conditions when the TCC
;  is applied.
;
;  Note! SS2 takes longer to apply than SS1, so SS1 is delayed in a
;  Manual Lever change from Man2 to Drive. (Stays in second gear)
;
;  It is necessay to use time delays for the following four conditions:
;  - During gear changes and TCC application to allow line pressure
;    to build up before solonoid state changes.
;  - To provide a time delay off to make sure line pressure is raised
;    for a sufficient length of time to ensure completion of friction
;    element state changes.
;  - To ensure that CCS is not energised in forth gear.
;  - To allow for the shift solonoid rate change on the Man2 to Drive shift.
;
;  TCC application is prohibited below engine speeds of ~1200RPM
;
;  Sequential Gear Selector Joy Stick contacts active low
;  (downstate = 0)
;
;  Torque Converter Clutch Selector Joy Stick contacts active low
;  (downstate = 0)
;
;  Outputs active low
;
; - OEM Observations:
;
;  - In Park, Neutral, and Reverse, EPC-sol voltage is inversly proportional
;    to TPS voltage (OEM TPS max 3.8v).
;  - In stall conditions (below aprox 4MPH), in Drive and Man2, EPC-sol
;    voltage is inversly proportional to TPS voltage (OEM TPS max 3.8v).
;  - Above stall conditions, in D and Man2, EPC-sol voltage is inversly
;    proportional to TPS voltage (OEM TPS max 3.8v), but EPC-sol voltage
;    min 3.8v (76%).
;  - In Man1, EPC-sol voltage is inversly proportional to TPS voltage
;    (OEM TPS max 3.8v), but EPC-sol voltage min 2.5v (50%).
;  - During upshifts, EPC-sol voltage is increased for a time duration
;    sufficient to complete the shift, then returns to above stall
;    condition.
;  - During TCC application, EPC-sol voltage is increased for a time duration
;    duration sufficient to complete the application.
;  - Above aprox 1200RPM, and at closed throttle, EPC-sol current
;    is reduced enough to result in 125 PSI line pressure to accomodate
;    engine braking in D4, D3 and Man2.
;
;  - PIP inputs ~12v,3 pulse per engine revolution for 6 cyl engine
;  - PIP inputs ~12v,4 pulse per engine revolution for 8 cyl engine
;
;  - VSS inputs >12v square wave AC peak to peak, 8000 pulse per mile
;
;****************************************************************************

;****************************************************************************
; - OEM Idle Air Control Solonoid information:
;
;  - IAC Solonoid resistance 6.0-13.0 ohms (internal fly back diode)
;    Observations of IAC solonoid PWM was a frequency of 161HZ.
;****************************************************************************

;****************************************************************************
; - Decel Fuel Cut permissive and Exhaust Brake information:
;
;   The engine controller has provision to limit the injector pulse width
;   to a low enough value that it doesn't have time to exceed the injector
;   dead band, so no fuel will be injected. The transmission controller has
;   a permissive for this condition. The transmission must be in a gear that
;   supports engine braking, the TPS must be at closed throttle, engine RPMs
;   must be above the "RPMtcc" set point, and the torque converter clutch
;   must be applied.
;   If the permissives for the TCC are met, and it is applied, the DFC
;   permissive can be enabled by depressing the "DFCen" switch momentarely.
;   It will be disabled automatically if any of the permissives are no
;   longer met, or the "DFCdis" switch is pressed momentarely.
;   The exhaust brake has the same permissive requirements as the DFC enable.
;   It can only be applied after the DFC permissive has been enabled, by
;   keeping the "DFCen" switch depressed for the "DFCdel" time period. The
;   Exhaust brake is released in the same manner as the DFC permissive.
;****************************************************************************


;****************************************************************************

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
;  IRQ	 - Engine Speed Sensor,(invert to IRQ)         - Pin 21
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
;  PTD4/T1CH0 - Electronic Pressure Control voltage            - Pin 37
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


.header 'MegaSquirt'    	    ; Listing file title
.pagewidth 130          	    ; Listing file width
.pagelength 90          	    ; Listing file height

.nolist                           ; Turn off listing file
     include "gp32.equ"           ; Include HC 908 equates
.list                             ; Turn on listing file
     org      ram_start           ; Origin  Memory location $0040=64
     include "MS_TECA_V12.inc"    ; Include definitions for MS_TECA_V12.asm

;***************************************************************************
;
; Main Routine Here - Initialization and main loop
;
; Note: Org down 256 bytes below the "rom_start" point
;       because of erase bug in bootloader routine
;
; Note: Items commented out after the Start entry point are
;       taken care of in the Boot_R12.asm code
;
;***************************************************************************


     org   {rom_start + 256}     ; Origin at memory location
                                 ; $8000+256 = 32,768+256 = 33,024=$8100

Start:
     ldhx   #init_stack+1     ; Load index register with value in
                              ; init_stack+1(Set the stack Pointer)
     txs                      ; Transfer value in index register Lo byte
                              ; to stack
                              ;(Move before burner to avoid conflict)

;* Note - uncomment this code if you do not use the Bootloader to initilize *
;       clra
;	sta	copctl
;	mov	#%00000001,config2
;	mov	#%00001001,config1
;	mov	#%00000001,config1
;	ldhx	#ram_last+1		; Set the stack Pointer
;	txs				;  to the bottom of RAM

;****************************************************************************
; - Set the phase lock loop for a bus frequency of 8.003584mhz
;  (Boot loader initially sets it at 7.3728mhz)
;****************************************************************************

;PllSet:
	bclr	BCS,pctl          ; Select external Clock Reference
	bclr	PLLON,pctl        ; Turn Of PLL
	mov	#$02,pctl         ; Set P and E Bits
	mov	#$D0,pmrs         ; Set L ($C0 for 7.37 MHz)
	mov	#$03,pmsh         ; Set N (MSB)
	mov	#$D1,pmsl         ; Set N (LSB) ($84 for 7.37 MHz)
	bset	AUTO,pbwc         ; Enable automatic bandwidth control
	bset	PLLON,pctl        ; Turn back on PLL
PLL_wait:
     brclr   LOCK,pbwc,PLL_wait   ; Wait for PLL to lock
     bset    BCS,pctl             ; Select VCO as base clock

;****************************************************************************
; - Set up the port data-direction registers, Set directions,
;   Preset state of pins to become outputs
;****************************************************************************

; Port A
     mov     #$FF,PTAPUE     ; Move %11111111 into Port A pullup register
                             ;(Set all pullups)
     mov     #$FF,PORTA      ; Move %11111111 into Port A Data Register
                             ;(preinit all pins Hi, no input signals)
     clr     DDRA            ; Clear Port A Data Direction Register
                             ;(Inputs on PTA7,6,5,4,3,2,1,0)
                             ;(= Shiftup,Shiftdn,TCCapp,TCCrel,DFCen,
                             ;(DFCdis,ExhPS,VehSpd,

; Port B
     clr     PORTB           ; Clear Port B Data Register
                             ;(Preinit all pins low)
     clr     DDRB            ; Clear Port B Data Direction Register
                             ;(Set as ADC inputs, "ADSEL" selects channel)

; Port C
     mov     #$1F,PORTC      ; Move %00011111 into Port C Data Register
                             ;(preinit output pins Hi, no outputs)
     lda     #$1F            ; Load accumulator with %00011111
                             ; (set up port directions, 1 = out)
     sta     DDRC            ; Copy to Port C Data Direction Register
                             ; Inputs on PTC7,6,5 = NA,NA,NA
                             ; Outputs on PTC4,3,2,1,0
                             ; = ExhBrk,CCS,TCC,SS1,SS2

; Port D
     mov     #$FF,PTDPUE     ; Move %11111111 into Port E pullup register
                             ;(Set all pullups)
     mov     #$3C,PORTD      ; Move %00111100 into Port D Data Register
                             ;(preinit output pins Hi, no outputs)
     lda     #$FC            ; Load accumulator with %11111100
                             ; (init port directions 1 = out)
     sta     DDRD            ; Copy to Port D Data Direction Register
                             ; Inputs on PTD1,0 = H2,H1
                             ; Outputs on PTD7,6,5,4,3,2
                             ; = NA,NA,IACsol,EPCsol,LoopFrq,DFCper

; Port E
     clr     PORTE           ; Clear Port E Data Register (to avoid glitches)
     lda     #$01            ; Load accumulator with %00000001
                             ; (set up port directions, 1 = out)
                             ; (Serial Comm Port)
     sta     DDRE            ; Copy to Port E Data Direction Register


;****************************************************************************
; Set up TIM2 as a free running ~1us counter. Set Channel 0 output compare
; to generate the ~100us(0.1ms) clock tick interupt vector "TIM2CH0_ISR:"
;****************************************************************************

     mov     #$33,T2SC       ; Move %00110011 into Timer2
                             ; Status and Control Register
                             ;(Disable interupts, stop timer)
                             ;(Prescale and counter cleared))
                             ;(Prescale for bus frequency / 8)
     mov     #$FF,T2MODH     ; Move decimal 255 into T2 modulo reg Hi
     mov     #$FF,T2MODL     ; Move decimal 255 into T2 modulo reg Lo
                             ;(free running timer)
     mov     #$00,T2CH0H     ; Move decimal 0 into T1CH0 O/C register Hi
     mov     #$64,T2CH0L     ; Move decimal 100 into T1CH0 O/C register Lo
                             ;(~100uS)=(~0.1ms)
     mov     #$54,T2SC0      ; Move %01010100 into Timer2
                             ; channel 0 status and control register
                             ; (Output compare, interrupt enabled)
     mov     #$03,T2SC       ; Move %00000011 into Timer2
                             ; Status and Control Register
                             ; Disable interupts, counter Active
                             ; Prescale for bus frequency / 8
                             ; 8,003584hz/8=1000448hz
                             ; = .0000009995sec


;****************************************************************************
; - Set up TIM1 as a ~0.125us counter. Set modulo interrupt to generate the
;   ~100us(.1ms) period (10KHZ),for PWM current control of EPC solonoid
;   pulse width "on" point. Interrupt vector "TIM1OV_ISR:"
;   Set channel 0 port control pin for PWM current control of EPC Solonoid
;   pulse width "off" point on PTD4.
;   Set channel 1 normal output pin for IAC PWM on PTD5
;****************************************************************************

     mov     #$30,T1SC       ; Move %00110000 into Timer1
                             ; Status and Control Register
                             ;(Disable interupts, stop timer)
                             ;(Prescale and counter cleared))
                             ;(Prescale for bus frequency / 1)
     mov     #$03,T1MODH     ; Move decimal 3 into T1 modulo reg Hi
     mov     #$20,T1MODL     ; Move decimal 32 into T1 modulo reg Lo
                             ;( decimal 800 = ~100uS period)=(~10KHZ)
     mov     #$00,T1CH0H     ; Move decimal 0 into T1CH0 O/C register Hi
     mov     #$00,T1CH0L     ; Move decimal 0 into T1CH0 O/C register Lo
     mov     #$30,T1SC0      ; Move %00110000 into TIM1 CH0 Status and
                             ; Control Register (PTD4 port control, logic Lo)
                             ; (PTD5 normal pin)
     mov     #$40,T1SC       ; Move %01000000 into Timer1
                             ; Status and Control Register
                             ; Enable interupts, counter Active
                             ; Prescale for bus frequency / 1
                             ; 8,003584hz / 1 = 8,003584hz
                             ; = .0000001249sec


;****************************************************************************
; - Set up Serial Communications Interface Module
;****************************************************************************

     lda      #$30           ; Load accumulator with %110000
     sta      SCBR           ; Copy to SCI Baud Rate Register
                             ; 8003584/(64*13*1)=9619.7 baud
     bset     ensci,SCC1     ; Set enable SCI bit of SCI Control Register 1
                             ; (Enable SCI)
     bset     RE,SCC2        ; Set receiver enable bit of SCI Control Reg. 2
                             ; (Enable receiver)
     bset     SCRIE,SCC2     ; Set SCI receive interrupt enable bit of
                             ; SCI Control Register 2 (Enable Rcv. Interrupt)
     lda      SCS1           ; Load accumulator with SCI Status Register 1
                             ; (Clear SCI transmitter Empty Bit)
     clr      txcnt          ; Clear SCI transmitter count
                             ; (incremented)(characters transmitted)
     clr      txgoal         ; Clear SCI number of bytes to transmit
                             ; (characters to be transmitted)


;****************************************************************************
; - Set up IRQ Interrupt (tach input)
;****************************************************************************

     mov     #$04,INTSCR     ; Move %00000100 into IRQ Status and Control
                             ; Register (Enable IRQ (turn on interrupts))
                             ; (falling edge only)


;****************************************************************************
;  - Set up Keyboard Interrupt on PTA0, Vehicle Speed Sensor
;   (8000 pulse per mile)
;****************************************************************************


     mov     #$02,INTKBSCR       ; Move %00000010 into Keyboard Status
                                 ; and Control Register
                                 ;(Keyboard interrupts masked)
                                 ;(interrupts on falling edges only)
     mov     #$01,INTKBIER       ; Move %00000001 into Keyboard Interrupt
                                 ; Enable Register
                                 ;(Keyboard interrupts on PTA0
     bset    ACKK,INTKBSCR       ; Set the Keyboard Acknowledge bit of
                                 ; Keyboard Status and Control Register
                                 ;(clear any false interrupts)
     bclr    IMASKK,INTKBSCR     ; Clear the Interrupt Mask bit of
                                 ; Keyboard Status and Control Register
                                 ;(enable interrupts)

;****************************************************************************
; - Load the configurable constants (TO_Table, etc) from Flash to RAM
;****************************************************************************

     clrh                        ; Clear index register Hi byte
     clrx                        ; Clear index register Lo byte

load_ram:
     lda     ms_rf_start_f,x     ; Load accumulator with value in
                                 ; "ms_rf_start_f" table, offset in index
                                 ; register Lo byte
     sta     ms_rf_start,x       ; Copy to "ms_rf_start" table, offset in
                                 ; index register Lo byte
     aix     #1                  ; Add immediate value (1)to index register
                                 ; H:X<_(H:X)+(16<<M)
     cphx    #ms_rf_size         ; Compare index register with memory
                                 ; (H:X)-(M:M+$0001)
     bne     load_ram            ; If the Z bit of CCR is clear, branch to
                                 ; load_ram:

;****************************************************************************
; - Initialize the variables to 0 or to some acceptable starting value
;****************************************************************************

     clr     secH            ; Seconds counter, Hi byte
     clr     secL            ; Seconds counter, Lo byte
     clr     RPM             ; Engine RPM/20 (0 to 5100 rpm in byte var)
     clr     MPH             ; MPH*2 (0 to 128 MPH in byte variable)
     clr     trans           ; Transmission status bit field variable(1 of 2)
     clr     trans2          ; Transmission status bit field variable(2 of 2)
     clr     gearcnt         ; Current gear count(sequential gear selector)
     clr     MAP             ; Manifold Absolute Pressure 8 bit ADC reading
     clr     MLPS            ; Manual Lever Position Switch 8 bit ADC reading
     clr     TOT             ; Transmission Oil Temperature 8 bit ADC reading
     clr     Lprs            ; Line Pressure 8 bit ADC reading
     clr     BAT             ; Battery Voltage 8 bit ADC reading
     clr     IAC             ; Engine Idle Air Control Sensor 8 bit ADC rdng
     clr     Trim            ; Electronic Pressure Control trim 8 bit ADC
     clr     TPS             ; Throttle Position Sensor 8 bit ADC reading
     clr     KPA             ; Manifold Absolute Pressure in KPA
     clr     MLPSp           ; Manual Lever Position Switch position
     clr     TOTemp          ; Transmission Oil Temperature variable
                             ; (degrees F to fit range in byte variable)
     clr     Lpsi            ; Line pressure in PSI
     clr     Volts           ; Battery voltage to 0.1V resolution
     clr     IACpw           ; IAC pulse width variable(0-60, 100uS res)
     clr     TPSp            ; TPSscale * 100 / TPSspan = TPSp(TPS percent)
     clr     TOTempDif       ; Difference between "TOTemp" and 180(140F)
     clr     TOTempP         ; TOTemp percent calculation value (TOTemp/180)
     clr     TOTAdd          ; Trans Oil Temp correction Add/Subtract value
     clr     TrimDif         ; Difference between "Trim" and 128(mid point)
     clr     TrimP           ; Trim percent calculation value (TrimDif/128)
     clr     TrimAdd         ; EPC Trim correction Add/Subtract value
     clr     df              ; EPC Duty Factor from "TO" table, stall or shift
                             ; tables, or, absolute values "EPC_TCC", or
                             ; "EPC_decel" (scaled to 256)
     clr     df1             ; "df" after TOT cor, before Trim cor
     clr     dff             ; "df1" after Trim cor(Final EPC Duty Factor)
     clr     EPCpwH          ; EPC Pulse Width Hi byte
     clr     EPCpwL          ; EPC Pulse Width Lo byte
     clr     shift           ; Gear change status bit field variable
     clr     tconf           ; Tuning Configuration status bit field variable
     clr     inputs          ; Input status bit field variable(1 of 2)
     clr     TPSscale        ; TPS - CT_cnt = TPSscale
     clr     uSx100          ; 100 Microseconds counter
     clr     mS              ; Milliseconds counter
     clr     mSx5            ; 5 Milliseconds counter
     clr     mSx20           ; 20 Milliseconds counter
     clr     mSx100          ; 100 Milliseconds counter
     clr     adsel           ; ADC Selector Variable
     clr     mlpsp_cur       ; MLPS position current
     clr     gearcnt_prv     ; Previous gear count
     clr     gear_cur        ; Current gear
     clr     gear_com        ; Commanded gear
     clr     RPMcH           ; RPM period counter Hi byte (100uS resolution)
     clr     RPMcL           ; RPM period counter Lo byte (100uS resolution)
     clr     RPMpH           ; RPM period Hi byte (100 uSresolution)
     clr     RPMpL           ; RPM period Lo byte (100 uSresolution)
     clr     MPHcH           ; MPH period counter Hi byte (100uS res)
     clr     MPHcL           ; MPH period counter Lo byte (100uS res)
     clr     MPHpH           ; MPH period Hi byte (100uS resolution)
     clr     MPHpL           ; MPH period Lo byte (100uS resolution)
     clr     dfSel           ; Duty Factor Select variable bit field
     clr     txcnt           ; SCI transmitter count (incremented)
     clr     txgoal          ; SCI number of bytes to transmit
     clr     txmode          ; Transmit mode flag
     clr     rxoffset        ; Offset placeholder when receiving constants
                             ; vis. SCI
     clr     LoopCntr        ; Loop counter for main loop frequency check
     clr     IACcnt          ; IAC counter for IACpw (100uS resolution)
     clr     TIMcnt          ; 20mS timer counter
     clr     TPS_prv         ; TPS rdg previous(update 0.1S for TPS DOT)
     clr     ShftUpDB        ; Shift Up contacts de-bounce 1mS counter var
     clr     ShftDnDB        ; Shift Dn contacts de-bounce 1mS counter var
     clr     AIACcnt         ; Auto IAC duration counter value(100mS res)
     clr     Spare1          ; Blank place holder for 16 byte increments
     clr     Spare2          ; Blank place holder for 16 byte increments
     clr     Spare3          ; Blank place holder for 16 byte increments
     clr     Spare4          ; Blank place holder for 16 byte increments
     clr     Spare5          ; Blank place holder for 16 byte increments
     clr     Spare6          ; Blank place holder for 16 byte increments
     clr     Spare7          ; Blank place holder for 16 byte increments
     clr     Spare8          ; Blank place holder for 16 byte increments
     clr     Spare9          ; Blank place holder for 16 byte increments
     clr     Spare10         ; Blank place holder for 16 byte increments

;****************************************************************************
; - Load "tconf" direct page bit field variables with values according to
;   the "TuneConfig" variable in flash/ram
;****************************************************************************

     lda     TuneConfig     ; Load accumulator with value in Tuning
                            ; Configuration variable
     bit      #$01          ; Bit test with %00000001 to see if
                            ; TOT cor enable bit is set
     bne     SET_TT         ; If Z bit of CCR is clear, branch
                            ; to SET_TT: (trans temp correction)
     bclr    tt,tconf       ; Clear "tt" bit of "tconf" variable
     bra     CHK_TR         ; Branch to CHK_TR:

SET_TT:
     bset    tt,tconf       ; Set "tt" bit of "tconf" variable

CHK_TR:
     bit      #$02          ; Bit test with %00000010 to see if
                            ; Trim cor enable bit is set
     bne     SET_TR         ; If Z bit of CCR is clear, branch
                            ; to SET_TR: (manual trim correction)
     bclr    tr,tconf       ; Clear "tr" bit of "tconf" variable
     bra     CHK_CYL        ; Branch to CHK_CYL:

SET_TR:
     bset    tr,tconf       ; Set "tr" bit of "tconf" variable

CHK_CYL:
     bit     #$04           ; Bit test with %00000100 to see if
                            ; 8 cylinder bit is set
     bne     SET_CYL        ; If Z bit of CCR is clear, branch
                            ; to SET_CYL:
     clrh                   ; Clear index register Hi byte
     bclr    cyl,tconf      ; Clear "cyl" bit of "tconf" variable
     ldx     #$27           ; Load index register Lo byte with $27 (6 cyl)
     stx     RPMk           ; Copy value in index register Lo byte to RPMk
     ldx     #$10           ; Load index register Lo byte with $10
     stx     RPMk+1         ; Copy value in index register Lo byte to RPMk+1
     bra     Stb_ADC        ; Branch to Stb_ADC:

SET_CYL:
     bset    cyl,tconf      ; Set "cyl" bit of "tconf" variable
     clrh                   ; Clear index register Hi byte
     ldx     #$1D           ; Load index register Lo byte with $1D
     stx     RPMk           ; Copy value in index register Lo byte to RPMk
     ldx     #$4C           ; Load index register Lo byte with $4C
     stx     RPMk+1         ; Copy value in index register Lo byte to RPMk+1

;****************************************************************************
; - Fire up the ADC, and perform one conversion, Set up clock source for ADC
;   Do an initial conversion just to stabilize the ADC
;****************************************************************************

Stb_ADC:
     lda     #$70      ; Load accumulator with %01110000
     sta     ADCLK     ; Copy to ADC Clock Register
                       ;( bus clock/8 = ~1mhz )
     lda     #$07      ; Load accumulator with %00000111
                       ;(one conversion, no interrupt on channel AD7)
     sta     ADSCR     ; Copy to ADC Status and Control Register

ADCWait:
     brclr   coco,ADSCR,ADCWait   ; If "conversions complete flag" bit of
                                  ; ADC Status and Control Register is clear
                                  ; branch to ADCWait lable
                                  ;(keep looping while COnversion
                                  ; COmplete flag = 0)
     lda    ADR                   ; Load accumulator with value in ADC Result
                                  ; Variable (read value from ADc Result)
     sta    Trim                  ; Copy to EPC trim ADC Reading
     clr    adsel                 ; Clear ADC channel selector variable

;****************************************************************************
; - Read the state of the Shift Up and Shift Down contacts, and copy state
;   to "pin state last" flags.
;****************************************************************************

     brset   Shiftup,porta,SUFLG_HI     ; if "Shiftup"bit of Port A is
                                        ; set, branch to SUFLG_HI:
     bclr    SUhi,inputs                ; Clear "SUhi" bit of "inputs" var
     bra     READ_SD                    ; Branch to READ_SD:

SUFLG_HI:
     bset    SUhi,inputs                ; Set "SUhi" bit of "inputs" var

READ_SD:
     brset   Shiftdn,porta,SDFLG_HI     ; if "Shiftdn"bit of Port A is
                                        ; set, branch to SDFLG_HI:
     bclr    SDhi,inputs                ; Clear "SDhi" bit of "inputs" var
     bra     TURN_ON_INTS               ; Branch to TURN_ON_INTS:

SDFLG_HI:
     bset    SDhi,inputs                ; Set "SDhi" bit of "inputs" var


TURN_ON_INTS:
     cli                          ; Clear intrupt mask
                                  ;( Turn on all interrupts now )


;****************************************************************************
;****************************************************************************
;********************    M A I N  E V E N T  L O O P     ********************
;****************************************************************************
;****************************************************************************

;****************************************************************************
; - Toggle pin 3 on Port D each program loop so frequency can be checked
;   with a frequency meter or scope. (for program developement)
;****************************************************************************

LOOPER:
     com     LoopCntr         ; Ones compliment "LoopCntr"
                              ;(flip state of "LoopCntr")
     bne     SET_LOOPCHK      ; If the Z bit of CCR is clear, branch
                              ; to SET_LOOPCHK
     bclr    LoopFrq,PORTD    ; Clear bit 3 of Port D (Program Loop LED)
     bra     LOOPCHK_DONE     ; Branch to LOOPCHK_DONE:

SET_LOOPCHK:
     bset    LoopFrq,PORTD    ; Set bit 3 of Port D (Program Loop LED)

LOOPCHK_DONE:

;****************************************************************************
; - Check to see if it's time to turn the IAC PWM off.
;****************************************************************************

     brclr   iacon,inputs,IAC_CHK_DONE  ; If "iacon" bit of "inputs"
                            ; variable is clear, branch to IAC_CHK_DONE:
     lda     IACcnt         ; Load accumulator with value in IAC counter
     bne     IAC_CHK_DONE   ; If Z bit of CCR is clear, branch to
                            ; IAC_CHK_DONE:
     bset    iacpwm,portd   ; Set "iacpwm" bit of Port D (IAC PW "off")
     bclr    iacon,inputs   ; Clear "iacon" bit of "inputs" variable

IAC_CHK_DONE:

;****************************************************************************
; - Check Exhaust Pressure to see if we have PSI permissive for Exhaust
;   Brake application. Switch is NC to ground, opens @ 32PSI.
;****************************************************************************

     brclr   ExhPS,PORTA,PSI_BRK  ; If "ExhPS" bit of Port A is clear,
                                  ; branch to PSI_BRK: (switch is grounded,
                                  ; exhaust brake permitted:
     bra     NO_PSI_BRK           ; Jump to NO_PSI_BRK:

PSI_BRK:
     brset   PSIbrk,trans,PSI_CHK_DONE  ; If "PSIbrk" bit of "trans" variable
                              ; is set, branch to PSI_CHK_DONE:
                              ; (bit is already set, so skip over)
     bset    PSIbrk,trans     ; Set "PSIbrk" bit of "trans" variable
     bra     PSI_CHK_DONE     ; Branch to PSI_CHK_DONE:

NO_PSI_BRK:
     brclr   PSIbrk,trans,PSI_CHK_DONE  ; If "PSIbrk" bit of "trans" variable
                              ; is clear, branch to PSI_CHK_DONE:
                              ; (bit is already clear, so skip over)
     bclr    PSIbrk,trans     ; Clear "PSIbrk" bit of "trans" variable
                              ;(exhaust pressure too high, switch not
                              ; grounded, exhaust brake not permitted

PSI_CHK_DONE:

;****************************************************************************
; - Update the ADC readings and conversions, and check for ADC related
;   permissives. This is done only once per ADC conversion completion, in
;   the first pass through the main loop after the ADC_ISR Interrupt routine
;   has been completed.
;****************************************************************************

     brset   adcc,inputs,ADC_LOOKUPS  ; If "adcc" bit of "inputs" variable
                                      ; is set, branch to ADC_LOOKUPS:
     jmp     NO_ADC_PASS              ; Jump to NO_ADC_PASS:

ADC_LOOKUPS:
     clrh                    ; Clear index register Hi byte
     clrx                    ; Clear index register Lo byte

;KPA_CALC:
     lda     MAP             ; Load accumulator with value in Manifold
                             ; Absolute Pressure 8 bit ADC reading
     tax                     ; Copy to index register Lo byte
     lda     KPAfac_RH,x     ; Load accumulator with value in KPAfac_RH
                             ; table, offset in index register Lo byte
     sta     KPA             ; Copy to Manifold Absolute Pressure in KPA

;MLPSP_CALC:
     lda     MLPS            ; Load accumulator with value in Manual Lever
                             ; Position Switch 8 bit ADC reading
     tax                     ; Copy to index register Lo byte
     lda     MLPSposit,x     ; Load accumulator with value in MLPS position
                             ; table, offset in index register Lo byte
     sta     MLPSp           ; Copy to Manual Lever Position Switch position
                             ; variable

;TOTEMP_CALC:
     lda     TOT             ; Load accumulator with value in Transmission
                             ; Oil Temperature sensor 8 bit ADC reading
     tax                     ; Copy to index register Lo byte
     lda     TOTdegrees,x    ; Load accumulator with value in Transmission
                             ; Oil Temp table, offset in index register Lo
     sta     TOTemp          ; Copy to Transmission Oil Temp variable

;LPSI_CALC:
     lda     Lprs            ; Load accumulator with value in Line Pressure
                             ; table 8 bit ADC reading
     tax                     ; Copy to index register Lo byte
     lda     LinePress,x     ; Load accumulator with value in Line Pressure
                             ; table, offset in index register Lo byte
     sta     Lpsi            ; Copy to Line Pressure in PSI variable

;VOLTS_CALC:
     lda     BAT             ; Load accumulator with value in Battery
                             ; Voltage 8 bit ADC reading
     tax                     ; Copy to index register Lo byte
     lda     BatVolt,x       ; Load accumulator with value in "BatVolt"
                             ; table, offset in index register Lo byte
     sta     Volts           ; Copy to Battery Voltage to 0.1V resolution

;IACPW_CALC:
     lda     IAC             ; Load accumulator with value in Idle
                             ; Position Sensor 8 bit ADC reading
     tax                     ; Copy to index register Lo byte
     lda     IACcntrl,x      ; Load accumulator with value in IAC control
                             ; table, offset in index register Lo byte
     sta     IACpw           ; Copy to Idle AIr Control pulse width variable

;TPS_CALC:
     lda     TPS          ; Load accumulator with value in Throttle
                          ; Position Sensor 8 bit ADC reading
     cmp     CT_cnt       ; Compare value in "TPS" to value in "CT_cnt"
     blo     RAIL_LO      ; If A<M, branch to RAIL_LO:
     bra     CHK_RAIL_HI  ; Branch to CHK_RAIL_HI:

RAIL_LO:
     clr     TPSp         ; Clear "TPTp" variable(TPSp = 0)
     bra     TPSCalc_DONE ; Branch to TPSCalc_DONE:

CHK_RAIL_HI:
     cmp     WOT_cnt      ; Compare value in "TPS" to value in "WOT_cnt"
     bhs     RAIL_HI      ; If A>=M, branch to RAIL_HI:
     bra     CALC_TPSP    ; Branch to CALC_TPSP:

RAIL_HI:
     mov     #$64,TPSp    ; Move decimal 100 into "TPSp" variable
     bra     TPSCalc_DONE ; Branch to TPSCalc_DONE:

CALC_TPSP:
     clrh                 ; Clear index register Hi byte
     sub     CT_cnt       ; Subtract A<-(A)-(M)(TPS - CT_cnt
     sta     TPSscale     ; Copy result to "TPSscale" variable
     ldx     #$64         ; Load index register Lo byte with decimal 100
     mul                  ; Multiply (X:A)<-(X)*(A)
     pshx                 ; Push value in index register Lo byte to stack
     pulh                 ; Pull value from stack to index register Hi byte
     ldx     TPSspan      ; Load index register Lo byte with value
                          ; in "TPSspan"
     div                  ; Divide (A)<-(H:A)/(X);(H)rem
     jsr     DIVROUND     ; Jump to "DIVROUND" subroutine,(round result)
     sta     TPSp         ; Copy to Throttle Position Percent variable

TPSCalc_DONE:

;****************************************************************************
; - Check Throttle position to see if we have TPS permissive for Decel Fuel
;   Cut and Exhaust Brake application.
;****************************************************************************

TPS_CHECK:
     lda     TPSp               ; Load accumulator with value in Throttle
                                ; Position Percent variable
     cmp     CT_min             ; Compare it with value in Closed Throttle
                                ; minimum % position variable
     bhs     NO_TPS_BRK         ; If (A)>=(M), branch to NO_TPS_BRK:
     bset    ClsThrt,trans      ; Set "ClsThrt" bit of "trans" variable
     bra     TPS_CHECK_DONE     ; Branch to TPS_CHECK_DONE:

NO_TPS_BRK:
     bclr    ClsThrt,trans      ; Clear "ClsThrt" bit of "trans" variable

TPS_CHECK_DONE:
     bclr    adcc,inputs        ; Clear "adcc" bit of "inputs" variable

NO_ADC_PASS:


;****************************************************************************
;
; ---------------------------- Computation of RPM ---------------------------
;
; RPM = CONSTANT/PERIOD
; Where:
; RPM         = Engine RPM
; RPM_K = 16 bit constant using .1ms clock tick (10khz)
;               ((10,000tickpsec*60secpmin)/(number of cyl/(stroke/2)))
; RPM_P = 16 bit period count between IRQ pulsed lines in 0.1ms
;               resolution
;   RPM_K
;   ----- = RPM
;   RPM_P
;
; 6cyl 4stroke RPMK = ((10,000*60)/3) = 200,000
; 8cyl 4stroke RPMK = ((10,000*60)/4) = 150,000
;
; We use the 100uS clock tick to calculate RPM/20. This allows us to use an
; 8 bit variable with a range of 0 to 5,100 RPM
; Our formula is now:
;
; rpm = constant/period
; Where:
; rpm         = Engine RPM/20 (0 to 5,100 RPM to fit in 8 bit variable)
; RPMk:RPMk+1 = 16 bit constant using 100uS clock tick (10khz)
;               ((10,000tickpsec*60secpmin)/(number of cyl/(stroke/2)))/20
; RPMpH:RPMpL = 16 bit period count between IRQ pulsed lines in 100uS
;               resolution
;   rpmk:rpmk+1
;   ----------- = rpm
;   RPMpH:RPMpL
;
; 6cyl 4stroke rpmK = ((10,000*60)/3)/20 = 10,000 = $2710
; 8cyl 4stroke rpmK = ((10,000*60)/4)/20 = 7,500  = $1D4C
; 6cyl RPM resolution is ~05@~1000, ~20@~2000, ~76@~3000, and ~128@~5000
; 8cyl RPM resolution is ~06@~1000, ~27@~2000, ~61@~3000, and ~172@~5000
;****************************************************************************

;****************************************************************************
; - Calculate Engine RPM/20. This is done only once per IRQ interrupt in the
;   first pass through the main loop after the interrupt routine has been
;   completed
;****************************************************************************

RPM_COMP:
     brclr   tachrise,inputs,NO_TACH_PASS  ; If "tachrise" bit of "inputs"
                               ; variable is clear, branch to NO_TACH_PASS:
     ldhx    RPMph             ; Load index register with value in
                               ; RPM Period Hy byte variable
     beq     ENG_STOP          ; If Z bit of CCR is set, branch to ENG_STOP:
     pshh                      ; Push value in index register Hi byte
                               ; to stack
     pula                      ; Pull value in stack to accumulator(H to A)
     tsta                      ; Test accumulator for Z or N
     beq     FAST_RPM_CALC     ; If the Z bit of CCR is set,
                               ; branch to FAST_RPM_CALC:
     bra     SLOW_RPM_CALC     ; Branch to SLOW_RPM_CALC:

ENG_STOP:
     clr     RPM               ; Clear "RPM" variable
     bset    Estop,trans       ; Set "Estop" bit of "trans" variable
     jmp     RPM_CHECK         ; Jump to RPM_CHECK(engine is not running)

SLOW_RPM_CALC:
     clr     intacc1           ; Clear intacc1 variable
     clr     intacc1+1         ; Clear intacc1+1 variable
     sthx    intacc2           ; Copy value in index register to
                               ; intacc2 variable
     lda     RPMk              ; Load accumulator with value in "RPMk"
     sta     intacc1+2         ; Copy to "intacc1+2
     lda     RPMk+1            ; Load accumulator with value in "RPMk+1"
     sta     intacc1+3         ; Copy to "intacc1+3"
     jsr     udvd32            ; Jump to subroutine udvd32 (32x16 divide)
     lda     intacc1+3         ; Load accumulator with value in intacc1+3
                               ; variable (8-bit RPM result)
     ldx     intacc2+1         ; Load index register Lo byte with value in
                               ; intacc2+1 (8 bit remainder)
     jsr     DIVROUND          ; Jump to "DIVROUND" subroutine
                               ; (round result)
     bra     AV_RPM            ; Branch to AV_RPM:

FAST_RPM_CALC:
     lda     RPMk          ; Load accumulator with value in RPMk variable
     psha                  ; Push value in accumulator to stack
     pulh                  ; Pull value from stack to index register Hi byte
     lda     RPMk+1        ; Load accumulator with value in rpmk+1 variable
     div                   ; Divide (A = (H:A) / X)
     jsr     DIVROUND      ; Jump to "DIVROUND" subroutine,(round result)

;****************************************************************************
; - Average the new "RPM" with the last "RPM" to stabilize the readings
;****************************************************************************

AV_RPM:
     add     RPM           ; Add (A)<-(A)+(M)(last"RPM"+new"RPM")
     rora                  ; Rotate right accumulator(divide by 2)
     sta     RPM           ; Copy result to "RPM"
     bclr    Estop,trans   ; Clear "Estop" bit of "trans" variable



;****************************************************************************
; - Check Engine RPM to see if we have RPM permissive for Exhaust Brake
;   and or Torque Converter Clutch application.
;****************************************************************************

RPM_CHECK:
     lda     RPM                ; Load accumulator with value in RPM/20
                                ; variable
     cmp     TCC_min_RPM        ; Compare it with value in TCC minimum
                                ; RPM variable
     bls     NO_TCC             ; If (A)<=(M), branch to NO_TCC:
     bset    RPMtcc,trans       ; Set "RPMtcc" bit of "trans" variable
     bra     RPM_CALC_DONE      ; Branch to RPM_CALC_DONE:

NO_TCC:
     bclr    RPMtcc,trans       ; Clear "RPMtcc" bit of "trans" variable

RPM_CALC_DONE:
     bclr    tachrise,inputs    ; Clear "tachrise" bit of "inputs" variable

NO_TACH_PASS:


;****************************************************************************
; - Check the state of Port A to see if the Vehicle Speed Sensor input has
;   returned to Hi state so the interrupt can be re-enabled.
;****************************************************************************

     brclr   VehSpd,porta,KEYBD_RESET_DONE  ; If "VehSpd" bit of Port A is
                                 ; clear, branch to KEYBD_RESET_DONE:
     bclr    IMASKK,INTKBSCR     ; Clear the Interrupt Mask bit of
                                 ; Keyboard Status and Control Register
                                 ;(enable interrupts)

KEYBD_RESET_DONE:


;****************************************************************************
;
; --------------------- Computation of Vehicle Speed ------------------------
;
; MPH = CONSTANT/PERIOD
; Where:
; mph           = Vehicle speed in MPH/2 (0 to 128 MPH to fit in 8 bit var)
; MPH_kH:MPH_kL   = 16 bit constant using .1ms clock tick (10khz)
;                 (10,000tickpsec*60secpmin*60minphr)/8000pulsepmile)*2
;                 mphk = 9000 = $2328
; MPH_pH:MPH_pL = 16 bit period count between PTA3 pulsed lines in 0.1ms
;                  resolution
;   MPH_kH:MPH_kL
;   ------------ = mph
;   MPH_pH:MPH_pL
;
; Resolution is ~2@~100MPH, ~1@~65MPH,<1@<65MPH
;
;****************************************************************************

;****************************************************************************
; - Calculate Vehicle Speed in MPH*2. This is done only once per receipt of
;   a VSS pulse on bit 3 of Port A, in the first pass through the main loop
;   after the pulse has been received
;****************************************************************************

MPH_CALC:
     brclr   vssp,inputs,NO_MPH_PASS  ; If "vssp" bit of "inputs" variable
                                      ; is clear, branch to NO_MPH_PASS:
     ldhx    MPHph             ; Load index register with value in
                               ; MPH Period Hy byte variable
     beq     VEH_STOP          ; If Z bit of CCR is set, branch to VEH_STOP:
     pshh                      ; Push value in index register Hi byte
                               ; to stack
     pula                      ; Pull value in stack to accumulator(H to A)
     tsta                      ; Test accumulator for Z or N
     beq     FAST_MPH_CALC     ; If the Z bit of CCR is set,
                               ; branch to FAST_MPH_CALC:
     bra     SLOW_MPH_CALC     ; Branch to SLOW_MPH_CALC:

VEH_STOP:
     clr     MPH               ; Clear "MPH" variable
     bset    Vstop,trans       ; Set "Vstop" bit of "trans" variable
     jmp     MPH_CHK           ; Jump to MPH_CHK(vehicle is not moving)

SLOW_MPH_CALC:
     clr     intacc1           ; Clear intacc1 variable
     clr     intacc1+1         ; Clear intacc1+1 variable
     sta     intacc2           ; Copy to intacc2 variable
     lda     MPHpL             ; Load accumulator with value in
                               ; MPH period Lo byte
     sta     intacc2+1         ; Copy to intacc2+1 variable
     lda     #$23              ; Load accumulator with value in "MPH_kH"
     sta     intacc1+2         ; Copy to "intacc1+2"
     lda     #$28              ; Load accumulator with value in "MPH_kL"
     sta     intacc1+3         ; Copy to "intacc1+3"
     jsr     udvd32            ; Jump to subroutine udvd32 (32x16 divide)
     lda     intacc1+3         ; Load accumulator with value in intacc1+3
                               ; variable (8-bit MPH/20 result)
     ldx     intacc2+1         ; Load index retgister Lo byte with value in
                               ; intacc2+1 (8 bit remainder)
     jsr     DIVROUND          ; Jump to "DIVROUND" subroutine (round result)
     bra     AV_MPH            ; Branch to AV_MPH:

FAST_MPH_CALC:
     lda     MPHpL         ; Load accumulator with value in MPH period
                           ; Lo byte
     tax                   ; Transfer value in accumulator to index register
                           ; Lo byte
     lda     #$23          ; Load accumulator with value in "MPH_kH"
     psha                  ; Push value in accumulator to stack
     pulh                  ; Pull value from stack to index register Hi byte
     lda     #$28          ; Load accumulator with value in "MPH_kL"
     div                   ; Divide (A = (H:A) / X)
     jsr     DIVROUND      ; Jump to "DIVROUND" subroutine (round result)

;****************************************************************************
; - Average the new "MPH" with the last "MPH" to stabilize the readings
;****************************************************************************

AV_MPH:
     add     MPH           ; Add (A)<-(A)+(M)(last"MPH"+new"MPH")
     rora                  ; Rotate right accumulator(divide by 2)
     sta     MPH           ; Copy result to "MPH"
     bclr    Vstop,trans   ; Clear "Vstop" bit of "trans" variable

MPH_CALC_DONE:
     bclr    vssp,inputs       ; Clear "vssp" bit of "inputs" variable

NO_MPH_PASS:


;****************************************************************************
; - Check Vehicle speed to see if we should use "stall" or "moving"
;   EPC tables.
;****************************************************************************

MPH_CHK:
     lda     MPH                ; Load accumulator with value in MPH*2
                                ; variable
     cmp     MPH_stall          ; Compare it with value in MPH maximum
                                ; value for "stall" EPC
     bhs     NO_STALL           ; If (A)>=(M), branch to NO_STALL:
     bset    MPHstall,trans     ; Set "MPHstall" bit of "trans" variable
     bra     MPH_CHK_DONE       ; Branch to MPH_CHK_DONE:

NO_STALL:
     bclr    MPHstall,trans     ; Clear "MPHstall" bit of "trans" variable

MPH_CHK_DONE:

;****************************************************************************
; - Compare the current TPS voltage reading with "TPS_prv" to determine if
;   the throttle plates are being opened, and if so, at what rate. If they
;   are being opened, and at a rate equal to or greater than the value in
;   the "TPSrate" variable, set the "accel" flag of "trans3" variable and
;   move the current "TPS" value into "TPS_prv". Otherwise, clear the
;   "accel" flag and move the current "TPS" value into "TPS_prv".
;   This is done only once per 100kuS(0.1S) clock tick in the first pass
;   through the main loop after the 100mS section of the TIM2 CH0 Interrupt
;   service routine has been completed.
;****************************************************************************

CHK_TPS_DOT:
     brclr   clk100K,inputs,NO_DOT_PASS  ; If "clk100K" bit of "inputs"
                               ; variable is clear, branch to NO_DOT_PASS:
     lda     TPS               ; Load accumulator with value in "TPS"
     cmp     TPS_prv           ; Compare it with value in "TPS_prv"
     beq     NO_ACCEL          ; If (A)=(M), branch to NO_ACCEL:
     blo     NO_ACCEL          ; If (A)<(M), branch to NO_ACCEL:
     sub     TPS_prv           ; Subtract (A)<-(A)-(M)
     cmp     TPSrate           ; Compare result to value in "TPSrate"
     blo     NO_ACCEL          ; If (A)<(M), branch to NO_ACCEL:
     bset    accel,trans       ; Set "accel" bit of "trans" variable
     mov     TPS,TPS_prv       ; Move value in "TPS" to "TPS_prv"
     bra     DOT_PASS_DONE     ; Branch to DOT_PASS_DONE:

NO_ACCEL:
     bclr    accel,trans       ; Clear "accel" bit of "trans" variable
     mov     TPS,TPS_prv       ; Move value in "TPS" to "TPS_prv"

DOT_PASS_DONE:
     bclr    clk100k,inputs    ; Clear "clk100k" bit of "inputs" variable

NO_DOT_PASS:

;****************************************************************************
; - Determine the Manual Lever Switch position and branch accordingly
;****************************************************************************

     lda     mlpsp         ; Load accumulator with value in MLPS position
     cbeqa   #P,PARK       ; Compare with value #P, if equal,
                           ; branch to PARK:
     cbeqa   #R,REVERSE    ; Compare with value #R, if equal,
                           ; branch to REVERSE:
     cbeqa   #N,NEUTRAL    ; Compare with value #N, if equal,
                           ; branch to NEUTRAL:
     cbeqa   #D,DRIVE_A    ; Compare with value #D, if equal,
                           ; branch to DRIVE_A:
     cbeqa   #M2,MAN2_A    ; Compare with value #m2, if equal,
                           ; branch to MAN2_A:
     cbeqa   #M1,MAN1      ; Compare with value #M1, if equal,
                           ; branch to MAN1:

     jmp     LOOPER        ; Jump to LOOPER:
                           ;(Either the sensor has failed, or we have a
                           ; non valid reading while changing lever
                           ; position, in either case, keep looping
                           ; until we have a valid reading)


DRIVE_A:
     jmp     DRIVE          ; Jump to DRIVE: (long branch)

MAN2_A:
     jmp     MAN2           ; Jump to MAN2: (long branch)


;****************************************************************************
; ----------------- "Park", "Reverse", "Neutral" section --------------------
; NOTE! - The gear change variables are not updated in neutral so that
;         MS_TECA will "remember" what gear it was in, primarily for a
;         D->N->D shift.
;****************************************************************************

PARK:
     lda     mlpsp_cur         ; Load accumulator with value in "mlpsp_cur"
     cbeqa   #P,PRN_DONE       ; Compare with value #P, if equal, branch
                               ; to PRN_DONE:
     bra     PARK_REV          ; Branch to PARK_REV:

REVERSE:
     lda     mlpsp_cur         ; Load accumulator with value in "mlpsp_cur"
     cbeqa   #R,PRN_DONE       ; Compare with value #R, if equal, branch
                               ; to PRN_DONE:
     bra     PARK_REV          ; Branch to PARK_REV:

NEUTRAL:
     lda     mlpsp_cur         ; Load accumulator with value in "mlpsp_cur"
     cbeqa   #N,PRN_DONE       ; Compare with value #N, if equal, branch
                               ; PRN_DONE:
     bra     NEUT              ; Branch to NEUT:

PARK_REV:
     jsr     GEAR1_VARS        ; Jump to subroutine at GEAR1_VARS:
                               ;(mov #first to gearcnt, gearcnt_prv,
                               ; gear_cur, gear_com)
NEUT:
     bset    TCC,portc         ; Set "TCC" bit of Port C (TCC off)
     bset    ExhBrk,portc      ; Set "ExhBrk" bit of Port C (ExhBrk off)
     bclr    DFCper,portd      ; Clear "DFCper" bit of Port D (DFC prohibit)
     bclr    SS1,portc         ; Clear "SS1" bit of Port C,(SS1 on)
     bset    SS2,portc         ; Set "SS2" bit of Port C,(SS2 off)
     bset    CCS,portc         ; Set "CCS" bit of Port C,(CCS off)
     clr     trans2            ; Clear "trans2" variable (Clear "CCSon",
                               ; "TCCon", "DFCon", "Brkon", "SSprog",
                               ; "TCCprog", "D1D2")
     clr     shift             ; Clear "shift" variable (Clear "EPCrTCC",
                               ; "EPChTCC", "EPCrSS", "EPChSS", "SS1del",
                               ; "CCSdel", "SSsdel", "Brkdel"
     clr     TIMcnt            ; Clear 20mS timer counter
     clr     dfSel             ; Clear "dfSel" variable
     bset    selStl,dfSel      ; Set "selStl" bit of "dfSel"
                               ;(EPC set for "stall" condition)
     mov     mlpsp,mlpsp_cur   ; Move value in "mlpsp" into "mlpsp_cur"

PRN_DONE:
     jmp     DFSEL_DONE        ; Jump to DFSEL_DONE:


;****************************************************************************
; ------------------------ "Manual First" section ---------------------------
;****************************************************************************

MAN1:
     lda     mlpsp_cur         ; Load accumulator with value in "mlpsp_cur"
     cbeqa   #M1,MAN1_EPC      ; Compare with value #M1, if equal, branch
                               ; to MAN1_EPC:
     bset    TCC,portc         ; Set "TCC" bit of Port C (TCC off)
     bset    ExhBrk,portc      ; Set "ExhBrk" bit of Port C (ExhBrk off)
     bclr    DFCper,portd      ; Clear "DFCper" bit of Port D (DFC prohibit)
     bclr    SS1,portc         ; Clear "SS1" bit of Port C,(SS1 on)
     bset    SS2,portc         ; Set "SS2" bit of Port C,(SS2 off)
     bclr    CCS,portc         ; Clear "CCS" bit of Port C,(CCS on)
     clr     trans2            ; Clear "trans2" variable (Clear "CCSon",
                               ; "TCCon", "DFCon", "Brkon", "SSprog",
                               ; "TCCprog", "D1D2")
     clr     shift             ; Clear "shift" variable (Clear "EPCrTCC",
                               ; "EPChTCC", "EPCrSS", "EPChSS", "SS1del",
                               ; "CCSdel", "SSsdel", "Brkdel"
     bset    CCSon,trans2      ; Set "CCSon" bit of "trans2" variable
     clr     TIMcnt            ; Clear 20mS timer counter
     jsr     GEAR1_VARS        ; Jump to subroutine at GEAR1_VARS:
                               ;(mov #first to gearcnt, gearcnt_prv,
                               ; gear_cur, gear_com)
     mov     mlpsp,mlpsp_cur   ; Move value in "mlpsp" into "mlpsp_cur"

MAN1_EPC:

;****************************************************************************
; - Check to see if we are above stall speed and at closed throttle, if so,
;   set EPC pulse width for Decel conditions, otherwise, set EPC according
;   to the "EPC_M1" table.
;****************************************************************************

     brclr  ClsThrt,trans,NO_DECEL_M1   ; If "ClsThrt" bit of "trans"
                                        ; variable is clear, branch to
                                        ; NO_DECEL_M1:
     brset  MPHstall,trans,NO_DECEL_M1  ; If "MPHstall" bit of "trans"
                                        ; variable is set, branch to
                                        ; NO_DECEL_M1:
     bra     DECEL_M1                   ; Branch to DECEL_M1:

NO_DECEL_M1
     brset   selM1,dfSel,MAN1_DONE     ; If "selM1" bit of "dfSel"
                                        ; variable is set, branch to
                                        ; MAN1_DONE:
     clr     dfSel                      ; Clear "dfSel" variable
     bset    selM1,dfSel               ; Set "selM1" bit of "dfSel"
     bra     MAN1_DONE                  ; Branch to MAN1_DONE:

DECEL_M1:
     brset   selDcl,dfSel,MAN1_DONE     ; If "selDcl" bit of "dfSel"
                                        ; variable is set, branch to
                                        ; MAN1_DONE:
     clr     dfSel                      ; Clear "dfSel" variable
     bset    selDcl,dfSel               ; Set "selDcl" bit of "dfSel"

MAN1_DONE:
     jmp     DFSEL_DONE                 ; Jump to DFSEL_DONE:


;****************************************************************************
; ------------------------ "Manual Second" section --------------------------
;****************************************************************************

MAN2:
     lda     mlpsp_cur          ; Load accumulator with value in "mlpsp_cur"
     cbeqa   #M2,DO_EPC_M2      ; Compare with value #M2, if equal, branch
                                ; to DO_EPC_M2:
     lda     gear_cur           ; Load accumulator with value in "gear_cur"
     cbeqa   #first,FIRST_M2    ; Compare with value #first, if equal,
                                ; branch to FIRST_M2:

;****************************************************************************
; - "mlps_cur" was something other than Man2, and "gear_cur" was something
;   other than first gear.
;   This means that the manual lever has commanded a downshift from D4 or
;   D3 to Man2, or a solenoid state change from D2 to Man2.
;****************************************************************************

     jsr     M2_SOLS           ; Jump to subroutine at M2_SOLS:
                               ;(SS1 off, SS2 off, CCS on, "CCSon" set,
                               ; "D1D2" clr)
     jsr     GEAR2_VARS        ; Jump to subroutine at GEAR2_VARS:
                               ;(mov #second to gearcnt, gearcnt_prv,
                               ; gear_cur, gear_com)
     clr     dfSel             ; Clear "dfSel" variable
     bset    selTO,dfSel       ; Set "selTO" bit of "dfSel"
                               ;(EPC set for torque output)
     mov     mlpsp,mlpsp_cur   ; Move value in "mlpsp" into "mlpsp_cur"
     bra     DO_EPC_M2         ; Branch to DO_EPC_M2:


;****************************************************************************
; - "mlps_cur" was something other than Man2, and the last solonoid
;     combination was for first gear. That means we are either upshifting
;     from D1 to M2, or M1 to M2.
;   Upshift sequence is as follows:
;   - Set the "EPCrSS" bit of "shift" variable, start the pressure rise
;     count down timer, and set EPC according to the "EPC_12" table
;   - When the pressure rise counter zeros, clear the "EPCrSS" bit and set
;     the "EPChSS" bit of "shift" variable. Change solonoid state and start
;     the pressure hold timer.
;   - When the pressure hold timer zeros, clear the "EPChSS" bit of "shift"
;     variable and set EPC according to the "EPC_move" table in the next
;     section.
;****************************************************************************

FIRST_M2:
     brset   EPCrSS,shift,FIRST_M2A    ; If "EPCrSS" bit of "shift" variable
                                       ; is set, branch to FIRST_M2A:
     bset    EPCrSS,shift              ; Set "EPCrSS" bit of "shift" variable
     lda     EPC_rise                  ; Load accumulator with value in
                                       ; "EPC_rise"
     sta     TIMcnt                    ; Copy to "TIMcnt"
     bset    SSprog,trans2             ; Set "SSprog" bit of "trans2" var
     clr     dfSel                     ; Clear "dfSel" variable
     bset    sel12,dfSel               ; Set "sel12" bit of "dfSel"
     bra     DO_EPC_M2                 ; Branch to DO_EPC_M2:

FIRST_M2A:
     lda     TIMcnt              ; Load accumulator with value in "TIMcnt"
                                 ; variable
     bne     DO_EPC_M2           ; If Z bit of CCR is clear, branch to
                                 ; DO_EPC_M2:
     bclr    EPCrSS,shift        ; Clear "EPCrSS" bit of "shift" variable
     bset    EPChSS,shift        ; Set "EPChSS" bit of "shift" variable
     jsr     M2_SOLS             ; Jump to subroutine at M2_SOLS:
                                 ;(SS1 off, SS2 off, CCS on, "CCSon" set,
                                 ; "D1D2" clr)
     jsr     GEAR2_VARS          ; Jump to subroutine at GEAR2_VARS:
                                 ;(mov #second to gearcnt, gearcnt_prv,
                                 ; gear_cur, gear_com)
     lda     EPC_hold            ; Load accumulator with value in "EPC_hold"
     sta     TIMcnt              ; Copy to "TIMcnt"
     mov     mlpsp,mlpsp_cur     ; Move value in "mlpsp" into "mlpsp_cur"

DO_EPC_M2:
     jmp     DO_DFSEL            ; Jump to DO_DFSEL:(long branch)


;****************************************************************************
; --------------------------- "Drive" section -------------------------------
;****************************************************************************

DRIVE:
     lda     mlpsp_cur         ; Load accumulator with value in "mlpsp_cur"
     cbeqa   #D,SEQ_SEL_CHECK  ; Compare with value #D, if equal, branch
                               ; to SEQ_SEL_CHECK:
     cbeqa   #M2,M2_D2_A       ; Compare with value #M1, if equal, branch
                               ; to M2_D2_A:
     cbeqa   #N,NEUTRAL_D_A    ; Compare with value #N, if equal, branch
                               ; to NEUTRAL_D_A:

M2_D2_A:
     jmp     M2_D2             ; Jump to M2_D2:(long branch)

NEUTRAL_D_A:
     jmp     NEUTRAL_D         ; Jump to NEUTRAL_D:(long branch)


SEQ_SEL_CHECK:

;****************************************************************************
; -  "mlps_prv" was "Drive", so no gear changes have been commanded by the
;    MLPS. Check sequential gear selector to see if a gear change has been
;    commanded by it.
;
;    Because the shift up/shift down contacts control the sequence logic, it
;    is necesary to provide the de-bounce feature. This feature provides a
;    time delay between polls to eliminate the effects of point bounce and
;    make it necesary for the contacts to open again before the next down
;    state is recognized a legitimate command. TCC and exhaust brake
;    contacts are not affected in this way as multiple contact make/break
;    events will have no adverse effect.
;
;    De-bounce works as follows:
;    All contacts use pull ups so normal state is Hi, active is Lo. At start
;    up, the program polls the pins and sets the last pass flag. In the loop
;    the pin is polled to check it's state. If it's Hi, and de-bounce is
;    not in progress, it continues. If it's Lo, it starts the de-bounce timer
;    (125mS initial, increased to 150ms, then flash configurable),
;    clears the flag and proceed to carry out the shift command.
;    The pin is not polled again until the timer times out. If it is still
;    Lo when next polled, and the flag is still clear, it continues on. If it
;    is Hi, the timer is set, and the flag is set. The pin is not polled
;    until the timer times out, and the proces repeats.
;****************************************************************************

     lda     ShftUpDB                        ; Load accumulator with value
                                             ; in Shift Up de-bounce counter
     bne     SHFTUP_CHK_DONE                 ; If Z bit of CCR is clear,
                                             ; branch to SHFTUP_CHK_DONE:
                                             ;(de-bounce in progress,
                                             ; skip over)
     brset   Shiftup,porta,SHFTUP_HI         ; If "Shiftup" bit of Porta A
                                             ; is set, branch toSHFTUP_HI:
                                             ;(is pin Hi?)
     brclr   SUhi,inputs,SHFTUP_CHK_DONE     ; If "SUhi" bit of "inputs"
                                             ; variable is clear, branch to
                                             ; SHFTUP_CHK_DONE:
                                             ;(pin is Lo, if pin state last
                                             ; pass is Lo, skip over)

;*****************************************************************************
; - Pin state change from Hi to Lo. Start de-bounce timer, clear pin state
;   last pass flag, and proceed to increment gear counter section.
;*****************************************************************************
     lda     DBup                            ; Load accumulator with value in
                                             ; "DBup" variable
     sta     ShftUpDB                        ; Store in "ShftUpDB" de-bounce
                                             ; counter
     bclr    SUhi,inputs                     ; Clear "SUhi" bit of "inputs"
     bra     UPSHIFT_COM                     ; Branch to UPSHIFT_COM:

SHFTUP_HI:
     brset   SUhi,inputs,SHFTUP_CHK_DONE     ; If "SUhi" bit of "inputs"
                                             ; variable is set, branch to
                                             ; SHFTUP_CHK_DONE:
                                             ;(pin is Hi, if pin state last
                                             ; pass is Hi, skip over)

;*****************************************************************************
; - Pin state change from Lo to Hi. Start de-bounce timer and set pin state
;   last pass flag.
;*****************************************************************************

     lda     DBup                            ; Load accumulator with value in
                                             ; "DBup" variable
     sta     ShftUpDB                        ; Store in "ShftUpDB" de-bounce
                                             ; counter
     bset    SUhi,inputs                     ; Set "SUhi" bit of "inputs" var

SHFTUP_CHK_DONE:
     lda     ShftDnDB                        ; Load accumulator with value in
                                             ; Shift Dn de-bounce counter
     bne     SHFTDN_CHK_DONE                 ; If Z bit of CCR is clear,
                                             ;  branch to SHFTDN_CHK_DONE:
                                             ;(de-bounce in progress,
                                             ; skip over)
     brset   Shiftdn,porta,SHFTDN_HI         ; If "Shiftdn" bit of Porta A is
                                             ; set, branch to SHFTDN_HI:
                                             ;(is pin Hi?)
     brclr   SDhi,inputs,SHFTDN_CHK_DONE     ; If "SDhi" bit of "inputs"
                                             ; variable is clear, branch to
                                             ; SHFTDN_CHK_DONE:
                                             ;(pin is Lo, if pin state last
                                             ; pass is Lo, skip over)

;*****************************************************************************
; - Pin state change from Hi to Lo. Start de-bounce timer, clear pin state
;   last pass flag, and proceed to decrement gear counter section.
;*****************************************************************************

     lda     DBdn                            ; Load accumulator with value in
                                             ; "DBdn" variable
     sta     ShftDnDB                        ; Store in "ShftDnDB" de-bounce
                                             ; counter
     bclr    SDhi,inputs                     ; Clear "SDhi" bit of "inputs2"
     bra     DNSHIFT_COM                     ; Branch to DNSHIFT_COM:

SHFTDN_HI:
     brset   SDhi,inputs,SHFTDN_CHK_DONE     ; If "SDhi" bit of "inputs"
                                             ; variable is set, branch to
                                             ; SHFTDN_CHK_DONE:
                                             ;(pin is Hi, if pin state last
                                             ; pass is Hi, skip over)

;*****************************************************************************
; - Pin state change from Lo to Hi. Start de-bounce timer and set pin state
;   last pass flag.
;*****************************************************************************

     lda     DBdn                            ; Load accumulator with value in
                                             ; "DBdn" variable
     sta     ShftDnDB                        ; Store in "ShftDnDB" de-bounce
                                             ; counter
     bset    SDhi,inputs                     ; Set "SDhi" bit of "inputs" var

SHFTDN_CHK_DONE:

;*****************************************************************************
; - No gear change has been commanded by the joy stick contacts. Check to see
;   if a shift is in progress, and branch accordingly.
;*****************************************************************************

     brclr   SSprog,trans2,NOSHIFT_D_A     ; If "SSprog" bit of "trans2"
                                           ; variable is clear, branch to
                                           ; NOSHIFT_D_A:
     bra     SEQ_SEL_CHECK_DONE            ; Branch to SEQ_SEL_CHECK_DONE



;*****************************************************************************
; - A shift has been commanded from Joy stick contacts. Use the flag states
;   to detirmine which shift it should be.
;*****************************************************************************

UPSHIFT_COM:
     lda     gearcnt_prv            ; Load accumulator with value in gear
                                    ; count previous
     cmp     #forth                 ; Compare it with value #forth
     beq     NOSHIFT_D_A            ; If Z bit of CCR is set, branch to
                                    ; NOSHIFT_D_A:
     inc     gearcnt                ; Increment Gear Count variable
     bra     GEARCNT_FLAGS          ; Branch to GEARCNT_FLAGS:

DNSHIFT_COM:
     lda     gearcnt_prv            ; Load accumulator with value in gear
                                    ; count previous
     cmp     #first                 ; Compare it with value #first
     beq     NOSHIFT_D_A            ; If Z bit of CCR is set, branch to
                                    ; NOSHIFT_D_A:
     dec     gearcnt                ; Decrement Gear Count variable

GEARCNT_FLAGS:
     mov     gearcnt,gearcnt_prv    ; Move value in "gearcnt" to
                                    ; "gearcnt_prv"
     mov     gearcnt,gear_com       ; Move value in "gearcnt" to "gear_com"

SEQ_SEL_CHECK_DONE:
     lda     gear_com     ; Load accumulator with value in commanded gear
     cmp     gear_cur     ; Compare it with value in Current Gear
     bhi     UPSHIFT      ; If (A)>(M), branch to UPSHIFT:
     blo     DNSHIFT      ; If (A)<(M), branch to DNSHIFT:
     bra     NOSHIFT_D_A  ; Branch to NOSHIFT_D_A:


;****************************************************************************
; - A sequential down shift has been commanded, or we have a shift in
;   progress.  The only time that the "SSprog" flag is set for a down shift
;   is the 43, which delays application of the coast clutch in third.
;   No harm will be done if a quick downshift from 4th through 3d to 2nd or
;   even 1st is commanded, so any shift in progress can be cancelled first,
;   then check the commanded gear status to determine which shift event it
;   should be.
;****************************************************************************

DNSHIFT:
     lda     gear_com            ; Load accumulator with value in "gear_com"
     cbeqa   #third,SHIFT43_A    ; Compare with value #third, if equal,
                                 ; branch to SHIFT43_A:
     cbeqa   #second,SHIFT32_A   ; Compare with value #second, if equal,
                                 ; branch to SHIFT32_A:
     cbeqa   #first,SHIFT21_A    ; Compare with value #first, if equal,
                                 ; branch to SHIFT21_A:
     bra     NOSHIFT_D_A         ; Branch to NOSHIFT_D_A:

SHIFT43_A:
     jmp     SHIFT43           ; Jump to SHIFT43:(long branch)

SHIFT32_A:
     jmp     SHIFT32           ; Jump to SHIFT32:(long branch)

SHIFT21_A:
     jmp     SHIFT21           ; Jump to SHIFT21:(long branch)


;****************************************************************************
; - A sequential up shift has been commanded.  Check the commanded gear
;   status to determine which shift event it should be.
;****************************************************************************

UPSHIFT:
     lda     gear_com            ; Load accumulator with value in "gear_com"
     cbeqa   #forth,SHIFT34_A    ; Compare with value #forth, if equal,
                                 ; branch to SHIFT34_A:
     cbeqa   #third,SHIFT23_A    ; Compare with value #third, if equal,
                                 ; branch to SHIFT23_A:
     cbeqa   #second,SHIFT12_A   ; Compare with value #second, if equal,
                                 ; branch to SHIFT12_A:

NOSHIFT_D_A:
     jmp     DO_DFSEL           ; Jump to DO_DFSEL:(long branch)

SHIFT34_A:
     jmp     SHIFT34            ; Jump to SHIFT34:(long branch)

SHIFT23_A:
     jmp     SHIFT23            ; Jump to SHIFT23:(long branch)

SHIFT12_A:
     jmp     SHIFT12            ; Jump to SHIFT12:(long branch)

M2_D2:

;****************************************************************************
; -  "mlps_cur" was "Man 2", so MLPS has commanded a solonoid state change
;     from Man 2 to Drive 2. Because SS2 takes longer to apply than SS1, we
;     have to delay the application of SS1 for a short time. There is no
;     provision for pressure rise as we are not changeing gear ratios.
;     Solonoid state change sequence is as follows:
;   - Set the "SSprog" bit of "trans2" variable. Set the "SS1del" bit of
;     "shift" variable. Energise SS2, CCS, Set "CCSon" bit of "trans2"
;     variable and start the SS1 delay countdown timer with the SS1 delay
;     value.
;   - When the SS1 delay counter zeros, Energise SS1 and update the Gear 2
;     variable, clear "SS1del" and "SSprog" bits. Set EPC according to the
;     "TO" table and move "Mlpsp" into "mlpsp_cur"
;****************************************************************************

     brset   SS1del,shift,M2_D2A  ; If "SS1del" bit of "shift" variable is
                                  ; set, branch to M2_D2A:
     bset    SSprog,trans2      ; Set "SSprog" bit of "trans2" var
     bset    SS1del,shift       ; Set "SS1del" bit of "shift" variable
     bclr    SS2,portc          ; Clear "SS2" bit of Port C,(SS2 on)
     bclr    CCS,portc          ; Cleer "CCS" bit of Port C,(CCS on)
     bset    CCSon,trans2       ; Set "CCSon" bit of "trans2" variable
     lda     SS1_del            ; Load accumulator with value in "SS1_del"
     sta     TIMcnt             ; Copy to "TIMcnt"
     bra     M2_D2_DONE         ; Branch to M2_D2_DONE:

M2_D2A:
     lda     TIMcnt             ; Load accumulator with value in "TIMcnt"
                                ; variable
     bne     M2_D2_DONE         ; If Z bit of CCR is clear, branch to
                                ; M2_D2_DONE:
     bclr    SS1,portc          ; Clear "SS1" bit of Port C(SS1 on)
     jsr     GEAR2_VARS         ; Jump to subroutine at GEAR2_VARS:
                                ;(mov #second to gearcnt, gearcnt_prv,
                                ; gear_cur, gear_com)
     bclr    SS1del,shift       ; Clear "SS1del" bit of "shift" variable
     bclr    SSprog,trans2      ; Clear "SSprog" bit of "trans2" var
     clr     dfSel              ; Clear "dfSel" variable
     bset    selTO,dfSel        ; Set "selTO" bit of "dfSel"
     mov     mlpsp,mlpsp_cur    ; Move value in "mlpsp" into "mlpsp_cur"

M2_D2_DONE:
     jmp     DO_DFSEL           ; Jump to DO_DFSEL:(long branch)


NEUTRAL_D:

;****************************************************************************
; -  "mlps_cur" was "Neutral" from "Reverse"/"Park", or "Drive". If the
;    Lever is moved from Drive to Neutral, and back to Drive, the TCU will
;    "remember " what gear it was in. This is primaraly used to reduce the
;    rotating mass of the transmission while shifting the auxilliary
;    transmission under way. There is no provision for pressure rise as this
;    should be done at low power settings.
;****************************************************************************

     bclr    SSprog,trans2     ; Clear "SSprog" bit of "trans2" variable
     bclr    TCprog,trans2     ; Clear "TCprog" bit of "trans2" variable
     clr     shift             ; Clear "shift" variable (Clear "EPCrTCC",
                               ; "EPChTCC", "EPCrSS", "EPChSS", "SS1del",
                               ; "CCSdel", "SSsdel"
     clr     TIMcnt            ; Clear 20mS timer counter
     jsr     TCC_BRK_OFF       ; Jump to subroutine at TCC_BRK_OFF:
                               ;(TCC off, DFCper off, ExhBrk off,
                               ; "EPCrTCC" clr, EPChTCC clr,
                               ; "TCCon" clr, "TCprog" clr, "DFCon" clr,
                               ; "Brkon" clr, "Brkdel" clr)
     lda     gear_cur          ; Load accumulator with value in "gear_cur"
     cbeqa   #forth,FOUR       ; Compare with value #forth, if equal,
                               ; branch to FOUR:
     cbeqa   #third,THREE      ; Compare with value #third, if equal,
                               ; branch to THREE:
     cbeqa   #second,TWO       ; Compare with value #second, if equal,
                               ; branch to TWO:
     cbeqa   #first,ONE        ; Compare with value #first, if equal,
                               ; branch to ONE:

FOUR:
     jsr     D4_SOLS             ; Jump to subroutine at D4_SOLS
                                 ;(SS1 off, SS2 off, CCS off,"CCSon" clr,
                                 ; "D1D2" clr)
     jsr     GEAR4_VARS          ; Jump to subroutine at GEAR4_VARS:
                                 ;(mov #forth to gearcnt, gearcnt_prv,
                                 ; gear_cur, gear_com)
     bra     NEUTRAL_D_DONE      ; Branch to NEUTRAL_D_DONE:

THREE:
     jsr     D3_SOLS             ; Jump to subroutine at D3_SOLS
                                 ;(SS1 off, SS2 on, CCS on, "CCSon" set,
                                 ; "D1D2" clr)
     jsr     GEAR3_VARS          ; Jump to subroutine at GEAR3_VARS:
                                 ;(mov #third to gearcnt, gearcnt_prv,
                                 ; gear_cur, gear_com)
     bra     NEUTRAL_D_DONE      ; Branch to NEUTRAL_D_DONE:

TWO:
     jsr     D2_SOLS             ; Jump to subroutine at D2_SOLS
                                 ;(SS1 on, SS2 on, CCS on, "CCSon" set)
     jsr     GEAR2_VARS          ; Jump to subroutine at GEAR2_VARS:
                                 ;(mov #second to gearcnt, gearcnt_prv,
                                 ; gear_cur, gear_com)
     bra     NEUTRAL_D_DONE      ; Branch to NEUTRAL_D_DONE:

ONE:
     jsr     M1D1_SOLS           ; Jump to subroutine at M1D1_SOLS
                                 ;(SS1 on, SS2 off, CCS on, "CCSon" set)
     bset    D1D2,trans2         ; Set"D1D2" bit of "trans2" variable
     jsr     GEAR1_VARS          ; Jump to subroutine at GEAR1_VARS:
                                 ;(mov #first to gearcnt, gearcnt_prv,
                                 ; gear_cur, gear_com)

NEUTRAL_D_DONE:
     clr     dfSel               ; Clear "dfSel" variable
     bset    selTO,dfSel         ; Set "selTO" bit of "dfSel"
     mov     mlpsp,mlpsp_cur     ; Move value in "mlpsp" into "mlpsp_cur"
     jmp     DO_DFSEL            ; Jump to DO_DFSEL:(long branch)


;SEQ_DN:

;****************************************************************************
; ---------------------- Sequential Downshift Section ----------------------
; - There is no provision for shift event pressure rise during downshifts
;   as experience has shown it not appear to be necessary. 3-2, and 2-1
;   downshifts just invlove simple state and flag changes. 4-3 downshift is
;   similar except provision must be made to ensure the Coast Clutch does
;   not engage until the shift solonoid state is completed.
;****************************************************************************

;****************************************************************************
; - Drive 2 to Drive 1 shift
;****************************************************************************

SHIFT21:
     bclr    SS1,portc            ; Clear "SS1" bit of Port C,(SS1 on)
     bset    SS2,portc            ; Set "SS2" bit of Port C,(SS2 off)
     bset    D1D2,trans2          ; Set"D1D2" bit of "trans2" variable
     bra     SEQ_DN_DONE          ; Branch to SEQ_DN_DONE:

;****************************************************************************
; - Drive 3 to Drive 2 shift
;****************************************************************************

SHIFT32:
     bclr    SS1,portc            ; Clear "SS1" bit of Port C,(SS1 on)
     bclr    SS2,portc            ; Clear "SS2" bit of Port C,(SS2 on)
     bset    D1D2,trans2          ; Set"D1D2" bit of "trans2" variable
     bra     SEQ_DN_DONE          ; Branch to SEQ_DN_DONE:

;****************************************************************************
; - Drive 4 to Drive 3 shift
;****************************************************************************

SHIFT43:
     bset    SS1,portc            ; Set "SS1" bit of Port C,(SS1 off)
     bclr    SS2,portc            ; Clear "SS2" bit of Port C,(SS2 on)
     bclr    D1D2,trans2          ; Clear "D1D2" bit of "trans2" variable
     lda     CCS_del              ; Load accumulator with value in "CCS_del"
     sta     TIMcnt               ; Copy to "TIMcnt" variable
     bset    CCSdel,shift         ; Set "CCsdel" bit of "shift" variable

SEQ_DN_DONE:
     mov     gear_com,gear_cur    ; Move value in "gear_com" to "gear_cur"
     clr     dfSel                ; Clear "dfSel" variable
     bset    selTO,dfSel          ; Set "selTO" bit of "dfSel"
     jmp     DO_DFSEL             ; Jump to DO_DFSEL:(long branch)


;****************************************************************************
; ------------------------ Sequential Upshift Section -----------------------
;  - The upshift events, 1-2,2-3,and 3-4, each have their own EPC tables.
;   Upshift sequence for 1-2, and 2-3 is as follows:
;   - Set the "EPCrSS" bit of "shift" variable, start the pressure rise
;     count down timer, and set EPC according to the "EPC_xx" table.
;   - When the pressure rise counter zeros, clear the "EPCrSS" bit and set
;     the "EPChSS" bit of "shift" variable. Change solonoid state and start
;     the pressure hold timer. Set EPC according to the "EPC_xx" table.
;   - When the pressure hold timer zeros, clear the "EPChSS" bit of "shift"
;     variable and set EPC according to the "EPC_move" table.
;   Upshift sequence for 3-4 is similar, except that provision must be made
;     to ensure the Coast Clutch is completely disengaged before the 3-4
;     solonoid state change takes place.
;   Upshift sequence for 3-4 is as follows:
;    - De-energise the Coast Clutch. Set the "CCSdel" bit of "shift"
;     variable and start the CCS delay timer. When the CCS delay timer
;     zeros, clear the "CCSdel" bit of "shift" variable, set the "EPCrSS"
;     bit of "shift" variable and proceed in the same order as the 1-2,
;     and 2-3 shifts.
;****************************************************************************

;****************************************************************************
; - Drive 1 to Drive 2 shift
;****************************************************************************

SHIFT12:
     brset   EPCrSS,shift,SHIFT12A  ; If "EPCrSS" bit of "shift" variable
                                    ; is set, branch to SHIFT12A:
     bset    EPCrSS,shift       ; Set "EPCrSS" bit of "shift" variable
     bset    SSprog,trans2      ; Set "SSprog" bit of "trans2" variable
     lda     EPC_rise           ; Load accumulator with value in "EPC_rise"
     sta     TIMcnt             ; Copy to "TIMcnt"
     clr     dfSel              ; Clear "dfSel" variable
     bset    sel12,dfSel        ; Set "sel12" bit of "dfSel"
     bra     SHIFT12_DONE       ; Branch to SHIFT12__DONE:

SHIFT12A:
     lda     TIMcnt             ; Load accumulator with value in "TIMcnt"
                                ; variable
     bne     SHIFT12_DONE       ; If Z bit of CCR is clear, branch to
                                ; SHIFT12_DONE:
     bclr    EPCrSS,shift       ; Clear "EPCrSS" bit of "shift" variable
     bset    EPChSS,shift       ; Set "EPChSS" bit of "shift" variable
     jsr     D2_SOLS            ; Jump to subroutine at D2_SOLS
                                ;(SS1 on, SS2 on, CCS on, "CCSon" set)
     bset    D1D2,trans2        ; Set"D1D2" bit of "trans2" variable
     lda     EPC_hold           ; Load accumulator with value in "EPC_hold"
     sta     TIMcnt             ; Copy to "TIMcnt"
     mov     gear_com,gear_cur  ; Move value in "gear_com" to "gear_cur"

SHIFT12_DONE:
     jmp     DO_DFSEL           ; Jump to DO_DFSEL:(long branch)


;****************************************************************************
; - Drive 2 to Drive 3 shift
;****************************************************************************

SHIFT23:
     brset   EPCrSS,shift,SHIFT23A  ; If "EPCrSS" bit of "shift" variable
                                    ; is set, branch to SHIFT23A:
     bset    EPCrSS,shift       ; Set "EPCrSS" bit of "shift" variable
     bset    SSprog,trans2      ; Set "SSprog" bit of "trans2" variable
     lda     EPC_rise           ; Load accumulator with value in "EPC_rise"
     sta     TIMcnt             ; Copy to "TIMcnt"
     clr     dfSel              ; Clear "dfSel" variable
     bset    sel23,dfSel        ; Set "sel23" bit of "dfSel"
     bra     SHIFT23_DONE       ; Branch to SHIFT23_DONE:

SHIFT23A:
     lda     TIMcnt             ; Load accumulator with value in "TIMcnt"
                                ; variable
     bne     SHIFT23_DONE       ; If Z bit of CCR is clear, branch to
                                ; SHIFT23_DONE:
     bclr    EPCrSS,shift       ; Clear "EPCrSS" bit of "shift" variable
     bset    EPChSS,shift       ; Set "EPChSS" bit of "shift" variable
     jsr     D3_SOLS            ; Jump to subroutine at D3_SOLS:
                                ;(SS1 off, SS2 on, CCS on, "CCSon" set,
                                ; "D1D2" clr)
     lda     EPC_hold           ; Load accumulator with value in "EPC_hold"
     sta     TIMcnt             ; Copy to "TIMcnt"
     mov     gear_com,gear_cur  ; Move value in "gear_com" to "gear_cur"

SHIFT23_DONE:
     jmp     DO_DFSEL           ; Jump to DO_DFSEL:(long branch)


;****************************************************************************
; - Drive 3 to Drive 4 shift
;****************************************************************************

SHIFT34:
     brset   SSsdel,shift,SHIFT34A  ; If "SSsdel" bit of "shift" variable
                                ; is set, branch to SHIFT34A:
     bset    CCS,portc          ; Set "CCS" bit of Port C,(CCS off)
     bclr    CCSon,trans2       ; Clear "CCSon" bit of "trans2" variable
     bset    SSprog,trans2      ; Set "SSprog" bit of "trans2" variable
     bset    SSsdel,shift       ; Set "SSsdel" bit of "shift" variable
     lda     SSs_del            ; Load accumulator with value in "SSs_del"
     sta     TIMcnt             ; Copy to "TIMcnt" variable
     clr     dfSel              ; Clear "dfSel" variable
     bset    sel34,dfSel        ; Set "sel34" bit of "dfSel"
     bra     SHIFT34_DONE       ; Branch to SHIFT34_DONE:

SHIFT34A:
     lda     TIMcnt             ; Load accumulator with value in "TIMcnt"
                                ; variable
     bne     SHIFT34_DONE       ; If Z bit of CCR is clear, branch to
                                ; SHIFT34_DONE:
     bclr    SSsdel,shift       ; Clear "SSsdel" bit of "shift" variable
     bset    EPChSS,shift       ; Set "EPChSS" bit of "shift" variable
     bset    SS1,portc          ; Set "SS1" bit of Port C(SS1 off)
     bset    SS2,portc          ; Set "SS2" bit of Port C(SS2 off)
     lda     EPC_hold           ; Load accumulator with value in "EPC_hold"
     sta     TIMcnt             ; Copy to "TIMcnt"
     mov     gear_com,gear_cur  ; Move value in "gear_com" to "gear_cur"

SHIFT34_DONE:

DO_DFSEL:

;****************************************************************************
; - Check to see if the CCS apply delay timer is active. If it is, check
;   status and act accordingly, otherwise, skip over.
;****************************************************************************


     brclr   CCSdel,shift,DO_DFSEL_A  ; If "CCSdel" bit of "shift" variable
                                      ; is clear, branch to DO_DFSEL_A:
     brset   CCSon,trans2,DO_DFSEL_A  ; If "CCSon" bit of "trans2" variable
                                      ; is set, branch to DO_DFSEL_A:
     lda     TIMcnt                   ; Load accumulator with value in
                                      ; "TIMcnt" variable
     bne     DO_DFSEL_A               ; If Z bit of CCR is clear, branch to
                                      ; DO_DFSEL_A:
     lda     gearcnt
     cbeqa   #forth,NO_CCS
     bclr    CCS,portc                ; Clear "CCS" bit of Port C,(CCS on)
     bset    CCSon,trans2             ; Set "CCSon" bit of "trans2" variable

NO_CCS:
     bclr    CCSdel,shift             ; Clear "CCSdel" bit of "shift" variable

;****************************************************************************
; - Check to see if the EPC Hold timer is active. If it is, check
;   status and act accordingly, otherwise, skip over. This keeps the current
;   EPC value until the hold timer times out. The next pass will return EPC
;   "TO" value
;****************************************************************************

DO_DFSEL_A:
     brclr   EPChSS,shift,DO_DFSEL_B  ; If "EPChSS" bit of "shift" variable
                                      ; is clear, branch to DO_DFSEL_B:
     lda     TIMcnt                   ; Load accumulator with value in
                                      ; "TIMcnt" variable
     bne     DO_DFSEL_B               ; If Z bit of CCR is clear, branch to
                                      ; DO_DFSEL_B:
     bclr    EPChSS,shift             ; Clear "EPChSS" bit of "shift" var
     bclr    SSprog,trans2            ; Clear "SSprog" bit of "trans2" var

;****************************************************************************
; - Check to see if the TCC Hold timer is active. If it is, check
;   status and act accordingly, otherwise, skip over. This keeps the current
;   EPC value until the hold timer times out. The next pass will return EPC
;   "TO" value
;****************************************************************************

DO_DFSEL_B:
     brclr   EPChTCC,shift,DO_DFSEL_C  ; If "EPChTCC" bit of "shift" var
                                       ; is clear, branch to DO_DFSEL_C:
     lda     TIMcnt                    ; Load accumulator with value in
                                       ; "TIMcnt" variable
     bne     DO_DFSEL_C                ; If Z bit of CCR is clear, branch
                                       ; DO_DFSEL_C:
     bclr    EPChTCC,shift             ; Clear "EPChTCC" bit of "shift" var
     bclr    TCprog,trans2             ; Clear "TCprog" bit of "trans2" var

;****************************************************************************
;    Check to see if Torque Converter Clutch should be on or off.
;****************************************************************************

DO_DFSEL_C:
     jsr     TCC_CHKOFF        ; Branch to subroutine at TCC_CHKOFF:

;****************************************************************************
; - Check to see if DFC enable and Exhaust brake should be on or off.
;****************************************************************************

     jsr     BRK_CHKOFF        ; Jump to subroutine at BRK_CHKOFF:

;****************************************************************************
;   In priority order:
; - Check to see if vehicle is in stall condition.
;   If it is, command "stall" EPC.
; - Check to see if a gear upshift is in progress. If it is, the appropriate
;   EPC was commanded at the start of the shift sequence.
; - Check to see if TCC apply is in progress, If it is, the "TCC" EPC was
;   commanded at the start of the application sequence.
;
;   NOTE! - If the TCC apply has been commanded during an up shift, the TCC
;   pressure rise sequence is ignored, the solenoid is energised and flags
;   set. The pressure rise sequence for the shift continues as normal.
;
; - Check to see if "decel" conditions are met (closed throttle, in D4,D3,
;   or M2, with TCC on. If it is, command "decel" EPC.
; - If none of the above conditions are met, assume the vehicle is moving
;   at a steady rate. First, check to see if the vehicle is accelerating.
;   If it is, command "TCC" EPC, otherwise, command "TO" (torque output) EPC.
;****************************************************************************


DO_DFSEL_D:
     brset   MPHstall,trans,STALL_EPC  ; If "MPHstall" bit of "trans" var
                                       ; is set, branch to STALL_EPC:
                                       ;(vehicle in "stall" condiditon)
     brset   SSprog,trans2,SHIFT_EPC   ; If "SSprog" bit of "trans2"
                                       ; variableis set, branch to
                                       ; SHIFT_EPC:(shift up in progress)
     brset   TCprog,trans2,TCC_EPC     ; If "TCprog" bit of "trans2" var
                                       ; is set, branch to TCC_EPC:
                                       ;(TCC application in progress)
     brclr   ClsThrt,trans,MOVE_EPC    ; If "ClsThrt" bit of "trans" var
                                       ; is clear, branch to MOVE_EPC:
                                       ;(throttle not closed)
     brset   D1D2,trans2,MOVE_EPC      ; If "D1D2" bit of "trans2" var
                                       ; is set, branch to MOVE_EPC:
                                       ;(in D1 or D2, no engine braking)
     brset   TCCon,trans2,DECEL_EPC    ; If "TCCon" bit of "trans2" var
                                       ; is set, branch to DECEL_EPC
                                       ;(TCC applied)
     bra     MOVE_EPC                  ; Branch to MOVE_EPC:

SHIFT_EPC:
     bra     DFSEL_DONE                ; Branch to DFSEL_DONE:

TCC_EPC:
     bra     DFSEL_DONE                ; Branch to DFSEL_DONE:

STALL_EPC:
     brset   selStl,dfSel,DFSEL_DONE   ; If "selStl" bit of "dfSel"
                                       ; variable is set, branch to
                                       ; EPC_DONE:
     clr     dfSel                     ; Clear "dfSel" variable
     bset    selStl,dfSel              ; Set "selStl" bit of "dfSel"
     bra     DFSEL_DONE                ; Branch to DFSEL_DONE:

DECEL_EPC:
     brset   selDcl,dfSel,DFSEL_DONE   ; If "selDcl" bit of "dfSel"
                                       ; variable is set, branch to
                                       ; EPC_DONE:
     clr     dfSel                     ; Clear "dfSel" variable
     bset    selDcl,dfSel              ; Set "selDcl" bit of "dfSel"
     bra     DFSEL_DONE                ; Branch to DFSEL_DONE:

MOVE_EPC:
     brset   accel,trans,ACCEL_EPC     ; If "accel" bit of "trans" variable
                                       ; is set, branch to ACCEL_EPC:
                                       ;(vehicle is accelerating)

     brset   selTO,dfSel,DFSEL_DONE    ; If "selTO" bit of "dfSel"
                                       ; variable is set, branch to
                                       ; EPC_DONE:
     clr     dfSel                     ; Clear "dfSel" variable
     bset    selTO,dfSel               ; Set "selTO" bit of "dfSel"
     bra     DFSEL_DONE                ; Branch to DFSEL_DONE:

ACCEL_EPC:
     brset   selTCC,dfSel,DFSEL_DONE   ; If "selTCC" bit of "dfSel"
                                       ; variable is set, branch to
                                       ; EPC_DONE:
     clr     dfSel                     ; Clear "dfSel" variable
     bset    selTCC,dfSel              ; Set "selTCC" bit of "dfSel"

DFSEL_DONE:

;****************************************************************************
; - Determine which duty factor has been selected, and branch accordingly
;****************************************************************************

     brset   selTO,dfSel,GET_TO       ; If "selTO" bit of "dfSel" variable
                                      ; is set, branch to GET_TO:
     brset   selStl,dfSel,GET_STL     ; If "selStl" bit of "dfSel" variable
                                      ; is set, branch to GET_STL:
     brset   sel12,dfSel,GET_12       ; If "sel12" bit of "dfSel" variable
                                      ; is set, branch to GET_12:
     brset   sel23,dfSel,GET_23       ; If "sel23" bit of "dfSel" variable
                                      ; is set, branch to GET_23:
     brset   sel34,dfSel,GET_34       ; If "sel34" bit of "dfSel" variable
                                      ; is set, branch to GET_34:
     brset   selTCC,dfSel,GET_TCC     ; If "selTCC" bit of "dfSel" variable
                                      ; is set, branch to GET_TCC:
     brset   selDcl,dfSel,GET_Dcl     ; If "selDcl" bit of "dfSel" variable
                                      ; is set, branch to GET_Dcl:
     brset   selM1,dfSel,GET_M1       ; If "selM1" bit of "dfSel" variable
                                      ; is set, branch to GET_M1:

GET_TO:
     jsr     MOVE_CALCS        ; Jump to subroutine at MOVE_CALCS:
     mov     tmp6,df           ; Copy value in "tmp6" to "df"
     jmp     DF_DONE           ; Jump to DF_DONE:

GET_STL:
     jsr     STALL_CALCS       ; Jump to subroutine at STALL_CALCS:
     mov     tmp6,df           ; Copy value in "tmp6" to "df"
     jmp     DF_DONE           ; Jump to DF_DONE:

GET_12:
     jsr     SHIFT12_CALCS     ; Jump to subroutine at SHIFT12_CALCS:
     mov     tmp6,df           ; Copy value in "tmp6" to "df"
     jmp     DF_DONE           ; Jump to DF_DONE:

GET_23:
     jsr     SHIFT23_CALCS     ; Jump to subroutine at SHIFT23_CALCS:
     mov     tmp6,df           ; Copy value in "tmp6" to "df"
     jmp     DF_DONE           ; Jump to DF_DONE:

GET_34:
     jsr     SHIFT34_CALCS     ; Jump to subroutine at SHIFT34_CALCS:
     mov     tmp6,df           ; Copy value in "tmp6" to "df"
     jmp     DF_DONE           ; Jump to DF_DONE:

GET_TCC:
     lda     EPC_TCC           ; Load accumulator with value in "EPC_TCC"
     sta     df                ; Copy value in accumulator to "df"
     jmp     DF_DONE           ; Jump to DF_DONE:

GET_Dcl:
     lda     mlpsp_cur         ; Load accumulator with value in "mlpsp_cur"
     cbeqa   #M1,M1_Dcl        ; Compare with value #M1, if equal, branch
                               ; to M1_Dcl:
     lda     EPC_decel         ; Load accumulator with value in "EPC_decel"
     sta     df                ; Copy value in accumulator to "df"
     jmp     DF_DONE           ; Jump to DF_DONE:

M1_Dcl:
     lda     EPC_M1_decel      ; Load accumulator with value in "EPC_M1_decel"
     sta     df                ; Copy value in accumulator to "df"
     jmp     DF_DONE           ; Jump to DF_DONE:

GET_M1:
     jsr     M1_CALCS          ; Jump to subroutine at M1_CALCS:
     mov     tmp6,df           ; Copy value in "tmp6" to "df"

DF_DONE:

;****************************************************************************
; ----------------- Computation of Final EPC Duty Factor -------------------
;
; - Depending on conditions, initial 8 bit EPC duty factor ("df"), is set by
;   the absolute values of "EPC_TCC", or "EPC_decel", "EPC_M1_decel",
;   or the computed values from the "TO table", "EPC_stall", "EPC_12",
;   "EPC_23", "EPC_34" or "EPC_M1" tables.
;   "df" is scaled to 255 for maximum resolution(255 = 100% duty).
;   "df" is then adjusted for transmission oil temperature correction, and
;   trim correction, the result saved as Duty Factor Final("dff") "dff" is
;   then multiplied by 800 and divided by 256 to get the EPC pulse width
;   (EPCpwH:EPCpwL)
;
;   The Transmission Oil Temperature Adder changes the duty factor by the
;   Add/Subtract values calculated below, with 0
;   correction at ~160 degrees F. It's purpose is to compensate for
;   for the effects of oil viscosity changes on the operation of the
;   solonoids and friction elements.
;
;   The EPC Trim Adder changes the duty factor by the Add/Subtract
;   values calculated using the "TrimFac" correction value and measured
;   EPC Trim Pot ADC count with 0 correction at 128 counts.
;   It's purpose is for a tuning aid.
;
;   Both correction adder variables can be enabled or disabled with
;   a software switch in the "TuneConfig" variable to aid in
;   troubleshooting and or tuning.
;
; - Method is as follows, where:
;   TOTempDif = Difference between "TOTemp" and 180(140F)
;   TOTempP   = TOTemp percent calculation value (TOTemp/180)
;   TOTAdd    = Trans Oil Temp correction Add/Subtract value
;   TrimDif   = Difference between "Trim" and 128(mid point)
;   TrimP     = Trim percent calculation value (TrimDif/128)
;   TrimAdd   = EPC Trim correction Add/Subtract value
;   df        = EPC Duty Factor from "TO" table, stall or shift tables,
;               or, absolute values "EPC_TCC", or "EPC_decel"
;               (scaled to 256)
;   df1       = "df" after TOT cor, before Trim cor
;   dff       = "df1" after Trim cor(Final EPC Duty Factor)
;
;   df +/- TOTAdd = df1
;   df1 +/- TrimAdd = dff
;
;****************************************************************************

;****************************************************************************
; - Check "tconf" variable to see if TOT correction has been enabled.
;   If so, branch to the "TOTCALC" section. If not, skip over to clear
;   "TOTempP" and "TOTAdd", copy value in "df" to "df1" (no correction).
;****************************************************************************

     brset   tt,tconf,TOTCALC     ; If "tt" bit of "tconf" variable is set,
                                  ; branch to TOTCALC:
     jmp     DF_OK               ; Jump to DF_OK:


;****************************************************************************
; - Calculate EPC Duty Factor after TOT Correction("df1")
;   This is done by determining what percent "TOTemp" is from the no
;   correction value) of 160 (160F). This value is then used to calculate
;   the percent of the maximum add/subtract value (TOTempFac), and
;   is stored as "TOTAdd". The "TOTAdd" value is either added to, or
;   subtracted from "df" to result in "df1", depending on which side of the
;   no correction value of 160 that "TOTemp" falls on.
;   Temperatures above 160F will require duty factor values to be reduced
;   (lowering EPC current thereby raising pressure) Temperatures below 160F
;   will require duty factor values to be increased (lowering EPC current
;   thereby raising pressure), but this is not expected to have to be done.
;   Correction is limited by the use of programmable "TOThi" and TOTlo" values.
;****************************************************************************

TOTCALC:
     lda     TOTemp           ; Load accumulator with value Trans Oil Temp
                              ; variable (degrees F)
     cmp     #$A0             ; Compare with decimal 160(no correction)
     beq     DF_OK            ; If (A)=(M) branch to DF_OK:
     blo     DECR_TOTP        ; IF (A)<(M) branch to DECR_TOTP:

INCR_TOTP:
     cmp     TOThi            ; Compare "TOTemp"(A) with "TOThi"(M)
     bhs     RAIL_TOT_HI      ; If A>=M branch to RAIL_TOT_HI:

SUB_A0:
     sub     #$A0             ; Subtract (A)<-(A)-(M)("TOTemp" - 160)
                              ; or ("TOThi" - 160)
     sta     TOTempDif        ; Copy result to "TOTempDif"
     jmp     DO_TOTP          ; Jump to DO_TOTP:

RAIL_TOT_HI:
     lda     TOThi            ; Load accumulator with value Trans Oil Temp
                              ; correction Hi limit (degrees F)
     bra     SUB_A0           ; Branch to SUB_A0:

DECR_TOTP:
     cmp     TOTlo            ; Compare "TOTemp"(A) with "TOTlo"(M)
     bls     RAIL_TOT_LO      ; If A<=M branch to RAIL_TOT_LO:
     lda     #$A0             ; Load accumulator with decimal 160
     sub     TOTemp           ; Subtract (A)<-(A)-(M)(160 - "TOTemp")
     sta     TOTempDif        ; Copy result to "TOTempDif"
     jmp     DO_TOTP          ; Jump to DO_TOTP:

RAIL_TOT_LO:
     lda     #$A0             ; Load accumulator with decimal 160
     sub     TOTlo            ; Subtract (A)<-(A)-(M)(160 - "TOTlo")
     sta     TOTempDif        ; Copy result to "TOTempDif"

DO_TOTP:
     tax                      ; Transfer value in accumulator to index register
                              ; Lo byte ("TOTempif" to X)
     lda     #$64             ; Load accumulator with decimal 100
     mul                      ; Multiply, (X:A)<-(X)*(A)(TOTempDif*100)
     pshx                     ; Push value in index register Lo byte to stack
     pulh                     ; Pull value from stack to index register Hi byte
                              ;(X to H)
     ldx     #$A0             ; Load index register Lo byte with decimal 160
                              ;(no correction)
     div                      ; Divide, (A)<-(H:A)/(X);(H)<-Rem (result/100)
     jsr     DIVROUND         ; Jump to subroutine at DIVROUND:
     sta     TOTempP          ; Copy result to "TOTempP"
     tax                      ; Transfer value in accumulator to index register
                              ; Lo byte ("TOTempP" to X)
     lda     TOTempFac        ; Load accumulator with value in "TOTempFac"
     mul                      ; Multiply, (X:A)<-(X)*(A)(TOTempP*TOTempFac)
     pshx                     ; Push value in index register Lo byte to stack
     pulh                     ; Pull value from stack to index register Hi byte
                              ;(X to H)
     ldx     #$64             ; Load index register Lo byte with decimal 100
     div                      ; Divide, (A)<-(H:A)/(X);(H)<-Rem (result/100)
     jsr     DIVROUND         ; Jump to subroutine at DIVROUND:
     sta     TOTAdd           ; Copy result to "TOTAdd"
     lda     TOTemp           ; Load accumulator with value Trans Oil Temp
                              ; variable (degrees F - 40)
     cmp     #$A0             ; Compare with decimal 160(no correction)
;*     blo     DECR_DF          ; IF (A)<(M) branch to DECR_DF:
     bhi     DECR_DF          ; IF (M)<(A) branch to DECR_DF:


INCR_DF:
     lda     df               ; Load accumulator with value in "df"
     add     TOTAdd           ; Add (A)<-(A)+(M)("df + "TOTAdd")
     sta     df1              ; Copy result to "df1"
     jmp     TOTCALC_DONE     ; Jump to TOTCALC_DONE:

DECR_DF:
     lda     df              ; Load accumulator with value in "df"
     sub     TOTAdd           ; Subtract (A)<-(A)-(M)("df - "TOTAdd")
     sta     df1              ; Copy result to "df1"
     jmp     TOTCALC_DONE     ; Jump to TOTCALC_DONE:

DF_OK:
     clr     TOTempP          ; Clear "TOTempP"
     clr     TOTAdd           ; Clear "TOTAdd"
     mov     df,df1          ; Move value in "df" to "df1"

TOTCALC_DONE:

;****************************************************************************
; - Check "tconf" variable to see if EPC Trim correction has been enabled.
;   If so, branch to the "TRIMCALC" section. If not, skip over to clear
;   "TrimP" and "TrimAdd", copy value in "df1" to "dff" (no correction).
;****************************************************************************

     brset   tr,tconf,TRIMCALC     ; If "tr" bit of "tconf" variable is set,
                                   ; branch to TRIMCALC:
     jmp     DF1_OK                ; Jump to DF1_OK:

;****************************************************************************
; - Calculate final EPC Duty Factor after EPC Trim Correction("dff")
;   This is done by determining what percent "Trim" is from the mid point
;   (no correction value) of 128 (TrimP). This value is then used to
;   calculate the percent of the maximum add/subtract value (TrimFac), and
;   is stored as "TrimAdd". The "TrimAdd" value is either added to, or
;   subtracted from "df1" to result in "dff", depending on which side of the
;   no correction value of 128 that "Trim" falls on.
;****************************************************************************

TRIMCALC:
     lda     Trim             ; Load accumulator with value in EPC Trim
                              ; ADC reading
     cmp     #$80             ; Compare with decimal 128(mid point)
     beq     DF1_OK           ; If (A)=(M) branch to DF1_OK:
     blo     DECR_TRIMP       ; IF (A)<(M) branch to DECR_TRIMP:

INCR_TRIMP:
     lda     Trim             ; Load accumulator with value in EPC Trim
                              ; ADC reading
     sub     #$80             ; Subtract (A)<-(A)-(M)("Trim" - 128)
     sta     TrimDif          ; Copy result to "TrimDif"
     jmp     DO_TRIMP         ; Jump to DO_TRIMP:

DECR_TRIMP:
     lda     #$80             ; Load accumulator with decimal 128
     sub     Trim             ; Subtract (A)<-(A)-(M)(128 - "Trim")
     sta     TrimDif          ; Copy result to "TrimDif"

DO_TRIMP:
     tax                      ; Transfer value in accumulator to index register
                              ; Lo byte ("TrimDif" to X)
     lda     #$64             ; Load accumulator with decimal 100
     mul                      ; Multiply, (X:A)<-(X)*(A)(TrimDif*100)
     pshx                     ; Push value in index register Lo byte to stack
     pulh                     ; Pull value from stack to index register Hi byte
                              ;(X to H)
     ldx     #$80             ; Load index register Lo byte with decimal 128
                              ;(mid point)
     div                      ; Divide, (A)<-(H:A)/(X);(H)<-Rem (result/100)
     jsr     DIVROUND         ; Jump to subroutine at DIVROUND:
     sta     TrimP            ; Copy result to "TrimP"
     tax                      ; Transfer value in accumulator to index register
                              ; Lo byte ("TrimP" to X)
     lda     TrimFac          ; Load accumulator with value in "TrimFac"
     mul                      ; Multiply, (X:A)<-(X)*(A)(TrimP*TrimFac)
     pshx                     ; Push value in index register Lo byte to stack
     pulh                     ; Pull value from stack to index register Hi byte
                              ;(X to H)
     ldx     #$64             ; Load index register Lo byte with decimal 100
     div                      ; Divide, (A)<-(H:A)/(X);(H)<-Rem (result/100)
     jsr     DIVROUND         ; Jump to subroutine at DIVROUND:
     sta     TrimAdd          ; Copy result to "TrimAdd"
     lda     Trim             ; Load accumulator with value in EPC Trim
                              ; ADC reading
     cmp     #$80             ; Compare with decimal 128(mid point)
     blo     DECR_DF1         ; IF (A)<(M) branch to DECR_DF1:

INCR_DF1:
     lda     df1              ; Load accumulator with value in "df1"
     add     TrimAdd          ; Add (A)<-(A)+(M)("df1 + "TrimAdd")
     sta     dff              ; Copy result to "dff"
     jmp     TRIMCALC_DONE    ; Jump to TRIMCALC_DONE:

DECR_DF1:
     lda     df1              ; Load accumulator with value in "df1"
     sub     TrimAdd          ; Subtract (A)<-(A)-(M)("df1 - "TrimAdd")
     sta     dff              ; Copy result to "dff"
     jmp     TRIMCALC_DONE    ; Jump to TRIMCALC_DONE:

DF1_OK:
     clr     TrimP            ; Clear "TrimP"
     clr     TrimAdd          ; Clear "TrimAdd"
     mov     df1,dff          ; Move value in "df1" to "dff"

TRIMCALC_DONE:

;****************************************************************************
; -------------- Calculate the EPC Pulse Width(EPCpwH:EPCpwL) ---------------
;
;   The EPC solenoid analog control voltage operates with a PWM at 10kHZ
;   The period is 0.0001 seconds. The timer counts in ~0.125uS intervals,
;   so the period is 800 counts (100% duty). (800*0.125=0.000100 seconds.
;
;   dff     = Final EPC Duty Factor
;   EPCpwH  = EPC pulse width Hi byte
;   EPCpwL  = EPC pulse width Lo byte
;
;   EPCpwH:EPCpwL<-(800*(dff))/256
;
;****************************************************************************

;****************************************************************************
; - Multiply 800 by value in "dff"
;****************************************************************************

     clr     tmp2          ; Clear "tmp2"
     clr     tmp5          ; Clear "tmp5"
     clr     tmp6          ; Clear "tmp6"
     clr     tmp7          ; Clear "tmp7"
     mov     dff,tmp1      ; Move value in "dff" to "tmp1"
     mov     #$20,tmp3     ; Move decimal 32 to "tmp3"
     mov     #$03,tmp4     ; Move decimal 3 to "tmp4"
                           ;(3*256=768+32=800)
     jsr     UMUL32        ; Jump to subroutine at UMUL32:

;****************************************************************************
; - Divide result by 256
;   This is done by ignoring the least significant byte of the result of
;   multiplication above, which has the effect of dividing by 256,
;   rounded down.
;****************************************************************************

     mov     tmp6,EPCpwL     ; Move value in "tmp6" to "EPCpwL"
     mov     tmp7,EPCpwH     ; Move value in "tmp7" to "EPCpwH"


LOOP_END:
     jmp     LOOPER    ; Jump to LOOPER: (End of Main Loop!!!)


;****************************************************************************
;
; * * * * * * * * * * * * * * Interrupt Section * * * * * * * * * * * * * *
;
; NOTE!!! If the interrupt service routine modifies the H register, or uses
; the indexed addressing mode, save the H register (pshh) and then restore
; it (pulh) prior to exiting the routine
;
;****************************************************************************

;****************************************************************************
;
; -------- Following interrupt service routines in priority order ----------
;
; IRQ_ISR:     - Tach input (tach input Hi, IRQ Lo, hardware inverted)
;
; TIM1OV_ISR   - TIM1 at modulo value ($0320 * 0.125uS) =
;               (100us period, 10KHZ, pulse width "on" for EPC PWM)
;
; TIM2CH0_ISR: - TIM2 CH0 output compare ($0064 * 1uS) (100us Timer Tick)
;
; SCIRCV_ISR:  - SCI receive
;
; SCITX_ISR:   - SCI transmit
;
; KYBD_ISR:    - Keyboard interrupt for Vehicle Speed input (8000 ppMi)
;                (Veh Spd input Hi, PTA0 Lo, hardware inverted)
;
; ADC_ISR:     - ADC Conversion Complete
;
;
;****************************************************************************

;****************************************************************************
;============================================================================
; - IRQ - Input trigger (tach) for RPM calculations
;============================================================================
;****************************************************************************

IRQ_ISR:

;****************************************************************************
; - Capture the RPM period in 100uS resolution from the RPM counter values
;****************************************************************************

     lda     RPMcH          ; Load accumulator with value in RPM counter
                            ; Hi byte
     sta     RPMpH          ; Copy to RPM Period Hi byte
     lda     RPMcL          ; Load accumulator with value in RPM counter
                            ; Lo byte
     sta     RPMpL          ; Copy to RPM Period Low byte
     clr     RPMcH          ; Clear RPM counter Hi byte
     clr     RPMcL          ; Clear RPM counter Lo byte

IRQ_DONE:
     bset    tachrise,inputs  ; Set "tachrise" bit of "inputs" variable
     rti                      ; Return from interrupt



;****************************************************************************
;============================================================================
; - TIM1 Modulo Interrupt(0.125uS resolution, 100uS period, 10KHZ)
;   (Pulse Width "on" for EPC PWM)
;============================================================================
;****************************************************************************

TIM1OV_ISR:

;****************************************************************************
; - Clear TIM1 modulo interrupt and set EPC pulse width "on"
;****************************************************************************

     lda     T1SC         ; Load accumulator with value in TIM1 Status
                          ; and Control Register (Arm TOF flag clear)
     bclr    TOF,T1SC     ; Clear TOF bit of TIM1 Status and
                          ; Control Register
     clr     T1SC0        ; Clear TIM1 CH0 Status and Control Register
                          ; (PTD4 port control, logic Hi)


;****************************************************************************
; - Load and arm TIM1 CH1 with output compare value for EPC pulse width "off"
;****************************************************************************

     mov     EPCpwH,T1CH0H  ; Move value in Final EPC pulse width Hi byte to
                            ; Tim1 Ch0 O/C register Hi byte
     mov     EPCpwL,T1CH0L  ; Move value in Final EPC pulse width Lo byte to
                            ; Tim1 Ch0 O/C register Lo byte
     mov     #$18,T1SC0     ; Move %00011000 into TIM1 CH0 Status and
                            ; Control Register,(PTD4,clear output on compare)
     rti                    ; Return from interrupt


;****************************************************************************
;============================================================================
; - TIM2 CH0 Interrupt (100 uS clock tick)
; - Generate time rates:
;   100 Microseconds,(for RPM, MPH, IAC PWM counters)
;   Milliseconds,(for ADC conversions and contact de-bounce counters)
;   5 Milliseconds,(for 200hz clock tick for IAC control PWM frequency
;   20 Milliseconds,(for EPC time delay functions)
;   100 Milliseconds,(for TPS DOT calcs, and EPC timer counters)
;   Seconds,(because we can)
;============================================================================
;****************************************************************************

TIM2CH0_ISR:
     pshh                  ; Push value in index register Hi byte to stack
     lda     T2SC0         ; Load accumulator with value in TIM2 CH0 Status
                           ; and Control Register (Arm CHxF flag clear)
     bclr    CHxF,T2SC0    ; Clear CHxF bit of TIM2 CH0 Status and
                           ; Control Register
     ldhx    T2CH0H        ; Load index register with value in TIM2 CH0
                           ; register H:L (output compare value)
     aix     #$64          ; Add decimal 100 (100 uS)
     sthx    T2CH0H        ; Copy result to TIM2 CH0 register
                           ;(new output compare value)


;============================================================================
;********************** 100 Microsecond section *****************************
;============================================================================


;****************************************************************************
; - Increment the RPM counter
;****************************************************************************

INC_RPM_CNTR:
     inc      RPMcL           ; Increment counter for RPM Lo byte
     bne      INC_MPH_CNTR    ; If the Z bit of the CCR is clear,
                              ; branch to INC_MPH_CNTR:
     inc      RPMcH           ; Increment counter for RPM Hi byte

;****************************************************************************
; - Check to see if the engine has stopped, or has not started yet. If the
;   engine is not running, set the "Estop" bit of "trans3" variable and
;   clear the "EPCpwH:EPCpwL". Clear the RPM counters and "RPM" variable.
;   If it is running, clear the "Estop" bit.
;****************************************************************************

E_STOP_CHK:
     lda     RPMcH            ; Load accumulator with value in RPM counter
                              ; Hi byte
     cmp     #$64             ; Compare with decimal 100(100*256=25600
                              ;(RPM period of ~2.56 sec = ~5.6RPM 8cyl)
     bne     E_RUN            ; If Z bit of CCR is clear, branch to E_RUN:
     bset    Estop,trans      ; Set "Estop" bit of "trans" variable
     clr     RPMcH            ; Clear RPM counter Hi Byte
     clr     RPMcL            ; Clear RPM counter Lo Byte
     clr     RPM              ; Clear RPM/20 variable
     bra     INC_MPH_CNTR     ; Branch to INC_MPH_CNTR:

E_RUN:
     bclr    Estop,trans      ; Clear "Estop" bit of "trans" variable


;****************************************************************************
; - Increment the MPH counter
;****************************************************************************

INC_MPH_CNTR:
     inc     MPHcL            ; Increment counter for MPH Lo byte
     bne     DEC_IAC_CNTR     ; If the Z bit of the CCR is clear,
                              ; branch to DEC_IAC_CNTR:
     inc     MPHcH            ; Increment counter for MPH Hi byte

;****************************************************************************
; - Check to see if the vehicle has stopped, or has not moved yet. If the
;   vehicle is not moving, set the "Vstop" bit of "trans3" variable and clear
;   the MPH counters and "MPH" variable.
;   If it is moving, clear the "Vstop" bit.
;****************************************************************************

V_STOP_CHK:
     lda     MPHcH            ; Load accumulator with value in MPH counter
                              ; Hi byte
     cmp     #$23             ; Compare with decimal 35(35*256=8960)
                              ;(MPH period of .896 sec = < .5MPH)
     bne     V_RUN            ; If Z bit of CCR is clear, branch to V_RUN:
     bset    Vstop,trans      ; Set "Vstop" bit of "trans" variable
     clr     MPHcH            ; Clear MPH counter Hi Byte
     clr     MPHcL            ; Clear MPH counter Lo Byte
     clr     MPH              ; Clear MPH*2 variable
     bra     DEC_IAC_CNTR     ; Branch to DEC_IAC_CNTR:

V_RUN:
     bclr    Vstop,trans      ; Clear "Vstop" bit of "trans" variable


;****************************************************************************
; - Decrement the IAC PWM counter
;****************************************************************************

DEC_IAC_CNTR:
     lda     IACcnt         ; Load accumulator with value in IAC counter
     beq     DEC_IAC_DONE   ; If Z bit of CCR is set, branch to DEC_IAC_DONE:
     dec     IACcnt         ; Decrement IAC PWM "off" counter

DEC_IAC_DONE:


;****************************************************************************
; - Increment 100 Microsecond counter
;****************************************************************************

INC_cuS:
     inc     uSx100       ; Increment 100 Microsecond counter
     lda     uSx100       ; Load accumulator with 100 Microsecond counter
     cmp     #$0A         ; Compare it with decimal 10
     bne     NOT_MS       ; If not equal, branch to NOT_MS:
     bra     FIRE_ADC     ; Branch to FIRE_ADC:

NOT_MS:     jmp     TIM2CH0_ISR_DONE

;============================================================================
;************************* millisecond section ******************************
;============================================================================


;****************************************************************************
; - Fire off another ADC conversion, channel is pointed to by "adsel"
;****************************************************************************

FIRE_ADC:
     lda     adsel          ; Load accumulator with ADC Selector Variable
     ora     #%01000000     ; Inclusive "or" with %01000000 and ADC Selector
                            ; Variable ( result in accumulator )
                            ;(Enables interupt with channel selected)
     sta     ADSCR          ; Copy result to ADC Status and Control Register

;****************************************************************************
; - Check the value of the contact de-bounce counter variables, if other
;   than zero, decrement them.
;****************************************************************************

     lda     ShftUpDB           ; Load accumulator with value in "ShftUpDB"
                                ; variable
     beq     ShftUpDB_CHK_DONE  ; If "Z" bit of "CCR is set, branch to
                                ; ShftUpDB_CHK_DONE:
     dec     ShftUpDB           ; Decrement "ShftUpDB" variable

ShftUpDB_CHK_DONE:

     lda     ShftDnDB           ; Load accumulator with value in "ShftDnDB"
                                ; variable
     beq     ShftDnDB_CHK_DONE  ; If "Z" bit of "CCR is set, branch to
                                ; ShftDnDB_CHK_DONE:
     dec     ShftDnDB           ; Decrement "ShftDnDB" variable

ShftDnDB_CHK_DONE:


;****************************************************************************
; - Increment millisecond counter
;****************************************************************************

INC_mS:
     clr     uSx100              ; Clear 100 Microsecond counter
     inc     mS                  ; Increment Millisecond counter
     lda     mS                  ; Load accumulator with value in
                                 ; Millisecond counter
     cmp     #$05                ; Compare it with decimal 5
     beq     DO_IAC              ; If Z bit of CCR is set, branch to DO_IAC:
                                 ;(mS=5)
     jmp     TIM2CH0_ISR_DONE    ; Jump to TIM2CH0_ISR_DONE:


;============================================================================
;************************** 5 Millisecond section ***************************
;============================================================================

;****************************************************************************
; - Set AIC solonoid PWM "on" PTD5
;****************************************************************************

DO_IAC:
     lda     IACpw         ; Load accumulator with value in IAC pulse width
     beq     NO_IAC        ; If Z bit of CCR is set, branch to NO_IAC:
                           ;(No IAC commanded so no PWM)
     sta     IACcnt        ; Copy to IAC PWM "off" counter
     bclr    iacpwm,portd  ; Clear "iacpwm" bit of Port D (IAC PW "on")
     bset    iacon,inputs  ; Set "iacon" bit of "inputs" variable
     bra     IAC_DONE      ; Branch to IAC_DONE:

NO_IAC:
     bset    iacpwm,portd  ; Set "iacpwm" bit of Port D (IAC PW "off")
     bclr    iacon,inputs  ; Clear "iacon" bit of "inputs" variable

IAC_DONE:

;****************************************************************************
; - Increment 5 millisecond counter
;****************************************************************************

INC_mSx5:
     clr     mS                  ; Clear "mS"
     inc     mSx5                ; Increment 5 Millisecond counter
     lda     mSx5                ; Load accumulator with value in
                                 ; 5 Millesecond counter
     cmp     #$04                ; Compare it with decimal 4
     bne     TIM2CH0_ISR_DONE    ; If the Z bit of CCR is clear,
                                 ; branch to TIM2CH0_ISR_DONE:

;============================================================================
;************************* 20 Millisecond section ***************************
;============================================================================

;****************************************************************************
; - Decrement the 20mS timer counter
;****************************************************************************

DEC_TIM_CNTR:
     lda     TIMcnt         ; Load accumulator with value in TIM counter
     beq     DEC_TIM_DONE   ; If Z bit of CCR is set, branch to DEC_TIM_DONE:
     dec     TIMcnt         ; Decrement 20mS timer counter

DEC_TIM_DONE:

;****************************************************************************
; - Increment 20 Millisecond counter
;****************************************************************************

INC_mSx20:
     clr     mSx5                ; Clear 5 Millisecond counter
     inc     mSx20               ; Increment 20 Millisecond counter
     lda     mSx20               ; Load accumulator with value in
                                 ; 20 Millesecond counter
     cmp     #$05                ; Compare it with decimal 5
     bne     TIM2CH0_ISR_DONE    ; If the Z bit of CCR is clear,
                                 ; branch to TIM2CH0_ISR_DONE:


;============================================================================
;************************* 100 Millisecond section **************************
;============================================================================

     bset    clk100k,inputs     ; Set "clk100k" bit of "inputs" variable

 ;****************************************************************************
; - Increment 100 Millisecond counter
;****************************************************************************

INC_cmS:
     clr     mSx20               ; Clear 20 Millisecond counter
     inc     mSx100              ; Increment 100 Millisecond counter
     lda     mSx100              ; Load accumulator with value in
                                 ; 100 Millisecond counter
     cmp     #$0A                ; Compare with decimal 10
     beq     INC_S               ; If Z bit of CCR is set, branch to INC_S:
     bra     TIM2CH0_ISR_DONE    ; Branch to TIM2CH0_ISR_DONE:

;============================================================================
;**************************** Seconds section *******************************
;============================================================================

;****************************************************************************
; - Increment Seconds counter
;****************************************************************************

INC_S:
     clr     mSx100              ; Clear 0.1 Second variable
     inc     secl                ; Increment "Seconds" Lo byte variable
     bne     TIM2CH0_ISR_DONE    ; If the Z bit of CCR is clear, branch
                                 ; to TIM2CH0_ISR_DONE:
     inc     sech                ; Increment "Seconds" Hi byte variable

TIM2CH0_ISR_DONE:
     pulh                  ; Pull value from stack to index register Hi byte
     rti                   ; Return from interrupt

;****************************************************************************
;
; -------------------- Serial Communications Interface ----------------------
;
; Communications is established when the PC communications program sends
; a command character - the particular character sets the mode:
;
; "A" = send all of the realtime variables via txport.
; "V" = send the Constants group 1 via txport (128 bytes)
;       (TO_table, RPM_range, KPA_range, TPS_range, EPC_stall, EPC_12,
;        EPC_23, EPC_34, tables, + 8 spares)
; "W"+<offset>+<newbyte> = receive new table byte value and store in
;       offset location
; "B" = jump to flash burner routine and all table/constant values in
;       RAM into flash
; "C" = Test communications - echo back SECL
; "Q" = Send over Embedded Code Revision Number (divide number by 10
;  - i.e. $21T is rev 2.1)
; "I" = send the Constants group 2 via txport (64 bytes)
; "J"+<offset>+<newbyte> =receive new table byte value and store in
;       offset location
;
; txmode:
;              01 = Getting realtime data
;              02 = ?
;              03 = Sending G1
;              04 = ?
;              05 = Getting offset G1
;              06 = Getting data G1
;              07 = Getting offset G2
;              08 = Getting data G2
;              09 = Sending G2
;
;***************************************************************************

SCIRCV_ISR:
     pshh                 ; Push value in index register Hi byte to Stack
     lda     SCS1         ; Load accumulator with value in "SCS1"
                          ;(Clear the SCRF bit by reading this register)
     lda     txmode       ; Load accumulator with value in "txmode" variable
                          ;(Check if we are in the middle of a receive
                          ; new VE/constant)
     cmp     #$05         ; Compare with decimal 5
     beq     TXMODE_5     ; If the Z bit of CCR is set, branch to TXMODE_5:
     cmp     #$06         ; Compare with decimal 6
     beq     TXMODE_6     ; If the Z bit of CCR is set, branch to TXMODE_6:
     cmp     #$07         ; Compare with decimal 7
     beq     TXMODE_7     ; If the Z bit of CCR is set, branch to TXMODE_7:
     cmp     #$08         ; Compare with decimal 8
     beq     TXMODE_8     ; If the Z bit of CCR is set, branch to TXMODE_8:
     bra     CHECK_TXCMD  ; Branch to CHECK_TXCMD:

TXMODE_5:                 ; (Getting offset for either W or J command)

TXMODE_7:
     mov     SCDR,rxoffset   ; Move value in "SCDR" to "rxoffset"
     inc     txmode          ; (continue to next mode)
     jmp     DONE_RCV        ; Jump to DONE_RCV:

TXMODE_6:
     clrh                 ; Clear index register Hi byte
     lda     rxoffset     ; Load accumulator with value in "rxoffset"
     tax                  ; Transfer value in accumulator to index register
                          ; Lo byte
     lda     SCDR         ; Load accumulator with value in "SCDR"
     sta     TO_table,x   ; Copy to TO_table, offset in index register Lo byte
                          ;(Write data to TO_table + offset)
     clr     txmode       ; Clear "txmode" variable
     jmp     DONE_RCV     ; Jump to DONE_RCV:

TXMODE_8:
     clrh                 ; Clear index register Hi byte
     lda     rxoffset     ; Load accumulator with value in "rxoffset"
                          ; (Get offset for data to be written)
     tax                  ; Transfer value in accumulator to index register
                          ; Lo byte
     lda     SCDR         ; Load accumulator with value in "SCDR"
     sta     EPC_TCC,x    ; Copy to EPC_TCC, offset in index register Lo byte
                          ;(Write data to EPC_TCC + offset)
     clr     txmode       ; Clear "txmode" variable
     jmp     DONE_RCV     ; Jump to DONE_RCV:

CHECK_TXCMD:
     lda     SCDR       ; Load accumulator with value in "SCDR"
                        ;(Get the command byte)
     cmp     #$41       ; Compare it with decimal 65 = ASCII "A"
                        ;(Is the recieve character a big "A" ->
                        ; Download real-time variables?)
     beq     MODE_A     ; If the Z bit of CCR is set, branch to Mode_A:
     cmp     #$42       ; Compare it with decimal 66 = ASCII "B"
     beq     MODE_B     ; If the Z bit of CCR is set, branch to Mode_B:
     cmp     #$43       ; Compare it with decimal 67 = ASCII "C"
     beq     MODE_C     ; If the Z bit of CCR is set, branch to Mode_C:
     cmp     #$56       ; Compare it with decimal 86 = ASCII "V"
     beq     MODE_V     ; If the Z bit of CCR is set, branch to Mode_V:
     cmp     #$57       ; Compare it with decimal 87 = ASCII "W"
     beq     MODE_W     ; If the Z bit of CCR is set, branch to Mode_W:
     cmp     #$51       ; Compare it with decimal 81 = ASCII "Q"
     beq     MODE_Q     ; If the Z bit of CCR is set, branch to Mode_Q:
     cmp     #'I'       ; Compare it with 'I' = ASCII decimal 73 $49
     beq     MODE_I     ; If the Z bit of CCR is set, branch to Mode_I:
     cmp     #'J'       ; Compare it with 'J' = ASCII decimal74 $4A
     beq     MODE_J     ; If the Z bit of CCR is set, branch to Mode_J:
     bra     DONE_RCV   ; Branch to DONE_RCV:

MODE_A
     clr     txcnt          ; Clear "txcnt"
     lda     #$01           ; Load accumulator with decimal 1
     sta     txmode         ; Copy to "txmode" variable
     lda     #$22           ; Load accumulator with decimal 34
                            ;(Set this for 1 more than the number of bytes
                            ; to send)
                            ;(33 Real time variables for MV_TECA)
;!     lda     #$70           ; Load accumulator with decimal 112
                            ;(Set this for 1 more than the number of bytes
                            ; to send)
                            ; Show all variables(for Megachat)
     sta     txgoal         ; Copy to "txgoal" variable
     bset    TE,SCC2        ; Set "TE" bit of SCC2 (Enable Transmit)
     bset    SCTIE,SCC2     ; Set "SCTIE" bit of SCC2
                            ;(Enable transmit interrupt)
     bra     DONE_RCV       ; Branch to DONE_RCV:

MODE_B:
     jsr     burnConst     ; Jump to "burnConst" subroutine
     clr     txmode        ; Clear "txmode" variable
     bra     DONE_RCV      ; Branch to DONE_RCV:

MODE_C:
     clr     txcnt          ; Clear "txcnt"
                            ; (Just send back SECL variable to test comm port)
     lda     #$01           ; Load accumulator with decimal 1
     sta     txmode         ; Copy to "txmode" variable
     lda     #$2            ; Load accumulator with decimal 2
     sta     txgoal         ; Copy to "txgoal" variable
     bset    TE,SCC2        ; Set "TE" bit of SCC2 (Enable Transmit)
     bset    SCTIE,SCC2     ; Set "SCTIE" bit of SCC2
                            ;(Enable transmit interrupt)
     bra     DONE_RCV       ; Branch to DONE_RCV:

MODE_V:
     clr     txcnt          ; Clear "txcnt"
     lda     #$03           ; Load accumulator with decimal 3
     sta     txmode         ; Copy to "txmode" variable
     lda     #$81           ; Load accumulator with decimal 129
                            ;(Set this for 1 more than the number of bytes
                            ; to send)
                            ;(Send 128 bytes, TO table, ranges, EPC stall
                            ; and shift tables + spares)
     sta     txgoal         ; Copy to "txgoal" variable
     bset    TE,SCC2        ; Set "TE" bit of SCC2 (Enable Transmit)
     bset    SCTIE,SCC2     ; Set "SCTIE" bit of SCC2
                            ;(Enable transmit interrupt)
     bra     DONE_RCV       ; Branch to DONE_RCV:

MODE_W:
     lda     #$05         ; Load accumulator with decimal 5
     sta     txmode       ; Copy to "txmode" variable
     bra     DONE_RCV     ; Branch to DONE_RCV:

MODE_Q:
     clr     txcnt          ; Clear "txcnt"
                            ; (Just send back SECL variable to test comm port)
     lda     #$05           ; Load accumulator with decimal 5
     sta     txmode         ; Copy to "txmode" variable
     lda     #$2            ; Load accumulator with decimal 2
     sta     txgoal         ; Copy to "txgoal" variable
     bset    TE,SCC2        ; Set "TE" bit of SCC2 (Enable Transmit)
     bset    SCTIE,SCC2     ; Set "SCTIE" bit of SCC2
                            ;(Enable transmit interrupt)
     bra     DONE_RCV       ; Branch to DONE_RCV:

MODE_I:
     clr     txcnt          ; Clear "txcnt"
     lda     #$09           ; Load accumulator with decimal 9
                            ; (txmode = sending Cons Group 2)
     sta     txmode         ; Copy to "txmode" variable
     lda     #$41           ; Load accumulator with decimal 65
                            ;(Set this for 1 more than the number of bytes
                            ; to send)
                            ;(Send 64 bytes, constants + spares)
     sta     txgoal         ; Copy to "txgoal" variable
     bset    TE,SCC2        ; Set "TE" bit of SCC2 (Enable Transmit)
     bset    SCTIE,SCC2     ; Set "SCTIE" bit of SCC2
                            ;(Enable transmit interrupt)
     bra     DONE_RCV       ; Branch to DONE_RCV:

MODE_J:
     lda     #$07         ; Load accumulator with decimal 7
                          ; (txmode = getting offset Cons group 2)
     sta     txmode       ; Copy to "txmode" variable

DONE_RCV
     pulh                 ; Pull value from Stack to index register Hi byte
     rti                  ; Return from interrupt

;****************************************************************************
;----------------- Transmit Character Interrupt Handler --------------------
;****************************************************************************

SCITX_ISR:
     pshh                  ; Push value in index register Hi byte to Stack
     lda     SCS1          ; Load accumulator with value in "SCS1"
                           ; (Clear the SCRF bit by reading this register)
     clrh                  ; Clear index register Hi byte
     lda     txcnt         ; Load accumulator with value in "txcnt" variable
     tax                   ; Transfer value in accumulator to index register
                           ; Lo byte
     lda     txmode        ; Load accumulator with value in "txmode" variable
     cmp     #$05          ; Compare it with decimal 5
     beq     IN_Q_MODE     ; If the Z bit of CCR is set, branch to IN_Q_MODE:
     cmp     #$09          ; Compare it with decimal 9
     beq     IN_I_MODE     ; If the Z bit of CCR is set, branch to IN_I_MODE:
     cmp     #$01          ; Compare it with decimal 1
     bne     IN_V_MODE     ; If the Z bit of CCR is clear, branch to IN_V_MODE:

IN_A_OR_C_MODE:
     lda     secH,X      ; Load accumulator with value in address "secH",
                         ; offset in index register Lo byte
     bra     CONT_TX     ; Branch to CONT_TX:

IN_V_MODE
     lda     TO_table,x  ; Load accumulator with value in address
                         ; "TO_table", offset in index register Lo byte
     bra     CONT_TX     ; Branch to CONT_TX:

IN_I_MODE
     lda     EPC_TCC,x   ; Load accumulator with value in address "EPC_TCC",
                         ; offset in index register Lo byte
     bra     CONT_TX     ; Branch to CONT_TX:

IN_Q_MODE
     lda     REVNUM,X   ; Load accumulator with value in address "REVNUM",
                        ; offset in index register Lo byte

CONT_TX:
     sta     SCDR           ; Copy to "SCDR" variable (Send char)
     lda     txcnt          ; Load accumulator with value in "txcnt" variable
     inca                   ; Increment value in accumulator
                            ;(Increase number of chars sent)
     sta     txcnt          ; Copy to "txcnt" variable
     cmp     txgoal         ; Compare it to value in "txgoal" (Check if done)
     bne     DONE_XFER      ; If the Z bit of CCR is clear, branch to DONE_XFER:
                            ;(Branch if NOT done to DONE_XFER !?!?!)
     clr     txcnt          ; Clear "txcnt"
     clr     txgoal         ; Clear "txgoal"
     clr     txmode         ; Clear "txmode"
     bclr    TE,SCC2        ; Clear "TE" bit of SCC2 (Disable Transmit)
     bclr    SCTIE,SCC2     ; Clear "SCTIE" bit of SCC2
                            ;(Disable transmit interrupt)

DONE_XFER
     pulh                   ; Pull value from Stack to index register Hi byte
     rti                    ; Return from interrupt

;****************************************************************************
;============================================================================
; - Keyboard interrupt (Vehicle Speed on PTA0)
;============================================================================
;****************************************************************************

KEYBD_ISR:

;****************************************************************************
; - Calculate Vehicle Speed Period
;****************************************************************************

     lda     MPHcH           ; Load accumulator value in MPH counter Hi byte
     sta     MPHpH           ; Copy to MPH period Hi byte
     lda     MPHcL           ; Load accumulator with value in MPH counter
                             ; Lo byte
     sta     MPHpL           ; Copy to MPH period Lo byte
     clr     MPHcH           ; Clear MPH counter Hi byte
     clr     MPHcL           ; Clear MPH counter Lo byte
     bset    IMASKK,INTKBSCR     ; Set the Interrupt Mask bit of
                                 ; Keyboard Status and Control Register
                                 ;(disable interrupts)
     bset    ACKK,INTKBSCR       ; Set the Keyboard Acknowledge bit of
                                 ; Keyboard Status and Control Register
                                 ;(clear interrupt)
     bset    vssp,inputs          ; Set "vssp" bit of "inputs" variable
     rti                         ; Return from interrupt



;****************************************************************************
; - ADC conversion complete Interrupt
;   ADC channel is set by "adsel" variable which starts at 0. This reads
;   channel 0, which is "MAP". When the conversion complete interrupt is
;   requested the current value in "epcr" is averaged with the result of the
;   ADC in the ADC Data Register (ADR) and stored as current "epcr" value.
;   This is to smooth out ADC "jitter". The "adsel" variable is then
;   incremented to the next channel and the process repeats until all
;   channels are read, at which time, "adsel" is cleared to start the
;   sequence again.
;****************************************************************************

ADC_ISR:
     pshh              ; Push index register Hi byte on to stack
                       ;(Do this because processor does not stack H)
     clrh              ; Clear index register Hi byte
     lda     adsel     ; Load accumulator with value in ADC Channel Selector
     tax               ; Transfer value in accumulator to index register Lo
     lda     ADR       ; Load accumulator with value in ADC Data Register
                       ;(this also clears conversion complete and
                       ; interupt enable bit)
     add     MAP,x     ; Add ADR and MAP,x (Add the two values)
     rora              ; Rotate right through carry (Divide by 2)
     sta     MAP,x     ; Copy result to address MAP,x
     lda     adsel     ; Load accumulator with value in ADC Channel Selector
     inca              ; Increment ADC Channel Selector Variable
     and     #$07      ; "Logical and" accumulator with %0000111
                       ;(this clears "adsel" when incremented to decimal 8)
     sta     adsel     ; Copy new value to ADC Channel Selector Variable
     bset    adcc,inputs  ; Set "adcc" bit of "inputs" variable
     pulh                ; Pull value from stack to index register Hi byte
     rti                 ; Return from interrupt


;**************************************************************************
;==========================================================================
;- Dummy ISR vector - there just to keep the assembler happy
;==========================================================================
;**************************************************************************

Dummy:
     rti     ; Return from interrupt

;***************************************************************************
;
; ---------------------------- SUBROUTINES --------------------------------
;
;  - Ordered Table Search routine
;  - Linear Interpolation routine
;  - 32 x 16 divide routine
;  - Round after division routine
;  - 16 x 16 multiply routine
;  - 8 x 16 multiply and 16 by 100 divide
;  - Calculations for EPC duty factor at stall routine
;  - Calculations for EPC duty factor above stall routine
;  - Calculations for EPC duty factor during 1st to 2nd gear shift routine
;  - Calculations for EPC duty factor during 2nd to 3d gear shift routine
;  - Calculations for EPC duty factor during 3d to 4th gear shift routine
;  - Torque Converter Clutch application/release routine
;  - DFC enable/disable and Exhaust Brake application/release routine
;  - Park, Reverse, Neutral solonoid group routine
;  - Manual 1, Drive 1 solonoid group routine
;  - Manual 2 solonoid group routine
;  - Drive 2 solonoid group routine
;  - Drive 3 solonoid group routine
;  - Drive 4 solonoid group routine
;  - TCC release, DFC prohibit Exhaust brake off routine
;  - Cancel shift routine
;  - First gear variable group routine
;  - Second gear variable group routine
;  - Third gear variable group routine
;  - Forth gear variable group routine
;
;***************************************************************************


;***************************************************************************
;
; -------------------- Ordered Table Search Subroutine ---------------------
;
;  X is pointing to the start of the first value in the table
;  tmp1:2 initially hold the start of table address,
;  then they hold the bound values
;  tmp3 is the end of the table ("n" elements - 1)
;  tmp4 is the comparison value
;  tmp5 is the index result - if zero then comp value is less
;  than beginning of table, and if equal to "n" elements then it is
;  rail-ed at upper end
;
;***************************************************************************

ORD_TABLE_FIND:
     clr     tmp5     ; Clear tmp5 variable
     ldhx    tmp1     ; Load high part of index register with value in tmp1
     lda     ,x	      ; Load accumulator with low part of index register???
     sta     tmp1     ; Copy to tmp1 variable
     sta     tmp2     ; Copy to tmp2 variable

REENT:
     incx                    ; Increment low part of index register
     inc     tmp5            ; Increment tmp5 variable
     mov     tmp2,tmp1       ; Move value in tmp2 variable to tmp1 variable
     lda     ,x              ; Load accumulator with value in index reg Lo??
     sta     tmp2            ; Copy to tmp2 variable
     cmp     tmp4            ; Compare it with tmp4 variable
     bhi     GOT_ORD_NUM     ; If higher, branch to GOT_ORD_NUM lable
     lda     tmp5            ; Load accumulator with value in tmp5 variable
     cmp     tmp3            ; Compare it with value in tmp3 variable
     bne     REENT           ; If the Z bit of CCR is clesr, branch to REENT:

GOT_ORD_NUM:
     rts                     ; Return from subroutine


;****************************************************************************
;
; ------------------ Linear Interpolation - 2D Subroutine -------------------
;
; Graph Plot         Z2
;                   Y2
;               X
;               Y
;         X1
;         Y1
;            (y2 - y1)
;  Y = Y1 +  --------- * (x - x1)
;            (x2 - x1)
;
;   tmp1 = x1
;   tmp2 = x2
;   tmp3 = y1
;   tmp4 = y2
;   tmp5 = x
;   tmp6 = y
;***************************************************************************

LININTERP:
     clr     tmp7          ; Clear tmp7 variable (This is the negative slope
                           ; detection bit) (tmp7 = 0)
     mov     tmp3,tmp6     ; Move value in tmp3 variable to tmp6 variable
                           ; (Y1 to tmp6)

CHECK_LESS_THAN:
     lda     tmp5               ; Load accumulator with value in tmp5 variable
                                ; (x)
     cmp     tmp1               ; Compare it with value in tmp1 variable
                                ; (x1)
     bhi     CHECK_GREATER_THAN ; If higher, branch to CHECK_GREATER_THAN:
                                ; (X>X1)
     bra     DONE_WITH_INTERP	; Branch to DONE_WITH_INTERP: (else (Y=Y1))

CHECK_GREATER_THAN:
     lda     tmp5             ; Load accumulator with value in tmp5 variable
                              ; (x)
     cmp     tmp2             ; Compare it with value in tmp2 variable
                              ; (X2)
     blo     DO_INTERP        ; If lower, branch to DO_INTERP lable
                              ; (X<X2)
     mov     tmp4,tmp6        ; Move value in tmp4 variable to tmp6 variable
                              ; (Y2 to tmp6)
     bra     DONE_WITH_INTERP ; Branch to DONE_WITH_INTERP lable (else (Y=Y2))

DO_INTERP:
     mov     tmp3,tmp6        ; Move value in tmp3 variable to tmp6 variable
                              ; (Y1 to tmp6)
     lda     tmp2             ; Load accumulator with value in tmp2 variable
                              ; (X2)
     sub     tmp1             ; Subtract tmp1 from tmp2 (A=X2-X1)
     beq     DONE_WITH_INTERP ; If the Z bit of CCR is set, branch to
                              ;DONE_WITH_INTERP:  else (Y=Y1)
     psha                     ; Push value in accumulator to stack
                              ; (X2-X1)(stack 1)
     lda     tmp4             ; Load accumulator with value in tmp4 variable
                              ; (Y2)
     sub     tmp3             ; Subtract tmp3 from tmp4 (A=Y2-Y1)
     bcc     POSINTERP        ; If C bit of CCR is clear, branch to POSINTERP:
     nega                     ; Negate accumulator      ??????????
     inc     tmp7             ; Increment tmp7 variable (tmp7 = 1)

POSINTERP:
     psha                     ; Push value in accumulator to stack
                              ; (negated Y2-Y1) (stack 2)
     lda     tmp5             ; Load accumulator with value in tmp5 variable
                              ; (X)
     sub     tmp1             ; Subtract tmp1 from tmp5 (A=X-X1)
     beq     ZERO_SLOPE	      ; If the Z bit of CCR is set,
                              ; branch to ZERO_SLOPE lable  (Y=Y1)
     pulx                     ; Pull value from stack to index register Lo
                              ;(negated Y2-Y1) (stack 2)
     mul                      ; Multiply it by the value in the accumulator
                              ; A=(negated Y2-Y1)*(X-X1)
     pshx                     ; Push the index register L to the stack
                              ; (stack 2)
     pulh                     ; Pull this value to index register Hi(stack 2)
     pulx                     ; Pull the next value to index register Lo
                              ;(stack 1)
     div                      ; Divide A<-(H:A)/(X);H<-Remainder
     psha                     ; Push the value in the accumulator onto stack
                              ; (stack 1)
     lda     tmp7             ; Load accumulator with value in tmp7 variable
     bne     NEG_SLOPE        ; If the Z bit of CCR is clear,
                              ; branch to NEG_SLOPE: (Y=Y1)
     pula                     ; Pull value from stack to accumulator (stack 1)
     add     tmp3             ; Add it with value in tmp3 variable
     sta     tmp6             ; Copy it to tmp6 variable
     bra     DONE_WITH_INTERP ; Branch to  DONE_WITH_INTERP:

NEG_SLOPE:
     pula                     ; Pull value from stack to accumulator(stack 1)
     sta     tmp7             ; Copy to tmp7 variable
     lda     tmp3             ; Load accumulator with value in tmp3  Y1)
     sub     tmp7             ; Subtract tmp7 from tmp3
     sta     tmp6             ; Copy result to tmp6 variable
     bra     DONE_WITH_INTERP ; Branch to  DONE_WITH_INTERP:

ZERO_SLOPE:
        pula    ; Pull value from stack to accumulator (clean stack)(stack 2)
        pula    ; Pull value from stack to accumulator (clean stack)(stack 1)

DONE_WITH_INTERP:
        rts      ; Return from subroutine

;****************************************************************************
;
; ----------------- 32 x 16 Unsigned Divide Subroutine ---------------------
;
; This routine takes the 32-bit dividend stored in INTACC1.....INTACC1+3
; and divides it by the 16-bit divisor stored in INTACC2:INTACC2+1.
; The quotient replaces the dividend and the remainder replaces the divisor.
;
;***************************************************************************

UDVD32    EQU     *
*
DIVIDEND  EQU     INTACC1+2
DIVISOR   EQU     INTACC2
QUOTIENT  EQU     INTACC1
REMAINDER EQU     INTACC1
*
        PSHH                            ;save h-reg value
        PSHA                            ;save accumulator
        PSHX                            ;save x-reg value
        AIS     #-3                     ;reserve three bytes of temp storage
        LDA     #!32                    ;
        STA     3,SP                    ;loop counter for number of shifts
        LDA     DIVISOR                 ;get divisor msb
        STA     1,SP                    ;put divisor msb in working storage
        LDA     DIVISOR+1               ;get divisor lsb
        STA     2,SP                    ;put divisor lsb in working storage

****************************************************************************
*     Shift all four bytes of dividend 16 bits to the right and clear
*     both bytes of the temporary remainder location
****************************************************************************

        MOV     DIVIDEND+1,DIVIDEND+3   ;shift dividend lsb
        MOV     DIVIDEND,DIVIDEND+2     ;shift 2nd byte of dividend
        MOV     DIVIDEND-1,DIVIDEND+1   ;shift 3rd byte of dividend
        MOV     DIVIDEND-2,DIVIDEND     ;shift dividend msb
        CLR     REMAINDER               ;zero remainder msb
        CLR     REMAINDER+1             ;zero remainder lsb

****************************************************************************
*     Shift each byte of dividend and remainder one bit to the left
****************************************************************************

SHFTLP  LDA     REMAINDER               ;get remainder msb
        ROLA                            ;shift remainder msb into carry
        ROL     DIVIDEND+3              ;shift dividend lsb
        ROL     DIVIDEND+2              ;shift 2nd byte of dividend
        ROL     DIVIDEND+1              ;shift 3rd byte of dividend
        ROL     DIVIDEND                ;shift dividend msb
        ROL     REMAINDER+1             ;shift remainder lsb
        ROL     REMAINDER               ;shift remainder msb

*****************************************************************************
*     Subtract both bytes of the divisor from the remainder
*****************************************************************************

        LDA     REMAINDER+1          ;get remainder lsb
        SUB     2,SP                 ;subtract divisor lsb from remainder lsb
        STA     REMAINDER+1          ;store new remainder lsb
        LDA     REMAINDER            ;get remainder msb
        SBC     1,SP                 ;subtract divisor msb from remainder msb
        STA     REMAINDER            ;store new remainder msb
        LDA     DIVIDEND+3           ;get low byte of dividend/quotient
        SBC     #0                   ;dividend low bit holds subtract carry
        STA     DIVIDEND+3           ;store low byte of dividend/quotient

*****************************************************************************
*     Check dividend/quotient lsb. If clear, set lsb of quotient to indicate
*     successful subraction, else add both bytes of divisor back to remainder
*****************************************************************************

        BRCLR   0,DIVIDEND+3,SETLSB     ;check for a carry from subtraction
                                        ;and add divisor to remainder if set
        LDA     REMAINDER+1             ;get remainder lsb
        ADD     2,SP                    ;add divisor lsb to remainder lsb
        STA     REMAINDER+1             ;store remainder lsb
        LDA     REMAINDER               ;get remainder msb
        ADC     1,SP                    ;add divisor msb to remainder msb
        STA     REMAINDER               ;store remainder msb
        LDA     DIVIDEND+3              ;get low byte of dividend
        ADC     #0                      ;add carry to low bit of dividend
        STA     DIVIDEND+3              ;store low byte of dividend
        BRA     DECRMT                  ;do next shift and subtract

SETLSB  BSET    0,DIVIDEND+3            ;set lsb of quotient to indicate
                                        ;successive subtraction
DECRMT  DBNZ    3,SP,SHFTLP             ;decrement loop counter and do next
                                        ;shift

*****************************************************************************
*     Move 32-bit dividend into INTACC1.....INTACC1+3 and put 16-bit
*     remainder in INTACC2:INTACC2+1
*****************************************************************************

        LDA     REMAINDER               ;get remainder msb
        STA     1,SP                    ;temporarily store remainder msb
        LDA     REMAINDER+1             ;get remainder lsb
        STA     2,SP                    ;temporarily store remainder lsb
        MOV     DIVIDEND,QUOTIENT       ;
        MOV     DIVIDEND+1,QUOTIENT+1   ;shift all four bytes of quotient
        MOV     DIVIDEND+2,QUOTIENT+2   ; 16 bits to the left
        MOV     DIVIDEND+3,QUOTIENT+3   ;
        LDA     1,SP                    ;get final remainder msb
        STA     INTACC2                 ;store final remainder msb
        LDA     2,SP                    ;get final remainder lsb
        STA     INTACC2+1               ;store final remainder lsb

*****************************************************************************
*     Deallocate local storage, restore register values, and return from
*     subroutine
*****************************************************************************

        AIS     #3                      ;deallocate temporary storage
        PULX                            ;restore x-reg value
        PULA                            ;restore accumulator value
        PULH                            ;restore h-reg value
        RTS                             ;return

*****************************************************************************


;****************************************************************************
; ----------  ----- ROUND after div (unsigned) Subroutine -------------------
;
;  1)  check for div overflow (carry set), rail result if detected
;  2)  if (remainder * 2) > divisor then     ; was remainder > (divisor / 2)
;  2a)    increment result, rail if over-flow
;
;****************************************************************************

DIVROUND:
     bcs     DIVROUND0     ; If C bit of CCR is set, branch to DIVROUND0:
                           ; (div overflow? yes, branch)
     stx     local_tmp     ; Copy value in index register Lo byte to
                           ; local_tmp variable (divisor)
     pshh                  ; Push value in index register Hi byte onto
                           ; stack (retrieve remainder)
     pulx                  ; Pull value on stack to index register Lo byte
     lslx                  ; Logical shift left index register lo byte (* 2)
     bcs     DIVROUND2     ; If C bit of CCR is set, branch to DIVROUND2:
                           ;(over-flow on left-shift, (remainder * 2) > $FF)
     cpx     local_tmp     ; Compare value in local_tmp variable with value
                           ; in index register Lo byte
                           ;(compare (remainder * 2) to divisor)
     blo     DIVROUND1     ; If lower, branch to DIVROUND1:


DIVROUND2:
     inca                   ; Increment accumulator (round-up result)
     bne      DIVROUND1     ; If Z bit of CCR is clear, branch to DIVROUND1:
                            ; (result roll over? no, branch)


DIVROUND0:
     lda     #$FF     ; Load accumulator with decimal 255 (rail result)


DIVROUND1:
     rts              ; return from subroutine


;****************************************************************************
;
; ------------------- 16 x 16 Unsigned Multiply Subroutine -----------------
;
;     tmp8...tmp5 = tmp4:tmp3 * tmp2:tmp1
;
;               tmp3*tmp1
;   +      tmp4*tmp1
;   +      tmp3*tmp2
;   + tmp4*tmp2
;   = ===================
;     tmp8 tmp7 tmp6 tmp5
;
;****************************************************************************

UMUL32:
     lda     tmp1        ; Load accumulator with value in tmp1 variable
     ldx     tmp3        ; Load index register Lo byte with value in tmp3
     mul                 ; Multiply X:A<-(X)*(A)
     sta     tmp5        ; Ccopy result to tmp5
     stx     tmp6        ; Copy value in index register Lo byte to tmp6
;
     lda     tmp2        ; Load accumulator with value in tmp2
     ldx     tmp4        ; Load index register Lo byte with value in tmp4
     mul                 ; Multiply X:A<-(X)*(A)
     sta     tmp7        ; Copy result to tmp7
     stx     tmp8        ; Copy value in index register Lo byte to tmp8
;
     lda     tmp1        ; Load accumulator with value in tmp1
     ldx     tmp4        ; Load index register Lo byte with value in tmp4
     mul                 ; Multiply X:A<-(X)*(A)
     add     tmp6        ; Add without carry, A<-(A)+(M)
     sta     tmp6        ; Copy result to tmp6
     txa                 ; Transfer value in index register Lo byte
                         ; to accumulator
     adc     tmp7        ; Add with carry, A<-(A)+(M)+(C)
     sta     tmp7        ; Copy result to tmp7
     bcc     UMUL32a     ; If C bit of CCR is clear, branch to UMUL32a:
     inc     tmp8        ; Increment value in tmp8


UMUL32a:
     lda     tmp2        ; Load accumulator with value in tmp2
     ldx     tmp3        ; Load index register Lo byte with value in tmp3
     mul                 ; Multiply X:A<-(X)*(A)
     add     tmp6        ; Add without carry, A<-(A)+(M)
     sta     tmp6        ; Copy result to tmp6
     txa                 ; Transfer value in index register Lo byte
                         ; to accumulator
     adc     tmp7        ; Add with carry, A<-(A)+(M)+(C)
     sta     tmp7        ; Copy result to tmp7
     bcc     UMUL32b     ; If C bit of CCR is clear, branch to UMUL32b:
     inc     tmp8        ; increment value in tmp8 variable


UMUL32b:
      rts                ; return from subroutine

;****************************************************************************
; Multiply then divide.
;****************************************************************************

uMulAndDiv:

;****************************************************************************
; 8 x 16 Multiply
;
; 8-bit value in Accumulator, 16-bit value in tmp11-12, result overwrites
; 16-bit input.  Assumes result cannot overflow.
;
;  tmp12:tmp11<-tmp12:tmp11*(A)
;
****************************************************************************

uMul16:
      psha              ; Push value in accumulator to stack
                        ;(Save multiplier)
      ldx     tmp11     ; Load index register Lo byte with value in "tmp11"
                        ;(LSB of multiplicand)
      mul               ; Multiply (X:A)<-(X)*(A)
      sta     tmp11     ; Copy value in accumulator to "tmp11"
                        ;(LSB of result stored)
      pula              ; Pull value from stack to acumulator
                        ;(Pop off multiplier)
      pshx              ; Push value in index register Lo byyte to stack
                        ;(Carry on stack)
      ldx     tmp12     ; Load index register Lo byte with value in "tmp12"
                        ;(MSB of multiplicand)
      mul               ; Multiply (X:A)<-(X)*(A)
      add     1,SP      ; Add (A)<-(A)+(SP)
                        ;(Add in carry from LSB)
      sta     tmp12     ; Copy value in accumulator to "tmp12"
                        ;(MSB of result)
      pula              ; Pull value in stack to accumulator
                        ;(Clear the stack)

;****************************************************************************
; 16-bit divide by 100T
;
; 16-bit value in tmp11-12 is divided by 100T.  Result is left in tmp11-12.
;
;  tmp12:tmp11<-tmp12:tmp11/100
;
;****************************************************************************

uDivBy100:
      clrh              ; Clear index register Hi byte
      lda     tmp12     ; Load accumulator with value in "tmp12"
                        ;(MSB of dividend)
      ldx     #$64      ; Load index register Lo byte with decimal 100
                        ;(Divisor)
      div               ; Divide (A)<-(H:A)/(X)
      sta     tmp12     ; Copy value in accumulator to "tmp12"
                        ;(MSB of quotient)
      lda     tmp11     ; Load accumulator with value in "tmp11"
                        ;(LSB of dividend)
      div               ; Divide (A)<-(H:A)/(X)
      sta     tmp11     ; Copy value in accumulator to "tmp11
                        ;(LSB of quotient)

;****************************************************************************
; - See if we need to round up the quotient.
;****************************************************************************
      pshh                         ; Push value in index register Hi byte
                                   ; to stack
      pula                         ; Pull value from stack to accumulator
                                   ;(H)->(A)(Remainder in A)
      cmp     #$32                 ; Compare value in accumulator with
                                   ; decimal 50(Half of the divisor)
      ble     uDivRoundingDone     ; If less than or equal, branch to
                                   ; uDivRoundingDone:
      inc     tmp11                ; Increment value in "tmp11"
      bcc     uDivRoundingDone     ; If carry bit is clear, branch to
                                   ; uDivRoundingDone:
      inc     tmp12                ; Increment value in "tmp12"

uDivRoundingDone:
      rts                          ; Return from subroutine

;****************************************************************************
;
; "TO"(Torque Output)3-D Table Lookup:
;
;  This is used to determine value of "TOcurr", based on RPM and MAP. The
;  "TOcurr" value is used to determine EPC pulse width for steady state
;  line pressure while above ~4MPH.
;  The table looks like:
;
;     105 +....+....+....+....+....+....+....+
;         ....................................
;      00 +....+....+....+....+....+....+....+
;                    ...
;  KPA                ...
;                        ...
;      35 +....+....+....+....+....+....+....+
;         5    15   25   35   45   55   65   75 RPM/20
;
;
; Steps:
;  1) Find the bracketing KPA positions via ORD_TABLE_FIND, put index in
;      tmp8 and bounding values in tmp9(kpa1) and tmp10(kpa2)
;  2) Find the bracketing RPM positions via ORD_TABLE_FIND, store index in
;      tmp11 and bounding values in tmp13(rpm1) and tmp14(rpm2)
;  3) Using the TO table, find the table TO values for tmp15=TO(kpa1,rpm1),
;      tmp16=TO(kpa1,rpm2), tmp17 = TO(kpa2,rpm1), and tmp18 = TO(kpa2,rpm2)
;  4) Find the interpolated TO value at the lower KPA range :
;      x1=rpm1, x2=rpm2, y1=TO(kpa1,rpm1), y2=TO(kpa1,rpm2) - put in tmp19
;  5) Find the interpolated TO value at the upper KPA range :
;      x1=f1, x2=rpm2, y1=TO(kpa2,rpm1), y2=TO(kpa2,rpm2) - put in tmp11
;  6) Find the final TO value using the two interpolated TO values:
;      x1=kpa1, x2=kpa2, y1=TO_FROM_STEP_4, y2=TO_FROM_STEP_5
;
;****************************************************************************

MOVE_CALCS:
     ldhx    #KPA_range         ; Load index register with value in
                                ; KPA_range table
     sthx    tmp1               ; Copy value to tmp1 variable
     lda     #$07               ; Load accumulator with decimal 7
     sta     tmp3               ; Copy to tmp3 variable
     lda     kpa                ; Load accumulator with value in MAP value
                                ; in units of KPa variable
     sta     tmp4               ; Copy to tmp4 variable
     jsr     ORD_TABLE_FIND     ; Jump to subroutine ORD_TABLE_FIND,
                                ; (result in tmp5)
     lda     tmp1               ; Load accumulator with value in tmp1
     lda     tmp2               ; Load accumulator with value in tmp2
     mov     tmp5,tmp8          ; move value from tmp5 to tmp8 (Index)
     mov     tmp1,tmp9          ; move value from tmp1 to tmp9 (X1)
     mov     tmp2,tmp10	        ; move value from tmp2 to tmp10 (X2)

;TO_STEP_2:
     ldhx    #RPM_range         ; Load index register with value from
                                ; RPM_range table
     sthx    tmp1               ; Copy value to tmp1 variable
     lda     #$07               ; Load accumulator with decimal 7
     sta     tmp3               ; Copy to tmp3 variable
     lda     rpm                ; Load accumulator with value in RPM/20
     sta     tmp4               ; Copy to tmp4 variable
     jsr     ORD_TABLE_FIND     ; Jump to subroutine ORD_TABLE_FIND,
                                ; result in tmp5
     mov     tmp5,tmp11	        ; Move value from tmp5 to tmp11 (Index)
     mov     tmp1,tmp13	        ; Move value from tmp1 to tmp13 (X1)
     mov     tmp2,tmp14	        ; Move value from tmp2 to tmp14 (X2)

;TO_STEP_3:
     clrh               ; Clear high part of index register
     lda     #$08       ; Load accumulator with decimal 8
     psha               ; Push this value onto stack
     pulx               ; Pull this value onto low part of index register
     lda     tmp8       ; Load accumulator with value in tmp8 variable
     deca               ; Decrement accumulator
     mul                ; Multiply value in index reg Lo by the accumulator
     add     tmp11	; Add to value in tmp11 variable
     deca               ; Decrement accumulator
     tax                ; Copy value to index register Lo byte
     lda     TO_table,x ; Load accumulator with value in X bin of TO_table
     sta     tmp15	; Copy value to tmp15 variable
     incx               ; Increment low part of index register
     lda     TO_table,x ; Load accumulator with value in X bin of TO_table
     sta     tmp16	; Copy value to tmp16 variable
     lda     #$08       ; Load accumulator with decimal 8
     psha               ; Push this value onto stack
     pulx               ; Pull this value into index registerLo byte
     lda     tmp8       ; Copy value in accumulator to tmp8 variable
     mul                ; Multiply value in index register Lo by accumulator
     add     tmp11	; Add to value in tmp11 variable
     deca               ; Decrement accumulator
     tax                ; Copy value to index register L byte
     lda     TO_table,x ; Load accumulator with value in X bin of TO_table
     sta     tmp17	; Copy value to tmp17 variable
     incx               ; Increment low part of index register
     lda     TO_table,x ; Load accumulator with value in X bin of TO_table
     sta     tmp18	; Copy value to tmp18 variable

;TO_STEP_4:
     mov     tmp13,tmp1	; Move value from tmp13 variable to tmp1 variable
     mov     tmp14,tmp2	; Move value from tmp14 variable to tmp2 variable
     mov     tmp15,tmp3	; Move value from tmp15 variable to tmp3 variable
     mov     tmp16,tmp4	; Move value from tmp16 variable to tmp4 variable
     mov     rpm,tmp5	; Move Throttle Position Sensor ADC Raw Reading
                        ; to tmp5 variable
     jsr     lininterp	; Jump to subroutine lininterp (result in tmp6)
     mov     tmp6,tmp19	; Move value from tmp6 variable to tmp19 variable

;TO_STEP_5:
     mov     tmp13,tmp1	; Move value from tmp13 variable to tmp1 variable
     mov     tmp14,tmp2	; Move value from tmp14 variable to tmp2 variable
     mov     tmp17,tmp3	; Move value from tmp17 variable to tmp3 variable
     mov     tmp18,tmp4	; Move value from tmp18 variable to tmp4 variable
     mov     rpm,tmp5	; Move RPM/20 to tmp5 variable
     jsr     lininterp	; Jump to subroutine lininterp (result in tmp6)
     mov     tmp6,tmp11	; Move value from tmp6 variable to tmp11 variable

;TO_STEP_6:
     mov     tmp9,tmp1	 ; Move value from tmp9 variable to tmp1 variable
     mov     tmp10,tmp2	 ; Move value from tmp10 variable to tmp2 variable
     mov     tmp19,tmp3	 ; Move value from tmp19 variable to tmp3 variable
     mov     tmp11,tmp4	 ; Move value from tmp11 variable to tmp4 variable
     mov     kpa,tmp5	 ; Move value in MAP value in units of KPa to tmp5
     lda     kpa         ; Load accumulator with value in MAP value in KPa
     jsr     lininterp	 ; Jump to subroutine lininterp (result in tmp6)
     rts                 ; Return from subroutine

;****************************************************************************
;
; --------------- EPC pulse width calculation subroutines ------------------
;
;   The EPC pulse width is a linear interpolated value from the EPC tables
;   (8 points) which are placed at different TPSp values in the
;   TPS_range table.
;   The EPC tables are:
;   - EPC_stall,  This is the table used at vehicle speeds below ~4MPH
;   - EPC_12,  This is the table used for First to Second gear upshift
;   - EPC_23,  This is the table used for Second to Third gear upshift
;   - EPC_34,  This is the table used for Third to Forth gear upshift
;   - EPC_M1,  This is the table used with MLPS in M1
;
; Method:
;
; 1) Perform ordered table search of TPS_range (using TPSp variable) to
;    determine which bin.
; 2) Perform linear interpolation of EPC table to get interpolated EPC
;    pulse width
;
;***************************************************************************

STALL_CALCS:
     ldhx    #TPS_range	      ; Load Index register H:X with address of
                              ; first two bytes of TPS_range table
     sthx    tmp1             ; Copy values into tmp1:tmp2 variables
     lda     #$07             ; Load accumulator with decimal 7
     sta     tmp3             ; Copy value into tmp3 variable
     lda     TPSp             ; Load accumulator with current TPS percent
     sta     tmp4             ; Copy value into tmp4 variable
     jsr     ORD_TABLE_FIND   ; jump to subroutine ORD_TABLE_FIND
                              ; (result in tmp5)
     clrh                     ; Clear index register hi byte
     lda     tmp5             ; Load accumulator with result from
                              ; ORD_TABLE_FIND (tmp5)
     tax                      ; Copy it to index register Lo byte
     lda     EPC_stall,x      ; Load the accumulator with the value in
                              ; EPC_stall table, offset in index register
                              ; Lo byte(upper bin)
     sta     tmp4             ; Copy it to the tmp4 variable
     decx                     ; Decrement index reg Lo byte (move down 1 bin)
     lda     EPC_stall,x      ; Load the accumulator with the value in
                              ; EPC_stall table, offset in index register
                              ; Lo byte(new bin value)
     sta     tmp3             ; Copy it to the tmp3 variable
     mov     TPSp,tmp5        ; Move current TPS percent to tmp5 variable
     jsr     lininterp        ; Jump to Lininterp subroutine
                              ; (result in temp6)
     rts                      ; Return from subroutine

SHIFT12_CALCS:
     ldhx    #TPS_range	      ; Load Index register H:X with address of
                              ; first two bytes of TPS_range table
     sthx    tmp1             ; Copy values into tmp1:tmp2 variables
     lda     #$7              ; Load accumulator with decimal 7
     sta     tmp3             ; Copy value into tmp3 variable
     lda     TPSp             ; Load accumulator with current TPS percent
     sta     tmp4             ; Copy value into tmp4 variable
     jsr     ORD_TABLE_FIND   ; jump to subroutine ORD_TABLE_FIND
                              ; (result in tmp5)
     clrh                     ; Clear index register hi byte
     lda     tmp5             ; Load accumulator with result from
                              ; ORD_TABLE_FIND (tmp5)
     tax                      ; Copy it to index register Lo byte
     lda     EPC_12,x         ; Load the accumulator with the value in
                              ; EPC_12 table, offset in index register
                              ; Lo byte(upper bin)
     sta     tmp4             ; Copy it to the tmp4 variable
     decx                     ; Decrement index reg Lo byte (move down 1 bin)
     lda     EPC_12,x         ; Load the accumulator with the value in
                              ; EPC_12 table, offset in index register
                              ; Lo byte(new bin value)
     sta     tmp3             ; Copy it to the tmp3 variable
     mov     TPSp,tmp5        ; Move current TPS percent to tmp5 variable
     jsr     lininterp        ; Jump to Lininterp subroutine
                              ; (result in temp6)
     rts                      ; Return from subroutine

SHIFT23_CALCS:
     ldhx    #TPS_range	      ; Load Index register H:X with address of
                              ; first two bytes of TPS_range table
     sthx    tmp1             ; Copy values into tmp1:tmp2 variables
     lda     #$7              ; Load accumulator with decimal 7
     sta     tmp3             ; Copy value into tmp3 variable
     lda     TPSp             ; Load accumulator with current TPS percent
     sta     tmp4             ; Copy value into tmp4 variable
     jsr     ORD_TABLE_FIND   ; jump to subroutine ORD_TABLE_FIND
                              ; (result in tmp5)
     clrh                     ; Clear index register hi byte
     lda     tmp5             ; Load accumulator with result from
                              ; ORD_TABLE_FIND (tmp5)
     tax                      ; Copy it to index register Lo byte
     lda     EPC_23,x         ; Load the accumulator with the value in
                              ; EPC_23 table, offset in index register
                              ; Lo byte(upper bin)
     sta     tmp4             ; Copy it to the tmp4 variable
     decx                     ; Decrement index reg Lo byte (move down 1 bin)
     lda     EPC_23,x         ; Load the accumulator with the value in
                              ; EPC_23 table, offset in index register
                              ; Lo byte(new bin value)
     sta     tmp3             ; Copy it to the tmp3 variable
     mov     TPSp,tmp5        ; Move current TPS percent to tmp5 variable
     jsr     lininterp        ; Jump to Lininterp subroutine
                              ; (result in temp6)
     rts                      ; Return from subroutine

SHIFT34_CALCS:
     ldhx    #TPS_range	      ; Load Index register H:X with address of
                              ; first two bytes of TPS_range table
     sthx    tmp1             ; Copy values into tmp1:tmp2 variables
     lda     #$7              ; Load accumulator with decimal 7
     sta     tmp3             ; Copy value into tmp3 variable
     lda     TPSp             ; Load accumulator with current TPS percent
     sta     tmp4             ; Copy value into tmp4 variable
     jsr     ORD_TABLE_FIND   ; jump to subroutine ORD_TABLE_FIND
                              ; (result in tmp5)
     clrh                     ; Clear index register hi byte
     lda     tmp5             ; Load accumulator with result from
                              ; ORD_TABLE_FIND (tmp5)
     tax                      ; Copy it to index register Lo byte
     lda     EPC_34,x         ; Load the accumulator with the value in
                              ; EPC_34 table, offset in index register
                              ; Lo byte(upper bin)
     sta     tmp4             ; Copy it to the tmp4 variable
     decx                     ; Decrement index reg Lo byte (move down 1 bin)
     lda     EPC_34,x         ; Load the accumulator with the value in
                              ; EPC_34 table, offset in index register
                              ; Lo byte(new bin value)
     sta     tmp3             ; Copy it to the tmp3 variable
     mov     TPSp,tmp5        ; Move current TPS percent to tmp5 variable
     jsr     lininterp        ; Jump to Lininterp subroutine
                              ; (result in temp6)
     rts                      ; Return from subroutine

M1_CALCS:
     ldhx    #TPS_range	      ; Load Index register H:X with address of
                              ; first two bytes of TPS_range table
     sthx    tmp1             ; Copy values into tmp1:tmp2 variables
     lda     #$07             ; Load accumulator with decimal 7
     sta     tmp3             ; Copy value into tmp3 variable
     lda     TPSp             ; Load accumulator with current TPS percent
     sta     tmp4             ; Copy value into tmp4 variable
     jsr     ORD_TABLE_FIND   ; jump to subroutine ORD_TABLE_FIND
                              ; (result in tmp5)
     clrh                     ; Clear index register hi byte
     lda     tmp5             ; Load accumulator with result from
                              ; ORD_TABLE_FIND (tmp5)
     tax                      ; Copy it to index register Lo byte
     lda     EPC_M1,x         ; Load the accumulator with the value in
                              ; EPC_M1 table, offset in index register
                              ; Lo byte(upper bin)
     sta     tmp4             ; Copy it to the tmp4 variable
     decx                     ; Decrement index reg Lo byte (move down 1 bin)
     lda     EPC_M1,x         ; Load the accumulator with the value in
                              ; EPC_M1 table, offset in index register
                              ; Lo byte(new bin value)
     sta     tmp3             ; Copy it to the tmp3 variable
     mov     TPSp,tmp5        ; Move current TPS percent to tmp5 variable
     jsr     lininterp        ; Jump to Lininterp subroutine
                              ; (result in temp6)
     rts                      ; Return from subroutine

;****************************************************************************
; --------------------------- TCC Subroutine --------------------------------
;****************************************************************************

TCC_CHKOFF:

;****************************************************************************
; - If the TCC is currently applied, or if application is in progress, check
;   to see if it still has permisives to remain applied or in progress.
;   If it does not, or if it is commanded off by a signal from the "TCCrel"
;   contacts of the joy stick, release it, otherwise, leave it applied or in
;   progress. If it is not applied, see if we have permissives to apply it,
;   and do so if we have a signal from the "TCCapp" contacts of the joy
;   stick. If the TCC application process is in progress, branch to the
;   appropriate section. Otherwise, leave it released.
;****************************************************************************

     brclr   tccrel,porta,TCC_CHKREL  ; If "tccrel" bit of Port A is clear,
                                      ; branch to TCC_CHKREL:
                                      ;(TCC release commanded)
     brclr   RPMtcc,trans,TCC_CHKREL  ; If "RPMtcc" bit of "trans" variable
                                      ; is clear, branch to TCC_CHKREL:
                                      ;(Min RPM permissive not met)
     brclr   D1D2,trans2,TCC_CHKON    ; If "D1D2"bit of "trans2 variable is
                                      ; clear, branch to TCC_CHKON:
                                      ;(not in Drive 1 or Drive 2)
     brclr   ClsThrt,trans,TCC_CHKON  ; If "ClsThrt" bit of "trans"
                                      ; variable is clear, branch to
                                      ; TCC_CHKON:
                                      ;(throttle not closed)
     bra     TCC_CHKREL               ; Branch to TCC_CHKREL:
                                      ;(in D1 or D2 and closed throttle)

TCC_CHKON:
     brset   TCprog,trans2,TCC_APP      ; If "SSprog" bit of "trans2"
                                        ; variable is set, branch to
                                        ; TCC_APP:
                                        ;(TCC apply in progress)
     brset   TCCon,trans2,TCC_CHK_DONE  ; If "TCCon" bit of "trans2" variable
                                        ; is set, branch to TCC_CHK_DONE:
                                        ;(TCC applied, no longer in progress,
                                        ; skip over)
     brclr   tccapp,porta,SHIFT_CHK     ; If "tccapp" bit of Port A is
                                        ; clear, branch to SHIFT_CHK:
                                        ;(TCC apply commanded)
     bra     TCC_CHK_DONE               ; Branch to TCC_CHK_DONE:
                                        ;(No change for TCC, skip over)

TCC_CHKREL:
     brclr   TCCon,trans2,TCC_CHK_DONE  ; If "TCCon" bit of "trans2" variable
                                        ; is clear, branch to TCC_CHK_DONE:
                                        ;(TCC not applied, skip over)


;****************************************************************************
; - Either the TCC has been commanded off by a signal from the TCC release
;   contacts on the joy stick, or we no longer have permissives for TCC
;   application. Release the TCC and clear the flag. Disable Decel Fuel
;   Cut permissive for the engine and clear the flag. De-energise the
;   Exhaust Brake control air, and engine secondary air solonoids, and clear
;   the flag. Clear the TCC apply in progress flag.
;****************************************************************************

TCC_REL:
     bset    TCC,portc         ; Set "TCC" bit of Port C (TCC off)
     bclr    TCCon,trans2      ; Clear "TCCon" bit of "trans2" variable
     bclr    EPCrTCC,shift     ; Clear "EPCrTCC" bit of "shift" variable
     bclr    EPChTCC,shift     ; Clear "EPChTCC" bit of "shift" variable
     bclr    TCprog,trans2     ; Clear "TCprog" bit of "trans2" variable
     bclr    DFCper,portd      ; Clear "DFCper" bit of Port D (DFC disabled)
     bclr    DFCon,trans2      ; Clear "DFCon" bit of "trans2" variable
     bset    ExhBrk,portc      ; Set "ExhBrk" bit of Port C (Exh Brk off)
     bclr    Brkon,trans2      ; Clear "Brkon" bit of "trans2" variable
     bclr    Brkdel,shift      ; Clear "Brkdel" bit of "shift" variable
     bra     TCC_CHK_DONE      ; Branch to TCC_CHK_DONE:



;****************************************************************************
;   We have permissives for Torque Converter Clutch application, and an
;   input signal from the TCC apply contacts on the joy stick, so the
;   Torque Converter Clutch solonoid can be energised.
;
;   - Check to see if a shift is in progress. If it is, check to see if the
;     pressure rise timer has timed out. If both connditions are met, apply
;     the cluctch, set the pressure for TCC apply and let the shift sequence
;     continue. If both conditions are not met, procede with the TCC
;     application process.
;   - Set the "EPCrTCC" bit of "shift" variable, start the pressure rise
;     count down timer and set the "TCprog" bit of "trans2".
;   - When the pressure rise counter zeros, clear the "EPCrTCC" bit and set
;     the "EPChTCC" bit of "shift" variable. Change solonoid state and start
;     the pressure hold timer.
;   - When the pressure hold timer zeros, clear the "EPChTCC" bit of "shift"
;     variable and clear the "TCprog" bit of "trans2".
;   - EPC pressure is set in either the MAN2_EPC, or DRIVE_EPC sections.
;****************************************************************************

SHIFT_CHK:
     brclr   SSprog,trans2,TCC_APP    ; If "SSprog" bit of "trans2"
                                      ; variable is clear, branch to
                                      ; TCC_APP:
                                      ;(gear change not in progress)
     brset   EPCrSS,shift,TCC_APP     ; If "EPCrSS" bit of "shift"
                                      ; variable is clear, branch to
                                      ; TCC_APP:
                                      ;(gear change not in progress but
                                      ; still on pressure rise)
     bclr    TCC,portc                ; Clear "TCC" bit of Port C (TCC on)
     bclr    EPCrTCC,shift            ; Clear "EPCrTCC" bit of "shift"
     bclr    EPChTCC,shift            ; Clear "EPChTCC" bit of "shift"
     bclr    TCprog,trans2            ; Clear "TCprog" bit of "trans2"
     bset    tccon,trans2             ; Set "tccon" bit of "trans2"
     clr     dfSel                    ; Clear "dfSel" variable
     bset    selTCC,dfSel             ; Set "selTCC" bit of "dfSel"
                                      ;(EPC set for TCC apply)
     bra     TCC_CHK_DONE             ; Branch to TCC_CHK_DONE:

TCC_APP:
     brset   EPCrTCC,shift,TCC_APP_A  ; If "EPCrTCC" bit of "shift" variable
                                      ; is set, branch to TCC_APP_A:
                                      ;(TCC/EPC rise in progress)
     brset   EPChTCC,shift,TCC_APP_B  ; If "EPChTCC" bit of "shift" variable
                                      ;is set, branch to TCC_APP_B:
     bset    EPCrTCC,shift     ; Set "EPCrTCC" bit of "shift" variable
     lda     EPC_rise          ; Load accumulator with value in "EPC_rise"
     sta     TIMcnt            ; Copy to "TIMcnt"
     bset    TCprog,trans2     ; Set "TCprog" bit of "trans2" variable
     clr     dfSel             ; Clear "dfSel" variable
     bset    selTCC,dfSel      ; Set "selTCC" bit of "dfSel"
                               ;(EPC set for TCC apply)
     bra     TCC_CHK_DONE      ; Branch to TCC_CHK_DONE:

TCC_APP_A:
     lda     TIMcnt            ; Load accumulator with value in "TIMcnt"
                               ; variable
     bne     TCC_CHK_DONE        ; If Z bit of CCR is clear, branch to
                               ; TCC_CHK_DONE:
     bclr    EPCrTCC,shift     ; Clear "EPCrTCC" bit of "shift" variable
     bset    EPChTCC,shift     ; Set "EPChTCC" bit of "shift" variable
     bclr    TCC,portc         ; Clear "TCC" bit of Port C (TCC on)
     bset    tccon,trans2      ; Set "tccon" bit of "trans2" variable
     lda     EPC_hold          ; Load accumulator with value in "EPC_hold"
     sta     TIMcnt            ; Copy to "TIMcnt"
     bra     TCC_CHK_DONE      ; Branch to TCC_CHK_DONE:

TCC_APP_B:
     lda     TIMcnt            ; Load accumulator with value in "TIMcnt" var
     bne     TCC_CHK_DONE      ; If Z bit of CCR is clear, branch to
                               ; TCC_CHK_DONE:
     bclr    EPChTCC,shift     ; Clear "EPChTCC" bit of "shift" variable
     bclr    TCprog,trans2     ; Clear "TCprog" bit of "trans2" variable
     clr     dfSel             ; Clear "dfSel" variable
     bset    selTO,dfSel       ; Set "selTO" bit of "dfSel"
                               ;(EPC set for torque output)

TCC_CHK_DONE:
     rts                       ; Return from subroutine


;****************************************************************************
; ----------------- DFC Enable/Exhaust Brake subroutine ---------------------
;****************************************************************************

BRK_CHKOFF:

;****************************************************************************
; - Check to see if if permissives for Decel Fuel Cut Permitted, or exhaust
;   brake application are met. If they are not, or if they have been
;   commanded off by a signal from the "DFCdis" contacts of the DFC switch,
;   disable the DFC and release the Exhaust Brake. Otherwise,leave them
;   permitted/applied. If the permissives are met, and DFC is not currently
;   permitted, or if the Exhaust Brake is not applied, see if we have a
;   signal from the "DFCen" contacts of the DFC switch. Otherwise, leave
;   the DFC disabled and the Exhaust Brake released. If we enabled DFC,
;   start the Exhaust Brake delay timer to see if the "DFCen" contacts of
;   the DFC switch will remain closed for the set time of the "ExhBrkdel"
;   variable. If they do, apply the exhaust brake. Otherwise, leave it
;   released.
;****************************************************************************

     brclr   DFCdis,porta,DFC_DIS     ; If "DFCdis" bit of Port A is clear,
                                      ; branch to DFC_DIS:
                                      ;(DFC prohibit and exhaust brake
                                      ; release commanded)
     brclr   TCCon,trans2,DFC_DIS     ; If "TCCon" bit of "trans2" variable
                                      ; is clear, branch to DFC_DIS:
                                      ;(TCC not applied)
     brclr   RPMtcc,trans,DFC_DIS     ; If "RPMtcc" bit of "trans" variable
                                      ; is clear, branch to DFC_DIS:
                                      ;(Engine speed too low)
     brclr   ClsThrt,trans,DFC_DIS    ; If "ClsThrt" bit of "trans" variable
                                      ; is clear, branch to DFC_DIS:
                                      ;(Not at closed throttle)
     brset   D1D2,trans2,DFC_DIS      ; If "D1D2"bit of "trans2" variable is
                                      ; set, branch to DFC_DIS:
                                      ;(In D2 or D1, no engine braking)
     brclr   PSIbrk,trans,BRK_REL     ; If "PSIbrk" bit of "trans" variable
                                      ; is clear, branch to BRK_REL:
                                      ;(Exhaust pressure too high)


DFC_CHKON:
     brset   SSprog,trans2,BRK_CHK_DONE   ; If "SSprog" bit of "trans2"
                                          ; variable is set, branch to
                                          ; BRK_CHK_DONE:
                                          ;(gear change in progress)
     brset   TCprog,trans2,BRK_CHK_DONE   ; If "TCprog" bit of "trans2"
                                      ; variable is set, branch to
                                      ; BRK_CHK_DONE:
                                      ;(TCC apply in progress)
     brclr   DFCen,porta,DFC_COM      ; If "DFCen" bit of Port A is clear,
                                      ; branch to DFC_COM:
                                      ;(DFC permissive or DFC permisive
                                      ; and exhaust brake commanded)
     brset   Brkdel,shift,CLR_DEL     ; If "Brkdel" bit of "shift" variable
                                      ; is set, branch to CLR_DEL:
     bra     BRK_CHK_DONE             ; Branch to BRK_CHK_DONE:
                                      ;(no DFC/Exh Brk commanded, skip over)

CLR_DEL:
     bclr    Brkdel,shift             ; Clear "Brkdel" bit of "shift"
                                      ;(Button was released before brake
                                      ; delay timer timed out, so clear the
                                      ; flag)
     bra     BRK_CHK_DONE             ; Branch to BRK_CHK_DONE:
                                      ;(no DFC/Exh Brk commanded, skip over)

DFC_COM:

;****************************************************************************
; - We have permissives for DFC/Exhaust Brake, and an input signal from the
;   Decel Fuel Cut Enable contacts on the joy stick. Enable the DFC
;   permissive and start the Exhaust Brake delay timer to see if the exhaust
;   brake application will be commanded.
;****************************************************************************

     brset   Brkon,trans2,BRK_CHK_DONE ; If "Brkon" bit of "trans2" variable
                                       ; is set, branch to BRK_CHK_DONE:
                                       ;(Exh brake already on, skip over)
     brset   Brkdel,shift,BRK_COM      ; If "Brkdel" bit of "shift"
                                       ; variable is set, branch to BRK_COM:
                                       ;(waiting to see if ExhBrk commanded)
     brset   DFCon,trans2,DFC_DONE     ; If "DFCon" bit of "trans2" variable
                                       ; is set, branch to DFC_DONE:
                                       ;(DFC already permitted, skip over)
     bset    DFCper,portd      ; Set "DFCper" bit of Port D (DFC permit)
     bset    DFCon,trans2      ; Set "DFCon" bit of "trans2" variable

DFC_DONE:
     lda     ExBrk_del         ; Load accumulator with value in "ExBrk_del"
     sta     TIMcnt            ; Copy to "TIMcnt" variable
     bset    Brkdel,shift      ; Set "Brkdel" bit of "shift" variable
     bra     BRK_CHK_DONE      ; Branch to BRK_CHK_DONE:

BRK_COM:
     lda     TIMcnt            ; Load accumulator with value in "TIMcnt"
     bne     BRK_CHK_DONE      ; If Z bit of CCR is clear, branch to
                               ; BRK_CHK_DONE:

;****************************************************************************
; - The "DFCen" switch has been depressed for the "ExhBrkdel" time period so
;   the control air, and engine secondary air solonoids can be energised,
;   the "Brkon" flag set, and "Brkdel" flag cleared.
;****************************************************************************

     bclr    ExhBrk,portc      ; Clear "ExhBrk" bit of Port C (solonoids on)
     bset    Brkon,trans2      ; Set "Brkon" bit of "trans2" variable
     bclr    Brkdel,shift      ; Clear "Brkdel" bit of "shift" variable
     bra     BRK_CHK_DONE      ; Branch to BRK_CHK_DONE:

DFC_DIS:

;****************************************************************************
; - Either the DFC/Exhaust Brake has been commanded off by a signal from the
;   Decel Fuel Cut disable contacts on the joy stick, or we no longer have
;   permissives for DFC/Exhaust Brake application. Disable the DFC
;   permissive and clear the flag. De-energise the control air, and engine
;   secondary air solonoids, and clear the flag.
;****************************************************************************

     brclr   DFCon,trans2,BRK_REL     ; If "DFCon" bit of "trans2" variable
                                      ; is clear, branch to BRK_REL:
                                      ;(DFC already prohibited, skip over)
     bclr    DFCper,portd     ; Clear "DFCper" bit of Port D (DFC prohibit)
     bclr    DFCon,trans2     ; Clear "DFCon" bit of "trans2" variable

BRK_REL:
     brclr   Brkon,trans2,BRK_CHK_DONE ; If "Brkon" bit of "trans2" variable
                                       ; is clear, branch to BRK_CHK_DONE:
                                       ;(Exhaust brake not on, skip over)
     bset    ExhBrk,portc     ; Set "ExhBrk" bit of Port C (ExhBrk off)
     bclr    Brkon,trans2     ; Clear "Brkon" bit of "trans2" variable
     bclr    Brkdel,shift     ; Clear "Brkdel" bit of "shift" variable

BRK_CHK_DONE:
     rts                      ; Return from subroutine


;****************************************************************************
; - Manual 1, Drive 1 solonoid group
;****************************************************************************

M1D1_SOLS:
     bclr    SS1,portc         ; Clear "SS1" bit of Port C,(SS1 on)
     bset    SS2,portc         ; Set "SS2" bit of Port C,(SS2 off)
     bclr    CCS,portc         ; Clear "CCS" bit of Port C,(CCS on)
     bset    CCSon,trans2      ; Set "CCSon" bit of "trans2" variable
     rts                       ; Return from subroutine

;****************************************************************************
; - Manual 2, solonoid group
;****************************************************************************

M2_SOLS:
     bset    SS1,portc        ; Set "SS1" bit of Port C,(SS1 off)
     bset    SS2,portc        ; Set "SS2" bit of Port C,(SS2 off)
     bclr    CCS,portc        ; Clear "CCS" bit of Port C,(CCS on)
     bclr    D1D2,trans2      ; Clear "D1D2" bit of "trans2" variable
     bset    CCSon,trans2     ; Set "CCSon" bit of "trans2" variable
     rts                      ; Return from subroutine

;****************************************************************************
; - Drive 2, solonoid group
;****************************************************************************

D2_SOLS:
     bclr    SS1,portc        ; Clear "SS1" bit of Port C,(SS1 on)
     bclr    SS2,portc        ; Clear "SS2" bit of Port C,(SS2 on)
     bclr    CCS,portc        ; Clear "CCS" bit of Port C,(CCS on)
     bset    D1D2,trans2      ; Set "D1D2" bit of "trans2" variable
     bset    CCSon,trans2     ; Set "CCSon" bit of "trans2" variable
     rts                      ; Return from subroutine

;****************************************************************************
; - Drive 3, solonoid group
;****************************************************************************

D3_SOLS:
     bset    SS1,portc        ; Set "SS1" bit of Port C,(SS1 off)
     bclr    SS2,portc        ; Clear "SS2" bit of Port C,(SS2 on)
     bclr    CCS,portc        ; Clear "CCS" bit of Port C,(CCS on)
     bclr    D1D2,trans2      ; Clear "D1D2" bit of "trans2" variable
     bset    CCSon,trans2     ; Set "CCSon" bit of "trans2" variable
     rts                      ; Return from subroutine

;****************************************************************************
; - Drive 4, solonoid group
;****************************************************************************

D4_SOLS:
     bset    SS1,portc        ; Set "SS1" bit of Port C,(SS1 off)
     bset    SS2,portc        ; Set "SS2" bit of Port C,(SS2 off)
     bset    CCS,portc        ; Set "CCS" bit of Port C,(CCS off)
     bclr    D1D2,trans2      ; Clear "D1D2" bit of "trans2" variable
     bclr    CCSon,trans2     ; Clear "CCSon" bit of "trans2" variable
     rts                      ; Return from subroutine

;****************************************************************************
; - TCC release, DFC prohibit, Exhaust brake off group
;****************************************************************************

TCC_BRK_OFF:
     bset    TCC,portc        ; Set "TCC" bit of Port C (TCC off)
     bclr    TCCon,trans2     ; Clear "TCCon" bit of "trans2" variable
     bclr    EPCrTCC,shift    ; Clear "EPCrTCC" bit of "shift" variable
     bclr    EPChTCC,shift    ; Clear "EPChTCC" bit of "shift" variable
     bclr    TCprog,trans2    ; Clear "TCprog" bit of "trans2" variable
     bclr    DFCper,portd     ; Clear "DFCper" bit of Port D (DFC prohibit)
     bclr    DFCon,trans2     ; Clear "DFCon" bit of "trans2" variable
     bset    ExhBrk,portc     ; Set "ExhBrk" bit of Port C (ExhBrk off)
     bclr    Brkon,trans2     ; Clear "Brkon" bit of "trans2" variable
     bclr    Brkdel,shift     ; Clear "Brkdel" bit of "shift" variable
     rts                      ; Return from subroutine

;****************************************************************************
; - Cancel shift group
;****************************************************************************

CANC_SHFT:
     bclr    SSprog,trans2     ; Clear "SSprog" bit of "trans2" variable
     bclr    TCprog,trans2     ; Clear "TCprog" bit of "trans2" variable
     clr     shift             ; Clear "shift" variable
                               ;("EPCrTCC", "EPChTCC", EPCrSS", "EPChSS",
                               ; "SS1del", "CCSdel" "SSsdel", "Brkdel")
     clr     TIMcnt            ; Clear "TIMcnt" variable
     rts                       ; Return from subroutine


;****************************************************************************
; - First gear variable group
;****************************************************************************

GEAR1_VARS:
     mov     #first,gearcnt       ; Move value #first into "gearcnt" var
     mov     #first,gearcnt_prv   ; Move value #first into "gearcnt_prv" var
     mov     #first,gear_cur      ; Move value #first into "gear_cur" var
     mov     #first,gear_com      ; Move value #first into "gear_com" var
     rts                          ; Return from subroutine

;****************************************************************************
; - Second gear variable group
;****************************************************************************

GEAR2_VARS:
     mov     #second,gearcnt      ; Move value #second into "gearcnt" var
     mov     #second,gearcnt_prv  ; Move value #second into "gearcnt_prv" var
     mov     #second,gear_cur     ; Move value #second into "gear_cur" var
     mov     #second,gear_com     ; Move value #second into "gear_com" var
     rts                          ; Return from subroutine

;****************************************************************************
; - Third gear variable group
;****************************************************************************

GEAR3_VARS:
     mov     #third,gearcnt      ; Move value #third into "gearcnt" variable
     mov     #third,gearcnt_prv  ; Move value #third into "gearcnt_prv" var
     mov     #third,gear_cur     ; Move value #third into "gear_cur" variable
     mov     #third,gear_com     ; Move value #third into "gear_com" variable
     rts                         ; Return from subroutine

;****************************************************************************
; - Forth gear variable group
;****************************************************************************

GEAR4_VARS:
     mov     #forth,gearcnt      ; Move value #forth into "gearcnt" variable
     mov     #forth,gearcnt_prv  ; Move value #forth into "gearcnt_prv" var
     mov     #forth,gear_cur     ; Move value #forth into "gear_cur" variable
     mov     #forth,gear_com     ; Move value #forth into "gear_com" variable
     rts                         ; Return from subroutine


;***************************************************************************
; - Flash Burn routine goes here
;***************************************************************************

     include "burner.asm"         ; Include Flash Burner routine

;****************************************************************************
;-------------------Constants not possible to burn--------------------------
;****************************************************************************

        org     $E000      ; (57344)


REVNUM:
        db      10T     ; Revision 1.0

Signature db 32T,'** V12.00 Embedded Code by RJH **'


;****************************************************************************
; - Flash Configuration Tables and Constants (copied into RAM at start up)
;****************************************************************************

        org     $E100      ; SE100 to $E1C0 (57600 to 57792)

ms_rf_start_f:

;****************************************************************************
; - First group of 64 bytes
;****************************************************************************

; - TO_table, EPC duty factor 0-255("df", function of "KPA" and "RPM")
                      ;       (KPA,RPM)
     db      235T     ; TO_table(0,0)
     db      233T     ; TO_table(0,1)
     db      232T     ; TO_table(0,2)
     db      231T     ; TO_table(0,3)
     db      231T     ; TO_table(0,4)
     db      231T     ; TO_table(0,5)
     db      232T     ; TO_table(0,6)
     db      233T     ; TO_table(0,7)
     db      233T     ; TO_table(1,0)
     db      230T     ; TO_table(1,1)
     db      228T     ; TO_table(1,2)
     db      227T     ; TO_table(1,3)
     db      227T     ; TO_table(1,4)
     db      227T     ; TO_table(1,5)
     db      228T     ; TO_table(1,6)
     db      229T     ; TO_table(1,7)
     db      232T     ; TO_table(2,0)
     db      227T     ; TO_table(2,1)
     db      225T     ; TO_table(2,2)
     db      223T     ; TO_table(2,3)
     db      223T     ; TO_table(2,4)
     db      223T     ; TO_table(2,5)
     db      224T     ; TO_table(2,6)
     db      225T     ; TO_table(2,7)
     db      231T     ; TO_table(3,0)
     db      225T     ; TO_table(3,1)
     db      220T     ; TO_table(3,2)
     db      220T     ; TO_table(3,3)
     db      220T     ; TO_table(3,4)
     db      220T     ; TO_table(3,5)
     db      221T     ; TO_table(3,6)
     db      222T     ; TO_table(3,7)
     db      228T     ; TO_table(4,0)
     db      220T     ; TO_table(4,1)
     db      214T     ; TO_table(4,2)
     db      212T     ; TO_table(4,3)
     db      212T     ; TO_table(4,4)
     db      212T     ; TO_table(4,5)
     db      213T     ; TO_table(4,6)
     db      214T     ; TO_table(4,7)
     db      225T     ; TO_table(5,0)
     db      214T     ; TO_table(5,1)
     db      205T     ; TO_table(5,2)
     db      199T     ; TO_table(5,3)
     db      199T     ; TO_table(5,4)
     db      199T     ; TO_table(5,5)
     db      202T     ; TO_table(5,6)
     db      204T     ; TO_table(5,7)
     db      223T     ; TO_table(6,0)
     db      209T     ; TO_table(6,1)
     db      189T     ; TO_table(6,2)
     db      183T     ; TO_table(6,3)
     db      183T     ; TO_table(6,4)
     db      183T     ; TO_table(6,5)
     db      186T     ; TO_table(6,6)
     db      189T     ; TO_table(6,7)
     db      220T     ; TO_table(7,0)
     db      199T     ; TO_table(7,1)
     db      173T     ; TO_table(7,2)
     db      161T     ; TO_table(7,3)
     db      161T     ; TO_table(7,4)
     db      161T     ; TO_table(7,5)
     db      166T     ; TO_table(7,6)
     db      173T     ; TO_table(7,7)

;****************************************************************************
; - Second group of 64 bytes
;****************************************************************************

; - RPM bins for 2D interpolation of "TO" table(RPM_range:)

     db      20T     ; RPM_range[0] 400 RPM
     db      50T     ; RPM_range[1] 1000 RPM
     db      70T     ; RPM_range[2] 1400 RPM
     db      90T     ; RPM_range[3] 1800 RPM
     db      115T    ; RPM_range[4] 2300 RPM
     db      135T    ; RPM_range[5] 2700 RPM
     db      155T    ; RPM_range[6] 3100 RPM
     db      175T    ; RPM_range[7] 3500 RPM

; - KPA bins for 2D interpolation of "TO" table(KPA_range:)

     db      30T     ; KPA_range[0] ~21.3" vacuum
     db      40T     ; KPA_range[1] ~18.3" vacuum
     db      50T     ; KPA_range[2] ~15.4" vacuum
     db      60T     ; KPA_range[3] ~12.4" vacuum
     db      70T     ; KPA_range[4] ~9.5" vacuum
     db      80T     ; KPA_range[5] ~6.5" vacuum
     db      90T     ; KPA_range[6] ~3.5" vacuum
     db      100T    ; KPA_range[7] ~0.3" vacuum

; - TPS bins for 2D interpolation of EPC tables(TPS_range:)

     db     0T       ; TPS_range[0] 0% throttle opening
     db     10T      ; TPS_range[1] 10% throttle opening
     db     20T      ; TPS_range[2] 20% throttle opening
     db     30T      ; TPS_range[3] 30% throttle opening
     db     40T      ; TPS_range[4] 40% throttle opening
     db     50T      ; TPS_range[5] 50% throttle opening
     db     75T      ; TPS_range[6] 75% throttle opening
     db     100T     ; TPS_range[7] 100% throttle opening

; - EPC_stall bins, EPC duty factor 0-255("df", function of "TPSp")

     db     235T     ; EPC_stall[0]
     db     227T     ; EPC_stall[1]
     db     216T     ; EPC_stall[2]
     db     191T     ; EPC_stall[3]
     db     133T     ; EPC_stall[4]
     db     91T      ; EPC_stall[5]
     db     52T      ; EPC_stall[6]
     db     39T      ; EPC_stall[7]

; - EPC_12 shift bins, EPC duty factor 0-255("df", function of "TPSp")

     db     235T     ; EPC_12[0]
     db     220T     ; EPC_12[1]
     db     210T     ; EPC_12[2]
     db     170T     ; EPC_12[3]
     db     120T     ; EPC_12[4]
     db     100T     ; EPC_12[5]
     db     50T      ; EPC_12[6]
     db     35T      ; EPC_12[7]

; - EPC_23 shift bins, EPC duty factor 0-255("df", function of "TPSp")

     db     235T     ; EPC_23[0]
     db     200T     ; EPC_23[1]
     db     170T     ; EPC_23[2]
     db     120T     ; EPC_23[3]
     db     91T      ; EPC_23[4]
     db     64T      ; EPC_23[5]
     db     52T      ; EPC_23[6]
     db     39T      ; EPC_23[7]

; - EPC_34 shift bins, EPC duty factor 0-255("df", function of "TPSp")

     db     235T     ; EPC_34[0]
     db     185T     ; EPC_34[1]
     db     135T     ; EPC_34[2]
     db     85T      ; EPC_34[3]
     db     64T      ; EPC_34[4]
     db     52T      ; EPC_34[5]
     db     39T      ; EPC_34[6]
     db     39T      ; EPC_34[7]

; - EPC_M1 bins, EPC duty factor 0-255("df", function of "TPSp")

     db     235T     ; EPC_M1[0]
     db     225T     ; EPC_M1[1]
     db     215T     ; EPC_M1[2]
     db     210T     ; EPC_M1[3]
     db     205T     ; EPC_M1[4]
     db     200T     ; EPC_M1[5]
     db     195T     ; EPC_M1[6]
     db     190T     ; EPC_M1[7]

;****************************************************************************
; - Third group of 64 bytes
;****************************************************************************

; - Flash Configuration Constants

     db     147T    ; EPC_TCC     ; EPC duty factor for TCC application
     db     147T    ; EPC_decel   ; EPC duty factor for decel conditions
     db     5T      ; EPC_rise    ; EPC rise time delay(20mS res)
     db     75T     ; EPC_hold    ; EPC hold time delay(20mS res)
     db     15T     ; SS1_del     ; SS1 apply time delay(20mS res)
     db     20T     ; CCS_del     ; CCS apply time delay(20mS res)
     db     20T     ; SSs_del     ; SSs release time delay(20mS res)
     db     20T     ; ExBrk_del   ; Exhaust brake apply time delay(20mS res)
     db     60T     ; TCC_min_RPM ; TCC apply minimum RPM permissive(RPM/20)
     db     6T      ; MPH_stall   ; MPH maximum for stall EPC(MPH*2)
     db     8T      ; TPSrate     ; TPS DOT rate threshold for EPC stall(V/S)
     db     48T     ; CT_cnt      ; Closed throttle position ADC count
     db     227T    ; WOT_cnt     ; Wide Open throttle position ADC count
     db     179T    ; TPSspan     ; WOT_cnt - CT_cnt = TPSspan
     db     2T      ; CT_min      ; Closed throttle position minimum %
     db     20T     ; TrimFac     ; EPC Trim adjustment max value
     db     3T      ; TuneConfig  ; Tuning configuration variable
     db     0T      ; RPMk(0)     ; RPM calculation constant Hi byte
     db     0T      ; RPMk(1)     ; RPM calculation constant Lo byte
     db     20T     ; TOTempFac   ; Trans Oil Temp adjustment max value
     db     255T    ; TOThi       ; TOT correction Hi Limit (degreesF + 40)
     db     160T    ; TOTlo       ; TOT correction Lo Limit (degreesF + 40)
     db     175T    ; DBup        ; Upshift debounce counter (ms)
     db     175T    ; DBdn        ; Downshift debounce counter (ms)
     db     190T    ; EPC_M1_decel; EPC pulse width for M1 decel conditions


; - Place holders for future use (39)

    db     0T       ; blank_2_0
    db     0T       ; blank_2_1
    db     0T       ; blank_2_2
    db     0T       ; blank_2_3
    db     0T       ; blank_2_4
    db     0T       ; blank_2_5
    db     0T       ; blank_2_6
    db     0T       ; blank_2_7
    db     0T       ; blank_2_8
    db     0T       ; blank_2_9
    db     0T       ; blank_2_10
    db     0T       ; blank_2_11
    db     0T       ; blank_2_12
    db     0T       ; blank_2_13
    db     0T       ; blank_2_14
    db     0T       ; blank_2_15
    db     0T       ; blank_2_16
    db     0T       ; blank_2_17
    db     0T       ; blank_2_18
    db     0T       ; blank_2_19
    db     0T       ; blank_2_20
    db     0T       ; blank_2_21
    db     0T       ; blank_2_22
    db     0T       ; blank_2_23
    db     0T       ; blank_2_24
    db     0T       ; blank_2_25
    db     0T       ; blank_2_26
    db     0T       ; blank_2_27
    db     0T       ; blank_2_28
    db     0T       ; blank_2_29
    db     0T       ; blank_2_30
    db     0T       ; blank_2_31
    db     0T       ; blank_2_32
    db     0T       ; blank_2_33
    db     0T       ; blank_2_34
    db     0T       ; blank_2_35
    db     0T       ; blank_2_36
    db     0T       ; blank_2_37
    db     0T       ; blank_2_38

ms_rf_end_f:

;***************************************************************************
; - Boot Loader routine goes here
;***************************************************************************

     include "boot_r12.asm"       ; Include Boot Loader routine

;****************************************************************************
; - Lookup Tables
;****************************************************************************

     org     $F000     ; $F000 to $F600 (61440 to 62976)

     include "KPAfac_RH.inc" ; table=KPAfac_RH:,  offset=MAP,  result=KPA
     include "BatVolt.inc"   ; table=BatVolt:,    offset=BAT,  result=Volts
     include "TOTdeg.inc"    ; table=TOTdegrees:, offset=TOT,  result=TOTemp
     include "LinePrs.inc"   ; table=LinePress:,  offset=Lprs, result=Lpsi
     include "IdleCntrl.inc" ; table=IACcntrl:,   offset=IAC,  result=IACpw
     include "MLPSpos.inc"   ; table=MLPSposit:,  offset=MLPS, result=MLPSp


;***************************************************************************
; - Start of bootloader-defined jump table/vector
;***************************************************************************

     org     $FAC3              ; start bootloader-defined jump table/vector
                                ;(64195)
     db      $12                ; scbr regi init value
     db      %00000001          ; config1
     db      %00000001          ; config2
     dw      {rom_start + 256}  ; megasquirt code start
     dw      $FB00              ; bootloader start(64256)

;****************************************************************************
; - Vector table (origin vec_timebase)
;****************************************************************************

        db      $CC
	dw	Dummy          ;Time Base Vector
        db      $CC
	dw	ADC_ISR        ;ADC Conversion Complete
        db      $CC
	dw	KEYBD_ISR      ;Keyboard Vector
        db      $CC
	dw	SCITX_ISR      ;SCI Transmit Vector
        db      $CC
	dw	SCIRCV_ISR     ;SCI Receive Vector
        db      $CC
	dw	Dummy          ;SCI Error Vecotr
        db      $CC
	dw	Dummy          ;SPI Transmit Vector
        db      $CC
	dw	Dummy          ;SPI Receive Vector
        db      $CC
	dw      Dummy        ;TIM2 Overflow Vector
        db      $CC
	dw	  Dummy        ;TIM2 Ch1 Vector
        db      $CC
	dw	TIM2CH0_ISR    ;TIM2 Ch0 Vector
        db      $CC
	dw	TIM1OV_ISR     ;TIM1 Overflow Vector
        db      $CC
	dw	Dummy          ;TIM1 Ch1 Vector
        db      $CC
	dw	Dummy          ;TIM1 Ch0 Vector
        db      $CC
	dw	Dummy          ;PLL Vector
        db      $CC
	dw	IRQ_ISR        ;IRQ Vector
        db      $CC
	dw	Dummy          ;SWI Vector
        db      $CC
	dw	Start          ;Reset Vector

	end

