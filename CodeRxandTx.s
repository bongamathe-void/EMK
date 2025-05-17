    title "Testing idea of state machine with UART code comm"
    
    PROCESSOR 18F45K22
    
    CONFIG FOSC = INTIO67
    CONFIG WDTEN = OFF
    
    systemUtilReg	EQU	0x00; Bit0: Code Recieved, Bit1: Calibrate, Bit2: 0 = Idle, 1 = Driving, Bit3: TouchSensed, Bit4: Recieving Message,
				    ; Bit5 : Motors are moving, Bit6 : offlineFlag
    HighLvlStateReg	EQU	0x01; Flag register for the fall through state machine
    Code1		EQU	0x02; First message digit recieved
    Code2		EQU	0x03; Second message digit recieved
    Code3		EQU	0x04; Third message digit recieved
    CodeCTR		EQU	0x05; Keeps count of digits recieved
    RxSCRATCH		EQU	0x06; Used in Rx ISR to hold recieved byte
    SCRATCH		EQU	0x07
    WTEMP		EQU	0x08
    followColor		EQU	0x09; Determines curren color being followed
    Count1		EQU	0x0A; used for delays and counting
    Count2		EQU	0x0B; used for delays and counting
    Count3		EQU	0x0C; used for delays and counting
    CLRCNT		EQU	0x0D; used to iterate over 3 colors in Calibration
    ADCONSEL		EQU	0x0E; used to determine which ADC channel to select
    THRESH3		EQU	0x0F; THRESH3 RGB : {R3, G3, B3}0x0F,0x10,0x11
    THRESH4		EQU	0x12; THRESH4 RGB : {R4, G4, B4}0x12,0x13,0x14
    THRESH5		EQU	0x15; THRESH5 RGB : {R5, G5, B5}0x15,0x16,0x17
    THRESH6		EQU	0x18; THRESH6 RGB : {R6, G6, B6}0x18,0x19,0x1A
    THRESH7		EQU	0x1B; THRESH7 RGB : {R7, G7, B7}0x1B,0x1C,0x1D
    SMPL0		EQU	0x1E; for averaging in readAndAverage
    SMPL1		EQU	0x1F; for averaging in readAndAverage
    TouchThresh		EQU	0x20; holds the low threshold for touch sensing
    LLIStateReg		EQU	0x21; bits represent LLI actions
    CURRENT3		EQU	0x22; CURRENT3 RGB : {R3, G3, B3}0x22,0x23,0x24
    CURRENT4		EQU	0x25; CURRENT4 RGB : {R4, G4, B4}0x25,0x26,0x27
    CURRENT5		EQU	0x28; CURRENT5 RGB : {R5, G5, B5}0x28,0x29,0x2A
    CURRENT6		EQU	0x2B; CURRENT6 RGB : {R6, G6, B6}0x2B,0x2C,0x2D
    CURRENT7		EQU	0x2E; CURRENT7 RGB : {R7, G7, B7}0x2E,0x2F,0x30
    ColorReg3		EQU	0x31; Bit0 : R, Bit1: G, Bit2 : B ; determines what color the sensor is on
    ColorReg4		EQU	0x32; Bit0 : R, Bit1: G, Bit2 : B
    ColorReg5		EQU	0x33; Bit0 : R, Bit1: G, Bit2 : B
    ColorReg6		EQU	0x34; Bit0 : R, Bit1: G, Bit2 : B
    ColorReg7		EQU	0x35; Bit0 : R, Bit1: G, Bit2 : B
    stopReg		EQU	0x36
    lowerClr		EQU	0x37; used in LineMapper
    SensorLine		EQU	0x38; used in LineMapper and LLI
    PageCTR		EQU	0x39; used to count page block of 8 bytes when writing I2C
    PageAddress		EQU	0x3A; used to point to current write address in I2C
    DiagStateReg	EQU     0x3B; used to determine what diagnostic state
    HighLvlStateReg2	EQU	0x3C; Bit0: TransitionState, Bit1 : offline home, Bit2 : offline SelectColor, Bit3 : offline Calibration, 
				    ; Bit4 : offline Race
	
    ;state  bits--------------
    stateHome	EQU	0
    stateSelC	EQU	1
    stateCal	EQU	2
    stateRace	EQU	3
    stateDiag	EQU	4
    stateProg	EQU	5
    ;offline state bits-------
    trans	EQU	0
    offlineHome	EQU	1
    offlineSelC	EQU	2
    offlineCal	EQU	3
    offlineRace	EQU	4
    ;Diagnostics bits---------
    stateForward EQU	0
    stateLeft	EQU	1
    stateRight	EQU	2
    stateStop	EQU	3
    ;Constants to switch RGBs--------
    RED		EQU	1
    GREEN	EQU	2
    BLUE	EQU	4
    ;Constants for channels of the ADC
    C3		EQU	00111101B;Leftmost sensor
    C4		EQU	01000001B
    C5		EQU	01000101B
    C6		EQU	01001001B
    C7		EQU	01001101B;Rightmost sensor
    D3	        EQU     01011101B ; D3 AN23
    D4	        EQU     01100001B ; D4 AN24
    ;Command bytes for I2C
    WriteByte	EQU	10100000B
    ReadByte	EQU	10100001B
	
    ;Steering Constants
    PA		EQU	00010001B
    PB		EQU	00010010B
    
    #include <xc.inc>
    #include "pic18f45k22.inc"
    
    PSECT code,abs,ovrld
    org	    00h
    GOTO    INIT
    org	    08h
    BTFSC   PIR3,5   
    GOTO    RxISR
    GOTO    ISR
    
INIT:
    MOVLB   0x0F
    
    ;16MHz clock
    MOVLW   01110110B
    MOVWF   OSCCON
    CLRF    OSCCON2
    
    ;using Port A for visual feedback 
    CLRF    PORTA
    CLRF    LATA
    CLRF    TRISA
    CLRF    ANSELA
    
    ;using PortB for Visual feedback and Interrupt
    CLRF    PORTB
    CLRF    LATB
    CLRF    ANSELB
    CLRF    TRISB
    BSF	    TRISB,0
    
    ;using PortE to switch on the RGBs
    CLRF    PORTE
    CLRF    LATE
    CLRF    ANSELE
    CLRF    TRISE
    
    ;using Port C to read sensors
    CLRF    PORTC
    CLRF    LATC
    MOVLW   11111000B
    MOVWF   ANSELC
    MOVWF   TRISC
    
    ;Configuring ADC 
    MOVLW   C5
    MOVWF   ADCON0
    CLRF    ADCON1;Internal Vdd of 3.3V and Vss of 0V
    MOVLW   00111111B;Left justified,20Tad,Frc
    MOVWF   ADCON2
    CLRF    ADRESH
    
    ;Config IO Pins for EUART and Touch Sensor
    CLRF    PORTD
    CLRF    LATD
    CLRF    ANSELD
    BSF	    ANSELD,4 ; Touch sensor pin
    MOVLW   11000011B
    MOVWF   TRISD ; set TRISD 6 AND 7 to 1; EUART automatical converts input and output mode where necessary
    
    ;115200 BaudRate
    MOVLW   34
    MOVWF   SPBRG2
    CLRF    SPBRGH2
    
    ;EUART Config 
    MOVLW   01001000B
    MOVWF   BAUDCON2
    MOVLW   00100110B 
    MOVWF   TXSTA2
    MOVLW   10010000B 
    MOVWF   RCSTA2
    
    ;Configuring I2C
    MOVLW   34
    MOVWF   SSP2ADD ; set baud rate to 115200
    CLRF    SSP2STAT
    MOVLW   00101000B
    MOVWF   SSP2CON1
    CLRF    SSP2CON2
    CLRF    SSP2CON3
    
    ;Config Interupts 
    MOVLW   10000001B
    MOVWF   INTCON2
    MOVLW   11010000B
    MOVWF   INTCON
    MOVLW   00100000B ; RC2IE 
    MOVWF   PIE3
    CLRF    PIR3
    
    ;Configuration of timers
    MOVLW	00000010B
    MOVWF	T2CON
    MOVLW	249
    MOVWF	PR2 ; PWM FREQUENCY IS SET TO 1KHz
    CLRF	TMR2
    
    ;Configuration of CCP
    MOVLW	00001100B
    MOVWF	CCP1CON
    MOVWF	CCP2CON
    MOVLW	0
    MOVWF	CCPR1L
    MOVWF	CCPR2L
    CLRF	CCPTMRS0
    MOVLW	PA
    MOVWF	PSTR1CON
    MOVWF	PSTR2CON
    
    MOVLB   0x00
    CLRF    HighLvlStateReg2
    CLRF    HighLvlStateReg
    BSF	    HighLvlStateReg,stateRace
    CLRF    systemUtilReg
    CLRF    CLRCNT
    GOTO    Main
    
;0000000000000000000000000000000000000000000000000000000000000000000000000000000
Main:
; High level state machine for Prac 3
; Each state peforms its function as a loop of subroutine calls
; After state function , checks if a code has been recieved via EUART  
; Fall through FSM is employed, where Check_Code_Recieved, determines the 
; flow of the state machine 
STATE_HOME:
    BTFSS   HighLvlStateReg,stateHome
    GOTO    STATE_SELECT_COLOR
    MOVLW   11111100B
    MOVWF   PORTA
    call    Check_Code_Recieved
;^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^   
STATE_SELECT_COLOR:
    BTFSS   HighLvlStateReg,stateSelC
    GOTO    STATE_CALIBRATION
    MOVLW   00001100B
    MOVWF   PORTA
    
    CLRF    PORTB
    MOVLW   0
    CPFSGT  followColor
    BRA	    showYellow
    MOVLW   1
    CPFSGT  followColor
    BRA	    showRed
    MOVLW   3
    CPFSGT  followColor
    BRA	    showGreen
    BRA	    showBlue
    showYellow:
	BSF	    PORTB,1
	BRA	    End_Of_State_SC
    showRed:
	BSF	    PORTB,5
	BRA	    End_Of_State_SC
    showGreen:
	BSF	    PORTB,4
	BRA	    End_Of_State_SC
    showBlue:
	BSF	    PORTB,3
End_Of_State_SC:
    call    Check_Code_Recieved
;^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^   
STATE_CALIBRATION:
    BTFSS   HighLvlStateReg,stateCal
    GOTO    STATE_RACE
    MOVLW   11011010B
    MOVWF   PORTA
    CLRF    PORTB
    BTFSS   systemUtilReg,1
    BRA	    End_Of_State_Cal
    MOVLW   3
    CPFSLT  CLRCNT
    BRA	    State_Cal_Red
    DECF    WREG
    CPFSLT  CLRCNT
    BRA	    State_Cal_Green
    DECF    WREG    
    CPFSLT  CLRCNT
    BRA	    State_Cal_Blue
    State_Cal_Done:
	BCF	    systemUtilReg,1
	BRA	    End_Of_State_Cal
    State_Cal_Red:
	BSF	    PORTB,5
	call	    Calibrate
	MOVLW	    6
	MOVWF	    SCRATCH
	BlinkRed:
	    BTG		    PORTB,5
	    call	    Delay333
	    DECFSZ	    SCRATCH
	    BRA		    BlinkRed
	    BRA		    End_Of_State_Cal
    State_Cal_Green:
	BSF	    PORTB,4
	call	    Calibrate
	MOVLW	    6
	MOVWF	    SCRATCH
	BlinkGreen:
	    BTG		    PORTB,4
	    call	    Delay333
	    DECFSZ	    SCRATCH
	    BRA		    BlinkGreen
	    BRA		    End_Of_State_Cal
    State_Cal_Blue:
	BSF	    PORTB,3
	call	    Calibrate
	MOVLW	    6
	MOVWF	    SCRATCH
	BlinkBlue:
	    BTG		    PORTB,3
	    call	    Delay333
	    DECFSZ	    SCRATCH
	    BRA		    BlinkBlue
End_Of_State_Cal:
    call    Check_Code_Recieved
;^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
STATE_RACE:
    BTFSS   HighLvlStateReg,stateRace
    GOTO    STATE_DIAGNOSTICS
    
    BTFSC   systemUtilReg,2
    BRA	    DrivingState
    IdleState:
	MOVLW   11110010B
	MOVWF   PORTA
	call	Idle 
	BTFSS	systemUtilReg,2 ; check if touch was detected and set drive flag
	BRA	End_Of_State_Race
	TransitionToDrive:
	    call	showRaceColor
    DrivingState:
	call	CurrentReading
	call	DetermineColor
	call	lineMapper
	call	LLI
	call	MotorControl
End_Of_State_Race:
    call    Check_Code_Recieved
;^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
STATE_DIAGNOSTICS:
    BTFSS   HighLvlStateReg,stateDiag
    GOTO    STATE_PROGRAM
    
    MOVLW   00101110B
    MOVWF   PORTA
    
    BTFSC   DiagStateReg,stateForward
    CALL    DiagForward
    
    BTFSC   DiagStateReg,stateLeft
    CALL    DiagLeft
    
    BTFSC   DiagStateReg,stateRight
    CALL    DiagRight
    
    BTFSC   DiagStateReg,stateStop
    CALL    DiagStop
    
End_Of_State_Diag:
    call    Check_Code_Recieved
;^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
STATE_PROGRAM:
    BTFSS   HighLvlStateReg,stateProg
    GOTO    STATE_TRANS
    MOVLW   10110110B
    MOVWF   PORTA
End_Of_State_Prog:
    call    Check_Code_Recieved
;^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
STATE_TRANS:
    BTFSS   HighLvlStateReg2,trans
    GOTO    OFFLINE_HOME
    
    MOVLW   11100010B
    MOVWF   PORTA
    
    BTFSC		systemUtilReg,6
    BRA			OfflineFlagSet
    OfflineFlagClear:
	BCF		PORTB,2
	BRA		CheckOnlineFlagChange
    OfflineFlagSet:
	BSF		PORTB,2
    
CheckOnlineFlagChange:
    BCF		systemUtilReg,3
    call	Touchsensor
    call	Touchsensor
    BTFSS	systemUtilReg,3 ; touch sensed
    BRA		End_Of_State_Trans
    ToggleOfflineFlag:
	BCF		systemUtilReg,3 ; acknowledge touch
	BTG		systemUtilReg,6 ; toggle flag
	call		Delay333
	
End_Of_State_Trans:
;^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
OFFLINE_HOME:
    BTFSS   HighLvlStateReg2,offlineHome
    GOTO    OFFLINE_SELC
    
    MOVLW   11111100B
    MOVWF   PORTA
End_Of_State_Offline_Home:
;^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
OFFLINE_SELC:
    BTFSS   HighLvlStateReg2,offlineSelC
    GOTO    OFFLINE_CALIBRATION
    
    MOVLW   00001100B
    MOVWF   PORTA
    
    CLRF    PORTB
    MOVLW   0
    CPFSGT  followColor
    BRA	    showYellow2
    MOVLW   1
    CPFSGT  followColor
    BRA	    showRed2
    MOVLW   3
    CPFSGT  followColor
    BRA	    showGreen2
    BRA	    showBlue2
    showYellow2:
	BSF	    PORTB,1
	BRA	    CheckColorChange
    showRed2:
	BSF	    PORTB,5
	BRA	    CheckColorChange
    showGreen2:
	BSF	    PORTB,4
	BRA	    CheckColorChange
    showBlue2:
	BSF	    PORTB,3
    CheckColorChange:
	BCF		systemUtilReg,3
	call		Touchsensor
	call		Touchsensor
	BTFSS		systemUtilReg,3 ; touch sensed
	BRA		End_Of_State_Offline_SelC
	ChangeColor:
	    BCF		systemUtilReg,3; acknowledge touch
	    call	ChangeRaceColor
	    call	Delay333
End_Of_State_Offline_SelC:
;^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
OFFLINE_CALIBRATION:
    BTFSS   HighLvlStateReg2,offlineCal
    GOTO    OFFLINE_RACE
    
    MOVLW   11011010B
    MOVWF   PORTA
    
    CLRF    PORTB
    BTFSS   systemUtilReg,1
    BRA	    CheckForCalibrationTrigger
    MOVLW   3
    CPFSLT  CLRCNT
    BRA	    State_Cal_Red2
    DECF    WREG
    CPFSLT  CLRCNT
    BRA	    State_Cal_Green2
    DECF    WREG    
    CPFSLT  CLRCNT
    BRA	    State_Cal_Blue2
    State_Cal_Done2:
	BCF	    systemUtilReg,1
	BRA	    End_Of_State_Offline_Cal
    State_Cal_Red2:
	BSF	    PORTB,5
	call	    Calibrate
	MOVLW	    6
	MOVWF	    SCRATCH
	BlinkRed2:
	    BTG		    PORTB,5
	    call	    Delay333
	    DECFSZ	    SCRATCH
	    BRA		    BlinkRed2
	    BRA		    End_Of_State_Offline_Cal
    State_Cal_Green2:
	BSF	    PORTB,4
	call	    Calibrate
	MOVLW	    6
	MOVWF	    SCRATCH
	BlinkGreen2:
	    BTG		    PORTB,4
	    call	    Delay333
	    DECFSZ	    SCRATCH
	    BRA		    BlinkGreen2
	    BRA		    End_Of_State_Offline_Cal
    State_Cal_Blue2:
	BSF	    PORTB,3
	call	    Calibrate
	MOVLW	    6
	MOVWF	    SCRATCH
	BlinkBlue2:
	    BTG		    PORTB,3
	    call	    Delay333
	    DECFSZ	    SCRATCH
	    BRA		    BlinkBlue2
	    BRA		    End_Of_State_Offline_Cal
    CheckForCalibrationTrigger:
	BCF		systemUtilReg,3
	call		Touchsensor
	call		Touchsensor
	BTFSS		systemUtilReg,3 ; touch sensed
	BRA		End_Of_State_Offline_Cal
	CalibrationTriggered:
	    BCF		systemUtilReg,3; acknowledge touch
	    BSF		systemUtilReg,1
	    MOVLW	3
	    MOVWF	CLRCNT
	    call	Delay333
End_Of_State_Offline_Cal:
;^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
OFFLINE_RACE:
    BTFSS   HighLvlStateReg2,offlineRace
    GOTO    Main
    
    BTFSC   systemUtilReg,2
    BRA	    DrivingState2
    IdleState2:
	MOVLW   11110010B
	MOVWF   PORTA
	call	Idle 
	BTFSS	systemUtilReg,2 ; check if touch was detected and set drive flag
	BRA	End_Of_State_Offline_Race
	TransitionToDrive2:
	    call	showRaceColor
    DrivingState2:
	call	CurrentReading
	call	DetermineColor
	call	lineMapper
	call	LLI
	call	MotorControl
End_Of_State_Offline_Race:
    GOTO    Main ; final loop catch statement
;0000000000000000000000000000000000000000000000000000000000000000000000000000000
    
ISR:
    MOVWF   WTEMP
    MOVLW   0
    CPFSGT  HighLvlStateReg2 ; coming from online to transition
    BRA	    OnlineToTransition
    BTFSC   HighLvlStateReg2,trans
    BRA	    TransToHome
    BTFSC   HighLvlStateReg2,offlineHome
    BRA	    HomeToSelC
    BTFSC   HighLvlStateReg2,offlineSelC
    BRA	    SelCToCal
    BTFSC   HighLvlStateReg2,offlineCal
    BRA	    CalToRace
    BTFSC   HighLvlStateReg2,offlineRace
    BRA	    RaceToTrans
    BRA	    End_Of_ISR
OnlineToTransition:
    call    Idle ; If the MARV is moving when the button is pressed, the MARV should stop
    CLRF    HighLvlStateReg
    CLRF    HighLvlStateReg2
    BSF	    HighLvlStateReg2,trans
    BCF	    RCSTA2,7 ; switch off UART
    BCF	    SSP2CON1,5 ; switch off I2C
    BRA	    End_Of_ISR
TransToHome: ; determines which home to go to, offline home or online home
    BTFSC   systemUtilReg,6 ; bit 6 : OfflineMode on flag
    BRA	    TransToOfflineHome
TransToOnlineHome:
    CLRF    HighLvlStateReg2
    CLRF    HighLvlStateReg
    BSF	    HighLvlStateReg,stateHome
    ;Enable UART AND I2C
    BSF	    RCSTA2,7 ; switch on UART module
    BSF	    SSP2CON1,5 ; switch on I2C module
    BRA	    End_Of_ISR
TransToOfflineHome:
    CLRF    HighLvlStateReg2
    CLRF    HighLvlStateReg
    BSF	    HighLvlStateReg2,offlineHome
    ;Disable UART AND I2C
    BCF	    RCSTA2,7 ; switch off UART module
    BCF	    SSP2CON1,5 ; switch off UART module
    BRA	    End_Of_ISR
HomeToSelC:
    CLRF    HighLvlStateReg
    CLRF    HighLvlStateReg2
    BSF	    HighLvlStateReg2,offlineSelC
    BRA	    End_Of_ISR
SelCToCal:
    CLRF    HighLvlStateReg
    CLRF    HighLvlStateReg2
    BSF	    HighLvlStateReg2,offlineCal
    BRA	    End_Of_ISR
CalToRace:
    CLRF    HighLvlStateReg
    CLRF    HighLvlStateReg2
    BSF	    HighLvlStateReg2,offlineRace
    BRA	    End_Of_ISR
RaceToTrans:
    call    Idle
    CLRF    HighLvlStateReg
    CLRF    HighLvlStateReg2
    BSF	    HighLvlStateReg2,trans
End_Of_ISR:
    BCF	    INTCON,1
    MOVF    WTEMP,W
    RETFIE
    
RxISR:
    BTFSC   systemUtilReg,4 ; check if message recieve mode been set
    BRA	    MessageISR
    MOVWF   WTEMP; Save WREG value 
    MOVF    RCREG2,W
    MOVWF   RxSCRATCH
    MOVLW   '#'
    CPFSEQ  RxSCRATCH ; check data recieved is a '#'
    BRA	    NotStart
    CLRF    CodeCTR ; start count of code symbols from 0
    BRA    End_Of_RxISR
    
NotStart:
    MOVLW   0x0A ; check if data recieved is a '\n'
    CPFSEQ  RxSCRATCH
    BRA	    NotEnd
    BSF	    systemUtilReg,0 ; indicate to system that a code has been recieved via EUART
    BRA    End_Of_RxISR
    
NotEnd:
    MOVLW   0
    CPFSGT  CodeCTR
    BRA	    FirstSymbol
    MOVLW   1
    CPFSGT  CodeCTR 
    BRA	    SecondSymbol
    MOVLW   2
    CPFSGT  CodeCTR
    BRA	    ThirdSymbol
    BRA	    End_Of_RxISR
    
FirstSymbol:
    MOVFF   RxSCRATCH,Code1
    INCF    CodeCTR
    BRA	    End_Of_RxISR
SecondSymbol:
    MOVFF   RxSCRATCH, Code2
    INCF    CodeCTR
    BRA	    End_Of_RxISR
ThirdSymbol:
    MOVFF   RxSCRATCH, Code3
    INCF    CodeCTR
    
End_Of_RxISR:
    MOVF    WTEMP,W ; Restore WREG
    RETFIE
    
MessageISR:
    MOVFF	RCREG2,RxSCRATCH
    MOVFF	RxSCRATCH,INDF2
    INCF	FSR2
    MOVLW	0x0A ; check if \n was recieved to terminate message
    CPFSEQ	RxSCRATCH
    BRA		End_Of_MessageISR
    MOVLW	'5'
    MOVWF	Code1
    MOVLW	'1'
    MOVWF	Code2
    MOVLW	'3'
    MOVWF	Code3
    MOVLW	0
    MOVWF	INDF2
    BSF		systemUtilReg,0
    BCF		systemUtilReg,4
End_Of_MessageISR:
    RETFIE

;########################SUB ROUTINE############################################
Check_Code_Recieved:
    BTFSS   systemUtilReg,0; check if the system has flagged that a code has been recieved via EUART
    GOTO    End_Of_Check_Code
    
    BCF	    systemUtilReg,0; code is being attended to 
    ;Write Code to determine what response to send back and what state
    ;the high level state machine must transition to 
    MOVF    Code1,W
    ADDLW   -0x30 ; converting digit 1 recieved  to a number 
    MOVWF   SCRATCH
    MOVLW   0     ; check for 0XX codes
    CPFSGT  SCRATCH
    GOTO    Home_codes
    INCF    WREG  ; check for 1XX codes
    CPFSGT  SCRATCH
    GOTO    SelectC_codes
    INCF    WREG  ; check for 2XX codes
    CPFSGT  SCRATCH
    GOTO    Calibration_codes
    INCF    WREG  ; check for 3XX codes
    CPFSGT  SCRATCH
    GOTO    Race_codes
    INCF    WREG  ; check for 4XX codes
    CPFSGT  SCRATCH
    GOTO    Diagnostics_codes
    INCF    WREG  ; check for 5XX codes
    CPFSGT  SCRATCH
    GOTO    Program_codes
    GOTO    Home_codes ; default to Home mode if non valid code recieved
;$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$    
Home_codes:
    ;MARV transistion to Home, and responds with Home Mode code
    MOVLW   HomeMode
    MOVWF   TBLPTRL,0
    MOVLW   (HomeMode>>8)
    MOVWF   TBLPTRH,0
    MOVLW   (HomeMode>>16)
    MOVWF   TBLPTRU,0
    
    call    Idle ; switch of the PWMs, consequently the motors if running
    CLRF    HighLvlStateReg
    BSF	    HighLvlStateReg,stateHome ; MAKE Fall through FSM go to Home
    
    call    TransmitMARVResponse
    GOTO    End_Of_Check_Code
;$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
SelectC_codes:
    MOVLW   SelectColorMode
    MOVWF   TBLPTRL,0
    MOVLW   (SelectColorMode>>8)
    MOVWF   TBLPTRH,0
    MOVLW   (SelectColorMode>>16)
    MOVWF   TBLPTRU,0
    
    CLRF    HighLvlStateReg
    BSF	    HighLvlStateReg,stateSelC ; MAKE Fall through FSM got to Select Clr
    
    MOVF    Code2,W
    ADDLW   -0x30 ; converting digit 2 recieved  to a number 
    MOVWF   SCRATCH
    MOVLW   0
    CPFSGT  SCRATCH
    BRA	    End_Of_Select_Color_Codes
    INCF    WREG
    CPFSGT  SCRATCH ; check if Choose Red to follow
    BRA	    SetColorToRed
    INCF    WREG
    CPFSGT  SCRATCH ; check if Choose Green to follow
    BRA	    SetColorToGreen
    INCF    WREG
    CPFSGT  SCRATCH ; check if Choose Blue to follow
    BRA	    SetColorToBlue
    
SetColorToBlack: ; default to Choose Black to follow
    CLRF    followColor ; followColor = 0 : Black
    CLRF    lowerClr ; lowerClr = 0
    BRA	    End_Of_Select_Color_Codes
SetColorToRed:
    MOVLW   1
    MOVWF   followColor ; followColor = 1 : Red
    MOVWF   lowerClr ; lowerClr = 1
    BRA	    End_Of_Select_Color_Codes
SetColorToGreen:
    MOVLW   3
    MOVWF   followColor ; followColor = 3 : Green
    DECF    WREG
    MOVWF   lowerClr ; lowerClr = 2
    BRA	    End_Of_Select_Color_Codes
SetColorToBlue:
    MOVLW   6
    MOVWF   followColor ; followColor = 6 : Blue
    MOVLW   4
    MOVWF   lowerClr	; lowerClr = 4
End_Of_Select_Color_Codes:
    call    TransmitMARVResponse
    GOTO    End_Of_Check_Code
;$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
Calibration_codes:
    MOVF    Code3,W
    ADDLW   -0x30 ; converting digit 3 recieved  to a number 
    MOVWF   SCRATCH
    MOVLW   0
    CPFSGT  SCRATCH ; checking if 200 was sent
    BRA	    Set_To_Calibration_Mode
    
    MOVLW   3
    CPFSLT  CLRCNT
    BRA	    Respond_Calibrating_Red
    DECF    WREG
    CPFSLT  CLRCNT
    BRA	    Respond_Calibrating_Green
    DECF    WREG
    CPFSLT  CLRCNT
    BRA	    Respond_Calibrating_Blue
   
Respond_Calibration_Done:
    MOVLW   CalDone
    MOVWF   TBLPTRL,0
    MOVLW   (CalDone>>8)
    MOVWF   TBLPTRH,0
    MOVLW   (CalDone>>16)
    MOVWF   TBLPTRU,0
    BRA	    End_Of_Calibration_Codes
Respond_Calibrating_Red:
    MOVLW   CalRed
    MOVWF   TBLPTRL,0
    MOVLW   (CalRed>>8)
    MOVWF   TBLPTRH,0
    MOVLW   (CalRed>>16)
    MOVWF   TBLPTRU,0
    BRA	    End_Of_Calibration_Codes
Respond_Calibrating_Green:
    MOVLW   CalGreen
    MOVWF   TBLPTRL,0
    MOVLW   (CalGreen>>8)
    MOVWF   TBLPTRH,0
    MOVLW   (CalGreen>>16)
    MOVWF   TBLPTRU,0
    BRA	    End_Of_Calibration_Codes
Respond_Calibrating_Blue:
    MOVLW   CalBlue
    MOVWF   TBLPTRL,0
    MOVLW   (CalBlue>>8)
    MOVWF   TBLPTRH,0
    MOVLW   (CalBlue>>16)
    MOVWF   TBLPTRU,0
    BRA	    End_Of_Calibration_Codes
    
Set_To_Calibration_Mode:
    MOVLW   CalibrationMode
    MOVWF   TBLPTRL,0
    MOVLW   (CalibrationMode>>8)
    MOVWF   TBLPTRH,0
    MOVLW   (CalibrationMode>>16)
    MOVWF   TBLPTRU,0
    
    CLRF    HighLvlStateReg
    BSF	    HighLvlStateReg,stateCal ; MAKE Fall through FSM got to Calibration
    BSF	    systemUtilReg, 1 ; Tell system to Calibrate
    MOVLW   3
    MOVWF   CLRCNT ; Start Calibrating on Red
End_Of_Calibration_Codes:
    call    TransmitMARVResponse
    GOTO    End_Of_Check_Code
;$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$  
Race_codes:
    MOVF    Code3,W
    ADDLW   -0x30 ; converting digit 3 recieved  to a number 
    MOVWF   SCRATCH
    MOVLW   0
    CPFSGT  SCRATCH ; check if 300 was sent
    BRA	    Set_To_Race_Mode
    MOVLW   2
    CPFSGT  SCRATCH ; check if 302 was sent
    BRA	    Respond_Race_Status
    
Respond_Race_Color:; default to responding with Race Color
    MOVLW   0
    CPFSGT  followColor
    BRA	    Respond_Race_Black
    INCF    WREG
    CPFSGT  followColor
    BRA	    Respond_Race_Red
    MOVLW   3
    CPFSGT  followColor
    BRA	    Respond_Race_Green
    Respond_Race_Blue:
	MOVLW   RaceBlue
	MOVWF   TBLPTRL,0
	MOVLW   (RaceBlue>>8)
	MOVWF   TBLPTRH,0
	MOVLW   (RaceBlue>>16)
	MOVWF   TBLPTRU,0
	BRA	    End_Of_Race_Codes
    Respond_Race_Black:
	MOVLW   RaceBlack
	MOVWF   TBLPTRL,0
	MOVLW   (RaceBlack>>8)
	MOVWF   TBLPTRH,0
	MOVLW   (RaceBlack>>16)
	MOVWF   TBLPTRU,0
	BRA	    End_Of_Race_Codes
    Respond_Race_Red:
	MOVLW   RaceRed
	MOVWF   TBLPTRL,0
	MOVLW   (RaceRed>>8)
	MOVWF   TBLPTRH,0
	MOVLW   (RaceRed>>16)
	MOVWF   TBLPTRU,0
	BRA	    End_Of_Race_Codes
    Respond_Race_Green:
	MOVLW   RaceGreen
	MOVWF   TBLPTRL,0
	MOVLW   (RaceGreen>>8)
	MOVWF   TBLPTRH,0
	MOVLW   (RaceGreen>>16)
	MOVWF   TBLPTRU,0
	BRA	    End_Of_Race_Codes
Respond_Race_Status:
    BTFSC   systemUtilReg,2
    BRA	    RespondDriving
    RespondIdle:
	MOVLW   CurrIdle
	MOVWF   TBLPTRL,0
	MOVLW   (CurrIdle>>8)
	MOVWF   TBLPTRH,0
	MOVLW   (CurrIdle>>16)
	MOVWF   TBLPTRU,0
	BRA	    End_Of_Race_Codes
    RespondDriving:
	MOVLW   CurrDrive
	MOVWF   TBLPTRL,0
	MOVLW   (CurrDrive>>8)
	MOVWF   TBLPTRH,0
	MOVLW   (CurrDrive>>16)
	MOVWF   TBLPTRU,0
	BRA	    End_Of_Race_Codes
Set_To_Race_Mode:
    MOVLW   RaceMode
    MOVWF   TBLPTRL,0
    MOVLW   (RaceMode>>8)
    MOVWF   TBLPTRH,0
    MOVLW   (RaceMode>>16)
    MOVWF   TBLPTRU,0
    
    CLRF    HighLvlStateReg
    BSF	    HighLvlStateReg,stateRace
End_Of_Race_Codes:
    call    TransmitMARVResponse
    GOTO    End_Of_Check_Code
;$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
Diagnostics_codes:
    MOVF    Code2,W
    ADDLW   -0x30           ; W = Code2 - '0'
    MOVWF   SCRATCH
    MOVLW   0
    CPFSGT  SCRATCH         ; if SCRATCH <= 0 ? it's 400
    BRA     Set_To_Diag_Mode
    MOVLW   1
    CPFSGT  SCRATCH         ; if SCRATCH <= 1 ? it's 410
    BRA     Run_Sensor_Test
    MOVLW   2
    CPFSGT  SCRATCH         ; 420
    BRA     Run_Forward_Test
    MOVLW   3
    CPFSGT  SCRATCH         ; 430
    BRA     Run_Left_Test
    MOVLW   4
    CPFSGT  SCRATCH         ; 440
    BRA     Run_Right_Test
    MOVLW   5
    CPFSGT  SCRATCH
    BRA	    Run_Stop_Test
    BRA	    End_Of_Diagnostics_Codes
    Set_To_Diag_Mode:
	CLRF    HighLvlStateReg
	BSF     HighLvlStateReg,stateDiag
	CLRF    DiagStateReg
	BSF	DiagStateReg,stateStop ; Make motors stop by deafault when Diagnostics button pressed
	BRA	End_Of_Diagnostics_Codes
    Run_Sensor_Test:
	call    CurrentReading
	call    DetermineColor
	call	TransmitSensorData
	GOTO    End_Of_Check_Code
    Run_Forward_Test:
	CLRF    DiagStateReg
	BSF	DiagStateReg,stateForward
	BRA	End_Of_Diagnostics_Codes
    Run_Left_Test:
	CLRF    DiagStateReg
	BSF	DiagStateReg,stateLeft
	BRA	End_Of_Diagnostics_Codes
    Run_Right_Test:
	CLRF    DiagStateReg
	BSF	DiagStateReg,stateRight
	BRA	End_Of_Diagnostics_Codes
    Run_Stop_Test:
	CLRF    DiagStateReg
	BSF	DiagStateReg,stateStop
	BRA	End_Of_Diagnostics_Codes
End_Of_Diagnostics_Codes:
    MOVLW   DiagnosticMode
    MOVWF   TBLPTRL,0
    MOVLW   (DiagnosticMode>>8)
    MOVWF   TBLPTRH,0
    MOVLW   (DiagnosticMode>>16)
    MOVWF   TBLPTRU,0
    call    TransmitMARVResponse
    GOTO    End_Of_Check_Code
;$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
Program_codes:
    MOVF	Code2,W
    ADDLW	-0x30
    MOVWF	SCRATCH
    MOVLW	0
    CPFSGT	SCRATCH
    BRA		Set_To_Program_Mode
    MOVF	Code3,W
    ADDLW	-0x30
    MOVWF	SCRATCH
    MOVLW	0
    CPFSGT	SCRATCH
    BRA		SloganChangeRequest
    MOVLW	2
    CPFSGT	SCRATCH
    BRA		ReplyWithSlogan
    MOVLW	3
    CPFSGT	SCRATCH
    BRA		IndicateSloganRecieved
    GOTO	End_Of_Check_Code
SloganChangeRequest:
    MOVLW	2
    MOVWF	FSR2H
    CLRF	FSR2L
	
    BSF		systemUtilReg,4 ; change all Rx receptions, to go to MessageISR
	
    MOVLW	SloganChangeAck
    MOVWF	TBLPTRL,0
    MOVLW	(SloganChangeAck>>8)
    MOVWF	TBLPTRH,0
    MOVLW	(SloganChangeAck>>16)
    MOVWF	TBLPTRU,0
    call	TransmitMARVResponse
    GOTO	End_Of_Check_Code
ReplyWithSlogan:
    call	ReadFromEEPROM
    call	MemoryToEUART
    BRA		End_Of_Check_Code
IndicateSloganRecieved:
    MOVLW	SloganRecieved
    MOVWF	TBLPTRL,0
    MOVLW	(SloganRecieved>>8)
    MOVWF	TBLPTRH,0
    MOVLW	(SloganRecieved>>16)
    MOVWF	TBLPTRU,0
    call	TransmitMARVResponse
    call	UpdateEEPROM
    GOTO	End_Of_Check_Code
Set_To_Program_Mode:
    CLRF    HighLvlStateReg
    BSF	    HighLvlStateReg,stateProg
    
    MOVLW   ProgramMode
    MOVWF   TBLPTRL,0
    MOVLW   (ProgramMode>>8)
    MOVWF   TBLPTRH,0
    MOVLW   (ProgramMode>>16)
    MOVWF   TBLPTRU,0
    call    TransmitMARVResponse
    GOTO    End_Of_Check_Code
;$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
End_Of_Check_Code:
    return
    
;#######################EUART TRANSMIT SUB ROUTINE##############################
TransmitMARVResponse:
ReadString:
    TBLRD*+ ; read byte from table with code
Wait_for_TX_ready:
    BTFSS   TXSTA2,1,0	; Wait until TSR is empty
    BRA     Wait_for_TX_ready
    
    MOVLW   0
    CPFSEQ  TABLAT; check if 0x00 has been read from table, indicating end of Code
    BRA	    Transmit
    BRA	    End_Of_TransmitResponse
Transmit:
    MOVFF   TABLAT, TXREG2
    BRA	    ReadString
End_Of_TransmitResponse:
    return
;####################
MemoryToEUART:
    MOVLW   2
    MOVWF   FSR2H
    MOVLW   0
    MOVWF   FSR2L
TransmitFromMemory:
    MOVFF   INDF2,SCRATCH
    INCF    FSR2
Wait_for_TX_readyMem:
    BTFSS   TXSTA2,1,0	; Wait until TSR is empty
    BRA     Wait_for_TX_readyMem
    
    MOVFF   SCRATCH,TXREG2
    MOVLW   0x0A ; check for \n
    CPFSEQ  SCRATCH
    BRA	    TransmitFromMemory
End_Of_MemoryToEUART:
    return
    
 ;---------------SUB ROUTINE FOR SENSOR DATA DIAGNOSTICS-------------------------
TransmitSensorData:
    LFSR    2,ColorReg3
    MOVLW   6
    MOVWF   Count1
    
    MOVLW   '*'
    MOVWF   SCRATCH
    BRA	    Wait_for_TX_readySens
ReadColorData:
    MOVFF   POSTINC2, SCRATCH
    MOVLW   0x30
    ADDWF   SCRATCH
Wait_for_TX_readySens:
    BTFSS   TXSTA2,1,0  ; Wait until TSR is empty
    BRA	    Wait_for_TX_readySens
    
    MOVFF   SCRATCH,TXREG2
    DECFSZ  Count1
    BRA	    ReadColorData
    
    MOVLW   0x0D ; \r
    MOVWF   SCRATCH
Wait_for_TX_readyCarridge:
    BTFSS   TXSTA2,1,0  ; Wait until TSR is empty
    BRA	    Wait_for_TX_readyCarridge
    
    MOVFF   SCRATCH, TXREG2
    
    MOVLW   0x0A ; \n
    MOVWF   SCRATCH
Wait_for_TX_newLine:
    BTFSS   TXSTA2,1,0  ; Wait until TSR is empty
    BRA	    Wait_for_TX_newLine
    
    MOVFF   SCRATCH, TXREG2
    
End_Of_TransmitSensorData:
    return
;-------------------------------------------------------------------------------
;#--------------------------END OF SUB ROUTINE----------------------------------
    
;########################CALIBRATION SUB ROUTINES################################
Calibrate:
    MOVLW   3
    CPFSLT  CLRCNT ; calibrating Red
    BRA	    FlashRed
    DECF    WREG
    CPFSLT  CLRCNT ; calibrating Green
    BRA	    FlashGreen
    DECF    WREG
    CPFSLT  CLRCNT ; calibrating Blue
    BRA	    FlashBlue
    NoCalibrate:
	BRA	    End_Of_Calibrate
    FlashRed:
	MOVLW	    RED
	BRA	    Flash
    FlashGreen:
	MOVLW	    GREEN
	BRA	    Flash
    FlashBlue:
	MOVLW	    BLUE
    Flash:
	MOVWF	    PORTE
    SensorItr:
	MOVF	    CLRCNT, W
	SUBLW	    3 ; Determine value to offset THRESH3 (3 - CLRCNT)
	ADDLW	    THRESH3
	MOVWF	    FSR0L ; MAKE FSR0 point to THRESH register : {R0, G0, B0}
	call	    longDelay
	MOVLW	    3
	MOVWF	    ADCONSEL
	MOVLW	    5
	MOVWF	    Count1 ; Used to iterate over the 5 sensors
	loopSensors:
	    call	loadADCON
	    call	readAndAverage
	    MOVWF	INDF0;move the ADC reading to register that FSR0 points to
	    MOVLW	3
	    ADDWF	FSR0L,F;Point to the next sensor register address
	    INCF	ADCONSEL;Increment so that in next loop, using next channel
	    DECFSZ	Count1
	    BRA		loopSensors
End_Of_Calibrate:
    DECF    CLRCNT
    CLRF    PORTE
    return
;#######################END OF SUB ROUTINE######################################
    
;###############################ADC SUB ROUTINES################################
readAndAverage: ; Takes consecutive reads on a sensors and then right shifts to divide
	MOVLW   8
	MOVWF   Count2
	CLRF    SMPL0
	CLRF    SMPL1
    CALloop:
	call    ADCread
	ADDWF   SMPL0,F
	BTFSC   STATUS,0
	INCF    SMPL1,F
	DECFSZ  Count2
	BRA	CALloop
    
	RRCF	SMPL1
	RRCF	SMPL0
	RRCF	SMPL1
	RRCF	SMPL0
	RRCF	SMPL1
	RRCF	SMPL0
	MOVF	SMPL0,W
    return
    
    ADCread:
    Poll:	
	BSF 	GO 		; Start conversion
	BTFSC 	GO 		; Is conversion done?
	BRA 	$-2 
    
	MOVF    ADRESH,W
    EndOfADCread:
	return
	
;-----------------SUB ROUTINE FOR Loading ADCON0 register-----------------------
loadADCON:
    MOVLW	7
    CPFSLT	ADCONSEL
    BRA		load7
    MOVLW	6
    CPFSLT	ADCONSEL
    BRA		load6
    MOVLW	5
    CPFSLT	ADCONSEL
    BRA		load5
    MOVLW	4
    CPFSLT	ADCONSEL
    BRA		load4
    BRA		load3
	
    load7:
	MOVLW	C7
	MOVWF	ADCON0
	BRA	endLoadADCON
    load6:
	MOVLW	C6
	MOVWF	ADCON0
	BRA	endLoadADCON
    load5:
	MOVLW	C5
	MOVWF	ADCON0
	BRA	endLoadADCON
    load4:
	MOVLW	C4
	MOVWF	ADCON0
	BRA	endLoadADCON
    load3:
	MOVLW	C3
	MOVWF	ADCON0
	BRA	endLoadADCON
endLoadADCON:
    return
;#---------------------------END OF ADC SUB ROUTINES----------------------------
    
;###############################DELAY SUB ROUTINES##############################
Delay333:
    MOVLW   21     
    MOVWF   Count3
    DelayLoop3:
	MOVLW   90 
	MOVWF   Count1
    DelayLoop1:
	MOVLW   183    
	MOVWF   Count2
    DelayLoop2:
	NOP            
	DECFSZ  Count2, f  
	BRA    DelayLoop2
	DECFSZ  Count1, f  
	BRA    DelayLoop1
	DECFSZ  Count3, f
	BRA    DelayLoop3 
    return           
    
longDelay: ; 3 Seconds
    MOVLW   80     
    MOVWF   Count3
    longLoop0:
	MOVLW   200 
	MOVWF   Count1
    longLoop1:
	MOVLW   220    
	MOVWF   Count2
    longLoop2:
	NOP            
	DECFSZ  Count2, f  
	BRA	longLoop2
	DECFSZ  Count1, f  
	BRA     longLoop1
	DECFSZ  Count3, f
	BRA     longLoop0 
    return
    
delay4ms:
    ;TODO: write delay sub routine for 4ms
    return
    
Delay6ms:
    MOVLW   100
    MOVWF   Count1
OuterLoop:
    MOVLW   94
    MOVWF   Count2
InnerLoop:
    DECFSZ  Count2
    BRA	    InnerLoop
    DECFSZ  Count1
    BRA	    OuterLoop
    return 
;#------------------------END OF DELAY SUB ROUTINES-----------------------------
    
;#########################RACE RELATED SUB ROUTINES#############################
Idle:; Idle is also called when moving the MARV to home mode, Idle is called to 
     ; switch off the motors, incase home is requested whilst the MARV is moving
    CLRF	CCP1CON
    CLRF	CCP2CON
    MOVLW	0
    MOVWF	CCPR1L
    MOVWF	CCPR2L
    BCF		TMR2ON ; stops timer
    CLRF	TMR2 ; resets the timer register
    BCF		PORTC,1
    BCF		PORTC,2
    BCF		PORTD,2
    BCF		PORTD,5
     
    BCF		systemUtilReg,2 ; indicate MARV is idle
    BTFSC	HighLvlStateReg,stateRace 
    BRA		IdleTouchSensing
    BTFSC	HighLvlStateReg2,offlineRace
    BRA		IdleTouchSensing
    BRA		End_Of_Idle
    
IdleTouchSensing:
    BCF		systemUtilReg,2 ; indicate that MARV is Idle, once again
    BCF		systemUtilReg,3
    call	Touchsensor
    call	Touchsensor
    BTFSS	systemUtilReg,3
    BRA		End_Of_Idle
    BSF		systemUtilReg,2 ; MARV is going into drive
    BCF		systemUtilReg,3 ; acknowledge touch

    ;switch on Motors
    MOVLW	00001100B
    MOVWF	CCP1CON
    MOVWF	CCP2CON
    MOVLW	PA
    MOVWF	PSTR1CON
    MOVWF	PSTR2CON
    BCF		PORTD,5
    BCF		PORTD,2
    MOVLW	75; set MotorR and MotorL to 30%
    MOVWF	CCPR1L
    MOVWF	CCPR2L
    BSF		TMR2ON
End_Of_Idle:
    return
    
Touchsensor:
    MOVLW	120
    MOVWF	TouchThresh
    BCF		TRISD,4 ;Make touch pin output
    BSF		PORTD,3 ; Charge Secondary line
    MOVLW	D3
    MOVWF	ADCON0
    NOP
    NOP
    
    BSF		TRISD,4 ; Make touch pin input
    MOVLW	D4
    MOVWF	ADCON0 ; Point ADC to touch pin
    BCF		PORTD,3 ; Ground secondary line
    call	ADCread
    CPFSLT	TouchThresh
    BSF		systemUtilReg,3 ; flag that a touch was sensed
    BCF		TRISD,4 ; Make touch pin output 
    return
showRaceColor:
    MOVLW   0
    CPFSGT  followColor
    BRA	    displayBlackRace
    INCF    WREG
    CPFSGT  followColor
    BRA	    displayRedRace
    MOVLW   3
    CPFSGT  followColor
    BRA	    displayGreenRace
    displayBlueRace:
	MOVLW	11100110B
	MOVWF	PORTA
	BRA	End_Of_showRaceColor
    displayBlackRace:
	MOVLW   01101110B
	MOVWF   PORTA
	BRA	End_Of_showRaceColor
    displayRedRace:
	MOVLW	01000010B
	MOVWF	PORTA
	BRA	End_Of_showRaceColor
    displayGreenRace:
	MOVLW	10111110B
	MOVWF	PORTA
End_Of_showRaceColor:
    return
    
CurrentReading:
    ;Performs Color strobing and reads ADC after each color is shone
	MOVLW	RED
	MOVWF	LATE
	LFSR	1,CURRENT3; point to red register
	call	read5Sensors
	MOVLW	GREEN
	MOVWF	LATE
	MOVLW	CURRENT3
	ADDLW	1;point to green register
	MOVWF	FSR1L
	call	read5Sensors
	MOVLW	BLUE
	MOVWF	LATE
	MOVLW	CURRENT3
	ADDLW	2
	MOVWF	FSR1L;point to blue register
	call	read5Sensors
	CLRF	LATE
    return
    
read5Sensors:
    MOVLW   3
    MOVWF   ADCONSEL
    MOVLW   5
    MOVWF   Count1
loop5Sensors:
    call    loadADCON
    call    readAndAverage
    MOVWF   INDF1;move the ADC reading to register that FSR0 points to
    MOVLW   3
    ADDWF   FSR1L,F;Point to the next sensor register address
    INCF    ADCONSEL
    DECFSZ  Count1
    BRA	    loop5Sensors
    return
    
DetermineColor:
    LFSR	0,CURRENT3
    LFSR	1,THRESH3
    call	DtrClrHelper
    MOVWF	ColorReg3
	
    LFSR	0,CURRENT4
    LFSR	1,THRESH4
    call	DtrClrHelper
    MOVWF	ColorReg4
	
    LFSR	0,CURRENT5
    LFSR	1,THRESH5
    call	DtrClrHelper
    MOVWF	ColorReg5
	
    LFSR	0,CURRENT6
    LFSR	1,THRESH6
    call	DtrClrHelper
    MOVWF	ColorReg6
	
    LFSR	0,CURRENT7
    LFSR	1,THRESH7
    call	DtrClrHelper
    MOVWF	ColorReg7
    return

DtrClrHelper:
    CLRF	SCRATCH
    MOVLW	5
    SUBWF	INDF1,W
    CPFSLT	INDF0; determine if sensor is Red
    BSF		SCRATCH,0
	
    MOVLW	1
    ADDWF	FSR0L,F
    ADDWF	FSR1L,F
    MOVLW	7
    SUBWF	INDF1,W; determine if sensor is Green
    CPFSLT	INDF0
    BSF		SCRATCH,1
	
    MOVLW	1
    ADDWF	FSR0L,F
    ADDWF	FSR1L,F
    MOVLW	2
    SUBWF	INDF1,W
    CPFSLT	INDF0
    BSF		SCRATCH,2; FSR0 ends on current Blue
    
    MOVF	SCRATCH,W
    SUBLW	3
    BZ		ResolveRedGreen
    MOVF	SCRATCH,W
    SUBLW	5
    BZ		ResolveRedBlue
    MOVF	SCRATCH,W
    SUBLW	6
    BZ		ResolveGreenBlue
    BRA		End_Of_DtrClrHelper

ResolveRedGreen:
    CLRF	SCRATCH
    DECF	FSR0L ; Make FSR0 point to current Green
    MOVF	INDF0,W ; Move current green to WREG
    DECF	FSR0L ; Make FSR0 point to current Red
    CPFSGT	INDF0 ; currentRed > currentGreen ? 
    BRA		GreenWinRed
    RedWinGreen:
	BSF		SCRATCH,0
	BRA		End_Of_DtrClrHelper
    GreenWinRed:
	BSF		SCRATCH,1
	BRA		End_Of_DtrClrHelper
ResolveRedBlue:
    CLRF	SCRATCH
    MOVF	INDF0,W ; Move current Blue to WREG
    DECF	FSR0L ; FSR0 points to Green
    DECF	FSR0L ; FSR0 point to Red
    CPFSGT	INDF0 ; currentRed > currentBlue ? 
    BRA		BlueWinRed
    RedWinBlue:
	BSF		SCRATCH,0
	BRA		End_Of_DtrClrHelper
    BlueWinRed:
	BSF		SCRATCH,2
	BRA		End_Of_DtrClrHelper
ResolveGreenBlue:
    CLRF	SCRATCH
    MOVF	INDF0,W ; Move current Blue to WREG
    DECF	FSR0L ; FSR0 points to current Green
    CPFSGT	INDF0 ; currentGreen > currentBlue
    BRA		BlueWinGreen
    GreenWinBlue:
	BSF		SCRATCH,1
	BRA		End_Of_DtrClrHelper
    BlueWinGreen:
	BSF		SCRATCH,2
	BRA		End_Of_DtrClrHelper
End_Of_DtrClrHelper:
    MOVF	SCRATCH,W
    return
    
lineMapper:
    CLRF    stopReg
    CLRF    SensorLine
    Sensor3:
	MOVLW   0
	CPFSGT  ColorReg3 ; test if on black
	BSF	stopReg,4

	MOVF    followColor,W
	ADDLW   1
	CPFSLT  ColorReg3
	BRA	Sensor4
	MOVF    lowerClr,W
	CPFSLT  ColorReg3
	BSF	SensorLine,4
    Sensor4:
	MOVLW   0
	CPFSGT  ColorReg4 ; test if on black
	BSF	stopReg,3

	MOVF    followColor,W
	ADDLW   1
	CPFSLT  ColorReg4
	BRA	Sensor5
	MOVF    lowerClr,W
	CPFSLT  ColorReg4
	BSF	SensorLine,3
    Sensor5:
	MOVLW   0
	CPFSGT  ColorReg5 ; test if on black
	BSF	stopReg,2

	MOVF    followColor,W
	ADDLW   1
	CPFSLT  ColorReg5
	BRA	Sensor6
	MOVF    lowerClr,W
	CPFSLT  ColorReg5
	BSF	SensorLine,2
    Sensor6:
	MOVLW   0
	CPFSGT  ColorReg6 ; test if on black
	BSF	stopReg,1

	MOVF    followColor,W
	ADDLW   1
	CPFSLT  ColorReg6
	BRA	Sensor7
	MOVF    lowerClr,W
	CPFSLT  ColorReg6
	BSF	SensorLine,1
    Sensor7:
	MOVLW   0
	CPFSGT  ColorReg7 ; test if on black
	BSF	stopReg,0

	MOVF    followColor,W
	CPFSLT  ColorReg7
	BRA	endOfLineMapper
	MOVF    lowerClr,W
	CPFSLT  ColorReg7
	BSF	SensorLine,0
endOfLineMapper:
    return
    
LLI:
    CLRF    LLIStateReg
    CLRF    PORTB
    MOVLW   31
    CPFSEQ  stopReg ; 31 = 11111b means all sensors are on Black
    BRA	    Evaluations
Stop:
    BCF	    systemUtilReg,2 ; set it to go back to Idle
    BSF	    LLIStateReg,5
    GOTO    endOfLLI
Evaluations: 
    MOVLW   31
    CPFSLT  SensorLine
    BRA	    EvalLost
    BTFSC   SensorLine,0
    BRA	    EvalExRight
    BTFSC   SensorLine,4
    BRA	    EvalExLeft
    BTFSC   SensorLine,1
    BRA	    EvalRight
    BTFSC   SensorLine,3
    BRA	    EvalLeft
EvalStraight:
    BSF	    PORTB,3
    BSF	    LLIStateReg,0
    BRA	    endOfLLI
EvalLeft:
    BSF	    PORTB,2
    BSF	    LLIStateReg,1
    BRA	    endOfLLI
EvalRight:
    BSF	    PORTB,4
    BSF	    LLIStateReg,2
    BRA	    endOfLLI
EvalExLeft:
    BSF	    PORTB,1
    BSF	    LLIStateReg,3
    BRA    endOfLLI
EvalExRight:
    BSF	    PORTB,5
    BSF	    LLIStateReg,4
    BRA	    endOfLLI
EvalLost:
    BSF	    LLIStateReg,6
endOfLLI:
    return
    
MotorControl:;LLIStateReg bits | 0 : Straight | 1 : Left | 2 : Right | 3 : ExLeft | 4 : ExRight| 5 : Stop | 6 : Lost
    BTFSC LLIStateReg,0
    BRA	  MC_STRAIGHT
    
    BTFSC LLIStateReg,1
    BRA	  MC_LEFT
    
    BTFSC LLIStateReg,2
    BRA	  MC_RIGHT
    
    BTFSC LLIStateReg,3
    BRA   MC_ExLEFT
    
    BTFSC LLIStateReg,4
    BRA   MC_ExRIGHT
    
    BTFSC LLIStateReg,5
    BRA   MC_STOP
    
    BTFSC LLIStateReg,6
    BRA   MC_LOST
    
MC_STRAIGHT:
    MOVLW	PA
    MOVWF	PSTR1CON
    MOVWF	PSTR2CON
    BCF		PORTD,5
    BCF		PORTD,2
    MOVLW	75; set MotorR and MotorL to 30%
    MOVWF	CCPR1L
    MOVWF	CCPR2L
    BRA	    End_Of_MotorControl
    
MC_LEFT:
    MOVLW	PA
    MOVWF	PSTR1CON
    MOVWF	PSTR2CON
    BCF		PORTD,5
    BCF		PORTD,2
    
    LeftStateMotorR:
	MOVLW	85
	CPFSLT	CCPR2L
	BRA	LeftStateMotorL
	MOVLW	1
	ADDWF	CCPR2L,f
    LeftStateMotorL:
	MOVLW	50
	CPFSGT	CCPR1L
	BRA	End_Of_MotorControl
	MOVLW	1
	SUBWF	CCPR1L,f
	BRA	End_Of_MotorControl

MC_RIGHT:
    MOVLW	PA
    MOVWF	PSTR1CON
    MOVWF	PSTR2CON
    BCF		PORTD,5
    BCF		PORTD,2
    
    RightStateMotorR:
	MOVLW	50
	CPFSGT	CCPR2L
	BRA	RightStateMotorL
	MOVLW	1
	SUBWF	CCPR2L,f
    RightStateMotorL:
	MOVLW	85
	CPFSLT	CCPR1L
	BRA	End_Of_MotorControl
	MOVLW	1
	ADDWF	CCPR1L, f
	BRA	End_Of_MotorControl
    
MC_ExLEFT:
    ExLeftStateMotorR:
	MOVLW	PA
	MOVWF	PSTR2CON
	BCF	PORTD,2
	MOVLW	85
	CPFSLT	CCPR2L
	BRA	ExLeftStateMotorL
	MOVLW	5
	ADDWF	CCPR2L,f
    ExLeftStateMotorL:
	MOVLW	PB
	MOVWF	PSTR1CON
	BCF	PORTC,2
	MOVLW	40
	CPFSLT	CCPR1L
	BRA	End_Of_MotorControl
	MOVLW	5
	ADDWF	CCPR1L,f
	BRA	End_Of_MotorControl
	
MC_ExRIGHT:
    ExRightStateMotorR:
	MOVLW	PB
	MOVWF	PSTR2CON
	BCF	PORTC,1
	MOVLW	40
	CPFSLT	CCPR2L
	BRA	ExRightStateMotorL
	MOVLW	5
	ADDWF	CCPR2L,f
    ExRightStateMotorL:
	MOVLW	PA
	MOVWF	PSTR1CON
	BCF	PORTD,5
	MOVLW	85
	CPFSLT	CCPR1L
	BRA	End_Of_MotorControl
	MOVLW	5
	ADDWF	CCPR1L,f
	BRA	End_Of_MotorControl
MC_STOP:
    CLRF	CCP1CON
    CLRF	CCP2CON
    MOVLW	0
    MOVWF	CCPR1L
    MOVWF	CCPR2L
    BCF		TMR2ON ; stops timer
    CLRF	TMR2 ; resets the timer register
    BCF		PORTC,1
    BCF		PORTC,2
    BCF		PORTD,5
    BCF		PORTD,2
    BRA	    End_Of_MotorControl
MC_LOST:
    MOVLW	PA
    MOVWF	PSTR1CON
    MOVWF	PSTR2CON
    BCF		PORTD,5
    BCF		PORTD,2
    MOVLW	75; set MotorR and MotorL to 30%
    MOVWF	CCPR1L
    MOVWF	CCPR2L
End_Of_MotorControl:
    return
;###############################################################################
;---------------------SUB ROUTINES FOR I2C--------------------------------------
UpdateEEPROM:
    MOVLW   2
    MOVWF   FSR2H
    MOVLW   0
    MOVWF   FSR2L ; point to the top of bank 2
    MOVLW   0
    MOVWF   PageAddress
PageLoop:
    MOVLW   8
    MOVWF   PageCTR
    TransmitionStart:
	call    InitiateStartCondition
	MOVLW   WriteByte
	MOVWF   SCRATCH
	call    TransmitByte
	MOVFF   PageAddress, SCRATCH
	call    TransmitByte
    
    TransmitSloganFromMem:
	MOVF    POSTINC2,W
	MOVWF   SCRATCH
	call    TransmitByte

	INCF    PageAddress
	DECFSZ  PageCTR
	BRA	StillInPage
	BRA	TransmitStop
    StillInPage:
	MOVLW   0
	CPFSEQ  INDF2 ; if \0 is reached in memory 
	BRA	TransmitSloganFromMem
	
TransmitStop:
    call    InitiateStopCondition
    call    Delay6ms
    MOVLW   0
    CPFSEQ  INDF2 ; if \0 is reached in memory
    BRA	    PageLoop
    return
;-------------------------------------------------------------------------------    
InitiateStartCondition:
waitForEmptyBufferToStart:
    BTFSC   SSP2STAT, 0
    BRA	    waitForEmptyBufferToStart
    BCF	    SSP2IF
    BSF	    SSP2CON2,0 ; generate start
waitForStart:
    BTFSC   SSP2CON2,0
    BRA	    waitForStart
    return
    
InitiateStopCondition:
waitForEmptyBufferToStop:
    BTFSC   SSP2STAT, 0
    BRA	    waitForEmptyBufferToStop
    
    BCF	    SSP2IF
    BSF	    SSP2CON2, 2 ; generate stop condition 
waitForStop:
    BTFSC   SSP2CON2, 2
    BRA	    waitForStop
    return
    
InitiateRestartCondition:
waitForEmptyBufferToRestart:
    BTFSC   SSP2STAT, 0
    BRA	    waitForEmptyBufferToRestart
    
    BCF	    SSP2IF
    BSF	    SSP2CON2, 1 ; generate restart condition 
waitForRestart:
    BTFSC   SSP2CON2, 1
    BRA	    waitForRestart
    return
;-------------------------------------------------------------------------------
TransmitByte:
    WaitForEmptyBuf:
    BTFSC   SSP2STAT,0
    BRA	    WaitForEmptyBuf
    
    MOVFF   SCRATCH, SSP2BUF
    BCF	    SSP2IF
    WaitFor9thClockGen:
    BTFSS   SSP2IF
    BRA	    WaitFor9thClockGen
    BCF	    SSP2IF
    return   
    
ReadFromEEPROM:
    MOVLW   2
    MOVWF   FSR2H
    MOVLW   0
    MOVWF   FSR2L ; point to the top of bank 2
    MOVLW   0
    
    call    InitiateStartCondition
    MOVLW   WriteByte
    MOVWF   SCRATCH
    call    TransmitByte
    MOVLW   0 ; point EEPROM PTR to top 
    MOVWF   SCRATCH
    call    TransmitByte
    call    InitiateRestartCondition
    MOVLW   ReadByte
    MOVWF   SCRATCH
    call    TransmitByte
LoopReadFromEEPROM:
    BSF	    SSP2CON2,3 ; enable recieve
    BCF	    SSP2IF
WaitForReceptionFinish:
    BTFSS   SSP2IF
    BRA	    WaitForReceptionFinish
    BCF	    SSP2IF
    MOVFF   SSP2BUF, SCRATCH
    MOVFF   SCRATCH, POSTINC2
    MOVLW   0x0A ; check for \n
    CPFSEQ  SCRATCH
    BRA	    Acknowledge
    BRA	    Nacknowledge
Acknowledge:
    BCF	    SSP2CON2, 5 ; set the ack bit to 0
    BSF	    SSP2CON2, 4 ; initiate ack sequence 
    BCF	    SSP2IF
    BRA	    WaitForAckToFinish
Nacknowledge:
    BSF	    SSP2CON2, 5 ; set the ack bit to 1
    BSF	    SSP2CON2, 4 ; initiate ack sequence
    BCF	    SSP2IF
WaitForAckToFinish:
    BTFSS   SSP2IF
    BRA	    WaitForAckToFinish
    
    BTFSS   SSP2CON2, 5 ; if NACK was sent, skip to stop condition 
    BRA	    LoopReadFromEEPROM
TransmitStop2:
    call    InitiateStopCondition
    return
;###############################################################################
;------------------------END OF I2C SUB ROUTINES--------------------------------
    
;------------------------DIAGNOSTICS SUB ROUTINES-------------------------------
DiagForward:
    BTFSS	systemUtilReg,5
    BRA		MotorSetUpF
    BRA		ForwardConfig
MotorSetUpF:
    MOVLW	00001100B
    MOVWF	CCP1CON
    MOVWF	CCP2CON
    BSF		systemUtilReg,5
ForwardConfig:
    MOVLW	PA
    MOVWF	PSTR1CON
    MOVWF	PSTR2CON
    BCF		PORTD,5
    BCF		PORTD,2
    MOVLW	75; set MotorR and MotorL to 30%
    MOVWF	CCPR1L
    MOVWF	CCPR2L
    BSF		TMR2ON
    return 
    
DiagRight:
    BTFSS	systemUtilReg,5
    BRA		MotorSetUpR
    BRA		RightConfig
MotorSetUpR:
    MOVLW	00001100B
    MOVWF	CCP1CON
    MOVWF	CCP2CON
    BSF		systemUtilReg,5
RightConfig:
    MOVLW	PA
    MOVWF	PSTR1CON
    BCF		PORTD,5
    MOVLW	PB
    MOVWF	PSTR2CON
    BCF		PORTC,1
    MOVLW	75           
    MOVWF	CCPR1L
    MOVLW	38         
    MOVWF	CCPR2L
    BSF		TMR2ON   
    return
    
DiagLeft:
    BTFSS	systemUtilReg,5
    BRA		MotorSetUpL
    BRA		LeftConfig
MotorSetUpL:
    MOVLW	00001100B
    MOVWF	CCP1CON
    MOVWF	CCP2CON
    BSF		systemUtilReg,5
LeftConfig:
    MOVLW	PB
    MOVWF	PSTR1CON
    BCF		PORTC,2
    MOVLW	PA
    MOVWF	PSTR2CON
    BCF		PORTD,2
    MOVLW	38
    MOVWF	CCPR1L
    MOVLW	75
    MOVWF	CCPR2L
    BSF		TMR2ON
    return
    
DiagStop:
    CLRF	CCP1CON
    CLRF	CCP2CON
    MOVLW	0
    MOVWF	CCPR1L
    MOVWF	CCPR2L
    BCF		TMR2ON ; stops timer
    CLRF	TMR2 ; resets the timer register
    BCF		PORTC,1
    BCF		PORTC,2
    BCF		PORTD,2
    BCF		PORTD,5
    BCF		systemUtilReg,5
    return
;-------------------------------------------------------------------------------
;-------------------------------------------------------------------------------
ChangeRaceColor:
    MoveOutOfBlue:
	MOVLW	6
	CPFSEQ	followColor ; currently following Blue
	BRA	MoveOutOfGreen
	MOVLW	3
	MOVWF	followColor ; make MARV follow Green line now
	MOVLW	2
	MOVWF	lowerClr
	BRA	End_Of_ChangeRaceColor
    MoveOutOfGreen:
	MOVLW	3
	CPFSEQ	followColor ; currently following Green
	BRA	MoveOutOfRed
	MOVLW	1
	MOVWF	followColor ; make MARV	follow Red line
	MOVWF	lowerClr
	BRA	End_Of_ChangeRaceColor
    MoveOutOfRed:
	MOVLW	1
	CPFSEQ	followColor ; currently following Red
	BRA	MoveOutOfBlack
	MOVLW	0
	MOVWF	followColor ; make MARV follow Black line
	MOVWF	lowerClr
	BRA	End_Of_ChangeRaceColor
    MoveOutOfBlack:
	MOVLW	6
	MOVWF	followColor ; make MARV follow Black line
	MOVLW	4
	MOVWF	lowerClr
End_Of_ChangeRaceColor:
    return
;-------------------------------------------------------------------------------
    
; Code responses for the MARV to send, are stored in a table in Program memory
ORG 0x5000 
    
HomeMode:		DB	'#001',0x0D,0x0A,0x00
SelectColorMode:	DB	'#101',0x0D,0x0A,0x00
CalibrationMode:	DB	'#201',0x0D,0x0A,0x00
CalRed:			DB	'#211',0x0D,0x0A,0x00
CalGreen:		DB	'#221',0x0D,0x0A,0x00
CalBlue:		DB	'#231',0x0D,0x0A,0x00
CalDone:		DB	'#241',0x0D,0x0A,0x00
RaceMode:		DB	'#301',0x0D,0x0A,0x00
CurrIdle:		DB	'#361',0x0D,0x0A,0x00
CurrDrive:		DB	'#371',0x0D,0x0A,0x00
RaceRed:		DB	'#311',0x0D,0x0A,0x00
RaceGreen:		DB	'#321',0x0D,0x0A,0x00
RaceBlue:		DB	'#331',0x0D,0x0A,0x00
RaceBlack:		DB	'#341',0x0D,0x0A,0x00
DiagnosticMode:		DB	'#401',0x0D,0x0A,0x00
ProgramMode:		DB	'#501',0x0D,0x0A,0x00
SloganChangeAck:	DB	'#511',0x0D,0x0A,0x00
SloganRecieved:		DB	'#514',0x0D,0x0A,0x00
		
    END   