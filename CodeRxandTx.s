    title "Testing idea of state machine with UART code comm"
    
    PROCESSOR 18F45K22
    
    CONFIG FOSC = INTIO67
    CONFIG WDTEN = OFF
    
    systemUtilReg	EQU	0x00; Bit0: Code Recieved, Bit1: Calibrate, Bit2: 0 = Idle, 1 = Driving
    HighLvlStateReg	EQU	0x01
    Code1		EQU	0x02
    Code2		EQU	0x03
    Code3		EQU	0x04
    CodeCTR		EQU	0x05
    RxSCRATCH		EQU	0x06
    SCRATCH		EQU	0x07
    WTEMP		EQU	0x08

		
    followColor		EQU	0x09 ; Determines curren color being followed
		
    Count1		EQU	0x0A
    Count2		EQU	0x0B
    Count3		EQU	0x0C
    CLRCNT		EQU	0x0D; used to iterate over 3 colors in Calibration
	
    ;state  bits--------------
    stateHome	EQU	0
    stateSelC	EQU	1
    stateCal	EQU	2
    stateRace	EQU	3
    ;-------------------------
    
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
    
    ;Config IO Pins for EUART
    CLRF    PORTD
    CLRF    LATD
    CLRF    ANSELD
    MOVLW   11000000B
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
    
    ;Config Interupts 
    MOVLW   11000000B
    MOVWF   INTCON
    MOVLW   00100000B ; RC2IE 
    MOVWF   PIE3
    CLRF    PIR3
    
    MOVLB   0x00
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
	BRA	    End_Of_State_Cal
    State_Cal_Green:
	BSF	    PORTB,4
	call	    Calibrate
	BRA	    End_Of_State_Cal
    State_Cal_Blue:
	BSF	    PORTB,3
	call	    Calibrate
End_Of_State_Cal:
    call    Check_Code_Recieved
;^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
STATE_RACE:
    BTFSS   HighLvlStateReg,stateRace
    GOTO    Main
    MOVLW   11110010B
    MOVWF   PORTA
    ;Write code for state
    call    Delay333
    call    Check_Code_Recieved
    GOTO    Main ; final loop catch statement
;0000000000000000000000000000000000000000000000000000000000000000000000000000000
    
ISR:
    RETFIE
    
RxISR:
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
    GOTO    Race_codes ; replace this line for checks to other codes
;$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$    
Home_codes:
    ;MARV transistion to Home, and responds with Home Mode code
    MOVLW   HomeMode
    MOVWF   TBLPTRL,0
    MOVLW   (HomeMode>>8)
    MOVWF   TBLPTRH,0
    MOVLW   (HomeMode>>16)
    MOVWF   TBLPTRU,0
    
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
    BRA	    End_Of_Select_Color_Codes
SetColorToRed:
    MOVLW   1
    MOVWF   followColor ; followColor = 1 : Red
    BRA	    End_Of_Select_Color_Codes
SetColorToGreen:
    MOVLW   3
    MOVWF   followColor ; followColor = 3 : Green
    BRA	    End_Of_Select_Color_Codes
SetColorToBlue:
    MOVLW   6
    MOVWF   followColor ; followColor = 6 : Blue
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
;#--------------------------END OF SUB ROUTINE----------------------------------
    
;########################CALIBRATION SUB ROUTINE################################
Calibrate:
    call    Delay333
    DECF    CLRCNT
    return
    
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
    GOTO    DelayLoop2
    DECFSZ  Count1, f  
    GOTO    DelayLoop1
    DECFSZ  Count3, f
    GOTO    DelayLoop3 
    return           

    
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
DiagnostMode:		DB	'#401',0x0D,0x0A,0x00
ProgramMode:		DB	'#501',0x0D,0x0A,0x00
SloganChangeAck:	DB	'#511',0x0D,0x0A,0x00

    END   