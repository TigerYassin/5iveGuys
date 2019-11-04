;Write an assembly program to check for a constant radius away from the robot

;lets pretend the robot has triggered and pivoted to face perpendictular to the robot.
;this would mean that the intial dy = 0
;and the initial x distance would be the value of the alarm triggered
;lets pretend that the robot starts out with sonar 5 facing the reflector
;
;To use: store dX and dY in global variables AtanX and AtanY.                 ;
; Call Atan2                                                                   ;
; Result (angle [0,359]) is returned in AC  
; I will call AtanY to be 0 
; and I will call AtanX to be negative the distance between sonar 5 and the xpos 
StartCircling:
      
	   
    OUT RESETPOS ;resets the odometry to 0,0,0 
	;then set xnot and ynot to be zero 
	LOAD XPOS
	STORE XNOT
	LOAD YPOS
	STORE YNOT
	LOAD Ft1andHalf ;loads the original distance between sonar 5 and a relector ;we might want to change this to be a known value like 1 foot 
	STORE DeltaX
	;i will only look at the x value changing but this can be changed
	LOAD FSlow ; go ahead slowly
	STORE DVel 
	LOADI -45
	STORE DTheta ; turn the robot right 45 degrees
	; I will enable the front sonars to check if anything gets within 6 inches 
	SEI  &B0001
	LOAD   &B00111000 ;I will enable sonars 345 
    OUT    SONAREN
	LOAD HalfFoot ;set the alarm distance to 1 foot
	OUT SONALARM
CheckDist: ;sets the alarm distance to one foot
	IN  SONALARM
	STORE AlarmTrigger
	JPOS StopAndDisplay ; if a bit has been set to positive then jump to another subroutine
	     ;The sonar that it came from is in the accumulator 
    JUMP  CheckDeltaX
	
CheckDeltaX:
    LOAD XPOS ; load the current xpos from odometry i assume it should increase initial
	Call Abs
	SUB DeltaX ;subtract deltaX
	JPOS Turn  
	JUMP CheckDist ;check the alarm distance
Turn:
    OUT RESETPOS
    LOAD Theta 
	ADDI -45
	STORE DTheta
    LOAD XPOS
	STORE XNOT
	LOAD YPOS
	STORE YNOT
	LOAD Ft1andHalf ;loads the original distance between sonar 5 and a relector ;we might want to change this to be a known value like 1 foot 
	STORE DeltaX
    JUMP CheckDeltaX	; turn theta 45 degrees to the right 
	
StopAndDisplay:
    LOADI 0
	STORE DVel ; stop the robot movement
	LOAD AlarmTrigger
	OUT SSEG1 ; out which sonar triggered the alarm 
	

	
	
	
    
	
	
	;set the differential x to be the alarm trigger distance
	
XNOT: DW 0
YNOT: DW 0
DeltaX: DW 0
Ft1: DW 290 ; this is in xpos units 1.05mm/count
Ft1andHalf: DW 435 
HalfFoot: DW 145
	   
  