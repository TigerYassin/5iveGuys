;write an assembly program that uses sonar interrupts in order to turn in a circle 
;lets pretend that we start by facing toward the reflector 

;I need to figure out how to use, sonaren, sonarint, and sonalarm all together
;sonarint 
Main:
	OUT    RESETPOS    ; reset the odometry to 0,0,0
	; configure timer interrupt for the movement control code
	LOADI  10          ; period = (10 ms * 10) = 0.1s, or 10Hz.
	OUT    CTIMER      ; turn on timer peripheral
	SEI    &B0010      ; enable interrupts from source 2 (timer)
	; at this point, timer interrupts will be firing at 10Hz, and
	; code in that ISR will attempt to control the robot.
	; If you want to take manual control of the robot,
	; execute CLI &B0010 to disable the timer interrupt.


;fiurst use the odometry to save your initial x and y position 
;my goal is to first turn the robot 90 degrees
;if the back left sonar triggers aka sonar 7., then add 54 degrees to the current theta
;let this happen 7 times then you know the circle has completed 


;first i will assume that the robot is facing the reflector it needs to circle 
TurnToBeginTurning:
     LOAD Theta ;load the current theta value
	 ADDI 90
	 STORE DTheta ;turn theta 90 degees to the left
     JUMP CheckTheta 

StartMoving:
     LOAD FSlow
     STORE DVel ;start moving slow
     ;i need to nenable the sonar interrupts!!!
     ;set the alarm distance to be 1 foot 
	 LOAD &b01100000 ;only enable sonars 5 and 6 
	 OUT SONAREN  
     LOAD Ft1 ;load the alarm distance
	 OUT SONALARM
	 LOAD &b01100000 
	 OUT SONARINT
	 
ShouldITurn:
    IN SONALARM ; in the alarm register 
	OUT SSEG!  ; out it to the sseg 
	IN SONALARM 
	AND Mask6 ; see if sonar 6 triggered the sonar
	JPOS TurnRight54
	JUMP ShouldITurn
TurnRight54:
    LOAD Theta ;load the current theta 
	ADDI -54 ; subtract 54
	STORE DTheta 
	LOAD TurnCount
	ADDI 1
	STORE TurnCount
	LOAD TurnCount
	OUT SSEG2
	JUMP ShouldITurn ; maybe jump to check theta 
	
	
	 
	 
	 
CheckTheta:
     CALL GetThetaErr
	 ADDI -3
	 JPOS CheckTheta
	 JUMP StartMoving





TurnCount:  DW 0
