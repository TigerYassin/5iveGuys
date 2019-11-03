;Write assembly code to make a box around a reflector given 
;that the robot starts 1 foot away from sensor 0
SetInitialPosition: 
   ; we should store the value of x and y and store that in a variable
   ;do not reset the odometry!
   LOAD XPOS ;load the xposition
   OUT SSEG1
   STORE XNOT
   LOAD YPOS
   OUT SSEG2
   STORE YNOT
   ;now move forward 6 inches 
   LOADI 0
   STORE DTheta
   LOAD FSlow ;load a slow speed
   STORE DVel 
MoveEast6Inches:
   ;I need to check the difference between XNOT and the X distance 
   LOAD XPOS 
   SUB  XNOT
   CALL Abs  ;get the difference between x and xnot 
             ;check if the robot has moved 6 inches
	ADDI -147 ; should be within 6 inches how much error should we allow for I will guess 20 these numbers can be adjusted
	          ; how do i check if the error is less than 20 mm
    ADDI 20
	
	JPOS MoveSouth2Feet ;jump to another subroutine
    JUMP MoveEast6Inches

MoveSouth2Feet:
    ;first we will want to turn the robot 90 degrees to the south  
    ;even before that we should stop the robot
	LOADI 0
	STORE DVel
	LOAD 
   
      
   







XNOT:  DW 0 ; i still need to move thesew variables to the scan sensors 1 foot code
YNOT:  DW 0 ;store the original x and y position