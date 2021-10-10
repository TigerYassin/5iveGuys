# DE2 Amiga Bot

## Objective
Use the various IR and odometry sensors on an Altera FPGA robot to circle around the most landmarks in an area in the least amount of time.
The pathfinding algorithm uses hardware interrupts written in assembly to detect the location of landmarks, move towards them, and circle around them without colliding.

## Results
While our robot was effective at circling landmarks without colliding into them,it overlooked obvious paths when landmarks reached the hardware's maximum detectable range.
However, once a landmark was detected, our pathfinding algorithm would follow the tightest possible path around the landmark using the robots odometry readings.
