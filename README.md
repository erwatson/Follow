# Follow
following car micropython pyboard project

Goal
Improve my hardware and hackery skills while having some fun and understanding more about self driving cars and robotics. 

Description
Car A: 
Light sensing diodes to find direction toward brightest light
Master uses serial transmission to tell Car B what motor control it is doing currently and sends signal wirelessly
Pyboard, battery, chassis
Car B: 
Slave in serial transmission to listen to Car A and learn and store what Car A motors are doing. Delay by certain time (based on distance behind, human toggle - not smart) and then mirror motor actions to follow Car A
Pyboard, battery, chassis

Simple project to start
Use two pyboards to communicate with each other via UART, I2C, or SPI - something that can be moved to serial wireless communication without too much work later.

Have input from light sensing diode into one board, do some math and transmit to other board.

I need:
Two pyboards
One bread board
wires
a few photo diodes

Also, I could do a software and hardware simulation with the two boards with some error perturbations and plot the outcomes in real time on the computer hopefully… or I could save it locally and then view it (maybe in an animation) on the PC. Then, move to real hardware instead of animation later as well as make communication wireless.
