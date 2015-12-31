# Follow
### Autonomous Vehicle Inter-vehicle Communication and Swarm Control - micropython pyboard project
by Eric Watson
If you want to borrow some code, go for it. If you want to contribute, fork away and send me a PR or email me at watson.eric.r@gmail.com to discuss direction.

### Motivation
Improve my hardware skills, learn more python, and sharpen simulation skills while having some fun and learning about autonomous vehicle control, swarm intelligence, and inter-vehicle communication.
Goals

**Stage 1:** One autonomous car completes navigation objective 
**Stage 2:** Add a second autonomous car without sensors in following formation with the first using inter-vehicle communication
**Stage 3:** Add sensors and intelligence to both vehicles to enable efficient swarm formation driving

### Plan of Attack
Starting with one stage at a time: simulate dynamics and control, then move to processor-in-the-loop (pyboard), and then full hardware runs with vehicles. Stick to this as closely as possible considering I’m living out of a backpack at the moment!

### Stage 1
One autonomous car completes navigation objective

**Car A (Alfred):** 
* Sensors and smarts to complete navigation goal → find brightest light
* Pyboard, light sensors, battery and chassis required
**Simulation**
* Python simulation including:
* Light source static location
* Sensor readings (left or right is brighter?)
* Simple vehicle dynamics model (constant velocity, max turn rate)
* Bang-bang control turning the vehicle toward brighter light
* Store and visualize light source location, sensor readings, vehicle location, and control values
Then, bring processor in the loop. Then, full hardware!

### Stage 2
Add a second autonomous car without sensors in following formation with the first using inter-vehicle communication

**Car A (Alfred):** 
* Sensors and smarts to complete navigation goal → find brightest light
* Add transmitter to share state with Bert
**Car B (Bert):** 
* “Knows” Alfred’s state and follows in formation
* Pyboard, battery, receiver, and chassis required

**Simulation**
* Sharing of control inputs from Alfred to Bert
* Similar Bert vehicle dynamics model with follow control implemented
* Store and visualize light source location, sensor readings, vehicle locations, and control values
Then, I’ll bring both processors in the loop with communication between them. Then, full hardware!

### Stage 3 
Add sensors and intelligence to both vehicles to enable efficient swarm formation driving

**All Vehicles (Alfred & Bert)**
* Add GPS receivers and 2-way communication sharing state with each other
* Swarm intelligence rules to formation drive to navigation objective

**Simulation**
* Share states between vehicles
* Implement formation driving rules
* Store and visualize light source location, sensor readings, vehicle locations, control values, and relative dynamics
Then, I’ll bring both processors in the loop with communication between them including simulated GPS data. Then, full hardware out in a parking lot!


