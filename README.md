# Warehouse-Automation-Systems

### Idea
The idea behind this project is to automate the process involved in a 
warehouse and this consist in move products into, within, and out of 
the inventory with minimal human assistance. As part of an automation 
project an important aspect is the elimination of labor-intensive duties 
that involve repetitive physical work and manual entry and analisys. For 
example a robot can move products from one end of the warehouse to the 
shipping zone and software keeping all records current. Improve the 
efficiency, speed, reliability and accuracy of tasks. Enable the 
cooperation between the robot and human together to accomplish repetitive 
tasks while minimizing the fatique and injury. The Internet of Robotic 
Things (IoRT) is a technology that become a key factor in the supply 
chain to increase the productivity and reduce the operation costs. 
Sensors, robotic arm, autonomous vehicle and conveyor belt intelligent 
connected can enable IoRT in different context with rapid development and 
deployment.

### Project
The architecture designed and developed can be divided four main 
wireless network nodes:

* Conveyor belt
* Robot Car
* Robotic Arm
* Warehouse Dashboard

the automation begins with the conveyor belt, the package is transported until it reaches the pick-up point, then the conveyor publish a message to the robotic arm. 
The robotic arm will pick the package and move it in front of the IP camera to scan the QR code, after the scan the arm will place the package on the robotic car that receive in real-time the right route to redirect the package to one of the parcel carrier terminal. 
Then after scanning the arm will place the package on the robotic car Finally the robotic car will return back to the home position and the conveyor belt will restart.  

 #### Conveyor belt
