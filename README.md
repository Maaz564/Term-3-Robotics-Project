**Robot Climbing System**

This repository contains the code and documentation for a robotic system designed to climb structures autonomously. The robot utilizes a pair of servo-operated grippers and a set of DC motors to navigate vertically, simulating an inchworm's motion.

`Getting Started`

These instructions will get your robot up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the robot on a live system.

`Prerequisites`

Arduino IDE 

Any Arduino-compatible board 

Servo motors (2x)

DC motors (3x)

HC-SR04 Ultrasonic Sensor

Necessary wires and power supply

`Installation`

Assemble the Hardware

Refer to the schematic diagrams for wiring and assembly instructions.

Connect the Arduino to Your Computer

Use the USB cable to connect your Arduino board to your computer.

Open the Arduino IDE

Launch the Arduino IDE on your computer.

Load the Sketch

Open the robot_climbing_system.ino file in the Arduino IDE.

Select Your Arduino Board and Port

Go to Tools > Board and select the type of Arduino board you are using.
Go to Tools > Port and select the COM port where your Arduino is connected.
Upload the Code

Click the upload button in the Arduino IDE to compile and upload the sketch to the Arduino board.

`Usage`

Once the code is uploaded and the robot is assembled:

Power On the Robot

Ensure that the power supply to the motors and servos is secure and switch on the power.

Starting the Robot

The robot will automatically start the climbing process once powered on.

Monitoring

Watch the serial monitor in the Arduino IDE to view the distance measurements and the current state of the robot.

`Code Overview`

The program operates the robot through several states reflecting different stages of the climbing process:

**Initializing:** Setting initial positions of the grippers.

**Climbing Up:** Grippers alternate to simulate climbing.

**Lifting Body Up:** Adjusting the robot's body to continue climbing efficiently.

**Climbing Down:** Reversing the climbing process for descending.

**Lowering Body Down:** Adjusting the body to safely descend.

The robot uses an ultrasonic sensor to measure distance and ensure it does not collide with obstacles.


`Built With`

Arduino - The IDE used

C++ - Programming language used

**Authors**

Muhammad Maaz

Payton Liao


