# robotlearningblock
Repo for smart busy block for developing and evaluating robot manipulation skills in the real-world.

The goal of the project is to provide an objective tool for measuring manipulation performance for both humans and robots. The microcontroller on the task box monitors the state of the mounted manipulation elements and records state change events and accelerometer readings to a cloud-hosted dashboard to aggregate performance data.

An example demonstration of usage of the connected task board can be viewed on [youtube](https://youtu.be/LJFTypNZrFs).

Below is a table of important stastics that can be collected using the connected task board.

~~~
TaskBoard_ID    Protocol_ID     Trial_Number    Developer   Platform    Completion Time     Score
TaskBoard-001   Protocol-001    010             pso         Panda-OTS   60sec               100  
TaskBoard-001   Protocol-001    011             pso         Panda-OTS   55sec               100  
TaskBoard-001   Protocol-001    012             pso         Panda-OTS   53sec               100  
~~~

![RobotSetupExample.png](/assets/images/RobotSetupExample.png)

### Quick Start
1. Plug in board to USB to charge.

	1.1 Power On = Hold Left button (of M5) on 2sec, Power Off = Hold Left button 6sec

	1.2 Reset Trial = Press Right button (of M5)

2. Connect board to WiFi

	2.1 Connect to Task Board SSID: “AutoConnectAP-task-board”, password is "password"

	2.2 Open web browser and go to URL: “192.168.4.1”

	2.3 Click ”Configure” button

	2.4 Select Desired WiFi from scanned list

	2.5 Enter WiFi password

	2.6 Click “Save” Button. Board will attempt to connect to new WiFi and present Trial Start Screen after successful connection. 

3. Begin using task board!

	3.1 Ensure the task board components are on their starting position prior to starting.

	3.2 Press M5 Button to Start trial!

![KaaIoTDashboard.png](/assets/images/KaaIoTwithDashboard.png)

### Usage 
The task board is designed to capture manipulation performance for use in the Robothon Grand Challenge. The microcontroller tracks users overall trial completion time, intermediate sub-task completion times, earned trial points, and cumulative interaction force. Performance trials are initiated by the user after restoring the components to their start configuration and then pressing the M5 button. A countdown timer ends trial attempts after 10 minutes. When the task board is connected to the internet, the microcontroller publishes the latest and ongoing trial data every 5 seconds to a remote cloud server. An interactive web dashboard renders the trial data from the task board to provide transparent, remote observability into performance trials. Trials end when the countdown timer expires or when all sub-tasks are completed and the user presses the red stop button. Final trial results are displayed on the task board LED display and on the web dashbaord.

### Core Features

The core features of the microcontroller are:
- [Status] Feature Description
- [Done] task board monitors a digital state of all mounted manipulation elements
- [Done] user can start a trial using the device without internet connection
- [Done] task board reports the timestamps for each successful task interaction trial over USB
- [Done] report the stae of the task box over wifi to the [Kaa internet dashboard](https://cloud.kaaiot.com/solutions/bvhkhrtbhnjc0btkj7r0/dashboards/) 
- [Done] setup wifi greeter to allow user to supply own wifi ssid and password
- [TODO] report the load cell readings on the task box surface
- [TODO] report the state of the task box over USB
- [TODO] device has a digital twin that can be represented in a robot simulator
- [TODO] task board reports are integrated into a Microsoft Azure IoT dashboard

### System Architecture

Each task board is equipped with a microcontroller to monitor its state. Task boards are designed with the following features in mind:
- Affordable. Task boards should ideally not exceed $200 to incentive others to use the platform instead of making their own. The design is intended to be open sourced. 
- Portable. Task boards will have the capability to record trials untethered on battery power.
- Extensible. Task board IO is extendable over I2C. 
- Networked. Each task board will have the ability to report recorded trials to an online database.

![SystemArchitecture.png](/assets/images/SystemArchitecture.png)

### Relevant Documents
- BOM [Google Sheet](https://docs.google.com/spreadsheets/d/1id1LLbRTHQwQDf9HCM8Hft5gwp9QI8y8CJt2JUEMMQk/edit?usp=sharing)
- CAD Design [Onshape](https://cad.onshape.com/documents/9a15cff68aad2604a1373593/w/144a51d8ddacf96586ad0e0d/e/052e579b24ce3c66ae263023)
- Assembly Drawing [PDF](https://drive.google.com/file/d/1hJSEHZe9U0Q7VRQKNKOsz96tIN9Y0mhF/view?usp=sharing)
- Wiring Diagram [Link](/assets/images/TaskBoard-5Level-Wiring.png)

### Deployment
The initial task boards will be equipped with two microcontrollers to read and send the sensors and report them to the internet dashboard. The reason for this is the weigh module and pbhub units are compatible to run on a single m5stickc board as they both need to be connected to the single grove port. 

An initial 15 task boards will be fabricated to be used in the Robothon 2021 competition at Automatica.

	controller1: taskboard state monitor 	Program: stopwatch_m5stickcplus.ino
	controller2: weight			Program: KAAweight2.ino

### References
- [M5Stack Docs](https://docs.m5stack.com/#/)
- [M5Stack Community Forum](https://community.m5stack.com/category/17/m5-stick-stickc)
- [KaaIoT Arduino SDK](https://github.com/kaaproject/kaa-arduino-sdk)
- [Robothon Website](https://www.robothon-grand-challenge.com)
