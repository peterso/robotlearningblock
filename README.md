# robotlearningblock

![CompetitonTaskBoardwithProtocol.png](/assets/images/CompetitonTaskBoardwithProtocol.png)

Repo for smart busy block for developing and evaluating robot manipulation skills in the real-world.

The goal of the project is to provide an objective tool for measuring manipulation performance for both humans and robots. A microcontroller on the task box monitors the state of the mounted manipulation elements and reports state change events to a [cloud-hosted dashboard](https://cloud.kaaiot.com/wd-public/c1v9jqmgul2l1s47m6bg/dashboards/931cb10a-3044-49c8-8530-5ce0951e291b?public_id=4e4990d1-dcab-4f1a-b1a6-8648e87bc5ad) to aggregate performance data. Device telemetry, including accelerometer data, is regularly logged to provide insight into the device's usage. Manipulation tasks are designed with objects that can be electronically verified to be in their designated start and goal positions. Trial protocols define sequences of actions on the task board and allow designers to create test scenarios. The execution time of subtasks within a protocol are recorded and automatically published on a web dashboard for consistent and convenient comparison with previous trial attempts. A modular framework for adding new task board objects was selected to encourage community members to design and build their own trial protocols.

An example demonstration of the internet-connected task board can be viewed on [YouTube](https://youtu.be/LJFTypNZrFs).

The internet-connected task board was first featured during the internationl robot manipulation competition, [Robothon Grand Challenge](https://www.robothon-grand-challenge.com) at automatica in 2021. The design of the task board has been expanded and reimplemented for the competition again in 2022 and has also been picked up by the pan-European robotics excellence collaboration project, [euROBIN](https://www.eurobin-project.eu/).


![RobotSetupExample.png](/assets/images/RobotSetupExample.png)

### Quick Start

1. Power on task board by plugging in USB to charge the device. "Picture of person plugging in task board USB"

2. Connect the task board to WiFi. "Picture of task board with PC displaying web dashboard"

3. Move task board components to match the starting configuration in the Trial Protocol and then press the Trial Start Button ("M5" Button on top of the controller). "Picture of task board in start configuration with finger on the Trial Start Button"

4. Begin using task board for your manipulation experiments! Your trial results will be automatically recorded on the web dashboard "Picture of user with robot and task board"

Trial protocols have a programmed time out of 10 minutes. While a trial is active, a red LED is illuminated on the controller. During active trials the agent under test should follow the trial protocol and complete all 5 subtasks on the task board as fast as possible. The results are recorded locally on the device and displayed on the controller's LED screen. The microcontroller publishes the latest available device data to the web server every 5 seconds when the device is connected to the internet over WiFi. 

![TrialProtocol2022.png](/assets/images/TrialProtocol2022.png)

### Detailed User Instructions 

The task board is designed to capture manipulation performance for use in the [Robothon Grand Challenge](https://www.robothon-grand-challenge.com). The microcontroller tracks users overall trial completion time, intermediate subtask completion times, earned trial points, and cumulative interaction force. Performance trials are initiated by the user after restoring the components to their start configuration and then pressing the M5 button. A countdown timer ends trial attempts after 10 minutes. When the task board is connected to the internet, the microcontroller publishes the latest and ongoing trial data every 5 seconds to a remote cloud server. An interactive web dashboard renders the trial data from the task board to provide transparent, remote observability into performance trials. Trials end when the countdown timer expires or when all sub-tasks are completed and the user presses the red stop button. Final trial results are displayed on the task board LED display and on the web dashbaord.

View the task board web dashboard at this URL: https://cloud.kaaiot.com/solutions/bvhkhrtbhnjc0btkj7r0/dashboards/ 

A publicly viewable version of the dashboard is available to view any of the released task boards here: https://cloud.kaaiot.com/wd-public/c1v9jqmgul2l1s47m6bg/dashboards/931cb10a-3044-49c8-8530-5ce0951e291b?public_id=4e4990d1-dcab-4f1a-b1a6-8648e87bc5ad

![KaaIoTDashboard.png](/assets/images/KaaIoTwithDashboard.png)
![ManualTrialRun](/assets/gifs/manual_trial_run.gif)
![InitializeTaskBoard](/assets/gifs/pickup_and_place_tb.gif)

#### Task Board Controller Button Operations

- Power On Controller: Hold Power button (Left of M5 Logo) for 2 seconds; Power Off Controller = Hold Power button for 6 seconds.

- Reset Trial: Press User button (Right of M5 Logo). The controller will automatically reset the trial results after 30 seconds from the end of the trial.

- Abort an active trial: Press Trial Start Button AND Trial Stop Button at the same time.

#### Connecting the Task Board Controller to the Internet

1. Connect to Task Board SSID: “AutoConnectAP-task-board”, password is "password"

2. Open web browser and go to URL: “192.168.4.1”

3. Click ”Configure” button

4. Select Desired WiFi from scanned list

5. Enter your SSID's WiFi password

6. Click “Save” Button. Board will attempt to connect to new WiFi and present Trial Start Screen after successful connection. 

#### Reading Task Board State over USB

When working with the task board, developers may want to read the task board state directly from the task board controller instead of referring to the web dashboard. To do this, use the python utility file to record the output to a local CSV file. See `/python/scripts/read-serial.py`.


### Core Design Features

The core design features of the task board microcontroller are:
[Status] Feature Description
- [Done!] task board monitors a digital state of all mounted manipulation elements
- [Done!] user can start a trial using the device without internet connection
- [Done!] task board reports the timestamps for each successful task interaction trial over USB
- [Done!] report the stae of the task box over wifi to the [Kaa internet dashboard](https://cloud.kaaiot.com/solutions/bvhkhrtbhnjc0btkj7r0/dashboards/) 
- [Done!] setup wifi greeter to allow user to supply own wifi ssid and password
- [Done!] report the state of the task box over USB
- [Done!] task board reports are integrated into a Microsoft Azure IoT dashboard
- [TODO] report the load cell readings on the task box surface
- [TODO] device has a digital twin that can be represented in a robot simulator i.e. Gazebo
- [TODO] integrate bluetooth power meter into task board telemetry
- [TODO] enable network support for "Eduroam"
 

### NEW Feature Requests

- [Done!] Over-The-Air Updates to deploy new trial protocols to task boards in the field
- [Done!] Make the task board surface more stiff/rigid -> removed load cell and mount top plate to housing directly
- [TODO] Add a testing procedure for a factory to execute prior to shipping task board
- [TODO] Look into Device Management with BalenaOS for multiple OTA updates 


### Design Goals & System Architecture

Each task board is equipped with a microcontroller to monitor its state. Task boards are designed with the following features in mind:
- Affordable. Task boards should ideally not exceed $200 to incentive others to use the platform instead of making their own. The design is intended to be open sourced. 
- Portable. Task boards will have the capability to record trials untethered on battery power.
- Extensible. Task board IO is extendable over I2C. 
- Networked. Each task board will have the ability to report recorded trials to an online database.

![SystemArchitecture.png](/assets/images/SystemArchitecture.png)


### Relevant Documents
- Bill of Materials (BOM) [Google Sheet](https://docs.google.com/spreadsheets/d/1id1LLbRTHQwQDf9HCM8Hft5gwp9QI8y8CJt2JUEMMQk/edit?usp=sharing)
- CAD Design [Onshape](https://cad.onshape.com/documents/9a15cff68aad2604a1373593/w/144a51d8ddacf96586ad0e0d/e/052e579b24ce3c66ae263023)
- Assembly Drawing [PDF](https://drive.google.com/file/d/1hJSEHZe9U0Q7VRQKNKOsz96tIN9Y0mhF/view?usp=sharing)
- Wiring Diagram [Link](/assets/images/TaskBoard-5Level-Wiring.png)
- Task Board Assembly Instructions [PDF](https://drive.google.com/file/d/1Znj0Do6tISIWl07lZ31Lp-qGyfQIPycp/view?usp=sharing)
- Task Board Inventory Tracking [Google Sheet](https://docs.google.com/spreadsheets/d/1id1LLbRTHQwQDf9HCM8Hft5gwp9QI8y8CJt2JUEMMQk/edit#gid=701724046)


### Deployment
- An initial 15 task boards were fabricated for the Robothon 2021 competition at automatica.
- 20 task boards were built for the Robothon 2022 competition.


### Battery Recycling Articles for Manipulation Task Selection
- WEF Report on battery recycling [Link](https://www3.weforum.org/docs/WEF_A_New_Circular_Vision_for_Electronics.pdf)
- German Battery Collection with GRS Batterien [Link](https://www.en.grs-batterien.de/grs-batterien/)
- Fun, kids video Curiousity Quest showing many of the processes [YouTube](https://www.youtube.com/watch?v=lMn-sDvgj4Q&ab_channel=curiosityquest)


### References
- [M5Stack Docs](https://docs.m5stack.com/)
- [M5Stack Community Forum](https://community.m5stack.com/category/17/m5-stick-stickc)
- [KaaIoT Arduino SDK](https://github.com/kaaproject/kaa-arduino-sdk)
- [Robothon Website](https://www.robothon-grand-challenge.com)
