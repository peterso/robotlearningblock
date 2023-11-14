# robotlearningblock

![DRJConcept.png](/assets/images/DRJ-Figure1_RealWorld-LaboratoryDR-J.png)


Repo for the smart busy block for developing and evaluating robot manipulation skills in the real-world with electronic task boards. This project is also known as the Digital Robotic Judge or DR.J Manipulation Skill Development Pipeline.

The goal of the project is to provide an objective tool for measuring manipulation performance for both humans and robots. A microcontroller on the task box monitors the state of the mounted manipulation elements and reports state change events to a cloud-hosted dashboard to aggregate performance data. Device telemetry, including accelerometer data, is regularly logged to provide insight into the device's usage. Manipulation tasks are designed with objects that can be electronically verified to be in their designated start and goal positions. Trial protocols define sequences of actions on the task board and allow designers to create test scenarios. The execution time of subtasks within a protocol are recorded and automatically published on a web dashboard for consistent and convenient comparison with previous trial attempts. A modular framework for adding new task board objects was selected to encourage community members to design and build their own trial protocols.

An example demonstration of the internet-connected task board can be viewed on [YouTube](https://www.youtube.com/watch?v=1jdvIbDURSA). Here is the very first test run with a Franka Emika Panda robot in the lab [YouTube](https://youtu.be/LJFTypNZrFs).

### Web Links to Web Dashboards for Deployed Task Boards:
- [TBv2023: Teams Single Board View](https://cloud.kaaiot.com/wd-public/c1v9jqmgul2l1s47m6bg/dashboards/0d2f0e4c-6a80-4cf4-a48d-5b25fcb35ac0/50cdf30f-955e-4ccf-b13a-8e1d0281f68a?public_id=4e4990d1-dcab-4f1a-b1a6-8648e87bc5ad)
- [TBv2023: Subtask Completion Times](https://cloud.kaaiot.com/wd-public/c1v9jqmgul2l1s47m6bg/dashboards/a8b5be5f-cc23-4724-8f06-3f67f8fef104?public_id=4e4990d1-dcab-4f1a-b1a6-8648e87bc5ad)
- [TBv2022: Teams Single Board View](https://cloud.kaaiot.com/wd-public/c1v9jqmgul2l1s47m6bg/dashboards/dashboard2/931cb10a-3044-49c8-8530-5ce0951e291b?public_id=4e4990d1-dcab-4f1a-b1a6-8648e87bc5ad)

A screenshot of a web dashboard from the 2021 Robothon Grand Challenge competition.
![KaaIoTDashboard.png](/assets/images/KaaIoTwithDashboard.png)

#### A little history on the smart task board development...
The internet-connected task board was first featured during the internationl robot manipulation competition, [Robothon Grand Challenge](https://www.robothon-grand-challenge.com) at automatica in 2021. The design of the task board has been expanded and reimplemented for the competition again in 2022 and has also been picked up by the pan-European robotics excellence collaboration project, [euROBIN](https://www.eurobin-project.eu/).

Here is an example setup of a exemplary robot testing setup with the task board.

![RobotSetupExample.png](/assets/images/RobotSetupExample.png)

The electronic task board allows for the direct comparison between a human and a robot performance to quantitatively measure the gap.

![TrialProtocolHumanRobot](/assets/images/TrialProtocolIllustration-2Rows2-v2.png)

Here is the trial protocol for the task board version TBv2021 used in the Robothon Grand Challenge in 2021 and 2022 to guide roboticists to develop reusable manipulation skills for e-waste sorting and disassembly.

![TaskBoardProtocol2022.png](/assets/images/TrialProtocol2021.jpg)

A new task board design (TBv2023) was designed and built to develop manipulation skills in performing electronic device assessments with a multimeter. The design was also adapted to meed the needs of the euROBIN network and was used in the Robothon Grand Challenge in 2023.

![RobothonTaskBoard2023.png](/assets/images/TrialProtocol2023.jpg)

As part of the competition, teams were challenged to demonstrate the manipulation skills of their robot platform on an e-waste object of their choice in the "Bring-Your-Won-Device (BYOD) Challenge". Below are some examples of the successful demonstrations. 

![Robothon-BYOD-Demos.png](/assets/images/Robothon-BYOD-Demos.png)


### Quick Start to connect a Task Board to the Web Dashboard

The uController (orange component) Interface has 3 Buttons and a USB-C charge/programming port:
***Power Button*** (side closest to USB-C), ***Button A*** (next to screen, labeled M5), ***Button B*** (side furthest from USB-C).

#### Turning on the Task Board ####

1. Toggle Power Manually = Hold Power Button for 2 seconds to turn on and 6 seconds to turn off.
2. Plugging in the uController will automatically turn on the device.

#### Connect board to your preferred WiFi Network using your laptop or smartphone ####

1. Connect to Task Board SSID: “AutoConnectAP-task-board-###” where ### refers to the number on the sticker on the side of the box (Password: “robothon”)
2. A configuration screen should automatically open in a browser on your device. Otherwise, open web browser and go to URL: “192.168.4.1”.

3. Click the ”Configure” button to setup a new WiFi network connection. 

4. Select Desired WiFi from the scanned list.

5. Enter your SSID's WiFi password.

6. Click the “Save” Button. The new credentials will be saved to the microcontrollers EEPROM memory for future connections. The Task Board will attempt to connect to new WiFi. After it has successfully connected to the Internet over the new network the Home Screen will be displayed on the screen. 

#### Begin using the task board with human subjects or your robot platform ####

1. Ensure the task board components are on their starting position prior to starting. The uController will display an alert on the screen if it detects an incorrect starting configuration.
2. Press Button A to start recording a trial run! Trial runs will be automatically recorded to the web dashboard. Scan the QR code on the sticker for more information.
3. Reset Trial Clock after a trialèPress Button B to reset the clock to zero.

#### Some Helpful Tips:

• Offline/Local Mode: Use your task board without the internet by holding Button A down when powering on the device.

• Abort an Active Trial: Stop a trial early by holding down the Red Push Button and pressing Button A.

### Detailed User Instructions 

The task board is designed to capture manipulation performance for use in the [Robothon Grand Challenge](https://www.robothon-grand-challenge.com). The microcontroller tracks users overall trial completion time, intermediate subtask completion times, earned trial points, and cumulative interaction force. Performance trials are initiated by the user after restoring the components to their start configuration and then pressing the M5 button. A countdown timer ends trial attempts after 10 minutes. When the task board is connected to the internet, the microcontroller publishes telemetry, during an active trial at 20Hz and 0.2Hz when idle, to a remote cloud server. An interactive web dashboard renders the trial data from the task board to provide transparent, remote observability into performance trials. Trials end when the countdown timer expires or when all sub-tasks are completed and the user presses the red stop button. Final trial results are displayed on the task board LED display and on the web dashbaord.

Trial protocols have a programmed time out of 10 minutes. While a trial is active, a red LED is illuminated on the controller. During active trials the agent under test should follow the trial protocol and complete all 5 subtasks on the task board as fast as possible. The results are recorded locally on the device and displayed on the controller's LED screen. 

Link to a [web simulator](https://replit.com/@peterso/TaskBoardSimulator?v=1) with Replit for testing new task board designs and trial protocols. NOTE: you will need to generate a new device token on the KaaIoT platform. 

Prior to running a new trial attempt, the trial protocol specifies the user must pick up and randomly place the task board in a new location in front of the user. An example of the location initialization is shown in the .gif below.

![InitializeTaskBoard](/assets/gifs/pickup_and_place_tb.gif)

After the task board location has been initialized, the user then presses the Trial Start button (M5 button) to begin a logged trial attempt. This .gif shows a human actor solving the 2021 trial protocol.

![ManualTrialRun](/assets/gifs/manual_trial_run.gif)

#### Task Board Controller Button Operations

- Power On Controller: Hold Power button (Left of M5 Logo) for 2 seconds; Power Off Controller = Hold Power button for 6 seconds.

- Reset Trial: Press User button (Right of M5 Logo). The controller will automatically reset the trial results after 30 seconds from the end of the trial.

- Abort an active trial: Press Trial Start Button AND Trial Stop Button at the same time.



#### Reading Task Board State over USB

When working with the task board, developers may want to read the task board state directly from the task board controller instead of referring to the web dashboard. To do this, use the python utility file to record the output to a local CSV file. See `/python/scripts/read-serial.py`.


### Core Design Features

![CompetitionConcept.png](/assets/images/CompetitionConcept.png)
The core design features of the task board microcontroller are:

- [Status] Feature Description

- [:white_check_mark:] task board monitors a digital state of all mounted manipulation elements
- [:white_check_mark:] user can start a trial using the device without internet connection
- [:white_check_mark:] task board reports the timestamps for each successful task interaction trial over USB
- [:white_check_mark:] report the stae of the task box over WiFi to the [Kaa internet dashboard](https://cloud.kaaiot.com/solutions/bvhkhrtbhnjc0btkj7r0/dashboards/) 
- [:white_check_mark:] setup wifi greeter to allow user to supply own wifi ssid and password
- [:white_check_mark:] report the state of the task box over USB
- [:white_check_mark:] task board reports are integrated into a Microsoft Azure IoT dashboard
- [TODO] report the load cell readings on the task box surface
- [TODO] device has a digital twin that can be represented in a robot simulator i.e. Gazebo
- [TODO] integrate bluetooth power meter into task board telemetry
- [TODO] enable network support for "Eduroam"
 

### NEW Feature Requests

- [:white_check_mark:] Over-The-Air Updates to deploy new trial protocols to task boards in the field
- [:white_check_mark:] Make the task board surface more stiff/rigid -> removed load cell and mount top plate to housing directly
- [TODO] Add a automated testing procedure for a contract manufacturer to execute prior to shipping task boards
- [TODO] Look into Device Management with BalenaOS for multiple OTA updates as an alternative to Kaa OTA
- [TODO] Add ROS messages to the task board microcontroller to broadcast task results to robots


### Design Goals & System Architecture

![SystemArchitecture.png](/assets/images/SystemArchitecture.png)
Each task board is equipped with a microcontroller to remotely monitor its state and track usage statistics. Task boards are designed with the following features in mind:
- ***Affordable.*** Task board components should not exceed $200 to allow community members to build their own. The design is intended to be open sourced and remixed. 
- ***Portable.*** Task boards will have the capability to record trials untethered on battery power.
- ***Shippable.*** Task boards should be about the size of a laptop and weigh less than 1kg. For competition events, task boards can be shipped to participants to ensure everyone receives the same physical demonstrator setup.
- ***Traceability.*** Experiments performed on the task board can be automatically logged to a central web server for community members to share their development journey as well as easily download their experimental results.
- ***Extensible.*** Task board IO is extendable over an I2C bus to incorporate new task elements in future designs. 
- ***Networked.*** Each task board has the ability to report recorded trials to an online database.


### Relevant Documents


| Design Document       | TBv2021 | TBv2023 |
|-----------------------|---------|---------|
| Design BOM            | [TBv2021](https://docs.google.com/spreadsheets/d/1id1LLbRTHQwQDf9HCM8Hft5gwp9QI8y8CJt2JUEMMQk/edit?usp=sharing)        | TODO        |
| Assembly Drawing      | [PDF](https://drive.google.com/file/d/1hJSEHZe9U0Q7VRQKNKOsz96tIN9Y0mhF/view?usp=sharing)        | TODO        |
| CAD Files             | [Onshape](https://cad.onshape.com/documents/9a15cff68aad2604a1373593/w/144a51d8ddacf96586ad0e0d/e/052e579b24ce3c66ae263023)        | TODO        |
| Assembly Instructions | [PDF](https://drive.google.com/file/d/1Znj0Do6tISIWl07lZ31Lp-qGyfQIPycp/view?usp=sharing)        | TODO        |
| Wiring Diagram | [PNG](/assets/images/TaskBoard-5Level-Wiring.png)        | TODO        |
| 3D Print Files        | ./assets        | TODO        |
| CNC Files             | ./assets        | TODO        |


### Deployment
- An initial 15 task boards were fabricated for the Robothon 2021 competition at automatica.
- 20 task boards were built for the Robothon 2022 competition.
- A new design was released for Robothon 2023. 20 task boards were made for the competition.
- 15 more task boards (TBv2023) were made for the [euROBIN project](https://www.eurobin-project.eu/).


### Battery Recycling Articles for Manipulation Task Selection
- WEF Report on battery recycling [Link](https://www3.weforum.org/docs/WEF_A_New_Circular_Vision_for_Electronics.pdf)
- German Battery Collection with GRS Batterien [Link](https://www.en.grs-batterien.de/grs-batterien/)
- Fun video from Curiousity Quest showing many e-waste handling processes [YouTube](https://www.youtube.com/watch?v=lMn-sDvgj4Q&ab_channel=curiosityquest)


### References
- [M5Stack Docs](https://docs.m5stack.com/)
- [M5Stack Community Forum](https://community.m5stack.com/category/17/m5-stick-stickc)
- [KaaIoT Arduino SDK](https://github.com/kaaproject/kaa-arduino-sdk)
- [Robothon Website](https://www.robothon-grand-challenge.com) at automatica/munich_i

### Citation
Please cite this work as ***Digital Robot Judge (DR.J): Building a Task-Centric Performance Database of Real-World Manipulation with Electronic Task Boards***. The manuscript is accepted for publication in IEEE Robotics and Automation Magazine. A preprint can be found [here](https://www.researchgate.net/publication/375329709_Digital_Robot_Judge_DRJ_Building_a_Task-Centric_Performance_Database_of_Real-World_Manipulation_with_Electronic_Task_Boards#fullTextFileContent).


Bibtex:
```
@article{So2023Digital,
 author = {So, Peter and Sarabakha, Andriy and Wu, Fan and Culha, Utku and {Abu-Dakka}, Fares J and Haddadin, Sami},
 journal = {Accepted for publication in IEEE Robotics and Automation Magazine},
 title = {Digital Robot Judge (DR.J): Building a Task-Centric Performance Database of Real-World Manipulation with Electronic Task Boards},
 year = {2023}
}

```

