# robotlearningblock

Welcome to the project repository for the internet-connected task board for evaluating real-world robot manipulation skills. This project, known as the Digital Robotic Judge, or DR.J, benchmarks asynchronous manipulation performances across the electronic task board network. Side note, the name is borrowed from the legendary basketball star [Julius "DR.J" Irving](https://www.youtube.com/watch?v=bWLXcg-V8FI). Industrial use cases collected through a survey are modelled and incorporated into portable electronic task boards to be solved by the community of robot developers. The best solutions across the network are ranked on a leaderboard and developers are encouraged to share and reproduce solutions of others in the network. If you find this work helpful, please cite our [IEEE-RAM paper](https://ieeexplore.ieee.org/document/10378967). 

See the illustration of DR.J crowdsourced robot solution for real-world use cases pipeline below.

![DRJConcept.png](/assets/images/DRJ-Figure1_RealWorld-LaboratoryDR-J.png)

## Quick Links

- IEEE-RAM Paper: [DOI: 10.1109/MRA.2023.3336473](https://doi.org/10.1109/MRA.2023.3336473)
- 1-min Paper [Highlight Video](https://drive.google.com/file/d/1jVCkPBq7-JO2KUGaQfxDcgK1_qGyWVMT/view) 
- Robothon Grand Challenge Competition [automatica website](https://automatica-munich.com/en/munich-i/robothon/)
- Links to Web Dashboards for Deployed Task Boards:
  - [TBv2023: Teams Single Board View](https://cloud.kaaiot.com/wd-public/c1v9jqmgul2l1s47m6bg/dashboards/0d2f0e4c-6a80-4cf4-a48d-5b25fcb35ac0/50cdf30f-955e-4ccf-b13a-8e1d0281f68a?public_id=4e4990d1-dcab-4f1a-b1a6-8648e87bc5ad)
  - [TBv2023: Subtask Completion Times](https://cloud.kaaiot.com/wd-public/c1v9jqmgul2l1s47m6bg/dashboards/a8b5be5f-cc23-4724-8f06-3f67f8fef104?public_id=4e4990d1-dcab-4f1a-b1a6-8648e87bc5ad)
  - [TBv2021: Teams Single Board View](https://cloud.kaaiot.com/wd-public/c1v9jqmgul2l1s47m6bg/dashboards/dashboard2/931cb10a-3044-49c8-8530-5ce0951e291b?public_id=4e4990d1-dcab-4f1a-b1a6-8648e87bc5ad)
- Fake Task Board Emulator (Python browser-based) [Replit](https://replit.com/@peterso/TaskBoardSimulator?v=1)
- Simulated Task Board on Mujoco for Offline Skill Development [Multiverse Github Project](https://github.com/Multiverse-Framework/Multiverse-Objects/tree/main/task_board)
- Request a physical task board [Task Board Request Google Form](https://forms.gle/EJiNF4A1qsrMpy8i6)
- Propose new tasks for future board designs [Use Case Survey Google Form](https://forms.gle/XueGB4yPwioggSN97)

## Project Description

The goal of the project is to provide an objective tool for measuring manipulation performance for both humans and robots. A microcontroller on the task box monitors the state of the mounted manipulation elements and reports state change events to a cloud-hosted dashboard to aggregate performance data. Device telemetry, including accelerometer data, is regularly logged to provide insight into the device's usage. Manipulation tasks are designed with objects that can be electronically verified to be in their designated start and goal positions. Trial protocols define sequences of actions on the task board and allow designers to create test scenarios. The execution time of subtasks within a protocol are recorded and automatically published on a web dashboard for consistent and convenient comparison with previous trial attempts. A modular framework for adding new task board objects was selected to encourage community members to design and build their own trial protocols.

An example demonstration of the internet-connected task board can be viewed on [YouTube](https://www.youtube.com/watch?v=1jdvIbDURSA). Here is the very first test run with a Franka Emika Panda robot in the lab [YouTube](https://youtu.be/LJFTypNZrFs).



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

As part of the competition, teams were challenged to demonstrate the manipulation skills of their robot platform on an e-waste object of their choice in the "Bring-Your-Own-Device (BYOD) Challenge". Below are some examples of the successful demonstrations. 

![Robothon-BYOD-Demos.png](/assets/images/Robothon-BYOD-Demos.png)


### Quick Start to connect a Task Board to the Web Dashboard

The microcontroller (orange component) has 3 Buttons and a USB-C charge/programming port:
- ***Power Button*** (side closest to USB-C), 
- ***Button A*** (next to screen, labeled M5), 
- ***Button B*** (side furthest from USB-C).

#### Turning on the Task Board ####

1. Toggle Power Manually = Hold Power Button for 2 seconds to turn on and 6 seconds to turn off.
2. Plugging in the uController will automatically turn on the device.

#### Connect board to the web dashboard over the internet with your preferred WiFi Network using your laptop or smartphone

1. Connect to Task Board SSID: “AutoConnectAP-task-board-###” where ### refers to the number on the sticker on the side of the box (Password: “robothon”)
2. A configuration screen should automatically open in a browser on your device. Otherwise, open web browser and go to URL: “192.168.4.1”.

3. Click the ”Configure” button to setup a new WiFi network connection. 

4. Select Desired WiFi from the scanned list.

5. Enter your SSID's WiFi password.

6. Click the “Save” Button. The new credentials will be saved to the microcontrollers EEPROM memory for future connections. The Task Board will attempt to connect to new WiFi. After it has successfully connected to the Internet over the new network the Home Screen will be displayed on the screen. 

#### Begin running experiments with the task board

1. Ensure the task board components are on their starting position prior to starting. The uController will display an alert on the screen if it detects an incorrect starting configuration.
2. Press Button A to start recording a trial run! Trial runs will be automatically recorded to the web dashboard. Scan the QR code on the sticker for more information.
3. Reset Trial Clock after a trialèPress Button B to reset the clock to zero.

#### Some Other Helpful Tips:

  • ***Offline/Local Mode:*** Use your task board without the internet by holding Button A down when powering on the device.

  • ***Abort an Active Trial:*** Stop a trial early by holding down the Red Push Button and pressing Button A.

### Detailed User Instructions 

The task board is designed to capture manipulation performance for use in the [Robothon Grand Challenge](https://www.robothon-grand-challenge.com). The microcontroller tracks users overall trial completion time, intermediate subtask completion times, earned trial points, and cumulative interaction force. Performance trials are initiated by the user after restoring the components to their start configuration and then pressing the M5 button. A countdown timer ends trial attempts after 10 minutes. When the task board is connected to the internet, the microcontroller publishes telemetry, during an active trial at 20Hz and 0.2Hz when idle, to a remote cloud server. An interactive web dashboard renders the trial data from the task board to provide transparent, remote observability into performance trials. Trials end when the countdown timer expires or when all sub-tasks are completed and the user presses the red stop button. Final trial results are displayed on the task board LED display and on the web dashbaord.

Trial protocols have a programmed time out of 10 minutes. While a trial is active, a red LED is illuminated on the controller. During active trials the agent under test should follow the trial protocol and complete all 5 subtasks on the task board as fast as possible. The results are recorded locally on the device and displayed on the controller's LED screen. 

Link to a [web simulator](https://replit.com/@peterso/TaskBoardSimulator?v=1) with Replit for testing new task board designs and trial protocols. NOTE: you will need to generate a new device token on the KaaIoT platform. 

Prior to running a new trial attempt, the trial protocol specifies the user must pick up and randomly place the task board in a new location in front of the user. An example of the location initialization is shown in the .gif below.

![InitializeTaskBoard](/assets/gifs/pickup_and_place_tb.gif)

After the task board location has been initialized, the user then presses the Trial Start button (M5 button) to begin a logged trial attempt. This .gif shows a human actor solving the 2021 trial protocol.

![ManualTrialRun](/assets/gifs/manual_trial_run.gif)

#### Reading Task Board State over USB

When working with the task board, developers may want to read the task board state directly from the task board controller instead of referring to the web dashboard. To do this, use the python utility file to record the output to a local CSV file. See `/python/scripts/read-serial.py`.


### Core Design Features

![CompetitionConcept.png](/assets/images/CompetitionConcept.png)

The core design features of the task board microcontroller are:

[Status] Feature Description

- [x] task board monitors a digital state of all mounted manipulation elements
- [x] user can start a trial using the device without internet connection
- [x] task board reports the timestamps for each successful task interaction trial over USB
- [x] report the stae of the task box over WiFi to the [Kaa internet dashboard](https://cloud.kaaiot.com/solutions/bvhkhrtbhnjc0btkj7r0/dashboards/) 
- [x] setup wifi greeter to allow user to supply own wifi ssid and password
- [x] report the state of the task box over USB
- [x] task board reports are integrated into a Microsoft Azure IoT dashboard
- [x] device has a digital twin that can be represented in a robot simulator i.e. Gazebo, Mujoco. 
- [ ] report the load cell readings on the task box surface
- [ ] integrate bluetooth power meter into task board telemetry
- [ ] enable internet connection network support for "Eduroam"


 

### NEW Feature Requests

- [x] Over-The-Air Updates to deploy new trial protocols to task boards in the field
- [x] Make the task board surface more stiff/rigid -> removed load cell and mount top plate to housing directly
- [ ] Add a automated testing procedure for a contract manufacturer to execute prior to shipping task boards
- [ ] Look into Device Management with BalenaOS for multiple OTA updates as an alternative to Kaa OTA
- [ ] Add ROS messages to the task board microcontroller to broadcast task results to robots

Please make a Github pull request with an updated README to add your new feature requests. Task board designs are updated annually and the firmware is continuously improved based on user feedback.

### Design Goals & System Architecture

![SystemArchitecture.png](/assets/images/SystemArchitecture.png)
Each task board is equipped with a microcontroller to remotely monitor its state and track usage statistics. Task boards are designed with the following features in mind:
- ***Affordable.*** Task board components should not exceed $200 to allow community members to build their own. The design is intended to be open sourced and remixed. 
- ***Portable.*** Task boards will have the capability to record trials untethered on battery power.
- ***Shippable.*** Task boards should be about the size of a laptop and weigh less than 1kg. For competition events, task boards can be shipped to participants to ensure everyone receives the same physical demonstrator setup.
- ***Traceability.*** Experiments performed on the task board can be automatically logged to a central web server for community members to share their development journey as well as easily download their experimental results.
- ***Extensible.*** Task board IO is extendable over an I2C bus to incorporate new task elements in future designs. 
- ***Networked.*** Each task board has the ability to report recorded trials to an online database.


### Relevant Design Documents


| Design Document       | TBv2021 | TBv2023 |
|-----------------------|---------|---------|
| Design BOM            | [TBv2021](https://docs.google.com/spreadsheets/d/1id1LLbRTHQwQDf9HCM8Hft5gwp9QI8y8CJt2JUEMMQk/edit?usp=sharing)        | [TBv2023](https://drive.google.com/file/d/1Rnd_u9MT5yowWGOLc9BP8yHH_inwEdMb/view?usp=sharing)        |
| Assembly Drawing      | [PDF](https://drive.google.com/file/d/1hJSEHZe9U0Q7VRQKNKOsz96tIN9Y0mhF/view?usp=sharing)        | [PDF](https://drive.google.com/file/d/1tS-sQoqfd0bgX27vQssPpRKi-OBj2Tsp/view?usp=sharing)        |
| CAD Files             | [Onshape](https://cad.onshape.com/documents/9a15cff68aad2604a1373593/w/144a51d8ddacf96586ad0e0d/e/052e579b24ce3c66ae263023)        | [Onshape](https://cad.onshape.com/documents/db5a4ee49311d077447cc358/w/9c0b93db9a3945ee3cde6ada/e/5ab54cca5c27c6df318a9a63?renderMode=0&uiState=6720a79a1d02e666789b5f03)        |
| Assembly Instructions | [PDF](https://drive.google.com/file/d/1Znj0Do6tISIWl07lZ31Lp-qGyfQIPycp/view?usp=sharing)        | [PDF](https://drive.google.com/file/d/1LZS_wPafdJOO1Q0lu-8TDO9xGrxpoSBB/view?usp=sharing)        |
| Wiring Diagram | [PNG](/assets/images/TaskBoard-5Level-Wiring.png)        | [PDF](https://drive.google.com/file/d/1LZS_wPafdJOO1Q0lu-8TDO9xGrxpoSBB/view?usp=sharing)        |
| 3D Print Files        | ./assets        | ./assets        |
| CNC Files             | ./assets        | ./assets        |


### Task Board Deployment
- An initial 15 task boards were fabricated for the Robothon 2021 competition at automatica.
- 20 task boards were built for the Robothon 2022 competition.
- A new design was released for Robothon 2023. 20 task boards were made for the competition.
- 15 more task boards (TBv2023) were made for the [euROBIN project](https://www.eurobin-project.eu/).
- ***Order your own task board*** If you are interested in getting a pre-assembled task board to demonstrate your robot platform's capabilities or to follow along in the robot benchmark, please write an e-mail request with the model year [TBv2021 or TBv2023] and the number of boards to peter@modularmotions.com.


### Battery Recycling Articles for Manipulation Task Selection
- WEF Report on battery recycling [Link to Article](https://www3.weforum.org/docs/WEF_A_New_Circular_Vision_for_Electronics.pdf)
- German Battery Collection with GRS Batterien [Link to Article](https://www.en.grs-batterien.de/grs-batterien/)
- Fun video from Curiousity Quest showing many e-waste handling processes [YouTube Video](https://www.youtube.com/watch?v=lMn-sDvgj4Q&ab_channel=curiosityquest)


### References
- [M5Stack Docs](https://docs.m5stack.com/)
- [M5Stack Community Forum](https://community.m5stack.com/category/17/m5-stick-stickc)
- [KaaIoT Arduino SDK](https://github.com/kaaproject/kaa-arduino-sdk)
- Robothon Grand Challenge at [automatica/munich_i](https://www.robothon-grand-challenge.com), [Digital Kickoff Meeting Slides 2022](https://drive.google.com/file/d/1OdqM9bOmNVIISuL4vuRz019WOS697p_8/view?usp=sharing)
- Robothon Grand Challenge at [automatica/munich_i](https://www.robothon-grand-challenge.com), [Digital Kickoff Meeting Slides 2023](https://drive.google.com/file/d/1x1t6U09iTEmM228orjCqwZNxu3CphAaz/view?usp=share_link)
- euROBIN Manipulation Skill Versatility Challenge at IROS 2024 [Competition Website](http://sites.google.com/view/eurobin-msvc/home)

### Citation 
If you find this work insightful to your research or you reference data from our Robothon Grand Challenge competitions please cite this work as ***Digital Robot Judge (DR.J): Building a Task-Centric Performance Database of Real-World Manipulation with Electronic Task Boards***. The manuscript is accepted for publication in IEEE Robotics and Automation Magazine. An early release is available on IEEE [here](https://ieeexplore.ieee.org/document/10378967).


Bibtex:
```
@ARTICLE{So2024DRJ,
  author={So, Peter and Sarabakha, Andriy and Wu, Fan and Culha, Utku and Abu-Dakka, Fares J. and Haddadin, Sami},
  journal={IEEE Robotics & Automation Magazine}, 
  title={Digital Robot Judge: Building a Task-centric Performance Database of Real-World Manipulation With Electronic Task Boards}, 
  year={2023},
  volume={},
  number={},
  pages={2-14},
  doi={10.1109/MRA.2023.3336473}}
```

