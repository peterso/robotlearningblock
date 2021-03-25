# robotlearningblock
Repo for smart busy block for developing and evaluating robot manipulation skills in the real-world.

The goal of the project is to provide an objective tool for measuring robot performance. 

An example demonstration of usage of the connected task board can be viewed [here](https://youtu.be/LJFTypNZrFs)

~~~
TaskBoard_ID    Protocol_ID     Trial_Number    Developer   Platform    Completion Time     Score
TaskBoard-001   Protocol-001    010             pso         Panda-OTS   60sec               100  
TaskBoard-001   Protocol-001    011             pso         Panda-OTS   55sec               100  
TaskBoard-001   Protocol-001    012             pso         Panda-OTS   53sec               100  
~~~

### System Architecture

Each task board will be equipped with a microcontroller to monitor its state. Task boards are designed with the following features:
- Affordable. Task boards should ideally not exceed $200 to incentive others to use the platform instead of making their own. The design is intended to be open sourced. 
- Portable. Task boards will have the capability to record trials untethered on battery power.
- Extensible. Task board IO is extendable over I2C. 
- Networked. Each task board will have the ability to report recorded trials to an online database.

![SystemArchitecture.png](/assets/images/SystemArchitecture.png)

### Relevant Documents
- BOM (Google Sheet Link TODO)
- Wiring Diagram [Link](/assets/images/TaskBoard-5Level-Wiring.png)
