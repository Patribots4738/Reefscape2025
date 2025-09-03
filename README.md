


![Gradle Build](https://img.shields.io/github/actions/workflow/status/Patribots4738/Reefscape2025/gradle.yml?label=Gradle%20Build&logo=Gradle)

<sup><sup>repo must be public for this to work ^</sup></sup>

[`src/main/java/frc/robot`](src/main/java/frc/robot) shortcut

[![Game Manual](https://static.wixstatic.com/media/695840_853bc2abe81d42d3a57225beb3304874~mv2.jpg)](https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf)

![Robot Image](images/robot.gif)

____

# _**The Patribots (FRC 4738)**_
### Visit our website at [patribots.org](https://www.patribots.org)!

<br />The Patribots are a school-based _FIRST&reg; Robotics Competition_ team from Patrick Henry High School, located in San Diego, California. 


This repository is entirely student-created and maintained.
Attached to this repository is a GitHub project called [Reefscape 2025](<https://github.com/orgs/Patribots4738/projects/15>) in which we utilize as the Agile framework to organize our workflow. With Agile, we map out the season by dividing it into many week-long sprints. As a team, we agree upon & decide what must be accomplished in each sprint. We declare each assignment by making issues and then implement it in an associated branch. An estimated priority & size are assigned to each issue/assignment which is then filtered into five categories:
  - **Backlog** -> Issues that have no status. Essentially a large to-do list.
  - **Ready** -> Issues that are assigned to a programmer & are ready to begin.
  - **In Progress** -> Issues that are currently being worked on by a programmer.
  - **In Review** -> Issues where the assigned programmer has requested revision by colleagues.
  - **Done** -> Resolved issues with corresponding branches which have merged into our master branch, [`main`](https://github.com/Patribots4738/Crescendo2025/tree/main/src/main).

We also love [drawing boards](https://www.tldraw.com/v/YKJloESPqAyu62wxqEQ8U?v=1783,102,6548,3115&p=page)!
    
We are a team of students, for students, and we are proud to be a part of the _FIRST&reg;_ community.
Thanks for checking us out, & be sure to star this repository if you find anything helpful or interesting!

### [See how we did!](https://www.statbotics.io/team/4738)

____

## ✨Highlights✨

>	- Automatic alignment to reef
>	- Two claws, one for algae and one for coral
>	- Log replay using Advantage Scope
>	- Modular autonomous routines
>	- April Tag detection using Limelight
>	- Fully simulated robot in Advantage Scope
> - Motion Magic that makes the motor move with a pre-specified velocity, acceleration, and optionally jerk during its journey towards its positional setpoint.
![image](/images/score_net.png)

## Simulation & Testing 🪄
We use AdvantageScope and its simulation tool to test our placing positions before we get the robot as well as using it for driver practice. It allows us to test things when we aren't able to use the robot. We also use Elastic to look at values from Network Tables when testing, during practice or at competitions.
![image](/images/intake_coral.png)

## Vision 📷
We use both Limelight 3g's and Limelight 4's to accurately calculate our Robots position on the field. During autonomous we put full faith in our cameras ability to give us a good estimation of our pose, but during teleop we trust our cameras more depending on if we see 2 or more tags, or see a large tag. We also switch between Megatag 1 and Megatag 2 when we need to find our rotional pose.

## State Machine
We use a state machine to ensure that our robot avoids damaging itself and field elements while moving its components such as the claw and climb.

## Autonomous 🤖
  ### PathPlanner!
  We use PathPlanner to make autopaths! We use named waypoints, scheduled commands, & bezier curves to generate a singular auto path between a starting position, a preferable placing position, or the coral station. We then link multiple auto paths together to make one predetermined autonomous path. Using the limelights mounted on it the robot can tell its position and will know where to move next. We have auto alignments that when activated will align the robot to the different reef faces, the processor, the coral station, the net, and the barge for consistency. 
![image](/images/extended1.png)

## Teleoperated 🎮
  ### Auto Alignment
  A feature that we have on the robot is Auto Alignment! This feature allows to align to field elements such as reef faces, the processor, intake station, and the barge. When alligned to a reef face, the driver can move the robot to align it to the target branch. This helps the driver with steering and alignment.
  ![image](/images/place_l3.png)

  ### Climblight
  To make sure all of the hooks on our climb and the foot has properly been locked into the cage, we use a Limelight 3g pointed where we index the cage. Using this we can make sure we dont accidentally drop test the robot during the climbing procces or fail climb in any other way.
  ![image](/images/climb_attempt.png)

### Joystick Axis Remapping
  We use Xbox controllers to control the robot. However, the range of the joystick axis from the Xbox controller is the shape of a skewed square with chamfered edges which is preferable for usage. In PatriBoxController, we remapped the range of input to a circle that is easier to use. Here's our [desmos](https://www.desmos.com/calculator/e07raajzh5) if you want to check out the math!


______
## Major Class Functions 🤩

### Subsystems
> [`subsystems/`](src/main/java/frc/robot/subsystems) Folder containing class files for each subsystem on the robot.
> - Superstructure [`subsystems/superstructure/`](src/main/java/frc/robot/subsystems/superstructure) Folder containing class files for superstructure subsystem
> - - Claw [`subsystems/superstructure/`](scr/main/java/frc/robot/subsytems/superstructure/claw) Folder containing files for both claw subsytems
>   - - Algae Claw [`subsytems/superstructure/`](scr/main/java/frc/robot/subsytems/superstructure/claw/algae) Files that allow the robot to have fun with the algae
>     - Coral Claw [`subsytems/subersturcture/`](scr/main/java/frc/robot/subsytems/superstructure/claw/coral) Files that allow the robot to manipulate
>   - Climb [`subsystems/superstructure/`](scr/main/java/frc/robot/subsytems/superstructure/climb) Folder containing files that let the robot climb (kinda... when it feels like it)
>    - Elevator [`subsystems/superstructure/`](scr/main/java/frc/robot/subsytems/superstructure/elevator) Folder containing files for the elevator that lets the robot get tall
>    - Wrist [`subsystems/superstructure/`](scr/main/java/frc/robot/subsytems/superstructure/wrist) Folder containing files for different reef angles
> - Vision [`subsystems/vision/`](src/main/java/frc/robot/subsystems/vision) Folder containing class files for the vision subsystem
> - Drive [`subsystems/drive/`](src/main/java/frc/robot/subsystems/drive) Folder containing class files for the drive subsystem
>
> 
### Commands
> [`commands/`](src/main/java/frc/robot/commands) Folder containing command files that control the robot.
>   - [`commands/characterization/`](src/main/java/frc/robot/commands/characterization) Folder that contains our FeedForward commands
>   - [`commands/drive/`](src/main/java/frc/robot/commands/drive) Folder containing our drive commands
>   - [`commands/managers/`](src/main/java/frc/robot/commands/managers) Folder containing our Hollonomic Drive Command Tuner file

### Utilities
> [`util/`](src/main/java/frc/robot/util) Folder containing values, logic, and math used by other files to help them function.
> - [`util/auto/`](src/main/java/frc/robot/util/auto) Folder containing the storage files for PathPlanner and for auto alignment
> - [`util/calc/`](src/main/java/frc/robot/util/calc) Folder containing the calculations we use for the robot's functions
> - [`util/custom/`](src/main/java/frc/robot/util/custom) Folder containing all of our custom files
> - [`util/hardware/`](src/main/java/frc/robot/util/hardware) Folder containing folders and files for each hardware component we program
> - [`util/Constants`](src/main/java/frc/robot/util/Constants.java) File containing constants used throughout the robot code to prevent a mismatch in data & hardcoding values

## Controls 🎮
![image](https://github.com/user-attachments/assets/590642e6-7ad0-47f0-adc2-b79603a9b17d)
![image](https://github.com/user-attachments/assets/3db3ddbe-14d3-4eb4-9c93-6a3d28a0b647)


## Components & Tools 🛠️
![image](/images/robot_software.svg)
![image](/images/robot_hardware.svg)

 
