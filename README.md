# Sawyer Mobile Device Interaction
This was my final project in Advanced Robotics. I enabled the arm robot Sawyer to perform tapping, swiping, and pinching gestures on a stationary touch-enabled device. This is the first step toward having collaborative robots perform automated application/phone tests, or be teleoperated using human biological signals as inputs.

## Challenges
- Designing and manufacturing a new end effector for Sawyer-touch screen interaction
- Controlling force applied by Sawyer to touch screen
- Fixing mobile device in stationary, upright position

## Implementation
### Hardware
#### End Effector
- CAD models made in Autodesk Fusion 360 and new "fingers" 3D printed

<img src="https://github.com/sebtona/sawyer-mobile-device-interaction/blob/main/gripper_model_front.png" alt="CAD Model Front View" width="400"/> <img src="https://github.com/sebtona/sawyer-mobile-device-interaction/blob/main/gripper_model_back.png" alt="CAD Model Back View" width="400"/>
- Fingers slide and lock onto electric parallel gripper

<img src="https://github.com/sebtona/sawyer-mobile-device-interaction/blob/main/gripper.jpg" alt="Finger on gripper" width="400"/>

- Capacitive stylus pen tips fixed to ends of fingers

<img src="https://github.com/sebtona/sawyer-mobile-device-interaction/blob/main/tip.png" alt="Stylus Pen Tip" width="300"/>

- Grounding wires run from fingertips to Sawyer endpoint
<img src="https://github.com/sebtona/sawyer-mobile-device-interaction/blob/main/parts_with_tips_and_wires.jpg" alt="Finger Assembly" width="400"/>

#### Mobile Device
- Fixed in one place using a metal pegboard and bolts

### Software
#### Sawyer
- Established LAN connection between Sawyer and workstation
- Booted Sawyer in Sawyer SDK mode to allow for communication through <a href="https://www.ros.org/" target="_blank">ROS</a>
- Created custom Python scripts for each gesture. Used <a href="https://rethinkrobotics.github.io/intera_sdk_docs/5.1.0/index.html" target="_blank">Intera SDK's</a> `intera_interface` library to access endpoint frame, joint angles, and force applied at endpoint
<img src="https://github.com/sebtona/sawyer-mobile-device-interaction/blob/main/code_sample.png" alt="Code sample" width="400"/>

- Explored alternative way to program Sawyer through built-in <a href="https://www.rethinkrobotics.com/intera" target="_blank">Intera</a> software platform
<img src="https://github.com/sebtona/sawyer-mobile-device-interaction/blob/main/intera.jpg" alt="Intera Software" width="400"/>

#### Mobile Device
- Used <a href="https://appadvice.com/app/touchscreen-test/1126113478" target="_blank">Touchscreen Test</a> mobile application as testing software to record each gesture being attempted on the device

## Deliverables
- Operation Demonstration

https://user-images.githubusercontent.com/46403390/170743748-325e1ee0-c41f-4d5c-8e6e-22a1d91676f6.mp4
- Project Presentation: https://github.com/sebtona/sawyer-mobile-device-interaction/blob/main/project_presentation.pptx
- IEEE Conference Paper: https://github.com/sebtona/sawyer-mobile-device-interaction/blob/main/ieee_conference_paper.pdf
- Python scripts: https://github.com/sebtona/sawyer-mobile-device-interaction/tree/main/scripts
- CAD model files
  - https://github.com/sebtona/sawyer-mobile-device-interaction/blob/main/touch_test_gripper.f3d
  - https://github.com/sebtona/sawyer-mobile-device-interaction/blob/main/touch_test_gripper.ipt
  - https://github.com/sebtona/sawyer-mobile-device-interaction/blob/main/touch_test_gripper.stl

## Environment Setup
### Sawyer
- Place Sawyer in front of a flat table within its workspace, ensure pegs or bolts can be secured into table and that the mobile device you are testing on can be held in place on the table by the pegs or bolts.
- Ensure that Sawyer's controller is connected to the same network as your development workstation using a CAT5 or CAT6 networking cable.
- Boot Sawyer normally, press the Rethink button to display the Head Screen Menu, use the Navigator Scroll Knob to select Info, then select About. Note Sawyer's IP address for later.
- Reboot Sawyer in SDK Mode by following the instructions in the "Switch Robot from Intera MFG to SDK Mode" section of <a href="https://support.rethinkrobotics.com/support/solutions/articles/80000980156-upgrade-sawyer-to-intera-sdk" target="_blank">this page</a>.
#### Gripper
- Acquire <a href="https://www.amazon.com/Zonon-Replacement-Conductive-Capacitive-Silicone/dp/B08P8MCZS1/ref=sr_1_6?crid=ICVNG695JGN4" target="_blank">Conductive Stylus Tips</a>, Gorilla Glue, wire strippers/cutters, electrical tape, and about two feet of wire.
- 3D Print two tool tip parts using the included touch_test_gripper.stl file, or manufacture the parts based off of the included touch_test_gripper.f3d (Fusion 360) file.
- Cut the wire in half, and strip both ends of both pieces. Wrap the end of one wire around the base of the base of the stylus tip section for each tool tip part, and lay the rest of each wire in each tool tip part's groove. Carefully Gorilla Glue the capacitive stylus tips onto the ends of each tool tip part, ensuring that they make permanent contact with the exposed wire. Gorilla Glue the unexposed wire sections into the tool tip part grooves. Slide each assembly onto Sawyer's electric parallel gripper, and use electrical tape to secure the ends of each wire to a conductive part of Sawyer's end effector.

### Workstation
- Follow the instructions <a href="https://sdk.rethinkrobotics.com/intera/Workstation_Setup" target="_blank">here</a> for initial setup of your development workstation. Enter Sawyer's IP address (noted earlier) in the 'robot_hostname' field of the intera.sh file. You do not need to setup Rviz.
- In a new terminal, navigate to your source folder and create a new package:
```sh
cd ~/ros_ws/src
catkin_create_pkg touch_test std_msgs rospy roscpp
cd ..
catkin_make
```
- Copy the included scripts folder and its contents into your new touch_test package directory.
- Ensure that all Python scripts are executable, then run another catkin_make:
```sh
cd ~/ros_ws/src/touch_test/scripts
sudo chmod +x get_endpoint_forces.py get_endpoint_pose.py get_joint_angles.py pinch.py swipe_left.py swipe_right.py tap.py
cd ~/ros_ws
catkin_make
```
- Reinitialize your SDK environment and reverify your connection to Sawyer's controller:
```sh
cd ~/ros_ws
./intera.sh
env | grep ROS
rostopic list
```

## Running Code
- Make sure you are in the SDK environment, you have sourced your workspace, and that you are in your root workspace folder:
```sh
cd ~/ros_ws
./intera.sh
source devel/setup.bash
```
- Rosrun any Python script you choose with the following syntax:
```sh
rosrun [package] [Python file]
```
- Example:
```sh
rosrun touch_test pinch.py
```

## Future Work
- ISR when applied force reaches set threshold
- Repeatability testing
- Add computer vision to recognize text/buttons and their relative positions
- Teleoperation using biological signals like EMG or gaze
