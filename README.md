## Project Overview

This project is a part of the Introduction to Robotics course in the Department of Electrical and Computer Engineering at University of Patras. It involves programming a KUKA youBot robot to perform a pick-and-place task using Python. The main objective is to automate the process of picking a cube from a conveyor belt and placing it into a designated box on a cart. 

The task involves the following steps:
1. Move the robot to the conveyor belt where the cube is located.
2. Adjust the robot's joints to pick up the cube.
3. Transport the cube to a specified box on a cart and place it accurately.

Both the robot and the cube start at known coordinates. The robot must use precise joint movements to grasp the cube and place it at the target location.

## Solution Description

### Goal 1: Reach and Grasp the Cube
- **Distance Calculation:** The distance between the robot's initial position and the cube is calculated.
- **Inverse Kinematics:** Joint angles are determined to maximize the arm's reach, minimizing the movement time.
  - **Joint Angle Calculation:** The primary joint (q1) rotates to -65°, the second joint (q2) adjusts to -25°, and the third joint (q3) is calculated to be -28.65° after practical adjustments.
- **Arm Length Calculation:** The effective length of the robot's arm is computed using the projections of the joint links.

### Goal 2: Move to Target Position
- **Backward Movement:** The robot moves backward to align its center with the target coordinates on the X-axis. The distance moved is calculated using the robot's wheel radius and perimeter, among other parameters.

### Goal 3: Rotate and Place the Cube
- **Rotation:** The robot rotates 90° to align the arm with the target position.
- **Distance Calculation:** The distance from the robot's arm to the target is computed, ensuring the cube is placed precisely.

## Extensions and Improvements

To enhance the robot's flexibility and adaptability:
- **Initial Position Adjustments:** The code is adapted to handle different initial Z-axis positions by rotating the robot and recalculating the path.
- **Practical Adjustments:** Minor errors are corrected using a trial-and-error approach to ensure accuracy in real-world applications.
- **Rotation Calculations:** Empirical data is used to determine the relationship between wheel rotations and the robot's angular displacement.

## Observations
- **Practical Adjustments:** Theoretical calculations required fine-tuning through practical experiments.
- **Rotation Dynamics:** Due to the unique nature of Mecanum wheels, specific experiments were conducted to establish accurate rotational movements.

## Sources
- [KUKA YouBot Technical Specs](https://www.generationrobots.com/img/Kuka-YouBot-Technical-Specs.pdf)
- [Cyberbotics YouBot Guide](https://www.cyberbotics.com/doc/guide/youbot)
- [YouTube Video on YouBot](https://www.youtube.com/watch?v=noqBUEgyQ8A)
- [Mecanum Wheel - Wikipedia](https://en.wikipedia.org/wiki/Mecanum_wheel)


## Result

https://github.com/alexkalergis/Robot-Pick-and-Place/assets/105602973/53bc6f45-58f8-47d7-8b70-3c20d83a6f95

