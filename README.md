# Autonomous Target Localization for Robotic Lung Ultrasound Imaging
## Developed a robotic system for semi-autonomous lung ultrasound imaging
### Directed Research in Medical FUSION lab, Worcester Polytechnic Institute
---
- Contributions:
  - Automatic patient recognition and scanning target localzaition using DensePose
  - Surface Normal Solver using RealSense camera
  - Velocity based PD controller for Franka Emika robot
  - System integration and validation

- Usage:
  - franka_example_controllers
    - lower level control of franka robot, modified from https://github.com/frankaemika/franka_ros/tree/kinetic-devel/franka_example_controllers
  - robotic_ultrasound
    - higher level control of franka robot
  - data_processing
    - MATLAB scripts to process experimental data
