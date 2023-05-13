# CS141_AssistBot
This repository is for final project of CS141: Probablistic Robotics at Tufts University, in 2023 Spring term.

The proposal is to build a health-care robot, as a personanl assistant, to do some simple tasks for the elderly or disabled crowds. The robots are developed, trained, and tested in the PyBullet similation platform. The main task of the mobile robot is to follow reach and closely follow a moving traget.

Here in the simulation, we use a TurtleBot equiped an RGB camera as well as a depth camera for the robot. For the target, you could use various objects (specified on the top, inside of the file `experiments.py`), but in practive the followed target should be a human.

While for the Reinforcement Leraning (DQN) for robotic grasping, we will be using the Kuka Robot Environment and applying our knowledge of DQN learning for the agent. Steps to perform the DQN learning can be found in the file `DQN Learning.ipynb`.

### Simulation Implementation
**Parameters Configuration**
- Target Object
  -  After loading the plane and TurtleBot as the robot, load the target object with a specified object name or `.urdf` file in `PyBullet_DATA`.
- Target Object Movement
  - In `experiment.py`, `KEYBOARD_CONTROL` defines the moving function for the target object -- if set `True`, you can use `UP/DOWN/LEFT/RIGHT` to control its movement. Otherwise it rotates clockwise in an oval orbit specified by `ORBIT_RADIUS_X`, `ORBIT_RADIUS_Y`.
- Robot Movement
  - In `experiment.py`, `AGENT` specifies using `baseline` or `predictive` motion control.
- Others(Tuning)
  - All other capital variables at the top of each file are customizable. However, to better tune the control function, below are recommended for adjusting:
    - `AHEAD_TIME_STEPS` in `control.py`

**Terminal Condition**
- The termimating condition is when the robot keeps within `REACH_THRESHOLD` distance to the target for `REACH_THRESHOLD` time seps.
- Therefore, for the criteria, we use the number of time steps for the robot to reach the terminating goal as the measurement of the robot performance.


### Run Experiments
Run the `experiment.py` under `/src` directory.
```bash
cd src
python experiment
```
Run the `DQN Learning.ipynb` under `/src` directory.  
The Juptyer Notebook has been broken down into different sections and information are provided within the Juptye Notebook to explain the purpose of each sections.  
We would like to thank for the detailed instruction and tutorial provided by [PyTorch DQN tutorial](https://pytorch.org/tutorials/intermediate/reinforcement_q_learning.html) and the post [OpeAI Gym Environments with Pybullet](https://www.etedal.net/2020/04/pybullet-panda_2.html).  
Note: for the default environemnt, it will take approximately 4 hours to complete. 
