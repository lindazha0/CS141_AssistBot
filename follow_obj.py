import pybullet as p
import pybullet_data
import time
import math

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("r2d2.urdf", [0, 0, 0.5])  # or will collide with ground
target = p.createVisualShape(p.GEOM_SPHERE, radius=0.1, rgbaColor=[1, 0, 0, 1])
target_obj = p.createMultiBody(
    baseMass=0, baseVisualShapeIndex=target, basePosition=[0, 0, 0.1])

# obtain robot joint information
num_joints = p.getNumJoints(robot)
for i in range(num_joints):
    joint_info = p.getJointInfo(robot, i)
    print(f"Joint index: {i}, Joint name: {joint_info[1].decode('utf-8')}")


def proportional_control(robot_pos, robot_orn, target_pos, gain):
    # calculate linear and angular velocity required to reach target
    error = [target_pos[i] - robot_pos[i] for i in range(2)]
    linear_vel = gain * math.sqrt(sum(e**2 for e in error))

    # calculate angular velocity required to reach target
    robot_yaw = p.getEulerFromQuaternion(robot_orn)[2]
    angular_err = math.atan2(error[1], error[0]) - robot_yaw

    # normalize to -pi to pi
    angular_err = math.atan2(math.sin(angular_err), math.cos(angular_err))
    angular_vel = gain * angular_err

    # convert to wheel velocities
    left_force = linear_vel - angular_vel
    right_force = linear_vel + angular_vel
    return left_force, right_force


# run simulation
time_step = 1/240
gain = 10
start_time = time.time()

# Set lateral friction for R2D2's wheels
p.changeDynamics(robot, 2, lateralFriction=1)
p.changeDynamics(robot, 3, lateralFriction=1)
p.changeDynamics(robot, 6, lateralFriction=1)
p.changeDynamics(robot, 7, lateralFriction=1)

while True:
    # update target
    elapsed_time = (time.time() - start_time) / 8
    target_pos = [math.sin(elapsed_time), math.cos(elapsed_time), 1]
    p.resetBasePositionAndOrientation(target_obj, target_pos, [0, 0, 0, 1])

    # get robot state
    robot_pos, robot_orn = p.getBasePositionAndOrientation(robot)

    # apply control
    # apply control
    left_vel, right_vel = proportional_control(robot_pos, robot_orn, target_pos, gain)
    p.setJointMotorControl2(robot, 2, p.VELOCITY_CONTROL, targetVelocity=right_vel)
    p.setJointMotorControl2(robot, 3, p.VELOCITY_CONTROL, targetVelocity=right_vel)
    p.setJointMotorControl2(robot, 6, p.VELOCITY_CONTROL, targetVelocity=left_vel)
    p.setJointMotorControl2(robot, 7, p.VELOCITY_CONTROL, targetVelocity=left_vel)

    # step simulation
    p.stepSimulation()
    time.sleep(time_step)
