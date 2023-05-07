# this file contains helper functions for the simulation
# including loading robot and object, moving object, etc
import pybullet as p
import math


DISTANCE_THRESHOLD = 0.5
REACH_THRESHOLD = 32  # terminate after 32 seconds of reaching target
ORBIT_RADIUS_X = 1.5
ORBIT_RADIUS_Y = 2


def load_robot(robo_name="turtlebot"):
    """Load robot into simulation."""
    available_robots = ["turtlebot"]
    assert robo_name in available_robots, "Robot name not supported. Please choose from: {}".format(available_robots)
    if robo_name == "turtlebot":
        robo_path = "../data/turtlebot.urdf"
        robo = p.loadURDF(
            robo_path,
            [0, 0, 0],
            flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL,
        )  # or will collide with ground
    return robo


def load_object(obj_name="sphere", obj_pos=[1, 0, 0.1]):
    """Load object into simulation."""
    available_objects = ["duck_vhacd.urdf", "soccerball.urdf", "samurai.urdf", "block.urdf", "sphere"]
    assert obj_name in available_objects, "Object name not supported. Please choose from: {}".format(available_objects)

    if obj_name == "sphere":
        obj_shape = p.createVisualShape(
            p.GEOM_SPHERE, radius=0.1, rgbaColor=[1, 0, 0, 1]
        )
        obj = p.createMultiBody(
            baseMass=0, baseVisualShapeIndex=obj_shape, basePosition=obj_pos
        )
    else:
        obj = p.loadURDF(obj_name, obj_pos)
    return obj


def move_object(t, obj, key_control=False):
    """Move object along an oval robit over time."""
    if key_control:
        dx, dy = 0, 0
        # capture keyboard events
        keys = p.getKeyboardEvents()
        for k, v in keys.items():
            if k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_TRIGGERED):
                dx = 0.5
            if k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED):
                dx = 0
            if k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_TRIGGERED):
                dx = -0.5
            if k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED):
                dx = 0

            if k == p.B3G_UP_ARROW and (v & p.KEY_WAS_TRIGGERED):
                dy = 0.5
            if k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED):
                dy = 0
            if k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_TRIGGERED):
                dy = -0.5
            if k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED):
                dy = 0
        target_pos = p.getBasePositionAndOrientation(obj)[0]
        target_pos = [target_pos[0] + dx, target_pos[1] + dy, 1]
    else: # move along an oval
        target_pos = [ORBIT_RADIUS_X * math.sin(t), ORBIT_RADIUS_Y * math.cos(t), 1]
    p.resetBasePositionAndOrientation(obj, target_pos, [0, 0, 0, 1])
    return target_pos