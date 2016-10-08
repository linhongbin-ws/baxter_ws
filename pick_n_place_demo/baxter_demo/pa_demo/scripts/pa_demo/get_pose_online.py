#!/usr/bin/env python
# This program is tightly connected to the pa_localizatio package as of June 2016.
# Requriments: pa_localization
# Description:
# It assumes that the visual system has been calibrated according to table_pos_calibration.py or table_pos_calibration2.py
# Baxter's arm (currently right) is expected to be teleoperated to the origin location as defined by the visual system, and 
# in a ready position to grasp a rectangular object for a pick and place operation. 
# 
# The code is used to continue to test the calibration routine for pa_localization. Once the picking box is moved to a location,
# whose angle is parallel to the visual system/table, then move arm to that location using teleoperation, and call this class.
# It will return the pose of the object at that location.
# One should then take this location and subtract the (x,y)_visualsys_coord from the baxte_pose. Note that (x,y)_visualsys_coord = (y,x)_baxter_coord
# 
# That result can then be used as the arm_manipulation.py::ref_origin_pose variable for baxter.
#------------------------------------ Imports ------------------------------
import ipdb

import argparse
import sys

import tf
import rospy


# Pose Stamped and Transformation
from geometry_msgs.msg import (
    PoseStamped,
    Quaternion,
)
# Angle Transformations
from rbx1_nav.transform_utils import quat_to_angle

# Kinematics
from baxter_pykdl import baxter_kinematics
import PyKDL
from math import pi

# Baxter Messages
import baxter_interface
from baxter_interface import CHECK_VERSION


def main():

    # Get argument string for limb
    arg_fmt = argparse.RawDescriptionHelpFormatter # create ArgumentParser object
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments') # set required strings
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='send joint trajectory to which limb'
    )
    args = parser.parse_args(rospy.myargv()[1:]) # return objects

    # Set limb and gripper side
    limb = args.limb

    # Get Current Joint Positions first and command them
    arm = baxter_interface.limb.Limb(limb)
    current_angles = [arm.joint_angle(joint) for joint in arm.joint_names()]

    # call the Limb's call endpoint_pose() to get the current end-effector pose.
    referencePose=arm.endpoint_pose()
    print(referencePose['position'])
    print(referencePose['orientation'])

    # Print the RPY angles (from the Quaternion)
    qref=referencePose['orientation']
    rot_mat=PyKDL.Rotation.Quaternion(qref.x,qref.y,qref.z,qref.w)
    rot_goal=rot_mat.GetRPY()
    rot_goal=list(rot_goal) 
    print(rot_goal)

    # Create and print the quaternioni as a baxter_interface.Quaternion object
    q_goal=tf.transformations.quaternion_from_euler(0,0,0,axes='sxyz').tolist()
    q_goal=baxter_interface.limb.Limb.Quaternion(q_goal[0],q_goal[1],q_goal[2],q_goal[3])
    print(q_goal)


if __name__ == "__main__":
    rospy.init_node("get_pose")
    main()
