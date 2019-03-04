#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from niryo_one_msgs.msg import RobotState
from moveit_msgs.msg import  RobotState as RbState #changing name to avoid conflict with newly made RobotState msg by Niryo
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import niryo_one_commander
from niryo_one_commander.position.position import Position

from niryo_one_python_api.niryo_one_api import *
import time

n = NiryoOne()

class MoveGroupPythonInterfaceTutorial(object):
  """MoveGroupPythonInterfaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonInterfaceTutorial, self).__init__()

		# BEGIN_SUB_TUTORIAL setup
		#
		# First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial')
  
  #------------------------------------------------------------------------------
		# Instantiate a `RobotCommander`_ object. Provides information such as the robot's
		# kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

		# Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
		# for getting, setting, and updating the robot's internal understanding of the
		# surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

		# Instantiate a `MoveGroupCommander`_ object.  This object is an interface
		# to a planning group (group of joints).  In this tutorial the group is the primary
		# arm joints in the Panda robot, so we set the group's name to "panda_arm".
		# If you are using a different robot, change this value to the name of your robot
		# arm planning group.
		# This interface can be used to plan and execute motions:
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    
        # Get end effector link
    self.end_effector_link = move_group.get_end_effector_link()
    
        # Set planning parameters
    move_group.allow_replanning(True)

    print("Successfully connected to move_group." +
                "\n" + "Started group     : " + str(move_group.get_name()) + 
                "\n" + "Planning_frame    : " + str(move_group.get_planning_frame()) + 
                "\n" + "End effector link : " + str(self.end_effector_link))
        
    print("Arm Moveit Commander has been started")
    
    self.move_group = move_group
    
    sub_pose = rospy.Subscriber('/niryo_one/robot_state', RobotState)
    
  def waypoints_path(self,scale=1): #This function is used to build a list of waypoints 
        
        move_group = self.move_group

        # --- Getting current pose
        wpose = move_group.get_current_pose().pose # ( -!!- values gave by MoveGroupCommander are different from true values)
    	wpose.position.x=n.get_arm_pose().position.x #updating with true values from niryo 
        wpose.position.y=n.get_arm_pose().position.y
        wpose.position.z=n.get_arm_pose().position.z
        wpose.orientation.x=n.get_arm_pose().rpy.roll
        wpose.orientation.y=n.get_arm_pose().rpy.pitch
        wpose.orientation.z=n.get_arm_pose().rpy.yaw
        
        waypoints=[]
        
        # --- Adding points to follow in path
        wpose.position.y += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))   #need deepcopy to recursively copy all the data

        wpose.position.x += scale * 0.05    
        wpose.position.y -= scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        
        wpose.position.x -= scale * 0.05
        wpose.position.y -= scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        
        wpose.position.x += scale * 0.05
        wpose.position.y += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        
        wpose.position.y += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.05    
        wpose.position.y -= scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        
        wpose.position.x -= scale * 0.05
        wpose.position.y -= scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        
        wpose.position.x += scale * 0.05
        wpose.position.y += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        
        wpose.position.y += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.05    
        wpose.position.y -= scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        
        wpose.position.x -= scale * 0.05
        wpose.position.y -= scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        
        wpose.position.x += scale * 0.05
        wpose.position.y += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        
        print("Trajectory points found: "+str(len(waypoints)))
        return waypoints

  def plan_cartesian_path(self, waypoints): #This function is used to plan the path (calculating accelerations,velocities,positions etc..)
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
        move_group = self.move_group
        
        
        # --- Start state
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['joint_1', 'joint_2','joint_3','joint_4','joint_5','joint_6']
        joint_state.position = [n.get_joints()[0], n.get_joints()[1],n.get_joints()[2],n.get_joints()[3],n.get_joints()[4],n.get_joints()[5]]
        moveit_robot_state = RbState()
        moveit_robot_state.joint_state = joint_state
        move_group.set_start_state(moveit_robot_state)
        
        
        # --- Getting waypoints
        
        waypoints = waypoints
        
		# We want the Cartesian path to be interpolated at a resolution of 1 cm
		# which is why we will specify 0.01 as the eef_step in Cartesian
		# translation.  We will disable the jump threshold by setting it to 0.0,
		# ignoring the check for infeasible jumps in joint space, which is sufficient
		# for this tutorial.
        fraction=0.0
        tries=0
        t_deb=time.time()
        while(fraction<1.0):
         (plan, fraction) = move_group.compute_cartesian_path(waypoints,0.001, 0.0)
										      # waypoints to follow# eef_step in meters# jump_threshold
         tries+=1
         print("\t---try:"+str(tries)+"\t---completed:"+str(fraction*100)+"%")#affichage des iterations
        t_fin=time.time()
        c_time=t_fin-t_deb
        print("==>  tries: "+str(tries)+" complete: "+str(fraction*100)+"%  in: "+str(c_time)+" sec")#recap du processus

        return plan , fraction, c_time

  def execute_plan(self, plan): #This function is used to allow execution of the trajectory by Niryo arm
    move_group = self.move_group
    t_deb=time.time()
    move_group.execute(plan,wait=True)
    t_fin=time.time()
    print("Started at: "+str(t_deb)+" elapsed time:"+str(t_fin-t_deb))

    # **Note:** The robot's current joint state must be within some tolerance of the
    # first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    # END_SUB_TUTORIAL
    
    
  def max_time_compute(self,waypoints,cycles): #This function is only used for benchmarking cartesian path method on 10 computation
  
   move_group=self.move_group
   max_time=0
   average=0
   for i in range (0,cycles):
    plan,fraction,c_time=self.plan_cartesian_path(waypoints)
    average=average+c_time
    if (c_time>max_time):
     max_time=c_time
   average=average/cycles
   print("==> average time computation: "+str(average)+" sec | max time computation: "+str(max_time)) 

Instance = MoveGroupPythonInterfaceTutorial()
way = Instance.waypoints_path()
cartesian_plan, fraction,c_time = Instance.plan_cartesian_path(way)
Instance.max_time_compute(way,30)
#Instance.execute_plan(cartesian_plan)

print("\n --- Program ended ---\n")
