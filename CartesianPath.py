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
    
    

  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose #recuperation pose actuelle ( -!!- differente de la vraie position: ecarts de pose)
    	wpose.position.x=n.get_arm_pose().position.x #mise a jour avec les vraies valeurs de pose
        wpose.position.y=n.get_arm_pose().position.y
        wpose.position.z=n.get_arm_pose().position.z
        wpose.orientation.x=n.get_arm_pose().rpy.roll
        wpose.orientation.y=n.get_arm_pose().rpy.pitch
        wpose.orientation.z=n.get_arm_pose().rpy.yaw
        print("\nPose :",wpose)
        
        wpose.position.y += scale * 0.1 #remplacement par le 1er mvt pour OUBLIER LA POSE INITIALE (moveit bug)
        wpose.position.y -= scale * 0.1
        #waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.05 
        waypoints.append(copy.deepcopy(wpose))
    
        wpose.position.y -= scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        
        wpose.position.x -= scale * 0.05
        waypoints.append(copy.deepcopy(wpose))

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
										      # waypoints to follow# eef_step# jump_threshold
         tries+=1
         print("\t---try:"+str(tries)+"\t---completed:"+str(fraction*100)+"%")#affichage des iterations
        t_fin=time.time()
        c_time=t_fin-t_deb
        print("==>  tries: "+str(tries)+" complete: "+str(fraction*100)+"%  in: "+str(c_time)+" sec")#recap du processus

        return plan , fraction, c_time


  def execute_plan(self, plan):
    move_group = self.move_group
    move_group.execute(plan,wait=True)

    # **Note:** The robot's current joint state must be within some tolerance of the
    # first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    # END_SUB_TUTORIAL
    
    
  def max_time_compute(self):#Only for benchmarking cartesian path method on 10 tries
  
   move_group=self.move_group
   max_time=0
   average=0
   for i in range (0,10):
    plan,fraction,c_time=tutorial.plan_cartesian_path()
    average=average+c_time
    if (c_time>max_time):
     max_time=c_time
   average=average/10
   print("==> average time computation: "+str(average)+" sec | max time computation: "+str(max_time)) 

tutorial = MoveGroupPythonInterfaceTutorial()
cartesian_plan, fraction,c_time = tutorial.plan_cartesian_path()
#tutorial.max_time_compute()
#print("\n",cartesian_plan)

tutorial.execute_plan(cartesian_plan)

print("\n====FIN\====\n")
