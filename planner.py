#!/usr/bin/env python
import os
import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
from math import pi
from math import sqrt
from math import pow
from std_msgs.msg import String
#from niryo_one_msgs.msg import RobotState
from moveit_msgs.msg import  RobotState as RbState #changing name to avoid conflict with newly made RobotState msg by Niryo
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from niryo_one_commander.position.position import Position
import tf

from niryo_one_python_api.niryo_one_api import *
import time

n = NiryoOne()

test_mode=0 #classic mode 0, test mode for benchmarking 1


class MoveGroupPythonInterfaceTutorial(object):
  """MoveGroupPythonInterfaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonInterfaceTutorial, self).__init__()
		# First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial')

		# Instantiate a `RobotCommander`_ object. Provides information such as the robot's
		# kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

		# Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
		# for getting, setting, and updating the robot's internal understanding of the
		# surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

		# Instantiate a `MoveGroupCommander`_ object.  This object is an interface
		# to a planning group (group of joints).
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
        # Get end effector link
    self.end_effector_link = move_group.get_end_effector_link()
    print "end effector link :", self.end_effector_link
    
        # Set planning parameters
    move_group.allow_replanning(True)
        
    print("Arm Moveit Commander has been started")
    self.move_group = move_group
    
  def waypoints_path(self,scale=1): #This function is used to build a list of waypoints 
        
        move_group = self.move_group

        # --- Getting current pose
        wpose = move_group.get_current_pose().pose # ( -!!- values given by MoveGroupCommander are different from true values)
    	wpose.position.x=n.get_arm_pose().position.x          #
        wpose.position.y=n.get_arm_pose().position.y         #
        wpose.position.z=n.get_arm_pose().position.z         # updating with true values from niryo 
        wpose.orientation.x=n.get_arm_pose().rpy.roll        #
        wpose.orientation.y=n.get_arm_pose().rpy.pitch      #
        wpose.orientation.z=n.get_arm_pose().rpy.yaw       #
        
        waypoints=[]
        waypoints.append(copy.deepcopy(wpose))#store start state to reduce jerk at the beginning
        # --- Adding points to follow in path
        print "\n\t === Acquire a trajectory === \n"
        dir=os.getcwd()+"/Trajectories"
        print "Current directory: ", dir
        print "Files found: \n", os.listdir(dir)
        print "Getting path from file: "
        filename=raw_input()
        way=open("Trajectories/"+filename+".csv",'r')
        os.system('clear')
        print("\t-> file : "+filename+".csv")
        nbr = 0        
        line=way.readline()
        #-- counting lines and checking errors
        while line:
         nbr += 1
         line=way.readline()
         if(len(line.split(' '))!=7 and line!=""):
          print "Error line ", nbr," does not contain 3 positions + 4 quaternions values :"
          print line
          return 0
        way.seek(0)
        i=0
        print "lines:",nbr
        
        #-- calculating travelling distance
        traveling_distance=0
        wpose.position.x=0 #reset pose
        wpose.position.y=0
        wpose.position.z=0
        
        while(i<=nbr-1):
         i+=1
         tab=way.readline().split(' ')
         traveling_distance += sqrt(pow(float(tab[0])-wpose.position.x,2) + pow(float(tab[1])-wpose.position.y,2) + pow(float(tab[2])-wpose.position.z,2))

         wpose.position.x=float(tab[0]) #- Position
         wpose.position.y=float(tab[1])
         wpose.position.z=float(tab[2]) 
       
         wpose.orientation.x= float(tab[3]) #- Quaternion
         wpose.orientation.y= float(tab[4])
         wpose.orientation.z= float(tab[5])
         wpose.orientation.w= float(tab[6]) 
         waypoints.append(copy.deepcopy(wpose))
        #-- Outing trajectory
        wpose.position.z+=0.1
        waypoints.append(copy.deepcopy(wpose))
        way.close()
        
        print("Trajectory points found: "+str(len(waypoints)))
        print("Traveling distance: "+str(traveling_distance)+" meters")
        return waypoints
        

  def plan_cartesian_path(self, waypoints): #This function is used to plan the path (calculating accelerations,velocities,positions etc..)

        move_group = self.move_group
        
        # --- Saving start state
        joints=n.get_joints()
        tab_joints=[joints[0], joints[1],joints[2],joints[3],joints[4],joints[5]]
        # --- Sending start state
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['joint_1', 'joint_2','joint_3','joint_4','joint_5','joint_6']
        joint_state.position = tab_joints
        initial_state = RbState()
        initial_state.joint_state = joint_state
        move_group.set_start_state(initial_state)
        
        
        # --- Getting waypoints
        
        waypoints = waypoints
        
		# We want the Cartesian path to be interpolated at a resolution of 1 cm
		# which is why we will specify 0.01 as the eef_step in Cartesian
		# translation.  We will disable the jump threshold by setting it to 0.0,
		# ignoring the check for infeasible jumps in joint space, which is sufficient
		# for this tutorial.
        print "\n\t === Compute a trajectory ===\n"
        
        #-- Parameters
        fraction=0.0
        tries=0
        max_tries=10
        eef_step=1.0 #eef_step at 1.0 considering gcode is already an interpolation
        velocity=0.03
        
        print "Max tries authorized : ", max_tries, "eef step : ", eef_step
        t_in=time.time()
        while(fraction<1.0 and tries<10):
         (plan, fraction) = move_group.compute_cartesian_path(waypoints,eef_step, 0.0) 
										      # waypoints to follow# eef_step in meters# jump_threshold
         tries+=1
         print("\t---try:"+str(tries)+"\t---completed:"+str(fraction*100)+"%")#printing iterations
        t_out=time.time()
        c_time=t_out-t_in
        print("==>  tries: "+str(tries)+" complete: "+str(fraction*100)+"%  in: "+str(c_time)+" sec")#process results
        
        print("Retiming trajectory at "+str(velocity*100)+"% speed..")
        plan2=move_group.retime_trajectory(initial_state, plan, velocity) #ref_state_in, plan, velocity scale
        print("Done")

        return plan2 , fraction, c_time

  def execute_plan(self, plan): #This function is used to allow execution of the trajectory by Niryo arm
    move_group = self.move_group
    t_in=time.time()
    move_group.execute(plan,wait=True)
    t_out=time.time()
    print("Started at: "+str(t_in)+" elapsed time:"+str(t_out-t_in))
    return t_out-t_in
    
  def max_time_compute(self,waypoints,cycles): #This function is only used for benchmarking cartesian path method on 'cycles' computation
  
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

  def precision_on_pose(self,ref):
   list=[0,0,0,0,0,0]
   pose=n.get_arm_pose()
   list[0]= pose.position.x -ref[0]
   list[1]= pose.position.y -ref[1]
   list[2]= pose.position.z -ref[2]
   list[3]= pose.rpy.roll -ref[3]
   list[4]= pose.rpy.pitch -ref[4]
   list[5]= pose.rpy.yaw -ref[5]
   repeatability=sqrt(pow(list[0],2)+pow(list[1],2)+pow(list[2],2))
   print "distance from desired pose: ", repeatability
   return list,repeatability
   
   
#------------------------------------------------------------
#  --------   MAIN  --------------------
#------------------------------------------------------------

os.system('clear')
Instance = MoveGroupPythonInterfaceTutorial()
way = Instance.waypoints_path()
  
if(test_mode==1 and len(way)==1):# Benchmarking mode -----------
 nb=input("Number of cycles ? ")
 t_exec_maxi=0
 t_exec_mini=999
 t_cmp_maxi=0
 t_cmp_mini=999
 repeat_max=0
 repeat_moy=0
 max_errors=[0,0,0,0,0,0] #containing maximal errors on x,y,z,rx,ry,rz
 
 for i in range(0,nb):
  n.move_pose(0.2,0,0.150,0,0,0,)#waiting position
  time.sleep(0.5)
  cartesian_plan, fraction,c_time = Instance.plan_cartesian_path(way)
  if(c_time>t_cmp_maxi):
   t_cmp_maxi=c_time
  if(c_time<t_cmp_mini):
   t_cmp_mini=c_time
   
  t_exec=Instance.execute_plan(cartesian_plan)
  
  if(t_exec>t_exec_maxi):
   t_exec_maxi=t_exec
  if(t_exec<t_exec_mini):
   t_exec_mini=t_exec
   
  time.sleep(1)#sleep to get enough time to stabilize the arm
  l,repeatability=Instance.precision_on_pose([0.3-0.0388,-0.1,0.25+0.04468,0,0,0])
  if(repeatability>repeat_max):
   repeat_max=repeatability
  repeat_moy+=repeatability
  for j in range(0,6):
   if(abs(l[j])>max_errors[j]):
    max_errors[j]=abs(l[j])
 repeat_moy=repeat_moy/nb
 print "\n T_execution_maxi: ",t_exec_maxi," T_execution_mini: ",t_exec_mini
 print "T_computation_maxi: ",t_cmp_maxi," T_computation_mini: ",t_cmp_mini
 print "Maximum errors: ",max_errors
 print "Maximum repeatability", repeat_max
 print "Average repeatability", repeat_moy
 
 
 
# ---------------------------------------------------------------------------------------------------------------------------------
if(not(test_mode) and way!=0):# Classic mode -----
 cartesian_plan, fraction,c_time = Instance.plan_cartesian_path(way)
 print("Execute trajectory ? (yes/no)")
 a=raw_input()
 if(a=='yes'):
  Instance.execute_plan(cartesian_plan)


print("\n --- Program ended ---\n")
