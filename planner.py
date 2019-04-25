#!/usr/bin/env python
import os
import sys
import copy
import rospy
import readline # autocompletion
from progress.bar import FillingCirclesBar #progress bar
from progress.bar import ChargingBar
from multiprocessing import Process
import signal
import moveit_commander
import geometry_msgs.msg
from math import pi, sqrt, pow
from std_msgs.msg import String
#from niryo_one_msgs.msg import RobotState
from moveit_msgs.msg import  RobotState as RbState #changing name to avoid conflict with newly made RobotState msg by Niryo
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from niryo_one_commander.position.position import Position
import tf


from niryo_one_python_api.niryo_one_api import *
import time

n = NiryoOne()

test_mode=0 #classic mode 0, test mode for benchmarking 1

#colors
cplanner='\033[104m'
cred='\033[91m'
cend='\033[0m'
cgreen='\033[92m'
cyellow='\033[93m'
cblue='\033[94m'

dir=os.getcwd()+"/Trajectories"
files=os.listdir(dir)

def completer(text, state):
    options = [x for x in files if x.startswith(text)]
    try:
        return options[state]
    except IndexError:
        return None
        
def time_bar(duration):
 t_in=time.time()
 t_actual=t_in
 state=ChargingBar('Progress',max=duration)
 while(int(t_actual-t_in)<duration):
  time.sleep(1)
  t_actual=time.time()
  state.next()
 state.finish()

class PlannerInterface(object):
  
  def __init__(self):
    super(PlannerInterface, self).__init__()
		# First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('PlannerInterface')

		# Instantiate a `MoveGroupCommander`_ object.  This object is an interface to a planning group (group of joints).
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
        print(cblue+"\n\t === Acquire a trajectory === \n"+cend)
        print "Current directory: ", dir
        print "Files found: \n"
        print '\n'.join(files)
        readline.set_completer(completer)#active autocompletion on filenames
        readline.parse_and_bind("tab: complete")
        
        filename=raw_input("\nGetting path from file: ")
        way=open("Trajectories/"+filename,'r')
        os.system('clear')
        print("\t-> file : "+filename)
        nbr = 0        
        line=way.readline()
        #-- counting lines and checking errors
        
        while line:
         nbr += 1
         line=way.readline()
         if(len(line.split(' '))!=7 and line!=""):
          print(cred+"Error line "+str(nbr)+" does not contain 3 positions + 4 quaternions values :"+cend)
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
        
        bar=FillingCirclesBar('Processing waypoints', max=nbr)
        while(i<=nbr-1):
         i+=1
         bar.next()
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
        bar.finish()
        print("Trajectory points found: "+str(len(waypoints)))
        print("Traveling distance: "+str(traveling_distance)+" meters")
        return waypoints
        

  def plan_cartesian_path(self, waypoints, start_joints): #This function is used to plan the path (calculating accelerations,velocities,positions etc..)

        move_group = self.move_group
        
        # --- Saving start state
        tab_joints=[start_joints[0], start_joints[1],start_joints[2],start_joints[3],start_joints[4],start_joints[5]]
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
        
        print(cblue+"\n\t === Compute a trajectory ===\n"+cend)
        
        #-- Parameters
        fraction=0.0
        tries=0
        max_tries=10
        eef_step=1.0 #eef_step at 1.0 considering gcode is already an interpolation
        velocity=0.03
        
        #-- Computation
        print "Max tries authorized : ", max_tries, "\neef step : ", eef_step
        t_in=time.time()
        while(fraction<1.0 and tries<10):
         (plan, fraction) = move_group.compute_cartesian_path(waypoints,eef_step, 0.0) 
										      # waypoints to follow# eef_step in meters# jump_threshold
         tries+=1
         if(fraction<1.0):
          print(cred+"\t---try:"+str(tries)+"\t---completed:"+str(fraction*100)+"%"+cend)#printing iterations
         else:
          print(cgreen+"\t---try:"+str(tries)+"\t---completed:"+str(fraction*100)+"%"+cend)#printing iterations
        t_out=time.time()
        c_time=t_out-t_in
        print("==>  tries: "+str(tries)+" complete: "+str(fraction*100)+"%  in: "+str(c_time)+" sec")#process results
        
        print("Retiming trajectory at "+str(velocity*100)+"% speed..")
        plan=move_group.retime_trajectory(initial_state, plan, velocity) #ref_state_in, plan, velocity scale
        print("Done")
        print "Expected printing time :", plan.joint_trajectory.points[-1].time_from_start.secs," secs",plan.joint_trajectory.points[-1].time_from_start.nsecs,"nsecs"
        end_joints=list(plan.joint_trajectory.points[-1].positions)
        return plan , end_joints

  def execute_plan(self, plan): #This function is used to allow execution of the trajectory by Niryo arm
    move_group = self.move_group
    
    t_in=time.time()
    print "Started at : ", time.asctime(time.localtime(t_in))
    move_group.execute(plan,wait=True)
    t_out=time.time()
    print "Finished at : ", time.asctime(time.localtime(t_out))
    print "Elapsed : ", t_out-t_in
  
  def tracker(self,duration): #--Not working: arm pose doesnt update
    for i in range(0,duration):
     a=n.get_arm_pose()
     print a
     print "\t---------------------------- ",i
     time.sleep(1)
    

    
   
#------------------------------------------------------------
#  --------   MAIN  --------------------
#------------------------------------------------------------

os.system('clear')
print(cplanner+"\n\t   >  - -    Demonstrateur Niryo One - Surmoul 3D    - -  <   \n"+cend)
Instance = PlannerInterface()
way = Instance.waypoints_path()

# ---------------------------------------------------------------------------------------------------------------------------------
if(not(test_mode) and way!=0):# Classic mode -----
 cartesian_plan, end_joints = Instance.plan_cartesian_path(way,n.get_joints())
 a=raw_input("Execute trajectory ? (yes/no) ")
 if(a=='yes'):
  job=Process(target=time_bar, args=(cartesian_plan.joint_trajectory.points[-1].time_from_start.secs,))
  job.start()
  Instance.execute_plan(cartesian_plan)
  os.kill(int(job.pid), signal.SIGKILL) #kill process by the hard method (terminate and join doesnt work)
  


print(cgreen+"\n --- Program ended ---\n"+cend)
