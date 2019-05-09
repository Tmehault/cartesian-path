#!/usr/bin/env python
#Modules
import os
import sys
import copy
import rospy
import readline # autocompletion
from progress.bar import FillingCirclesBar, ChargingBar #progress bar
from multiprocessing import Process
import signal
import moveit_commander
import time
from math import pi, sqrt, pow
from tf.transformations import *
from niryo_one_python_api.niryo_one_api import *
#Messages
from std_msgs.msg import String, Header
from moveit_msgs.msg import  RobotState as RbState #changing name to avoid conflict with newly made RobotState msg by Niryo
from moveit_msgs.msg import RobotTrajectory, Constraints
from moveit_msgs.srv import GetCartesianPath
from sensor_msgs.msg import JointState


n = NiryoOne()

offsets_niryo=[0,0,0] #offsets for 2D+ printing (gcode origin: 0,0,0) [-0.89349, -0.08603, -0.332] 

#colors
cplanner='\033[1;104m'
cred='\033[91m'
cend='\033[0m'
cgreen='\033[1;42m'
cyellow='\033[1;93m'
cblue='\033[1;94m'

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
 state=ChargingBar('Progress',max=duration-1)
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
    
        # Set planning parameters
    move_group.allow_replanning(True)
    
    print("Successfully connected to move_group." +
                "\n" + "Started group     : " + str(move_group.get_name()) + 
                "\n" + "Planning_frame    : " + str(move_group.get_planning_frame()) + 
                "\n" + "End effector link : " + str(move_group.get_end_effector_link()) +
                "\n" + "Planning time : " + str(move_group.get_planning_time()) +
                "\n" + "pose reference frame : " + str(move_group.get_pose_reference_frame()) +
                "\n" + "goal joint tolerance : " + str(move_group.get_goal_joint_tolerance()) +
                "\n" + "goal orientation tolerance : " + str(move_group.get_goal_orientation_tolerance()) +
                "\n" + "path constraints : " + str(move_group.get_path_constraints()) +
                "\n" + "goal position tolerance : " + str(move_group.get_goal_position_tolerance()) )

    print("Arm Moveit Commander has been started")
    self.move_group = move_group
    
  def waypoints_path(self,scale=1): #This function is used to build a list of waypoints 
        
        move_group = self.move_group

        # --- Getting current pose
        wpose = move_group.get_current_pose().pose # ( -!!- values given by MoveGroupCommander are different from true values)
        
        waypoints=[]
        
        # --- Adding points to follow in path
        print(cblue+"\n\t === Acquire a trajectory === \n"+cend)
        print "Current directory: ", dir
        print "Files found: \n"
        print '\n'.join(files)
        readline.set_completer(completer)#active autocompletion on filenames
        readline.parse_and_bind("tab: complete")

        filename=raw_input(cyellow+"\nGetting path from file: "+cend)
        way=open("Trajectories/"+filename,'r')
        os.system('clear')
        print(cgreen+"\n\t   >  - -    Demonstrateur Niryo One - Surmoul 3D    - -  <   \n"+cend)
        print("\n-> file : "+'\033[4m'+filename+cend)
        print("\noffsets:"+offsets_niryo)      
        line=way.readline()
        nbr = 1  
        
        #-- counting lines and looking for errors
        while line:
         nbr += 1
         line=way.readline()
         if(len(line.split(' '))!=8 and line!=""):
          print(cred+"Error line "+str(nbr)+" does not contain 3 positions + 4 quaternions values + 1 extrusion value:"+cend)
          print line
          return 0
        way.seek(0)
        
        extru=[0] #initialized with zero to prevent extrusion between start state and beginning of the real trajectory
        traveling_distance=0 #to calculate travelling distance
        wpose.position.x=0  #reset pose
        wpose.position.y=0
        wpose.position.z=0
        
        q2=quaternion_from_euler(pi/2,0,0)
        

        bar=FillingCirclesBar('Processing waypoints', max=nbr-1)
        i=0
        while(i<=nbr-2):
         i+=1
         bar.next()
         tab=way.readline().split(' ')
         if(extru[i-1]==1):
          traveling_distance += sqrt(pow(float(tab[0])-wpose.position.x,2) + pow(float(tab[1])-wpose.position.y,2) + pow(float(tab[2])-wpose.position.z,2))

         wpose.position.x= float(tab[0]) + offsets_niryo[0]#- Position
         wpose.position.y= float(tab[1]) + offsets_niryo[1]
         wpose.position.z= float(tab[2]) + offsets_niryo[2]
       
         q1=[ - float(tab[3]),float(tab[4]),float(tab[5]),float(tab[6])] #inverse quaternion
         q3=quaternion_multiply(q1, q2) #then rotate
         q3[0]=-q3[0] #invert it back
         wpose.orientation.w= q3[0] #- Quaternion
         wpose.orientation.x= q3[1]
         wpose.orientation.y= q3[2]
         wpose.orientation.z= q3[3]
         
         extru.append(int(tab[7]))
         waypoints.append(copy.deepcopy(wpose))
         
        #-- Outing trajectory---------------
        wpose.position.z+=0.1
        wpose.orientation.x= 0 #- Quaternion
        wpose.orientation.y= 0
        wpose.orientation.z= 0
        wpose.orientation.w= 1
        waypoints.append(copy.deepcopy(wpose))
        extru.append(0) #to stop extrusion at the end
        
        way.close()
        bar.finish()
        traveling_distance=round( traveling_distance / 2.1874 , 2) #ratio filament diameter 1.75mm / nozzle 0.8mm
        print("Trajectory points found: "+str(len(waypoints)))
        print "Necessary Filament: %2dm %2dmm"%(divmod(traveling_distance,1000))
        return waypoints, extru
        

  def plan_cartesian_path(self, waypoints, start_joints): #This function is used to plan the path (calculating accelerations,velocities,positions etc..)

        move_group = self.move_group
        
        #check for robot stability 
        if(n.get_learning_mode()):
         print "Make sure that Niryo is not in learning mode and in a safe position close to where you want to print"
         raw_input("Then press enter..")
        
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
        
        
        # --- Getting waypoints localy
        waypoints = waypoints

        #-- Parameters
        fraction=0.0
        tries=0
        max_tries=5
        eef_step=1.0 #eef_step at 1.0 considering gcode is already an interpolation
        velocity=1.0
        
        print "\n --- Computing parameters ---"
        print "| Max tries authorized : ", max_tries, "\n| Eef step : ", eef_step, "\n| Velocity : ",velocity*100,"%"
        print " ------------------------------\n"
        
        try:
            rospy.wait_for_service('compute_cartesian_path',2)
        except (rospy.ServiceException, rospy.ROSException), e:
            print("Service call failed:",e)
            return None 
        try:
            moveit_cartesian_path = rospy.ServiceProxy('compute_cartesian_path', GetCartesianPath) #call cartesian service
        except rospy.ServiceException,e:
            print("Service call failed:",e)
            return(None)
            
        #-- Computation
        best_frac=0.0
        t_in=time.time()
        while(fraction<1.0 and tries<max_tries):
         rospy.wait_for_service('compute_cartesian_path',2) #wait for service to be ready
         response = moveit_cartesian_path(Header(), initial_state, 'arm', 'tool_link',waypoints, eef_step, 0.0, True, Constraints())
         tries+=1
         fraction=round(response.fraction,5)
         if(fraction<1.0):
          print(cred+"---try:"+str(tries)+"\t---completed:"+str(fraction*100)+"%"+cend)#printing iterations
          if(fraction>best_frac):#saving best plan
           best_plan=response.solution
           best_frac=response.fraction
         else:
          print("---try:"+str(tries)+"\t---completed:"+str(fraction*100)+"%")#printing iterations
          best_plan=response.solution
          best_frac=response.fraction
          
         time.sleep(1) #time sleep to cut CPU usage and let some cooling time
          
        t_out=time.time()
        c_time=round(t_out-t_in,2)
        print("\n==>  tries: "+str(tries)+" complete: "+str(best_frac*100)+"%  in: "+str(c_time)+" sec")#process results
        
        #-- Scaling speeds for Niryo One
        if(velocity<1.0):
         print"==>  Retiming trajectory at ",velocity*100,"% speed..",
         best_plan=move_group.retime_trajectory(initial_state, best_plan, velocity) #ref_state_in, plan, velocity sc
         print "Done"
        
        expect_m, expect_s = divmod( best_plan.joint_trajectory.points[-1].time_from_start.secs , 60)
        expect_h, expect_m = divmod( expect_m  , 60)
        print "\nExpected printing time : %dh%02dm%02ds" %(expect_h,expect_m,expect_s)
        end_joints=list(best_plan.joint_trajectory.points[-1].positions)# returns last position in case of using multiple trajectories that are following each other
        
        return best_plan , end_joints

  def execute_plan(self, plan): #This function is used to allow execution of the trajectory by Niryo arm
    move_group = self.move_group
    t_in=time.time()
    print "Started at : ", time.asctime(time.localtime(t_in))
    move_group.execute(plan,wait=True)
    t_out=time.time()
    print "Finished at : ", time.asctime(time.localtime(t_out))
    m,s=divmod(t_out-t_in,60)
    h,m=divmod(m,60)
    print "Elapsed : %dh%02dm%02ds" %(h,m,s)
    
  def timing_extrusion(self, extru, traj):
    tref=time.time()
    for i in range(0,len(extru)):
     t=tref + traj.points[i].time_from_start.secs + traj.points[i].time_from_start.nsecs * pow(10,-9)
     if(traj.points[i].time_from_start.secs - traj.points[i-1].time_from_start.secs > 1):
      time.sleep(traj.points[i].time_from_start.secs - traj.points[i-1].time_from_start.secs - 0.5) #sleep to reduce CPU usage when high duration between points
     while( time.time() < (t)):
      pass
     n.digital_write(GPIO_1C, extru[i])
          

    
   
#------------------------------------------------------------
#  --------   MAIN  --------------------
#------------------------------------------------------------

Instance = PlannerInterface()
quit=False
while(not(quit)):
 os.system('clear')
 print(cgreen+"\n\t   >  - -    Demonstrateur Niryo One - Surmoul 3D    - -  <   \n"+cend)
 way, extru = Instance.waypoints_path()
# ---------------------------------------------------------------------------------------------------------------------------------
 
 if( way!=0):
  cartesian_plan, end_joints = Instance.plan_cartesian_path(way,n.get_joints())
  f=open('traj.txt','w')
  f.write(str(cartesian_plan))
  f.close()
  a=raw_input(cyellow+"Execute trajectory ? (yes/no/quit) "+cend)
  if(a=='yes'):
   n.pin_mode(GPIO_1B,PIN_MODE_OUTPUT)
   n.digital_write(GPIO_1B,1)
   n.pin_mode(GPIO_1C,PIN_MODE_OUTPUT) 
   job=Process(target=time_bar, args=(cartesian_plan.joint_trajectory.points[-1].time_from_start.secs,))
   job2=Process(target=Instance.timing_extrusion, args=(extru, cartesian_plan.joint_trajectory,))
   job.start()
   job2.start()
   Instance.execute_plan(cartesian_plan)
   os.kill(int(job.pid), signal.SIGKILL) #kill process by the hard method (terminate and join doesnt work)
   os.kill(int(job2.pid), signal.SIGKILL) #kill process by the hard method (terminate and join doesnt work)
   quit=True
  if(a=='quit'):
   quit=True
  


print("\n"+cgreen+"\t\t --- Program end --- "+cend+"\n")
