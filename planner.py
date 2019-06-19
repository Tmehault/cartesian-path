#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Modules
import os
import sys
import copy
import rospy
import readline # autocompletion
import serial #arduino serial connection
from progress.bar import Bar #progress bar
from multiprocessing import Process
import signal
import moveit_commander
import time
from math import pi, sqrt, pow, ceil
from tf.transformations import quaternion_from_euler, quaternion_multiply,quaternion_conjugate
from niryo_one_python_api.niryo_one_api import *
#Messages
from std_msgs.msg import String, Header
from moveit_msgs.msg import  RobotState as RbState #changing name to avoid conflict with newly made RobotState msg by Niryo
from moveit_msgs.msg import RobotTrajectory, Constraints
from moveit_msgs.srv import GetCartesianPath
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose


n = NiryoOne()

offsets_niryo=[0,0,0] #offsets for 2D+ printing (gcode origin: 0,0,0) 

#Colors 
cplanner='\033[1;104m'
cred='\033[91m'
cend='\033[0m'
cgreen='\033[1;42m'
cyellow='\033[1;93m'
cblue='\033[1;94m'

#Global variables
dir=os.getcwd()+"/Trajectories"
files=os.listdir(dir)

def completer(text, state): #This function is doing the autocompletion to choose a file easier
    options = [x for x in files if x.startswith(text)]
    try:
        return options[state]
    except IndexError:
        return None
        
def time_bar(duration): #This function is only updating a loading bar while printing (visual effect)
 t_in=time.time()
 t_actual=t_in
 state= Bar('Progress',max=duration-1,empty_fill='-',fill=cblue+'>'+cend,suffix='%(percent).1f%% - Remaining:%(eta)ds')
 while(int(t_actual-t_in)<duration):
  time.sleep(1)
  t_actual=time.time()
  state.next()
 state.finish()


def check_trajectory(cartesian_plan, frac, way, distance_btwn_points):
   print("-> Auto-check trajectory module")
   if(frac!=1.0):
     a=raw_input(cyellow+"Avoid singularities ?(y/n)"+cend)
     if(a=='y'): #-Skip unreachable points
         avoided=0
         total_trajectory=len(way)
         while(frac<0.4 and avoided<0.15*total_trajectory): #need 100% of the trajectory or <15% of the total trajectory avoided
             os.system('clear')
             print(cgreen+"\n\t   >  - -     Niryo One - Surmoul 3D    - -  <   \n"+cend)
             print "\nPoints avoided: %d\n"%(avoided)
             i=int(ceil(frac*len(way))) #get last point to calculate: frac represents the already calculated path and ceil rounds the number up so we get the next point that caused the problem
             del way[i]
             distance_btwn_points[i+1]+=distance_btwn_points[i] #updating distance_btwn_points in case we need to print
             del distance_btwn_points[i]
             avoided+=1
             cartesian_plan, end_joints ,frac = Instance.plan_cartesian_path(1,way,n.get_joints())
         print "\nPoints avoided to satisfy trajectory %d on %d (%.1f %%)"%(avoided,total_trajectory, avoided*100/(total_trajectory))
   error=0
   for i in range(0,len(cartesian_plan.joint_trajectory.points)-1):
       t=cartesian_plan.joint_trajectory.points[i].time_from_start.secs + cartesian_plan.joint_trajectory.points[i].time_from_start.nsecs * pow(10,-9)
       t1=cartesian_plan.joint_trajectory.points[i+1].time_from_start.secs + cartesian_plan.joint_trajectory.points[i+1].time_from_start.nsecs * pow(10,-9)
       if(t1<=t):
           cartesian_plan.joint_trajectory.points[i+1].time_from_start.nsecs +=1 #to avoid not increasing time error
           error+=1
   print "-Time not increasing- errors fixed : %d"%(error)

#----------------------------
#------- OUTILS Arduino------
#----------------------------
'''Actions possibles:
    t1-chauffe
    t2-maj objectif temperature
    t3-affiche temperature
    t4-chargement materiau
    t5-déchargement materiau
    t6-remplir tableau d extrusion
    t7-extrusion vitesse donnee
    t8-stop extrusion
    t9-stop chauffe
  
  '''   
class Arduino():
    n.pin_mode(GPIO_1B,PIN_MODE_OUTPUT)
    n.pin_mode(GPIO_1C,PIN_MODE_OUTPUT)
    com=serial.Serial('/dev/ttyACM0',57600)
    time.sleep(1) #delay to safely start serial communication
    com.reset_input_buffer() #emptying previous data transfers
    com.reset_output_buffer()

    def empty(self):
        self.com.reset_input_buffer()
        self.com.reset_output_buffer()

    def send(self,message):
        self.com.write(message)
        n.digital_write(GPIO_1B,True)
        print self.com.readline()
        n.digital_write(GPIO_1B,False)

    def heat(self):
        self.send('t1')
        
    def temperature_objective(self):
        temp=int(raw_input('Temperature to reach ? '))
        self.send('t2x'+str(temp))

    def print_temperature(self):
        self.send('t3')
        print self.com.readline()

    def filament_load(self):
        self.send('t4')

    def filament_unload(self):
        self.send('t5')

    def extrusion_tab(self,tab,distance_btwn_points):
        self.send('t6')
        coeff_extru=0.5
        self.com.write('x'+str(len(tab)-1)) #Send number of orders
        for i in range(0,len(tab)-1): #Send orders
           t_next= tab[i].time_from_start.secs + tab[i].time_from_start.nsecs * pow(10,-9)
           dt= tab[i+1].time_from_start.secs + tab[i+1].time_from_start.nsecs * pow(10,-9) - t_next
           self.com.write('x'+str(int(distance_btwn_points[i]*coeff_extru/dt*1000)) ) #Send necessary speed
           time.sleep(0.001) #delay to be sure there is no loss while transferring via serial connection
        print "Sending finished"
        print self.com.readline()

    def manual_extrusion(self):
        speed=int(raw_input("Extrusion speed ? "))
        self.send('t7x'+str(speed))
        
    def stop_extrusion(self):
        self.send('t8')

    def stop_heating(self):
        self.send('t9')

class PlannerInterface(object):
  
  def __init__(self):
    super(PlannerInterface, self).__init__()
		# First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('PlannerInterface')

		# Instantiate a `MoveGroupCommander`_ object.  This object is an interface to a planning group (group of joints).
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    move_group.allow_replanning(True)# Set planning parameters

    print("Arm Moveit Commander has been started")
    self.move_group = move_group

    
  def create_path(self,scale=1): #This function is used to build a list of waypoints 
        
        move_group = self.move_group

        # --- Getting pose message
        wpose = Pose()
        waypoints=[]
        
        # --- Adding points to follow in path
        print(cblue+"\n\t === Acquire a trajectory === \n"+cend)
        print "Current directory: ", dir
        print "Files found: \n"
        print '\n'.join(files)
        readline.set_completer(completer)#active autocompletion on filenames
        readline.parse_and_bind("tab: complete")
        
        filename=raw_input(cyellow+"\nGetting path from file: "+cend)
        filepath="Trajectories/"+filename
        #-- Test to check file availability
        try:
            way=open(filepath,'r')
        except IOError:# in case of mispelling filename
            print(cred+"Looks like the file doesnt exist"+cend)
            return None,None

        os.system('clear')
        print(cgreen+"\n\t   >  - -     Niryo One - Surmoul 3D    - -  <   \n"+cend)
        print("\n-> file : "+'\033[4m'+filename+cend+" - Modified : "+time.ctime(os.path.getmtime(filepath)) )
        line=way.readline()

        nbr = 1
        #-- Counting lines and looking for errors
        while line:
         nbr += 1
         line=way.readline()
         if(len(line.split(' '))!=7 and line!=""):
          print(cred+"Error line "+str(nbr)+" does not contain 3 positions + 4 quaternions values :"+cend)
          print line
          return None,None
        way.seek(0)
        debug=False
        if(filename=='interactive.txt'):
          debug=True
          nbr=int(raw_input('Nbr of lines to read : '))

        traveling_distance=0 #to calculate travelling distance
        
        bar=Bar('Processing waypoints', max=nbr-1,width=10)
        i=0
        distance_btwn_points=[]
        prev_x=0
        prev_y=0
        prev_z=0
        while(i<=nbr-2):
         i+=1
         bar.next()
         tab=way.readline().split(' ')
         q1=[ float(tab[3]),float(tab[4]),float(tab[5]),float(tab[6])]

         wpose.orientation.w= q1[0] #- Quaternion
         wpose.orientation.x= q1[1]
         wpose.orientation.y= q1[2]
         wpose.orientation.z= q1[3]

         wpose.position.x= float(tab[0]) + offsets_niryo[0]  #- Position
         wpose.position.y= float(tab[1]) + offsets_niryo[1] 
         wpose.position.z= float(tab[2]) + offsets_niryo[2] 
         
         distance_btwn_points.append( sqrt(pow(wpose.position.x - prev_x,2) + pow(wpose.position.y - prev_y,2) + pow(wpose.position.z - prev_z,2) ) )
         traveling_distance += distance_btwn_points[-1]
         prev_x=wpose.position.x
         prev_y=wpose.position.y
         prev_z=wpose.position.z
         waypoints.append(copy.deepcopy(wpose))
         
        #-- Outing trajectory---------------
        if(not(debug)):
            wpose.position.x += -0.07
            wpose.position.z+=0.05
            wpose.orientation.x= 0 #- Quaternion 
            wpose.orientation.y= 0.259
            wpose.orientation.z= 0
            wpose.orientation.w= 0.966
            waypoints.append(copy.deepcopy(wpose))
            distance_btwn_points.append(0)
        
        way.close()
        bar.finish()
        distance_btwn_points[0]=0.005 #5mm to empty and fill nozzle

        traveling_distance=round( traveling_distance / 2.1874 , 2) #ratio filament diameter 1.75mm / nozzle 0.8mm
        a,b=divmod(traveling_distance,1000)
        print "Trajectory points found: %d" %(len(waypoints)-1)
        print "Necessary Filament: %2dm %3dmm"%(a,b*100)

        return waypoints, distance_btwn_points
        

  def plan_cartesian_path(self, max_tries, waypoints, start_joints): #This function is used to plan the path (calculating accelerations,velocities,positions etc..)

        move_group = self.move_group
        
        #-- Check for robot stability 
        if(n.get_learning_mode()):
         print "Make sure that Niryo is not in learning mode and in a safe position close to where you want to print"
         raw_input("Then press enter..")
        
        #-- Saving start state
        tab_joints=[start_joints[0], start_joints[1],start_joints[2],start_joints[3],start_joints[4],start_joints[5]]
        #-- Sending start state
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['joint_1', 'joint_2','joint_3','joint_4','joint_5','joint_6']
        joint_state.position = tab_joints
        initial_state = RbState()
        initial_state.joint_state = joint_state
        move_group.set_start_state(initial_state)

        #-- Parameters
        fraction=0.0
        tries=0
        max_tries=max_tries  #maximum tries allowed
        eef_step=1.0 #eef_step at 1.0 considering gcode is already an interpolation
        velocity=0.25 #velocity scaling factor applied to max velocity
        
        print "\n --- Computing parameters ---"
        print "| Max tries authorized : %2d \n| Eef step : %.4f \n| Velocity : %3d %%" %(max_tries,eef_step,velocity*100)
        print " ------------------------------\n"
        
        #-- Call cartesian service
        try:
            moveit_cartesian_path = rospy.ServiceProxy('compute_cartesian_path', GetCartesianPath) 
        except rospy.ServiceException,e:
            print("Service call failed:",e)
            return(None)
            
        #-- Computation
        best_frac=0.0
        t_in=time.time()

        while(fraction<1.0 and tries<max_tries):
         rospy.wait_for_service('compute_cartesian_path',2) #wait for service to be ready
         response = moveit_cartesian_path(Header(), initial_state, 'arm', 'tool_link',waypoints, eef_step, 0.0, True, Constraints())#send request
         tries+=1
         fraction=round(response.fraction,5)
         if(fraction<1.0): #in case solution is not complete we print iteration info in red (missing points)
          print(cred+"---try:"+str(tries)+"\t---completed:"+str(fraction*100)+"%"+cend)
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
        print "\n==>  tries: %d complete: %d %%  in: %.2f sec" %(tries, best_frac*100,c_time)#print process results
        if(best_frac<1.0):
            print "In most cases if the service doesnt compute 100% of the trajectory it is due to unreachable points or orientation"
            print "The problem is occuring at line (approx) : %d"%(round(fraction*len(waypoints)))
        
        #-- Scaling speeds for Niryo One
        if(velocity<1.0 and best_frac==1.0):
         print"==>  Retiming trajectory at %3d%% speed.."%(velocity*100)
         best_plan=move_group.retime_trajectory(initial_state, best_plan, velocity) #ref_state_in, plan, velocity sc
        
        expect_m, expect_s = divmod( best_plan.joint_trajectory.points[-1].time_from_start.secs , 60)
        expect_h, expect_m = divmod( expect_m  , 60)
        print "\nExpected printing time : %dh%02dm%02ds" %(expect_h,expect_m,expect_s)
        end_joints=list(best_plan.joint_trajectory.points[-1].positions)# returns last joint position in case of using multiple trajectories that are following each other
        
        return best_plan , end_joints , best_frac

  def execute_plan(self, plan): #This function execute the trajectory
    move_group = self.move_group
    t_in=time.time()
    print "Started at : ", time.asctime(time.localtime(t_in))
    move_group.execute(plan,wait=True)
    t_out=time.time()
    print "Finished at : ", time.asctime(time.localtime(t_out))
    m,s=divmod(t_out-t_in,60)
    h,m=divmod(m,60)
    print "Elapsed : %dh%02dm%02ds" %(h,m,s)
    
  def timing_extrusion(self, traj): #This function times the extrusion, aiming to be a real-time function
    tref=time.time()
    a=True;
    for i in range(0,len(traj.points)-1):
     t_next= traj.points[i].time_from_start.secs + traj.points[i].time_from_start.nsecs * pow(10,-9)
     t=tref + t_next
     dt= traj.points[i+1].time_from_start.secs + traj.points[i+1].time_from_start.nsecs * pow(10,-9) - t_next
     if(traj.points[i].time_from_start.secs - traj.points[i-1].time_from_start.secs > 1):
      time.sleep(traj.points[i].time_from_start.secs - traj.points[i-1].time_from_start.secs - 0.5) #sleep to reduce CPU usage when high duration between points
     while( time.time() < (t)): #void loop to wait for the right time
      pass
     n.digital_write(GPIO_1C,a) #send change signal on gpio interrupt
     a=not(a)
    n.digital_write(GPIO_1C,a) # last point (should be the 0 to stop extrusion)


     
    
   
#------------------------------------------------------------
#  --------   MAIN  --------------------
#------------------------------------------------------------
os.system('clear')
os.system('figlet  Niryo - Surmoul ')
Instance = PlannerInterface()
quit=False

while(not(quit)):
 os.system('clear')
 print(cgreen+"\n\t   >  - -     Niryo One - Surmoul 3D    - -  <   \n"+cend)
 print("\n\t1 - Trajectory execution\n\t2 - Tools\n\t0- Quit")
 choix=int(raw_input("\nChoice ? "))
 if(choix==0):
     quit=True
 if (choix==1) :
     way, distance_btwn_points = Instance.create_path()
     # -------------
     if(way==None): #Error while creating path
          raw_input()
     else:
       
       cartesian_plan, end_joints ,frac = Instance.plan_cartesian_path(3, way,n.get_joints())  
       check_trajectory(cartesian_plan, frac, way, distance_btwn_points)
       a=raw_input(cyellow+"Execute trajectory ? (yes/print/no/quit) "+cend)
     
       if(a=='print'):
          
          arduino=Arduino()
          arduino.extrusion_tab(cartesian_plan.joint_trajectory.points,distance_btwn_points)
          raw_input("Wait then press enter")
          job=Process( target=time_bar, args=(cartesian_plan.joint_trajectory.points[-1].time_from_start.secs,) ) #Multiprocessing to approx real-time control the extrusion and visual effect
          job2=Process( target=Instance.timing_extrusion, args=(cartesian_plan.joint_trajectory,) )
          time.sleep(1)
          job.start()
          job2.start()
          Instance.execute_plan(cartesian_plan)
     
          time.sleep(0.5)
          os.kill(int(job.pid), signal.SIGKILL) #kill process by the hard method (terminate and join doesnt work)
          os.kill(int(job2.pid), signal.SIGKILL) #kill process by the hard method (terminate and join doesnt work)
          
          arduino.stop_extrusion()
          arduino.stop_heating()
          quit=True
     
       if(a=='yes'):
        job=Process(target=time_bar, args=(cartesian_plan.joint_trajectory.points[-1].time_from_start.secs,)) #Multiprocessing to approx real-time control the extrusion and visual effect
        job.start()
        Instance.execute_plan(cartesian_plan)
        os.kill(int(job.pid), signal.SIGKILL) #kill process by the hard method (terminate and join doesnt work)
        quit=True
       if(a=='quit'):
        quit=True

 if(choix==2):
   arduino=Arduino()
   quit_tools=False
   while not(quit_tools):
       print('''\n\t1-chauffe
           t2-maj objectif temperature
           t3-affiche temperature
           t4-chargement materiau
           t5-déchargement materiau
           t6-remplir tableau d extrusion (indisponible)
           t7-extrusion vitesse donnee
           t8-stop extrusion
           t9-stop chauffe
           t10-empty buffers
           0--Quit
            ''')
       action=int(raw_input("Action ? "))
       if(action==0):
            quit_tools=True
       if(action==1):
            arduino.heat()
                     
       if(action==2):
            arduino.temperature_objective()
                     
       if(action==3):
            arduino.print_temperature()
                     
       if(action==4):
            arduino.filament_load()
                     
       if(action==5):
            arduino.filament_unload()
                     
       if(action==6):
            print("Not available")
                     
       if(action==7):
            arduino.manual_extrusion()
                     
       if(action==8):
            arduino.stop_extrusion()
                     
       if(action==9):
            arduino.stop_heating()
       if(action==10):
            arduino.empty()
              
print("\n"+cgreen+"\t\t --- Program end --- "+cend+"\n")
