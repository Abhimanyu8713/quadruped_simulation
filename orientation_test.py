"""twolink controller."""

from controller import Robot
from controller import Motor
from controller import PositionSensor
from controller import InertialUnit
import math
#create a robot instace
arm=Robot()
target1=0
target2=0
#target2=0
#kp=50
#kd=5
count=0
r=0
timestep = int(arm.getBasicTimeStep())
dt = 0.001*timestep
#print(timestep)
logfile_name="twolink_pd.log"
#initialise every component
motor1=arm.getMotor("rm@flt")
motor2=arm.getMotor("rm@flk")
motor3=arm.getMotor("rm@blt")
motor4=arm.getMotor("rm@blk")
motor5=arm.getMotor("rm@frt")
motor6=arm.getMotor("rm@frk")
motor7=arm.getMotor("rm@brt")
motor8=arm.getMotor("rm@brk")

ps1=arm.getPositionSensor("ps@flt")
ps2=arm.getPositionSensor("ps@flk")
ps3=arm.getPositionSensor("ps@blt")
ps4=arm.getPositionSensor("ps@blk")
ps5=arm.getPositionSensor("ps@frt")
ps6=arm.getPositionSensor("ps@frk")
ps7=arm.getPositionSensor("ps@brt")
ps8=arm.getPositionSensor("ps@brk")
imu=arm.getInertialUnit("imu")

ps1.enable(timestep)
ps2.enable(timestep)
ps3.enable(timestep)
ps4.enable(timestep)
ps5.enable(timestep)
ps6.enable(timestep)
ps7.enable(timestep)
ps8.enable(timestep)
imu.enable(timestep)

val1=ps1.getValue()
val2=ps2.getValue()
l1=0.45
l2=0.22
y1=0.1
y =0.3
count=0
#error_prior=0;
while arm.step(timestep) != -1:
  val=ps1.getValue()
  rpy_d=[]
  target1=target1+0.3
  #x3=math.sqrt((0.2)**2-(y-0.1)**2)+0.5
  #x=0.6
  #y=-0.2
  def ik_solver(x,y):
   lh=math.sqrt(x**2+y**2)
   beta=math.acos((l1**2+l2**2-lh**2)/(2*l1*l2))
   alpha=math.atan(y/x)-math.asin((l2*math.sin(beta))/lh)
   beta1=math.pi-beta
   return alpha,beta1
  
  def range_solver(a,b):
   max_x=((0.449)-b**2+a**2)/(2*a)
   min_x=((0.0529)-b**2+a**2)/(2*a)
   max_X=a+b
   min_X=a-b
   #print(max(min_x,min_X),min(max_x,max_X))
   return max(min_x,min_X),min(max_x,max_X)
   
  def get_angle(x):
   a=0.5
   b=0.2
   min_x,max_x=range_solver(a,b)
   if x>min_x and x<max_x:
    y=-math.sqrt(b**2-(x-a)**2)
    alpha,beta1=ik_solver(x,y)
   else:
    x=(min_x+max_x)/2
    y=-math.sqrt(b**2-(x-a)**2)
    alpha,beta1=ik_solver(x,y)
   return alpha,beta1
   
  def set_rpy(theta):
   h=0.45
   la=(h/math.cos(theta)) + 0.7*math.tan(theta)
   lb=(h/math.cos(theta)) - 0.7*math.tan(theta)
   return la,lb
  
  la,lb=set_rpy((12*math.pi)/180)
  
  alpha,beta1=get_angle(la)
  alpha1,beta2=get_angle(lb)
  
  if count<100:
   motor1.setPosition(0)
   motor2.setPosition(0)
   motor3.setPosition(0)
   motor4.setPosition(0)
   motor5.setPosition(0)
   motor6.setPosition(0)
   motor7.setPosition(0)
   motor8.setPosition(0)
  else:
   motor1.setPosition(alpha)
   motor2.setPosition(beta1)
   motor3.setPosition(alpha)
   motor4.setPosition(beta1)
   motor5.setPosition(-alpha1)
   motor6.setPosition(-beta2)
   motor7.setPosition(-alpha1)
   motor8.setPosition(-beta2)
  
  rpy= imu.getRollPitchYaw()
  for i in rpy:
   rpy_d.append(i*180/(math.pi))
  print(rpy_d)
  count=count+1
  #print(alpha)
  #print(count)
  #print(x)
  pass