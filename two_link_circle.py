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
l1=0.5
l2=0.3
y1=0.1
y =0.3
count=0
#error_prior=0;}
while arm.step(timestep) != -1:
  val=ps1.getValue()
  
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
   
  x=0.1*math.cos(target1)+0.5
  y=0.1*math.sin(target1)-0.1
  alpha,beta1=ik_solver(x,y)
  
  x1=0.1*math.cos(target1+3.14)+0.5
  y1=0.1*math.sin(target1+3.14)-0.1
  alpha1,beta2=ik_solver(x1,y1)
  
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
   #alpha=0
   #beta1=0
   motor1.setPosition(alpha)
   motor2.setPosition(beta1)
   motor3.setPosition(alpha1)
   motor4.setPosition(beta2)
   motor5.setPosition(-alpha1)
   motor6.setPosition(-beta2)
   motor7.setPosition(-alpha)
   motor8.setPosition(-beta1)
  
  rpy= imu.getRollPitchYaw()
  print(rpy)
  count=count+1
  #print(alpha)
  print(count)
  #print(x)
  pass