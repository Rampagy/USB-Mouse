import vpython as vp # python -m pip install vpython
import serial # python -m pip install pyserial
import math


def ToQuaternion(pitch, roll, yaw):
  cr = math.cos(roll * 0.5)
  sr = math.sin(roll * 0.5)
  cp = math.cos(pitch * 0.5)
  sp = math.sin(pitch * 0.5)
  cy = math.cos(yaw * 0.5)
  sy = math.sin(yaw * 0.5)

  w = cr * cp * cy + sr * sp * sy
  x = sr * cp * cy - cr * sp * sy
  y = cr * sp * cy + sr * cp * sy
  z = cr * cp * sy - sr * sp * cy

  return w, x, y, z

ad=serial.Serial('com6',230400)


vp.scene.range=5
vp.scene.forward=vp.vector(-1,-1,-1)
vp.scene.width=600
vp.scene.height=600


x_arrow = vp.arrow(axis=vp.vector(1,0,0), length=2, shaftwidth=.1, color=vp.color.red)
y_arrow = vp.arrow(axis=vp.vector(0,1,0), length=2, shaftwidth=.1, color=vp.color.green)
z_arrow = vp.arrow(axis=vp.vector(0,0,1), length=2, shaftwidth=.1, color=vp.color.blue)

pcb=vp.box(length=6, width=4, height=.2, opacity=0.5, color=vp.vector(9/255, 77/255, 28/255))
frontArrow=vp.arrow(length=4,shaftwidth=.1,color=vp.color.purple,axis=vp.vector(1,0,0))
upArrow=vp.arrow(length=1,shaftwidth=.1,color=vp.color.magenta,axis=vp.vector(0,1,0))
sideArrow=vp.arrow(length=2,shaftwidth=.1,color=vp.color.orange,axis=vp.vector(0,0,1))

while (True):
  while (ad.inWaiting()==0):
    pass
  dataPacket = ad.readline()
  dataPacket=str(dataPacket,'utf-8')
  splitPacket = dataPacket.split(',')
  pitch = float(splitPacket[3])*3.14159/180
  roll = float(splitPacket[4])*3.14159/180
  yaw = float(splitPacket[5])*3.14159/180
  print(splitPacket[3], splitPacket[4], splitPacket[5])
  #q0, q1, q2, q3 = ToQuaternion(pitch, roll, yaw)
  vp.rate(50)

  k=vp.vector(math.cos(yaw)*math.cos(pitch), math.sin(pitch), math.sin(yaw)*math.cos(pitch))
  y=vp.vector(0,1,0)
  s=vp.cross(k,y)
  v=vp.cross(s,k)
  vrot=v*math.cos(roll)+vp.cross(k,v)*math.sin(roll)

  frontArrow.axis=k
  sideArrow.axis=vp.cross(k,vrot)
  upArrow.axis=vrot
  pcb.axis=k
  #pcb.up=vrot
  sideArrow.length=2
  frontArrow.length=4
  upArrow.length=1