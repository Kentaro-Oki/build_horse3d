import pybullet as p
import pybullet_data
import time
import numpy as np

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([-0.7,0.7,0])
boxId = p.loadURDF("./urdf/horse3d.urdf", cubeStartPos, cubeStartOrientation)

# control value
maxForce = 100
mode = p.POSITION_CONTROL
AMP = 0.25 #rad
PERIOD = 0.5 # sec
timestep = 0

while True:
    # target_pos = AMP*np.sin(2*np.pi*1*timestep/(240.*PERIOD)+np.array([0,np.pi/3,2*np.pi/3]))
    # p.setJointMotorControl2(bodyIndex=boxId, jointIndex=0, targetPosition=target_pos[0], controlMode=mode, force=maxForce)
    # p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=target_pos[1], controlMode=mode, force=maxForce)
    # p.setJointMotorControl2(bodyIndex=boxId, jointIndex=2, targetPosition=target_pos[2], controlMode=mode, force=maxForce)
    p.stepSimulation()
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    print(cubePos,cubeOrn)
    time.sleep(1/240.)
    timestep += 1

p.disconnect()