import pybullet as p
import pybullet_data
import time
import numpy as np

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
horseStartPos = [0,0,1]
horseStartOrientation = p.getQuaternionFromEuler([0, 0 ,0])
horseId = p.loadURDF("./urdf/horse3d.urdf", horseStartPos, horseStartOrientation)

# enable joint F/T sensor
num_joints = p.getNumJoints(horseId)
for i in range(num_joints):
    p.enableJointForceTorqueSensor(horseId, i)

# # control value
# maxForce = 100
# mode = p.POSITION_CONTROL
# AMP = 0.25 #rad
# PERIOD = 0.5 # sec
timestep = 0

def get_joint_data_array(body_id, joint_id):
    joint_ft = np.array(p.getJointState(body_id, joint_id)[2])
    link_pos_w = np.array(p.getLinkState(body_id, joint_id)[0])
    link_orn_w = np.array(p.getEulerFromQuaternion(list(p.getLinkState(body_id, joint_id)[1])))
    return np.concatenate([link_pos_w, link_orn_w, joint_ft])

def get_robot_data_array(body_id):
    robot_data = np.empty(0)
    for i in range(num_joints):
        robot_data = np.concatenate([robot_data, get_joint_data_array(body_id, i)])
    return robot_data

while True:
    # target_pos = AMP*np.sin(2*np.pi*1*timestep/(240.*PERIOD)+np.array([0,np.pi/3,2*np.pi/3]))
    # p.setJointMotorControl2(bodyIndex=boxId, jointIndex=0, targetPosition=target_pos[0], controlMode=mode, force=maxForce)
    # p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=target_pos[1], controlMode=mode, force=maxForce)
    # p.setJointMotorControl2(bodyIndex=boxId, jointIndex=2, targetPosition=target_pos[2], controlMode=mode, force=maxForce)
    p.stepSimulation()
    horsePos, horseOrn = p.getBasePositionAndOrientation(horseId)
    print('state:', get_robot_data_array(horseId).shape)
    time.sleep(1/240.)
    timestep += 1

p.disconnect()