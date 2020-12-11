import pybullet as p
import pybullet_data
import time
import numpy as np
import pandas as pd 
import read_csv

# control value
maxForce = 100
mode = p.POSITION_CONTROL

# read pvt file
file_name = './pvt_data/translation.csv'
read_csv = read_csv.ReadCsv(file_name)
pvt = read_csv()

# set env.
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
snakeStartPos = [0,0,0.5]
snakeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
snakeId = p.loadURDF("./urdf/trident_snake.urdf", snakeStartPos, snakeStartOrientation)
p.changeDynamics(bodyUniqueId=snakeId, linkIndex=2, lateralFriction=1)
p.changeDynamics(bodyUniqueId=snakeId, linkIndex=5, lateralFriction=1)
p.changeDynamics(bodyUniqueId=snakeId, linkIndex=7, lateralFriction=1)
timestep = 0

# enable joint F/T sensor
num_joints = p.getNumJoints(snakeId)    
for i in range(num_joints):
    p.enableJointForceTorqueSensor(snakeId, i)

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

# simulation
for i in range(len(pvt)):
    p.setJointMotorControl2(bodyIndex=snakeId, jointIndex=6, targetPosition=pvt['pos_0'][i], targetVelocity=pvt['vel_0'][i], controlMode=mode, force=maxForce)
    p.setJointMotorControl2(bodyIndex=snakeId, jointIndex=1, targetPosition=pvt['pos_1'][i], targetVelocity=pvt['vel_1'][i], controlMode=mode, force=maxForce)
    p.setJointMotorControl2(bodyIndex=snakeId, jointIndex=4, targetPosition=pvt['pos_2'][i], targetVelocity=pvt['vel_2'][i], controlMode=mode, force=maxForce)
    p.setJointMotorControl2(bodyIndex=snakeId, jointIndex=2, controlMode=p.TORQUE_CONTROL, force=0)
    p.setJointMotorControl2(bodyIndex=snakeId, jointIndex=5, controlMode=p.TORQUE_CONTROL, force=0)
    p.setJointMotorControl2(bodyIndex=snakeId, jointIndex=7, controlMode=p.TORQUE_CONTROL, force=0)
    p.stepSimulation()
    time.sleep(1/240.)

p.disconnect()