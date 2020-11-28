import pybullet as p
import pybullet_data
import time
import numpy as np

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
horseStartPos = [0,0,0.5]
horseStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
horseId = p.loadURDF("./urdf/trident_snake.urdf", horseStartPos, horseStartOrientation)

# enable joint F/T sensor
num_joints = p.getNumJoints(horseId)
for i in range(num_joints):
    p.enableJointForceTorqueSensor(horseId, i)

# control value
maxForce = 100
mode = p.POSITION_CONTROL
AMP = 1.0 #rad
PERIOD = 1.0 # sec
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
    # target_pos = AMP*np.sin(2*np.pi*1*timestep/(240.*PERIOD)+np.array([0,np.pi/3,2*np.pi/3,3*np.pi/3,4*np.pi/3,5*np.pi/3]))
    # target_vel = AMP*np.cos(2*np.pi*1*timestep/(240.*PERIOD)+np.array([0,np.pi/3,2*np.pi/3,3*np.pi/3,4*np.pi/3,5*np.pi/3]))
    # target_vel = 0.1
    # p.setJointMotorControl2(bodyIndex=horseId, jointIndex=0, targetPosition=target_pos[0], targetVelocity=target_vel[0], controlMode=mode, force=maxForce)
    p.stepSimulation()
    horsePos, horseOrn = p.getBasePositionAndOrientation(horseId)
    # print('state:', get_robot_data_array(horseId).shape)
    time.sleep(1/240.)
    timestep += 1

p.disconnect()