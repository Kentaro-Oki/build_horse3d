import pybullet as p
import pybullet_data
import time
import numpy as np

# control value
maxForce = 100
mode = p.POSITION_CONTROL
AMP = 1.0 #rad
MODE = 'translation'
INVERSE = True

# set translation mode
if MODE == 'translation': 
    if not INVERSE:
        PERIOD = np.array([2.0, 2.0, 2.0]) # sec
        PHASE = np.array([np.pi/4, 0, 0])
    else:
        PERIOD = np.array([2.0, 2.0, 2.0]) # sec
        PHASE = np.array([np.pi/4, 0, 0])
else:
    if not INVERSE:
        PERIOD = np.array([2.0, 2.0, 2.0]) # sec
        PHASE = np.array([0, 2*np.pi/3, -2*np.pi/3])
    else:
        PERIOD = np.array([2.0, 2.0, 2.0]) # sec
        PHASE = np.array([0, -2*np.pi/3, 2*np.pi/3])
timestep = 0

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
while True:
    target_pos = AMP*np.sin(2*np.pi*1*timestep/(240.*PERIOD)+ PHASE)
    target_vel = AMP*np.cos(2*np.pi*1*timestep/(240.*PERIOD)+ PHASE)
    p.setJointMotorControl2(bodyIndex=snakeId, jointIndex=6, targetPosition=target_pos[0], targetVelocity=target_vel[0], controlMode=p.POSITION_CONTROL, force=maxForce)
    p.setJointMotorControl2(bodyIndex=snakeId, jointIndex=1, targetPosition=target_pos[1], targetVelocity=target_vel[1], controlMode=p.POSITION_CONTROL, force=maxForce)
    p.setJointMotorControl2(bodyIndex=snakeId, jointIndex=4, targetPosition=target_pos[2], targetVelocity=target_vel[2], controlMode=p.POSITION_CONTROL, force=maxForce)
    p.setJointMotorControl2(bodyIndex=snakeId, jointIndex=2, controlMode=p.TORQUE_CONTROL, force=0)
    p.setJointMotorControl2(bodyIndex=snakeId, jointIndex=5, controlMode=p.TORQUE_CONTROL, force=0)
    p.setJointMotorControl2(bodyIndex=snakeId, jointIndex=7, controlMode=p.TORQUE_CONTROL, force=0)
    p.stepSimulation()
    snakePos, snakeOrn = p.getBasePositionAndOrientation(snakeId)
    # print('state:', get_robot_data_array(snakeId).shape)
    time.sleep(1/240.)
    timestep += 1

p.disconnect()