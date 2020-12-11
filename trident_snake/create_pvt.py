'''
Create PVT data as a csv file from cyclic motion
'''

import numpy as np
import write_log

# configs
DURATION = 20 #[sec]
FREQ = 240 #[Hz]
AMP = 1 #[rad]
PERIOD = np.array([2,2,2]) #[sec]
PHASE = np.array([2*np.pi/3, 0, -2*np.pi/3])

# create log file
filename = './pvt_data/rotation_inverse.csv'
header = ['time', 'pos_0', 'pos_1', 'pos_2', 'vel_0', 'vel_1', 'vel_2']
write_log = write_log.WriteLog(filename, header)

time = 0
for i in range(DURATION*FREQ):
    target_pos = AMP*np.sin(2*np.pi*time/PERIOD+ PHASE)
    target_vel = AMP*np.cos(2*np.pi*time/PERIOD+ PHASE)
    log_data = np.concatenate((np.array([time]), target_pos, target_vel))
    write_log(log_data)
    time += 1/FREQ