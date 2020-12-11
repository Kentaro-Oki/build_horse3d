import numpy as np
import datetime

class WriteLog(object):
    """docstring for WriteData."""
    def __init__(self, filename):
        super(WriteLog, self).__init__()
        now = datetime.datetime.now()
        self.filename = filename.split('.csv')[0] + now.strftime('_%Y%m%d_%H%M%S') + '.csv'
        title = np.array(['cmd_dir', 's_sensor', 'cf_e', 'cf_g', 'judge',\
                'x_e', 'y_e', 'z_e', 'rx_e', 'ry_e', 'rz_e', \
                'x_g', 'y_g', 'z_g', 'rx_g', 'ry_g', 'rz_g',  \
                'cov_xx','cov_yy','cov_zz','cov_rxrx','cov_ryry','cov_rzrz'])

        with open(self.filename, 'a') as f_handle:
            np.savetxt(f_handle, title.reshape(1,-1), delimiter=',', fmt='%s')

    def __call__(self, data):
        with open(self.filename, 'a') as f_handle:
            np.savetxt(f_handle, data.reshape(1,-1), delimiter=',')