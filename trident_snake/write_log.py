import numpy as np
import datetime

class WriteLog(object):
    """docstring for WriteData."""
    def __init__(self, filename, header):
        super(WriteLog, self).__init__()
        now = datetime.datetime.now()
        self.filename = filename
        title = np.array(header)

        with open(self.filename, 'a') as f_handle:
            np.savetxt(f_handle, title.reshape(1,-1), delimiter=',', fmt='%s')

    def __call__(self, data):
        with open(self.filename, 'a') as f_handle:
            np.savetxt(f_handle, data.reshape(1,-1), delimiter=',')