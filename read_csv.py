import numpy as np
import pandas as pd

class ReadCsv(object):
    """ Class to convert csv filed data to numpy array."""
    def __init__(self, filename):
        super(ReadCsv, self).__init__()
        self.df = pd.read_csv(filename)
    
    def __call__(self):
        return self.df