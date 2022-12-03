#####################################################
# This script renames all files replacing 'from_' to 'to_' in the string
# in all files in the selected directory and its subdirectories
#####################################################

import os

opts = ['train/', 'valid/', 'test/']
aa = ['images/', 'labels/']
def renameFiles(dir, from_, to_):
    for opt in opts:
        for a in aa:
            for file in os.listdir(dir + block + '/' + opt + a):
                if file.startswith(from_):
                    os.rename(dir + block + '/' + opt + a + file, dir + block + '/' + opt + a + to_ + file[len(from_):])
dir = '/Users/amirgheser/Robotics/dataset1/'
block = 'X1-Y2-Z1'
from_ = 'X1-Y1-Z2'
to_ = block
renameFiles(dir, from_, to_)
