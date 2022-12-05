############################
# script to rename all files starting by a number with the block name
############################
import os
import shutil

dir = '/Users/amirgheser/Robotics/finalDataset/'
opts = ['train/', 'valid/', 'test/']
aa = ['images/', 'labels/']
for opt in opts:
    for a in aa:
        dir1 = dir + opt + a
        for file in os.listdir(dir1):
            if file.startswith('X1-Y2-Z1-X1-Y2-Z1'):
                os.rename(dir1 + file, dir1 + 'X1-Y2-Z1' + file[17:])
