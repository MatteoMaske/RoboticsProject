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
            a = False
            for i in range(10):
                if (file.startswith(str(i))):
                    a = True
            if a:
                os.rename(dir1 + file, dir1 + 'X1-Y1-Z2-' + file)
                