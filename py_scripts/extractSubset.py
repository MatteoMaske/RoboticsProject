import os
import shutil
from natsort import natsorted


initial_dir = '/Users/amirgheser/Robotics/finalDataset/'
final_dir = '/Users/amirgheser/Robotics/subset/'

opts = ['train/', 'valid/', 'test/']

blocks = ['X1-Y1-Z2','X1-Y2-Z1','X1-Y2-Z2','X1-Y2-Z2-CHAMFER','X1-Y2-Z2-TWINFILLET','X1-Y3-Z2', 'X1-Y3-Z2-FILLET','X1-Y4-Z1','X1-Y4-Z2','X2-Y2-Z2','X2-Y2-Z2-FILLET']

if not os.path.exists(final_dir):
    os.makedirs(final_dir)
    for opt in opts:
        if not os.path.exists(final_dir+opt):
            os.makedirs(final_dir+opt)
            os.makedirs(final_dir+opt+'images/')
            os.makedirs(final_dir+opt+'labels/')

# for every block 
# from i in range 1 to 13068
# if i is modulo 13
# look for images "blockName-{i}" in every folder in initial_dir
# if found, copy it to final_dir and then copy the respective label file
for block in blocks:
    for i in range(1,13069):
        if i%13 == 0:
            for opt in opts:
                for file in os.listdir(initial_dir+opt+'images/'):
                    if file.startswith(block+'-'+str(i)):
                        shutil.copy(initial_dir+opt+'images/'+file, final_dir+opt+'images/')
                        shutil.copy(initial_dir+opt+'labels/'+file[:-4]+'.txt', final_dir+opt+'labels/')
