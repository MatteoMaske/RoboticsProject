blocks = ['X1-Y1-Z2','X1-Y2-Z1','X1-Y2-Z2','X1-Y2-Z2-CHAMFER','X1-Y2-Z2-TWINFILLET','X1-Y3-Z2', 'X1-Y3-Z2-FILLET',
'X1-Y4-Z1','X1-Y4-Z2','X2-Y2-Z2','X2-Y2-Z2-FILLET']

opts = ['train', 'valid', 'test']
dataset1path = '/Users/amirgheser/Robotics/dataset1'
finalDatasetPath = '/Users/amirgheser/Robotics/finalDataset'
# create dir finalDataset
# create subdirs train, valid, test
# for each directory in dataset1 (X1-Y1-Z2, X1-Y2-Z1, etc)
# move each file from block/train to finalDataset/train after renaming it from block-*.png to *.png
# do the same for valid and test

import os
import shutil

# create dir finalDataset
dir = '/Users/amirgheser/Robotics/finalDataset/'
if not os.path.exists(dir):
    print("Creating finalDataset directory")
    os.mkdir(dir)
    for i in opts:
        dir1 = dir + i
        if not os.path.exists(dir1):
            os.mkdir(dir1)
            if not os.path.exists(dir1 + '/images'):
                os.mkdir(dir1 + '/images')
            if not os.path.exists(dir1 + '/labels'):
                os.mkdir(dir1 + '/labels')

def moveFiles(source, destination, block_name):
    for file in os.listdir(source):
        # rename files
        if file.endswith('.png') or file.endswith('.txt') and not file.startswith('X'):
            os.rename(source + '/' + file, source + '/' + block_name + '-' + file)
    for file in os.listdir(source):
        #move file to destination 
        shutil.move(source + file, destination + file)


# for each directory in dataset1 (X1-Y1-Z2, X1-Y2-Z1, etc)
for block in blocks:
    # move each file from block/train to finalDataset/train after renaming it from block-*.png to *.png
    for opt in opts:
        moveFiles(f'{dataset1path}/{block}/{opt}/images/', f'{finalDatasetPath}/{opt}/images/', block)
        moveFiles(f'{dataset1path}/{block}/{opt}/labels/', f'{finalDatasetPath}/{opt}/labels/', block)
    