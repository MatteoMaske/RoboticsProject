import shutil
import random
import os
IMAGENUMBER = 13068

TRAININGSET = 0.6
VALIDSET = 0.2
TESTSET = 0.2

index = 10
generalPath = '/Users/amirgheser/Robotics/dataset/'
blocks = ['X1-Y1-Z2','X1-Y2-Z1','X1-Y2-Z2','X1-Y2-Z2-CHAMFER','X1-Y2-Z2-TWINFILLET','X1-Y3-Z2', 'X1-Y3-Z2-FILLET','X1-Y4-Z1','X1-Y4-Z2','X2-Y2-Z2','X2-Y2-Z2-FILLET']
targetBlock = blocks[index]
destinationPath = '/Users/amirgheser/Robotics/dataset1/'+targetBlock+'/'
blocksImagesPaths = []
blockslabelsPath = []

# Creates all directories
dir = destinationPath
if not os.path.exists(dir):
    os.makedirs(dir)
opt = ['train', 'valid', 'test']
for i in opt:
    dir = destinationPath+i
    if not os.path.exists(dir):
        os.makedirs(dir)
        os.makedirs(dir+'/images')
        os.makedirs(dir+'/labels')
####################

for block in blocks:
    blocksImagesPaths.append(generalPath + block + '/')
    blockslabelsPath.append(generalPath + block + '/labels/')

#Choose the megaBlockSet
imagesPath = blocksImagesPaths[index]
labelsPath = blockslabelsPath[index]

index = []

#Create the index ancd shuffle it
for n in range(1,13069):
    index.append(n)

random.shuffle(index)

#Calculate the number of images for each set
trainNumber = int(IMAGENUMBER * TRAININGSET)
validNumber = int(IMAGENUMBER * VALIDSET)
testNumber = IMAGENUMBER - trainNumber - validNumber

## Training set
for n in range(1,trainNumber+1):
    imageName = str(index[n-1]) + '.png'
    labelName = str(index[n-1]) + '.txt'
    # print(imageName)
    # print(labelName)
    dir = destinationPath + 'train/'
    if not os.path.exists(dir):
        os.mkdir(dir)
    shutil.copy(imagesPath + imageName, destinationPath + '/train/images')
    shutil.copy(labelsPath + labelName, destinationPath + '/train/labels')

print('Done Training set')

## Validation set
for n in range(trainNumber+1,trainNumber+validNumber+1):
    imageName = str(index[n-1]) + '.png'
    labelName = str(index[n-1]) + '.txt'
    # print(imageName)
    # print(labelName)
    dir = destinationPath + 'valid/'
    if not os.path.exists(dir):
        os.mkdir(dir)
    shutil.copy(imagesPath + imageName, destinationPath + '/valid/images')
    shutil.copy(labelsPath + labelName, destinationPath + '/valid/labels')

print('Done Validation set')

## Test set
for n in range(trainNumber+validNumber+1,IMAGENUMBER+1):
    imageName = str(index[n-1]) + '.png'
    labelName = str(index[n-1]) + '.txt'
    # print(imageName)
    # print(labelName)
    dir = destinationPath + 'test/'
    if not os.path.exists(dir):
        os.mkdir(dir)
    shutil.copy(imagesPath + imageName, destinationPath + '/test/images')
    shutil.copy(labelsPath + labelName, destinationPath + '/test/labels')

print('Done Test set')