import shutil
import random

IMAGENUMBER = 13068

TRAININGSET = 0.6
VALIDSET = 0.2
TESTSET = 0.2

generalPath = '/home/stefano/modelliMegaBlocks/megaBlockSet/sets/'

blocksImagesPaths = [generalPath + 'X1-Y1-Z2/', generalPath + 'X1-Y2-Z1/', generalPath + 'X1-Y2-Z2/', generalPath + 'X1-Y2-Z2-CHAMFER/']
blockslabelsPath = [generalPath + 'X1-Y1-Z2/labels/', generalPath + 'X1-Y2-Z1/labels/', generalPath + 'X1-Y2-Z2/labels/', generalPath + 'X1-Y2-Z2-CHAMFER/labels/']

#Choose the megaBlockSet
imagesPath = blocksImagesPaths[1]
labelsPath = blockslabelsPath[1]

blocks = ['X1-Y1-Z2','X1-Y2-Z1','X1-Y2-Z2','X1-Y2-Z2-CHAMFER','X1-Y2-Z2-TWINFILLET','X1-Y3-Z2', 'X1-Y3-Z2-FILLET','X1-Y4-Z1','X1-Y4-Z2','X2-Y2-Z2','X2-Y2-Z2-FILLET']
destinationPath = '/home/stefano/datasets/' + blocks[1]

index = []

#Create the index ancd shuffle it
for n in range(1,13069):
    index.append(n)

random.shuffle(index)

#Calculate the number of images for each set
trainNumber = int(IMAGENUMBER * TRAININGSET)
validNumber = int(IMAGENUMBER * VALIDSET)
testNumber = int(IMAGENUMBER * TESTSET)

## Training set
for n in range(1,trainNumber+1):
    imageName = str(index[n]) + '.png'
    labelName = str(index[n]) + '.txt'
    # print(imageName)
    # print(labelName)
    shutil.copy(imagesPath + imageName, destinationPath + '/train/images')
    shutil.copy(labelsPath + labelName, destinationPath + '/train/labels')

print('Done Training set')

## Validation set
for n in range(trainNumber+1,trainNumber+validNumber+1):
    imageName = str(index[n]) + '.png'
    labelName = str(index[n]) + '.txt'
    # print(imageName)
    # print(labelName)
    shutil.copy(imagesPath + imageName, destinationPath + '/valid/images')
    shutil.copy(labelsPath + labelName, destinationPath + '/valid/labels')

print('Done Validation set')

## Test set
for n in range(trainNumber+validNumber+1,trainNumber+validNumber+testNumber+1):
    imageName = str(index[n]) + '.png'
    labelName = str(index[n]) + '.txt'
    # print(imageName)
    # print(labelName)
    shutil.copy(imagesPath + imageName, destinationPath + '/test/images')
    shutil.copy(labelsPath + labelName, destinationPath + '/test/labels')

print('Done Test set')