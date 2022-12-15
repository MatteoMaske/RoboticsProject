#This script copy the images and labels from the selected folder to the selected destination
import shutil
import os
import random

IMAGENUMBER = 342

folderName = 'datasetV4'

#change this to change the number of images in subsets
TRAININGSET = 0.7
VALIDSET = 0.2
TESTSET = 0.1

blocks = ['X1-Y1-Z2','X1-Y2-Z1','X1-Y2-Z2','X1-Y2-Z2-CHAMFER','X1-Y2-Z2-TWINFILLET','X1-Y3-Z2', 'X1-Y3-Z2-FILLET','X1-Y4-Z1','X1-Y4-Z2','X2-Y2-Z2','X2-Y2-Z2-FILLET']
#source path
generalSourcePath = '/home/stefano/modelliMegaBlocks/megaBlockSet/sets/'
#change this to change the destination path
generalDestinationPath = '/home/stefano/datasets/'

#Calculate the number of images for each set
trainNumber = int(IMAGENUMBER * TRAININGSET)
validNumber = int(IMAGENUMBER * VALIDSET)
testNumber = IMAGENUMBER - trainNumber - validNumber

###############################################################################################
#ORGINIZE FILES
print('STARTING ORGANIZING FILES')

#Create the index list
index = []
for n in range(1,IMAGENUMBER+1):
    index.append(n)

for block in blocks:

    #check if block folder exists
    if not os.path.exists(generalDestinationPath + block):
        os.mkdir(generalDestinationPath + block)
        os.mkdir(generalDestinationPath + block + '/train')
        os.mkdir(generalDestinationPath + block + '/train/images')
        os.mkdir(generalDestinationPath + block + '/train/labels')
        os.mkdir(generalDestinationPath + block + '/valid')
        os.mkdir(generalDestinationPath + block + '/valid/images')
        os.mkdir(generalDestinationPath + block + '/valid/labels')
        os.mkdir(generalDestinationPath + block + '/test')
        os.mkdir(generalDestinationPath + block + '/test/images')
        os.mkdir(generalDestinationPath + block + '/test/labels')
        print('  Created folder for ' + block)
    else:
        print('  Folder ' + block + ' already exists')

    print('  Working on ' + block)

    #change the path for the current block
    imagesPath = generalSourcePath + block + '/'
    labelsPath = generalSourcePath + block + '/labels/'
    destinationPath = generalDestinationPath + block + '/'

    #shuffle the index list
    random.shuffle(index)
    print('    Shuffled index for ' + block)

    ## Training set
    for n in range(0,trainNumber):
        imageName = block + '-' + str(index[n]) + '.png'
        labelName = block + '-' + str(index[n]) + '.txt'

        shutil.copy(imagesPath + imageName, destinationPath + '/train/images')
        shutil.copy(labelsPath + labelName, destinationPath + '/train/labels')

    print('    Done Training set for ' + block)

    ## Validation set
    for n in range(trainNumber,trainNumber+validNumber):
        imageName = block + '-' + str(index[n]) + '.png'
        labelName = block + '-' + str(index[n]) + '.txt'
    
        shutil.copy(imagesPath + imageName, destinationPath + '/valid/images')
        shutil.copy(labelsPath + labelName, destinationPath + '/valid/labels')

    print('    Done Validation set for ' + block)

    ## Test set
    for n in range(trainNumber+validNumber,trainNumber+validNumber+testNumber):
        imageName = block + '-' + str(index[n]) + '.png'
        labelName = block + '-' + str(index[n]) + '.txt'

        shutil.copy(imagesPath + imageName, destinationPath + '/test/images')
        shutil.copy(labelsPath + labelName, destinationPath + '/test/labels') 

    print('    Done Test set for ' + block)

print('DONE ORGANIZING FILES')
###############################################################################################

#MERGE DATASETS
###############################################################################################
print('STARTING MERGING DATASETS')

#generalDestinationPath = '/home/stefano/datasets/'

#check if datasetBlocks exists
if not os.path.exists(generalDestinationPath + 'datasetBlocks'):
    os.mkdir(generalDestinationPath + 'datasetBlocks')
    os.mkdir(generalDestinationPath + 'datasetBlocks/train')
    os.mkdir(generalDestinationPath + 'datasetBlocks/train/images')
    os.mkdir(generalDestinationPath + 'datasetBlocks/train/labels')
    os.mkdir(generalDestinationPath + 'datasetBlocks/valid')
    os.mkdir(generalDestinationPath + 'datasetBlocks/valid/images')
    os.mkdir(generalDestinationPath + 'datasetBlocks/valid/labels')
    os.mkdir(generalDestinationPath + 'datasetBlocks/test')
    os.mkdir(generalDestinationPath + 'datasetBlocks/test/images')
    os.mkdir(generalDestinationPath + 'datasetBlocks/test/labels')
else:
    print('  Folder datasetBlocks already exists')

for block in blocks:
    print('  Working on ' + block)

    currentPath = generalDestinationPath + block

    #copy training images and labels
    print('     Copying training images and labels')
    images = os.listdir(currentPath + '/train/images')
    labels = os.listdir(currentPath + '/train/labels')
    for image in images:
        shutil.copy(currentPath + '/train/images/' + image, generalDestinationPath + 'datasetBlocks/train/images')
    for label in labels:
        shutil.copy(currentPath + '/train/labels/' + label, generalDestinationPath + 'datasetBlocks/train/labels')

    #copy validation images and labels
    print('     Copying validation images and labels')
    images = os.listdir(currentPath + '/valid/images')
    labels = os.listdir(currentPath + '/valid/labels')
    for image in images:
        shutil.copy(currentPath + '/valid/images/' + image, generalDestinationPath + 'datasetBlocks/valid/images')
    for label in labels:
        shutil.copy(currentPath + '/valid/labels/' + label, generalDestinationPath + 'datasetBlocks/valid/labels')

    #copy test images and labels
    print('     Copying test images and labels')
    images = os.listdir(currentPath + '/test/images')
    labels = os.listdir(currentPath + '/test/labels')
    for image in images:
        shutil.copy(currentPath + '/test/images/' + image, generalDestinationPath + 'datasetBlocks/test/images')
    for label in labels:
        shutil.copy(currentPath + '/test/labels/' + label, generalDestinationPath + 'datasetBlocks/test/labels')
    
    print('  Done with ' + block)
        
print('DONE MERGING DATASETS')
###############################################################################################
print('STARTING COPYING ROBOFLOW DATASET')

roboflowSource = '/home/stefano/datasets/MegaBlock Recognition.v6i.yolov5pytorch/' #source path
destination = '/home/stefano/datasets/' + folderName + '/' #destination path

#Copy dataset directory
print('  Copying dataset')
source = '/home/stefano/datasets/datasetBlocks/'
#remove directory if it exists
if os.path.exists(destination):
    shutil.rmtree(destination)

#copy dataset
shutil.copytree(source, destination)
print('  Done copying ' + source + ' to ' + destination)
###############################################################################################
print('  Merging roboflow dataset')
images = os.listdir(roboflowSource + 'train/images')
labels = os.listdir(roboflowSource + 'train/labels')

#copy train images and labels
print('    Copying training images and labels')
for image in images:
    shutil.copy(roboflowSource + 'train/images/' + image, destination + 'train/images/')
for label in labels:
    shutil.copy(roboflowSource + 'train/labels/' + label, destination + 'train/labels/')

#copy validation images and labels
print('    Copying validation images and labels')
images = os.listdir(roboflowSource + 'valid/images')
labels = os.listdir(roboflowSource + 'valid/labels')
for image in images:
    shutil.copy(roboflowSource + 'valid/images/' + image, destination + 'valid/images/')
for label in labels:
    shutil.copy(roboflowSource + 'valid/labels/' + label, destination + 'valid/labels/')

#copy test images and labels
print('    Copying test images and labels')
images = os.listdir(roboflowSource + 'test/images')
labels = os.listdir(roboflowSource + 'test/labels')
for image in images:
    shutil.copy(roboflowSource + 'test/images/' + image, destination + 'test/images/')
for label in labels:
    shutil.copy(roboflowSource + 'test/labels/' + label, destination + 'test/labels/')

print('DONE MERGING ROBOFLOW DATASET')
