#This script copy the images and labels from the selected folder to the selected destination
import shutil
import os
import random

IMAGENUMBER = 1258

#change this to change the number of images in subsets
TRAININGSET = 0.6
VALIDSET = 0.2
TESTSET = 0.2

blocks = ['X1-Y1-Z2','X1-Y2-Z1','X1-Y2-Z2','X1-Y2-Z2-CHAMFER','X1-Y2-Z2-TWINFILLET','X1-Y3-Z2', 'X1-Y3-Z2-FILLET','X1-Y4-Z1','X1-Y4-Z2','X2-Y2-Z2','X2-Y2-Z2-FILLET']
generalSourcePath = '/home/stefano/modelliMegaBlocks/megaBlockSet/sets/' #source path
generalDestinationPath = '/home/stefano/datasets/' #destination path

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

generalPath = '/home/stefano/datasets/'

#check if datasetBlocks exists
if not os.path.exists(generalPath + 'datasetBlocks'):
    os.mkdir(generalPath + 'datasetBlocks')
    os.mkdir(generalPath + 'datasetBlocks/train')
    os.mkdir(generalPath + 'datasetBlocks/train/images')
    os.mkdir(generalPath + 'datasetBlocks/train/labels')
    os.mkdir(generalPath + 'datasetBlocks/valid')
    os.mkdir(generalPath + 'datasetBlocks/valid/images')
    os.mkdir(generalPath + 'datasetBlocks/valid/labels')
    os.mkdir(generalPath + 'datasetBlocks/test')
    os.mkdir(generalPath + 'datasetBlocks/test/images')
    os.mkdir(generalPath + 'datasetBlocks/test/labels')
else:
    print('  Folder datasetBlocks already exists')

for block in blocks:
    print('  Working on ' + block)

    currentPath = generalPath + block

    #copy training images and labels
    print('     Copying training images and labels')
    images = os.listdir(currentPath + '/train/images')
    labels = os.listdir(currentPath + '/train/labels')
    for image in images:
        shutil.copy(currentPath + '/train/images/' + image, generalPath + 'datasetBlocks/train/images')
    for label in labels:
        shutil.copy(currentPath + '/train/labels/' + label, generalPath + 'datasetBlocks/train/labels')

    #copy validation images and labels
    print('     Copying validation images and labels')
    images = os.listdir(currentPath + '/valid/images')
    labels = os.listdir(currentPath + '/valid/labels')
    for image in images:
        shutil.copy(currentPath + '/valid/images/' + image, generalPath + 'datasetBlocks/valid/images')
    for label in labels:
        shutil.copy(currentPath + '/valid/labels/' + label, generalPath + 'datasetBlocks/valid/labels')

    #copy test images and labels
    print('     Copying test images and labels')
    images = os.listdir(currentPath + '/test/images')
    labels = os.listdir(currentPath + '/test/labels')
    for image in images:
        shutil.copy(currentPath + '/test/images/' + image, generalPath + 'datasetBlocks/test/images')
    for label in labels:
        shutil.copy(currentPath + '/test/labels/' + label, generalPath + 'datasetBlocks/test/labels')
    
    print('  Done with ' + block)
        
print('DONE MERGING DATASETS')
###############################################################################################
