#This script creates the dataset for YOLOv5

""" YOU ONLY NEED TO CHANGE THE PATH FOR SOURCE AND DESTINATION FOLDERS """

import shutil
import os
import random
import cv2

#change this to decide the percentage of new images
PERCENTOFNEWIMAGES = 0.33

#change this to decide the number of images in subsets
TRAININGSET = 0.7
VALIDSET = 0.2
TESTSET = 0.1

###############################################################################################
folderName = 'dataset'
#change path of roboflow dataset
roboflowFolder = '/home/stefano/datasets/MegaBlock Recognition.v7i.yolov5pytorch/'
#source path
generalSourcePath = '/home/stefano/modelliMegaBlocks/megaBlockSet/sets/'
#change this to change the destination path
generalDestinationPath = '/home/stefano/datasets/'
###############################################################################################

blocks = ['X1-Y1-Z2','X1-Y2-Z1','X1-Y2-Z2','X1-Y2-Z2-CHAMFER','X1-Y2-Z2-TWINFILLET','X1-Y3-Z2', 'X1-Y3-Z2-FILLET','X1-Y4-Z1','X1-Y4-Z2','X2-Y2-Z2','X2-Y2-Z2-FILLET']

def countImages():
    counter = 0
    for item in os.listdir(generalSourcePath + blocks[0] + '/'):
        if item.endswith('.png'):
            counter += 1

    return counter

#Current number of images for each block
IMAGENUMBER = countImages()
#Calculate number of images for each block for augmentation
numNewImages = int(IMAGENUMBER * PERCENTOFNEWIMAGES)

###############################################################################################
#Ask if you want to augment dataset
augment = False
while True:
    keyInput = input('Do you want to augment dataset? (y/n)')
    if keyInput == 'y':
        print('Dataset will be augmented')
        augment = True
        break
    elif keyInput == 'n':
        print('Dataset will not be augmented')
        break
    else:
        print('Invalid input')
###############################################################################################

#ORGINIZE FILES
print('STARTING ORGANIZING FILES')

for block in blocks:

    #check if block folder exists
    if os.path.exists(generalDestinationPath + block):
        shutil.rmtree(generalDestinationPath + block)

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

    print('  Working on ' + block)

    #change the path for the current block
    imagesPath = generalSourcePath + block + '/'
    labelsPath = generalSourcePath + block + '/labels/'
    destinationPath = generalDestinationPath + block + '/'

    #Modify images with OpenCV
    if augment:
        print('    Augmenting images for ' + block)
        count = 0
        #numNewImages = int(IMAGENUMBER * PERCENTOFNEWIMAGES)
        for n in range(0,numNewImages):
            
            count = count + 1

            randList = []
            tmp = random.randint(1,IMAGENUMBER)
            while(tmp in randList):
                tmp = random.randint(1,IMAGENUMBER)
            randList.append(tmp)

            imageName = block + '-' + str(tmp) + '.png'
            #read image
            img = cv2.imread(imagesPath + imageName)

            k = random.randint(0,3)
            if(k == 0):
                #convert to gray
                img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            elif(k == 1):
                #blur image
                blur = random.randint(4,6)
                img = cv2.blur(img, (blur,blur))
            elif(k == 2):
                #add noise
                img = cv2.add(img, random.randint(50,100))
            else:
                #draw rectangle
                for n in range(0,10):
                    x1 = random.randint(0,500)
                    y1 = random.randint(0,500)
                    img = cv2.rectangle(img, (x1,y1), (x1+20,y1+20), (0,0,0), -1)

            imageNum = IMAGENUMBER + count
            newImageName = block + '-' + str(imageNum) + '.png'
            newLabelName = block + '-' +  str(imageNum) + '.txt'

            #save new label
            with open(labelsPath + imageName[:-4] + '.txt', 'r') as f:
                line = f.read()
                with open(labelsPath + newLabelName, "w") as f1:
                    f1.write(line)

            #save new image
            cv2.imwrite(imagesPath + newImageName, img)

        print('    Done augmenting images for ' + block)

    ############################################################################################################
    if block == blocks[0]:
        #Count image number for each block
        counter = countImages()
        
        #Calculate the number of images for each set
        trainNumber = int(counter * TRAININGSET)
        validNumber = int(counter * VALIDSET)
        testNumber = counter - trainNumber - validNumber

        #Create the index list
        index = []
        for n in range(1,counter+1):
            index.append(n)
    ############################################################################################################

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

#check if datasetBlender exists
if os.path.exists(generalDestinationPath + 'datasetBlender'):
    shutil.rmtree(generalDestinationPath + 'datasetBlender')

os.mkdir(generalDestinationPath + 'datasetBlender')
os.mkdir(generalDestinationPath + 'datasetBlender/train')
os.mkdir(generalDestinationPath + 'datasetBlender/train/images')
os.mkdir(generalDestinationPath + 'datasetBlender/train/labels')
os.mkdir(generalDestinationPath + 'datasetBlender/valid')
os.mkdir(generalDestinationPath + 'datasetBlender/valid/images')
os.mkdir(generalDestinationPath + 'datasetBlender/valid/labels')
os.mkdir(generalDestinationPath + 'datasetBlender/test')
os.mkdir(generalDestinationPath + 'datasetBlender/test/images')
os.mkdir(generalDestinationPath + 'datasetBlender/test/labels')

for block in blocks:
    print('  Working on ' + block)

    currentPath = generalDestinationPath + block

    #copy training images and labels
    print('     Copying training images and labels')
    images = os.listdir(currentPath + '/train/images')
    labels = os.listdir(currentPath + '/train/labels')
    for image in images:
        shutil.copy(currentPath + '/train/images/' + image, generalDestinationPath + 'datasetBlender/train/images')
    for label in labels:
        shutil.copy(currentPath + '/train/labels/' + label, generalDestinationPath + 'datasetBlender/train/labels')

    #copy validation images and labels
    print('     Copying validation images and labels')
    images = os.listdir(currentPath + '/valid/images')
    labels = os.listdir(currentPath + '/valid/labels')
    for image in images:
        shutil.copy(currentPath + '/valid/images/' + image, generalDestinationPath + 'datasetBlender/valid/images')
    for label in labels:
        shutil.copy(currentPath + '/valid/labels/' + label, generalDestinationPath + 'datasetBlender/valid/labels')

    #copy test images and labels
    print('     Copying test images and labels')
    images = os.listdir(currentPath + '/test/images')
    labels = os.listdir(currentPath + '/test/labels')
    for image in images:
        shutil.copy(currentPath + '/test/images/' + image, generalDestinationPath + 'datasetBlender/test/images')
    for label in labels:
        shutil.copy(currentPath + '/test/labels/' + label, generalDestinationPath + 'datasetBlender/test/labels')
    
    print('  Done with ' + block)
        
print('DONE MERGING DATASETS')
###############################################################################################
print('STARTING COPYING ROBOFLOW DATASET')

roboflowSource = roboflowFolder #source path
destination = '/home/stefano/datasets/' + folderName + '/' #destination path

#Copy dataset directory
print('  Copying dataset')
source = '/home/stefano/datasets/datasetBlender/'

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

#Remove unnecessary directories
print('REMOVING UNNECESSARY DIRECTORIES')
shutil.rmtree(generalDestinationPath + 'datasetBlender')
for block in blocks:
    shutil.rmtree(generalDestinationPath + block)
print('DONE REMOVING UNNECESSARY DIRECTORIES')

#Zip dataset
print('ZIPPING DATASET')
shutil.make_archive(destination, 'zip', generalDestinationPath, folderName)
print('DONE ZIPPING DATASET')
