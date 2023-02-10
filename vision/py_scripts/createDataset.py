#This script creates the dataset for YOLO

""" YOU ONLY NEED TO CHANGE THE PATH FOR SOURCE AND DESTINATION FOLDERS """

import shutil
import os
import random
import cv2

#Current number of images for set of blocks
IMAGENUMBER = 3294

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

#blocks = ['X1-Y1-Z2','X1-Y2-Z1','X1-Y2-Z2','X1-Y2-Z2-CHAMFER','X1-Y2-Z2-TWINFILLET','X1-Y3-Z2', 'X1-Y3-Z2-FILLET','X1-Y4-Z1','X1-Y4-Z2','X2-Y2-Z2','X2-Y2-Z2-FILLET']

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

#Loop on the three sets
for i in range(0,3):

    print('Starting set: ' + str(i))

    #Check if the set's folder exists
    if os.path.exists(generalDestinationPath + str(i)):
        shutil.rmtree(generalDestinationPath + str(i))

    os.mkdir(generalDestinationPath + str(i))
    os.mkdir(generalDestinationPath + str(i) + '/train')
    os.mkdir(generalDestinationPath + str(i) + '/train/images')
    os.mkdir(generalDestinationPath + str(i) + '/train/labels')
    os.mkdir(generalDestinationPath + str(i) + '/valid')
    os.mkdir(generalDestinationPath + str(i) + '/valid/images')
    os.mkdir(generalDestinationPath + str(i) + '/valid/labels')
    os.mkdir(generalDestinationPath + str(i) + '/test')
    os.mkdir(generalDestinationPath + str(i) + '/test/images')
    os.mkdir(generalDestinationPath + str(i) + '/test/labels')

    #change the path for the current block
    imagesPath = generalSourcePath + str(i) + '/'
    labelsPath = generalSourcePath + str(i) + '/labels/'
    destinationPath = generalDestinationPath + str(i) + '/'

    #Modify images with OpenCV
    if augment:
        print('  Augmenting images for: ' + str(i))
        count = 0
        #numNewImages = int(IMAGENUMBER * PERCENTOFNEWIMAGES)
        for n in range(0,numNewImages):
            count += 1
            #Select a random image
            randomImage = random.choice(os.listdir(imagesPath))
            while not randomImage.endswith('.png'):
                randomImage = random.choice(os.listdir(imagesPath))
            #Open the image
            img = cv2.imread(imagesPath + randomImage)
            #Select a random number between 1 and 4
            randomNum = random.randint(1,4)
            #Apply the transformation
            if randomNum == 1:
                #convert to gray
                img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            elif randomNum == 2:
                #blur image
                blur = random.randint(4,6)
                img = cv2.blur(img, (blur,blur))
            elif randomNum == 3:
                #add noise
                img = cv2.add(img, random.randint(50,100))
            elif randomNum == 4:
                #draw black pixels
                for n in range(0,10):
                    x1 = random.randint(0,500)
                    y1 = random.randint(0,500)
                    img = cv2.rectangle(img, (x1,y1), (x1+20,y1+20), (0,0,0), -1)
            
            #Save the image
            if i == 0:
                cv2.imwrite(imagesPath + 'aug-' + str(count) + '.png', img)
            if i == 1:
                cv2.imwrite(imagesPath + 'aug-' + str(count + IMAGENUMBER) + '.png', img)
            if i == 2:
                cv2.imwrite(imagesPath + 'aug-' + str(count + IMAGENUMBER*2) + '.png', img)

            #Save new label
            with open(labelsPath + randomImage[:-4] + '.txt', 'r') as f:
                line = f.read()
                if i == 0:
                    with open(labelsPath + 'aug-' + str(count) + '.txt', "w") as f1:
                        f1.write(line)
                if i == 1:
                    with open(labelsPath + 'aug-' + str(count + IMAGENUMBER) + '.txt', "w") as f1:
                        f1.write(line)
                if i == 2:
                    with open(labelsPath + 'aug-' + str(count + IMAGENUMBER*2) + '.txt', "w") as f1:
                        f1.write(line)

    print('  Done augmenting images for set: ' + str(i))

    #Calculate the number of images for each set
    numImages = len([f for f in os.listdir(generalSourcePath + str(i)) if f.endswith(".png")])
    trainNumber = int(numImages * TRAININGSET)
    validNumber = int(numImages * VALIDSET)
    testNumber = numImages - trainNumber - validNumber

    #Training set
    print('  Starting Training set for set: ' + str(i))
    for n in range(0,trainNumber):

        randomImage = random.choice(os.listdir(imagesPath))
        while not randomImage.endswith('.png'):
                randomImage = random.choice(os.listdir(imagesPath))

        imageName = randomImage
        labelName = randomImage[:-4] + '.txt'

        shutil.move(imagesPath + imageName, destinationPath + 'train/images')
        shutil.move(labelsPath + labelName, destinationPath + 'train/labels')
    print('    Done Training set for set' + str(i))

    #Validation set
    print('  Starting Validation set for set: ' + str(i))
    for n in range(trainNumber,trainNumber+validNumber):
        
        randomImage = random.choice(os.listdir(imagesPath))
        while not randomImage.endswith('.png'):
                randomImage = random.choice(os.listdir(imagesPath))

        imageName = randomImage
        labelName = randomImage[:-4] + '.txt'

        shutil.move(imagesPath + imageName, destinationPath + 'valid/images')
        shutil.move(labelsPath + labelName, destinationPath + 'valid/labels')
    print('    Done Validation set for set' + str(i))

    #Test set
    print('  Starting Test set for set: ' + str(i))
    for n in range(trainNumber+validNumber,trainNumber+validNumber+testNumber):

        randomImage = random.choice(os.listdir(imagesPath))
        while not randomImage.endswith('.png'):
                randomImage = random.choice(os.listdir(imagesPath))

        imageName = randomImage
        labelName = randomImage[:-4] + '.txt'

        shutil.move(imagesPath + imageName, destinationPath + 'test/images')
        shutil.move(labelsPath + labelName, destinationPath + 'test/labels')
    print('    Done Test set for set' + str(i))

    print('Done organizing files for set: ' + str(i))
print('DONE ORGANIZING FILES')

print('STARTING MERGING IMAGES')

#Check if destination folder exists
if os.path.exists(generalDestinationPath + folderName):
    shutil.rmtree(generalDestinationPath + folderName)

os.mkdir(generalDestinationPath + folderName)
os.mkdir(generalDestinationPath + folderName + '/train')
os.mkdir(generalDestinationPath + folderName + '/train/images')
os.mkdir(generalDestinationPath + folderName + '/train/labels')
os.mkdir(generalDestinationPath + folderName + '/valid')
os.mkdir(generalDestinationPath + folderName + '/valid/images')
os.mkdir(generalDestinationPath + folderName + '/valid/labels')
os.mkdir(generalDestinationPath + folderName + '/test')
os.mkdir(generalDestinationPath + folderName + '/test/images')
os.mkdir(generalDestinationPath + folderName + '/test/labels')

#generalSourcePath = '/home/stefano/modelliMegaBlocks/megaBlockSet/sets/'
#generalDestinationPath = '/home/stefano/datasets/'

for i in range(0,3):
    print('  Merging set: ' + str(i))

    #Training set
    print('    Merging Training set for set: ' + str(i))
    for f in os.listdir(generalDestinationPath + str(i) + '/train/images'):
        shutil.move(generalDestinationPath + str(i) + '/train/images/' + f, generalDestinationPath + folderName + '/train/images')
    for f in os.listdir(generalDestinationPath + str(i) + '/train/labels'):
        shutil.move(generalDestinationPath + str(i) + '/train/labels/' + f, generalDestinationPath + folderName + '/train/labels')
    print('      Done Merging Training set for set: ' + str(i))

    #Validation set
    print('    Merging Validation set for set: ' + str(i))
    for f in os.listdir(generalDestinationPath + str(i) + '/valid/images'):
        shutil.move(generalDestinationPath + str(i) + '/valid/images/' + f, generalDestinationPath + folderName + '/valid/images')
    for f in os.listdir(generalDestinationPath + str(i) + '/valid/labels'):
        shutil.move(generalDestinationPath + str(i) + '/valid/labels/' + f, generalDestinationPath + folderName + '/valid/labels')
    print('      Done Merging Validation set for set: ' + str(i))

    #Test set
    print('    Merging Test set for set: ' + str(i))
    for f in os.listdir(generalDestinationPath + str(i) + '/test/images'):
        shutil.move(generalDestinationPath + str(i) + '/test/images/' + f, generalDestinationPath + folderName + '/test/images')
    for f in os.listdir(generalDestinationPath + str(i) + '/test/labels'):
        shutil.move(generalDestinationPath + str(i) + '/test/labels/' + f, generalDestinationPath + folderName + '/test/labels')
    print('      Done Merging Test set for set: ' + str(i))

    print('    Done Merging set: ' + str(i))
print('DONE MERGING IMAGES')

print('STARTING COPYING ROBOFLOW DATASET')

destination = generalDestinationPath + folderName + '/'

#Copy train images and labels
for f in os.listdir(roboflowFolder + 'train/images'):
    shutil.copy(roboflowFolder + 'train/images/' + f, destination + 'train/images/')
    shutil.copy(roboflowFolder + 'train/labels/' + f[:-4] + '.txt', destination + 'train/labels/')

#Copy validation images and labels
for f in os.listdir(roboflowFolder + 'valid/images'):
    shutil.copy(roboflowFolder + 'valid/images/' + f, destination + 'valid/images/')
    shutil.copy(roboflowFolder + 'valid/labels/' + f[:-4] + '.txt', destination + 'valid/labels/')

#Copy test images and labels
for f in os.listdir(roboflowFolder + 'test/images'):
    shutil.copy(roboflowFolder + 'test/images/' + f, destination + 'test/images/')
    shutil.copy(roboflowFolder + 'test/labels/' + f[:-4] + '.txt', destination + 'test/labels/')

print('DONE COPYING ROBOFLOW DATASET')

print('REMOVING TEMPORARY FOLDERS')
for i in range(0,3):
    shutil.rmtree(generalDestinationPath + str(i))