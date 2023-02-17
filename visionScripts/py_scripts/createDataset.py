"""
This script creates a dataset for YOLO from a folder of images and labels created with blender
"""

import shutil
import os
import random
import cv2

#change this to decide the percentage of new images
PERCENTOFNEWIMAGES = 0.33

#change this to decide the number of images in subsets
TRAININGSET = 0.6
VALIDSET = 0.2
TESTSET = 0

folderName= 'dataset'
sourcePath = '/home/stefano/modelliMegaBlocks/megaBlockSet/sets/blocks/'
destinationPath = '/home/stefano/datasets/' + folderName + '/'
roboflowFolder = '/home/stefano/datasets/MegaBlock Recognition.v7i.yolov5pytorch/'

#count the number of images in the source folder
IMAGENUMBER = len([f for f in os.listdir(sourcePath) if f.endswith(".png")])

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

#if the destination folder exists, delete it
if os.path.exists(destinationPath):
    shutil.rmtree(destinationPath)

#Create the destination folder
os.mkdir(destinationPath)
os.mkdir(destinationPath + '/train')
os.mkdir(destinationPath + '/train/images')
os.mkdir(destinationPath + '/train/labels')
os.mkdir(destinationPath + '/valid')
os.mkdir(destinationPath + '/valid/images')
os.mkdir(destinationPath + '/valid/labels')
os.mkdir(destinationPath + '/test')
os.mkdir(destinationPath + '/test/images')
os.mkdir(destinationPath + '/test/labels')

#Modify images with OpenCV
print("Augmenting images")

imagesPath = sourcePath
labelsPath = sourcePath + '/labels/'
#calculate the number of new images to create
numNewImages = int(IMAGENUMBER * PERCENTOFNEWIMAGES)

if augment:
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
        cv2.imwrite(imagesPath + 'aug-' + str(count) + '.png', img)
        
        #Save new label
        with open(labelsPath + randomImage[:-4] + '.txt', 'r') as f:
            line = f.read()
            with open(labelsPath + 'aug-' + str(count) + '.txt', "w") as f1:
                f1.write(line)
print("Done augmenting images")

#Calculate the number of images for each set
numImages = len([f for f in os.listdir(sourcePath) if f.endswith(".png")])
trainNumber = int(numImages * TRAININGSET)
validNumber = int(numImages * VALIDSET)
testNumber = numImages - trainNumber - validNumber

print("Organizing images")
#Training set
print('  Training set')
for n in range(0,trainNumber):

    randomImage = random.choice(os.listdir(imagesPath))
    while not randomImage.endswith('.png'):
            randomImage = random.choice(os.listdir(imagesPath))

    imageName = randomImage
    labelName = randomImage[:-4] + '.txt'

    shutil.move(imagesPath + imageName, destinationPath + 'train/images')
    shutil.move(labelsPath + labelName, destinationPath + 'train/labels')
print('  Done Training set')

#Validation set
print('  Validation set')
for n in range(trainNumber,trainNumber+validNumber):
    
    randomImage = random.choice(os.listdir(imagesPath))
    while not randomImage.endswith('.png'):
            randomImage = random.choice(os.listdir(imagesPath))

    imageName = randomImage
    labelName = randomImage[:-4] + '.txt'

    shutil.move(imagesPath + imageName, destinationPath + 'valid/images')
    shutil.move(labelsPath + labelName, destinationPath + 'valid/labels')
print('  Done Validation set')

#Test set
print('  Test set')
for n in range(trainNumber+validNumber,trainNumber+validNumber+testNumber):

    randomImage = random.choice(os.listdir(imagesPath))
    while not randomImage.endswith('.png'):
            randomImage = random.choice(os.listdir(imagesPath))

    imageName = randomImage
    labelName = randomImage[:-4] + '.txt'

    shutil.move(imagesPath + imageName, destinationPath + 'test/images')
    shutil.move(labelsPath + labelName, destinationPath + 'test/labels')
print('  Done Test set')
print('Done organizing images')

print("Starting merging roboflow dataset")
#Copy train images and labels
destination = destinationPath + '/'

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

print('Done merging roboflow dataset')
print("DONE")
