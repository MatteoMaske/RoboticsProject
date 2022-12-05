#This script changes the name of the selected object

import os

IMAGENUMBER = 13068

generalPath = '/home/stefano/modelliMegaBlocks/megaBlockSet/sets/'

blocksImagesPaths = [generalPath + 'X1-Y1-Z2/', generalPath + 'X1-Y2-Z1/', generalPath + 'X1-Y2-Z2/', generalPath + 'X1-Y2-Z2-CHAMFER/']

#Choose the megaBlockSet
block = blocksImagesPaths[3]

for n in range(1,IMAGENUMBER+1):
    imageName = str(n) + '.png'
    labelName = str(n) + '.txt'

    os.rename(block + imageName, block + 'X1-Y1-Z2-' + imageName)
    os.rename(block + 'labels/' + labelName, block + 'labels/' + 'X1-Y1-Z2-' + labelName)

print('Done')