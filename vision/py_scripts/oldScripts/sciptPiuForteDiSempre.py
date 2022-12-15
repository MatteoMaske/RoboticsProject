#This script must fix all the FUCKING LABELS in the dataset
# For each label ("*.txt") in the dataset we must replace the first int found with
# THE CORRECT LABEL number


import os
#dictionary for the labels
blocks = ['X1-Y1-Z2','X1-Y2-Z1','X1-Y2-Z2','X1-Y2-Z2-CHAMFER','X1-Y2-Z2-TWINFILLET','X1-Y3-Z2', 'X1-Y3-Z2-FILLET','X1-Y4-Z1','X1-Y4-Z2','X2-Y2-Z2','X2-Y2-Z2-FILLET']

# Fa schifo così, però l'ha fatto copilot
labels = {blocks[0]: 0, blocks[1]: 1, blocks[2]: 2, blocks[3]: 3, blocks[4]: 4, blocks[5]: 5, blocks[6]: 6, blocks[7]: 7, blocks[8]: 8, blocks[9]: 9, blocks[10]: 10}
# Path to the dataset
path = '/Users/amirgheser/Robotics/datasetBlocks/'
opts = ['train/', 'valid/', 'test/']

# For every option (train, valid, test) in the folder "datasetBlocks"+opt+"/labels/"
# for every "*.txt" file in the folder, 
# open it and replace the first '0' found with the correct label
# To get the correct label we use the name of the file, which is the name of the image
# as key in the dictionary "labels" and we get the value.

def getFileName(file):
    x = file.split('-')
    return '-'.join(x[:-1])

for opt in opts:
    for file in os.listdir(path+opt+'labels/'):
        name = getFileName(file)
        # store the whole content into a buffer
        with open(path+opt+'labels/'+file, 'r') as f:
            buffer = f.read()
        # replace the first '0' found with the correct label
        buffer = buffer.replace('0', str(labels[name]), 1)
        # write the buffer into the file
        with open(path+opt+'labels/'+file, 'w') as f:
            f.write(buffer)

