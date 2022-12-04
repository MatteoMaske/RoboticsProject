# this file get all the images form the data/obj folder 
# and creates test.txt, train.txt and valid.txt with contaiting list of all images

import os

generalPath = 'content/drive/MyDrive/darknet/dataSetBlocksSerio/'

##TEST##
files_list = []
path = 'test/images'
#list all files
for file in os.listdir(path):
    if file.endswith('.png'):
        files_list.append((generalPath + "test/images/"+file))
print(files_list) 
with open("test.txt","w") as new_file:
    for file in files_list:
        new_file.write(file)
        new_file.write("\n")
    new_file.close()
print("test.txt created")

##TRAIN##
files_list = []
path = 'train/images'
#list all files
for file in os.listdir(path):
    if file.endswith('.png'):
        files_list.append((generalPath + "train/images/"+file))
print(files_list) 
with open("train.txt","w") as new_file:
    for file in files_list:
        new_file.write(file)
        new_file.write("\n")
    new_file.close()
print("train.txt created")

##VALID##
files_list = []
path = 'valid/images'
#list all files
for file in os.listdir(path):
    if file.endswith('.png'):
        files_list.append((generalPath + "valid/images/"+file))
print(files_list) 
with open("valid.txt","w") as new_file:
    for file in files_list:
        new_file.write(file)
        new_file.write("\n")
    new_file.close()
print("valid.txt created")