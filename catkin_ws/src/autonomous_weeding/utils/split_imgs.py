import numpy as np 
import os

imgs = "../../darknet_ros/darknet/data/images/"
#Get Images, split into classes
imlist = os.listdir(imgs)
first_row_li = [x for x in imlist if x[0]=="F"]
second_row_li = [x for x in imlist if x[0]=="S"]
third_row_li = [x for x in imlist if x[0]=="T"]

train_imgs = []
test_imgs = []
#Percentage of images to be used in train set (0-1)
percentage_train = 0.8

for i, each in enumerate([first_row_li,second_row_li,third_row_li]):
    #Shuffle images
    np.random.shuffle(each)
    perc = int(percentage_train*len(each))
    train_imgs.extend(each[:perc])
    test_imgs.extend(each[perc:])


#Save Train image list
with open("train.txt", 'w') as f:
    string = ""
    for line in train_imgs:
        string += line+"\n"
    f.write(string)

#Save test image list
with open("test.txt", 'w') as f:
    string = ""
    for line in test_imgs:
        string += line+"\n"
    f.write(string)

print(len(train_imgs))
print(len(test_imgs))