import cv2
import os


def img_flip(path, direction="h"):
    src = cv2.imread(path)
    flipcode= 1 if direction=="h" else 0
    img = cv2.flip(src, flipcode)
    return img

def label_horizontal_flip(labels):
    new_labels = []
    #FOR HORIZONTAL FLIP
    with open(labels, 'r') as f:
        data = f.readlines()
        for line in data:
            line = line.strip('\n')
            li = line.split(" ")        
            #0th element is class, 1st is x_centre and 2 is y_centre
            new_x_centre = abs(width - (width*float(li[1])))/width
            new_line = li[0] + " " + str(new_x_centre) + " " + li[2] + " " + li[3] + " " + li[4] + "\n"
            new_labels.append(new_line)
    return new_labels

def label_vertical_flip(labels):
    new_labels=[]
    #FOR Vertical FLIP
    with open(labels, 'r') as f:
        data = f.readlines()
        for line in data:
            line = line.strip('\n')
            li = line.split(" ")        
            #0th element is class, 1st is x_centre and 2 is y_centre
            new_y_centre = abs(height - (height*float(li[2])))/height
            new_line = li[0] + " " + li[1] + " " + str(new_y_centre) + " " + li[3] + " " + li[4] + "\n"
            new_labels.append(new_line)
    return new_labels


#images are 1920x1080
width = 1920
height = 1080


#Path to file
img_path = "../../darknet_ros/darknet/data/images"
label_path = "../../darknet_ros/darknet/data/labels"
new_imgs = "../../darknet_ros/darknet/data/new_imgs"
new_labels = "../../darknet_ros/darknet/data/new_labels"

img_list = os.listdir(img_path)
label_list = os.listdir(label_path)


for sample in img_list:
    name = sample.split(".")
    img_loc = os.path.join(img_path, sample)
    hflip_img = img_flip(img_loc, direction="h")
    vflip_img = img_flip(img_loc, direction="v")

    
    new_hflip_img_name = name[0] + "_HFLIP." + name[1]

    cv2.imwrite(os.path.join(new_imgs, new_hflip_img_name), hflip_img)


    new_vflip_img_name = name[0] + "_VFLIP." + name[1]
    cv2.imwrite(os.path.join(new_imgs, new_vflip_img_name), vflip_img)

    label_name = name[0] + ".txt"
    label_loc = os.path.join(label_path, label_name)
    hlabel = "".join([str(x) for x in label_horizontal_flip(label_loc)])
    vlabel = "".join([str(x) for x in label_vertical_flip(label_loc)])

    hlabel_name = name[0] + "_HFLIP" + ".txt"
    with open(os.path.join(new_labels, hlabel_name), 'w') as f:
        f.write(hlabel)
    vlabel_name = name[0] + "_VFLIP" + ".txt"
    with open(os.path.join(new_labels, vlabel_name), 'w') as f:
        f.write(vlabel)







