import numpy as np 
import matplotlib
import matplotlib.pyplot as plt 

matplotlib.rcParams.update({'font.size': 18})


y_val = [2100,70,7,4.5,3.5,3,3,2.8,2.7,2.4,2.2,2.2,2.4]
x_val = [x*500 for x in range(13)]
print(x_val)
plt.title("YOLOv3 Later-stage Overfitting")
plt.xlabel("Epoch")
plt.ylabel("Test Set Error")
plt.plot(x_val[2:], y_val[2:])
plt.show()