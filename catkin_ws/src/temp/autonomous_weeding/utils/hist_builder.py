import numpy as np 
import cv2 
import matplotlib.pyplot as plt 
import os 

#list dir
#get img list
# for each img, get mean green
#build matplot histogram

class HistogramBuilder():
	"""
	Class for building mean green histogram of dataset	
	"""
	def __init__(self, directory):
		self.directory = directory
		self.file_li = self.get_files_from_dir()
		self.mean_green_li = []
		self.num_bins = 10
		self.plt_style = 'seaborn'

	def get_files_from_dir(self):
		"""
		Gets list of files from the object's directory
		|returns file list
		"""
		files = os.listdir(self.directory)
		print("Found %d files in directory %s . First file found: %s" % (len(files), self.directory, files[0]))
		return files
	
	def img_from_file(self,file_path):
		"""
		Opens file and creates data structure for the image
		| returns array representation of img, or None object on error
		"""
		try:
			img = cv2.imread(file_path, 1) 
			return img
		except OSError as e:
			print(e)
			return None

	def get_mean_green(self,img):
		"""
		Calculates mean green value of passed img
		|returns float of mean green
		"""
		return np.mean(img[:,:,1])

	def build_hist(self):
		for fname in self.file_li:
			fpath = self.directory + "/" + fname
			img = self.img_from_file(fpath)
			self.mean_green_li.append(self.get_mean_green(img))
		plt.style.use(self.plt_style)
		fig,ax = plt.subplots()
		ax.set_title("Mean Green Distribution of Entire Dataset")
		ax.set_xlabel("Green Channel")
		ax.set_ylabel("Frequency")

		ax.hist(self.mean_green_li, bins=self.num_bins)
		plt.show()


if __name__=='__main__':
	dir = '../../darknet_ros/darknet/data/images/'
	hist = HistogramBuilder(dir)
	hist.build_hist()
