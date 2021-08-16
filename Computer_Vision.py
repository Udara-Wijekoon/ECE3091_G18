#This File is responsible for computer programming vision part 
# 
# This file currently contains the basic code applied in the lectures using the Open CV approach, to put this into testing we need to assemble the camera.
# But here's the test code - Neerav

import numpy as np
import cv2 # opencv file
from matplotlib import pyplot as plt # math plot

im = cv2.imread('blocks.png') #load an image

print("A colour image is a 2D grid of numbers, with 3 channels for colour components. Image Shape: ", im.shape)
print("We can access image pixel values by array indexing. Im[10,10,:]: ", im[10,10,:])

# Our image is stored with channels ordered as blue, green, red 
plt.subplot(1,2,1)
plt.imshow(im) # our plotting expects channels red,green,blue
plt.title('BGR')
plt.subplot(1,2,2)
plt.imshow(im[:,:,[2,1,0]]) # Reversing the channel order shows the true image
plt.title('RGB')
plt.show()

# This code is trying to identify blue blocks in an image, we need to find a silver block, in order to that we need to use the camera and click the picture of the ball 
# identify its rgb values, and then use the same method to look for our ball bearings


# Lets manually crop out a blue block
plt.figure(figsize=(15,5))

plt.subplot(1,4,1)
plt.imshow(im[150:180,40:70,[2,1,0]])
plt.title('Cropped image')

# Lets plot a histogram of the colours in this patch

plt.subplot(1,4,2)
plt.hist(im[150:180,40:70,0].ravel())
plt.title('Blue')
plt.subplot(1,4,3)
plt.hist(im[150:180,40:70,1].ravel())
plt.title('Green')
plt.subplot(1,4,4)
plt.hist(im[150:180,40:70,2].ravel())
plt.title('Red')
plt.show()

# Looks like most of the colours are between 
# 150-250 in the blue channel
# 0 - 150 in the green channel
# 50 - 75 in the red channel


# Lets threshold the image so that only these blocks remain - we'll use the opencv inRange function
im_threshold = cv2.inRange(im, (150, 0, 50), (250, 150, 75))
plt.imshow(im_threshold)
plt.title('Yay! We\'ve found two blue blocks')
plt.show()


# We'll now use an opencv simple blob detection function to find the centroids of these blobs in this thresholded image.
# Note, the simple blob detector can also threshold based on colour internally. Read the opencv documentation to learn more. 

# There are many parameters to tune

params = cv2.SimpleBlobDetector_Params()
params.filterByArea = True
params.filterByInertia = False
params.filterByConvexity = False
params.filterByColor = False

params.minArea = 50
params.maxArea = 1000

detector = cv2.SimpleBlobDetector_create(params) # create a blob detector

# Detect keypoints
keypoints = detector.detect(im_threshold)

print("Keypoints is a list of 2D co-ordinates, eg: Keypoint 1: ",keypoints[0].pt)

# Lets use an opencv function to draw circles around the detected objects
im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

plt.imshow(im_with_keypoints[:,:,[2,1,0]],extent=[0,im.shape[1],im.shape[0],0])
plt.plot([keypoints[0].pt[0]],[keypoints[0].pt[1]],'r+')
plt.plot([keypoints[1].pt[0]],[keypoints[1].pt[1]],'r+')
plt.show()

