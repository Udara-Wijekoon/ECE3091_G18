import torch
import numpy
import detecto
from detecto import core, utils, visualize
import matplotlib.pyplot as plt

#loading the model
model = core.Model.load('model_weights.pth', ['ball'])

#loading the test image
image = utils.read_image('test/13.png')

#using the model to top_predict the target ball
predictions = model.predict_top(image)

#printing out the predictions
labels, boxes, scores = predictions
print(predictions)
visualize.show_labeled_image(image, boxes, labels)
print("Done")
#