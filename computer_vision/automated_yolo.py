from ultralytics import YOLOWorld
from PIL import Image
import cv2
import numpy as np
import sys


model = YOLOWorld()
# model.set_classes(["Fire hydrant",
#                    "Stop sign", 
#                    "Person", 
#                    "Car", 
#                    "Motorcycle",
#                    "Airplane",
#                    "Bus",
#                    "Boat",
#                    "Bench",
#                    "Snowboard",
#                    "Umbrella",
#                    "Sports ball",
#                    "Kite",
#                    "Baseball Bat",
#                    "Bed/mattress",
#                    "Handbag",
#                    "Microwave",
#                    "Clock",
#                    "Tennis racket",
#                    "Suitcase",
#                    "Skis"
#                    ])

image_path = sys.argv[1] 

image = cv2.imread(image_path)

bordered_image = cv2.copyMakeBorder(
    image, 
    top=25, 
    bottom=25, 
    left=25, 
    right=25, 
    borderType=cv2.BORDER_CONSTANT, 
    value=[0, 0, 0]  # Black color (BGR format)
)

# Save or display the new image
cv2.imwrite('image_with_border.jpg', bordered_image)

image = cv2.imread("image_with_border.jpg")

# Image Sharpening Filter Matrix
matrix = np.array([[-1/9, -1/9, -1/9],
                   [-1/9, 17/9, -1/9],
                   [-1/9, -1/9, -1/9]])

depth = -1


# Apply a combination of gaussian blur and applying image sharpening filter
gaussian_kernel_shape = 5
for i in range(5):
    image = cv2.GaussianBlur(image, (gaussian_kernel_shape, gaussian_kernel_shape), 0)
    image = cv2.filter2D(image, depth, matrix)

image = cv2.filter2D(image, depth, matrix)


# Predict using the YOLO model
results = model.predict(image,conf=0.10)
results[0].save()

# Get the bounding boxes
bounding_boxes = results[0].boxes  # Bounding boxes for the first image
object_list = []

# Loop through the bounding boxes and print out their coordinates
for box in bounding_boxes:
    x_min, y_min, x_max, y_max = box.xyxy[0].tolist()

    conf = box.conf[0].item()  # Confidence score
    cls = int(box.cls[0].item())  # Class index
    class_name = model.names[cls] # Class name

    # Print out the class name, confidence score as well as the bouding box coordinates
    print("Class:", class_name)
    print("Confidence:", conf)
    print("x min: ", x_min, " ", "y_min: ", y_min, " ", "x_max: ", x_max, " ", "y_max: ", y_max, " ")
    print(" ")
    