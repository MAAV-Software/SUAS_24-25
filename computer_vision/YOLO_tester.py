from ultralytics import YOLOWorld
from PIL import Image
import cv2
import numpy as np

# Function to apply image filtering (blurring) to image
def filter(img, Gaussian, kernel):
    if Gaussian == True: # If Gaussian blurring is requested
        image = cv2.GaussianBlur(img, (kernel.shape[0], kernel.shape[1]), 0) # Run Gaussian blurring with given kernel size
        return image
    else: # Used if we want to blur the image with a pre-defined kernel, not Gaussian
        image = cv2.filter2D(img, -1, kernel)
        return image
    
# Function to loop through and print out the classes, confidence and bounding box coordinates
def print_output(bounding_boxes):
    for box in bounding_boxes:
        # Get the coordinates for each box
        x_min, y_min, x_max, y_max = box.xyxy[0].tolist()

        conf = box.conf[0].item()  # Confidence score
        cls = int(box.cls[0].item())  # Class index
        class_name = model.names[cls] # Class name

        # Print out the class name, confidence score as well as the bouding box coordinates
        print(" ")
        print("Class:", class_name)
        print("Confidence:", conf)
        print("x min: ", x_min, " ", "y_min: ", y_min, " ", "x_max: ", x_max, " ", "y_max: ", y_max, " ")

# Load the YOLO world model
model_path = "/Users/dervint/Desktop/Clubs/MAAV/SUAS_24-25/computer_vision/yolov8s-world.pt"
model = YOLOWorld(model_path)

image_path = input("Enter Image path here: ") # Get the image that we want from the user
Blur = input("Do you want to blur (Yes/No): ") # Ask the user if they want to apply blurring
if Blur == "Yes":
    Gaussian = input("Do you want to use Gaussian (Yes/No): ") # Ask the user if they want to use Gaussian Blurring
    if Gaussian == "Yes":
        kernel_shape = int(input("Enter Guassian Kernel Shape Here (Enter 0 if you want no blurring): ")) # Get the kernel shape that will be used for Gaussian Kernel
    else:
        kernel_size = int(input("Enter size of wanted kernel (just use a square one for now): ")) # Get a (N, N) shape for a image filter kernel
        matrix = np.zeros((kernel_size, kernel_size))
        for i in range(kernel_size):
            for j in range(kernel_size):
                matrix[i, j] = input(f"Value at ({i}, {j}) is: ") # Get each of the values that will be used at each step point in the filter

# Read in the image and open it
image = cv2.imread(image_path)
original_image = Image.open(image_path)

# Apply the image filtering that was specificed in the command line
if Blur == "Yes" or Blur == "yes" or Blur == "y" or Blur == "Y":
    if Gaussian == "Yes" or Gaussian == "yes" or Gaussian == "y" or Gaussian == "Y":
        if not kernel_shape == 0:
            gaussian_kernel = np.zeros((kernel_shape, kernel_shape))
            image = filter(image, True, gaussian_kernel)
    else:
        image = filter(image, False, matrix)



# Predict using the YOLO model and show the result
results = model.predict(image,conf=0.10)
results[0].show()

# Get the bounding boxes
bounding_boxes = results[0].boxes  # Bounding boxes for the YOLO Predictions Image
print_output(bounding_boxes) # Loop through the bounding boxes and print out their coordinates
