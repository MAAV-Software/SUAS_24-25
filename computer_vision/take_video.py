import cv2
import time

# Open the default camera
cam = cv2.VideoCapture(0)

# Get the default frame width and height
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output_3_23.mp4', fourcc, 20.0, (frame_width, frame_height))

counter = 0

while True:
    ret, frame = cam.read()

    # Write the frame to the output file
    out.write(frame)

    # Display the captured frame
    cv2.imshow('Camera', frame)

    time.sleep(0.1)

    counter += 1
    if counter > 100:
        break



# Release the capture and writer objects
cam.release()
out.release()
cv2.destroyAllWindows()




'''
How to run in c++:
void take_test_capture() {
FILE* pipe = popen("python3 script.py", "r");
    if (!pipe) {
        std::cerr << "Error running Python script!" << std::endl;
        return 1;
    }
    
    char buffer[128];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        std::cout << buffer;  // Print output from Python script
    }

    pclose(pipe);
}
'''