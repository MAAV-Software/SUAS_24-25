#include <iostream>
#include <cstdlib>  // For system()

int main() {
    std::string image_path = "test_images/traffic2.jpg";
    std::string command = "python3 automated_yolo.py " + image_path;

    int result = system(command.c_str());

    if (result != 0) {
        std::cerr << "Error: Failed to run YOLO_script.py" << std::endl;
        return 1;
    }

    return 0;
}
