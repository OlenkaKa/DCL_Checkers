# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Find required packages
FIND_PACKAGE( OpenCV REQUIRED )
FIND_PACKAGE(roscpp)
FIND_PACKAGE(std_msgs)
FIND_PACKAGE(image_transport)
FIND_PACKAGE(rosconsole)
FIND_PACKAGE(cv_bridge)

# Create an executable file from sources:
ADD_LIBRARY(ImageSender SHARED ${files})

# Link external libraries
TARGET_LINK_LIBRARIES(ImageSender ${DisCODe_LIBRARIES} ${OpenCV_LIBS} ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES} ${image_transport_LIBRARIES} ${cv_bridge_LIBRARIES})

INSTALL_COMPONENT(ImageSender)
