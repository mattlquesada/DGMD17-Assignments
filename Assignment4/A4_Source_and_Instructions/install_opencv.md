## INSTALL OPENCV ON UBUNTU 18.04

Resource: https://linuxize.com/post/how-to-install-opencv-on-ubuntu-18-04/

Refresh the packages index and install the OpenCV package by typing:

	sudo apt update
	sudo apt install python3-opencv

The command above will install all packages necessary to run OpenCV.

To verify the installation, import the cv2 module and print the OpenCV version:

	python3 -c "import cv2; print(cv2.__version__)"

Output:

	3.2.0

--------------------------------------------------------------------------------
## PYTHON AND OPENCV VERSIONS USED TO TEST THE 'track_and_follow.py' EXAMPLE

	Python 3.6.9
	OpenCV 3.2.0

It could also work with other versions, preferably newer.

Because the MAVSDK-Python library uses 'asyncio', you'll need at least Python 3.6 to run MAVSDK-Python

--------------------------------------------------------------------------------
## TEST YOUR OPENCV INSTALLATION

You can also run the script 'detect_test.py' included with the source code to verify your installation. This script runs a basic detection algorithm over a stream coming from a video file (included also with the source code).

1. Change to your source code directory, for instance:
	cd ~/Source_code

2. Run the test script:
	python3 detect_test.py

--------------------------------------------------------------------------------
## OTHER RESOURCES OF INTEREST

You can explore histogram equalization as a tool to atenuate the effects of brightness variation in the image stream, which in turn makes the color vary.

OpenCV Python equalizeHist colored image
https://stackoverflow.com/questions/31998428/opencv-python-equalizehist-colored-image

