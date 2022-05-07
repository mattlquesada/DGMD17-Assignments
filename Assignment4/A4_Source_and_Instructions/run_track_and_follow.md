## STEPS TO RUN THE CODE EXAMPLE 

Follow these steps to run the 'track_and_follow.py' example:

1. Make sure you have the autonomous flight development toolchain propperly installed, if not, install it first by following the 'install_run_simulation.md' guide file.

2. Install the OpenCV library by following the 'install_opencv.md' guide file.

3. Run the PX4 & Gazebo or PX4 & JMAVSim simulator, as well as QGroundControl ground station software. To do so, follow the step 9 (RUN YOUR FIRST MAVSDK-PYTHON EXAMPLES) oulined in the 'install_run_simulation.md' file.

4. Run the 'track_and_follow.py' example:
	i. Change your path to the source code directory, for instance:
		cd ~/Source_code
	ii. Run the code example:
		python3 track_and_follow.py

--------------------------------------------------------------------------------
## WARNING!

I don't recommend running the code example with a real quadcopter, unless you have enough experience flying real quadcopters and understand the code sufficiently well to make the necessary changes. This example is just a learning excercise, I didn't tested it with a real drone. I'm willing to do so, as soon as time permits.

--------------------------------------------------------------------------------
## GET THE COORDINATE SYSTEM CONVERSION FACTOR EMPIRICALLY

Because the 'track_and_follow.py' example is just a simulation example which main purpose is to introduce computer vision and MAVSDK autonomous flight, the COORD_SYS_CONV_FACTOR constant has been chosen by trial and error. With COORD_SYS_CONV_FACTOR = 0.1, we are equating one pixel in the image frame with 0.1 meters in the ground.

To calibrate the camera for a real drone, for instance, the following empirical procedure can be followed: 

With the drone flying at a predetermined altitude, you take a picture of an object with a known lenght, and then measure the lenght of the same object in the image frame in pixels units. Next, you divide the lenght in meters by the lenght in pixels to obtain the conversion factor. After the calibration, the drone must always fly at the same altitud when tracking the target. This is a very rough approximation and an over simplification that doesn't take into account the effects of the 3D to 2D perspective transformation.

--------------------------------------------------------------------------------
## GET THE COLOR RANGE FOR YOUR TARGET

The script 'threshold_inRange.py' from the OpenCV library's code repository is not only useful for learning, but it will also help us obtain the color range values for the object we want to track.

I included a copy of this script with the article's source code. To get the color range for your target, follow these steps:

1. Navigate to the directory containing the script. For instance:
	cd ~/Source_code

2. Check that your webcam is connected to your PC and run the script:
	python3 threshold_inRange.py

You will see two windows: one displaying the original image and a second window with Hue, Saturation and Value (Low/High) slider controls.

3. Put your target object in front of the camera and move the sliders until you obtain a well defined mask for it in the second window (below the slider controls). Try first to define your Hue range (the type of color you are aiming at, e. g. 'greenish', 'redish', etc.) Next, try to define your Saturation range (the variation in 'whiteness' you want for your color range). Finally, define your Value range (the 'darkness' variation).

4. Copy the obtained values to the corresponding list variables in the code, in this order:
	HSV_LOWER_BOUND = (Low H, Low S, Low V)
    HSV_UPPER_BOUND = (High H, High S, High V)

At first, it can take you a while to get your mask well defined; it's a trial and error process. The more you understand how HSV color space works, the faster you'll get your mask for a given color range.

Avoid using reflective objects as targets. Cloth materials with very low reflectivity work better.

--------------------------------------------------------------------------------
## CAMERA MOUNTING IN A REAL DRONE

To run the code example with a real drone, we should mount the camera with a stabilization gimbal. Without the gimbal, the drone's sudden changes in orientation would make the image too unsteady to have the detection working propperly.

**Camera position and orientation:**
There are two obvious options for mounting the camera in the drone:

a) Vertically facing towards the ground, along the drone's vertical axis (pitch angle 90 degrees down).

b) Facing towards the ground in an angle less than 90 degrees, say pitch angle around 45 degrees down.

Intuitively, option a) should render more accurate conversions from pixel to meter units when using the COORD_SYS_CONV_FACTOR conversion factor.
--------------------------------------------------------------------------------
