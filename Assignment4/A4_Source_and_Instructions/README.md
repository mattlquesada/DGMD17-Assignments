## Write a Basic Object Tracking Drone Application

Author: Raul Alvarez-Torrico (raul@tecbolivia.com)

This is the source code for my Circuit Cellar Magazine article "Write a Basic Object Tracking Drone Application - 
With OpenCV, MAVSDK and PX4 SITL Simulation".

### ABSTRACT
In this article I discuss how to develop a basic autonomous object tracking quadrotor application in simulation, by integrating computer vision object detection with the MAVSDK MAVLink library for autonomous flight control. PX4 Software in The Loop (SITL) simulation will be used to test the code example. I will talk about how to set up a development environment in the Ubuntu operating system, that includes all required tools: the PX4 SITL simulation environment, the MAVSDK-Python library and the Python OpenCV library. I will then explain a workflow to read and process images to detect objects by using a very basic computer vision technique: color range segmentation. Results obtained from the detection process will then be used to control a simulated quadcopter with the MAVSDK-Python MAVLink library to make it track autonomously the detected object. Color range segmentation is not the best, nor the most robust approach to detect and track objects with computer vision, but it serves well for the purpose of introducing concepts about interfacing computer vision detection tasks with drone autonomous flight.
The aim of this article is to present a basic example about how to integrate OpenCV, MASVDK and the PX4 simulation environment for students and hobbyists interested in experimenting with autonomous drone tracking. To follow this article, you must have at least very basic experience with OpenCV, as well as general knowledge on how quadcopters work, the MAVLink protocol and how to use PX4 SITL simulation to write drone autonomous flight applications with the MAVSDK-Python Library. See my previously published article “Quadrotor Autonomous Flight with PX4 and MAVSDK” [Reference 1] for a quick introduction to MAVSDK development with PX4 SITL simulation. 

"Pexels Videos 1572547_540p.mp4" video file belongs to The Lazy Artist Gallery from Pexels (https://www.pexels.com/video/mini-cooper-on-highway-along-the-desert-1572547/)
