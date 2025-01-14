Steps before starting executing the project:

-Cloning the repo 
-Moving to the workspace (cd {workspace/../../})
-Building the project by using (colcon build) 
-Sourcing the project (source install/setup.bash)
-Downloading needed dependencies (
sudo apt install ros-humble-usb-cam
sudo apt install ros-humble-gazebo-ros
sudo apt install python3-opencv
)


Steps during the excuting the project :


-After ensuring the pre-steps were done properly you need to open Visiual Studio code and run the code , the camera node will immediatly be created.
-Subscribing to the camera node that was created earlier by using the following command (ros2 run camera_subscriber camera_node)
-Launching the robot visualisation environment by (
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg \
ros2 launch turtlebot3_gazebo empty_world.launch.py
)



