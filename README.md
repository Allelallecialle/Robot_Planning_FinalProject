# Target Rescue
## Environment Setup
After installing the Docker environment following 
[loco-nav](https://github.com/idra-lab/loco_nav/tree/master)
instructions, launch the container with:

~~~
lab_planning
~~~

Create the ROS environment:
~~~
cd ros_ws
catkin_make
source devel/setup.bash
~~~

## Running
### To run the simulation:
~~~
roslaunch
~~~

### To run individual planners

Open a new terminal in the container with:
~~~
dock-other
~~~
Run the combinatorial planner:
~~~
rosrun
~~~
Run the sample-based planner:
~~~
rosrun
~~~


