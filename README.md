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
Run the combinatorial planners:
~~~
rosrun
~~~
Run the sample-based planners with roslaunch file. Default version with rrt:
~~~
roslaunch rescue_planner rescue_map.launch
~~~
or input the planner parameter (prm, rrt, rrt_star):
~~~
roslaunch rescue_planner rescue_map.launch planner_type:=prm
~~~


