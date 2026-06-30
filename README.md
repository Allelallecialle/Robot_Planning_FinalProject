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
Run the combinatorial (visibility-graph) planner:
~~~
roslaunch rescue_planner rescue_map.launch planner_type:=visibility
~~~
or directly as a node (after the simulation/map are up):
~~~
rosrun rescue_planner map_receiver _planner_type:=visibility
~~~
Run the sample-based planners with roslaunch file. Default version with rrt:
~~~
roslaunch rescue_planner rescue_map.launch
~~~
or input the planner parameter (prm, rrt, rrt_star):
~~~
roslaunch rescue_planner rescue_map.launch planner_type:=prm
~~~

### Combinatorial planner overview
`planner_type:=visibility` builds an exact visibility graph over the
clearance-inflated obstacle corners plus the robot start, victim centres and the
gate; runs Dijkstra to get pairwise distances; solves an Orienteering Problem
(Held-Karp DP for small instances, greedy + 2-opt fallback) to choose which
victims to rescue within the `/victims_timeout` budget; then stitches Dubins
manoeuvres (with per-victim heading optimisation and curved-arc clearance
re-checking) and publishes the trajectory as `loco_planning/Reference` on
`/<robot>/ref`. Roadmap time, planning time and the selected total value are
logged and published on `/visibility_planner/stats`.


