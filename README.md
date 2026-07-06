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

### To run the simulation with the Benchmark
Choose the planner type as in the previous cases presented. 
The data about the simulation is saved in the `benchmark.csv` in the `test_benchmark` folder.
~~~
roslaunch rescue_planner benchmark.launch planner_type:=...
~~~

## Testing

The `rescue_planner` package includes a **gtest** suite (`test/test_rescue_planner.cpp`) that
exercises the combinatorial planning modules (geometry, visibility graph, cell decomposition,
Voronoi roadmap, orienteering, Dubins, and the shared tour pipeline) plus the sampling planners'
raw collision checker. **No ROS master or simulation is required** — the tests build mock
worlds in memory and call the planning logic directly.

From inside the Docker container, after sourcing the workspace:

~~~
cd ros_ws
source devel/setup.bash
~~~

**Build and run all tests** (recommended):

~~~
catkin_make run_tests
~~~

To run only the `rescue_planner` tests:

~~~
catkin_make run_tests_rescue_planner_gtest_test_rescue_planner
~~~

Or run the test binary directly (after `catkin_make`):

~~~
./devel/lib/rescue_planner/test_rescue_planner
~~~

If you use **catkin_tools** instead of `catkin_make`:

~~~
catkin build rescue_planner
catkin test rescue_planner
~~~

A successful run prints `PASSED` for every test case and exits with code 0.

**Time budget in tests:** all integration and roadmap-budget checks use a **fixed
30 second** rescue timeout (`Dmax = 0.30 m/s × 30 s × 0.85 = 7.65 m`), matching
the course standard — never an unlimited budget. A separate 60 s budget test
verifies victim collection when the graph tour is long enough to require it.

**What is covered**

| Area | Module / target |
|------|-----------------|
| Clearance geometry | `comb::geometry_utils` |
| Shortest-path roadmap | `comb::visibility_graph` |
| Cell decomposition | `comb::cell_decomposition` |
| Maximum-clearance roadmap | `comb::voronoi_roadmap` |
| Victim selection | `comb::orienteering` |
| Flyable paths | `comb::dubins`, `comb::planTour` |
| Sampling collision checks | `collision_checker` (teammate code, tests only) |

The ROS planner nodes (`map_receiver`, launch files) are validated separately by running
`roslaunch rescue_planner rescue_map.launch planner_type:=...` as described above.

