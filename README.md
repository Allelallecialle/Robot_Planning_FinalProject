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

If needed, open a new terminal in the container with:
~~~
dock-other
~~~

## Running

### To run individual planners

Run the sample-based planners with roslaunch file. Default version with rrt:
~~~
roslaunch rescue_planner rescue_map.launch
~~~
or input the planner parameter (prm, rrt, rrt_star, visibility, cell_decomp, cell_decomp_approx, voronoi):
~~~
roslaunch rescue_planner rescue_map.launch planner_type:=prm
~~~

### To run the simulation with the Benchmark
Choose the planner type as in the previous case presented. 
The data about the simulation will be saved in the `benchmark.csv` in the `test_benchmark` folder.
~~~
roslaunch rescue_planner benchmark.launch planner_type:=...
~~~

### Building a custom / deterministic map

The map (borders, obstacles, gate, victims) is described in
`loco_nav/map_pkg/config/map_config.yaml`. On every launch (with the default
`generate_new_config:=true`) `generate_config_file.py` reads this file and
produces the resolved `full_config.yaml` that the spawn nodes consume.

To get the **same map on every run** so you can compare planners fairly, pin all
sources of randomness in `map_config.yaml`:

| Setting | Value for a deterministic map |
|---------|-------------------------------|
| `seed` (under `/**`) | any fixed integer, e.g. `42` (use `-1` for a fresh random map each launch) |
| `n_obstacles` (`send_obstacles`) | `0` so only your `vect_*` obstacles are spawned |
| `n_victims` (`send_victims`) | `0` so only your `vect_*` victims are spawned |
| gate `x` / `y` (`send_gates`) | any **non-`(0.0, 0.0)`** position (a gate at `(0,0)` is placed randomly) |

Then list your elements explicitly, keeping every `vect_*` list within a section
the same length. Example (already shipped in `map_config.yaml`):

~~~yaml
/**/send_obstacles:
  ros__parameters:
    n_obstacles: 0
    vect_type: ["box", "box", "cylinder", "cylinder"]
    vect_x:     [3.0, -3.0,  0.0, -4.0]
    vect_y:     [3.0,  3.0, -2.0, -4.0]
    vect_yaw:   [0.0,  0.5,  0.0,  0.0]
    vect_dim_x: [1.0,  1.5,  1.0,  0.8]   # box width / cylinder size
    vect_dim_y: [1.0,  1.0,  1.0,  0.8]   # ignored for cylinders

/**/send_gates:
  ros__parameters:
    x:   [0.0]
    y:   [-8.0]
    yaw: [1.5708]

/**/send_victims:
  ros__parameters:
    victims_activated: true
    n_victims: 0
    vect_x:      [4.0, -4.0, 2.0]
    vect_y:      [0.0,  0.0, 6.0]
    vect_weight: [100,  200, 150]

/**:
  ros__parameters:
    map: hexagon
    dx: 10.0
    seed: 42
~~~

Notes / gotchas:
- Every element (with its footprint) must lie **inside** the map polygon and must
  not overlap another element, the gate, or the robot at `(0,0)`; otherwise the
  generator aborts the launch with an `assert`.
- The launch reads the config from the **install space**, so after editing the
  YAML you must rebuild before it takes effect:
  ~~~
  cd ros_ws
  catkin_make
  source devel/setup.bash
  ~~~
- Run it as usual; the map is now reproducible:
  ~~~
  roslaunch rescue_planner rescue_map.launch planner_type:=voronoi
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
| Sampling collision checks | `collision_checker` |

The ROS planner nodes (`map_receiver`, launch files) are validated separately by running
`roslaunch rescue_planner rescue_map.launch planner_type:=...` as described above.

