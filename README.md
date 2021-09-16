## Straight line planner adhering to nav\_core::BaseGlobalPlanner class

To use the package add following line in your launch file under move\_base node package

`<param name="base_global_planner" value="straight_planner_ns/straight_planner"/>`

A simple configuration setup is provided in the `nav_stack` folder consisting of basic configuration and launch files. An rviz file is also provided to aid in saving some time which is usually rather wasted adding widgets.

### Quickstart

1. Go to your workspace and do `catkin_make`.

2. Source the workspace using `source <your_workspace>/devel/setup.bash`

3. Planner should be available as a plugin to `nav_core::BaseGlobalPlanner` now. Confirm by typing `rospack plugins --attrib=plugin nav_core` in the terminal, one of the planners should be `straight_planner` with its path specified.

### Branches

1. `master`: Empty template code that can be customized.

2. `Straight-line`: Straight line planner.
