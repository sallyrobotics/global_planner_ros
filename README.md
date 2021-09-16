## Template for nav\_core::BaseGlobalPlanner path planner

To use the package add following line in your launch file under move\_base node package

`<param name="base_global_planner" value="template_planner_ns/template_planner"/>`

**NOTE: You are required to fill the `findPath` function in `template_planner_pkg/src/template_planner.cpp` without which the planner won't work. A helper code is provided comented inside the function body that can be used to test out the planner quickly before writing your own algorithm.**

A simple configuration setup is provided in the `nav_stack` folder consisting of basic configuration and launch files. An rviz file is also provided to aid in saving some time which is usually rather wasted adding widgets.

[Implementation video](https://youtu.be/PxI2eISoebc)

### Quickstart

1. Go to `template_planner_pkg/src/template_planner.cpp` and uncomment contents of `findPath()` function.

2. Go to your workspace and do `catkin_make`.

3. Source your workspace using `source <your_workspace>/devel/setup.bash`.

4. Planner should be available as a plugin to `nav_core::BaseGlobalPlanner` now. Confirm by typing `rospack plugins --attrib=plugin nav_core` in the terminal, one of the planners should be `straight_planner` with its path specified. Try launching `roslaunch nav_stack move_base.launch` with turtlebot3\_house or turtlebot3\_world.

To customize, put in your own algorithm inside the `findPath` function in `template_planner_pkg/src/template_planner.cpp`.

### What is findPath function?

This is template code for global\_planner plugins that are used to do grid-based path planning with ROS.

To work with it, write your grid-based path planning code in `findPath` funciton of `template_planner_pkg/src/template_planner.cpp`. 

Inputs to this function are:

1. startCell: Index, instead of row & column of the corresponding cell, is passed to this function after flattening 2D grid in row-major 1D array.

2. goalCell: Index, instead of row & column of the corresponding cell, is passed to this function after flattening 2D grid in row-major 1D array.

3. width: Integer value representing number of cells in one row of the grid.

4. height: Integer value representing number of rows in the grid.

Return value must be a vector of integers representing index values of cells which are part of the path, first element of vector being start cell's index and last element being goal cell's index.

### Branches

1. `master`: Empty template code that can be customized.

2. `Straight-line`: Straight line planner.
