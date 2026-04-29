# UAV Flight Code

This repository contains code to fly the UAV.

## How to Use

1. Clone the repository to your local machine.
2. Run the provided install script:
    ```bash
    . install.sh
    ```
3. Source ROS 2 and build the packages using the provided makefile `make` command or `colcon build --symlink-install` at workspace root, as long as all packages are finished building without being skipped or dropped (stderr is common and probably fine):
    ```bash
    source /opt/ros/jazzy/setup.bash
    make # run twice initially to ensure all packages' printouts are started and finished
    make
    ```

## Running the code

1. Source the ROS 2 and the workspace on all new terminal tabs for each of the later commands:
    ```bash
    source /opt/ros/jazzy/setup.bash # ROS 2
    source install/setup.bash # Workspace
    ```
8. To start Challenge 1 node:
    ```bash
    make challenge-1
    ```
