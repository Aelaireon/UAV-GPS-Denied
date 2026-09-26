# UAV Flight Code

UAV stands for Unmanned Aerial Vehicle. This repository contains code to fly the UAV.

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
2. Run the make command to colcon build:
    ```bash
    make
    ```
3. You will need the gcs node up on the gcs computer:
    ```bash
    make gcs
    ```
4. You will need the mavros node up on the UAV:
    ```bash
    make mavros
    ```
5. You will need the flight node up on the UAV:
    ```bash
    make uav
    ```
6. You will need the tfminiplus node up on the UAV:
    ```bash
    make tfmini
    ```
7. You may need the optical flow node up on the UAV:
    ```bash
    make flow
    ```
