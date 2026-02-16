
ROS 2 packages to implement generic ros2-controllers based with limo simulation.

## Table of Contents

- [Limo series - Gazebo simulation](#)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Installation](#installation)
  - [Usage](#usage)
    - [With ROS](#with-ros-2)
  - [Development](#development)
    - [Pre-Commit](#pre-commit)
    - [Tests](#tests)
  - [Known Bugs](#known-bugs)
  - [Author](#author)

## Overview

## Installation

These packages have been tested with ROS 2 Humble and ROS 2 Iron on an Ubuntu system.

To use Torch with an NVIDIA graphics card, it is necessary to install the NVIDIA drivers for Ubuntu. [Here](https://letmegooglethat.com/?q=Install+nvidia+drivers+ubuntu).

## Usage

Create a workspace and clone the repo in the source directory (don't  call your workspace a generic `<my_workspace>`)
```shell
mkdir <my_workspace>
cd <my_workspace>
git clone <whatever you want>
```

Build the workspace with
```shell
colcon build --symlink-install
```

Source the workspace with (you have to add it to the `~/.bashrc` or do it on every newly opened terminal)
```shell
source install/setup.bash
```

### Scripts

- limo_simulation
  provide a gazebo simulation for testing limo models in gazebo.  

#### Examples
```shell
ros2 launch limo_simulation gazebo_models_diff.launch.py
```
this will spawn n robots depending on the position passed in the launch file. Robot are differential drive
Then you can develop your own algorithm and test it in gazebo.
Inside src folder, there is a simple diff_drive_publisher BUT be carefull to adjust the namespacing if you are spawning more robots


## Author

- [Federico Iadarola](https://github.com/fedeiada)
