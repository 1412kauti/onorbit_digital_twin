# Digital Twins for Onorbit Space Operations

## Installation
### Automated Script (Recommended)
> Should not require `sudo` at any point of the operation
```bash
cd utils
bash getting-started.sh
```
### Manual Installation

#### Copying kcl_project Extension to the .local folder
```
cp -r isaacsim_extensions/kcl_project "~/.local/share/ov/data/Kit/Isaac-Sim Full/4.5/exts/3/kcl_project"
```
#### Create a ROS2 Humble Workspace (if it doesnt exist already)
```
mkdir -p ~/colcon_ws/src
```
#### Copy CanadaArm2 MoveIt Config Package to ROS2 Humble Workspace
```
cp -r ros2_packages/canadarm2_description/ ~/colcon_ws/src
```
#### Copy CanadaArm2 MoveIt Config Package to ROS2 Humble Workspace
```
cp -r ros2_packages/canadarm2_moveit_config ~/colcon_ws/src
```

## Usage

The Extension now should open up as an optional Extension that You need to enable to be welcomed by an Isaac Sim Native UI.

### For ROS2 (System Level Install )
Prior to running to Isaac Sim, Make sure to source the base ROS2 workspace, considering that you are using the default installation of ROS2, it should open with
```bash
source /opt/ros/humble/setup.bash
```
and then run your Isaac Sim as usual (Again, considering that Isaac Sim is installed at ``~/isaacsim`)
```bash
.~/isaacsim/isaac-sim.sh
```
### For ROS2 (Using ROS2 Bridge) 
Incase you dont have ROS2 Installed, you can use the preinstalled ROS2 packages in Isaac Sim with ROS2 bridge using:
```bash
.~/isaacsim/isaac-sim.selector.sh
```

