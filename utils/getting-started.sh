echo "Copying Extension to the .local folder"
cp -r isaacsim_extensions/kcl_project "~/.local/share/ov/data/Kit/Isaac-Sim Full/4.5/exts/3/kcl_project"
echo "Creating a new ROS2 Workspace"
mkdir -p ~/colcon_ws/src
echo "Copying CanadaArm2 Description Workspace to ROS2 Workspace"
cp -r ros2_packages/canadarm2_description/ ~/colcon_ws/src
echo "Copying CanadaArm2 Description Workspace to ROS2 Workspace"
cp -r ros2_packages/canadarm2_moveit_config ~/colcon_ws/src
