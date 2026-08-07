cd /media/asus/3C2A2653CEF25C8F/rohand_ros2_pkg-main
colcon build
source install/setup.bash
sudo chmod 666 /dev/ttyUSB0
ros2 run rohand rohand_modbus_ap001



ros2 topic pub --once /rohand_node/target_joint_states sensor_msgs/msg/JointState \
"{header: {frame_id: 'rohand_2'}, position: [36.0, 174.0, 174.0, 174.0, 178.0, 0.0]}"