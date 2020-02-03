trap 'exit' INT TERM
trap 'kill 0' EXIT

roscore &
sleep 3

echo
echo Starting 'pdw_turtlebot3_fake_node'
python pdw_turtlebot3_fake_node.wyv.py > /dev/null &
sleep 3
echo
echo 'pdw_turtlebot3_fake_node' started

echo
echo Starting 'pdw_robot_state_publisher'
python pdw_robot_state_publisher.wyv.py > /dev/null 2>&1 &
sleep 3
echo
echo 'pdw_robot_state_publisher' started

python pdw_turtlebot3_teleop_keyboard.wyv.py
