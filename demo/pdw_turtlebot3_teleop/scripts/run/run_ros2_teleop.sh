trap 'exit' INT TERM
trap 'kill 0' EXIT

echo
echo Starting 'pdw_turtlebot3_fake_node'
python3 pdw_turtlebot3_fake_node_.wyv.py > /dev/null &
sleep 3
echo
echo 'pdw_turtlebot3_fake_node' started

echo
echo Starting 'pdw_robot_state_publisher'
python3 pdw_robot_state_publisher_.wyv.py > /dev/null 2>&1 &
sleep 3
echo
echo 'pdw_robot_state_publisher' started

python3 pdw_turtlebot3_teleop_keyboard_.wyv.py
