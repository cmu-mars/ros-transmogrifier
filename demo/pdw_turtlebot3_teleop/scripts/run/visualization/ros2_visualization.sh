DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo Starting RViz

python3 "$DIR/robot_desc_publisher.py" cpp/pdw_turtlebot3_fake/urdf/pdw_turtlebot3_burger.urdf &
URDF_PUBLISHER_PID=$!

sleep 3

rviz2 -d "$DIR/rviz/pdw_turtlebot3_fake_ros2.rviz"

kill -9 $URDF_PUBLISHER_PID
