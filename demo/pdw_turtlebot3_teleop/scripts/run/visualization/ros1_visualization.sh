export URDF=$(cat cpp/pdw_turtlebot3_fake/urdf/pdw_turtlebot3_burger.urdf)
rosparam set robot_description "$URDF"

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
rviz -d "$DIR/rviz/pdw_turtlebot3_fake_ros1.rviz"
