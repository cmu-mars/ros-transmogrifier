set -e
set -x

wyarch -cnc TurtleBot3Teleop.wyc -deploy TurtleBot3Teleop.wyd
wypy pdw_turtlebot3_fake_node_.wyv
wypy pdw_robot_state_publisher_.wyv
wypy pdw_turtlebot3_teleop_keyboard_.wyv
