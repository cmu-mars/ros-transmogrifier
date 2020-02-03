set -e
set -x

wyarch -cnc TurtleBot3Teleop.wyc -deploy TurtleBot3Teleop.wyd
wypy pdw_turtlebot3_fake_node.wyv
wypy pdw_robot_state_publisher.wyv
wypy pdw_turtlebot3_teleop_keyboard.wyv
