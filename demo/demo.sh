set -x
cd ../examples/teleop/brass-pdw-2019-11

sudo docker build -t brass-pdw/teleop .

cd ../../../demo

mkdir -p rosdiscover_output

sudo docker pull christimperley/rosdiscover
sudo docker tag christimperley/rosdiscover rosdiscover
sudo docker run --rm -v /var/run/docker.sock:/var/run/docker.sock -v $PWD/rosdiscover_output/:/output \
	 -it rosdiscover launch --output /output/teleop_arch.yml brass-pdw/teleop \
	 /root/brass_pdw/src/pdw_turtlebot3_fake/launch/pdw_turtlebot3_fake.launch \
	 /root/brass_pdw/src/pdw_turtlebot3_teleop/launch/pdw_turtlebot3_teleop_key.launch

mkdir -p src/pdw_robot_state_publisher
cp -r ../examples/teleop/brass-pdw-2019-11/pdw_robot_state_publisher/include src/pdw_robot_state_publisher
cp -r ../examples/teleop/brass-pdw-2019-11/pdw_robot_state_publisher/src src/pdw_robot_state_publisher

mkdir -p src/pdw_turtlebot3_fake
cp -r ../examples/teleop/brass-pdw-2019-11/pdw_turtlebot3_fake/include src/pdw_turtlebot3_fake
cp -r ../examples/teleop/brass-pdw-2019-11/pdw_turtlebot3_fake/src src/pdw_turtlebot3_fake

mkdir -p src/pdw_turtlebot3_fake/meshes/bases
cp ../examples/teleop/brass-pdw-2019-11/pdw_turtlebot3_fake/meshes/bases/burger_base.stl src/pdw_turtlebot3_fake/meshes/bases

mkdir -p src/pdw_turtlebot3_fake/meshes/sensors
cp ../examples/teleop/brass-pdw-2019-11/pdw_turtlebot3_fake/meshes/sensors/lds.stl src/pdw_turtlebot3_fake/meshes/sensors

mkdir -p src/pdw_turtlebot3_fake/meshes/wheels
cp ../examples/teleop/brass-pdw-2019-11/pdw_turtlebot3_fake/meshes/wheels/left_tire.stl src/pdw_turtlebot3_fake/meshes/wheels
cp ../examples/teleop/brass-pdw-2019-11/pdw_turtlebot3_fake/meshes/wheels/right_tire.stl src/pdw_turtlebot3_fake/meshes/wheels

cp ../examples/teleop/brass-pdw-2019-11/pdw_turtlebot3_fake/urdf/pdw_turtlebot3_burger.urdf.xacro pdw_turtlebot3_burger.urdf.xacro

cp ../examples/teleop/brass-pdw-2019-11/pdw_turtlebot3_fake/rviz/ros1/pdw_turtlebot3_fake.rviz pdw_turtlebot3_fake_ros1.rviz
cp ../examples/teleop/brass-pdw-2019-11/pdw_turtlebot3_fake/rviz/ros2/model.rviz pdw_turtlebot3_fake_ros2.rviz

sudo docker build -t brass-pdw/teleop-demo .

rm -r src
rm pdw_turtlebot3_burger.urdf.xacro
rm pdw_turtlebot3_fake_ros1.rviz
rm pdw_turtlebot3_fake_ros2.rviz
