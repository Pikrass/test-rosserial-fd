all:
	cd catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Debug install
	rm -rf arduino/ros_lib/
	rosrun rosserial_arduino make_libraries.py arduino
	make -C arduino/test
