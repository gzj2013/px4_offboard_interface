all: px4_offboard_control

px4_offboard_control: git_submodule mavlink_control.cpp
	g++ -I mavlink/include/mavlink/v1.0 mavlink_control.cpp serial_port.cpp autopilot_interface.cpp -o px4_offboard_control -lpthread

git_submodule:
	git submodule update --init --recursive

clean:
	 rm -rf *o px4_offboard_control
