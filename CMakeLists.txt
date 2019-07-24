cmake_minimum_required(VERSION 2.8.3)
project(rospy_wrapper)

find_package(catkin REQUIRED
	COMPONENTS
	message_generation
	std_msgs)

catkin_python_setup()

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
	CATKIN_DEPENDS
	std_msgs)

if(CATKIN_ENABLE_TESTING)
	catkin_add_nosetests(test/test_topic.py)
	catkin_add_nosetests(test/test_bag.py)
endif()
