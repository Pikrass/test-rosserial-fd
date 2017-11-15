#include "test.h"

#include <stdio.h>

Test::Test()
	: nh(), current_test(-1)
{
	pub_start = nh.advertise<std_msgs::UInt8>("start_test", 3);
	pub = nh.advertise<test::Test>("world_to_arduino", 3);
	sub_ready = nh.subscribe("arduino_ready", 3, &Test::on_arduino_ready, this);
	sub = nh.subscribe("arduino_to_world", 3, &Test::callback, this);

	printf("Initialized test_node\n");
}

void Test::spinOnce()
{
	if (current_test > -1)
		publish_test(current_test);

	ros::spinOnce();
}

void Test::callback(const test::Test &msg)
{
	if (msg.data[0] != current_test)
		return;

	bool succeeded = true;

	const uint8_t *test_data = codec_tests[current_test];
	unsigned int len = *test_data++;

	for (int i = 1 ; i < len+1 ; ++i) {
		if (msg.data[i] != *test_data++) {
			succeeded = false;
			break;
		}
	}

	if (succeeded)
		printf(" OK\n");
	else
		printf(" FAIL\n");

	if (current_test < TEST_COUNT - 1)
		run_test(current_test + 1);
	else
		stop();
}

void Test::on_arduino_ready(const std_msgs::Bool &msg)
{
	if (msg.data) {
		printf("Arduino ready, starting tests\n");
		run_test(0);
	} else {
		stop();
	}
}

void Test::run_test(unsigned int test)
{
	printf("Test %d...", test + 1);

	std_msgs::UInt8 start_msg;
	start_msg.data = test;
	pub_start.publish(start_msg);

	current_test = test;
}

void Test::publish_test(unsigned int test)
{
	test::Test msg;
	const uint8_t *test_data = codec_tests[test];
	unsigned int len = *test_data++;

	msg.data.reserve(len);

	for (int i = 0 ; i < len ; ++i)
		msg.data.push_back(*test_data++);

	pub.publish(msg);
}

void Test::stop()
{
	current_test = -1;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test");

	Test test;

	ros::Rate r(100);
	while (ros::ok()) {
		test.spinOnce();
		r.sleep();
	}

	return 0;
}
