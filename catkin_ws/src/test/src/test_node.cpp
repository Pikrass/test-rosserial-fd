#include "test.h"

#include <stdio.h>

Test::Test()
	: nh(), exit(false), current_test(-1)
{
	pub_start = nh.advertise<std_msgs::UInt8>("start_test", 3);
	pub = nh.advertise<test::Test>("world_to_arduino", 3);
	sub_ready = nh.subscribe("arduino_ready", 3, &Test::on_arduino_ready, this);
	sub = nh.subscribe("arduino_to_world", 3, &Test::callback, this);

	printf("Initialized test_node\n");
}

void Test::spinOnce()
{
	if (current_test > -1) {
		if (delay_next_test-- == 0)
			announce_test(current_test);
		else if (delay_next_test < 0 && delay_data-- <= 0)
			publish_test(current_test);
	}

	ros::spinOnce();
}

int Test::next_test(int cur_test)
{
	int type = cur_test & TYPE_MASK;
	int num = cur_test & ~TYPE_MASK;

	switch (type) {
	case TYPE_CODEC:
		if (num < CODEC_TEST_COUNT - 1)
			return type | (num + 1);
		else
			return TYPE_BURST;
	case TYPE_BURST:
		return -1;
	default:
		return -1;
	}
}

int Test::check_codec_test(const test::Test &msg)
{
	int succeeded = 1;

	const uint8_t *test_data = codec_tests[current_test];
	unsigned int len = *test_data++;

	for (int i = 1 ; i < len+1 ; ++i) {
		if (msg.data[i] != *test_data++) {
			succeeded = 0;
			break;
		}
	}

	return succeeded;
}

int Test::check_burst_test(const test::Test &msg)
{
	static bool first = true;
	bool succeeded = true;

	const uint8_t *test_data = codec_tests[0];
	unsigned int len = *test_data++;

	for (int i = 1 ; i < len+1 ; ++i) {
		if (msg.data[i] != *test_data++) {
			succeeded = false;
			break;
		}
	}

	if (!succeeded) {
		return 0;
	} else {
		if (first) {
			first = false;
			return -1;
		} else {
			first = true;
			return 1;
		}
	}
}

void Test::callback(const test::Test &msg)
{
	if (msg.data[0] != current_test)
		return;

	int succeeded;

	switch (current_test & TYPE_MASK) {
	case TYPE_CODEC:
		succeeded = check_codec_test(msg);
		break;
	case TYPE_BURST:
		succeeded = check_burst_test(msg);
		break;
	}

	if (succeeded == 1) {
		printf(" OK\n");
	} else if (succeeded == 0) {
		printf(" FAIL\n");
		printf("Received:");
		int len = msg.data.size();
		for (int i = 0 ; i < len ; ++i)
			printf(" %02x", msg.data[i]);
		printf("\n");
	} else {
		return;
	}

	int next = next_test(current_test);
	if (next > -1)
		start_test(next);
	else
		stop();
}

void Test::on_arduino_ready(const std_msgs::Bool &msg)
{
	if (msg.data) {
		printf("Arduino ready, starting tests\n");
		start_test(0);
	} else {
		stop();
	}
}

void Test::start_test(unsigned int test)
{
	printf("Series %d test %d...",
	       TEST_SERIES_PRINT(test), TEST_NUM_PRINT(test));

	delay_next_test = 3;
	delay_data = 3;
	current_test = test;
}

void Test::announce_test(unsigned int test)
{
	std_msgs::UInt8 start_msg;
	start_msg.data = current_test;
	pub_start.publish(start_msg);
}

void Test::publish_codec_test(unsigned int test)
{
	test::Test msg;
	const uint8_t *test_data = codec_tests[test];
	unsigned int len = *test_data++;

	msg.data.reserve(len);

	for (int i = 0 ; i < len ; ++i)
		msg.data.push_back(*test_data++);

	pub.publish(msg);
}

void Test::publish_burst_test(unsigned int test)
{
	test::Test msg;
	const uint8_t *test_data = codec_tests[0];
	unsigned int len = *test_data++;

	msg.data.reserve(len);

	for (int i = 0 ; i < len ; ++i)
		msg.data.push_back(*test_data++);

	pub.publish(msg);
	pub.publish(msg);
}

void Test::publish_test(unsigned int test)
{
	switch (test & TYPE_MASK) {
	case TYPE_CODEC:
		publish_codec_test(test & ~TYPE_MASK);
		break;
	case TYPE_BURST:
		publish_burst_test(test & ~TYPE_MASK);
		break;
	}
}

void Test::stop()
{
	current_test = -1;
	exit = true;
}

bool Test::running()
{
	return !exit;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test");

	Test test;

	ros::Rate r(10);
	while (ros::ok() && test.running()) {
		test.spinOnce();
		r.sleep();
	}

	return 0;
}
