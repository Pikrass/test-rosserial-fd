#include "test.h"

#include <stdio.h>

Test::Test()
	: nh(), exit(false), current_test(-1)
{
	pub_start = nh.advertise<std_msgs::UInt8>("start_test", 3);
	pub = nh.advertise<test::Test>("world_to_arduino", 3);
	pub2 = nh.advertise<test::Test2>("world_to_arduino2", 3);
	pub_inject = nh.advertise<test::Inject>("inject", 3);
	sub_ready = nh.subscribe("arduino_ready", 3, &Test::on_arduino_ready, this);
	sub = nh.subscribe("arduino_to_world", 3, &Test::callback, this);
	sub2 = nh.subscribe("arduino_to_world2", 3, &Test::callback2, this);

	printf("Initialized test_node\n");
}

void Test::spinOnce()
{
	if (current_test > -1) {
		if (delay_next_test-- == 0)
			announce_test(current_test);
		else if (delay_next_test < 0 && delay_data-- == 0)
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
			return TYPE_CODEC2;
	case TYPE_CODEC2:
		if (num < CODEC2_TEST_COUNT - 1)
			return type | (num + 1);
		else
			return TYPE_BURST;
	case TYPE_BURST:
		return TYPE_TRUNC;
	case TYPE_TRUNC:
		if (num < TRUNC_TEST_COUNT - 1)
			return type | (num + 1);
		else
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

int Test::check_trunc_test(const test::Test &msg)
{
	int succeeded = 1;

	const uint8_t test_data[] = {0x42, 0xff, 0xff, 0x01};
	unsigned int len = 4;

	for (int i = 0 ; i < len ; ++i) {
		if (msg.data[i+1] != test_data[i]) {
			succeeded = 0;
			break;
		}
	}

	return succeeded;
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
	case TYPE_TRUNC:
		succeeded = check_trunc_test(msg);
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

void Test::callback2(const test::Test2_res &msg)
{
	if (msg.test != current_test)
		return;

	const uint8_t *data = codec2_tests[current_test & ~TYPE_MASK];
	if (msg.data1 == data[0]
	    && msg.data2 == data[1]
	    && msg.data3 == data[2]) {
		printf(" OK\n");
	} else {
		printf(" FAIL\n");
		printf("Received: %02x %02x %02x\n",
		       msg.data1, msg.data2, msg.data3);
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

void Test::publish_codec2_test(unsigned int test)
{
	test::Test2 msg;
	const uint8_t *data = codec2_tests[test];

	msg.data1 = data[0];
	msg.data2 = data[1];
	msg.data3 = data[2];

	pub2.publish(msg);
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

void Test::publish_trunc_test(unsigned int test)
{
	const char* const packet_read =
		"\xff\xff\xff\xfd"
		"\x08\x00\xf7"
		"\x00\x65\x00\x00"
		"\x04\x00\x00\x00"
		"\x42\xff\xff\x01"
		"\x00\x55";
	const char* const packet_write =
		"\xff\xff\xff\xfd"
		"\x08\x00\xf7"
		"\x00\x7e\x00\x00"
		"\x04\x00\x00\x00"
		"\x42\xff\xff\x01"
		"\x00\x3c";
	const unsigned int packet_len = 21;

	test::Inject msg;
	const uint8_t *test_data = trunc_tests[test % TRUNC_TEST_COUNT_ONE_WAY];
	unsigned int len = *test_data++;

	const char* p = test >= TRUNC_TEST_COUNT_ONE_WAY
		? packet_write : packet_read;

	msg.data.reserve(len + packet_len);

	for (int i = 0 ; i < len ; ++i)
		msg.data.push_back(*test_data++);
	for (int i = 0 ; i < packet_len ; ++i)
		msg.data.push_back(*p++);

	msg.write_side = (test >= TRUNC_TEST_COUNT_ONE_WAY);
	pub_inject.publish(msg);
}

void Test::publish_test(unsigned int test)
{
	switch (test & TYPE_MASK) {
	case TYPE_CODEC:
		publish_codec_test(test & ~TYPE_MASK);
		break;
	case TYPE_CODEC2:
		publish_codec2_test(test & ~TYPE_MASK);
		break;
	case TYPE_BURST:
		publish_burst_test(test & ~TYPE_MASK);
		break;
	case TYPE_TRUNC:
		publish_trunc_test(test & ~TYPE_MASK);
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
