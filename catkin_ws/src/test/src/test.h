#ifndef TEST_H
#define TEST_H

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <test/Test.h>
#include <test/Test2.h>
#include <test/Test2_res.h>
#include <test/Inject.h>

#define TYPE_MASK   (7<<5)
#define TYPE_CODEC  (0<<5)
#define TYPE_CODEC2 (1<<5)
#define TYPE_BURST  (2<<5)
#define TYPE_TRUNC  (3<<5)

#define TEST_NUM_PRINT(t)    (((t) & ~TYPE_MASK) + 1)
#define TEST_SERIES_PRINT(t) ((((t) & TYPE_MASK) >> 5) + 1)

#define CODEC_TEST_COUNT 7
#define CODEC2_TEST_COUNT 3
#define TRUNC_TEST_COUNT_ONE_WAY 11
#define TRUNC_TEST_COUNT (TRUNC_TEST_COUNT_ONE_WAY*2)

class Test
{
public:
	Test();
	void spinOnce(void);
	bool running(void);

private:
	ros::NodeHandle nh;
	ros::Publisher pub_start;
	ros::Publisher pub;
	ros::Publisher pub2;
	ros::Publisher pub_inject;
	ros::Subscriber sub_ready;
	ros::Subscriber sub;
	ros::Subscriber sub2;

	bool exit;
	int current_test;
	int delay_next_test, delay_data;

	int next_test(int cur_test);

	void callback(const test::Test &msg);
	void callback2(const test::Test2_res &msg);
	void on_arduino_ready(const std_msgs::Bool &msg);

	void start_test(unsigned int test);
	void announce_test(unsigned int test);
	void publish_test(unsigned int test);
	void publish_codec_test(unsigned int test);
	void publish_codec2_test(unsigned int test);
	void publish_burst_test(unsigned int test);
	void publish_trunc_test(unsigned int test);

	int check_codec_test(const test::Test &msg);
	int check_burst_test(const test::Test &msg);
	int check_trunc_test(const test::Test &msg);

	void stop(void);

	const uint8_t codec_test_data[44] = {
		 6, 0x01, 0xff, 0xff, 0xff, 0x01, 0x01,
		 6, 0x01, 0xff, 0xff, 0x00, 0x01, 0x01,
		 6, 0x01, 0xff, 0xff, 0x01, 0x01, 0x01,
		10, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01,
		 3, 0xff, 0xff, 0xff,
		 3, 0xff, 0xff, 0x00,
		 3, 0xff, 0xff, 0x01,
	};

	const uint8_t *codec_tests[CODEC_TEST_COUNT] = {
		codec_test_data,
		codec_test_data + 7,
		codec_test_data + 14,
		codec_test_data + 21,
		codec_test_data + 32,
		codec_test_data + 36,
		codec_test_data + 40
	};

	const uint8_t codec2_tests[CODEC2_TEST_COUNT][3] = {
		{ 0xff, 0xff, 0xff },
		{ 0xff, 0xff, 0x00 },
		{ 0xff, 0xff, 0x01 },
	};

	const uint8_t trunc_test_data[105] = {
		2, 0xff, 0xff,

		5, 0xff, 0xff, 0xff, 0xfd, 0x01,

		5, 0xff, 0xff, 0xff, 0xfd, 0xff,

		6, 0xff, 0xff, 0xff, 0xfd, 0x06, 0x00,

		7, 0xff, 0xff, 0xff, 0xfd, 0x06, 0x00, 0xf9,

		8, 0xff, 0xff, 0xff, 0xfd, 0x06, 0x00, 0xf9, 0x00,

		9, 0xff, 0xff, 0xff, 0xfd, 0x06, 0x00, 0xf9, 0x00, 0x65,

		10, 0xff, 0xff, 0xff, 0xfd, 0x06, 0x00, 0xf9, 0x00, 0x65, 0x00,

		11, 0xff, 0xff, 0xff, 0xfd, 0x06, 0x00, 0xf9,
		0x00, 0x65, 0x00, 0x00,

		15, 0xff, 0xff, 0xff, 0xfd, 0x06, 0x00, 0xf9,
		0x00, 0x65, 0x00, 0x00,
		0x03, 0xff, 0xff, 0x00,

		16, 0xff, 0xff, 0xff, 0xfd, 0x06, 0x00, 0xf9,
		0x00, 0x65, 0x00, 0x00,
		0x03, 0xff, 0xff, 0x00, 0xff
	};

	const uint8_t *trunc_tests[TRUNC_TEST_COUNT_ONE_WAY] = {
		trunc_test_data,
		trunc_test_data + 3,
		trunc_test_data + 9,
		trunc_test_data + 15,
		trunc_test_data + 22,
		trunc_test_data + 30,
		trunc_test_data + 39,
		trunc_test_data + 49,
		trunc_test_data + 60,
		trunc_test_data + 72,
		trunc_test_data + 88
	};
};

#endif
