#ifndef TEST_H
#define TEST_H

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <test/Test.h>

#define TYPE_MASK   (7<<5)
#define TYPE_CODEC  (0<<5)
#define TYPE_BURST  (1<<5)

#define TEST_NUM_PRINT(t)    (((t) & ~TYPE_MASK) + 1)
#define TEST_SERIES_PRINT(t) ((((t) & TYPE_MASK) >> 5) + 1)

#define CODEC_TEST_COUNT 7

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
	ros::Subscriber sub_ready;
	ros::Subscriber sub;

	bool exit;
	int current_test;
	int delay_next_test, delay_data;

	int next_test(int cur_test);

	void callback(const test::Test &msg);
	void on_arduino_ready(const std_msgs::Bool &msg);

	void start_test(unsigned int test);
	void announce_test(unsigned int test);
	void publish_test(unsigned int test);
	void publish_codec_test(unsigned int test);
	void publish_burst_test(unsigned int test);

	int check_codec_test(const test::Test &msg);
	int check_burst_test(const test::Test &msg);

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
};

#endif
