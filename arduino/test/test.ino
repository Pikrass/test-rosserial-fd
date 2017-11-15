#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <test/Test.h>
#include <test/Test2.h>
#include <test/Test2_res.h>

#define DELAY     (20)
#define READY_MOD (1000/DELAY)

void start_test(const std_msgs::UInt8 &msg);
void callback(const test::Test &msg);
void callback2(const test::Test2 &test_data);

test::Test msg;
test::Test2_res msg2;
std_msgs::Bool ready_msg;

ros::NodeHandle nh;
ros::Subscriber<std_msgs::UInt8> sub_start("start_test", start_test);
ros::Subscriber<test::Test> sub("world_to_arduino", callback);
ros::Subscriber<test::Test2> sub2("world_to_arduino2", callback2);
ros::Publisher pub_ready("arduino_ready", &ready_msg);
ros::Publisher pub("arduino_to_world", &msg);
ros::Publisher pub2("arduino_to_world2", &msg2);

int current_test = -1;
unsigned int ready_wait = 0;

void setup()
{
	ready_msg.data = true;

	nh.initNode();
	nh.subscribe(sub_start);
	nh.subscribe(sub);
	nh.subscribe(sub2);
	nh.advertise(pub_ready);
	nh.advertise(pub);
	nh.advertise(pub2);
}

void start_test(const std_msgs::UInt8 &msg)
{
	current_test = msg.data;
}

void callback(const test::Test &test_data)
{
	if (current_test < 0)
		return;

	uint8_t res[100];
	unsigned int len = test_data.data_length + 1;

	res[0] = current_test;

	for (unsigned int i = 0 ; i < test_data.data_length ; ++i)
		res[i+1] = test_data.data[i];

	msg.data_length = len;
	msg.data = res;
	pub.publish(&msg);
}

void callback2(const test::Test2 &test_data)
{
	if (current_test < 0)
		return;

	msg2.test = current_test;
	msg2.data1 = test_data.data1;
	msg2.data2 = test_data.data2;
	msg2.data3 = test_data.data3;
	pub2.publish(&msg2);
}

void loop()
{
	if (current_test == -1 && ready_wait++ % READY_MOD == 0)
		pub_ready.publish(&ready_msg);

	nh.spinOnce();

	delay(DELAY);
}
