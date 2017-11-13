#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <test/Test.h>

#define DELAY     (20)
#define READY_MOD (1000/DELAY)

void start_test(const std_msgs::UInt8 &msg);
void callback(const test::Test &msg);

test::Test msg;
std_msgs::Bool ready_msg;

ros::NodeHandle nh;
ros::Subscriber<std_msgs::UInt8> sub_start("start_test", start_test);
ros::Subscriber<test::Test> sub("world_to_arduino", callback);
ros::Publisher pub_ready("arduino_ready", &ready_msg);
ros::Publisher pub("arduino_to_world", &msg);

int current_test = -1;
unsigned int ready_wait = 0;

void setup()
{
	ready_msg.data = true;

	nh.initNode();
	nh.subscribe(sub_start);
	nh.subscribe(sub);
	nh.advertise(pub_ready);
	nh.advertise(pub);
}

void start_test(const std_msgs::UInt8 &msg)
{
}

void callback(const test::Test &msg)
{
}

void loop()
{
	if (current_test == -1 && ready_wait++ % READY_MOD == 0)
		pub_ready.publish(&ready_msg);

	nh.spinOnce();

	delay(DELAY);
}
