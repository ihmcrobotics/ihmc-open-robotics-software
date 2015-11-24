#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

class EncodingConverter
{

	private:
		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Subscriber sub;
		image_transport::Publisher pub;
	
	public:
		EncodingConverter()
			: it(nh)
		{
			sub = it.subscribe("/kinect2/sd/image_depth_rect", 1, &EncodingConverter::imageCallback, this);
			pub = it.advertise("/ihmc/kinect2/sd/image_depth_rect/converted", 1);
		};

		~EncodingConverter()
		{
		};

		void imageCallback(const sensor_msgs::ImageConstPtr& msg)
		{
			sensor_msgs::Image converted_msg;
			converted_msg.header = msg->header;
			converted_msg.height = msg->height;
			converted_msg.width = msg->width;
			converted_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
			converted_msg.is_bigendian = msg->is_bigendian;
			converted_msg.step = msg->step;
			converted_msg.data = msg->data;

			pub.publish(converted_msg);
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "encoding_converter");
	EncodingConverter ec;
	ros::spin();
	return 0;
}