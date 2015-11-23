#include <ros/ros.h>
#include <image_transport/image_transport.h>

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
			pub.publish(msg);
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "encoding_converter");
	EncodingConverter ec;
	ros::spin();
	return 0;
}