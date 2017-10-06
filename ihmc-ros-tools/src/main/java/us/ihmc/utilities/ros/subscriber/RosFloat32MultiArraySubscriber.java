package us.ihmc.utilities.ros.subscriber;

public abstract class RosFloat32MultiArraySubscriber extends AbstractRosTopicSubscriber<std_msgs.Float32MultiArray> 
{	
	public RosFloat32MultiArraySubscriber()
	{
		super(std_msgs.Float32MultiArray._TYPE);	
	}
	
	public abstract void onNewMessage(std_msgs.Float32MultiArray message);
	// check if dimension size is > 2 throw exception
	// vector mag = data[dim0.stride*0+dim1.stride*0]
	// vector angle = data[dim0.stride*0+dim1.stride*1]
	
	
	
}
