package us.ihmc.utilities.ros.subscriber;

import sensor_msgs.JointState;

public class RosJointStateSubscriber extends AbstractRosTopicSubscriber<sensor_msgs.JointState>
{
	public RosJointStateSubscriber()
	{
		super(sensor_msgs.JointState._TYPE);
	}

	@Override
	public void onNewMessage(JointState message)
	{
		// TODO Auto-generated method stub
		
	}
	
	@Override
	public void connected()
	{
		
	}

}
