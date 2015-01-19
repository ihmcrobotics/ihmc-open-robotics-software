package us.ihmc.utilities.ros.publisher;

public class RosEmptyMessagePublisher extends RosTopicPublisher<std_msgs.Empty>
{
	public RosEmptyMessagePublisher()
	{
		this(false);
	}
	
	public RosEmptyMessagePublisher(boolean latched)
	{
		super(std_msgs.Empty._TYPE, latched);
	}
	
	public void publish()
	{
		publish(getMessage());
	}
}
