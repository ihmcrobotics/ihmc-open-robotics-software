package us.ihmc.utilities.ros;

public abstract class RosTimestampSubscriber extends AbstractRosTopicSubscriber<std_msgs.Time>
{
   public RosTimestampSubscriber()
   {
      super(std_msgs.Time._TYPE);
   }
}
