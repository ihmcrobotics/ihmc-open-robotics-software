package us.ihmc.utilities.ros.subscriber;

public abstract class RosTimestampSubscriber extends AbstractRosTopicSubscriber<multisense_ros.StampedPps>
{
   public RosTimestampSubscriber()
   {
      super(multisense_ros.StampedPps._TYPE);
   }
}
