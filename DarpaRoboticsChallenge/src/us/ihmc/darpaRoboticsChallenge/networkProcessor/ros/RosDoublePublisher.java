package us.ihmc.darpaRoboticsChallenge.networkProcessor.ros;

import std_msgs.Float64;

public class RosDoublePublisher extends RosTopicPublisher<std_msgs.Float64>
{
   public RosDoublePublisher(boolean latched)
   {
      super(std_msgs.Float64._TYPE, latched);
   }

   public void publish(double value)
   {
      Float64 message = getMessage();
      message.setData(value);
      publish(message);
   }
}
