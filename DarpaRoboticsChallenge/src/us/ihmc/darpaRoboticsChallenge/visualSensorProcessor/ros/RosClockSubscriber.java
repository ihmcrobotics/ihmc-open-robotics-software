package us.ihmc.darpaRoboticsChallenge.visualSensorProcessor.ros;

import us.ihmc.utilities.net.TimeStampProvider;

public class RosClockSubscriber extends RosTopicSubscriber<rosgraph_msgs.Clock> implements TimeStampProvider
{
   private long timeStamp = 0;
   
   public RosClockSubscriber()
   {
      super(rosgraph_msgs.Clock._TYPE);
   }

   public synchronized void onNewMessage(rosgraph_msgs.Clock message)
   {
      timeStamp = message.getClock().totalNsecs();
   }

   public synchronized long getTimeStamp()
   {
      return timeStamp;
   }

}
