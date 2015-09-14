package us.ihmc.utilities.ros.subscriber;

import us.ihmc.tools.TimestampProvider;

public class RosClockSubscriber extends AbstractRosTopicSubscriber<rosgraph_msgs.Clock> implements TimestampProvider
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

   public synchronized long getTimestamp()
   {
      return timeStamp;
   }

}
