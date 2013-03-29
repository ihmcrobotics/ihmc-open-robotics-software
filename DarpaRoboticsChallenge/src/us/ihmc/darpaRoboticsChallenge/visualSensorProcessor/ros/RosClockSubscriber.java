package us.ihmc.darpaRoboticsChallenge.visualSensorProcessor.ros;

import com.yobotics.simulationconstructionset.time.TimeProvider;

public class RosClockSubscriber extends RosTopicSubscriber<rosgraph_msgs.Clock> implements TimeProvider
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
