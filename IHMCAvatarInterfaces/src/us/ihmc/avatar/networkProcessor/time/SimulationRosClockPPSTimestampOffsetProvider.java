package us.ihmc.avatar.networkProcessor.time;

import org.ros.message.Time;

import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosClockPublisher;

public class SimulationRosClockPPSTimestampOffsetProvider implements DRCROSPPSTimestampOffsetProvider
{

   private RosClockPublisher clockPubisher;
   private long previousTimestamp = 0;

   public SimulationRosClockPPSTimestampOffsetProvider()
   {
      clockPubisher = new RosClockPublisher();
   }

   public long getCurrentTimestampOffset()
   {
      return 0;
   }

   public long requestNewestRobotTimestamp()
   {
      return 0;
   }

   public long adjustTimeStampToRobotClock(long timeStamp)
   {
      return timeStamp;
   }

   @Override
   public void attachToRosMainNode(RosMainNode rosMainNode)
   {
      rosMainNode.attachPublisher("/clock", clockPubisher);
   }

   public boolean offsetIsDetermined()
   {
      return true;
   }

   @Override
   public long adjustRobotTimeStampToRosClock(long timeStamp)
   {
      publishRosClock(timeStamp);
      return timeStamp;
   }

   public void publishRosClock(long timestamp)
   {
      if (timestamp <= previousTimestamp)
         return; // Do not set timestamps from the past, screws up ROS.

      Time time = Time.fromNano(timestamp);
      clockPubisher.publish(time);
      previousTimestamp = timestamp;
   }

   @Override
   public void receivedPacket(RobotConfigurationData packet)
   {
      
   }
}
