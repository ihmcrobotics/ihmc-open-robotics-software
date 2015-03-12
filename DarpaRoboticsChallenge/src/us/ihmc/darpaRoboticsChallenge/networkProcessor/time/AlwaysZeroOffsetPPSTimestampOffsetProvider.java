package us.ihmc.darpaRoboticsChallenge.networkProcessor.time;


import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;

public class AlwaysZeroOffsetPPSTimestampOffsetProvider implements PPSTimestampOffsetProvider
{
   public long getCurrentTimestampOffset()
   {
      return 0;
   }

   public long adjustTimeStampToRobotClock(long timeStamp)
   {
      return timeStamp;
   }

   public void attachToRosMainNode(RosMainNode rosMainNode)
   {
      return;
   }

   public boolean offsetIsDetermined()
   {
      return true;
   }

   @Override
   public long adjustRobotTimeStampToRosClock(long timeStamp)
   {
      return timeStamp;
   }

   @Override
   public void receivedPacket(RobotConfigurationData packet)
   {
      
   }
}
