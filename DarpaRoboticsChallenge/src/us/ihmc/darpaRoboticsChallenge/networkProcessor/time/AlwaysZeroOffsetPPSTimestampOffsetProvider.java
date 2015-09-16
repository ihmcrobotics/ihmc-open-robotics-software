package us.ihmc.darpaRoboticsChallenge.networkProcessor.time;


import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.darpaRoboticsChallenge.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;

public class AlwaysZeroOffsetPPSTimestampOffsetProvider implements DRCROSPPSTimestampOffsetProvider
{
   public long getCurrentTimestampOffset()
   {
      return 0;
   }

   public long adjustTimeStampToRobotClock(long timeStamp)
   {
      return timeStamp;
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

   @Override
   public void attachToRosMainNode(RosMainNode rosMainNode)
   {
   }
}
