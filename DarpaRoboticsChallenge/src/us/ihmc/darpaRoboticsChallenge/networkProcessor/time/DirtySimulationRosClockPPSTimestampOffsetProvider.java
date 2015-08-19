package us.ihmc.darpaRoboticsChallenge.networkProcessor.time;

import org.ros.time.WallTimeProvider;

import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.darpaRoboticsChallenge.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;

public class DirtySimulationRosClockPPSTimestampOffsetProvider implements DRCROSPPSTimestampOffsetProvider
{
   private WallTimeProvider wallTimeProvider;
   
   public DirtySimulationRosClockPPSTimestampOffsetProvider()
   {
      wallTimeProvider = new WallTimeProvider();
   }
   
   public long getCurrentTimestampOffset() {
      return 0;
   }

   public long requestNewestRobotTimestamp() {
      return 0;
   }

   public long adjustTimeStampToRobotClock(long timeStamp) {
      return timeStamp;
   }

   public boolean offsetIsDetermined() {
      return true;
   }

   @Override
   public long adjustRobotTimeStampToRosClock(long timeStamp)
   {
      return wallTimeProvider.getCurrentTime().totalNsecs();
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
