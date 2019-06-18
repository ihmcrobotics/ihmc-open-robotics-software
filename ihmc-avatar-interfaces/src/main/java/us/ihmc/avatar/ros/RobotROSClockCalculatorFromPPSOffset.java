package us.ihmc.avatar.ros;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.utilities.ros.RosMainNode;

public class RobotROSClockCalculatorFromPPSOffset implements RobotROSClockCalculator
{
   private final DRCROSPPSTimestampOffsetProvider ppsTimestampOffsetProvider;

   public RobotROSClockCalculatorFromPPSOffset(DRCROSPPSTimestampOffsetProvider ppsTimestampOffsetProvider)
   {
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
   }

   @Override
   public void setROSMainNode(RosMainNode rosMainNode)
   {
      ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);
   }

   @Override
   public void receivedRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      ppsTimestampOffsetProvider.receivedPacket(robotConfigurationData);
   }

   @Override
   public long computeROSTime(long wallTime, long monotonicTime)
   {
      return ppsTimestampOffsetProvider.adjustTimeStampToRobotClock(monotonicTime);
   }

   @Override
   public long computeRobotMonotonicTime(long rosTime)
   {
      return ppsTimestampOffsetProvider.adjustTimeStampToRobotClock(rosTime);
   }
}
