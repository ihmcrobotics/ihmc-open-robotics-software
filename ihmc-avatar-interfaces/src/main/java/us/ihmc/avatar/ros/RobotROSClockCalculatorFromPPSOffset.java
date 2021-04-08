package us.ihmc.avatar.ros;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.log.LogTools;
import us.ihmc.utilities.ros.RosNodeInterface;

public class RobotROSClockCalculatorFromPPSOffset implements RobotROSClockCalculator
{
   private final DRCROSPPSTimestampOffsetProvider ppsTimestampOffsetProvider;

   public RobotROSClockCalculatorFromPPSOffset(DRCROSPPSTimestampOffsetProvider ppsTimestampOffsetProvider)
   {
      LogTools.info("Using PPS offset provider: {}", ppsTimestampOffsetProvider.getClass().getSimpleName());
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
   }

   @Override
   public void subscribeToROS1Topics(RosNodeInterface ros1Node)
   {
      ppsTimestampOffsetProvider.subscribeToROS1Topics(ros1Node);
   }

   @Override
   public void unsubscribeFromROS1Topics(RosNodeInterface ros1Node)
   {
      ppsTimestampOffsetProvider.unsubscribeFromROS1Topics(ros1Node);
   }

   @Override
   public void receivedRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      ppsTimestampOffsetProvider.receivedPacket(robotConfigurationData);
   }

   @Override
   public long computeROSTime(long wallTime, long monotonicTime)
   {
      return ppsTimestampOffsetProvider.adjustRobotTimeStampToRosClock(monotonicTime);
   }

   @Override
   public long computeRobotMonotonicTime(long rosTime)
   {
      return ppsTimestampOffsetProvider.adjustTimeStampToRobotClock(rosTime);
   }

   @Override
   public boolean offsetIsDetermined()
   {
      return ppsTimestampOffsetProvider.offsetIsDetermined();
   }

   @Override
   public long getCurrentTimestampOffset()
   {
      return ppsTimestampOffsetProvider.getCurrentTimestampOffset();
   }
}
