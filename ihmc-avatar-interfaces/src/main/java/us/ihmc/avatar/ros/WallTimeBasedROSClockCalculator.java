package us.ihmc.avatar.ros;

import java.util.concurrent.atomic.AtomicLong;

import controller_msgs.msg.dds.RobotConfigurationData;

public class WallTimeBasedROSClockCalculator implements RobotROSClockCalculator
{
   private final AtomicLong currentTimestampOffset = new AtomicLong(-1L);

   public WallTimeBasedROSClockCalculator()
   {
   }

   @Override
   public void receivedRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      currentTimestampOffset.set(robotConfigurationData.getWallTime() - robotConfigurationData.getMonotonicTime());
   }

   @Override
   public long computeROSTime(long wallTime, long monotonicTime)
   {
      return wallTime;
   }

   @Override
   public long computeRobotMonotonicTime(long rosTime)
   {
      if (currentTimestampOffset.get() == -1L)
         return -1L;
      else
         return rosTime - currentTimestampOffset.get();
   }
}
