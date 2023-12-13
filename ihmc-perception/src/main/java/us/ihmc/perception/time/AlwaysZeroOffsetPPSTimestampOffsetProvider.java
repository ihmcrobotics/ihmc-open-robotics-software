package us.ihmc.perception.time;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;

public class AlwaysZeroOffsetPPSTimestampOffsetProvider implements PPSTimestampOffsetProvider
{

   public AlwaysZeroOffsetPPSTimestampOffsetProvider()
   {
      super();
   }

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

}