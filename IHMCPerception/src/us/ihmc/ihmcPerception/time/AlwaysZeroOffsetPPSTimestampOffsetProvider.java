package us.ihmc.ihmcPerception.time;

import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;

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