package us.ihmc.humanoidRobotics.kryo;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;

public interface PPSTimestampOffsetProvider extends PacketConsumer<RobotConfigurationData>
{
   public long getCurrentTimestampOffset();

   public long adjustTimeStampToRobotClock(long timeStamp);
   
   public boolean offsetIsDetermined();

   public long adjustRobotTimeStampToRosClock(long timeStamp);
}
