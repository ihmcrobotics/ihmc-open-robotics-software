package us.ihmc.humanoidRobotics.kryo;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.communication.net.PacketConsumer;

public interface PPSTimestampOffsetProvider extends PacketConsumer<RobotConfigurationData>
{
   public long getCurrentTimestampOffset();

   public long adjustTimeStampToRobotClock(long timeStamp);
   
   public boolean offsetIsDetermined();

   public long adjustRobotTimeStampToRosClock(long timeStamp);
}
