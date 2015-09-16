package us.ihmc.humanoidRobotics.communication.subscribers;

import us.ihmc.humanoidRobotics.communication.packets.sensing.DrillDetectionPacket;

public interface DrillDetectionStatusListener
{
   public abstract void updateStatusPacket(DrillDetectionPacket drillDetectionPacket);
}
