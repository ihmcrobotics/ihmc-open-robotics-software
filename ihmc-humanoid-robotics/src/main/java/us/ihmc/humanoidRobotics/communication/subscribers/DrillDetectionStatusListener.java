package us.ihmc.humanoidRobotics.communication.subscribers;

import controller_msgs.msg.dds.DrillDetectionPacket;

public interface DrillDetectionStatusListener
{
   public abstract void updateStatusPacket(DrillDetectionPacket drillDetectionPacket);
}
