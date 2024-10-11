package us.ihmc.humanoidOperatorInterface.communication.subscribers;

import perception_msgs.msg.dds.DrillDetectionPacket;

public interface DrillDetectionStatusListener
{
   public abstract void updateStatusPacket(DrillDetectionPacket drillDetectionPacket);
}
