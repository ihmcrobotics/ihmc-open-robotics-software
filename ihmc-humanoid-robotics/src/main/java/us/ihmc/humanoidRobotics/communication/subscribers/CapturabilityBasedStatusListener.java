package us.ihmc.humanoidRobotics.communication.subscribers;

import controller_msgs.msg.dds.CapturabilityBasedStatus;

public interface CapturabilityBasedStatusListener
{
   public abstract void updateStatusPacket(CapturabilityBasedStatus capturabilityBasedStatus);
}
