package us.ihmc.humanoidRobotics.communication.subscribers;

import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;

public interface CapturabilityBasedStatusListener
{
   public abstract void updateStatusPacket(CapturabilityBasedStatus capturabilityBasedStatus);
}
