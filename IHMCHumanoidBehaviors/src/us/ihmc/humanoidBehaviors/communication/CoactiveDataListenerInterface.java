package us.ihmc.humanoidBehaviors.communication;

import us.ihmc.humanoidRobotics.communication.packets.behaviors.SimpleCoactiveBehaviorDataPacket;

public interface CoactiveDataListenerInterface
{
   public void coactiveDataRecieved(SimpleCoactiveBehaviorDataPacket data);
}
