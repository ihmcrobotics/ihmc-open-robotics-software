package us.ihmc.humanoidBehaviors.communication;

import toolbox_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket;

public interface CoactiveDataListenerInterface
{
   public void coactiveDataRecieved(SimpleCoactiveBehaviorDataPacket data);
}
