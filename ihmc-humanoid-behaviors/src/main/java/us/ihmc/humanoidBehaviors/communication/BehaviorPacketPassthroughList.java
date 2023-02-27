package us.ihmc.humanoidBehaviors.communication;

import ihmc_common_msgs.msg.dds.StampedPosePacket;

public class BehaviorPacketPassthroughList
{
   public static Class[] PACKETS_TO_ALWAYS_PASS_FROM_NP_TO_CONTROLLER_THROUGH_BEHAVIORS = {
      StampedPosePacket.class
 };
}
