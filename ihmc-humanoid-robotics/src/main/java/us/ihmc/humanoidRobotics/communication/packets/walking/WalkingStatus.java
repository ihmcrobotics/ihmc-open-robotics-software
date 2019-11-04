package us.ihmc.humanoidRobotics.communication.packets.walking;

import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;

public enum WalkingStatus
{
   @RosEnumValueDocumentation(documentation = "The robot has begun its initial transfer/sway at the start of a walking plan")
   STARTED,
   @RosEnumValueDocumentation(documentation = "The robot has finished its final transfer/sway at the end of a walking plan")
   COMPLETED,
   @RosEnumValueDocumentation(documentation = "A walking abort has been requested")
   ABORT_REQUESTED,
   @RosEnumValueDocumentation(documentation = "The robot is back to standing on a break waiting for either an un-pause command or new footsteps.")
   PAUSED,
   @RosEnumValueDocumentation(documentation = "The robot is resuming the series of footsteps that were paused.")
   RESUMED;

   public static final WalkingStatus[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static WalkingStatus fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}