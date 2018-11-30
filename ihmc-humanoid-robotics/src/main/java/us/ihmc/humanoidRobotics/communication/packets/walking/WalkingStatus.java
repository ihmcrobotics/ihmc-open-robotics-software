package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;

public enum WalkingStatus
{
   @RosEnumValueDocumentation(documentation = "The robot has begun its initial transfer/sway at the start of a walking plan")
   STARTED,
   @RosEnumValueDocumentation(documentation = "The robot has finished its final transfer/sway at the end of a walking plan")
   COMPLETED,
   @RosEnumValueDocumentation(documentation = "A walking abort has been requested")
   ABORT_REQUESTED;

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