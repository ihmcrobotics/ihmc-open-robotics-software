package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;

public enum FootstepStatus
{
   @RosEnumValueDocumentation(documentation = "execution of a footstep has begun. actualFootPositionInWorld and actualFootOrientationInWorld should be ignored in this state")
   STARTED, @RosEnumValueDocumentation(documentation = "a footstep is completed")
   COMPLETED;

   public static final FootstepStatus[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static FootstepStatus fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}