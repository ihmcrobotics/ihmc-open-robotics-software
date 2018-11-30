package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;

public enum LoadBearingRequest
{
   @RosEnumValueDocumentation(documentation = "Request to load the given end-effector.")
   LOAD,
   @RosEnumValueDocumentation(documentation = "Request to unload the given end-effector.")
   UNLOAD;

   public static final LoadBearingRequest[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static LoadBearingRequest fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}