package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;

public enum HumanoidBodyPart
{
   @RosEnumValueDocumentation(documentation = "Request the chest to go back to a straight up configuration.")
   ARM,
   @RosEnumValueDocumentation(documentation = "Request the arm to go to a preconfigured home configuration that is elbow lightly flexed, forearm pointing forward, and upper pointing downward.")
   CHEST,
   @RosEnumValueDocumentation(documentation = "Request the pelvis to go back to between the feet, zero pitch and roll, and headed in the same direction as the feet.")
   PELVIS;

   public static final HumanoidBodyPart[] values = values();

   public boolean isRobotSideNeeded()
   {
      switch (this)
      {
      case ARM:
         return true;
      case CHEST:
      case PELVIS:
         return false;
      default:
         throw new RuntimeException("Should not get there.");
      }
   }

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static HumanoidBodyPart fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}