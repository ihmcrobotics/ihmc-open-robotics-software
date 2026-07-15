package us.ihmc.humanoidRobotics.communication.packets.walking;

public enum HumanoidBodyPart
{
   ARM,
   CHEST,
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
