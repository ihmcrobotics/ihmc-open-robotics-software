package us.ihmc.behaviors.door;

import us.ihmc.communication.packets.ToolboxState;

public enum DoorType
{
   UNKNOWN_TYPE,
   PUSH_HANDLE_LEFT,
   PUSH_HANDLE_RIGHT,
   PULL_HANDLE_LEFT,
   PULL_HANDLE_RIGHT;

   public static final DoorType[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static DoorType fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      else
         return values[enumAsByte];
   }
}
