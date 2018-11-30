package us.ihmc.communication.packets;

public enum ToolboxState
{
   WAKE_UP, REINITIALIZE, SLEEP;

   public static final ToolboxState[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static ToolboxState fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}