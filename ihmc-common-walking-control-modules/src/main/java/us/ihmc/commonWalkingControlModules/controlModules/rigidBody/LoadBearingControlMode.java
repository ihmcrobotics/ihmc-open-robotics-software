package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

public enum LoadBearingControlMode
{
   JOINTSPACE, ORIENTATION;

   public static final LoadBearingControlMode[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static LoadBearingControlMode fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}
