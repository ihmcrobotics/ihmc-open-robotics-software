package us.ihmc.tools.processManagement;

public class UnsignedByteTools
{
   public static int toInt(byte unsignedByte)
   {
      return (unsignedByte & 0xFF);
   }
   
   public static byte fromInt(int value)
   {
      return (byte) value;
   }
}
