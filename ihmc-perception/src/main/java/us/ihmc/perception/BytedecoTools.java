package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;

public class BytedecoTools
{
   public static String stringFromByteBuffer(BytePointer bytePointerWithString)
   {
      return bytePointerWithString.getString().trim();
   }
}
