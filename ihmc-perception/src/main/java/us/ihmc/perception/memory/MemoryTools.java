package us.ihmc.perception.memory;

import org.bytedeco.javacpp.BytePointer;

public class MemoryTools
{
   /**
    * Uses Bytedeco native bindings to call a C memcpy, which should be as fast as possible.
    * Make sure the positions and limits of the BytePointers are set appropriately.
    */
   public static void memoryCopy(BytePointer from, BytePointer to)
   {
      from.put(to);
   }
}
