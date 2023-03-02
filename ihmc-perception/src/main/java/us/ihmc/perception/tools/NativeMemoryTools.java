package us.ihmc.perception.tools;

import org.bytedeco.javacpp.BytePointer;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * A tools class for native memory operations that are more than a one liner
 * or not super obvious.
 *
 * IHMC Commons already has a class called MemoryTools, so we specify "Native" here.
 */
public class NativeMemoryTools
{
   /**
    * Uses Bytedeco native bindings to call a C memcpy, which should be as fast as possible.
    * Make sure the positions and limits of the BytePointers are set appropriately.
    */
   public static void copy(BytePointer from, BytePointer to)
   {
      to.put(from);
   }

   /**
    * A native buffer is direct, meaning it's available to native code,
    * is outside of the normal garbage-collected heap, and is subject to
    * native memory I/O operations.
    *
    * In this method we also set the byte order to match the system's native
    * byte order, which is not guaranteed otherwise. This is necessary to
    * integrate with native libraries which also assume system byte order.
    */
   public static ByteBuffer allocate(int capacity)
   {
      ByteBuffer nativeBuffer = ByteBuffer.allocateDirect(capacity);
      nativeBuffer.order(ByteOrder.nativeOrder());
      return nativeBuffer;
   }
}
