package us.ihmc.perception.memory;

import org.bytedeco.javacpp.BytePointer;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;

import java.nio.ByteBuffer;

public class NativeMemoryToolsTest
{
   @Test
   public void testNativeCopy()
   {
      ByteBuffer buffer1 = NativeMemoryTools.allocate(5);
      ByteBuffer buffer2 = NativeMemoryTools.allocate(5);

      String testString = "Hello";
      buffer1.put(testString.getBytes());

      LogTools.info("Buffer 1 position: {}", buffer1.position());
      Assertions.assertEquals(5, buffer1.position());
      buffer1.flip();
      Assertions.assertEquals(0, buffer1.position());
      Assertions.assertEquals(5, buffer1.limit());

      BytePointer buffer1Pointer = new BytePointer(buffer1);
      BytePointer buffer2Pointer = new BytePointer(buffer2);

      NativeMemoryTools.copy(buffer1Pointer, buffer2Pointer);

      LogTools.info("Buffer 2 position: {}", buffer2.position());
      LogTools.info("Buffer 2 pointer position: {}", buffer2Pointer.position());

      Assertions.assertEquals(0, buffer1.position());
      Assertions.assertEquals(0, buffer1Pointer.position());
      Assertions.assertEquals(0, buffer2.position());
      Assertions.assertEquals(0, buffer2Pointer.position());

      byte[] heapArray = new byte[5];
      buffer2.get(heapArray);
      String copiedString = new String(heapArray);
      LogTools.info("Buffer 2 contents: {}", copiedString);
      Assertions.assertEquals("Hello", copiedString);
   }
}
