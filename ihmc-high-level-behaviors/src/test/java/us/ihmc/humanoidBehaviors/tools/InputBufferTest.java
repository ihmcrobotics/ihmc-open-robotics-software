package us.ihmc.humanoidBehaviors.tools;

import org.junit.jupiter.api.Test;
import us.ihmc.humanoidBehaviors.lookAndStep.InputBuffer;

import java.util.function.Supplier;

import static org.junit.jupiter.api.Assertions.*;

public class InputBufferTest
{
   @Test
   public void testInputBuffer()
   {
      InputBuffer<String> inputBuffer = new InputBuffer<>();

      assertThrows(RuntimeException.class, inputBuffer::store);
      assertThrows(RuntimeException.class, inputBuffer::getStored);
      assertThrows(RuntimeException.class, inputBuffer::getLatest);

      inputBuffer.accept("a");

      assertEquals("a", inputBuffer.getLatest());
      assertThrows(RuntimeException.class, inputBuffer::getStored);

      inputBuffer.store();

      assertEquals("a", inputBuffer.getStored());

      inputBuffer.accept("b");

      assertEquals("a", inputBuffer.getStored());
      assertEquals("b", inputBuffer.getLatest());

      inputBuffer.store();

      assertEquals("b", inputBuffer.getStored());
      assertEquals("b", inputBuffer.getLatest());
   }
   
   @Test
   public void test()
   {
      
   }

   private void performTask(String data)
   {

   }
}
