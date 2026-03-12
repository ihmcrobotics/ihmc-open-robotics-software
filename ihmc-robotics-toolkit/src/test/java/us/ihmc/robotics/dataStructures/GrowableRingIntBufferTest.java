package us.ihmc.robotics.dataStructures;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;

import static org.junit.jupiter.api.Assertions.*;

public class GrowableRingIntBufferTest
{
   private static final double epsilon = 1e-8;

   @Test
   public void testResizing()
   {
      // first we'll start small, and fill the buffer
      GrowableRingIntBuffer buffer = new GrowableRingIntBuffer(3);

      assertEquals(0, buffer.getCurrentSize());

      buffer.add(0);
      buffer.add(1);

      assertEquals(2, buffer.getCurrentSize());
      assertEquals(3, buffer.getCapacity());

      assertEquals(0, buffer.get(0));
      assertEquals(0, buffer.getStart());
      assertEquals(1, buffer.get(1));
      assertEquals(1, buffer.getEnd());

      // Reach the end of the buffer, and check again
      buffer.add(2);

      assertEquals(3, buffer.getCurrentSize());
      assertEquals(3, buffer.getCapacity());

      assertEquals(0, buffer.get(0));
      assertEquals(0, buffer.getStart());
      assertEquals(1, buffer.get(1));
      assertEquals(2, buffer.get(2));
      assertEquals(2, buffer.getEnd());


      // Wrap around the buffer, and check again
      buffer.add(3);

      assertEquals(3, buffer.getCurrentSize());
      assertEquals(3, buffer.getCapacity());

      assertEquals(1, buffer.get(0));
      assertEquals(1, buffer.getStart());
      assertEquals(2, buffer.get(1));
      assertEquals(3, buffer.get(2));
      assertEquals(3, buffer.getEnd());


      // Increase the size to 5, and check again
      buffer.resize(5);

      assertEquals(3, buffer.getCurrentSize());
      assertEquals(5, buffer.getCapacity());

      assertEquals(1, buffer.get(0));
      assertEquals(1, buffer.getStart());
      assertEquals(2, buffer.get(1));
      assertEquals(3, buffer.get(2));
      assertEquals(3, buffer.getEnd());

      buffer.add(4);
      buffer.add(5);

      assertEquals(5, buffer.getCurrentSize());
      assertEquals(5, buffer.getCapacity());

      assertEquals(1, buffer.get(0));
      assertEquals(1, buffer.getStart());
      assertEquals(2, buffer.get(1));
      assertEquals(3, buffer.get(2));
      assertEquals(4, buffer.get(3));
      assertEquals(5, buffer.get(4));
      assertEquals(5, buffer.getEnd());

      // Wrap around the buffer a little more
      buffer.add(0);
      buffer.add(1);

      assertEquals(5, buffer.getCurrentSize());
      assertEquals(5, buffer.getCapacity());

      assertEquals(3, buffer.get(0));
      assertEquals(3, buffer.getStart());
      assertEquals(4, buffer.get(1));
      assertEquals(5, buffer.get(2));
      assertEquals(0, buffer.get(3));
      assertEquals(1, buffer.get(4));
      assertEquals(1, buffer.getEnd());

      // Shrink the buffer to three and test
      buffer.resize(3);

      assertEquals(3, buffer.getCurrentSize());
      assertEquals(3, buffer.getCapacity());

      assertEquals(5, buffer.get(0));
      assertEquals(5, buffer.getStart());
      assertEquals(0, buffer.get(1));
      assertEquals(1, buffer.get(2));
      assertEquals(1, buffer.getEnd());

      // Add and wrap around
      buffer.add(2);
      buffer.add(3);

      assertEquals(3, buffer.getCurrentSize());
      assertEquals(3, buffer.getCapacity());

      assertEquals(1, buffer.get(0));
      assertEquals(1, buffer.getStart());
      assertEquals(2, buffer.get(1));
      assertEquals(3, buffer.get(2));
      assertEquals(3, buffer.getEnd());

      // Grow the buffer a bit
      buffer.resize(4);

      assertEquals(3, buffer.getCurrentSize());
      assertEquals(4, buffer.getCapacity());

      assertEquals(1, buffer.get(0));
      assertEquals(1, buffer.getStart());
      assertEquals(2, buffer.get(1));
      assertEquals(3, buffer.get(2));
      assertEquals(3, buffer.getEnd());

      buffer.add(4);

      assertEquals(4, buffer.getCurrentSize());
      assertEquals(4, buffer.getCapacity());

      assertEquals(1, buffer.get(0));
      assertEquals(1, buffer.getStart());
      assertEquals(2, buffer.get(1));
      assertEquals(3, buffer.get(2));
      assertEquals(4, buffer.get(3));
      assertEquals(4, buffer.getEnd());

      // Grow the buffer a bit more
      buffer.resize(5);

      assertEquals(4, buffer.getCurrentSize());
      assertEquals(5, buffer.getCapacity());

      assertEquals(1, buffer.get(0));
      assertEquals(1, buffer.getStart());
      assertEquals(2, buffer.get(1));
      assertEquals(3, buffer.get(2));
      assertEquals(4, buffer.get(3));
      assertEquals(4, buffer.getEnd());

      buffer.add(5);

      assertEquals(5, buffer.getCurrentSize());
      assertEquals(5, buffer.getCapacity());

      assertEquals(1, buffer.get(0));
      assertEquals(1, buffer.getStart());
      assertEquals(2, buffer.get(1));
      assertEquals(3, buffer.get(2));
      assertEquals(4, buffer.get(3));
      assertEquals(5, buffer.get(4));
      assertEquals(5, buffer.getEnd());

      // Shrink back to 2
      buffer.resize(2);

      assertEquals(2, buffer.getCurrentSize());
      assertEquals(2, buffer.getCapacity());

      assertEquals(4, buffer.get(0));
      assertEquals(4, buffer.getStart());
      assertEquals(5, buffer.get(1));
      assertEquals(5, buffer.getEnd());

      // resize it big
      buffer.resize(6);
      buffer.add(0);

      assertEquals(3, buffer.getCurrentSize());
      assertEquals(6, buffer.getCapacity());

      assertEquals(4, buffer.get(0));
      assertEquals(4, buffer.getStart());
      assertEquals(5, buffer.get(1));
      assertEquals(0, buffer.get(2));
      assertEquals(0, buffer.getEnd());

      // Shrink back to larger than the current size
      buffer.resize(4);

      assertEquals(3, buffer.getCurrentSize());
      assertEquals(4, buffer.getCapacity());

      assertEquals(4, buffer.get(0));
      assertEquals(4, buffer.getStart());
      assertEquals(5, buffer.get(1));
      assertEquals(0, buffer.get(2));
      assertEquals(0, buffer.getEnd());

      // TODO grow it beyond, add something less than capacity, then shrink to that size.
      buffer.resize(6);

      buffer.add(1);
      buffer.add(2);

      assertEquals(5, buffer.getCurrentSize());
      assertEquals(6, buffer.getCapacity());

      assertEquals(4, buffer.get(0));
      assertEquals(4, buffer.getStart());
      assertEquals(5, buffer.get(1));
      assertEquals(0, buffer.get(2));
      assertEquals(1, buffer.get(3));
      assertEquals(2, buffer.get(4));
      assertEquals(2, buffer.getEnd());

      buffer.resize(5);

      assertEquals(5, buffer.getCurrentSize());
      assertEquals(5, buffer.getCapacity());

      assertEquals(4, buffer.get(0));
      assertEquals(4, buffer.getStart());
      assertEquals(5, buffer.get(1));
      assertEquals(0, buffer.get(2));
      assertEquals(1, buffer.get(3));
      assertEquals(2, buffer.get(4));
      assertEquals(2, buffer.getEnd());
   }
}
