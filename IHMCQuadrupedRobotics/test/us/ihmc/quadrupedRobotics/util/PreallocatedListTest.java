package us.ihmc.quadrupedRobotics.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;

public class PreallocatedListTest
{
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCapacity()
   {
      PreallocatedList<MutableDouble> doubleList = new PreallocatedList<>(10, MutableDouble.class);
      for (int i = 0; i < 10; i++)
      {
         assertTrue(doubleList.add());
      }
      assertFalse(doubleList.add());
      assertEquals(doubleList.size(), 10);
      assertEquals(doubleList.capacity(), 10);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDefaultElementFactory()
   {
      double epsilon = 0.001;

      PreallocatedList<MutableDouble> doubleList = new PreallocatedList<>(10, new PreallocatedList.DefaultElementFactory<MutableDouble>()
      {
         @Override
         public MutableDouble createDefaultElement()
         {
            return new MutableDouble(0);
         }
      });

      for (int i = 0; i < 10; i++)
      {
         doubleList.add();
         doubleList.get(i).setValue(i);
      }

      // front of list
      doubleList.remove(0);
      assertEquals(doubleList.size(), 9);
      assertEquals(doubleList.get(0).doubleValue(), 1.0, epsilon);
      assertEquals(doubleList.get(1).doubleValue(), 2.0, epsilon);
      doubleList.add(0);
      doubleList.get(0).setValue(0);
      assertEquals(doubleList.size(), 10);
      assertEquals(doubleList.get(0).doubleValue(), 0.0, epsilon);
      assertEquals(doubleList.get(1).doubleValue(), 1.0, epsilon);

      // first half
      doubleList.remove(3);
      assertEquals(doubleList.size(), 9);
      assertEquals(doubleList.get(2).doubleValue(), 2.0, epsilon);
      assertEquals(doubleList.get(3).doubleValue(), 4.0, epsilon);
      assertEquals(doubleList.get(4).doubleValue(), 5.0, epsilon);
      doubleList.add(3);
      doubleList.get(3).setValue(3);
      assertEquals(doubleList.size(), 10);
      assertEquals(doubleList.get(2).doubleValue(), 2.0, epsilon);
      assertEquals(doubleList.get(3).doubleValue(), 3.0, epsilon);
      assertEquals(doubleList.get(4).doubleValue(), 4.0, epsilon);

      // second half
      doubleList.remove(7);
      assertEquals(doubleList.size(), 9);
      assertEquals(doubleList.get(6).doubleValue(), 6.0, epsilon);
      assertEquals(doubleList.get(7).doubleValue(), 8.0, epsilon);
      assertEquals(doubleList.get(8).doubleValue(), 9.0, epsilon);
      doubleList.add(7);
      doubleList.get(7).setValue(7);
      assertEquals(doubleList.size(), 10);
      assertEquals(doubleList.get(6).doubleValue(), 6.0, epsilon);
      assertEquals(doubleList.get(7).doubleValue(), 7.0, epsilon);
      assertEquals(doubleList.get(8).doubleValue(), 8.0, epsilon);

      // end of list
      doubleList.remove(9);
      assertEquals(doubleList.size(), 9);
      assertEquals(doubleList.get(7).doubleValue(), 7.0, epsilon);
      assertEquals(doubleList.get(8).doubleValue(), 8.0, epsilon);
      doubleList.add(9);
      doubleList.get(9).setValue(9);
      assertEquals(doubleList.size(), 10);
      assertEquals(doubleList.get(8).doubleValue(), 8.0, epsilon);
      assertEquals(doubleList.get(9).doubleValue(), 9.0, epsilon);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPreallocatedListConstructor()
   {
      double epsilon = 0.001;

      ArrayList<MutableDouble> elements = new ArrayList<>();
      for (int i = 0; i < 10; i++)
      {
         elements.add(new MutableDouble(i));
      }
      PreallocatedList<MutableDouble> doubleList = new PreallocatedList<>(elements);

      // front of list
      doubleList.remove(0);
      assertEquals(doubleList.size(), 9);
      assertEquals(doubleList.get(0).doubleValue(), 1.0, epsilon);
      assertEquals(doubleList.get(1).doubleValue(), 2.0, epsilon);
      doubleList.add(0);
      doubleList.get(0).setValue(0);
      assertEquals(doubleList.size(), 10);
      assertEquals(doubleList.get(0).doubleValue(), 0.0, epsilon);
      assertEquals(doubleList.get(1).doubleValue(), 1.0, epsilon);

      // first half
      doubleList.remove(3);
      assertEquals(doubleList.size(), 9);
      assertEquals(doubleList.get(2).doubleValue(), 2.0, epsilon);
      assertEquals(doubleList.get(3).doubleValue(), 4.0, epsilon);
      assertEquals(doubleList.get(4).doubleValue(), 5.0, epsilon);
      doubleList.add(3);
      doubleList.get(3).setValue(3);
      assertEquals(doubleList.size(), 10);
      assertEquals(doubleList.get(2).doubleValue(), 2.0, epsilon);
      assertEquals(doubleList.get(3).doubleValue(), 3.0, epsilon);
      assertEquals(doubleList.get(4).doubleValue(), 4.0, epsilon);

      // second half
      doubleList.remove(7);
      assertEquals(doubleList.size(), 9);
      assertEquals(doubleList.get(6).doubleValue(), 6.0, epsilon);
      assertEquals(doubleList.get(7).doubleValue(), 8.0, epsilon);
      assertEquals(doubleList.get(8).doubleValue(), 9.0, epsilon);
      doubleList.add(7);
      doubleList.get(7).setValue(7);
      assertEquals(doubleList.size(), 10);
      assertEquals(doubleList.get(6).doubleValue(), 6.0, epsilon);
      assertEquals(doubleList.get(7).doubleValue(), 7.0, epsilon);
      assertEquals(doubleList.get(8).doubleValue(), 8.0, epsilon);

      // end of list
      doubleList.remove(9);
      assertEquals(doubleList.size(), 9);
      assertEquals(doubleList.get(7).doubleValue(), 7.0, epsilon);
      assertEquals(doubleList.get(8).doubleValue(), 8.0, epsilon);
      doubleList.add(9);
      doubleList.get(9).setValue(9);
      assertEquals(doubleList.size(), 10);
      assertEquals(doubleList.get(8).doubleValue(), 8.0, epsilon);
      assertEquals(doubleList.get(9).doubleValue(), 9.0, epsilon);
   }
}
