package us.ihmc.quadrupedRobotics.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class YoPreallocatedListTest
{
   YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCapacity()
   {
      YoPreallocatedList<DoubleYoVariable> doubleList = new YoPreallocatedList<>("doubles", registry, 10,
            new YoPreallocatedList.DefaultElementFactory<DoubleYoVariable>()
            {
               @Override
               public DoubleYoVariable createDefaultElement(String prefix, YoVariableRegistry registry)
               {
                  return new DoubleYoVariable(prefix, registry);
               }
            });

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

      YoPreallocatedList<DoubleYoVariable> doubleList = new YoPreallocatedList<>("doubles", registry, 10,
            new YoPreallocatedList.DefaultElementFactory<DoubleYoVariable>()
            {
               @Override
               public DoubleYoVariable createDefaultElement(String prefix, YoVariableRegistry registry)
               {
                  return new DoubleYoVariable(prefix, registry);
               }
            });

      for (int i = 0; i < 10; i++)
      {
         doubleList.add();
         doubleList.get(i).set(i);
      }

      // front of list
      doubleList.remove(0);
      assertEquals(doubleList.size(), 9);
      assertEquals(doubleList.get(0).getDoubleValue(), 1.0, epsilon);
      assertEquals(doubleList.get(1).getDoubleValue(), 2.0, epsilon);
      doubleList.add(0);
      doubleList.get(0).set(0);
      assertEquals(doubleList.size(), 10);
      assertEquals(doubleList.get(0).getDoubleValue(), 0.0, epsilon);
      assertEquals(doubleList.get(1).getDoubleValue(), 1.0, epsilon);

      // first half
      doubleList.remove(3);
      assertEquals(doubleList.size(), 9);
      assertEquals(doubleList.get(2).getDoubleValue(), 2.0, epsilon);
      assertEquals(doubleList.get(3).getDoubleValue(), 4.0, epsilon);
      assertEquals(doubleList.get(4).getDoubleValue(), 5.0, epsilon);
      doubleList.add(3);
      doubleList.get(3).set(3);
      assertEquals(doubleList.size(), 10);
      assertEquals(doubleList.get(2).getDoubleValue(), 2.0, epsilon);
      assertEquals(doubleList.get(3).getDoubleValue(), 3.0, epsilon);
      assertEquals(doubleList.get(4).getDoubleValue(), 4.0, epsilon);

      // second half
      doubleList.remove(7);
      assertEquals(doubleList.size(), 9);
      assertEquals(doubleList.get(6).getDoubleValue(), 6.0, epsilon);
      assertEquals(doubleList.get(7).getDoubleValue(), 8.0, epsilon);
      assertEquals(doubleList.get(8).getDoubleValue(), 9.0, epsilon);
      doubleList.add(7);
      doubleList.get(7).set(7);
      assertEquals(doubleList.size(), 10);
      assertEquals(doubleList.get(6).getDoubleValue(), 6.0, epsilon);
      assertEquals(doubleList.get(7).getDoubleValue(), 7.0, epsilon);
      assertEquals(doubleList.get(8).getDoubleValue(), 8.0, epsilon);

      // end of list
      doubleList.remove(9);
      assertEquals(doubleList.size(), 9);
      assertEquals(doubleList.get(7).getDoubleValue(), 7.0, epsilon);
      assertEquals(doubleList.get(8).getDoubleValue(), 8.0, epsilon);
      doubleList.add(9);
      doubleList.get(9).set(9);
      assertEquals(doubleList.size(), 10);
      assertEquals(doubleList.get(8).getDoubleValue(), 8.0, epsilon);
      assertEquals(doubleList.get(9).getDoubleValue(), 9.0, epsilon);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPreallocatedListConstructor()
   {
      double epsilon = 0.001;

      ArrayList<DoubleYoVariable> elements = new ArrayList<>();
      for (int i = 0; i < 10; i++)
      {
         elements.add(new DoubleYoVariable("" + i, registry));
         elements.get(i).set(i);
      }
      YoPreallocatedList<DoubleYoVariable> doubleList = new YoPreallocatedList<>("doubles", registry, elements);

      // front of list
      doubleList.remove(0);
      assertEquals(doubleList.size(), 9);
      assertEquals(doubleList.get(0).getDoubleValue(), 1.0, epsilon);
      assertEquals(doubleList.get(1).getDoubleValue(), 2.0, epsilon);
      doubleList.add(0);
      doubleList.get(0).set(0);
      assertEquals(doubleList.size(), 10);
      assertEquals(doubleList.get(0).getDoubleValue(), 0.0, epsilon);
      assertEquals(doubleList.get(1).getDoubleValue(), 1.0, epsilon);

      // first half
      doubleList.remove(3);
      assertEquals(doubleList.size(), 9);
      assertEquals(doubleList.get(2).getDoubleValue(), 2.0, epsilon);
      assertEquals(doubleList.get(3).getDoubleValue(), 4.0, epsilon);
      assertEquals(doubleList.get(4).getDoubleValue(), 5.0, epsilon);
      doubleList.add(3);
      doubleList.get(3).set(3);
      assertEquals(doubleList.size(), 10);
      assertEquals(doubleList.get(2).getDoubleValue(), 2.0, epsilon);
      assertEquals(doubleList.get(3).getDoubleValue(), 3.0, epsilon);
      assertEquals(doubleList.get(4).getDoubleValue(), 4.0, epsilon);

      // second half
      doubleList.remove(7);
      assertEquals(doubleList.size(), 9);
      assertEquals(doubleList.get(6).getDoubleValue(), 6.0, epsilon);
      assertEquals(doubleList.get(7).getDoubleValue(), 8.0, epsilon);
      assertEquals(doubleList.get(8).getDoubleValue(), 9.0, epsilon);
      doubleList.add(7);
      doubleList.get(7).set(7);
      assertEquals(doubleList.size(), 10);
      assertEquals(doubleList.get(6).getDoubleValue(), 6.0, epsilon);
      assertEquals(doubleList.get(7).getDoubleValue(), 7.0, epsilon);
      assertEquals(doubleList.get(8).getDoubleValue(), 8.0, epsilon);

      // end of list
      doubleList.remove(9);
      assertEquals(doubleList.size(), 9);
      assertEquals(doubleList.get(7).getDoubleValue(), 7.0, epsilon);
      assertEquals(doubleList.get(8).getDoubleValue(), 8.0, epsilon);
      doubleList.add(9);
      doubleList.get(9).set(9);
      assertEquals(doubleList.size(), 10);
      assertEquals(doubleList.get(8).getDoubleValue(), 8.0, epsilon);
      assertEquals(doubleList.get(9).getDoubleValue(), 9.0, epsilon);
   }
}
