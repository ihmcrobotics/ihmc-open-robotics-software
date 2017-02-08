package us.ihmc.quadrupedRobotics.util;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;

public class TimeIntervalToolsTest
{

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSortMethods()
   {
      double epsilon = 1e-6;

      int size = 10;
      ArrayList<TimedValue> arrayValues = new ArrayList<>(size);
      PreallocatedList<TimedValue> preallocatedValues = new PreallocatedList<>(size, TimedValue.class);

      for (int i = 0; i < size; i++)
      {
         TimedValue tv = new TimedValue(i, new TimeInterval(i, i + 1));
         arrayValues.add(tv);
         preallocatedValues.add();
         preallocatedValues.get(i).set(tv);
      }

      TimeIntervalTools.sortByReverseStartTime(arrayValues);
      TimeIntervalTools.sortByReverseStartTime(preallocatedValues);
      for (int i = 0; i < size; i++)
      {
         assertEquals(arrayValues.get(i).getValue(), size - 1 - i, epsilon);
         assertEquals(preallocatedValues.get(i).getValue(), size - 1 - i, epsilon);
      }

      TimeIntervalTools.sortByStartTime(arrayValues);
      TimeIntervalTools.sortByStartTime(preallocatedValues);
      for (int i = 0; i < size; i++)
      {
         assertEquals(arrayValues.get(i).getValue(), i, epsilon);
         assertEquals(preallocatedValues.get(i).getValue(), i, epsilon);
      }

      TimeIntervalTools.sortByReverseEndTime(arrayValues);
      TimeIntervalTools.sortByReverseEndTime(preallocatedValues);
      for (int i = 0; i < size; i++)
      {
         assertEquals(arrayValues.get(i).getValue(), size - 1 - i, epsilon);
         assertEquals(preallocatedValues.get(i).getValue(), size - 1 - i, epsilon);
      }


      TimeIntervalTools.sortByEndTime(arrayValues);
      TimeIntervalTools.sortByEndTime(preallocatedValues);
      for (int i = 0; i < arrayValues.size(); i++)
      {
         assertEquals(arrayValues.get(i).getValue(), i, epsilon);
         assertEquals(preallocatedValues.get(i).getValue(), i, epsilon);
      }
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRemoveMethods()
   {
      double epsilon = 1e-6;

      int size = 10;
      ArrayList<TimedValue> arrayValues = new ArrayList<>(size);
      PreallocatedList<TimedValue> preallocatedValues = new PreallocatedList<>(size, TimedValue.class);

      for (int i = 0; i < size; i++)
      {
         TimedValue tv = new TimedValue(i, new TimeInterval(i, i + 1));
         arrayValues.add(tv);
         preallocatedValues.add();
         preallocatedValues.get(i).set(tv);
      }

      TimeIntervalTools.removeEndTimesGreaterThan(8.5, arrayValues);
      TimeIntervalTools.removeEndTimesGreaterThan(8.5, preallocatedValues);
      size = 8;
      assertEquals(arrayValues.size(), size);
      assertEquals(preallocatedValues.size(), size);
      for (int i = 0; i < size; i++)
      {
         assertEquals(arrayValues.get(i).getValue(), i, epsilon);
         assertEquals(preallocatedValues.get(i).getValue(), i, epsilon);
      }

      TimeIntervalTools.removeStartTimesGreaterThan(6.5, arrayValues);
      TimeIntervalTools.removeStartTimesGreaterThan(6.5, preallocatedValues);
      size = 7;
      assertEquals(arrayValues.size(), size);
      assertEquals(preallocatedValues.size(), size);
      for (int i = 0; i < size; i++)
      {
         assertEquals(arrayValues.get(i).getValue(), i, epsilon);
         assertEquals(preallocatedValues.get(i).getValue(), i, epsilon);
      }

      TimeIntervalTools.removeStartTimesLessThan(1.5, arrayValues);
      TimeIntervalTools.removeStartTimesLessThan(1.5, preallocatedValues);
      size = 5;
      assertEquals(arrayValues.size(), size);
      assertEquals(preallocatedValues.size(), size);
      for (int i = 0; i < size; i++)
      {
         assertEquals(arrayValues.get(i).getValue(), i + 2, epsilon);
         assertEquals(preallocatedValues.get(i).getValue(), i + 2, epsilon);
      }

      TimeIntervalTools.removeEndTimesLessThan(6.5, arrayValues);
      TimeIntervalTools.removeEndTimesLessThan(6.5, preallocatedValues);
      assertEquals(arrayValues.size(), 1);
      assertEquals(preallocatedValues.size(), 1);
      assertEquals(arrayValues.get(0).getValue(), 6, epsilon);
      assertEquals(preallocatedValues.get(0).getValue(), 6, epsilon);
   }
}
