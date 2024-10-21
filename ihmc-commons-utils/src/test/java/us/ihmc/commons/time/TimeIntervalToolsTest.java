package us.ihmc.commons.time;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

public class TimeIntervalToolsTest
{
   @Test
   public void testSortMethods()
   {
      double epsilon = 1e-6;

      int size = 10;
      ArrayList<TimedValue> arrayValues = new ArrayList<>(size);

      for (int i = 0; i < size; i++)
      {
         TimedValue tv = new TimedValue(i, new TimeInterval(i, i + 1));
         arrayValues.add(tv);
      }

      TimeIntervalTools.sortByReverseStartTime(arrayValues);
      for (int i = 0; i < size; i++)
      {
         assertEquals(arrayValues.get(i).getValue(), size - 1 - i, epsilon);
      }

      TimeIntervalTools.sortByStartTime(arrayValues);
      for (int i = 0; i < size; i++)
      {
         assertEquals(arrayValues.get(i).getValue(), i, epsilon);
      }

      TimeIntervalTools.sortByReverseEndTime(arrayValues);
      for (int i = 0; i < size; i++)
      {
         assertEquals(arrayValues.get(i).getValue(), size - 1 - i, epsilon);
      }


      TimeIntervalTools.sortByEndTime(arrayValues);
      for (int i = 0; i < arrayValues.size(); i++)
      {
         assertEquals(arrayValues.get(i).getValue(), i, epsilon);
      }
   }

   @Test
   public void testRemoveMethods()
   {
      double epsilon = 1e-6;

      int size = 10;
      ArrayList<TimedValue> arrayValues = new ArrayList<>(size);

      for (int i = 0; i < size; i++)
      {
         TimedValue tv = new TimedValue(i, new TimeInterval(i, i + 1));
         arrayValues.add(tv);
      }

      TimeIntervalTools.removeEndTimesGreaterThan(8.5, arrayValues);
      size = 8;
      assertEquals(arrayValues.size(), size);
      for (int i = 0; i < size; i++)
      {
         assertEquals(arrayValues.get(i).getValue(), i, epsilon);
      }

      TimeIntervalTools.removeStartTimesGreaterThan(6.5, arrayValues);
      size = 7;
      assertEquals(arrayValues.size(), size);
      for (int i = 0; i < size; i++)
      {
         assertEquals(arrayValues.get(i).getValue(), i, epsilon);
      }

      TimeIntervalTools.removeStartTimesLessThan(1.5, arrayValues);
      size = 5;
      assertEquals(arrayValues.size(), size);
      for (int i = 0; i < size; i++)
      {
         assertEquals(arrayValues.get(i).getValue(), i + 2, epsilon);
      }

      TimeIntervalTools.removeEndTimesLessThan(6.5, arrayValues);
      assertEquals(arrayValues.size(), 1);
      assertEquals(arrayValues.get(0).getValue(), 6, epsilon);
   }
}
