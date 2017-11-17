package us.ihmc.tools;

import org.junit.Test;
import us.ihmc.tools.FloatArrayCollector;

import java.util.Arrays;
import java.util.Collections;
import java.util.stream.IntStream;

import static org.junit.Assert.*;

public class FloatArrayCollectorTest
{
   @Test(timeout = 30000)
   public void testCollectZeroItemsToAZeroLengthArray()
   {
      float[] collectedArrayOfFloats = Collections.<Double> emptyList().stream().collect(FloatArrayCollector.create());
      assertEquals(0, collectedArrayOfFloats.length);
   }

   @Test(timeout = 30000)
   public void testCollectOneItem()
   {
      float[] collectedArrayOfFloats = Collections.singletonList(1.0).stream().collect(FloatArrayCollector.create());
      assertEquals(1, collectedArrayOfFloats.length);
      assertEquals(1.0f, collectedArrayOfFloats[0], 1e-5);
   }

   @Test(timeout = 30000)
   public void testCollectManyItems()
   {
      float[] collectedArrayOfFloats = Arrays.stream(new double[8192]).mapToObj(Double::new).collect(FloatArrayCollector.create());
      assertEquals(collectedArrayOfFloats.length, 8192);
      float sum = 0;
      for (float floatValue : collectedArrayOfFloats)
         sum += floatValue;
      assertEquals(0, sum, 1e-5);
   }

   @Test(timeout = 30000)
   public void testCollectParallel()
   {
      final int n = 8192;
      float[] collectedArrayOfFloats = IntStream.range(1, n + 1)
                                                  .mapToObj(Double::new)
                                                  .parallel()
                                                  .collect(FloatArrayCollector.create());
      assertEquals(collectedArrayOfFloats.length, n);
      double sum = 0;
      for (float floatValue : collectedArrayOfFloats)
         sum += floatValue;
      assertEquals(n * (n + 1) / 2, sum, 1e-5);
   }
}
