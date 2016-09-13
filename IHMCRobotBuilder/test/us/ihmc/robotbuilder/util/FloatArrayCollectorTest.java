package us.ihmc.robotbuilder.util;

import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;
import java.util.stream.IntStream;

import static org.junit.Assert.*;

public class FloatArrayCollectorTest
{
   @Test
   public void testCollectZeroItemsToAZeroLengthArray()
   {
      float[] collectedArrayOfFloats = Collections.<Double> emptyList().stream().collect(new FloatArrayCollector<Double>());
      assertEquals(0, collectedArrayOfFloats.length);
   }

   @Test
   public void testCollectOneItem()
   {
      float[] collectedArrayOfFloats = Collections.singletonList(1.0).stream().collect(new FloatArrayCollector<Double>());
      assertEquals(1, collectedArrayOfFloats.length);
      assertEquals(1.0f, collectedArrayOfFloats[0], 1e-5);
   }

   @Test
   public void testCollectManyItems()
   {
      float[] collectedArrayOfFloats = Arrays.stream(new double[8192]).mapToObj(Double::new).collect(new FloatArrayCollector<Double>());
      assertEquals(collectedArrayOfFloats.length, 8192);
      float sum = 0;
      for (float floatValue : collectedArrayOfFloats)
         sum += floatValue;
      assertEquals(0, sum, 1e-5);
   }

   @Test
   public void testCollectParallel()
   {
      final int n = 8192;
      float[] collectedArrayOfFloats = IntStream.iterate(1, i -> i + 1).limit(n).mapToObj(Double::new).parallel().collect(new FloatArrayCollector<Double>());
      assertEquals(collectedArrayOfFloats.length, n);
      double sum = 0;
      for (float floatValue : collectedArrayOfFloats)
         sum += floatValue;
      assertEquals(n * (n + 1) / 2, sum, 1e-5);
   }
}
