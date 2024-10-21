package us.ihmc.commons;

import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class DeadbandToolsTest
{
   private static final double epsilon = 1e-9;
   private static final int iters = 100;

   @Test
   public void testScalar()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         double value = RandomNumbers.nextDouble(random, 0.0, 100.0);
         double noPassDeadband = RandomNumbers.nextDouble(random, Math.abs(value) + 10.0 * epsilon, 100);
         double somePassDeadband = RandomNumbers.nextDouble(random, Math.abs(value) - 10.0 * epsilon);

         double offset = RandomNumbers.nextDouble(random, 100.0);

         assertEquals(0.0, DeadbandTools.applyDeadband(noPassDeadband, value), epsilon, "iter = " + iter);
         assertEquals(0.0, DeadbandTools.applyDeadband(noPassDeadband, -value), epsilon, "iter = " + iter);

         assertEquals(offset, DeadbandTools.applyDeadband(noPassDeadband, offset, offset + value), "iter = " + iter);
         assertEquals(offset, DeadbandTools.applyDeadband(noPassDeadband, offset, offset - value), "iter = " + iter);

         assertEquals(value - somePassDeadband,
                      DeadbandTools.applyDeadband(somePassDeadband, value),
                      epsilon,
                      "Deadband = " + somePassDeadband + ", value = " + value);
         assertEquals(-value + somePassDeadband,
                      DeadbandTools.applyDeadband(somePassDeadband, -value),
                      epsilon,
                      "Deadband = " + somePassDeadband + ", value = " + (-value));

         assertEquals(offset + value - somePassDeadband,
                      DeadbandTools.applyDeadband(somePassDeadband, offset, offset + value),
                      epsilon,
                      "Center = " + offset + ", deadband = " + somePassDeadband + ", value = " + (offset + value));
         assertEquals(offset - value + somePassDeadband,
                      DeadbandTools.applyDeadband(somePassDeadband, offset, offset - value),
                      epsilon,
                      "Center = " + offset + ", deadband = " + somePassDeadband + ", value = " + (offset - value));
      }
   }
}
