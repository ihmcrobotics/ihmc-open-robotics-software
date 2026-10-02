package us.ihmc.stateEstimation.invariantEstimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.lang.management.ManagementFactory;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple2D.Vector2D;

public class FootReferencedYawCorrectorAngleTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testMatchesWrappedYawDifference()
   {
      Random random = new Random(1738L);
      Vector2D start = new Vector2D();
      Vector2D end = new Vector2D();
      for (int i = 0; i < 10_000; i++)
      {
         double startYaw = (random.nextDouble() * 2.0 - 1.0) * 10.0;
         double endYaw = (random.nextDouble() * 2.0 - 1.0) * 10.0;
         double scale = 0.1 + random.nextDouble() * 5.0; // the norms must not matter
         start.set(Math.cos(startYaw), Math.sin(startYaw));
         end.set(scale * Math.cos(endYaw), scale * Math.sin(endYaw));

         double expected = Math.IEEEremainder(endYaw - startYaw, 2.0 * Math.PI);
         double actual = FootReferencedYawCorrector.signedAngleFromTo(start, end);
         assertEquals(0.0, Math.IEEEremainder(actual - expected, 2.0 * Math.PI), 1.0e-9);
         assertTrue(actual > -Math.PI && actual <= Math.PI, "angle out of (-pi, pi]: " + actual);
      }
   }

   @Test
   public void testParallelAntiparallelAndZero()
   {
      Random random = new Random(42L);
      Vector2D start = new Vector2D();
      Vector2D end = new Vector2D();
      for (int i = 0; i < 1_000; i++)
      {
         double yaw = (random.nextDouble() * 2.0 - 1.0) * Math.PI;
         start.set(Math.cos(yaw), Math.sin(yaw));
         end.set(start);
         assertEquals(0.0, FootReferencedYawCorrector.signedAngleFromTo(start, end), EPSILON);
         end.scale(-1.0);
         assertEquals(Math.PI, FootReferencedYawCorrector.signedAngleFromTo(start, end), EPSILON);
      }
      assertEquals(0.0, FootReferencedYawCorrector.signedAngleFromTo(new Vector2D(), new Vector2D(1.0, 0.0)), EPSILON);
   }

   @Test
   public void testAllocationFree()
   {
      com.sun.management.ThreadMXBean threadMXBean = (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();
      assumeTrue(threadMXBean.isThreadAllocatedMemorySupported(), "Thread allocation counting not supported on this JVM.");
      threadMXBean.setThreadAllocatedMemoryEnabled(true);

      Vector2D start = new Vector2D(1.0, 0.2);
      Vector2D end = new Vector2D(-0.3, 0.9);
      double sink = 0.0;
      for (int i = 0; i < 50_000; i++)
         sink += FootReferencedYawCorrector.signedAngleFromTo(start, end);

      long before = threadMXBean.getCurrentThreadAllocatedBytes();
      for (int i = 0; i < 200_000; i++)
      {
         start.setX(1.0 + 1.0e-9 * i);
         sink += FootReferencedYawCorrector.signedAngleFromTo(start, end);
      }
      long after = threadMXBean.getCurrentThreadAllocatedBytes();
      assertTrue(Double.isFinite(sink));
      assertTrue(after - before < 16 * 200_000 / 1000, "signedAngleFromTo allocated " + (after - before) + " bytes over 200k calls");
   }
}
