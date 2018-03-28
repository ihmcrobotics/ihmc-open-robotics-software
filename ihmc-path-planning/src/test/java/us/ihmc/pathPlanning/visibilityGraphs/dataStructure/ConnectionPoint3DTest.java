package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import org.apache.commons.math3.util.Precision;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class ConnectionPoint3DTest
{
   private static final int ITERATIONS = 10000;
   private static final double EPSILON = 1.0e-12;

   @ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 1000)
   public void testRound() throws Exception
   {
      Random random = new Random(43566787);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double raw = EuclidCoreRandomTools.nextDouble(random, 1000.0);
         double expected = Precision.round(raw, 4);
         double actual = ConnectionPoint3D.round(raw);
         assertEquals(expected, actual, EPSILON);
      }
   }
}
