package us.ihmc.robotics;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;

public class EuclidCoreMissingToolsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRoundToGivenPrecision()
   {
      double longDouble = 0.12345678910111213;

      double roundedNumber = MathTools.floorToGivenPrecision(longDouble, 1e-7);
      assertEquals(roundedNumber, 0.1234567, 1e-14);

      roundedNumber = MathTools.floorToGivenPrecision(longDouble, 1e-3);
      assertEquals(roundedNumber, 0.123, 1e-14);

      Vector3D preciseVector = new Vector3D(0.12345678910111213, 100.12345678910111213, 1000.12345678910111213);
      Vector3D roundedVector = new Vector3D(preciseVector);

      EuclidCoreMissingTools.floorToGivenPrecision(roundedVector, 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0.1234567, 100.1234567, 1000.1234567), roundedVector, 1e-12);

      EuclidCoreMissingTools.floorToGivenPrecision(roundedVector, 1e-3);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0.123, 100.123, 1000.123), roundedVector, 1e-14);
   }
}
