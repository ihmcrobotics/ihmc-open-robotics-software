package us.ihmc.robotics;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class EuclidCoreMissingToolsTest
{

   private static final int ITERATIONS = 1000;


   @Test
   public void testRoundToGivenPrecision()
   {
      double longDouble = 0.12345678910111213;

      double roundedNumber = MathTools.floorToPrecision(longDouble, 1e-7);
      assertEquals(roundedNumber, 0.1234567, 1e-14);

      roundedNumber = MathTools.floorToPrecision(longDouble, 1e-3);
      assertEquals(roundedNumber, 0.123, 1e-14);

      Vector3D preciseVector = new Vector3D(0.12345678910111213, 100.12345678910111213, 1000.12345678910111213);
      Vector3D roundedVector = new Vector3D(preciseVector);

      EuclidCoreMissingTools.floorToGivenPrecision(roundedVector, 1e-7);
      EuclidCoreTestTools.assertEquals(new Vector3D(0.1234567, 100.1234567, 1000.1234567), roundedVector, 1e-12);

      EuclidCoreMissingTools.floorToGivenPrecision(roundedVector, 1e-3);
      EuclidCoreTestTools.assertEquals(new Vector3D(0.123, 100.123, 1000.123), roundedVector, 1e-14);
   }
}
