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

   @Test
   public void testIsZero() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double x = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double y = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double z = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double epsilon = RandomNumbers.nextDouble(random, 0.0, 10.0);

         boolean isTuple2dZero = x < epsilon && y < epsilon;
         boolean isTuple3dZero = x < epsilon && y < epsilon && z < epsilon;

         assertEquals(isTuple2dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point2D(x, y), epsilon));
         assertEquals(isTuple2dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point2D(-x, y), epsilon));
         assertEquals(isTuple2dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point2D(-x, -y), epsilon));
         assertEquals(isTuple2dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point2D(x, -y), epsilon));
         assertEquals(isTuple2dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point2D(x, y), epsilon));
         assertEquals(isTuple2dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point2D(-x, y), epsilon));
         assertEquals(isTuple2dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point2D(-x, -y), epsilon));
         assertEquals(isTuple2dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point2D(x, -y), epsilon));

         assertEquals(isTuple3dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point3D(x, y, z), epsilon));
         assertEquals(isTuple3dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point3D(x, y, -z), epsilon));
         assertEquals(isTuple3dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point3D(x, -y, z), epsilon));
         assertEquals(isTuple3dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point3D(x, -y, -z), epsilon));
         assertEquals(isTuple3dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point3D(-x, y, z), epsilon));
         assertEquals(isTuple3dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point3D(-x, y, -z), epsilon));
         assertEquals(isTuple3dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point3D(-x, -y, z), epsilon));
         assertEquals(isTuple3dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point3D(-x, -y, -z), epsilon));
         assertEquals(isTuple3dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point3D(x, y, z), epsilon));
         assertEquals(isTuple3dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point3D(x, y, -z), epsilon));
         assertEquals(isTuple3dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point3D(x, -y, z), epsilon));
         assertEquals(isTuple3dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point3D(x, -y, -z), epsilon));
         assertEquals(isTuple3dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point3D(-x, y, z), epsilon));
         assertEquals(isTuple3dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point3D(-x, y, -z), epsilon));
         assertEquals(isTuple3dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point3D(-x, -y, z), epsilon));
         assertEquals(isTuple3dZero, us.ihmc.euclid.tools.EuclidCoreMissingTools.isZero(new Point3D(-x, -y, -z), epsilon));
      }
   }
}
