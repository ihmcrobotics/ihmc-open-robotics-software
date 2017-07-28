package us.ihmc.robotics.random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;

public class RandomGeometryTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testNextPoint3D32()
   {
      Random random = new Random();

      Point3D32 min = new Point3D32(-7.0f, -5.0f, -6.0f);
      Point3D32 max = new Point3D32(5.0f, 4.0f, 8.0f);
      Point3D32 randomPoint3f;

      for (int i = 0; i < 1000; i++)
      {
         randomPoint3f = RandomGeometry.nextPoint3D32(random, min, max);

         assertTrue((randomPoint3f.getX() >= -7.0f) && (randomPoint3f.getX() <= 5.0));
         assertTrue((randomPoint3f.getY() >= -5.0f) && (randomPoint3f.getY() <= 4.0));
         assertTrue((randomPoint3f.getZ() >= -6.0f) && (randomPoint3f.getZ() <= 8.0));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testNextPoint3D32Array()
   {
      Random random = new Random();

      Point3D32 min = new Point3D32(-7.0f, -5.0f, -6.0f);
      Point3D32 max = new Point3D32(5.0f, 4.0f, 8.0f);
      Point3D32[] randomPoint3fCloud = RandomGeometry.nextPoint3D32Array(random, 1000, min, max);

      for (int i = 0; i < 1000; i++)
      {
         assertTrue((randomPoint3fCloud[i].getX() >= -7.0f) && (randomPoint3fCloud[i].getX() <= 5.0));
         assertTrue((randomPoint3fCloud[i].getY() >= -5.0f) && (randomPoint3fCloud[i].getY() <= 4.0));
         assertTrue((randomPoint3fCloud[i].getZ() >= -6.0f) && (randomPoint3fCloud[i].getZ() <= 8.0));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testNextPoint3D()
   {
      Random random = new Random(4876L);
      double[] range1 = {0.0, 0.0, 0.0};
      double[] range2 = {12, 12, 12};

      for (int i = 0; i < 25; i++)
      {
         Point3D point = RandomGeometry.nextPoint3D(random, range1, range2);
         assertTrue(range1[0] < point.getX() && point.getX() < range2[0]);
         assertTrue(range1[1] < point.getY() && point.getY() < range2[1]);
         assertTrue(range1[2] < point.getZ() && point.getZ() < range2[2]);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testNextVector3D()
   {
      Random random = new Random(4876L);
      Point3D boundary1 = new Point3D(0.0, 0.0, 0.0);
      Point3D boundary2 = new Point3D(12, 12, 12);

      for (int i = 0; i < 25; i++)
      {
         Vector3D vector = RandomGeometry.nextVector3D(random, boundary1, boundary2);
         assertTrue(boundary1.getX() < vector.getX() && vector.getX() < boundary2.getX());
         assertTrue(boundary1.getY() < vector.getY() && vector.getY() < boundary2.getY());
         assertTrue(boundary1.getZ() < vector.getZ() && vector.getZ() < boundary2.getZ());
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testNextOrthogonalVector3D()
   {
      Random random = new Random(4876L);

      for (int i = 0; i < 100; i++)
      {
         double length = RandomNumbers.nextDouble(random, 0.1, 100.0);
         Vector3D vector = RandomGeometry.nextVector3D(random, length);
         Vector3D orthoVector = RandomGeometry.nextOrthogonalVector3D(random, vector, true);

         assertEquals(0.0, vector.dot(orthoVector), 1.0e-12);
         assertEquals(1.0, orthoVector.length(), 1.0e-12);
      }
   }
}
