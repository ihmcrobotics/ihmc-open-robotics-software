package us.ihmc.robotics.random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;

public class RandomToolsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGenerateRandomFloatArray()
   {
      Random random = new Random();

      float[] randomFloatArray = RandomTools.generateRandomFloatArray(random, 1000, 0, 10);

      for (float randomFloat : randomFloatArray)
      {
         assertTrue((randomFloat >= 0) && (randomFloat <= 10));
      }

      randomFloatArray = RandomTools.generateRandomFloatArray(random, 1000, 5);

      for (float randomFloat : randomFloatArray)
      {
         assertTrue((randomFloat >= -5) && (randomFloat <= 5));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGenerateRandomIntArray()
   {
      Random random = new Random();

      int[] randomIntArray = RandomTools.generateRandomIntArray(random, 1000, 0, 10);

      for (int randomInt : randomIntArray)
      {
         assertTrue((randomInt >= 0) && (randomInt <= 10));
      }

      randomIntArray = RandomTools.generateRandomIntArray(random, 1000, 5);

      for (float randomFloat : randomIntArray)
      {
         assertTrue((randomFloat >= -5) && (randomFloat <= 5));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGenerateRandomPoint3f()
   {
      Random random = new Random();

      Point3D32 min = new Point3D32(-7.0f, -5.0f, -6.0f);
      Point3D32 max = new Point3D32(5.0f, 4.0f, 8.0f);
      Point3D32 randomPoint3f;

      for (int i = 0; i < 1000; i++)
      {
         randomPoint3f = RandomTools.generateRandomPoint3f(random, min, max);

         assertTrue((randomPoint3f.getX() >= -7.0f) && (randomPoint3f.getX() <= 5.0));
         assertTrue((randomPoint3f.getY() >= -5.0f) && (randomPoint3f.getY() <= 4.0));
         assertTrue((randomPoint3f.getZ() >= -6.0f) && (randomPoint3f.getZ() <= 8.0));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGenerateRandomPoint3fCloud()
   {
      Random random = new Random();

      Point3D32 min = new Point3D32(-7.0f, -5.0f, -6.0f);
      Point3D32 max = new Point3D32(5.0f, 4.0f, 8.0f);
      Point3D32[] randomPoint3fCloud = RandomTools.generateRandomPoint3fCloud(random, 1000, min, max);

      for (int i = 0; i < 1000; i++)
      {
         assertTrue((randomPoint3fCloud[i].getX() >= -7.0f) && (randomPoint3fCloud[i].getX() <= 5.0));
         assertTrue((randomPoint3fCloud[i].getY() >= -5.0f) && (randomPoint3fCloud[i].getY() <= 4.0));
         assertTrue((randomPoint3fCloud[i].getZ() >= -6.0f) && (randomPoint3fCloud[i].getZ() <= 8.0));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGenerateRandomPoint3d()
   {
      Random random = new Random(4876L);
      double[] range1 = {0.0, 0.0, 0.0};
      double[] range2 = {12, 12, 12};

      for (int i = 0; i < 25; i++)
      {
         Point3D point = RandomTools.generateRandomPoint3d(random, range1, range2);
         assertTrue(range1[0] < point.getX() && point.getX() < range2[0]);
         assertTrue(range1[1] < point.getY() && point.getY() < range2[1]);
         assertTrue(range1[2] < point.getZ() && point.getZ() < range2[2]);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGenerateRandomVector3d()
   {
      Random random = new Random(4876L);
      Point3D boundary1 = new Point3D(0.0, 0.0, 0.0);
      Point3D boundary2 = new Point3D(12, 12, 12);

      for (int i = 0; i < 25; i++)
      {
         Vector3D vector = RandomTools.generateRandomVector(random, boundary1, boundary2);
         assertTrue(boundary1.getX() < vector.getX() && vector.getX() < boundary2.getX());
         assertTrue(boundary1.getY() < vector.getY() && vector.getY() < boundary2.getY());
         assertTrue(boundary1.getZ() < vector.getZ() && vector.getZ() < boundary2.getZ());
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGenerateRandomOrthogonalVector3d()
   {
      Random random = new Random(4876L);

      for (int i = 0; i < 100; i++)
      {
         double length = RandomTools.generateRandomDouble(random, 0.1, 100.0);
         Vector3D vector = RandomTools.generateRandomVector(random, length);
         Vector3D orthoVector = RandomTools.generateRandomOrthogonalVector3d(random, vector, true);

         assertEquals(0.0, vector.dot(orthoVector), 1.0e-12);
         assertEquals(1.0, orthoVector.length(), 1.0e-12);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGenerateRandomDoubleInRange()
   {
      Random random = new Random(1876L);
      double range1 = 100.0;
      double range2 = 30.0;

      for (int i = 0; i < 25; i++)
      {
         double actualReturn = RandomTools.generateRandomDoubleInRange(random, range1, range2);
         assertTrue(((range1 < actualReturn) && (actualReturn < range2)) || ((range2 < actualReturn) && (actualReturn < range1)));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGenerateRandomFloatInRange()
   {
      Random random = new Random(1876L);
      float range1 = 100.0f;
      float range2 = 30.0f;

      for (int i = 0; i < 25; i++)
      {
         double actualReturn = RandomTools.generateRandomFloatInRange(random, range1, range2);
         assertTrue(((range1 < actualReturn) && (actualReturn < range2)) || ((range2 < actualReturn) && (actualReturn < range1)));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGenerateRandomBoolean()
   {
      Random random = new Random(1876L);

      boolean randomBoolean;

      ArrayList<Boolean> listOfRandomBooleans = new ArrayList<>();

      for (int i = 0; i < 10; i++)
      {
         randomBoolean = RandomTools.generateRandomBoolean(random);
         listOfRandomBooleans.add(randomBoolean);
      }

      assertTrue(listOfRandomBooleans.get(0));
      assertTrue(listOfRandomBooleans.get(3));
      assertTrue(listOfRandomBooleans.get(5));
      assertFalse(listOfRandomBooleans.get(1));
      assertFalse(listOfRandomBooleans.get(2));
      assertFalse(listOfRandomBooleans.get(4));
   }
}
