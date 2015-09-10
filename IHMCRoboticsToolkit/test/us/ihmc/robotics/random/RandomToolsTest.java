package us.ihmc.robotics.random;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Point3f;

import org.junit.Test;

import us.ihmc.tools.random.RandomTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class RandomToolsTest
{

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGenerateRandomFloatArray()
   {
      Random random = new Random();
      
      float[] randomFloatArray = RandomTools.generateRandomFloatArray(random, 1000, 0, 10);
      
      for (float randomFloat : randomFloatArray)
      {
         assertTrue(randomFloat >= 0 && randomFloat <= 10);
      }
      
      randomFloatArray = RandomTools.generateRandomFloatArray(random, 1000, 5);
      
      for (float randomFloat : randomFloatArray)
      {
         assertTrue(randomFloat >= -5 && randomFloat <= 5);
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGenerateRandomIntArray()
   {
      Random random = new Random();
      
      int[] randomIntArray = RandomTools.generateRandomIntArray(random, 1000, 0, 10);
      
      for (int randomInt : randomIntArray)
      {
         assertTrue(randomInt >= 0 && randomInt <= 10);
      }
      
      randomIntArray = RandomTools.generateRandomIntArray(random, 1000, 5);
      
      for (float randomFloat : randomIntArray)
      {
         assertTrue(randomFloat >= -5 && randomFloat <= 5);
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGenerateRandomPoint3f()
   {
      Random random = new Random();

      Point3f min = new Point3f(-7.0f, -5.0f, -6.0f);
      Point3f max = new Point3f(5.0f, 4.0f, 8.0f);
      Point3f randomPoint3f;
      
      for (int i = 0; i < 1000; i++)
      {
         randomPoint3f = RandomTools.generateRandomPoint3f(random, min, max);
         
         assertTrue(randomPoint3f.x >= -7.0f && randomPoint3f.x <= 5.0);
         assertTrue(randomPoint3f.y >= -5.0f && randomPoint3f.y <= 4.0);
         assertTrue(randomPoint3f.z >= -6.0f && randomPoint3f.z <= 8.0);
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGenerateRandomPoint3fCloud()
   {
      Random random = new Random();

      Point3f min = new Point3f(-7.0f, -5.0f, -6.0f);
      Point3f max = new Point3f(5.0f, 4.0f, 8.0f);
      Point3f[] randomPoint3fCloud = RandomTools.generateRandomPoint3fCloud(random, 1000, min, max);
      
      for (int i = 0; i < 1000; i++)
      {
         assertTrue(randomPoint3fCloud[i].x >= -7.0f && randomPoint3fCloud[i].x <= 5.0);
         assertTrue(randomPoint3fCloud[i].y >= -5.0f && randomPoint3fCloud[i].y <= 4.0);
         assertTrue(randomPoint3fCloud[i].z >= -6.0f && randomPoint3fCloud[i].z <= 8.0);
      }
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGenerateRandomPoint3d()
   {
      Random random = new Random(4876L);
      double[] range1 = {0.0, 0.0, 0.0};
      double[] range2 = {12, 12, 12};

      RandomTools.generateRandomPoint3d(random, range1, range2);
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
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

   @DeployableTestMethod(estimatedDuration = 0.0)
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
}
