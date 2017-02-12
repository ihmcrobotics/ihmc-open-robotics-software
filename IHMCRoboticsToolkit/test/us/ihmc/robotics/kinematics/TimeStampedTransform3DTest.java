package us.ihmc.robotics.kinematics;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;

public class TimeStampedTransform3DTest
{

   private static final double EPSILON = 1.0e-15;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testEmptyConstructor()
   {
      TimeStampedTransform3D toBeTested = new TimeStampedTransform3D();

      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      long expectedTimestamp = 0;

      assertEquals("Timestamp is different from what was expected", expectedTimestamp, toBeTested.getTimeStamp());
      assertTrue("Transform is different from what was expected", expectedTransform.epsilonEquals(toBeTested.getTransform3D(), EPSILON));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructor()
   {
      Random random = new Random(3213620L);
      RigidBodyTransform expectedTransform = RigidBodyTransform.generateRandomTransform(random);
      long expectedTimestamp = RandomTools.generateRandomInt(random, 132, 51568418);
      

      TimeStampedTransform3D toBeTested = new TimeStampedTransform3D(expectedTransform, expectedTimestamp);

      assertEquals("Timestamp is different from what was expected", expectedTimestamp, toBeTested.getTimeStamp());
      assertTrue("Transform is different from what was expected", expectedTransform.epsilonEquals(toBeTested.getTransform3D(), EPSILON));

      assertTrue("TimestampedTransform should only copy the given transform into an internal variable", expectedTransform != toBeTested.getTransform3D());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetters()
   {
      TimeStampedTransform3D toBeTested = new TimeStampedTransform3D();

      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      long expectedTimestamp = 0;

      assertEquals("Timestamp is different from what was expected", expectedTimestamp, toBeTested.getTimeStamp());
      assertTrue("Transform is different from what was expected", expectedTransform.epsilonEquals(toBeTested.getTransform3D(), EPSILON));

      Random random = new Random(3213620L);
      expectedTimestamp = RandomTools.generateRandomInt(random, 132, 51568418);
      toBeTested.setTimeStamp(expectedTimestamp);

      assertEquals("Timestamp is different from what was expected", expectedTimestamp, toBeTested.getTimeStamp());
      assertTrue("Transform is different from what was expected", expectedTransform.epsilonEquals(toBeTested.getTransform3D(), EPSILON));

      expectedTransform = RigidBodyTransform.generateRandomTransform(random);
      toBeTested.setTransform3D(expectedTransform);

      assertEquals("Timestamp is different from what was expected", expectedTimestamp, toBeTested.getTimeStamp());
      assertTrue("Transform is different from what was expected", expectedTransform.epsilonEquals(toBeTested.getTransform3D(), EPSILON));

      expectedTimestamp = RandomTools.generateRandomInt(random, 132, 51568418);
      expectedTransform = RigidBodyTransform.generateRandomTransform(random);
      TimeStampedTransform3D expectedTimeStampedTransform = new TimeStampedTransform3D(expectedTransform, expectedTimestamp);

      toBeTested.set(expectedTimeStampedTransform);

      assertEquals("Timestamp is different from what was expected", expectedTimeStampedTransform.getTimeStamp(), toBeTested.getTimeStamp());
      assertTrue("Transform is different from what was expected", expectedTimeStampedTransform.getTransform3D().epsilonEquals(toBeTested.getTransform3D(), EPSILON));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetTransform()
   {
      TimeStampedTransform3D toBeTested = new TimeStampedTransform3D();

      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      // Test that the getter returns the original transform and not a copy.
      toBeTested.getTransform3D().set(expectedTransform);

      assertTrue("Transform is different from what was expected", expectedTransform.epsilonEquals(toBeTested.getTransform3D(), EPSILON));
   }
}
