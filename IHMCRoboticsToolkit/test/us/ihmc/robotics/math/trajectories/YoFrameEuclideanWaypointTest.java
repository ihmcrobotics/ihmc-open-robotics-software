package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFrameEuclideanWaypointTest
{

   @Test
   public void testConstructor()
   {
      double epsilon = 1.0e-20;
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FramePoint expectedPosition = new FramePoint(expectedFrame);
      FrameVector expectedLinearVelocity = new FrameVector(expectedFrame);

      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameEuclideanWaypoint testedYoFrameEuclideanWaypoint = new YoFrameEuclideanWaypoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanWaypoint, epsilon);
   }

   @Test
   public void testSetters()
   {
      double epsilon = 1.0e-20;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FramePoint expectedPosition = new FramePoint(expectedFrame);
      FrameVector expectedLinearVelocity = new FrameVector(expectedFrame);

      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameEuclideanWaypoint testedYoFrameEuclideanWaypoint = new YoFrameEuclideanWaypoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanWaypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedYoFrameEuclideanWaypoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanWaypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedYoFrameEuclideanWaypoint.set(expectedTime, expectedPosition.getPoint(), expectedLinearVelocity.getVector());

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanWaypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      YoFrameEuclideanWaypoint expectedYoFrameEuclideanWaypoint = new YoFrameEuclideanWaypoint("sdfsd", "asd", new YoVariableRegistry("asawe"), expectedFrame);

      testedYoFrameEuclideanWaypoint.set(expectedYoFrameEuclideanWaypoint);
      FrameEuclideanWaypoint expectedFrameEuclideanWaypoint = expectedYoFrameEuclideanWaypoint.getFrameEuclideanWaypointCopy();

      assertTrue(expectedYoFrameEuclideanWaypoint.epsilonEquals(testedYoFrameEuclideanWaypoint, epsilon));
      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrameEuclideanWaypoint.getReferenceFrame(), expectedFrameEuclideanWaypoint.getTime(),
            expectedFrameEuclideanWaypoint.getPosition(), expectedFrameEuclideanWaypoint.getLinearVelocity(), testedYoFrameEuclideanWaypoint,
            epsilon);
   }

   @Test
   public void testChangeFrame() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      FramePoint expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameVector expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameEuclideanWaypoint testedYoFrameEuclideanWaypoint = new YoFrameEuclideanWaypoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);
      
      testedYoFrameEuclideanWaypoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      ReferenceFrame[] randomFrames = new ReferenceFrame[10];
      randomFrames[0] = worldFrame;
      for (int i = 1; i < 10; i++)
         randomFrames[i] = ReferenceFrame.generateRandomReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : randomFrames[random.nextInt(i)]);
      
      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = randomFrames[random.nextInt(10)];
         testedYoFrameEuclideanWaypoint.registerReferenceFrame(expectedFrame);

         expectedPosition.changeFrame(expectedFrame);
         expectedLinearVelocity.changeFrame(expectedFrame);
         testedYoFrameEuclideanWaypoint.changeFrame(expectedFrame);

         assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanWaypoint, epsilon);
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      FramePoint expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameVector expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameEuclideanWaypoint testedYoFrameEuclideanWaypoint = new YoFrameEuclideanWaypoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);
      testedYoFrameEuclideanWaypoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero();
      expectedLinearVelocity.setToZero();
      testedYoFrameEuclideanWaypoint.setToZero();
      
      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanWaypoint, epsilon);

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      testedYoFrameEuclideanWaypoint.registerReferenceFrame(expectedFrame);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedYoFrameEuclideanWaypoint.switchCurrentReferenceFrame(worldFrame);
      testedYoFrameEuclideanWaypoint.registerReferenceFrame(worldFrame);
      testedYoFrameEuclideanWaypoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero(expectedFrame);
      expectedLinearVelocity.setToZero(expectedFrame);
      testedYoFrameEuclideanWaypoint.switchCurrentReferenceFrame(expectedFrame);
      
      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanWaypoint, epsilon);
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      FramePoint expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameVector expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameEuclideanWaypoint testedYoFrameEuclideanWaypoint = new YoFrameEuclideanWaypoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);
      testedYoFrameEuclideanWaypoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      testedYoFrameEuclideanWaypoint.setToNaN();
      assertTrue(Double.isNaN(testedYoFrameEuclideanWaypoint.getTime()));
      assertTrue(testedYoFrameEuclideanWaypoint.getPosition().containsNaN());
      assertTrue(testedYoFrameEuclideanWaypoint.getLinearVelocity().containsNaN());

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      testedYoFrameEuclideanWaypoint.registerReferenceFrame(expectedFrame);
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedYoFrameEuclideanWaypoint.switchCurrentReferenceFrame(worldFrame);
      testedYoFrameEuclideanWaypoint.registerReferenceFrame(worldFrame);
      testedYoFrameEuclideanWaypoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      testedYoFrameEuclideanWaypoint.setToNaN(expectedFrame);

      assertTrue(expectedFrame == testedYoFrameEuclideanWaypoint.getReferenceFrame());
      assertTrue(Double.isNaN(testedYoFrameEuclideanWaypoint.getTime()));
      assertTrue(testedYoFrameEuclideanWaypoint.getPosition().containsNaN());
      assertTrue(testedYoFrameEuclideanWaypoint.getLinearVelocity().containsNaN());
   }

   private void assertWaypointContainsExpectedData(String expectedNamePrefix, String expectedNameSuffix, ReferenceFrame expectedFrame, double expectedTime, FramePoint expectedPosition,
         FrameVector expectedLinearVelocity, YoFrameEuclideanWaypoint testedYoFrameEuclideanWaypoint, double epsilon)
   {
      assertTrue(expectedFrame == testedYoFrameEuclideanWaypoint.getReferenceFrame());
      assertEquals(expectedTime, testedYoFrameEuclideanWaypoint.getTime(), epsilon);
      assertEquals(expectedNamePrefix, testedYoFrameEuclideanWaypoint.getNamePrefix());
      assertEquals(expectedNameSuffix, testedYoFrameEuclideanWaypoint.getNameSuffix());
      assertTrue(expectedPosition.epsilonEquals(testedYoFrameEuclideanWaypoint.getPosition().getFramePointCopy(), epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(testedYoFrameEuclideanWaypoint.getLinearVelocity().getFrameVectorCopy(), epsilon));

      FrameEuclideanWaypoint actualFrameEuclideanWaypoint = new FrameEuclideanWaypoint();
      testedYoFrameEuclideanWaypoint.getFrameEuclideanWaypointIncludingFrame(actualFrameEuclideanWaypoint);
      FrameEuclideanWaypointTest.assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, actualFrameEuclideanWaypoint, epsilon);
      actualFrameEuclideanWaypoint = new FrameEuclideanWaypoint(expectedFrame);
      testedYoFrameEuclideanWaypoint.getFrameEuclideanWaypoint(actualFrameEuclideanWaypoint);
      FrameEuclideanWaypointTest.assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, actualFrameEuclideanWaypoint, epsilon);

      Point3d actualPosition = new Point3d();
      Vector3d actualLinearVelocity = new Vector3d();

      testedYoFrameEuclideanWaypoint.getPosition(actualPosition);
      testedYoFrameEuclideanWaypoint.getLinearVelocity(actualLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualPosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualLinearVelocity, epsilon));

      FramePoint actualFramePosition = new FramePoint(expectedFrame);
      FrameVector actualFrameLinearVelocity = new FrameVector(expectedFrame);

      testedYoFrameEuclideanWaypoint.getPosition(actualFramePosition);
      testedYoFrameEuclideanWaypoint.getLinearVelocity(actualFrameLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));

      actualFramePosition = new FramePoint();
      actualFrameLinearVelocity = new FrameVector();

      testedYoFrameEuclideanWaypoint.getPositionIncludingFrame(actualFramePosition);
      testedYoFrameEuclideanWaypoint.getLinearVelocityIncludingFrame(actualFrameLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
   }

}
