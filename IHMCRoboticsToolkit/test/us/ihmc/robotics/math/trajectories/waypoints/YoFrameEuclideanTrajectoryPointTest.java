package us.ihmc.robotics.math.trajectories.waypoints;

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

public class YoFrameEuclideanTrajectoryPointTest
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
      YoFrameEuclideanTrajectoryPoint testedYoFrameEuclideanTrajectoryPoint = new YoFrameEuclideanTrajectoryPoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanTrajectoryPoint, epsilon);
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
      YoFrameEuclideanTrajectoryPoint testedYoFrameEuclideanTrajectoryPoint = new YoFrameEuclideanTrajectoryPoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedYoFrameEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedYoFrameEuclideanTrajectoryPoint.set(expectedTime, expectedPosition.getPoint(), expectedLinearVelocity.getVector());

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      YoFrameEuclideanTrajectoryPoint expectedYoFrameEuclideanTrajectoryPoint = new YoFrameEuclideanTrajectoryPoint("sdfsd", "asd", new YoVariableRegistry("asawe"), expectedFrame);

      testedYoFrameEuclideanTrajectoryPoint.set(expectedYoFrameEuclideanTrajectoryPoint);

      assertTrue(expectedYoFrameEuclideanTrajectoryPoint.epsilonEquals(testedYoFrameEuclideanTrajectoryPoint, epsilon));
      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, testedYoFrameEuclideanTrajectoryPoint.getReferenceFrame(), testedYoFrameEuclideanTrajectoryPoint.getTime(),
            testedYoFrameEuclideanTrajectoryPoint.getPosition().getFramePointCopy(), testedYoFrameEuclideanTrajectoryPoint.getLinearVelocity().getFrameVectorCopy(), testedYoFrameEuclideanTrajectoryPoint,
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
      YoFrameEuclideanTrajectoryPoint testedYoFrameEuclideanTrajectoryPoint = new YoFrameEuclideanTrajectoryPoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);
      
      testedYoFrameEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      ReferenceFrame[] randomFrames = new ReferenceFrame[10];
      randomFrames[0] = worldFrame;
      for (int i = 1; i < 10; i++)
         randomFrames[i] = ReferenceFrame.generateRandomReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : randomFrames[random.nextInt(i)]);
      
      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = randomFrames[random.nextInt(10)];
         testedYoFrameEuclideanTrajectoryPoint.registerReferenceFrame(expectedFrame);

         expectedPosition.changeFrame(expectedFrame);
         expectedLinearVelocity.changeFrame(expectedFrame);
         testedYoFrameEuclideanTrajectoryPoint.changeFrame(expectedFrame);

         assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanTrajectoryPoint, epsilon);
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
      YoFrameEuclideanTrajectoryPoint testedYoFrameEuclideanTrajectoryPoint = new YoFrameEuclideanTrajectoryPoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);
      testedYoFrameEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero();
      expectedLinearVelocity.setToZero();
      testedYoFrameEuclideanTrajectoryPoint.setToZero();
      
      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      testedYoFrameEuclideanTrajectoryPoint.registerReferenceFrame(expectedFrame);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedYoFrameEuclideanTrajectoryPoint.switchCurrentReferenceFrame(worldFrame);
      testedYoFrameEuclideanTrajectoryPoint.registerReferenceFrame(worldFrame);
      testedYoFrameEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero(expectedFrame);
      expectedLinearVelocity.setToZero(expectedFrame);
      testedYoFrameEuclideanTrajectoryPoint.switchCurrentReferenceFrame(expectedFrame);
      
      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanTrajectoryPoint, epsilon);
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
      YoFrameEuclideanTrajectoryPoint testedYoFrameEuclideanTrajectoryPoint = new YoFrameEuclideanTrajectoryPoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);
      testedYoFrameEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      testedYoFrameEuclideanTrajectoryPoint.setToNaN();
      assertTrue(Double.isNaN(testedYoFrameEuclideanTrajectoryPoint.getTime()));
      assertTrue(testedYoFrameEuclideanTrajectoryPoint.getPosition().containsNaN());
      assertTrue(testedYoFrameEuclideanTrajectoryPoint.getLinearVelocity().containsNaN());

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      testedYoFrameEuclideanTrajectoryPoint.registerReferenceFrame(expectedFrame);
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedYoFrameEuclideanTrajectoryPoint.switchCurrentReferenceFrame(worldFrame);
      testedYoFrameEuclideanTrajectoryPoint.registerReferenceFrame(worldFrame);
      testedYoFrameEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      testedYoFrameEuclideanTrajectoryPoint.setToNaN(expectedFrame);

      assertTrue(expectedFrame == testedYoFrameEuclideanTrajectoryPoint.getReferenceFrame());
      assertTrue(Double.isNaN(testedYoFrameEuclideanTrajectoryPoint.getTime()));
      assertTrue(testedYoFrameEuclideanTrajectoryPoint.getPosition().containsNaN());
      assertTrue(testedYoFrameEuclideanTrajectoryPoint.getLinearVelocity().containsNaN());
   }

   private void assertWaypointContainsExpectedData(String expectedNamePrefix, String expectedNameSuffix, ReferenceFrame expectedFrame, double expectedTime, FramePoint expectedPosition,
         FrameVector expectedLinearVelocity, YoFrameEuclideanTrajectoryPoint testedYoFrameEuclideanTrajectoryPoint, double epsilon)
   {
      assertTrue(expectedFrame == testedYoFrameEuclideanTrajectoryPoint.getReferenceFrame());
      assertEquals(expectedTime, testedYoFrameEuclideanTrajectoryPoint.getTime(), epsilon);
      assertEquals(expectedNamePrefix, testedYoFrameEuclideanTrajectoryPoint.getNamePrefix());
      assertEquals(expectedNameSuffix, testedYoFrameEuclideanTrajectoryPoint.getNameSuffix());
      assertTrue(expectedPosition.epsilonEquals(testedYoFrameEuclideanTrajectoryPoint.getPosition().getFramePointCopy(), epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(testedYoFrameEuclideanTrajectoryPoint.getLinearVelocity().getFrameVectorCopy(), epsilon));

      FrameEuclideanTrajectoryPoint actualFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedFrame);
      testedYoFrameEuclideanTrajectoryPoint.get(actualFrameEuclideanTrajectoryPoint);
      FrameEuclideanTrajectoryPointTest.assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, actualFrameEuclideanTrajectoryPoint, epsilon);
      actualFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedFrame);
      testedYoFrameEuclideanTrajectoryPoint.get(actualFrameEuclideanTrajectoryPoint);
      FrameEuclideanTrajectoryPointTest.assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, actualFrameEuclideanTrajectoryPoint, epsilon);

      Point3d actualPosition = new Point3d();
      Vector3d actualLinearVelocity = new Vector3d();

      testedYoFrameEuclideanTrajectoryPoint.getPosition(actualPosition);
      testedYoFrameEuclideanTrajectoryPoint.getLinearVelocity(actualLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualPosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualLinearVelocity, epsilon));

      FramePoint actualFramePosition = new FramePoint(expectedFrame);
      FrameVector actualFrameLinearVelocity = new FrameVector(expectedFrame);

      testedYoFrameEuclideanTrajectoryPoint.getPosition(actualFramePosition);
      testedYoFrameEuclideanTrajectoryPoint.getLinearVelocity(actualFrameLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));

      actualFramePosition = new FramePoint();
      actualFrameLinearVelocity = new FrameVector();

      testedYoFrameEuclideanTrajectoryPoint.getPositionIncludingFrame(actualFramePosition);
      testedYoFrameEuclideanTrajectoryPoint.getLinearVelocityIncludingFrame(actualFrameLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
   }

}
