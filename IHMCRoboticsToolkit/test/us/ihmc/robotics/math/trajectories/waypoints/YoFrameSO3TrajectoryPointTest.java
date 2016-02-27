package us.ihmc.robotics.math.trajectories.waypoints;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFrameSO3TrajectoryPointTest
{

   @Test
   public void testConstructor()
   {
      double epsilon = 1.0e-20;
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FrameOrientation expectedOrientation = new FrameOrientation(expectedFrame);
      FrameVector expectedAngularVelocity = new FrameVector(expectedFrame);

      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSO3TrajectoryPoint testedYoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint(expectedNamePrefix, expectedNameSuffix,
            new YoVariableRegistry("schnoop"), expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
            testedYoFrameSO3TrajectoryPoint, epsilon);
   }

   @Test
   public void testSetters()
   {
      double epsilon = 1.0e-20;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FrameOrientation expectedOrientation = new FrameOrientation(expectedFrame);
      FrameVector expectedAngularVelocity = new FrameVector(expectedFrame);

      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSO3TrajectoryPoint testedYoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint(expectedNamePrefix, expectedNameSuffix,
            new YoVariableRegistry("schnoop"), expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
            testedYoFrameSO3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedYoFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
            testedYoFrameSO3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedYoFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation.getQuaternion(), expectedAngularVelocity.getVector());

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
            testedYoFrameSO3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      YoFrameSO3TrajectoryPoint expectedYoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint("sdfsd", "asd", new YoVariableRegistry("asawe"), expectedFrame);

      testedYoFrameSO3TrajectoryPoint.set(expectedYoFrameSO3TrajectoryPoint);

      assertTrue(expectedYoFrameSO3TrajectoryPoint.epsilonEquals(testedYoFrameSO3TrajectoryPoint, epsilon));
      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, testedYoFrameSO3TrajectoryPoint.getReferenceFrame(),
            testedYoFrameSO3TrajectoryPoint.getTime(), testedYoFrameSO3TrajectoryPoint.getOrientation().getFrameOrientationCopy(),
            testedYoFrameSO3TrajectoryPoint.getAngularVelocity().getFrameVectorCopy(), testedYoFrameSO3TrajectoryPoint, epsilon);
   }

   @Test
   public void testChangeFrame() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      FrameOrientation expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      FrameVector expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSO3TrajectoryPoint testedYoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint(expectedNamePrefix, expectedNameSuffix,
            new YoVariableRegistry("schnoop"), expectedFrame);

      testedYoFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      ReferenceFrame[] randomFrames = new ReferenceFrame[10];
      randomFrames[0] = worldFrame;
      for (int i = 1; i < 10; i++)
         randomFrames[i] = ReferenceFrame.generateRandomReferenceFrame("randomFrame" + i, random,
               random.nextBoolean() ? worldFrame : randomFrames[random.nextInt(i)]);

      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = randomFrames[random.nextInt(10)];
         testedYoFrameSO3TrajectoryPoint.registerReferenceFrame(expectedFrame);

         expectedOrientation.changeFrame(expectedFrame);
         expectedAngularVelocity.changeFrame(expectedFrame);
         testedYoFrameSO3TrajectoryPoint.changeFrame(expectedFrame);

         assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
               testedYoFrameSO3TrajectoryPoint, epsilon);
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
      FrameOrientation expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      FrameVector expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSO3TrajectoryPoint testedYoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint(expectedNamePrefix, expectedNameSuffix,
            new YoVariableRegistry("schnoop"), expectedFrame);
      testedYoFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedOrientation.setToZero();
      expectedAngularVelocity.setToZero();
      testedYoFrameSO3TrajectoryPoint.setToZero();

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
            testedYoFrameSO3TrajectoryPoint, epsilon);

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      testedYoFrameSO3TrajectoryPoint.registerReferenceFrame(expectedFrame);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedYoFrameSO3TrajectoryPoint.switchCurrentReferenceFrame(worldFrame);
      testedYoFrameSO3TrajectoryPoint.registerReferenceFrame(worldFrame);
      testedYoFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedOrientation.setToZero(expectedFrame);
      expectedAngularVelocity.setToZero(expectedFrame);
      testedYoFrameSO3TrajectoryPoint.switchCurrentReferenceFrame(expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
            testedYoFrameSO3TrajectoryPoint, epsilon);
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      FrameOrientation expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      FrameVector expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSO3TrajectoryPoint testedYoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint(expectedNamePrefix, expectedNameSuffix,
            new YoVariableRegistry("schnoop"), expectedFrame);
      testedYoFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedYoFrameSO3TrajectoryPoint.setToNaN();
      assertTrue(Double.isNaN(testedYoFrameSO3TrajectoryPoint.getTime()));
      assertTrue(testedYoFrameSO3TrajectoryPoint.getOrientation().containsNaN());
      assertTrue(testedYoFrameSO3TrajectoryPoint.getAngularVelocity().containsNaN());

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      testedYoFrameSO3TrajectoryPoint.registerReferenceFrame(expectedFrame);
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedYoFrameSO3TrajectoryPoint.switchCurrentReferenceFrame(worldFrame);
      testedYoFrameSO3TrajectoryPoint.registerReferenceFrame(worldFrame);
      testedYoFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedYoFrameSO3TrajectoryPoint.setToNaN(expectedFrame);

      assertTrue(expectedFrame == testedYoFrameSO3TrajectoryPoint.getReferenceFrame());
      assertTrue(Double.isNaN(testedYoFrameSO3TrajectoryPoint.getTime()));
      assertTrue(testedYoFrameSO3TrajectoryPoint.getOrientation().containsNaN());
      assertTrue(testedYoFrameSO3TrajectoryPoint.getAngularVelocity().containsNaN());
   }

   private void assertWaypointContainsExpectedData(String expectedNamePrefix, String expectedNameSuffix, ReferenceFrame expectedFrame, double expectedTime,
         FrameOrientation expectedOrientation, FrameVector expectedAngularVelocity, YoFrameSO3TrajectoryPoint testedYoFrameSO3TrajectoryPoint, double epsilon)
   {
      assertTrue(expectedFrame == testedYoFrameSO3TrajectoryPoint.getReferenceFrame());
      assertEquals(expectedTime, testedYoFrameSO3TrajectoryPoint.getTime(), epsilon);
      assertEquals(expectedNamePrefix, testedYoFrameSO3TrajectoryPoint.getNamePrefix());
      assertEquals(expectedNameSuffix, testedYoFrameSO3TrajectoryPoint.getNameSuffix());
      assertTrue(expectedOrientation.epsilonEquals(testedYoFrameSO3TrajectoryPoint.getOrientation().getFrameOrientationCopy(), epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(testedYoFrameSO3TrajectoryPoint.getAngularVelocity().getFrameVectorCopy(), epsilon));

      FrameSO3TrajectoryPoint actualFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint();
      testedYoFrameSO3TrajectoryPoint.getIncludingFrame(actualFrameSO3TrajectoryPoint);
      FrameSO3TrajectoryPointTest.assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
            actualFrameSO3TrajectoryPoint, epsilon);
      actualFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedFrame);
      testedYoFrameSO3TrajectoryPoint.get(actualFrameSO3TrajectoryPoint);
      FrameSO3TrajectoryPointTest.assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
            actualFrameSO3TrajectoryPoint, epsilon);

      Quat4d actualOrientation = new Quat4d();
      Vector3d actualAngularVelocity = new Vector3d();

      testedYoFrameSO3TrajectoryPoint.getOrientation(actualOrientation);
      testedYoFrameSO3TrajectoryPoint.getAngularVelocity(actualAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, epsilon));

      FrameOrientation actualFrameOrientation = new FrameOrientation(expectedFrame);
      FrameVector actualFrameAngularVelocity = new FrameVector(expectedFrame);

      testedYoFrameSO3TrajectoryPoint.getOrientation(actualFrameOrientation);
      testedYoFrameSO3TrajectoryPoint.getAngularVelocity(actualFrameAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));

      actualFrameOrientation = new FrameOrientation();
      actualFrameAngularVelocity = new FrameVector();

      testedYoFrameSO3TrajectoryPoint.getOrientationIncludingFrame(actualFrameOrientation);
      testedYoFrameSO3TrajectoryPoint.getAngularVelocityIncludingFrame(actualFrameAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));
   }
}
