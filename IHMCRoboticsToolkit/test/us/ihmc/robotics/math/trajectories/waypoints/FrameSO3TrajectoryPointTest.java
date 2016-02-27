package us.ihmc.robotics.math.trajectories.waypoints;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameSO3TrajectoryPointTest
{

   @Test
   public void testConstructors()
   {
      double epsilon = 1.0e-20;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame aFrame = ReferenceFrame.generateRandomReferenceFrame("aFrame", random, worldFrame);

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FrameOrientation expectedOrientation = new FrameOrientation(expectedFrame);
      FrameVector expectedAngularVelocity = new FrameVector(expectedFrame);

      FrameSO3TrajectoryPoint testedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint,
            epsilon);

      expectedFrame = aFrame;
      expectedTime = 0.0;
      expectedOrientation = new FrameOrientation(expectedFrame);
      expectedAngularVelocity = new FrameVector(expectedFrame);
      testedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedFrame);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint,
            epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint,
            epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      FrameSO3TrajectoryPoint expectedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedFrameSO3TrajectoryPoint);

      assertTrue(expectedFrameSO3TrajectoryPoint.epsilonEquals(testedFrameSO3TrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedFrameSO3TrajectoryPoint.getReferenceFrame(), expectedFrameSO3TrajectoryPoint.getTime(),
            expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint, epsilon);

      final ReferenceFrame expectedFinalFrame = aFrame;
      final double expectedFinalTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      final FrameOrientation expectedFinalOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFinalFrame);
      final FrameVector expectedFinalAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFinalFrame);

      SimpleSO3TrajectoryPoint expectedSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint();
      expectedSO3TrajectoryPoint.setTime(expectedFinalTime);
      expectedSO3TrajectoryPoint.setOrientation(expectedFinalOrientation.getQuaternion());
      expectedSO3TrajectoryPoint.setAngularVelocity(expectedFinalAngularVelocity.getVector());

      testedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedFinalFrame, expectedSO3TrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalFrame, expectedFinalTime, expectedFinalOrientation, expectedFinalAngularVelocity,
            testedFrameSO3TrajectoryPoint, epsilon);

   }

   @Test
   public void testSetters()
   {
      double epsilon = 1.0e-15;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame aFrame = ReferenceFrame.generateRandomReferenceFrame("aFrame", random, worldFrame);

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FrameOrientation expectedOrientation = new FrameOrientation(expectedFrame);
      FrameVector expectedAngularVelocity = new FrameVector(expectedFrame);

      final FrameSO3TrajectoryPoint testedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint,
            epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint,
            epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation.getQuaternion(), expectedAngularVelocity.getVector());

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint,
            epsilon);

      expectedFrame = aFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameSO3TrajectoryPoint.setIncludingFrame(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint,
            epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      FrameSO3TrajectoryPoint expectedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedFrameSO3TrajectoryPoint.setIncludingFrame(expectedFrameSO3TrajectoryPoint);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      expectedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedFrameSO3TrajectoryPoint.set(expectedFrameSO3TrajectoryPoint);

      assertTrue(expectedFrameSO3TrajectoryPoint.epsilonEquals(testedFrameSO3TrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedFrameSO3TrajectoryPoint.getReferenceFrame(), expectedFrameSO3TrajectoryPoint.getTime(),
            expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint, epsilon);

      final ReferenceFrame expectedFinalFrame = aFrame;
      final double expectedFinalTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      final FrameOrientation expectedFinalOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFinalFrame);
      final FrameVector expectedFinalAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFinalFrame);

      SimpleSO3TrajectoryPoint expectedSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint();
      expectedSO3TrajectoryPoint.setTime(expectedFinalTime);
      expectedSO3TrajectoryPoint.setOrientation(expectedFinalOrientation.getQuaternion());
      expectedSO3TrajectoryPoint.setAngularVelocity(expectedFinalAngularVelocity.getVector());

      testedFrameSO3TrajectoryPoint.setIncludingFrame(expectedFinalFrame, expectedSO3TrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalFrame, expectedFinalTime, expectedFinalOrientation, expectedFinalAngularVelocity,
            testedFrameSO3TrajectoryPoint, epsilon);

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
      FrameSO3TrajectoryPoint testedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = ReferenceFrame.generateRandomReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : expectedFrame);

         expectedOrientation.changeFrame(expectedFrame);
         expectedAngularVelocity.changeFrame(expectedFrame);
         testedFrameSO3TrajectoryPoint.changeFrame(expectedFrame);

         assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint,
               epsilon);
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
      FrameSO3TrajectoryPoint testedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedOrientation.setToZero();
      expectedAngularVelocity.setToZero();
      testedFrameSO3TrajectoryPoint.setToZero();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint,
            epsilon);

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedFrameSO3TrajectoryPoint.setIncludingFrame(expectedTime, expectedOrientation, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedOrientation.setToZero(expectedFrame);
      expectedAngularVelocity.setToZero(expectedFrame);
      testedFrameSO3TrajectoryPoint.setToZero(expectedFrame);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint,
            epsilon);
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
      FrameSO3TrajectoryPoint testedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedFrameSO3TrajectoryPoint.setToNaN();
      assertTrue(Double.isNaN(testedFrameSO3TrajectoryPoint.getTime()));
      assertTrue(testedFrameSO3TrajectoryPoint.containsNaN());

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedFrameSO3TrajectoryPoint.setIncludingFrame(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedFrameSO3TrajectoryPoint.setToNaN(expectedFrame);

      assertTrue(expectedFrame == testedFrameSO3TrajectoryPoint.getReferenceFrame());
      assertTrue(Double.isNaN(testedFrameSO3TrajectoryPoint.getTime()));
      assertTrue(testedFrameSO3TrajectoryPoint.containsNaN());
   }

   static void assertTrajectoryPointContainsExpectedData(ReferenceFrame expectedFrame, double expectedTime, FrameOrientation expectedOrientation,
         FrameVector expectedAngularVelocity, FrameSO3TrajectoryPoint testedFrameSO3TrajectoryPoint, double epsilon)
   {
      assertTrue(expectedFrame == testedFrameSO3TrajectoryPoint.getReferenceFrame());
      assertEquals(expectedTime, testedFrameSO3TrajectoryPoint.getTime(), epsilon);
      assertTrue(expectedOrientation.epsilonEquals(testedFrameSO3TrajectoryPoint.getSimpleWaypoint().getOrientation(), epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(testedFrameSO3TrajectoryPoint.getSimpleWaypoint().getAngularVelocity(), epsilon));

      Quat4d actualOrientation = new Quat4d();
      Vector3d actualAngularVelocity = new Vector3d();

      testedFrameSO3TrajectoryPoint.getOrientation(actualOrientation);
      testedFrameSO3TrajectoryPoint.getAngularVelocity(actualAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, epsilon));

      FrameOrientation actualFrameOrientation = new FrameOrientation();
      FrameVector actualFrameAngularVelocity = new FrameVector();

      testedFrameSO3TrajectoryPoint.getOrientationIncludingFrame(actualFrameOrientation);
      testedFrameSO3TrajectoryPoint.getAngularVelocityIncludingFrame(actualFrameAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));

      actualFrameOrientation = new FrameOrientation(expectedFrame);
      actualFrameAngularVelocity = new FrameVector(expectedFrame);

      testedFrameSO3TrajectoryPoint.getOrientation(actualFrameOrientation);
      testedFrameSO3TrajectoryPoint.getAngularVelocity(actualFrameAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));
   }
}
