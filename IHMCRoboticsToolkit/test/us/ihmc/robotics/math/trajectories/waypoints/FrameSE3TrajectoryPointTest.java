package us.ihmc.robotics.math.trajectories.waypoints;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameSE3TrajectoryPointTest
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
      FramePoint expectedPosition = new FramePoint(expectedFrame);
      FrameOrientation expectedOrientation = new FrameOrientation(expectedFrame);
      FrameVector expectedLinearVelocity = new FrameVector(expectedFrame);
      FrameVector expectedAngularVelocity = new FrameVector(expectedFrame);

      FrameSE3TrajectoryPoint testedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

      expectedFrame = aFrame;
      expectedTime = 0.0;
      expectedPosition = new FramePoint(expectedFrame);
      expectedOrientation = new FrameOrientation(expectedFrame);
      expectedLinearVelocity = new FrameVector(expectedFrame);
      expectedAngularVelocity = new FrameVector(expectedFrame);
      testedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedFrame);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      FrameSE3TrajectoryPoint expectedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation,
            expectedLinearVelocity, expectedAngularVelocity);

      testedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedFrameSE3TrajectoryPoint);

      assertTrue(expectedFrameSE3TrajectoryPoint.epsilonEquals(testedFrameSE3TrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedFrameSE3TrajectoryPoint.getReferenceFrame(), expectedFrameSE3TrajectoryPoint.getTime(),
            expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

      final ReferenceFrame expectedFinalFrame = aFrame;
      final double expectedFinalTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      final FramePoint expectedFinalPosition = FramePoint.generateRandomFramePoint(random, expectedFinalFrame, 10.0, 10.0, 10.0);
      final FrameOrientation expectedFinalOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFinalFrame);
      final FrameVector expectedFinalLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFinalFrame);
      final FrameVector expectedFinalAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFinalFrame);

      SimpleSE3TrajectoryPoint expectedSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();
      expectedSE3TrajectoryPoint.setTime(expectedFinalTime);
      expectedSE3TrajectoryPoint.setPosition(expectedFinalPosition.getPoint());
      expectedSE3TrajectoryPoint.setOrientation(expectedFinalOrientation.getQuaternion());
      expectedSE3TrajectoryPoint.setLinearVelocity(expectedFinalLinearVelocity.getVector());
      expectedSE3TrajectoryPoint.setAngularVelocity(expectedFinalAngularVelocity.getVector());

      testedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedFinalFrame, expectedSE3TrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalFrame, expectedFinalTime, expectedFinalPosition, expectedFinalOrientation,
            expectedFinalLinearVelocity, expectedFinalAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

   }

   @Test
   public void testSetters()
   {
      double epsilon = 1.0e-20;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame aFrame = ReferenceFrame.generateRandomReferenceFrame("aFrame", random, worldFrame);

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FramePoint expectedPosition = new FramePoint(expectedFrame);
      FrameOrientation expectedOrientation = new FrameOrientation(expectedFrame);
      FrameVector expectedLinearVelocity = new FrameVector(expectedFrame);
      FrameVector expectedAngularVelocity = new FrameVector(expectedFrame);

      final FrameSE3TrajectoryPoint testedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameSE3TrajectoryPoint.set(expectedTime, expectedPosition.getPoint(), expectedOrientation.getQuaternion(), expectedLinearVelocity.getVector(),
            expectedAngularVelocity.getVector());

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

      expectedFrame = aFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameSE3TrajectoryPoint.setIncludingFrame(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      FrameSE3TrajectoryPoint expectedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation,
            expectedLinearVelocity, expectedAngularVelocity);

      testedFrameSE3TrajectoryPoint.setIncludingFrame(expectedFrameSE3TrajectoryPoint);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      expectedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity);

      testedFrameSE3TrajectoryPoint.set(expectedFrameSE3TrajectoryPoint);

      assertTrue(expectedFrameSE3TrajectoryPoint.epsilonEquals(testedFrameSE3TrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedFrameSE3TrajectoryPoint.getReferenceFrame(), expectedFrameSE3TrajectoryPoint.getTime(),
            expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

      final ReferenceFrame expectedFinalFrame = aFrame;
      final double expectedFinalTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      final FramePoint expectedFinalPosition = FramePoint.generateRandomFramePoint(random, expectedFinalFrame, 10.0, 10.0, 10.0);
      final FrameOrientation expectedFinalOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFinalFrame);
      final FrameVector expectedFinalLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFinalFrame);
      final FrameVector expectedFinalAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFinalFrame);

      SimpleSE3TrajectoryPoint expectedSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();
      expectedSE3TrajectoryPoint.setTime(expectedFinalTime);
      expectedSE3TrajectoryPoint.setPosition(expectedFinalPosition.getPoint());
      expectedSE3TrajectoryPoint.setOrientation(expectedFinalOrientation.getQuaternion());
      expectedSE3TrajectoryPoint.setLinearVelocity(expectedFinalLinearVelocity.getVector());
      expectedSE3TrajectoryPoint.setAngularVelocity(expectedFinalAngularVelocity.getVector());

      testedFrameSE3TrajectoryPoint.setIncludingFrame(expectedFinalFrame, expectedSE3TrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalFrame, expectedFinalTime, expectedFinalPosition, expectedFinalOrientation,
            expectedFinalLinearVelocity, expectedFinalAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

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
      FrameOrientation expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      FrameVector expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameVector expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameSE3TrajectoryPoint testedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation,
            expectedLinearVelocity, expectedAngularVelocity);

      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = ReferenceFrame.generateRandomReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : expectedFrame);

         expectedPosition.changeFrame(expectedFrame);
         expectedOrientation.changeFrame(expectedFrame);
         expectedLinearVelocity.changeFrame(expectedFrame);
         expectedAngularVelocity.changeFrame(expectedFrame);
         testedFrameSE3TrajectoryPoint.changeFrame(expectedFrame);

         assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
               expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);
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
      FrameOrientation expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      FrameVector expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameVector expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameSE3TrajectoryPoint testedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation,
            expectedLinearVelocity, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero();
      expectedOrientation.setToZero();
      expectedLinearVelocity.setToZero();
      expectedAngularVelocity.setToZero();
      testedFrameSE3TrajectoryPoint.setToZero();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedFrameSE3TrajectoryPoint.setIncludingFrame(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero(expectedFrame);
      expectedOrientation.setToZero(expectedFrame);
      expectedLinearVelocity.setToZero(expectedFrame);
      expectedAngularVelocity.setToZero(expectedFrame);
      testedFrameSE3TrajectoryPoint.setToZero(expectedFrame);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      FramePoint expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameOrientation expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      FrameVector expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameVector expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameSE3TrajectoryPoint testedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation,
            expectedLinearVelocity, expectedAngularVelocity);

      testedFrameSE3TrajectoryPoint.setToNaN();
      assertTrue(Double.isNaN(testedFrameSE3TrajectoryPoint.getTime()));
      assertTrue(testedFrameSE3TrajectoryPoint.containsNaN());

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedFrameSE3TrajectoryPoint.setIncludingFrame(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      testedFrameSE3TrajectoryPoint.setToNaN(expectedFrame);

      assertTrue(expectedFrame == testedFrameSE3TrajectoryPoint.getReferenceFrame());
      assertTrue(Double.isNaN(testedFrameSE3TrajectoryPoint.getTime()));
      assertTrue(testedFrameSE3TrajectoryPoint.containsNaN());
   }

   static void assertTrajectoryPointContainsExpectedData(ReferenceFrame expectedFrame, double expectedTime, FramePoint expectedPosition,
         FrameOrientation expectedOrientation, FrameVector expectedLinearVelocity, FrameVector expectedAngularVelocity,
         FrameSE3TrajectoryPoint testedFrameSE3TrajectoryPoint, double epsilon)
   {
      assertTrue(expectedFrame == testedFrameSE3TrajectoryPoint.getReferenceFrame());
      assertEquals(expectedTime, testedFrameSE3TrajectoryPoint.getTime(), epsilon);
      assertTrue(expectedPosition.epsilonEquals(testedFrameSE3TrajectoryPoint.getGeometryObject().getEuclideanWaypoint().getPosition(), epsilon));
      assertTrue(expectedOrientation.epsilonEquals(testedFrameSE3TrajectoryPoint.getGeometryObject().getSO3Waypoint().getOrientation(), epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(testedFrameSE3TrajectoryPoint.getGeometryObject().getEuclideanWaypoint().getLinearVelocity(), epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(testedFrameSE3TrajectoryPoint.getGeometryObject().getSO3Waypoint().getAngularVelocity(), epsilon));

      Point3d actualPosition = new Point3d();
      Quat4d actualOrientation = new Quat4d();
      Vector3d actualLinearVelocity = new Vector3d();
      Vector3d actualAngularVelocity = new Vector3d();

      testedFrameSE3TrajectoryPoint.getPosition(actualPosition);
      testedFrameSE3TrajectoryPoint.getOrientation(actualOrientation);
      testedFrameSE3TrajectoryPoint.getLinearVelocity(actualLinearVelocity);
      testedFrameSE3TrajectoryPoint.getAngularVelocity(actualAngularVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualPosition, epsilon));
      assertTrue(expectedOrientation.epsilonEquals(actualOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, epsilon));

      FramePoint actualFramePosition = new FramePoint();
      FrameOrientation actualFrameOrientation = new FrameOrientation();
      FrameVector actualFrameLinearVelocity = new FrameVector();
      FrameVector actualFrameAngularVelocity = new FrameVector();

      testedFrameSE3TrajectoryPoint.getPositionIncludingFrame(actualFramePosition);
      testedFrameSE3TrajectoryPoint.getOrientationIncludingFrame(actualFrameOrientation);
      testedFrameSE3TrajectoryPoint.getLinearVelocityIncludingFrame(actualFrameLinearVelocity);
      testedFrameSE3TrajectoryPoint.getAngularVelocityIncludingFrame(actualFrameAngularVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));

      actualFramePosition = new FramePoint(expectedFrame);
      actualFrameOrientation = new FrameOrientation(expectedFrame);
      actualFrameLinearVelocity = new FrameVector(expectedFrame);
      actualFrameAngularVelocity = new FrameVector(expectedFrame);

      testedFrameSE3TrajectoryPoint.getPosition(actualFramePosition);
      testedFrameSE3TrajectoryPoint.getOrientation(actualFrameOrientation);
      testedFrameSE3TrajectoryPoint.getLinearVelocity(actualFrameLinearVelocity);
      testedFrameSE3TrajectoryPoint.getAngularVelocity(actualFrameAngularVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));
   }
}
