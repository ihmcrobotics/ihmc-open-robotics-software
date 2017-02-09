package us.ihmc.robotics.math.trajectories.waypoints;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.transformables.EuclideanWaypoint;
import us.ihmc.robotics.geometry.transformables.SE3Waypoint;
import us.ihmc.robotics.geometry.transformables.SO3Waypoint;
import us.ihmc.robotics.geometry.transformables.TransformablePoint3d;
import us.ihmc.robotics.geometry.transformables.TransformableQuat4d;
import us.ihmc.robotics.geometry.transformables.TransformableVector3d;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3TrajectoryPointInterface;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SimpleSE3TrajectoryPointTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCommonUsageExample()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose(worldFrame));

      FramePoint poseFramePosition = new FramePoint(worldFrame, new Point3d(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameOrientation poseOrientation = new FrameOrientation(worldFrame, new AxisAngle4d(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      SimpleSE3TrajectoryPoint simpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();
      SimpleSE3TrajectoryPoint simpleTrajectoryPoint = new SimpleSE3TrajectoryPoint();

      double time = 3.4;
      Point3d position = new Point3d(1.0, 2.1, 3.7);
      TransformableQuat4d orientation = new TransformableQuat4d(new Quat4d(0.1, 0.22, 0.34, 0.56));
      orientation.normalize();

      Vector3d linearVelocity = new Vector3d(-0.4, 1.2, 3.3);
      Vector3d angularVelocity = new Vector3d(1.7, 8.4, 2.2);

      simpleTrajectoryPoint.set(time, position, orientation, linearVelocity, angularVelocity);
      simpleSE3TrajectoryPoint.set(simpleTrajectoryPoint);
      simpleSE3TrajectoryPoint.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));

      // Do some checks:
      RigidBodyTransform transformToPoseFrame = worldFrame.getTransformToDesiredFrame(poseFrame);
      transformToPoseFrame.transform(position);
      orientation.applyTransform(transformToPoseFrame);
      transformToPoseFrame.transform(linearVelocity);
      transformToPoseFrame.transform(angularVelocity);

      SimpleSE3TrajectoryPoint expectedSimpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();

      expectedSimpleSE3TrajectoryPoint.setTime(time);
      expectedSimpleSE3TrajectoryPoint.setPosition(position);
      expectedSimpleSE3TrajectoryPoint.setOrientation(orientation);

      expectedSimpleSE3TrajectoryPoint.setLinearVelocity(linearVelocity);
      expectedSimpleSE3TrajectoryPoint.setAngularVelocity(angularVelocity);

      assertEquals(3.4, simpleSE3TrajectoryPoint.getTime(), 1e-7);
      assertEquals(3.4, expectedSimpleSE3TrajectoryPoint.getTime(), 1e-7);
      assertTrue(expectedSimpleSE3TrajectoryPoint.epsilonEquals(simpleSE3TrajectoryPoint, 1e-10));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructors()
   {
      double epsilon = 1.0e-14;
      Random random = new Random(21651016L);

      double expectedTime = 0.0;
      Point3d expectedPosition = new Point3d();
      Quat4d expectedOrientation = new TransformableQuat4d();
      Vector3d expectedLinearVelocity = new Vector3d();
      Vector3d expectedAngularVelocity = new Vector3d();

      SimpleSE3TrajectoryPoint testedSimpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedSimpleSE3TrajectoryPoint, epsilon);

      expectedTime = 0.0;
      expectedPosition = new Point3d();
      expectedOrientation = new TransformableQuat4d();
      expectedLinearVelocity = new Vector3d();
      expectedAngularVelocity = new Vector3d();
      testedSimpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedSimpleSE3TrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      expectedOrientation = RandomTools.generateRandomQuaternion(random);
      expectedLinearVelocity = RandomTools.generateRandomVector(random);
      expectedAngularVelocity = RandomTools.generateRandomVector(random);

      testedSimpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedSimpleSE3TrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      expectedOrientation = RandomTools.generateRandomQuaternion(random);
      expectedLinearVelocity = RandomTools.generateRandomVector(random);
      expectedAngularVelocity = RandomTools.generateRandomVector(random);

      SimpleSE3TrajectoryPoint expectedSimpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation,
            expectedLinearVelocity, expectedAngularVelocity);

      testedSimpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint(expectedSimpleSE3TrajectoryPoint);

      assertTrue(expectedSimpleSE3TrajectoryPoint.epsilonEquals(testedSimpleSE3TrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedSimpleSE3TrajectoryPoint.getTime(),
            expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity, testedSimpleSE3TrajectoryPoint, epsilon);

      final double expectedFinalTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      final Point3d expectedFinalPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      final Quat4d expectedFinalOrientation = RandomTools.generateRandomQuaternion(random);
      final Vector3d expectedFinalLinearVelocity = RandomTools.generateRandomVector(random);
      final Vector3d expectedFinalAngularVelocity = RandomTools.generateRandomVector(random);

      SimpleSE3TrajectoryPoint expectedSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();
      expectedSE3TrajectoryPoint.setTime(expectedFinalTime);
      expectedSE3TrajectoryPoint.setPosition(expectedFinalPosition);
      expectedSE3TrajectoryPoint.setOrientation(expectedFinalOrientation);
      expectedSE3TrajectoryPoint.setLinearVelocity(expectedFinalLinearVelocity);
      expectedSE3TrajectoryPoint.setAngularVelocity(expectedFinalAngularVelocity);

      testedSimpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint(expectedSE3TrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalTime, expectedFinalPosition, expectedFinalOrientation,
            expectedFinalLinearVelocity, expectedFinalAngularVelocity, testedSimpleSE3TrajectoryPoint, epsilon);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetters()
   {
      double epsilon = 1.0e-14;
      Random random = new Random(21651016L);

      double expectedTime = 0.0;
      Point3d expectedPosition = new Point3d();
      Quat4d expectedOrientation = new TransformableQuat4d();
      Vector3d expectedLinearVelocity = new Vector3d();
      Vector3d expectedAngularVelocity = new Vector3d();

      final SimpleSE3TrajectoryPoint testedSimpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedSimpleSE3TrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      expectedOrientation = RandomTools.generateRandomQuaternion(random);
      expectedLinearVelocity = RandomTools.generateRandomVector(random);
      expectedAngularVelocity = RandomTools.generateRandomVector(random);

      testedSimpleSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedSimpleSE3TrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      expectedOrientation = RandomTools.generateRandomQuaternion(random);
      expectedLinearVelocity = RandomTools.generateRandomVector(random);
      expectedAngularVelocity = RandomTools.generateRandomVector(random);

      testedSimpleSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedSimpleSE3TrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      expectedOrientation = RandomTools.generateRandomQuaternion(random);
      expectedLinearVelocity = RandomTools.generateRandomVector(random);
      expectedAngularVelocity = RandomTools.generateRandomVector(random);

      testedSimpleSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedSimpleSE3TrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      expectedOrientation = RandomTools.generateRandomQuaternion(random);
      expectedLinearVelocity = RandomTools.generateRandomVector(random);
      expectedAngularVelocity = RandomTools.generateRandomVector(random);

      SimpleSE3TrajectoryPoint expectedSimpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation,
            expectedLinearVelocity, expectedAngularVelocity);

      testedSimpleSE3TrajectoryPoint.set(expectedSimpleSE3TrajectoryPoint);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      expectedOrientation = RandomTools.generateRandomQuaternion(random);
      expectedLinearVelocity = RandomTools.generateRandomVector(random);
      expectedAngularVelocity = RandomTools.generateRandomVector(random);

      expectedSimpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity);

      testedSimpleSE3TrajectoryPoint.set(expectedSimpleSE3TrajectoryPoint);

      assertTrue(expectedSimpleSE3TrajectoryPoint.epsilonEquals(testedSimpleSE3TrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedSimpleSE3TrajectoryPoint.getTime(),
            expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity, testedSimpleSE3TrajectoryPoint, epsilon);

      final double expectedFinalTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      final Point3d expectedFinalPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      final Quat4d expectedFinalOrientation = RandomTools.generateRandomQuaternion(random);
      final Vector3d expectedFinalLinearVelocity = RandomTools.generateRandomVector(random);
      final Vector3d expectedFinalAngularVelocity = RandomTools.generateRandomVector(random);

      SimpleSE3TrajectoryPoint expectedSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();
      expectedSE3TrajectoryPoint.setTime(expectedFinalTime);
      expectedSE3TrajectoryPoint.setPosition(expectedFinalPosition);
      expectedSE3TrajectoryPoint.setOrientation(expectedFinalOrientation);
      expectedSE3TrajectoryPoint.setLinearVelocity(expectedFinalLinearVelocity);
      expectedSE3TrajectoryPoint.setAngularVelocity(expectedFinalAngularVelocity);

      testedSimpleSE3TrajectoryPoint.set(expectedSE3TrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalTime, expectedFinalPosition, expectedFinalOrientation,
            expectedFinalLinearVelocity, expectedFinalAngularVelocity, testedSimpleSE3TrajectoryPoint, epsilon);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testChangeFrame() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      TransformablePoint3d expectedPosition = new TransformablePoint3d(RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0));
      TransformableQuat4d expectedOrientation = new TransformableQuat4d(RandomTools.generateRandomQuaternion(random));
      TransformableVector3d expectedLinearVelocity = new TransformableVector3d(RandomTools.generateRandomVector(random));
      TransformableVector3d expectedAngularVelocity = new TransformableVector3d(RandomTools.generateRandomVector(random));
      SimpleSE3TrajectoryPoint testedSimpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation,
            expectedLinearVelocity, expectedAngularVelocity);

      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = ReferenceFrame.generateRandomReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : expectedFrame);

         expectedPosition.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));
         expectedOrientation.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));
         expectedLinearVelocity.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));
         expectedAngularVelocity.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));
         testedSimpleSE3TrajectoryPoint.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));

         assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
               expectedAngularVelocity, testedSimpleSE3TrajectoryPoint, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetToZero() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);

      double expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      Point3d expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      Quat4d expectedOrientation = RandomTools.generateRandomQuaternion(random);
      Vector3d expectedLinearVelocity = RandomTools.generateRandomVector(random);
      Vector3d expectedAngularVelocity = RandomTools.generateRandomVector(random);
      SimpleSE3TrajectoryPoint testedSimpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation,
            expectedLinearVelocity, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedPosition.set(0.0, 0.0, 0.0);
      expectedOrientation.set(0.0, 0.0, 0.0, 1.0);
      expectedLinearVelocity.set(0.0, 0.0, 0.0);
      expectedAngularVelocity.set(0.0, 0.0, 0.0);
      testedSimpleSE3TrajectoryPoint.setToZero();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedSimpleSE3TrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      expectedOrientation = RandomTools.generateRandomQuaternion(random);
      expectedLinearVelocity = RandomTools.generateRandomVector(random);
      expectedAngularVelocity = RandomTools.generateRandomVector(random);
      testedSimpleSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedPosition.set(0.0, 0.0, 0.0);
      expectedOrientation.set(0.0, 0.0, 0.0, 1.0);
      expectedLinearVelocity.set(0.0, 0.0, 0.0);
      expectedAngularVelocity.set(0.0, 0.0, 0.0);
      testedSimpleSE3TrajectoryPoint.setToZero();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedSimpleSE3TrajectoryPoint, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(21651016L);

      double expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      Point3d expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      Quat4d expectedOrientation = RandomTools.generateRandomQuaternion(random);
      Vector3d expectedLinearVelocity = RandomTools.generateRandomVector(random);
      Vector3d expectedAngularVelocity = RandomTools.generateRandomVector(random);
      SimpleSE3TrajectoryPoint testedSimpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation,
            expectedLinearVelocity, expectedAngularVelocity);

      testedSimpleSE3TrajectoryPoint.setToNaN();
      assertTrue(Double.isNaN(testedSimpleSE3TrajectoryPoint.getTime()));
      assertTrue(testedSimpleSE3TrajectoryPoint.containsNaN());

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      expectedOrientation = RandomTools.generateRandomQuaternion(random);
      expectedLinearVelocity = RandomTools.generateRandomVector(random);
      expectedAngularVelocity = RandomTools.generateRandomVector(random);
      testedSimpleSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      testedSimpleSE3TrajectoryPoint.setToNaN();

      assertTrue(Double.isNaN(testedSimpleSE3TrajectoryPoint.getTime()));
      assertTrue(testedSimpleSE3TrajectoryPoint.containsNaN());
   }

   static void assertTrajectoryPointContainsExpectedData(double expectedTime, Point3d expectedPosition,
         Quat4d expectedOrientation, Vector3d expectedLinearVelocity, Vector3d expectedAngularVelocity,
         SimpleSE3TrajectoryPoint testedSimpleSE3TrajectoryPoint, double epsilon)
   {
      assertEquals(expectedTime, testedSimpleSE3TrajectoryPoint.getTime(), epsilon);
      assertTrue(expectedPosition.epsilonEquals(testedSimpleSE3TrajectoryPoint.getEuclideanWaypoint().getPosition(), epsilon));
      assertTrue(expectedOrientation + ", " + testedSimpleSE3TrajectoryPoint.getSO3Waypoint().getOrientation(), expectedOrientation.epsilonEquals(testedSimpleSE3TrajectoryPoint.getSO3Waypoint().getOrientation(), epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(testedSimpleSE3TrajectoryPoint.getEuclideanWaypoint().getLinearVelocity(), epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(testedSimpleSE3TrajectoryPoint.getSO3Waypoint().getAngularVelocity(), epsilon));

      Point3d actualPosition = new Point3d();
      Quat4d actualOrientation = new TransformableQuat4d();
      Vector3d actualLinearVelocity = new Vector3d();
      Vector3d actualAngularVelocity = new Vector3d();

      testedSimpleSE3TrajectoryPoint.getPosition(actualPosition);
      testedSimpleSE3TrajectoryPoint.getOrientation(actualOrientation);
      testedSimpleSE3TrajectoryPoint.getLinearVelocity(actualLinearVelocity);
      testedSimpleSE3TrajectoryPoint.getAngularVelocity(actualAngularVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualPosition, epsilon));
      assertTrue(expectedOrientation.epsilonEquals(actualOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, epsilon));

      Point3d actualFramePosition = new Point3d();
      Quat4d actualFrameOrientation = new TransformableQuat4d();
      Vector3d actualFrameLinearVelocity = new Vector3d();
      Vector3d actualFrameAngularVelocity = new Vector3d();

      testedSimpleSE3TrajectoryPoint.getPosition(actualFramePosition);
      testedSimpleSE3TrajectoryPoint.getOrientation(actualFrameOrientation);
      testedSimpleSE3TrajectoryPoint.getLinearVelocity(actualFrameLinearVelocity);
      testedSimpleSE3TrajectoryPoint.getAngularVelocity(actualFrameAngularVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));

      actualFramePosition = new Point3d();
      actualFrameOrientation = new TransformableQuat4d();
      actualFrameLinearVelocity = new Vector3d();
      actualFrameAngularVelocity = new Vector3d();

      testedSimpleSE3TrajectoryPoint.getPosition(actualFramePosition);
      testedSimpleSE3TrajectoryPoint.getOrientation(actualFrameOrientation);
      testedSimpleSE3TrajectoryPoint.getLinearVelocity(actualFrameLinearVelocity);
      testedSimpleSE3TrajectoryPoint.getAngularVelocity(actualFrameAngularVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSomeSetsAngGets()
   {
      SimpleSE3TrajectoryPoint simpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();

      SimpleSE3TrajectoryPoint simpleTrajectoryPoint = new SimpleSE3TrajectoryPoint();

      double time = 3.4;
      Point3d position = new Point3d(1.0, 2.1, 3.7);
      TransformableQuat4d orientation = new TransformableQuat4d(new Quat4d(0.1, 0.22, 0.34, 0.56));
      orientation.normalize();

      Vector3d linearVelocity = new Vector3d(-0.4, 1.2, 3.3);
      Vector3d angularVelocity = new Vector3d(1.7, 8.4, 2.2);

      simpleTrajectoryPoint.set(time, position, orientation, linearVelocity, angularVelocity);
      simpleSE3TrajectoryPoint.set(simpleTrajectoryPoint);

      // Check some get calls: 
      Point3d pointForVerification = new Point3d();
      Quat4d quaternionForVerification = new TransformableQuat4d();
      Vector3d linearVelocityForVerification = new Vector3d();
      Vector3d angularVelocityForVerification = new Vector3d();

      simpleSE3TrajectoryPoint.getPosition(pointForVerification);
      simpleSE3TrajectoryPoint.getOrientation(quaternionForVerification);
      simpleSE3TrajectoryPoint.getLinearVelocity(linearVelocityForVerification);
      simpleSE3TrajectoryPoint.getAngularVelocity(angularVelocityForVerification);

      assertEquals(time, simpleSE3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(pointForVerification.epsilonEquals(position, 1e-10));
      assertTrue(quaternionForVerification.epsilonEquals(orientation, 1e-10));
      assertTrue(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-10));
      assertTrue(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-10));

      // Check NaN calls:
      assertFalse(simpleSE3TrajectoryPoint.containsNaN());
      simpleSE3TrajectoryPoint.setPositionToNaN();
      assertTrue(simpleSE3TrajectoryPoint.containsNaN());
      simpleSE3TrajectoryPoint.setPositionToZero();

      assertFalse(simpleSE3TrajectoryPoint.containsNaN());
      simpleSE3TrajectoryPoint.setOrientationToNaN();
      assertTrue(simpleSE3TrajectoryPoint.containsNaN());
      simpleSE3TrajectoryPoint.setOrientationToZero();

      assertFalse(simpleSE3TrajectoryPoint.containsNaN());
      simpleSE3TrajectoryPoint.setLinearVelocityToNaN();
      assertTrue(simpleSE3TrajectoryPoint.containsNaN());
      simpleSE3TrajectoryPoint.setLinearVelocityToZero();

      assertFalse(simpleSE3TrajectoryPoint.containsNaN());
      simpleSE3TrajectoryPoint.setAngularVelocityToNaN();
      assertTrue(simpleSE3TrajectoryPoint.containsNaN());
      simpleSE3TrajectoryPoint.setAngularVelocityToZero();
      assertFalse(simpleSE3TrajectoryPoint.containsNaN());

      simpleSE3TrajectoryPoint.getPosition(position);
      simpleSE3TrajectoryPoint.getOrientation(orientation);
      simpleSE3TrajectoryPoint.getLinearVelocity(linearVelocity);
      simpleSE3TrajectoryPoint.getAngularVelocity(angularVelocity);

      // Make sure they are all equal to zero:
      assertTrue(position.epsilonEquals(new Point3d(), 1e-10));
      assertTrue(orientation.epsilonEquals(new TransformableQuat4d(), 1e-10));
      assertTrue(linearVelocity.epsilonEquals(new Vector3d(), 1e-10));
      assertTrue(angularVelocity.epsilonEquals(new Vector3d(), 1e-10));

      time = 9.9;
      pointForVerification.set(3.9, 2.2, 1.1);
      quaternionForVerification.set(0.2, 0.6, 1.1, 2.1);
      quaternionForVerification.normalize();
      linearVelocityForVerification.set(8.8, 1.4, 9.22);
      angularVelocityForVerification.set(7.1, 2.2, 3.33);

      assertFalse(Math.abs(simpleSE3TrajectoryPoint.getTime() - time) < 1e-7);
      assertFalse(pointForVerification.epsilonEquals(position, 1e-7));
      assertFalse(quaternionForVerification.epsilonEquals(orientation, 1e-7));
      assertFalse(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-7));
      assertFalse(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-7));

      simpleSE3TrajectoryPoint.set(time, pointForVerification, quaternionForVerification, linearVelocityForVerification, angularVelocityForVerification);

      simpleSE3TrajectoryPoint.getPosition(position);
      simpleSE3TrajectoryPoint.getOrientation(orientation);
      simpleSE3TrajectoryPoint.getLinearVelocity(linearVelocity);
      simpleSE3TrajectoryPoint.getAngularVelocity(angularVelocity);

      assertEquals(time, simpleSE3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(pointForVerification.epsilonEquals(position, 1e-10));
      assertTrue(quaternionForVerification.epsilonEquals(orientation, 1e-10));
      assertTrue(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-10));
      assertTrue(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-10));

      SimpleSE3TrajectoryPoint simpleSE3TrajectoryPointTwo = new SimpleSE3TrajectoryPoint();

      double positionDistance = simpleSE3TrajectoryPoint.positionDistance(simpleSE3TrajectoryPointTwo);
      assertEquals(4.610856753359402, positionDistance, 1e-7);
      assertFalse(simpleSE3TrajectoryPoint.epsilonEquals(simpleSE3TrajectoryPointTwo, 1e-7));

      simpleSE3TrajectoryPointTwo.set(simpleSE3TrajectoryPoint);
      positionDistance = simpleSE3TrajectoryPoint.positionDistance(simpleSE3TrajectoryPointTwo);
      assertEquals(0.0, positionDistance, 1e-7);
      assertTrue(simpleSE3TrajectoryPoint.epsilonEquals(simpleSE3TrajectoryPointTwo, 1e-7));

      SimpleSE3TrajectoryPoint simplePoint = new SimpleSE3TrajectoryPoint();
      simpleSE3TrajectoryPoint.get(simplePoint);

      simpleSE3TrajectoryPoint.setToNaN();
      assertTrue(simpleSE3TrajectoryPoint.containsNaN());
      positionDistance = simpleSE3TrajectoryPoint.positionDistance(simpleSE3TrajectoryPointTwo);
      assertTrue(Double.isNaN(positionDistance));
      assertFalse(simpleSE3TrajectoryPoint.epsilonEquals(simpleSE3TrajectoryPointTwo, 1e-7));

      SE3TrajectoryPointInterface<?> trajectoryPointAsInterface = simplePoint;
      simpleSE3TrajectoryPoint.set(trajectoryPointAsInterface);

      positionDistance = simpleSE3TrajectoryPoint.positionDistance(simpleSE3TrajectoryPointTwo);
      assertEquals(0.0, positionDistance, 1e-7);
      
      assertTrue(simpleSE3TrajectoryPoint.epsilonEquals(simpleSE3TrajectoryPointTwo, 1e-7));

      String string = simpleSE3TrajectoryPoint.toString();
      String expectedString = "SE3 trajectory point: (time =  9.90, SE3 waypoint: [position = ( 3.90,  2.20,  1.10), orientation = ( 0.08,  0.24,  0.45,  0.86), linearVelocity = ( 8.80,  1.40,  9.22), angular velocity = ( 7.10,  2.20,  3.33)].)";
      assertEquals(expectedString, string);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSomeMoreSettersAndGetters()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      SimpleSE3TrajectoryPoint simpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();

      double time = 3.4;
      TransformablePoint3d position = new TransformablePoint3d(1.0, 2.1, 3.7);
      TransformableQuat4d orientation = new TransformableQuat4d(new Quat4d(0.1, 0.22, 0.34, 0.56));

      TransformableVector3d linearVelocity = new TransformableVector3d(-0.4, 1.2, 3.3);
      TransformableVector3d angularVelocity = new TransformableVector3d(1.7, 8.4, 2.2);

      simpleSE3TrajectoryPoint.setTime(time);
      simpleSE3TrajectoryPoint.setPosition(position);
      simpleSE3TrajectoryPoint.setOrientation(orientation);
      simpleSE3TrajectoryPoint.setLinearVelocity(linearVelocity);
      simpleSE3TrajectoryPoint.setAngularVelocity(angularVelocity);

      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose(worldFrame));

      FramePoint poseFramePosition = new FramePoint(worldFrame, new Point3d(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameOrientation poseOrientation = new FrameOrientation(worldFrame, new AxisAngle4d(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      simpleSE3TrajectoryPoint.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));

      assertFalse(position.epsilonEquals(simpleSE3TrajectoryPoint.getPositionCopy(), 1e-10));
      assertFalse(orientation.epsilonEquals(simpleSE3TrajectoryPoint.getOrientationCopy(), 1e-10));
      assertFalse(linearVelocity.epsilonEquals(simpleSE3TrajectoryPoint.getLinearVelocityCopy(), 1e-10));
      assertFalse(angularVelocity.epsilonEquals(simpleSE3TrajectoryPoint.getAngularVelocityCopy(), 1e-10));

      position.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));
      orientation.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));
      linearVelocity.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));
      angularVelocity.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));

      assertTrue(position.epsilonEquals(simpleSE3TrajectoryPoint.getPositionCopy(), 1e-10));
      assertTrue(orientation.epsilonEquals(simpleSE3TrajectoryPoint.getOrientationCopy(), 1e-10));
      assertTrue(linearVelocity.epsilonEquals(simpleSE3TrajectoryPoint.getLinearVelocityCopy(), 1e-10));
      assertTrue(angularVelocity.epsilonEquals(simpleSE3TrajectoryPoint.getAngularVelocityCopy(), 1e-10));

      SimpleSE3TrajectoryPoint simpleSE3TrajectoryPointTwo = new SimpleSE3TrajectoryPoint();
      simpleSE3TrajectoryPointTwo.setTime(time);
      simpleSE3TrajectoryPointTwo.setPosition(position);
      simpleSE3TrajectoryPointTwo.setOrientation(orientation);
      simpleSE3TrajectoryPointTwo.setLinearVelocity(linearVelocity);
      simpleSE3TrajectoryPointTwo.setAngularVelocity(angularVelocity);
      assertTrue(simpleSE3TrajectoryPointTwo.epsilonEquals(simpleSE3TrajectoryPoint, 1e-10));

      simpleSE3TrajectoryPointTwo = new SimpleSE3TrajectoryPoint();
      simpleSE3TrajectoryPointTwo.set(time, position, orientation, linearVelocity, angularVelocity);
      assertTrue(simpleSE3TrajectoryPointTwo.epsilonEquals(simpleSE3TrajectoryPoint, 1e-10));

      simpleSE3TrajectoryPointTwo = new SimpleSE3TrajectoryPoint();
      SE3Waypoint simpleSE3Waypoint = new SE3Waypoint();
      simpleSE3TrajectoryPoint.get(simpleSE3Waypoint);
      simpleSE3TrajectoryPointTwo.set(time, simpleSE3Waypoint);
      assertTrue(simpleSE3TrajectoryPointTwo.epsilonEquals(simpleSE3TrajectoryPoint, 1e-10));

      simpleSE3TrajectoryPointTwo = new SimpleSE3TrajectoryPoint();
      simpleSE3TrajectoryPointTwo.set(time, simpleSE3Waypoint);
      assertTrue(simpleSE3TrajectoryPointTwo.epsilonEquals(simpleSE3TrajectoryPoint, 1e-10));

      
      simpleSE3TrajectoryPointTwo = new SimpleSE3TrajectoryPoint();
      EuclideanWaypoint euclideanWaypoint = simpleSE3TrajectoryPoint.getEuclideanWaypoint();
      SO3Waypoint so3Waypoint = simpleSE3TrajectoryPoint.getSO3Waypoint();
      
      simpleSE3TrajectoryPointTwo.set(time, euclideanWaypoint, so3Waypoint);
      assertTrue(simpleSE3TrajectoryPointTwo.epsilonEquals(simpleSE3TrajectoryPoint, 1e-10));
      
      simpleSE3TrajectoryPointTwo = new SimpleSE3TrajectoryPoint();
      euclideanWaypoint = new EuclideanWaypoint();
      so3Waypoint = new SO3Waypoint();
      simpleSE3TrajectoryPoint.get(euclideanWaypoint, so3Waypoint);
      
      simpleSE3TrajectoryPointTwo.set(time, euclideanWaypoint, so3Waypoint);
      assertTrue(simpleSE3TrajectoryPointTwo.epsilonEquals(simpleSE3TrajectoryPoint, 1e-10));

      Point3d positionToPack = new Point3d();
      Quat4d orientationToPack = new Quat4d();
      Vector3d linearVelocityToPack = new Vector3d();
      Vector3d angularVelocityToPack = new Vector3d();
      simpleSE3TrajectoryPoint.get(positionToPack, orientationToPack, linearVelocityToPack, angularVelocityToPack);

      simpleSE3TrajectoryPointTwo = new SimpleSE3TrajectoryPoint();
      simpleSE3TrajectoryPointTwo.set(time, positionToPack, orientationToPack, linearVelocityToPack, angularVelocityToPack);
      assertTrue(simpleSE3TrajectoryPointTwo.epsilonEquals(simpleSE3TrajectoryPoint, 1e-10));


   }
}
