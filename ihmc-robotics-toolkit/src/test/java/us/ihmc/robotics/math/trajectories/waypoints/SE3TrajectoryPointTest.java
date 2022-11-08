package us.ihmc.robotics.math.trajectories.waypoints;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SE3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointBasics;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class SE3TrajectoryPointTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testCommonUsageExample()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose3D(worldFrame));

      FramePoint3D poseFramePosition = new FramePoint3D(worldFrame, new Point3D(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameQuaternion poseOrientation = new FrameQuaternion(worldFrame, new AxisAngle(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      SE3TrajectoryPoint simpleSE3TrajectoryPoint = new SE3TrajectoryPoint();
      SE3TrajectoryPoint simpleTrajectoryPoint = new SE3TrajectoryPoint();

      double time = 3.4;
      Point3D position = new Point3D(1.0, 2.1, 3.7);
      Quaternion orientation = new Quaternion(new Quaternion(0.1, 0.22, 0.34, 0.56));
      orientation.normalize();

      Vector3D linearVelocity = new Vector3D(-0.4, 1.2, 3.3);
      Vector3D angularVelocity = new Vector3D(1.7, 8.4, 2.2);

      simpleTrajectoryPoint.set(time, position, orientation, linearVelocity, angularVelocity);
      simpleSE3TrajectoryPoint.set(simpleTrajectoryPoint);
      simpleSE3TrajectoryPoint.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));

      // Do some checks:
      RigidBodyTransform transformToPoseFrame = worldFrame.getTransformToDesiredFrame(poseFrame);
      transformToPoseFrame.transform(position);
      orientation.applyTransform(transformToPoseFrame);
      transformToPoseFrame.transform(linearVelocity);
      transformToPoseFrame.transform(angularVelocity);

      SE3TrajectoryPoint expectedSimpleSE3TrajectoryPoint = new SE3TrajectoryPoint();

      expectedSimpleSE3TrajectoryPoint.setTime(time);
      expectedSimpleSE3TrajectoryPoint.getPosition().set(position);
      expectedSimpleSE3TrajectoryPoint.getOrientation().set((Orientation3DReadOnly) orientation);

      expectedSimpleSE3TrajectoryPoint.getLinearVelocity().set(linearVelocity);
      expectedSimpleSE3TrajectoryPoint.getAngularVelocity().set(angularVelocity);

      assertEquals(3.4, simpleSE3TrajectoryPoint.getTime(), 1e-7);
      assertEquals(3.4, expectedSimpleSE3TrajectoryPoint.getTime(), 1e-7);
      assertTrue(expectedSimpleSE3TrajectoryPoint.epsilonEquals(simpleSE3TrajectoryPoint, 1e-10));
   }

   @Test
   public void testConstructors()
   {
      double epsilon = 1.0e-14;
      Random random = new Random(21651016L);

      double expectedTime = 0.0;
      Point3D expectedPosition = new Point3D();
      Quaternion expectedOrientation = new Quaternion();
      Vector3D expectedLinearVelocity = new Vector3D();
      Vector3D expectedAngularVelocity = new Vector3D();

      SE3TrajectoryPoint testedSimpleSE3TrajectoryPoint = new SE3TrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedTime,
                                                expectedPosition,
                                                expectedOrientation,
                                                expectedLinearVelocity,
                                                expectedAngularVelocity,
                                                testedSimpleSE3TrajectoryPoint,
                                                epsilon);

      expectedTime = 0.0;
      expectedPosition = new Point3D();
      expectedOrientation = new Quaternion();
      expectedLinearVelocity = new Vector3D();
      expectedAngularVelocity = new Vector3D();
      testedSimpleSE3TrajectoryPoint = new SE3TrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedTime,
                                                expectedPosition,
                                                expectedOrientation,
                                                expectedLinearVelocity,
                                                expectedAngularVelocity,
                                                testedSimpleSE3TrajectoryPoint,
                                                epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      expectedOrientation = RandomGeometry.nextQuaternion(random);
      expectedLinearVelocity = RandomGeometry.nextVector3D(random);
      expectedAngularVelocity = RandomGeometry.nextVector3D(random);

      testedSimpleSE3TrajectoryPoint = new SE3TrajectoryPoint(expectedTime,
                                                              expectedPosition,
                                                              expectedOrientation,
                                                              expectedLinearVelocity,
                                                              expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime,
                                                expectedPosition,
                                                expectedOrientation,
                                                expectedLinearVelocity,
                                                expectedAngularVelocity,
                                                testedSimpleSE3TrajectoryPoint,
                                                epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      expectedOrientation = RandomGeometry.nextQuaternion(random);
      expectedLinearVelocity = RandomGeometry.nextVector3D(random);
      expectedAngularVelocity = RandomGeometry.nextVector3D(random);

      SE3TrajectoryPoint expectedSimpleSE3TrajectoryPoint = new SE3TrajectoryPoint(expectedTime,
                                                                                   expectedPosition,
                                                                                   expectedOrientation,
                                                                                   expectedLinearVelocity,
                                                                                   expectedAngularVelocity);

      testedSimpleSE3TrajectoryPoint = new SE3TrajectoryPoint(expectedSimpleSE3TrajectoryPoint);

      assertTrue(expectedSimpleSE3TrajectoryPoint.epsilonEquals(testedSimpleSE3TrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedSimpleSE3TrajectoryPoint.getTime(),
                                                expectedPosition,
                                                expectedOrientation,
                                                expectedLinearVelocity,
                                                expectedAngularVelocity,
                                                testedSimpleSE3TrajectoryPoint,
                                                epsilon);

      final double expectedFinalTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      final Point3D expectedFinalPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      final Quaternion expectedFinalOrientation = RandomGeometry.nextQuaternion(random);
      final Vector3D expectedFinalLinearVelocity = RandomGeometry.nextVector3D(random);
      final Vector3D expectedFinalAngularVelocity = RandomGeometry.nextVector3D(random);

      SE3TrajectoryPoint expectedSE3TrajectoryPoint = new SE3TrajectoryPoint();
      expectedSE3TrajectoryPoint.setTime(expectedFinalTime);
      expectedSE3TrajectoryPoint.getPosition().set(expectedFinalPosition);
      expectedSE3TrajectoryPoint.getOrientation().set((Orientation3DReadOnly) expectedFinalOrientation);
      expectedSE3TrajectoryPoint.getLinearVelocity().set(expectedFinalLinearVelocity);
      expectedSE3TrajectoryPoint.getAngularVelocity().set(expectedFinalAngularVelocity);

      testedSimpleSE3TrajectoryPoint = new SE3TrajectoryPoint(expectedSE3TrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalTime,
                                                expectedFinalPosition,
                                                expectedFinalOrientation,
                                                expectedFinalLinearVelocity,
                                                expectedFinalAngularVelocity,
                                                testedSimpleSE3TrajectoryPoint,
                                                epsilon);

   }

   @Test
   public void testSetters()
   {
      double epsilon = 1.0e-14;
      Random random = new Random(21651016L);

      double expectedTime = 0.0;
      Point3D expectedPosition = new Point3D();
      Quaternion expectedOrientation = new Quaternion();
      Vector3D expectedLinearVelocity = new Vector3D();
      Vector3D expectedAngularVelocity = new Vector3D();

      final SE3TrajectoryPoint testedSimpleSE3TrajectoryPoint = new SE3TrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedTime,
                                                expectedPosition,
                                                expectedOrientation,
                                                expectedLinearVelocity,
                                                expectedAngularVelocity,
                                                testedSimpleSE3TrajectoryPoint,
                                                epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      expectedOrientation = RandomGeometry.nextQuaternion(random);
      expectedLinearVelocity = RandomGeometry.nextVector3D(random);
      expectedAngularVelocity = RandomGeometry.nextVector3D(random);

      testedSimpleSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime,
                                                expectedPosition,
                                                expectedOrientation,
                                                expectedLinearVelocity,
                                                expectedAngularVelocity,
                                                testedSimpleSE3TrajectoryPoint,
                                                epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      expectedOrientation = RandomGeometry.nextQuaternion(random);
      expectedLinearVelocity = RandomGeometry.nextVector3D(random);
      expectedAngularVelocity = RandomGeometry.nextVector3D(random);

      testedSimpleSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime,
                                                expectedPosition,
                                                expectedOrientation,
                                                expectedLinearVelocity,
                                                expectedAngularVelocity,
                                                testedSimpleSE3TrajectoryPoint,
                                                epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      expectedOrientation = RandomGeometry.nextQuaternion(random);
      expectedLinearVelocity = RandomGeometry.nextVector3D(random);
      expectedAngularVelocity = RandomGeometry.nextVector3D(random);

      testedSimpleSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime,
                                                expectedPosition,
                                                expectedOrientation,
                                                expectedLinearVelocity,
                                                expectedAngularVelocity,
                                                testedSimpleSE3TrajectoryPoint,
                                                epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      expectedOrientation = RandomGeometry.nextQuaternion(random);
      expectedLinearVelocity = RandomGeometry.nextVector3D(random);
      expectedAngularVelocity = RandomGeometry.nextVector3D(random);

      SE3TrajectoryPoint expectedSimpleSE3TrajectoryPoint = new SE3TrajectoryPoint(expectedTime,
                                                                                   expectedPosition,
                                                                                   expectedOrientation,
                                                                                   expectedLinearVelocity,
                                                                                   expectedAngularVelocity);

      testedSimpleSE3TrajectoryPoint.set(expectedSimpleSE3TrajectoryPoint);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      expectedOrientation = RandomGeometry.nextQuaternion(random);
      expectedLinearVelocity = RandomGeometry.nextVector3D(random);
      expectedAngularVelocity = RandomGeometry.nextVector3D(random);

      expectedSimpleSE3TrajectoryPoint = new SE3TrajectoryPoint(expectedTime,
                                                                expectedPosition,
                                                                expectedOrientation,
                                                                expectedLinearVelocity,
                                                                expectedAngularVelocity);

      testedSimpleSE3TrajectoryPoint.set(expectedSimpleSE3TrajectoryPoint);

      assertTrue(expectedSimpleSE3TrajectoryPoint.epsilonEquals(testedSimpleSE3TrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedSimpleSE3TrajectoryPoint.getTime(),
                                                expectedPosition,
                                                expectedOrientation,
                                                expectedLinearVelocity,
                                                expectedAngularVelocity,
                                                testedSimpleSE3TrajectoryPoint,
                                                epsilon);

      final double expectedFinalTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      final Point3D expectedFinalPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      final Quaternion expectedFinalOrientation = RandomGeometry.nextQuaternion(random);
      final Vector3D expectedFinalLinearVelocity = RandomGeometry.nextVector3D(random);
      final Vector3D expectedFinalAngularVelocity = RandomGeometry.nextVector3D(random);

      SE3TrajectoryPoint expectedSE3TrajectoryPoint = new SE3TrajectoryPoint();
      expectedSE3TrajectoryPoint.setTime(expectedFinalTime);
      expectedSE3TrajectoryPoint.getPosition().set(expectedFinalPosition);
      expectedSE3TrajectoryPoint.getOrientation().set((Orientation3DReadOnly) expectedFinalOrientation);
      expectedSE3TrajectoryPoint.getLinearVelocity().set(expectedFinalLinearVelocity);
      expectedSE3TrajectoryPoint.getAngularVelocity().set(expectedFinalAngularVelocity);

      testedSimpleSE3TrajectoryPoint.set(expectedSE3TrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalTime,
                                                expectedFinalPosition,
                                                expectedFinalOrientation,
                                                expectedFinalLinearVelocity,
                                                expectedFinalAngularVelocity,
                                                testedSimpleSE3TrajectoryPoint,
                                                epsilon);

   }

   @Test
   public void testChangeFrame() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      Point3D expectedPosition = new Point3D(RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0));
      Quaternion expectedOrientation = new Quaternion(RandomGeometry.nextQuaternion(random));
      Vector3D expectedLinearVelocity = new Vector3D(RandomGeometry.nextVector3D(random));
      Vector3D expectedAngularVelocity = new Vector3D(RandomGeometry.nextVector3D(random));
      SE3TrajectoryPoint testedSimpleSE3TrajectoryPoint = new SE3TrajectoryPoint(expectedTime,
                                                                                 expectedPosition,
                                                                                 expectedOrientation,
                                                                                 expectedLinearVelocity,
                                                                                 expectedAngularVelocity);

      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = EuclidFrameRandomTools.nextReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : expectedFrame);

         expectedPosition.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));
         expectedOrientation.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));
         expectedLinearVelocity.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));
         expectedAngularVelocity.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));
         testedSimpleSE3TrajectoryPoint.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));

         assertTrajectoryPointContainsExpectedData(expectedTime,
                                                   expectedPosition,
                                                   expectedOrientation,
                                                   expectedLinearVelocity,
                                                   expectedAngularVelocity,
                                                   testedSimpleSE3TrajectoryPoint,
                                                   epsilon);
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);

      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      Point3D expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      Quaternion expectedOrientation = RandomGeometry.nextQuaternion(random);
      Vector3D expectedLinearVelocity = RandomGeometry.nextVector3D(random);
      Vector3D expectedAngularVelocity = RandomGeometry.nextVector3D(random);
      SE3TrajectoryPoint testedSimpleSE3TrajectoryPoint = new SE3TrajectoryPoint(expectedTime,
                                                                                 expectedPosition,
                                                                                 expectedOrientation,
                                                                                 expectedLinearVelocity,
                                                                                 expectedAngularVelocity);

      expectedTime = 0.0;
      expectedPosition.set(0.0, 0.0, 0.0);
      expectedOrientation.set(0.0, 0.0, 0.0, 1.0);
      expectedLinearVelocity.set(0.0, 0.0, 0.0);
      expectedAngularVelocity.set(0.0, 0.0, 0.0);
      testedSimpleSE3TrajectoryPoint.setToZero();

      assertTrajectoryPointContainsExpectedData(expectedTime,
                                                expectedPosition,
                                                expectedOrientation,
                                                expectedLinearVelocity,
                                                expectedAngularVelocity,
                                                testedSimpleSE3TrajectoryPoint,
                                                epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      expectedOrientation = RandomGeometry.nextQuaternion(random);
      expectedLinearVelocity = RandomGeometry.nextVector3D(random);
      expectedAngularVelocity = RandomGeometry.nextVector3D(random);
      testedSimpleSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedPosition.set(0.0, 0.0, 0.0);
      expectedOrientation.set(0.0, 0.0, 0.0, 1.0);
      expectedLinearVelocity.set(0.0, 0.0, 0.0);
      expectedAngularVelocity.set(0.0, 0.0, 0.0);
      testedSimpleSE3TrajectoryPoint.setToZero();

      assertTrajectoryPointContainsExpectedData(expectedTime,
                                                expectedPosition,
                                                expectedOrientation,
                                                expectedLinearVelocity,
                                                expectedAngularVelocity,
                                                testedSimpleSE3TrajectoryPoint,
                                                epsilon);
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(21651016L);

      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      Point3D expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      Quaternion expectedOrientation = RandomGeometry.nextQuaternion(random);
      Vector3D expectedLinearVelocity = RandomGeometry.nextVector3D(random);
      Vector3D expectedAngularVelocity = RandomGeometry.nextVector3D(random);
      SE3TrajectoryPoint testedSimpleSE3TrajectoryPoint = new SE3TrajectoryPoint(expectedTime,
                                                                                 expectedPosition,
                                                                                 expectedOrientation,
                                                                                 expectedLinearVelocity,
                                                                                 expectedAngularVelocity);

      testedSimpleSE3TrajectoryPoint.setToNaN();
      assertTrue(Double.isNaN(testedSimpleSE3TrajectoryPoint.getTime()));
      assertTrue(testedSimpleSE3TrajectoryPoint.containsNaN());

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      expectedOrientation = RandomGeometry.nextQuaternion(random);
      expectedLinearVelocity = RandomGeometry.nextVector3D(random);
      expectedAngularVelocity = RandomGeometry.nextVector3D(random);
      testedSimpleSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      testedSimpleSE3TrajectoryPoint.setToNaN();

      assertTrue(Double.isNaN(testedSimpleSE3TrajectoryPoint.getTime()));
      assertTrue(testedSimpleSE3TrajectoryPoint.containsNaN());
   }

   static void assertTrajectoryPointContainsExpectedData(double expectedTime,
                                                         Point3D expectedPosition,
                                                         Quaternion expectedOrientation,
                                                         Vector3D expectedLinearVelocity,
                                                         Vector3D expectedAngularVelocity,
                                                         SE3TrajectoryPoint testedSimpleSE3TrajectoryPoint,
                                                         double epsilon)
   {
      assertEquals(expectedTime, testedSimpleSE3TrajectoryPoint.getTime(), epsilon);
      assertTrue(expectedPosition.epsilonEquals(testedSimpleSE3TrajectoryPoint.getPosition(), epsilon));
      assertTrue(expectedOrientation + ", " + testedSimpleSE3TrajectoryPoint.getOrientation(),
                 expectedOrientation.epsilonEquals(testedSimpleSE3TrajectoryPoint.getOrientation(), epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(testedSimpleSE3TrajectoryPoint.getLinearVelocity(), epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(testedSimpleSE3TrajectoryPoint.getAngularVelocity(), epsilon));

      Point3D actualPosition = new Point3D();
      Quaternion actualOrientation = new Quaternion();
      Vector3D actualLinearVelocity = new Vector3D();
      Vector3D actualAngularVelocity = new Vector3D();

      actualPosition.set(testedSimpleSE3TrajectoryPoint.getPosition());
      actualOrientation.set(testedSimpleSE3TrajectoryPoint.getOrientation());
      actualLinearVelocity.set(testedSimpleSE3TrajectoryPoint.getLinearVelocity());
      actualAngularVelocity.set(testedSimpleSE3TrajectoryPoint.getAngularVelocity());

      assertTrue(expectedPosition.epsilonEquals(actualPosition, epsilon));
      assertTrue(expectedOrientation.epsilonEquals(actualOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, epsilon));

      Point3D actualFramePosition = new Point3D();
      Quaternion actualFrameOrientation = new Quaternion();
      Vector3D actualFrameLinearVelocity = new Vector3D();
      Vector3D actualFrameAngularVelocity = new Vector3D();

      actualFramePosition.set(testedSimpleSE3TrajectoryPoint.getPosition());
      actualFrameOrientation.set(testedSimpleSE3TrajectoryPoint.getOrientation());
      actualFrameLinearVelocity.set(testedSimpleSE3TrajectoryPoint.getLinearVelocity());
      actualFrameAngularVelocity.set(testedSimpleSE3TrajectoryPoint.getAngularVelocity());

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));

      actualFramePosition = new Point3D();
      actualFrameOrientation = new Quaternion();
      actualFrameLinearVelocity = new Vector3D();
      actualFrameAngularVelocity = new Vector3D();

      actualFramePosition.set(testedSimpleSE3TrajectoryPoint.getPosition());
      actualFrameOrientation.set(testedSimpleSE3TrajectoryPoint.getOrientation());
      actualFrameLinearVelocity.set(testedSimpleSE3TrajectoryPoint.getLinearVelocity());
      actualFrameAngularVelocity.set(testedSimpleSE3TrajectoryPoint.getAngularVelocity());

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));
   }

   @Test
   public void testSomeSetsAngGets()
   {
      SE3TrajectoryPoint simpleSE3TrajectoryPoint = new SE3TrajectoryPoint();

      SE3TrajectoryPoint simpleTrajectoryPoint = new SE3TrajectoryPoint();

      double time = 3.4;
      Point3D position = new Point3D(1.0, 2.1, 3.7);
      Quaternion orientation = new Quaternion(new Quaternion(0.1, 0.22, 0.34, 0.56));
      orientation.normalize();

      Vector3D linearVelocity = new Vector3D(-0.4, 1.2, 3.3);
      Vector3D angularVelocity = new Vector3D(1.7, 8.4, 2.2);

      simpleTrajectoryPoint.set(time, position, orientation, linearVelocity, angularVelocity);
      simpleSE3TrajectoryPoint.set(simpleTrajectoryPoint);

      // Check some get calls:
      Point3D pointForVerification = new Point3D();
      Quaternion quaternionForVerification = new Quaternion();
      Vector3D linearVelocityForVerification = new Vector3D();
      Vector3D angularVelocityForVerification = new Vector3D();

      pointForVerification.set(simpleSE3TrajectoryPoint.getPosition());
      quaternionForVerification.set(simpleSE3TrajectoryPoint.getOrientation());
      linearVelocityForVerification.set(simpleSE3TrajectoryPoint.getLinearVelocity());
      angularVelocityForVerification.set(simpleSE3TrajectoryPoint.getAngularVelocity());

      assertEquals(time, simpleSE3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(pointForVerification.epsilonEquals(position, 1e-10));
      assertTrue(quaternionForVerification.epsilonEquals(orientation, 1e-10));
      assertTrue(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-10));
      assertTrue(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-10));

      // Check NaN calls:
      assertFalse(simpleSE3TrajectoryPoint.containsNaN());
      simpleSE3TrajectoryPoint.getPosition().setToNaN();
      assertTrue(simpleSE3TrajectoryPoint.containsNaN());
      simpleSE3TrajectoryPoint.getPosition().setToZero();

      assertFalse(simpleSE3TrajectoryPoint.containsNaN());
      simpleSE3TrajectoryPoint.getOrientation().setToNaN();
      assertTrue(simpleSE3TrajectoryPoint.containsNaN());
      simpleSE3TrajectoryPoint.getOrientation().setToZero();

      assertFalse(simpleSE3TrajectoryPoint.containsNaN());
      simpleSE3TrajectoryPoint.getLinearVelocity().setToNaN();
      assertTrue(simpleSE3TrajectoryPoint.containsNaN());
      simpleSE3TrajectoryPoint.getLinearVelocity().setToZero();

      assertFalse(simpleSE3TrajectoryPoint.containsNaN());
      simpleSE3TrajectoryPoint.getAngularVelocity().setToNaN();
      assertTrue(simpleSE3TrajectoryPoint.containsNaN());
      simpleSE3TrajectoryPoint.getAngularVelocity().setToZero();
      assertFalse(simpleSE3TrajectoryPoint.containsNaN());

      position.set(simpleSE3TrajectoryPoint.getPosition());
      orientation.set(simpleSE3TrajectoryPoint.getOrientation());
      linearVelocity.set(simpleSE3TrajectoryPoint.getLinearVelocity());
      angularVelocity.set(simpleSE3TrajectoryPoint.getAngularVelocity());

      // Make sure they are all equal to zero:
      assertTrue(position.epsilonEquals(new Point3D(), 1e-10));
      assertTrue(orientation.epsilonEquals(new Quaternion(), 1e-10));
      assertTrue(linearVelocity.epsilonEquals(new Vector3D(), 1e-10));
      assertTrue(angularVelocity.epsilonEquals(new Vector3D(), 1e-10));

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

      position.set(simpleSE3TrajectoryPoint.getPosition());
      orientation.set(simpleSE3TrajectoryPoint.getOrientation());
      linearVelocity.set(simpleSE3TrajectoryPoint.getLinearVelocity());
      angularVelocity.set(simpleSE3TrajectoryPoint.getAngularVelocity());

      assertEquals(time, simpleSE3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(pointForVerification.epsilonEquals(position, 1e-10));
      assertTrue(quaternionForVerification.epsilonEquals(orientation, 1e-10));
      assertTrue(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-10));
      assertTrue(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-10));

      SE3TrajectoryPoint simpleSE3TrajectoryPointTwo = new SE3TrajectoryPoint();

      double positionDistance = simpleSE3TrajectoryPoint.positionDistance(simpleSE3TrajectoryPointTwo);
      assertEquals(4.610856753359402, positionDistance, 1e-7);
      assertFalse(simpleSE3TrajectoryPoint.epsilonEquals(simpleSE3TrajectoryPointTwo, 1e-7));

      simpleSE3TrajectoryPointTwo.set(simpleSE3TrajectoryPoint);
      positionDistance = simpleSE3TrajectoryPoint.positionDistance(simpleSE3TrajectoryPointTwo);
      assertEquals(0.0, positionDistance, 1e-7);
      assertTrue(simpleSE3TrajectoryPoint.epsilonEquals(simpleSE3TrajectoryPointTwo, 1e-7));

      SE3TrajectoryPoint simplePoint = new SE3TrajectoryPoint();
      simplePoint.set(simpleSE3TrajectoryPoint);

      simpleSE3TrajectoryPoint.setToNaN();
      assertTrue(simpleSE3TrajectoryPoint.containsNaN());
      positionDistance = simpleSE3TrajectoryPoint.positionDistance(simpleSE3TrajectoryPointTwo);
      assertTrue(Double.isNaN(positionDistance));
      assertFalse(simpleSE3TrajectoryPoint.epsilonEquals(simpleSE3TrajectoryPointTwo, 1e-7));

      SE3TrajectoryPointBasics trajectoryPointAsInterface = simplePoint;
      simpleSE3TrajectoryPoint.set(trajectoryPointAsInterface);

      positionDistance = simpleSE3TrajectoryPoint.positionDistance(simpleSE3TrajectoryPointTwo);
      assertEquals(0.0, positionDistance, 1e-7);

      assertTrue(simpleSE3TrajectoryPoint.epsilonEquals(simpleSE3TrajectoryPointTwo, 1e-7));
   }

   @Test
   public void testSomeMoreSettersAndGetters()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      SE3TrajectoryPoint simpleSE3TrajectoryPoint = new SE3TrajectoryPoint();

      double time = 3.4;
      Point3D position = new Point3D(1.0, 2.1, 3.7);
      Quaternion orientation = new Quaternion(new Quaternion(0.1, 0.22, 0.34, 0.56));

      Vector3D linearVelocity = new Vector3D(-0.4, 1.2, 3.3);
      Vector3D angularVelocity = new Vector3D(1.7, 8.4, 2.2);

      simpleSE3TrajectoryPoint.setTime(time);
      simpleSE3TrajectoryPoint.getPosition().set(position);
      simpleSE3TrajectoryPoint.getOrientation().set((Orientation3DReadOnly) orientation);
      simpleSE3TrajectoryPoint.getLinearVelocity().set(linearVelocity);
      simpleSE3TrajectoryPoint.getAngularVelocity().set(angularVelocity);

      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose3D(worldFrame));

      FramePoint3D poseFramePosition = new FramePoint3D(worldFrame, new Point3D(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameQuaternion poseOrientation = new FrameQuaternion(worldFrame, new AxisAngle(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      simpleSE3TrajectoryPoint.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));

      assertFalse(position.epsilonEquals(simpleSE3TrajectoryPoint.getPosition(), 1e-10));
      assertFalse(orientation.epsilonEquals(simpleSE3TrajectoryPoint.getOrientation(), 1e-10));
      assertFalse(linearVelocity.epsilonEquals(simpleSE3TrajectoryPoint.getLinearVelocity(), 1e-10));
      assertFalse(angularVelocity.epsilonEquals(simpleSE3TrajectoryPoint.getAngularVelocity(), 1e-10));

      position.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));
      orientation.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));
      linearVelocity.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));
      angularVelocity.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));

      assertTrue(position.epsilonEquals(simpleSE3TrajectoryPoint.getPosition(), 1e-10));
      assertTrue(orientation.epsilonEquals(simpleSE3TrajectoryPoint.getOrientation(), 1e-10));
      assertTrue(linearVelocity.epsilonEquals(simpleSE3TrajectoryPoint.getLinearVelocity(), 1e-10));
      assertTrue(angularVelocity.epsilonEquals(simpleSE3TrajectoryPoint.getAngularVelocity(), 1e-10));

      SE3TrajectoryPoint simpleSE3TrajectoryPointTwo = new SE3TrajectoryPoint();
      simpleSE3TrajectoryPointTwo.setTime(time);
      simpleSE3TrajectoryPointTwo.getPosition().set(position);
      simpleSE3TrajectoryPointTwo.getOrientation().set((Orientation3DReadOnly) orientation);
      simpleSE3TrajectoryPointTwo.getLinearVelocity().set(linearVelocity);
      simpleSE3TrajectoryPointTwo.getAngularVelocity().set(angularVelocity);
      assertTrue(simpleSE3TrajectoryPointTwo.epsilonEquals(simpleSE3TrajectoryPoint, 1e-10));

      simpleSE3TrajectoryPointTwo = new SE3TrajectoryPoint();
      simpleSE3TrajectoryPointTwo.set(time, position, orientation, linearVelocity, angularVelocity);
      assertTrue(simpleSE3TrajectoryPointTwo.epsilonEquals(simpleSE3TrajectoryPoint, 1e-10));

      simpleSE3TrajectoryPointTwo = new SE3TrajectoryPoint();
      SE3Waypoint simpleSE3Waypoint = new SE3Waypoint();
      simpleSE3Waypoint.set(simpleSE3TrajectoryPoint);
      simpleSE3TrajectoryPointTwo.set(time, simpleSE3Waypoint);
      assertTrue(simpleSE3TrajectoryPointTwo.epsilonEquals(simpleSE3TrajectoryPoint, 1e-10));

      simpleSE3TrajectoryPointTwo = new SE3TrajectoryPoint();
      simpleSE3TrajectoryPointTwo.set(time, simpleSE3Waypoint);
      assertTrue(simpleSE3TrajectoryPointTwo.epsilonEquals(simpleSE3TrajectoryPoint, 1e-10));

      simpleSE3TrajectoryPointTwo = new SE3TrajectoryPoint();
      EuclideanWaypointBasics euclideanWaypoint = simpleSE3TrajectoryPoint;
      SO3WaypointBasics so3Waypoint = simpleSE3TrajectoryPoint;

      simpleSE3TrajectoryPointTwo.set(time, euclideanWaypoint, so3Waypoint);
      assertTrue(simpleSE3TrajectoryPointTwo.epsilonEquals(simpleSE3TrajectoryPoint, 1e-10));

      simpleSE3TrajectoryPointTwo = new SE3TrajectoryPoint();
      euclideanWaypoint = new EuclideanWaypoint();
      so3Waypoint = new SO3Waypoint();
      euclideanWaypoint.set(simpleSE3TrajectoryPoint.getEuclideanWaypoint());
      so3Waypoint.set(simpleSE3TrajectoryPoint.getSO3Waypoint());

      simpleSE3TrajectoryPointTwo.set(time, euclideanWaypoint, so3Waypoint);
      assertTrue(simpleSE3TrajectoryPointTwo.epsilonEquals(simpleSE3TrajectoryPoint, 1e-10));

      Point3D positionToPack = new Point3D();
      Quaternion orientationToPack = new Quaternion();
      Vector3D linearVelocityToPack = new Vector3D();
      Vector3D angularVelocityToPack = new Vector3D();
      positionToPack.set(simpleSE3TrajectoryPoint.getPosition());
      linearVelocityToPack.set(simpleSE3TrajectoryPoint.getLinearVelocity());
      orientationToPack.set(simpleSE3TrajectoryPoint.getOrientation());
      angularVelocityToPack.set(simpleSE3TrajectoryPoint.getAngularVelocity());

      simpleSE3TrajectoryPointTwo = new SE3TrajectoryPoint();
      simpleSE3TrajectoryPointTwo.set(time, positionToPack, orientationToPack, linearVelocityToPack, angularVelocityToPack);
      assertTrue(simpleSE3TrajectoryPointTwo.epsilonEquals(simpleSE3TrajectoryPoint, 1e-10));

   }
}
