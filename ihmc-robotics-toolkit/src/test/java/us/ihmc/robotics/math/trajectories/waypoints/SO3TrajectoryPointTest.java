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
import us.ihmc.robotics.math.trajectories.trajectorypoints.SO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SO3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointBasics;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.referenceFrame.PoseReferenceFrame;

public class SO3TrajectoryPointTest
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

      SO3TrajectoryPoint simpleSO3TrajectoryPoint = new SO3TrajectoryPoint();
      SO3TrajectoryPoint simpleTrajectoryPoint = new SO3TrajectoryPoint();

      double time = 3.4;
      Quaternion orientation = new Quaternion(new Quaternion(0.1, 0.22, 0.34, 0.56));
      orientation.normalize();
      Vector3D angularVelocity = new Vector3D(1.7, 8.4, 2.2);

      simpleTrajectoryPoint.set(time, orientation, angularVelocity);
      simpleSO3TrajectoryPoint.set(simpleTrajectoryPoint);
      simpleSO3TrajectoryPoint.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));

      // Do some checks:
      RigidBodyTransform transformToPoseFrame = worldFrame.getTransformToDesiredFrame(poseFrame);
      orientation.applyTransform(transformToPoseFrame);
      transformToPoseFrame.transform(angularVelocity);

      SO3TrajectoryPoint expectedSimpleSO3TrajectoryPoint = new SO3TrajectoryPoint();

      expectedSimpleSO3TrajectoryPoint.setTime(time);
      expectedSimpleSO3TrajectoryPoint.getOrientation().set((Orientation3DReadOnly) orientation);
      expectedSimpleSO3TrajectoryPoint.getAngularVelocity().set(angularVelocity);

      assertEquals(3.4, simpleSO3TrajectoryPoint.getTime(), 1e-7);
      assertEquals(3.4, expectedSimpleSO3TrajectoryPoint.getTime(), 1e-7);
      assertTrue(expectedSimpleSO3TrajectoryPoint.epsilonEquals(simpleSO3TrajectoryPoint, 1e-10));
   }

   @Test
   public void testConstructors()
   {
      double epsilon = 1.0e-14;
      Random random = new Random(21651016L);

      double expectedTime = 0.0;
      Quaternion expectedOrientation = new Quaternion();
      Vector3D expectedAngularVelocity = new Vector3D();

      SO3TrajectoryPoint testedSimpleSO3TrajectoryPoint = new SO3TrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedOrientation,
            expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

      expectedTime = 0.0;
      expectedOrientation = new Quaternion();
      expectedAngularVelocity = new Vector3D();
      testedSimpleSO3TrajectoryPoint = new SO3TrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedOrientation,
            expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidCoreRandomTools.nextQuaternion(random);
      expectedAngularVelocity = EuclidCoreRandomTools.nextVector3D(random);

      testedSimpleSO3TrajectoryPoint = new SO3TrajectoryPoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedOrientation,
            expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidCoreRandomTools.nextQuaternion(random);
      expectedAngularVelocity = EuclidCoreRandomTools.nextVector3D(random);

      SO3TrajectoryPoint expectedSimpleSO3TrajectoryPoint = new SO3TrajectoryPoint(expectedTime, expectedOrientation,
            expectedAngularVelocity);

      testedSimpleSO3TrajectoryPoint = new SO3TrajectoryPoint(expectedSimpleSO3TrajectoryPoint);

      assertTrue(expectedSimpleSO3TrajectoryPoint.epsilonEquals(testedSimpleSO3TrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedSimpleSO3TrajectoryPoint.getTime(),
            expectedOrientation, expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

      final double expectedFinalTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      final Quaternion expectedFinalOrientation = EuclidCoreRandomTools.nextQuaternion(random);
      final Vector3D expectedFinalAngularVelocity = EuclidCoreRandomTools.nextVector3D(random);

      SO3TrajectoryPoint expectedSO3TrajectoryPoint = new SO3TrajectoryPoint();
      expectedSO3TrajectoryPoint.setTime(expectedFinalTime);
      expectedSO3TrajectoryPoint.getOrientation().set((Orientation3DReadOnly) expectedFinalOrientation);
      expectedSO3TrajectoryPoint.getAngularVelocity().set(expectedFinalAngularVelocity);

      testedSimpleSO3TrajectoryPoint = new SO3TrajectoryPoint(expectedSO3TrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalTime, expectedFinalOrientation,
            expectedFinalAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

   }

   @Test
   public void testSetters()
   {
      double epsilon = 1.0e-14;
      Random random = new Random(21651016L);

      double expectedTime = 0.0;
      Quaternion expectedOrientation = new Quaternion();
      Vector3D expectedAngularVelocity = new Vector3D();

      final SO3TrajectoryPoint testedSimpleSO3TrajectoryPoint = new SO3TrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedOrientation,
            expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidCoreRandomTools.nextQuaternion(random);
      expectedAngularVelocity = EuclidCoreRandomTools.nextVector3D(random);

      testedSimpleSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedOrientation,
            expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidCoreRandomTools.nextQuaternion(random);
      expectedAngularVelocity = EuclidCoreRandomTools.nextVector3D(random);

      testedSimpleSO3TrajectoryPoint.set(expectedTime, expectedOrientation,
            expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedOrientation,
            expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidCoreRandomTools.nextQuaternion(random);
      expectedAngularVelocity = EuclidCoreRandomTools.nextVector3D(random);

      testedSimpleSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedOrientation,
            expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidCoreRandomTools.nextQuaternion(random);
      expectedAngularVelocity = EuclidCoreRandomTools.nextVector3D(random);

      SO3TrajectoryPoint expectedSimpleSO3TrajectoryPoint = new SO3TrajectoryPoint(expectedTime, expectedOrientation,
            expectedAngularVelocity);

      testedSimpleSO3TrajectoryPoint.set(expectedSimpleSO3TrajectoryPoint);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidCoreRandomTools.nextQuaternion(random);
      expectedAngularVelocity = EuclidCoreRandomTools.nextVector3D(random);

      expectedSimpleSO3TrajectoryPoint = new SO3TrajectoryPoint(expectedTime, expectedOrientation,
            expectedAngularVelocity);

      testedSimpleSO3TrajectoryPoint.set(expectedSimpleSO3TrajectoryPoint);

      assertTrue(expectedSimpleSO3TrajectoryPoint.epsilonEquals(testedSimpleSO3TrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedSimpleSO3TrajectoryPoint.getTime(),
            expectedOrientation, expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

      final double expectedFinalTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      final Quaternion expectedFinalOrientation = EuclidCoreRandomTools.nextQuaternion(random);
      final Vector3D expectedFinalAngularVelocity = EuclidCoreRandomTools.nextVector3D(random);

      SO3TrajectoryPoint expectedSO3TrajectoryPoint = new SO3TrajectoryPoint();
      expectedSO3TrajectoryPoint.setTime(expectedFinalTime);
      expectedSO3TrajectoryPoint.getOrientation().set((Orientation3DReadOnly) expectedFinalOrientation);
      expectedSO3TrajectoryPoint.getAngularVelocity().set(expectedFinalAngularVelocity);

      testedSimpleSO3TrajectoryPoint.set(expectedSO3TrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalTime, expectedFinalOrientation,
            expectedFinalAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

   }

   @Test
   public void testChangeFrame() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      Quaternion expectedOrientation = new Quaternion(EuclidCoreRandomTools.nextQuaternion(random));
      Vector3D expectedAngularVelocity = new Vector3D(EuclidCoreRandomTools.nextVector3D(random));
      SO3TrajectoryPoint testedSimpleSO3TrajectoryPoint = new SO3TrajectoryPoint(expectedTime, expectedOrientation,
            expectedAngularVelocity);

      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = EuclidFrameRandomTools.nextReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : expectedFrame);

         expectedOrientation.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));
         expectedAngularVelocity.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));
         testedSimpleSO3TrajectoryPoint.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));

         assertTrajectoryPointContainsExpectedData(expectedTime, expectedOrientation,
               expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);

      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      Quaternion expectedOrientation = EuclidCoreRandomTools.nextQuaternion(random);
      Vector3D expectedAngularVelocity = EuclidCoreRandomTools.nextVector3D(random);
      SO3TrajectoryPoint testedSimpleSO3TrajectoryPoint = new SO3TrajectoryPoint(expectedTime, expectedOrientation,
            expectedAngularVelocity);

      expectedTime = 0.0;
      expectedOrientation.set(0.0, 0.0, 0.0, 1.0);
      expectedAngularVelocity.set(0.0, 0.0, 0.0);
      testedSimpleSO3TrajectoryPoint.setToZero();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedOrientation, expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidCoreRandomTools.nextQuaternion(random);
      expectedAngularVelocity = EuclidCoreRandomTools.nextVector3D(random);
      testedSimpleSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedOrientation.set(0.0, 0.0, 0.0, 1.0);
      expectedAngularVelocity.set(0.0, 0.0, 0.0);
      testedSimpleSO3TrajectoryPoint.setToZero();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedOrientation,
            expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(21651016L);

      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      Quaternion expectedOrientation = EuclidCoreRandomTools.nextQuaternion(random);
      Vector3D expectedAngularVelocity = EuclidCoreRandomTools.nextVector3D(random);
      SO3TrajectoryPoint testedSimpleSO3TrajectoryPoint = new SO3TrajectoryPoint(expectedTime, expectedOrientation,
            expectedAngularVelocity);

      testedSimpleSO3TrajectoryPoint.setToNaN();
      assertTrue(Double.isNaN(testedSimpleSO3TrajectoryPoint.getTime()));
      assertTrue(testedSimpleSO3TrajectoryPoint.containsNaN());

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidCoreRandomTools.nextQuaternion(random);
      expectedAngularVelocity = EuclidCoreRandomTools.nextVector3D(random);
      testedSimpleSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedSimpleSO3TrajectoryPoint.setToNaN();

      assertTrue(Double.isNaN(testedSimpleSO3TrajectoryPoint.getTime()));
      assertTrue(testedSimpleSO3TrajectoryPoint.containsNaN());
   }

   static void assertTrajectoryPointContainsExpectedData(double expectedTime,
         Quaternion expectedOrientation, Vector3D expectedAngularVelocity,
         SO3TrajectoryPoint testedSimpleSO3TrajectoryPoint, double epsilon)
   {
      assertEquals(expectedTime, testedSimpleSO3TrajectoryPoint.getTime(), epsilon);
      assertTrue(expectedOrientation + ", " + testedSimpleSO3TrajectoryPoint.getOrientation(), expectedOrientation.epsilonEquals(testedSimpleSO3TrajectoryPoint.getOrientation(), epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(testedSimpleSO3TrajectoryPoint.getAngularVelocity(), epsilon));

      Quaternion actualOrientation = new Quaternion();
      Vector3D actualAngularVelocity = new Vector3D();

      actualOrientation.set(testedSimpleSO3TrajectoryPoint.getOrientation());
      actualAngularVelocity.set(testedSimpleSO3TrajectoryPoint.getAngularVelocity());

      assertTrue(expectedOrientation.epsilonEquals(actualOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, epsilon));

      Quaternion actualFrameOrientation = new Quaternion();
      Vector3D actualFrameAngularVelocity = new Vector3D();

      actualFrameOrientation.set(testedSimpleSO3TrajectoryPoint.getOrientation());
      actualFrameAngularVelocity.set(testedSimpleSO3TrajectoryPoint.getAngularVelocity());

      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));

      actualFrameOrientation = new Quaternion();
      actualFrameAngularVelocity = new Vector3D();

      actualFrameOrientation.set(testedSimpleSO3TrajectoryPoint.getOrientation());
      actualFrameAngularVelocity.set(testedSimpleSO3TrajectoryPoint.getAngularVelocity());

      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));
   }

   @Test
   public void testSomeSetsAngGets()
   {
      SO3TrajectoryPoint simpleSO3TrajectoryPoint = new SO3TrajectoryPoint();

      SO3TrajectoryPoint simpleTrajectoryPoint = new SO3TrajectoryPoint();

      double time = 3.4;
      Quaternion orientation = new Quaternion(new Quaternion(0.1, 0.22, 0.34, 0.56));
      orientation.normalize();
      Vector3D angularVelocity = new Vector3D(1.7, 8.4, 2.2);

      simpleTrajectoryPoint.set(time, orientation, angularVelocity);
      simpleSO3TrajectoryPoint.set(simpleTrajectoryPoint);

      // Check some get calls:
      Point3D pointForVerification = new Point3D();
      Quaternion quaternionForVerification = new Quaternion();
      Vector3D linearVelocityForVerification = new Vector3D();
      Vector3D angularVelocityForVerification = new Vector3D();

      quaternionForVerification.set(simpleSO3TrajectoryPoint.getOrientation());
      angularVelocityForVerification.set(simpleSO3TrajectoryPoint.getAngularVelocity());

      assertEquals(time, simpleSO3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(quaternionForVerification.epsilonEquals(orientation, 1e-10));
      assertTrue(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-10));

      // Check NaN calls:
      assertFalse(simpleSO3TrajectoryPoint.containsNaN());
      simpleSO3TrajectoryPoint.getOrientation().setToNaN();
      assertTrue(simpleSO3TrajectoryPoint.containsNaN());
      simpleSO3TrajectoryPoint.getOrientation().setToZero();

      assertFalse(simpleSO3TrajectoryPoint.containsNaN());
      simpleSO3TrajectoryPoint.getAngularVelocity().setToNaN();
      assertTrue(simpleSO3TrajectoryPoint.containsNaN());
      simpleSO3TrajectoryPoint.getAngularVelocity().setToZero();
      assertFalse(simpleSO3TrajectoryPoint.containsNaN());

      orientation.set(simpleSO3TrajectoryPoint.getOrientation());
      angularVelocity.set(simpleSO3TrajectoryPoint.getAngularVelocity());

      // Make sure they are all equal to zero:
      assertTrue(orientation.epsilonEquals(new Quaternion(), 1e-10));
      assertTrue(angularVelocity.epsilonEquals(new Vector3D(), 1e-10));

      time = 9.9;
      pointForVerification.set(3.9, 2.2, 1.1);
      quaternionForVerification.set(0.2, 0.6, 1.1, 2.1);
      quaternionForVerification.normalize();
      linearVelocityForVerification.set(8.8, 1.4, 9.22);
      angularVelocityForVerification.set(7.1, 2.2, 3.33);

      assertFalse(Math.abs(simpleSO3TrajectoryPoint.getTime() - time) < 1e-7);
      assertFalse(quaternionForVerification.epsilonEquals(orientation, 1e-7));
      assertFalse(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-7));

      simpleSO3TrajectoryPoint.set(time, quaternionForVerification, angularVelocityForVerification);

      orientation.set(simpleSO3TrajectoryPoint.getOrientation());
      angularVelocity.set(simpleSO3TrajectoryPoint.getAngularVelocity());

      assertEquals(time, simpleSO3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(quaternionForVerification.epsilonEquals(orientation, 1e-10));
      assertTrue(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-10));

      SO3TrajectoryPoint simpleSO3TrajectoryPointTwo = new SO3TrajectoryPoint();
      assertFalse(simpleSO3TrajectoryPoint.epsilonEquals(simpleSO3TrajectoryPointTwo, 1e-7));

      simpleSO3TrajectoryPointTwo.set(simpleSO3TrajectoryPoint);
      assertTrue(simpleSO3TrajectoryPoint.epsilonEquals(simpleSO3TrajectoryPointTwo, 1e-7));

      SO3TrajectoryPoint simplePoint = new SO3TrajectoryPoint();
      simplePoint.set(simpleSO3TrajectoryPoint);

      simpleSO3TrajectoryPoint.setToNaN();
      assertTrue(simpleSO3TrajectoryPoint.containsNaN());
      assertFalse(simpleSO3TrajectoryPoint.epsilonEquals(simpleSO3TrajectoryPointTwo, 1e-7));

      SO3TrajectoryPointBasics trajectoryPointAsInterface = simplePoint;
      simpleSO3TrajectoryPoint.set(trajectoryPointAsInterface);

      assertTrue(simpleSO3TrajectoryPoint.epsilonEquals(simpleSO3TrajectoryPointTwo, 1e-7));
   }

   @Test
   public void testSomeMoreSettersAndGetters()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      SO3TrajectoryPoint simpleSO3TrajectoryPoint = new SO3TrajectoryPoint();

      double time = 3.4;
      Quaternion orientation = new Quaternion(new Quaternion(0.1, 0.22, 0.34, 0.56));
      Vector3D angularVelocity = new Vector3D(1.7, 8.4, 2.2);

      simpleSO3TrajectoryPoint.setTime(time);
      simpleSO3TrajectoryPoint.getOrientation().set((Orientation3DReadOnly) orientation);
      simpleSO3TrajectoryPoint.getAngularVelocity().set(angularVelocity);

      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose3D(worldFrame));

      FramePoint3D poseFramePosition = new FramePoint3D(worldFrame, new Point3D(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameQuaternion poseOrientation = new FrameQuaternion(worldFrame, new AxisAngle(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      simpleSO3TrajectoryPoint.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));

      assertFalse(orientation.epsilonEquals(simpleSO3TrajectoryPoint.getOrientation(), 1e-10));
      assertFalse(angularVelocity.epsilonEquals(simpleSO3TrajectoryPoint.getAngularVelocity(), 1e-10));

      orientation.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));
      angularVelocity.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));

      assertTrue(orientation.epsilonEquals(simpleSO3TrajectoryPoint.getOrientation(), 1e-10));
      assertTrue(angularVelocity.epsilonEquals(simpleSO3TrajectoryPoint.getAngularVelocity(), 1e-10));


      SO3TrajectoryPoint simpleSO3TrajectoryPointTwo = new SO3TrajectoryPoint();
      simpleSO3TrajectoryPointTwo.setTime(time);
      simpleSO3TrajectoryPointTwo.getOrientation().set((Orientation3DReadOnly) orientation);
      simpleSO3TrajectoryPointTwo.getAngularVelocity().set(angularVelocity);
      assertTrue(simpleSO3TrajectoryPointTwo.epsilonEquals(simpleSO3TrajectoryPoint, 1e-10));

      simpleSO3TrajectoryPointTwo = new SO3TrajectoryPoint();
      simpleSO3TrajectoryPointTwo.set(time, orientation, angularVelocity);
      assertTrue(simpleSO3TrajectoryPointTwo.epsilonEquals(simpleSO3TrajectoryPoint, 1e-10));

      simpleSO3TrajectoryPointTwo = new SO3TrajectoryPoint();
      SO3Waypoint simpleSO3Waypoint = new SO3Waypoint();
      simpleSO3Waypoint.set(simpleSO3TrajectoryPoint);
      simpleSO3TrajectoryPointTwo.set(time, simpleSO3Waypoint);
      assertTrue(simpleSO3TrajectoryPointTwo.epsilonEquals(simpleSO3TrajectoryPoint, 1e-10));

      simpleSO3TrajectoryPointTwo = new SO3TrajectoryPoint();
      simpleSO3TrajectoryPointTwo.set(time, simpleSO3Waypoint);
      assertTrue(simpleSO3TrajectoryPointTwo.epsilonEquals(simpleSO3TrajectoryPoint, 1e-10));


      simpleSO3TrajectoryPointTwo = new SO3TrajectoryPoint();
      SO3WaypointBasics so3Waypoint = simpleSO3TrajectoryPoint;

      simpleSO3TrajectoryPointTwo.set(time, so3Waypoint);
      assertTrue(simpleSO3TrajectoryPointTwo.epsilonEquals(simpleSO3TrajectoryPoint, 1e-10));

      simpleSO3TrajectoryPointTwo = new SO3TrajectoryPoint();
      so3Waypoint = new SO3Waypoint();
      so3Waypoint.set(simpleSO3TrajectoryPoint);

      simpleSO3TrajectoryPointTwo.set(time, so3Waypoint);
      assertTrue(simpleSO3TrajectoryPointTwo.epsilonEquals(simpleSO3TrajectoryPoint, 1e-10));

      Quaternion orientationToPack = new Quaternion();
      Vector3D angularVelocityToPack = new Vector3D();
      orientationToPack.set(simpleSO3TrajectoryPoint.getOrientation());
      angularVelocityToPack.set(simpleSO3TrajectoryPoint.getAngularVelocity());

      simpleSO3TrajectoryPointTwo = new SO3TrajectoryPoint();
      simpleSO3TrajectoryPointTwo.set(time, orientationToPack, angularVelocityToPack);
      assertTrue(simpleSO3TrajectoryPointTwo.epsilonEquals(simpleSO3TrajectoryPoint, 1e-10));

      orientationToPack = new Quaternion();
      angularVelocityToPack = new Vector3D();
      orientationToPack.set(simpleSO3TrajectoryPoint.getOrientation());
      angularVelocityToPack.set(simpleSO3TrajectoryPoint.getAngularVelocity());

      simpleSO3TrajectoryPointTwo = new SO3TrajectoryPoint();
      simpleSO3TrajectoryPointTwo.set(time, orientationToPack, angularVelocityToPack);
      assertTrue(simpleSO3TrajectoryPointTwo.epsilonEquals(simpleSO3TrajectoryPoint, 1e-10));

      assertTrue(simpleSO3TrajectoryPointTwo.getOrientation().epsilonEquals(orientationToPack, 1e-10));
      assertTrue(simpleSO3TrajectoryPointTwo.getAngularVelocity().epsilonEquals(angularVelocityToPack, 1e-10));

   }
}
