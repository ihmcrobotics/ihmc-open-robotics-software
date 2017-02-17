package us.ihmc.robotics.math.trajectories.waypoints;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.transformables.SO3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3TrajectoryPointInterface;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SimpleSO3TrajectoryPointTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCommonUsageExample()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose(worldFrame));

      FramePoint poseFramePosition = new FramePoint(worldFrame, new Point3D(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameOrientation poseOrientation = new FrameOrientation(worldFrame, new AxisAngle(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      SimpleSO3TrajectoryPoint simpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint();
      SimpleSO3TrajectoryPoint simpleTrajectoryPoint = new SimpleSO3TrajectoryPoint();

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

      SimpleSO3TrajectoryPoint expectedSimpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint();

      expectedSimpleSO3TrajectoryPoint.setTime(time);
      expectedSimpleSO3TrajectoryPoint.setOrientation(orientation);
      expectedSimpleSO3TrajectoryPoint.setAngularVelocity(angularVelocity);

      assertEquals(3.4, simpleSO3TrajectoryPoint.getTime(), 1e-7);
      assertEquals(3.4, expectedSimpleSO3TrajectoryPoint.getTime(), 1e-7);
      assertTrue(expectedSimpleSO3TrajectoryPoint.epsilonEquals(simpleSO3TrajectoryPoint, 1e-10));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructors()
   {
      double epsilon = 1.0e-14;
      Random random = new Random(21651016L);

      double expectedTime = 0.0;
      Quaternion expectedOrientation = new Quaternion();
      Vector3D expectedAngularVelocity = new Vector3D();

      SimpleSO3TrajectoryPoint testedSimpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedOrientation,
            expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

      expectedTime = 0.0;
      expectedOrientation = new Quaternion();
      expectedAngularVelocity = new Vector3D();
      testedSimpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedOrientation,
            expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = RandomTools.generateRandomQuaternion(random);
      expectedAngularVelocity = RandomTools.generateRandomVector(random);

      testedSimpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedOrientation,
            expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = RandomTools.generateRandomQuaternion(random);
      expectedAngularVelocity = RandomTools.generateRandomVector(random);

      SimpleSO3TrajectoryPoint expectedSimpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint(expectedTime, expectedOrientation,
            expectedAngularVelocity);

      testedSimpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint(expectedSimpleSO3TrajectoryPoint);

      assertTrue(expectedSimpleSO3TrajectoryPoint.epsilonEquals(testedSimpleSO3TrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedSimpleSO3TrajectoryPoint.getTime(),
            expectedOrientation, expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

      final double expectedFinalTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      final Quaternion expectedFinalOrientation = RandomTools.generateRandomQuaternion(random);
      final Vector3D expectedFinalAngularVelocity = RandomTools.generateRandomVector(random);

      SimpleSO3TrajectoryPoint expectedSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint();
      expectedSO3TrajectoryPoint.setTime(expectedFinalTime);
      expectedSO3TrajectoryPoint.setOrientation(expectedFinalOrientation);
      expectedSO3TrajectoryPoint.setAngularVelocity(expectedFinalAngularVelocity);

      testedSimpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint(expectedSO3TrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalTime, expectedFinalOrientation,
            expectedFinalAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetters()
   {
      double epsilon = 1.0e-14;
      Random random = new Random(21651016L);

      double expectedTime = 0.0;
      Quaternion expectedOrientation = new Quaternion();
      Vector3D expectedAngularVelocity = new Vector3D();

      final SimpleSO3TrajectoryPoint testedSimpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedOrientation,
            expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = RandomTools.generateRandomQuaternion(random);
      expectedAngularVelocity = RandomTools.generateRandomVector(random);

      testedSimpleSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedOrientation,
            expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = RandomTools.generateRandomQuaternion(random);
      expectedAngularVelocity = RandomTools.generateRandomVector(random);

      testedSimpleSO3TrajectoryPoint.set(expectedTime, expectedOrientation,
            expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedOrientation,
            expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = RandomTools.generateRandomQuaternion(random);
      expectedAngularVelocity = RandomTools.generateRandomVector(random);

      testedSimpleSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedOrientation,
            expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = RandomTools.generateRandomQuaternion(random);
      expectedAngularVelocity = RandomTools.generateRandomVector(random);

      SimpleSO3TrajectoryPoint expectedSimpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint(expectedTime, expectedOrientation,
            expectedAngularVelocity);

      testedSimpleSO3TrajectoryPoint.set(expectedSimpleSO3TrajectoryPoint);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = RandomTools.generateRandomQuaternion(random);
      expectedAngularVelocity = RandomTools.generateRandomVector(random);

      expectedSimpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint(expectedTime, expectedOrientation,
            expectedAngularVelocity);

      testedSimpleSO3TrajectoryPoint.set(expectedSimpleSO3TrajectoryPoint);

      assertTrue(expectedSimpleSO3TrajectoryPoint.epsilonEquals(testedSimpleSO3TrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedSimpleSO3TrajectoryPoint.getTime(),
            expectedOrientation, expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

      final double expectedFinalTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      final Quaternion expectedFinalOrientation = RandomTools.generateRandomQuaternion(random);
      final Vector3D expectedFinalAngularVelocity = RandomTools.generateRandomVector(random);

      SimpleSO3TrajectoryPoint expectedSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint();
      expectedSO3TrajectoryPoint.setTime(expectedFinalTime);
      expectedSO3TrajectoryPoint.setOrientation(expectedFinalOrientation);
      expectedSO3TrajectoryPoint.setAngularVelocity(expectedFinalAngularVelocity);

      testedSimpleSO3TrajectoryPoint.set(expectedSO3TrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalTime, expectedFinalOrientation,
            expectedFinalAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testChangeFrame() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      Quaternion expectedOrientation = new Quaternion(RandomTools.generateRandomQuaternion(random));
      Vector3D expectedAngularVelocity = new Vector3D(RandomTools.generateRandomVector(random));
      SimpleSO3TrajectoryPoint testedSimpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint(expectedTime, expectedOrientation,
            expectedAngularVelocity);

      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = ReferenceFrame.generateRandomReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : expectedFrame);

         expectedOrientation.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));
         expectedAngularVelocity.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));
         testedSimpleSO3TrajectoryPoint.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));

         assertTrajectoryPointContainsExpectedData(expectedTime, expectedOrientation, 
               expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetToZero() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);

      double expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      Quaternion expectedOrientation = RandomTools.generateRandomQuaternion(random);
      Vector3D expectedAngularVelocity = RandomTools.generateRandomVector(random);
      SimpleSO3TrajectoryPoint testedSimpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint(expectedTime, expectedOrientation,
            expectedAngularVelocity);

      expectedTime = 0.0;
      expectedOrientation.set(0.0, 0.0, 0.0, 1.0);
      expectedAngularVelocity.set(0.0, 0.0, 0.0);
      testedSimpleSO3TrajectoryPoint.setToZero();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedOrientation, expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = RandomTools.generateRandomQuaternion(random);
      expectedAngularVelocity = RandomTools.generateRandomVector(random);
      testedSimpleSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedOrientation.set(0.0, 0.0, 0.0, 1.0);
      expectedAngularVelocity.set(0.0, 0.0, 0.0);
      testedSimpleSO3TrajectoryPoint.setToZero();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedOrientation,
            expectedAngularVelocity, testedSimpleSO3TrajectoryPoint, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(21651016L);

      double expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      Quaternion expectedOrientation = RandomTools.generateRandomQuaternion(random);
      Vector3D expectedAngularVelocity = RandomTools.generateRandomVector(random);
      SimpleSO3TrajectoryPoint testedSimpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint(expectedTime, expectedOrientation,
            expectedAngularVelocity);

      testedSimpleSO3TrajectoryPoint.setToNaN();
      assertTrue(Double.isNaN(testedSimpleSO3TrajectoryPoint.getTime()));
      assertTrue(testedSimpleSO3TrajectoryPoint.containsNaN());

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = RandomTools.generateRandomQuaternion(random);
      expectedAngularVelocity = RandomTools.generateRandomVector(random);
      testedSimpleSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedSimpleSO3TrajectoryPoint.setToNaN();

      assertTrue(Double.isNaN(testedSimpleSO3TrajectoryPoint.getTime()));
      assertTrue(testedSimpleSO3TrajectoryPoint.containsNaN());
   }

   static void assertTrajectoryPointContainsExpectedData(double expectedTime, 
         Quaternion expectedOrientation, Vector3D expectedAngularVelocity,
         SimpleSO3TrajectoryPoint testedSimpleSO3TrajectoryPoint, double epsilon)
   {
      assertEquals(expectedTime, testedSimpleSO3TrajectoryPoint.getTime(), epsilon);
      assertTrue(expectedOrientation + ", " + testedSimpleSO3TrajectoryPoint.getSO3Waypoint().getOrientation(), expectedOrientation.epsilonEquals(testedSimpleSO3TrajectoryPoint.getSO3Waypoint().getOrientation(), epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(testedSimpleSO3TrajectoryPoint.getSO3Waypoint().getAngularVelocity(), epsilon));

      Quaternion actualOrientation = new Quaternion();
      Vector3D actualAngularVelocity = new Vector3D();

      testedSimpleSO3TrajectoryPoint.getOrientation(actualOrientation);
      testedSimpleSO3TrajectoryPoint.getAngularVelocity(actualAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, epsilon));

      Quaternion actualFrameOrientation = new Quaternion();
      Vector3D actualFrameAngularVelocity = new Vector3D();

      testedSimpleSO3TrajectoryPoint.getOrientation(actualFrameOrientation);
      testedSimpleSO3TrajectoryPoint.getAngularVelocity(actualFrameAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));

      actualFrameOrientation = new Quaternion();
      actualFrameAngularVelocity = new Vector3D();

      testedSimpleSO3TrajectoryPoint.getOrientation(actualFrameOrientation);
      testedSimpleSO3TrajectoryPoint.getAngularVelocity(actualFrameAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSomeSetsAngGets()
   {
      SimpleSO3TrajectoryPoint simpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint();

      SimpleSO3TrajectoryPoint simpleTrajectoryPoint = new SimpleSO3TrajectoryPoint();

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

      simpleSO3TrajectoryPoint.getOrientation(quaternionForVerification);
      simpleSO3TrajectoryPoint.getAngularVelocity(angularVelocityForVerification);

      assertEquals(time, simpleSO3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(quaternionForVerification.epsilonEquals(orientation, 1e-10));
      assertTrue(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-10));

      // Check NaN calls:
      assertFalse(simpleSO3TrajectoryPoint.containsNaN());
      simpleSO3TrajectoryPoint.setOrientationToNaN();
      assertTrue(simpleSO3TrajectoryPoint.containsNaN());
      simpleSO3TrajectoryPoint.setOrientationToZero();

      assertFalse(simpleSO3TrajectoryPoint.containsNaN());
      simpleSO3TrajectoryPoint.setAngularVelocityToNaN();
      assertTrue(simpleSO3TrajectoryPoint.containsNaN());
      simpleSO3TrajectoryPoint.setAngularVelocityToZero();
      assertFalse(simpleSO3TrajectoryPoint.containsNaN());

      simpleSO3TrajectoryPoint.getOrientation(orientation);
      simpleSO3TrajectoryPoint.getAngularVelocity(angularVelocity);

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

      simpleSO3TrajectoryPoint.getOrientation(orientation);
      simpleSO3TrajectoryPoint.getAngularVelocity(angularVelocity);

      assertEquals(time, simpleSO3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(quaternionForVerification.epsilonEquals(orientation, 1e-10));
      assertTrue(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-10));

      SimpleSO3TrajectoryPoint simpleSO3TrajectoryPointTwo = new SimpleSO3TrajectoryPoint();
      assertFalse(simpleSO3TrajectoryPoint.epsilonEquals(simpleSO3TrajectoryPointTwo, 1e-7));

      simpleSO3TrajectoryPointTwo.set(simpleSO3TrajectoryPoint);
      assertTrue(simpleSO3TrajectoryPoint.epsilonEquals(simpleSO3TrajectoryPointTwo, 1e-7));

      SimpleSO3TrajectoryPoint simplePoint = new SimpleSO3TrajectoryPoint();
      simpleSO3TrajectoryPoint.get(simplePoint);

      simpleSO3TrajectoryPoint.setToNaN();
      assertTrue(simpleSO3TrajectoryPoint.containsNaN());
      assertFalse(simpleSO3TrajectoryPoint.epsilonEquals(simpleSO3TrajectoryPointTwo, 1e-7));

      SO3TrajectoryPointInterface<?> trajectoryPointAsInterface = simplePoint;
      simpleSO3TrajectoryPoint.set(trajectoryPointAsInterface);

      assertTrue(simpleSO3TrajectoryPoint.epsilonEquals(simpleSO3TrajectoryPointTwo, 1e-7));

      String string = simpleSO3TrajectoryPoint.toString();
      String expectedString = "SO3 trajectory point: (time =  9.90, SO3 waypoint: [orientation = ( 0.08,  0.24,  0.45,  0.86), angular velocity = ( 7.10,  2.20,  3.33)].)";
      assertEquals(expectedString, string);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSomeMoreSettersAndGetters()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      SimpleSO3TrajectoryPoint simpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint();

      double time = 3.4;
      Quaternion orientation = new Quaternion(new Quaternion(0.1, 0.22, 0.34, 0.56));
      Vector3D angularVelocity = new Vector3D(1.7, 8.4, 2.2);

      simpleSO3TrajectoryPoint.setTime(time);
      simpleSO3TrajectoryPoint.setOrientation(orientation);
      simpleSO3TrajectoryPoint.setAngularVelocity(angularVelocity);

      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose(worldFrame));

      FramePoint poseFramePosition = new FramePoint(worldFrame, new Point3D(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameOrientation poseOrientation = new FrameOrientation(worldFrame, new AxisAngle(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      simpleSO3TrajectoryPoint.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));

      assertFalse(orientation.epsilonEquals(simpleSO3TrajectoryPoint.getOrientationCopy(), 1e-10));
      assertFalse(angularVelocity.epsilonEquals(simpleSO3TrajectoryPoint.getAngularVelocityCopy(), 1e-10));

      orientation.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));
      angularVelocity.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));

      assertTrue(orientation.epsilonEquals(simpleSO3TrajectoryPoint.getOrientationCopy(), 1e-10));
      assertTrue(angularVelocity.epsilonEquals(simpleSO3TrajectoryPoint.getAngularVelocityCopy(), 1e-10));

      
      SimpleSO3TrajectoryPoint simpleSO3TrajectoryPointTwo = new SimpleSO3TrajectoryPoint();
      simpleSO3TrajectoryPointTwo.setTime(time);
      simpleSO3TrajectoryPointTwo.setOrientation(orientation);
      simpleSO3TrajectoryPointTwo.setAngularVelocity(angularVelocity);
      assertTrue(simpleSO3TrajectoryPointTwo.epsilonEquals(simpleSO3TrajectoryPoint, 1e-10));

      simpleSO3TrajectoryPointTwo = new SimpleSO3TrajectoryPoint();
      simpleSO3TrajectoryPointTwo.set(time, orientation, angularVelocity);
      assertTrue(simpleSO3TrajectoryPointTwo.epsilonEquals(simpleSO3TrajectoryPoint, 1e-10));

      simpleSO3TrajectoryPointTwo = new SimpleSO3TrajectoryPoint();
      SO3Waypoint simpleSO3Waypoint = new SO3Waypoint();
      simpleSO3TrajectoryPoint.get(simpleSO3Waypoint);
      simpleSO3TrajectoryPointTwo.set(time, simpleSO3Waypoint);
      assertTrue(simpleSO3TrajectoryPointTwo.epsilonEquals(simpleSO3TrajectoryPoint, 1e-10));

      simpleSO3TrajectoryPointTwo = new SimpleSO3TrajectoryPoint();
      simpleSO3TrajectoryPointTwo.set(time, simpleSO3Waypoint);
      assertTrue(simpleSO3TrajectoryPointTwo.epsilonEquals(simpleSO3TrajectoryPoint, 1e-10));

      
      simpleSO3TrajectoryPointTwo = new SimpleSO3TrajectoryPoint();
      SO3Waypoint so3Waypoint = simpleSO3TrajectoryPoint.getSO3Waypoint();
      
      simpleSO3TrajectoryPointTwo.set(time, so3Waypoint);
      assertTrue(simpleSO3TrajectoryPointTwo.epsilonEquals(simpleSO3TrajectoryPoint, 1e-10));

      simpleSO3TrajectoryPointTwo = new SimpleSO3TrajectoryPoint();
      so3Waypoint = new SO3Waypoint();
      simpleSO3TrajectoryPoint.get(so3Waypoint);
      
      simpleSO3TrajectoryPointTwo.set(time, so3Waypoint);
      assertTrue(simpleSO3TrajectoryPointTwo.epsilonEquals(simpleSO3TrajectoryPoint, 1e-10));

      Quaternion orientationToPack = new Quaternion();
      Vector3D angularVelocityToPack = new Vector3D();
      simpleSO3TrajectoryPoint.get(orientationToPack, angularVelocityToPack);

      simpleSO3TrajectoryPointTwo = new SimpleSO3TrajectoryPoint();
      simpleSO3TrajectoryPointTwo.set(time, orientationToPack, angularVelocityToPack);
      assertTrue(simpleSO3TrajectoryPointTwo.epsilonEquals(simpleSO3TrajectoryPoint, 1e-10));

      orientationToPack = new Quaternion();
      angularVelocityToPack = new Vector3D();
      simpleSO3TrajectoryPoint.get( orientationToPack, angularVelocityToPack);

      simpleSO3TrajectoryPointTwo = new SimpleSO3TrajectoryPoint();
      simpleSO3TrajectoryPointTwo.set(time, orientationToPack, angularVelocityToPack);
      assertTrue(simpleSO3TrajectoryPointTwo.epsilonEquals(simpleSO3TrajectoryPoint, 1e-10));
      
      assertTrue(simpleSO3TrajectoryPointTwo.getOrientation().epsilonEquals(orientationToPack, 1e-10));
      assertTrue(simpleSO3TrajectoryPointTwo.getAngularVelocity().epsilonEquals(angularVelocityToPack, 1e-10));

   }
}
