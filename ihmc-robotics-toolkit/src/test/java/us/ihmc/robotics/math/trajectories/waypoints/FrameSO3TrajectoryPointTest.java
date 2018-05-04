package us.ihmc.robotics.math.trajectories.waypoints;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.frameObjects.FrameSO3Waypoint;
import us.ihmc.robotics.geometry.transformables.SO3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3TrajectoryPointInterface;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class FrameSO3TrajectoryPointTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCommonUsageExample()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose3D(worldFrame));

      FramePoint3D poseFramePosition = new FramePoint3D(worldFrame, new Point3D(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameQuaternion poseOrientation = new FrameQuaternion(worldFrame, new AxisAngle(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      FrameSO3TrajectoryPoint frameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(worldFrame);
      SimpleSO3TrajectoryPoint simpleTrajectoryPoint = new SimpleSO3TrajectoryPoint();

      double time = 3.4;
      Quaternion orientation = new Quaternion(new Quaternion(0.1, 0.22, 0.34, 0.56));
      orientation.normalize();

      Vector3D angularVelocity = new Vector3D(1.7, 8.4, 2.2);

      simpleTrajectoryPoint.set(time, orientation, angularVelocity);
      frameSO3TrajectoryPoint.setIncludingFrame(worldFrame, simpleTrajectoryPoint);
      frameSO3TrajectoryPoint.changeFrame(poseFrame);

      // Do some checks:
      RigidBodyTransform transformToPoseFrame = worldFrame.getTransformToDesiredFrame(poseFrame);
      orientation.applyTransform(transformToPoseFrame);
      transformToPoseFrame.transform(angularVelocity);

      FrameSO3TrajectoryPoint expectedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(poseFrame);

      expectedFrameSO3TrajectoryPoint.setTime(time);
      expectedFrameSO3TrajectoryPoint.setOrientation(orientation);

      expectedFrameSO3TrajectoryPoint.setAngularVelocity(angularVelocity);

      assertEquals(3.4, frameSO3TrajectoryPoint.getTime(), 1e-7);
      assertEquals(3.4, expectedFrameSO3TrajectoryPoint.getTime(), 1e-7);
      assertTrue(expectedFrameSO3TrajectoryPoint.epsilonEquals(frameSO3TrajectoryPoint, 1e-10));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructors()
   {
      double epsilon = 1.0e-20;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame aFrame = EuclidFrameRandomTools.nextReferenceFrame("aFrame", random, worldFrame);

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FrameQuaternion expectedOrientation = new FrameQuaternion(expectedFrame);
      FrameVector3D expectedAngularVelocity = new FrameVector3D(expectedFrame);

      FrameSO3TrajectoryPoint testedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint,
                                                epsilon);

      expectedFrame = aFrame;
      expectedTime = 0.0;
      expectedOrientation = new FrameQuaternion(expectedFrame);
      expectedAngularVelocity = new FrameVector3D(expectedFrame);
      testedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedFrame);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint,
                                                epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      testedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint,
                                                epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      FrameSO3TrajectoryPoint expectedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedFrameSO3TrajectoryPoint);

      assertTrue(expectedFrameSO3TrajectoryPoint.epsilonEquals(testedFrameSO3TrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedFrameSO3TrajectoryPoint.getReferenceFrame(), expectedFrameSO3TrajectoryPoint.getTime(),
                                                expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint, epsilon);

      final ReferenceFrame expectedFinalFrame = aFrame;
      final double expectedFinalTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      final FrameQuaternion expectedFinalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFinalFrame);
      final FrameVector3D expectedFinalAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFinalFrame);

      SimpleSO3TrajectoryPoint expectedSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint();
      expectedSO3TrajectoryPoint.setTime(expectedFinalTime);
      expectedSO3TrajectoryPoint.setOrientation(expectedFinalOrientation);
      expectedSO3TrajectoryPoint.setAngularVelocity(expectedFinalAngularVelocity);

      testedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedFinalFrame, expectedSO3TrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalFrame, expectedFinalTime, expectedFinalOrientation, expectedFinalAngularVelocity,
                                                testedFrameSO3TrajectoryPoint, epsilon);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetters()
   {
      double epsilon = 1.0e-15;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame aFrame = EuclidFrameRandomTools.nextReferenceFrame("aFrame", random, worldFrame);

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FrameQuaternion expectedOrientation = new FrameQuaternion(expectedFrame);
      FrameVector3D expectedAngularVelocity = new FrameVector3D(expectedFrame);

      final FrameSO3TrajectoryPoint testedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint,
                                                epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      testedFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint,
                                                epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      testedFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint,
                                                epsilon);

      expectedFrame = aFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      testedFrameSO3TrajectoryPoint.setIncludingFrame(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint,
                                                epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      FrameSO3TrajectoryPoint expectedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedFrameSO3TrajectoryPoint.setIncludingFrame(expectedFrameSO3TrajectoryPoint);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      expectedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedFrameSO3TrajectoryPoint.set(expectedFrameSO3TrajectoryPoint);

      assertTrue(expectedFrameSO3TrajectoryPoint.epsilonEquals(testedFrameSO3TrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedFrameSO3TrajectoryPoint.getReferenceFrame(), expectedFrameSO3TrajectoryPoint.getTime(),
                                                expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint, epsilon);

      final ReferenceFrame expectedFinalFrame = aFrame;
      final double expectedFinalTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      final FrameQuaternion expectedFinalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFinalFrame);
      final FrameVector3D expectedFinalAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFinalFrame);

      SimpleSO3TrajectoryPoint expectedSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint();
      expectedSO3TrajectoryPoint.setTime(expectedFinalTime);
      expectedSO3TrajectoryPoint.setOrientation(expectedFinalOrientation);
      expectedSO3TrajectoryPoint.setAngularVelocity(expectedFinalAngularVelocity);

      testedFrameSO3TrajectoryPoint.setIncludingFrame(expectedFinalFrame, expectedSO3TrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalFrame, expectedFinalTime, expectedFinalOrientation, expectedFinalAngularVelocity,
                                                testedFrameSO3TrajectoryPoint, epsilon);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testChangeFrame() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      FrameQuaternion expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      FrameVector3D expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      FrameSO3TrajectoryPoint testedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = EuclidFrameRandomTools.nextReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : expectedFrame);

         expectedOrientation.changeFrame(expectedFrame);
         expectedAngularVelocity.changeFrame(expectedFrame);
         testedFrameSO3TrajectoryPoint.changeFrame(expectedFrame);

         assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint,
                                                   epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetToZero() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      FrameQuaternion expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      FrameVector3D expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      FrameSO3TrajectoryPoint testedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedOrientation.setToZero();
      expectedAngularVelocity.setToZero();
      testedFrameSO3TrajectoryPoint.setToZero();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint,
                                                epsilon);

      expectedFrame = EuclidFrameRandomTools.nextReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
      testedFrameSO3TrajectoryPoint.setIncludingFrame(expectedTime, expectedOrientation, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedOrientation.setToZero(expectedFrame);
      expectedAngularVelocity.setToZero(expectedFrame);
      testedFrameSO3TrajectoryPoint.setToZero(expectedFrame);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3TrajectoryPoint,
                                                epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      FrameQuaternion expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      FrameVector3D expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      FrameSO3TrajectoryPoint testedFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedFrameSO3TrajectoryPoint.setToNaN();
      assertTrue(Double.isNaN(testedFrameSO3TrajectoryPoint.getTime()));
      assertTrue(testedFrameSO3TrajectoryPoint.containsNaN());

      expectedFrame = EuclidFrameRandomTools.nextReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
      testedFrameSO3TrajectoryPoint.setIncludingFrame(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedFrameSO3TrajectoryPoint.setToNaN(expectedFrame);

      assertTrue(expectedFrame == testedFrameSO3TrajectoryPoint.getReferenceFrame());
      assertTrue(Double.isNaN(testedFrameSO3TrajectoryPoint.getTime()));
      assertTrue(testedFrameSO3TrajectoryPoint.containsNaN());
   }

   static void assertTrajectoryPointContainsExpectedData(ReferenceFrame expectedFrame, double expectedTime, FrameQuaternionReadOnly expectedOrientation,
                                                         FrameVector3DReadOnly expectedAngularVelocity, FrameSO3TrajectoryPoint testedFrameSO3TrajectoryPoint,
                                                         double epsilon)
   {
      assertTrue(expectedFrame == testedFrameSO3TrajectoryPoint.getReferenceFrame());
      assertEquals(expectedTime, testedFrameSO3TrajectoryPoint.getTime(), epsilon);
      assertTrue(expectedOrientation.epsilonEquals(testedFrameSO3TrajectoryPoint.getGeometryObject().getOrientation(), epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(testedFrameSO3TrajectoryPoint.getGeometryObject().getAngularVelocity(), epsilon));

      Quaternion actualOrientation = new Quaternion();
      Vector3D actualAngularVelocity = new Vector3D();

      testedFrameSO3TrajectoryPoint.getOrientation(actualOrientation);
      testedFrameSO3TrajectoryPoint.getAngularVelocity(actualAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, epsilon));

      FrameQuaternion actualFrameOrientation = new FrameQuaternion();
      FrameVector3D actualFrameAngularVelocity = new FrameVector3D();

      testedFrameSO3TrajectoryPoint.getOrientationIncludingFrame(actualFrameOrientation);
      testedFrameSO3TrajectoryPoint.getAngularVelocityIncludingFrame(actualFrameAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));

      actualFrameOrientation = new FrameQuaternion(expectedFrame);
      actualFrameAngularVelocity = new FrameVector3D(expectedFrame);

      testedFrameSO3TrajectoryPoint.getOrientation(actualFrameOrientation);
      testedFrameSO3TrajectoryPoint.getAngularVelocity(actualFrameAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSomeSetsAngGets()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FrameSO3TrajectoryPoint FrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(worldFrame);
      FrameSO3TrajectoryPoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      SimpleSO3TrajectoryPoint simpleTrajectoryPoint = new SimpleSO3TrajectoryPoint();

      double time = 3.4;
      Quaternion orientation = new Quaternion(new Quaternion(0.1, 0.22, 0.34, 0.56));
      orientation.normalize();

      Vector3D angularVelocity = new Vector3D(1.7, 8.4, 2.2);

      simpleTrajectoryPoint.set(time, orientation, angularVelocity);
      FrameSO3TrajectoryPoint.setIncludingFrame(worldFrame, simpleTrajectoryPoint);

      // Check some get calls: 
      FrameQuaternion quaternionForVerification = new FrameQuaternion(worldFrame);
      FrameVector3D angularVelocityForVerification = new FrameVector3D(worldFrame);

      FrameSO3TrajectoryPoint.getOrientation(quaternionForVerification);
      FrameSO3TrajectoryPoint.getAngularVelocity(angularVelocityForVerification);

      assertEquals(time, FrameSO3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(quaternionForVerification.epsilonEquals(orientation, 1e-10));
      assertTrue(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-10));

      // Check NaN calls:
      assertFalse(FrameSO3TrajectoryPoint.containsNaN());
      FrameSO3TrajectoryPoint.setOrientationToNaN();
      assertTrue(FrameSO3TrajectoryPoint.containsNaN());
      FrameSO3TrajectoryPoint.setOrientationToZero();

      assertFalse(FrameSO3TrajectoryPoint.containsNaN());
      FrameSO3TrajectoryPoint.setAngularVelocityToNaN();
      assertTrue(FrameSO3TrajectoryPoint.containsNaN());
      FrameSO3TrajectoryPoint.setAngularVelocityToZero();
      assertFalse(FrameSO3TrajectoryPoint.containsNaN());

      FrameSO3TrajectoryPoint.getOrientation(orientation);
      FrameSO3TrajectoryPoint.getAngularVelocity(angularVelocity);

      // Make sure they are all equal to zero:
      assertTrue(orientation.epsilonEquals(new Quaternion(), 1e-10));
      assertTrue(angularVelocity.epsilonEquals(new Vector3D(), 1e-10));

      time = 9.9;
      quaternionForVerification.setYawPitchRoll(0.2, 0.6, 1.1);
      angularVelocityForVerification.set(7.1, 2.2, 3.33);

      assertFalse(Math.abs(FrameSO3TrajectoryPoint.getTime() - time) < 1e-7);
      assertFalse(quaternionForVerification.epsilonEquals(orientation, 1e-7));
      assertFalse(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-7));

      FrameSO3TrajectoryPoint.set(time, quaternionForVerification, angularVelocityForVerification);

      FrameSO3TrajectoryPoint.getOrientation(orientation);
      FrameSO3TrajectoryPoint.getAngularVelocity(angularVelocity);

      assertEquals(time, FrameSO3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(quaternionForVerification.epsilonEquals(orientation, 1e-10));
      assertTrue(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-10));

      FrameSO3TrajectoryPoint frameSO3TrajectoryPointTwo = new FrameSO3TrajectoryPoint(worldFrame);
      assertFalse(FrameSO3TrajectoryPoint.epsilonEquals(frameSO3TrajectoryPointTwo, 1e-7));

      frameSO3TrajectoryPointTwo.set(FrameSO3TrajectoryPoint);
      assertTrue(FrameSO3TrajectoryPoint.epsilonEquals(frameSO3TrajectoryPointTwo, 1e-7));

      SimpleSO3TrajectoryPoint simplePoint = new SimpleSO3TrajectoryPoint();
      FrameSO3TrajectoryPoint.get(simplePoint);

      FrameSO3TrajectoryPoint.setToNaN();
      assertTrue(FrameSO3TrajectoryPoint.containsNaN());
      assertFalse(FrameSO3TrajectoryPoint.epsilonEquals(frameSO3TrajectoryPointTwo, 1e-7));

      SO3TrajectoryPointInterface<?> trajectoryPointAsInterface = simplePoint;
      FrameSO3TrajectoryPoint.set(trajectoryPointAsInterface);

      assertTrue(FrameSO3TrajectoryPoint.epsilonEquals(frameSO3TrajectoryPointTwo, 1e-7));

      String string = FrameSO3TrajectoryPoint.toString();
      String expectedString = "SO3 trajectory point: (time =  9.90, SO3 waypoint: [orientation = ( 0.472,  0.301, -0.072,  0.826), angular velocity = ( 7.100,  2.200,  3.330)].)-World";
      assertEquals(expectedString, string);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSomeMoreSettersAndGetters()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FrameSO3TrajectoryPoint frameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(worldFrame);
      frameSO3TrajectoryPoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      double time = 3.4;
      FramePoint3D position = new FramePoint3D(worldFrame, 1.0, 2.1, 3.7);
      FrameQuaternion orientation = new FrameQuaternion(worldFrame, new Quaternion(0.1, 0.22, 0.34, 0.56));

      FrameVector3D linearVelocity = new FrameVector3D(worldFrame, -0.4, 1.2, 3.3);
      FrameVector3D angularVelocity = new FrameVector3D(worldFrame, 1.7, 8.4, 2.2);

      frameSO3TrajectoryPoint.setTime(time);
      frameSO3TrajectoryPoint.setOrientation(orientation);
      frameSO3TrajectoryPoint.setAngularVelocity(angularVelocity);

      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose3D(worldFrame));

      FramePoint3D poseFramePosition = new FramePoint3D(worldFrame, new Point3D(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameQuaternion poseOrientation = new FrameQuaternion(worldFrame, new AxisAngle(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      frameSO3TrajectoryPoint.changeFrame(poseFrame);

      assertFalse(orientation.epsilonEquals(frameSO3TrajectoryPoint.getOrientationCopy(), 1e-10));
      assertFalse(angularVelocity.epsilonEquals(frameSO3TrajectoryPoint.getAngularVelocityCopy(), 1e-10));

      position.changeFrame(poseFrame);
      orientation.changeFrame(poseFrame);
      linearVelocity.changeFrame(poseFrame);
      angularVelocity.changeFrame(poseFrame);

      assertTrue(orientation.epsilonEquals(frameSO3TrajectoryPoint.getOrientationCopy(), 1e-10));
      assertTrue(angularVelocity.epsilonEquals(frameSO3TrajectoryPoint.getAngularVelocityCopy(), 1e-10));

      FrameSO3TrajectoryPoint frameSO3TrajectoryPointTwo = new FrameSO3TrajectoryPoint(poseFrame);
      frameSO3TrajectoryPointTwo.setTime(time);
      frameSO3TrajectoryPointTwo.setOrientation(orientation);
      frameSO3TrajectoryPointTwo.setAngularVelocity(angularVelocity);
      assertTrue(frameSO3TrajectoryPointTwo.epsilonEquals(frameSO3TrajectoryPointTwo, 1e-10));

      frameSO3TrajectoryPointTwo = new FrameSO3TrajectoryPoint(worldFrame);
      frameSO3TrajectoryPointTwo.setIncludingFrame(poseFrame, time, orientation, new Vector3D(angularVelocity));
      assertTrue(frameSO3TrajectoryPointTwo.epsilonEquals(frameSO3TrajectoryPointTwo, 1e-10));

      frameSO3TrajectoryPointTwo = new FrameSO3TrajectoryPoint(poseFrame);
      SO3Waypoint SO3Waypoint = new SO3Waypoint();
      frameSO3TrajectoryPoint.getSO3Waypoint(SO3Waypoint);
      frameSO3TrajectoryPointTwo.set(time, SO3Waypoint);
      assertTrue(frameSO3TrajectoryPointTwo.epsilonEquals(frameSO3TrajectoryPoint, 1e-10));

      frameSO3TrajectoryPointTwo = new FrameSO3TrajectoryPoint(worldFrame);
      frameSO3TrajectoryPointTwo.setIncludingFrame(poseFrame, time, SO3Waypoint);
      assertTrue(frameSO3TrajectoryPointTwo.epsilonEquals(frameSO3TrajectoryPoint, 1e-10));

      frameSO3TrajectoryPointTwo = new FrameSO3TrajectoryPoint(poseFrame);
      FrameSO3Waypoint frameSO3Waypoint = new FrameSO3Waypoint(poseFrame);
      frameSO3TrajectoryPoint.getFrameSO3Waypoint(frameSO3Waypoint);
      frameSO3TrajectoryPointTwo.set(time, frameSO3Waypoint);
      assertTrue(frameSO3TrajectoryPointTwo.epsilonEquals(frameSO3TrajectoryPoint, 1e-10));

      frameSO3TrajectoryPointTwo = new FrameSO3TrajectoryPoint(worldFrame);
      frameSO3TrajectoryPointTwo.setIncludingFrame(time, frameSO3Waypoint);
      assertTrue(frameSO3TrajectoryPointTwo.epsilonEquals(frameSO3TrajectoryPoint, 1e-10));

   }
}
