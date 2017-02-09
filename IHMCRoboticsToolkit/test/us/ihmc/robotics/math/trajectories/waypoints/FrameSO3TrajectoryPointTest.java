package us.ihmc.robotics.math.trajectories.waypoints;

import static org.junit.Assert.*;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.frameObjects.FrameSO3Waypoint;
import us.ihmc.robotics.geometry.transformables.SO3Waypoint;
import us.ihmc.robotics.geometry.transformables.TransformableQuat4d;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3TrajectoryPointInterface;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class FrameSO3TrajectoryPointTest
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

      FrameSO3TrajectoryPoint frameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(worldFrame);
      SimpleSO3TrajectoryPoint simpleTrajectoryPoint = new SimpleSO3TrajectoryPoint();

      double time = 3.4;
      TransformableQuat4d orientation = new TransformableQuat4d(new Quat4d(0.1, 0.22, 0.34, 0.56));
      orientation.normalize();

      Vector3d angularVelocity = new Vector3d(1.7, 8.4, 2.2);

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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
      assertTrue(expectedOrientation.epsilonEquals(testedFrameSO3TrajectoryPoint.getGeometryObject().getOrientation(), epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(testedFrameSO3TrajectoryPoint.getGeometryObject().getAngularVelocity(), epsilon));

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
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSomeSetsAngGets()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FrameSO3TrajectoryPoint FrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(worldFrame);
      FrameSO3TrajectoryPoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      SimpleSO3TrajectoryPoint simpleTrajectoryPoint = new SimpleSO3TrajectoryPoint();

      double time = 3.4;
      TransformableQuat4d orientation = new TransformableQuat4d(new Quat4d(0.1, 0.22, 0.34, 0.56));
      orientation.normalize();

      Vector3d angularVelocity = new Vector3d(1.7, 8.4, 2.2);

      simpleTrajectoryPoint.set(time, orientation, angularVelocity);
      FrameSO3TrajectoryPoint.setIncludingFrame(worldFrame, simpleTrajectoryPoint);

      // Check some get calls: 
      FrameOrientation quaternionForVerification = new FrameOrientation(worldFrame);
      FrameVector angularVelocityForVerification = new FrameVector(worldFrame);

      FrameSO3TrajectoryPoint.getOrientation(quaternionForVerification);
      FrameSO3TrajectoryPoint.getAngularVelocity(angularVelocityForVerification);

      assertEquals(time, FrameSO3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(quaternionForVerification.getQuaternion().epsilonEquals(orientation, 1e-10));
      assertTrue(angularVelocityForVerification.getVector().epsilonEquals(angularVelocity, 1e-10));

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
      assertTrue(orientation.epsilonEquals(new TransformableQuat4d(), 1e-10));
      assertTrue(angularVelocity.epsilonEquals(new Vector3d(), 1e-10));

      time = 9.9;
      quaternionForVerification.setYawPitchRoll(0.2, 0.6, 1.1);
      angularVelocityForVerification.set(7.1, 2.2, 3.33);

      assertFalse(Math.abs(FrameSO3TrajectoryPoint.getTime() - time) < 1e-7);
      assertFalse(quaternionForVerification.getQuaternion().epsilonEquals(orientation, 1e-7));
      assertFalse(angularVelocityForVerification.getVector().epsilonEquals(angularVelocity, 1e-7));

      FrameSO3TrajectoryPoint.set(time, quaternionForVerification, angularVelocityForVerification);

      FrameSO3TrajectoryPoint.getOrientation(orientation);
      FrameSO3TrajectoryPoint.getAngularVelocity(angularVelocity);

      assertEquals(time, FrameSO3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(quaternionForVerification.getQuaternion().epsilonEquals(orientation, 1e-10));
      assertTrue(angularVelocityForVerification.getVector().epsilonEquals(angularVelocity, 1e-10));

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
      String expectedString = "SO3 trajectory point: (time =  9.90, SO3 trajectory point: (time =  9.90, SO3 waypoint: [orientation = ( 0.47,  0.30, -0.07,  0.83), angular velocity = ( 7.10,  2.20,  3.33)].)World)";
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
      FramePoint position = new FramePoint(worldFrame, 1.0, 2.1, 3.7);
      FrameOrientation orientation = new FrameOrientation(worldFrame, new Quat4d(0.1, 0.22, 0.34, 0.56));

      FrameVector linearVelocity = new FrameVector(worldFrame, -0.4, 1.2, 3.3);
      FrameVector angularVelocity = new FrameVector(worldFrame, 1.7, 8.4, 2.2);

      frameSO3TrajectoryPoint.setTime(time);
      frameSO3TrajectoryPoint.setOrientation(orientation);
      frameSO3TrajectoryPoint.setAngularVelocity(angularVelocity);

      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose(worldFrame));

      FramePoint poseFramePosition = new FramePoint(worldFrame, new Point3d(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameOrientation poseOrientation = new FrameOrientation(worldFrame, new AxisAngle4d(1.2, 3.9, 4.7, 2.2));
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
      frameSO3TrajectoryPointTwo.setIncludingFrame(poseFrame, time, orientation.getQuaternionCopy(), angularVelocity.getVectorCopy());
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
