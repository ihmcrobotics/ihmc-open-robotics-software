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
import us.ihmc.robotics.geometry.transformables.TransformablePoint3d;
import us.ihmc.robotics.geometry.transformables.TransformableVector3d;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanTrajectoryPointInterface;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SimpleEuclideanTrajectoryPointTest
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

      SimpleEuclideanTrajectoryPoint simpleEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint();
      SimpleEuclideanTrajectoryPoint simpleTrajectoryPoint = new SimpleEuclideanTrajectoryPoint();

      double time = 3.4;
      Point3d position = new Point3d(1.0, 2.1, 3.7);
      Vector3d linearVelocity = new Vector3d(-0.4, 1.2, 3.3);

      simpleTrajectoryPoint.set(time, position, linearVelocity);
      simpleEuclideanTrajectoryPoint.set(simpleTrajectoryPoint);
      simpleEuclideanTrajectoryPoint.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));

      // Do some checks:
      RigidBodyTransform transformToPoseFrame = worldFrame.getTransformToDesiredFrame(poseFrame);
      transformToPoseFrame.transform(position);
      transformToPoseFrame.transform(linearVelocity);

      SimpleEuclideanTrajectoryPoint expectedSimpleEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint();

      expectedSimpleEuclideanTrajectoryPoint.setTime(time);
      expectedSimpleEuclideanTrajectoryPoint.setPosition(position);
      expectedSimpleEuclideanTrajectoryPoint.setLinearVelocity(linearVelocity);

      assertEquals(3.4, simpleEuclideanTrajectoryPoint.getTime(), 1e-7);
      assertEquals(3.4, expectedSimpleEuclideanTrajectoryPoint.getTime(), 1e-7);
      assertTrue(expectedSimpleEuclideanTrajectoryPoint.epsilonEquals(simpleEuclideanTrajectoryPoint, 1e-10));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructors()
   {
      double epsilon = 1.0e-14;
      Random random = new Random(21651016L);

      double expectedTime = 0.0;
      Point3d expectedPosition = new Point3d();
      Vector3d expectedLinearVelocity = new Vector3d();

      SimpleEuclideanTrajectoryPoint testedSimpleEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedLinearVelocity,
            testedSimpleEuclideanTrajectoryPoint, epsilon);

      expectedTime = 0.0;
      expectedPosition = new Point3d();
      expectedLinearVelocity = new Vector3d();
      testedSimpleEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedLinearVelocity,
            testedSimpleEuclideanTrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      expectedLinearVelocity = RandomTools.generateRandomVector(random);

      testedSimpleEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedLinearVelocity,
            testedSimpleEuclideanTrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      expectedLinearVelocity = RandomTools.generateRandomVector(random);

      SimpleEuclideanTrajectoryPoint expectedSimpleEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint(expectedTime, expectedPosition,
            expectedLinearVelocity);

      testedSimpleEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint(expectedSimpleEuclideanTrajectoryPoint);

      assertTrue(expectedSimpleEuclideanTrajectoryPoint.epsilonEquals(testedSimpleEuclideanTrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedSimpleEuclideanTrajectoryPoint.getTime(),
            expectedPosition, expectedLinearVelocity, testedSimpleEuclideanTrajectoryPoint, epsilon);

      final double expectedFinalTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      final Point3d expectedFinalPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      final Vector3d expectedFinalLinearVelocity = RandomTools.generateRandomVector(random);

      SimpleEuclideanTrajectoryPoint expectedEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint();
      expectedEuclideanTrajectoryPoint.setTime(expectedFinalTime);
      expectedEuclideanTrajectoryPoint.setPosition(expectedFinalPosition);
      expectedEuclideanTrajectoryPoint.setLinearVelocity(expectedFinalLinearVelocity);

      testedSimpleEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint(expectedEuclideanTrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalTime, expectedFinalPosition,
            expectedFinalLinearVelocity, testedSimpleEuclideanTrajectoryPoint, epsilon);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetters()
   {
      double epsilon = 1.0e-14;
      Random random = new Random(21651016L);

      double expectedTime = 0.0;
      Point3d expectedPosition = new Point3d();
      Vector3d expectedLinearVelocity = new Vector3d();

      final SimpleEuclideanTrajectoryPoint testedSimpleEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedLinearVelocity,
            testedSimpleEuclideanTrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      expectedLinearVelocity = RandomTools.generateRandomVector(random);

      testedSimpleEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedLinearVelocity,
            testedSimpleEuclideanTrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      expectedLinearVelocity = RandomTools.generateRandomVector(random);

      testedSimpleEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedLinearVelocity,
            testedSimpleEuclideanTrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      expectedLinearVelocity = RandomTools.generateRandomVector(random);

      testedSimpleEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedLinearVelocity,
            testedSimpleEuclideanTrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      expectedLinearVelocity = RandomTools.generateRandomVector(random);

      SimpleEuclideanTrajectoryPoint expectedSimpleEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint(expectedTime, expectedPosition,
            expectedLinearVelocity);

      testedSimpleEuclideanTrajectoryPoint.set(expectedSimpleEuclideanTrajectoryPoint);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      expectedLinearVelocity = RandomTools.generateRandomVector(random);

      expectedSimpleEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      testedSimpleEuclideanTrajectoryPoint.set(expectedSimpleEuclideanTrajectoryPoint);

      assertTrue(expectedSimpleEuclideanTrajectoryPoint.epsilonEquals(testedSimpleEuclideanTrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedSimpleEuclideanTrajectoryPoint.getTime(),
            expectedPosition, expectedLinearVelocity, testedSimpleEuclideanTrajectoryPoint, epsilon);

      final double expectedFinalTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      final Point3d expectedFinalPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      final Quat4d expectedFinalOrientation = RandomTools.generateRandomQuaternion(random);
      final Vector3d expectedFinalLinearVelocity = RandomTools.generateRandomVector(random);
      final Vector3d expectedFinalAngularVelocity = RandomTools.generateRandomVector(random);

      SimpleEuclideanTrajectoryPoint expectedEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint();
      expectedEuclideanTrajectoryPoint.setTime(expectedFinalTime);
      expectedEuclideanTrajectoryPoint.setPosition(expectedFinalPosition);
      expectedEuclideanTrajectoryPoint.setLinearVelocity(expectedFinalLinearVelocity);

      testedSimpleEuclideanTrajectoryPoint.set(expectedEuclideanTrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalTime, expectedFinalPosition,
            expectedFinalLinearVelocity, testedSimpleEuclideanTrajectoryPoint, epsilon);

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
      TransformablePoint3d expectedPosition = new TransformablePoint3d(RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0));
      TransformableVector3d expectedLinearVelocity = new TransformableVector3d(RandomTools.generateRandomVector(random));
      SimpleEuclideanTrajectoryPoint testedSimpleEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint(expectedTime, expectedPosition,
            expectedLinearVelocity);

      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = ReferenceFrame.generateRandomReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : expectedFrame);

         expectedPosition.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));
         expectedLinearVelocity.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));
         testedSimpleEuclideanTrajectoryPoint.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));

         assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedLinearVelocity,
               testedSimpleEuclideanTrajectoryPoint, epsilon);
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
      Vector3d expectedLinearVelocity = RandomTools.generateRandomVector(random);
      SimpleEuclideanTrajectoryPoint testedSimpleEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint(expectedTime, expectedPosition,
            expectedLinearVelocity);

      expectedTime = 0.0;
      expectedPosition.set(0.0, 0.0, 0.0);
      expectedLinearVelocity.set(0.0, 0.0, 0.0);
      testedSimpleEuclideanTrajectoryPoint.setToZero();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedLinearVelocity,
            testedSimpleEuclideanTrajectoryPoint, epsilon);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      expectedLinearVelocity = RandomTools.generateRandomVector(random);
      testedSimpleEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      expectedTime = 0.0;
      expectedPosition.set(0.0, 0.0, 0.0);
      expectedLinearVelocity.set(0.0, 0.0, 0.0);
      testedSimpleEuclideanTrajectoryPoint.setToZero();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedLinearVelocity,
            testedSimpleEuclideanTrajectoryPoint, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(21651016L);

      double expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      Point3d expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      Vector3d expectedLinearVelocity = RandomTools.generateRandomVector(random);
      SimpleEuclideanTrajectoryPoint testedSimpleEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      testedSimpleEuclideanTrajectoryPoint.setToNaN();
      assertTrue(Double.isNaN(testedSimpleEuclideanTrajectoryPoint.getTime()));
      assertTrue(testedSimpleEuclideanTrajectoryPoint.containsNaN());

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      expectedLinearVelocity = RandomTools.generateRandomVector(random);
      testedSimpleEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      testedSimpleEuclideanTrajectoryPoint.setToNaN();

      assertTrue(Double.isNaN(testedSimpleEuclideanTrajectoryPoint.getTime()));
      assertTrue(testedSimpleEuclideanTrajectoryPoint.containsNaN());
   }

   static void assertTrajectoryPointContainsExpectedData(double expectedTime, Point3d expectedPosition,
         Vector3d expectedLinearVelocity,
         SimpleEuclideanTrajectoryPoint testedSimpleEuclideanTrajectoryPoint, double epsilon)
   {
      assertEquals(expectedTime, testedSimpleEuclideanTrajectoryPoint.getTime(), epsilon);
      assertTrue(expectedPosition.epsilonEquals(testedSimpleEuclideanTrajectoryPoint.getEuclideanWaypoint().getPosition(), epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(testedSimpleEuclideanTrajectoryPoint.getEuclideanWaypoint().getLinearVelocity(), epsilon));

      Point3d actualPosition = new Point3d();
      Vector3d actualLinearVelocity = new Vector3d();

      testedSimpleEuclideanTrajectoryPoint.getPosition(actualPosition);
      testedSimpleEuclideanTrajectoryPoint.getLinearVelocity(actualLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualPosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualLinearVelocity, epsilon));

      Point3d actualFramePosition = new Point3d();
      Vector3d actualFrameLinearVelocity = new Vector3d();

      testedSimpleEuclideanTrajectoryPoint.getPosition(actualFramePosition);
      testedSimpleEuclideanTrajectoryPoint.getLinearVelocity(actualFrameLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));

      actualFramePosition = new Point3d();
      actualFrameLinearVelocity = new Vector3d();

      testedSimpleEuclideanTrajectoryPoint.getPosition(actualFramePosition);
      testedSimpleEuclideanTrajectoryPoint.getLinearVelocity(actualFrameLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testSomeSetsAngGets()
   {
      SimpleEuclideanTrajectoryPoint simpleEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint();

      SimpleEuclideanTrajectoryPoint simpleTrajectoryPoint = new SimpleEuclideanTrajectoryPoint();

      double time = 3.4;
      Point3d position = new Point3d(1.0, 2.1, 3.7);
      Vector3d linearVelocity = new Vector3d(-0.4, 1.2, 3.3);

      simpleTrajectoryPoint.set(time, position, linearVelocity);
      simpleEuclideanTrajectoryPoint.set(simpleTrajectoryPoint);

      // Check some get calls: 
      Point3d pointForVerification = new Point3d();
      Vector3d linearVelocityForVerification = new Vector3d();

      simpleEuclideanTrajectoryPoint.getPosition(pointForVerification);
      simpleEuclideanTrajectoryPoint.getLinearVelocity(linearVelocityForVerification);

      assertEquals(time, simpleEuclideanTrajectoryPoint.getTime(), 1e-10);
      assertTrue(pointForVerification.epsilonEquals(position, 1e-10));
      assertTrue(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-10));

      // Check NaN calls:
      assertFalse(simpleEuclideanTrajectoryPoint.containsNaN());
      simpleEuclideanTrajectoryPoint.setPositionToNaN();
      assertTrue(simpleEuclideanTrajectoryPoint.containsNaN());
      simpleEuclideanTrajectoryPoint.setPositionToZero();

      assertFalse(simpleEuclideanTrajectoryPoint.containsNaN());
      simpleEuclideanTrajectoryPoint.setLinearVelocityToNaN();
      assertTrue(simpleEuclideanTrajectoryPoint.containsNaN());
      simpleEuclideanTrajectoryPoint.setLinearVelocityToZero();

      simpleEuclideanTrajectoryPoint.getPosition(position);
      simpleEuclideanTrajectoryPoint.getLinearVelocity(linearVelocity);

      // Make sure they are all equal to zero:
      assertTrue(position.epsilonEquals(new Point3d(), 1e-10));
      assertTrue(linearVelocity.epsilonEquals(new Vector3d(), 1e-10));

      time = 9.9;
      pointForVerification.set(3.9, 2.2, 1.1);
      linearVelocityForVerification.set(8.8, 1.4, 9.22);

      assertFalse(Math.abs(simpleEuclideanTrajectoryPoint.getTime() - time) < 1e-7);
      assertFalse(pointForVerification.epsilonEquals(position, 1e-7));
      assertFalse(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-7));

      simpleEuclideanTrajectoryPoint.set(time, pointForVerification, linearVelocityForVerification);

      simpleEuclideanTrajectoryPoint.getPosition(position);
      simpleEuclideanTrajectoryPoint.getLinearVelocity(linearVelocity);

      assertEquals(time, simpleEuclideanTrajectoryPoint.getTime(), 1e-10);
      assertTrue(pointForVerification.epsilonEquals(position, 1e-10));
      assertTrue(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-10));

      SimpleEuclideanTrajectoryPoint simpleEuclideanTrajectoryPointTwo = new SimpleEuclideanTrajectoryPoint();

      double positionDistance = simpleEuclideanTrajectoryPoint.positionDistance(simpleEuclideanTrajectoryPointTwo);
      assertEquals(4.610856753359402, positionDistance, 1e-7);
      assertFalse(simpleEuclideanTrajectoryPoint.epsilonEquals(simpleEuclideanTrajectoryPointTwo, 1e-7));

      simpleEuclideanTrajectoryPointTwo.set(simpleEuclideanTrajectoryPoint);
      positionDistance = simpleEuclideanTrajectoryPoint.positionDistance(simpleEuclideanTrajectoryPointTwo);
      assertEquals(0.0, positionDistance, 1e-7);
      assertTrue(simpleEuclideanTrajectoryPoint.epsilonEquals(simpleEuclideanTrajectoryPointTwo, 1e-7));

      SimpleEuclideanTrajectoryPoint simplePoint = new SimpleEuclideanTrajectoryPoint();
      simpleEuclideanTrajectoryPoint.get(simplePoint);

      simpleEuclideanTrajectoryPoint.setToNaN();
      assertTrue(simpleEuclideanTrajectoryPoint.containsNaN());
      positionDistance = simpleEuclideanTrajectoryPoint.positionDistance(simpleEuclideanTrajectoryPointTwo);
      assertTrue(Double.isNaN(positionDistance));
      assertFalse(simpleEuclideanTrajectoryPoint.epsilonEquals(simpleEuclideanTrajectoryPointTwo, 1e-7));

      EuclideanTrajectoryPointInterface<?> trajectoryPointAsInterface = simplePoint;
      simpleEuclideanTrajectoryPoint.set(trajectoryPointAsInterface);

      positionDistance = simpleEuclideanTrajectoryPoint.positionDistance(simpleEuclideanTrajectoryPointTwo);
      assertEquals(0.0, positionDistance, 1e-7);
      
      assertTrue(simpleEuclideanTrajectoryPoint.epsilonEquals(simpleEuclideanTrajectoryPointTwo, 1e-7));

      String string = simpleEuclideanTrajectoryPoint.toString();
      String expectedString = "Euclidean trajectory point: (time =  9.90, Euclidean waypoint: [position = ( 3.90,  2.20,  1.10), linearVelocity = ( 8.80,  1.40,  9.22)].)";
      assertEquals(expectedString, string);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSomeMoreSettersAndGetters()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      SimpleEuclideanTrajectoryPoint simpleEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint();

      double time = 3.4;
      TransformablePoint3d position = new TransformablePoint3d(1.0, 2.1, 3.7);
      TransformableVector3d linearVelocity = new TransformableVector3d(-0.4, 1.2, 3.3);

      simpleEuclideanTrajectoryPoint.setTime(time);
      simpleEuclideanTrajectoryPoint.setPosition(position);
      simpleEuclideanTrajectoryPoint.setLinearVelocity(linearVelocity);

      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose(worldFrame));

      FramePoint poseFramePosition = new FramePoint(worldFrame, new Point3d(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameOrientation poseOrientation = new FrameOrientation(worldFrame, new AxisAngle4d(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      simpleEuclideanTrajectoryPoint.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));

      assertFalse(position.epsilonEquals(simpleEuclideanTrajectoryPoint.getPositionCopy(), 1e-10));
      assertFalse(linearVelocity.epsilonEquals(simpleEuclideanTrajectoryPoint.getLinearVelocityCopy(), 1e-10));

      position.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));
      linearVelocity.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));

      assertTrue(position.epsilonEquals(simpleEuclideanTrajectoryPoint.getPositionCopy(), 1e-10));
      assertTrue(linearVelocity.epsilonEquals(simpleEuclideanTrajectoryPoint.getLinearVelocityCopy(), 1e-10));

      
      SimpleEuclideanTrajectoryPoint simpleEuclideanTrajectoryPointTwo = new SimpleEuclideanTrajectoryPoint();
      simpleEuclideanTrajectoryPointTwo.setTime(time);
      simpleEuclideanTrajectoryPointTwo.setPosition(position);
      simpleEuclideanTrajectoryPointTwo.setLinearVelocity(linearVelocity);
      assertTrue(simpleEuclideanTrajectoryPointTwo.epsilonEquals(simpleEuclideanTrajectoryPoint, 1e-10));

      simpleEuclideanTrajectoryPointTwo = new SimpleEuclideanTrajectoryPoint();
      simpleEuclideanTrajectoryPointTwo.set(time, position, linearVelocity);
      assertTrue(simpleEuclideanTrajectoryPointTwo.epsilonEquals(simpleEuclideanTrajectoryPoint, 1e-10));

      simpleEuclideanTrajectoryPointTwo = new SimpleEuclideanTrajectoryPoint();
      EuclideanWaypoint simpleEuclideanWaypoint = new EuclideanWaypoint();
      simpleEuclideanTrajectoryPoint.get(simpleEuclideanWaypoint);
      simpleEuclideanTrajectoryPointTwo.set(time, simpleEuclideanWaypoint);
      assertTrue(simpleEuclideanTrajectoryPointTwo.epsilonEquals(simpleEuclideanTrajectoryPoint, 1e-10));

      simpleEuclideanTrajectoryPointTwo = new SimpleEuclideanTrajectoryPoint();
      simpleEuclideanTrajectoryPointTwo.set(time, simpleEuclideanWaypoint);
      assertTrue(simpleEuclideanTrajectoryPointTwo.epsilonEquals(simpleEuclideanTrajectoryPoint, 1e-10));

      
      simpleEuclideanTrajectoryPointTwo = new SimpleEuclideanTrajectoryPoint();
      EuclideanWaypoint euclideanWaypoint = simpleEuclideanTrajectoryPoint.getEuclideanWaypoint();
      
      simpleEuclideanTrajectoryPointTwo.set(time, euclideanWaypoint);
      assertTrue(simpleEuclideanTrajectoryPointTwo.epsilonEquals(simpleEuclideanTrajectoryPoint, 1e-10));
      
      
      
      
      simpleEuclideanTrajectoryPointTwo = new SimpleEuclideanTrajectoryPoint();
      euclideanWaypoint = new EuclideanWaypoint();
      simpleEuclideanTrajectoryPoint.get(euclideanWaypoint);
      
      simpleEuclideanTrajectoryPointTwo.set(time, euclideanWaypoint);
      assertTrue(simpleEuclideanTrajectoryPointTwo.epsilonEquals(simpleEuclideanTrajectoryPoint, 1e-10));

      
      Point3d positionToPack = new Point3d();
      Vector3d linearVelocityToPack = new Vector3d();
      simpleEuclideanTrajectoryPoint.get(positionToPack, linearVelocityToPack);

      simpleEuclideanTrajectoryPointTwo = new SimpleEuclideanTrajectoryPoint();
      simpleEuclideanTrajectoryPointTwo.set(time, positionToPack, linearVelocityToPack);
      assertTrue(simpleEuclideanTrajectoryPointTwo.epsilonEquals(simpleEuclideanTrajectoryPoint, 1e-10));

      
      
      
      positionToPack = new Point3d();
      linearVelocityToPack = new Vector3d();
      simpleEuclideanTrajectoryPoint.get( positionToPack, linearVelocityToPack);

      simpleEuclideanTrajectoryPointTwo = new SimpleEuclideanTrajectoryPoint();
      simpleEuclideanTrajectoryPointTwo.set(time, positionToPack, linearVelocityToPack);
      assertTrue(simpleEuclideanTrajectoryPointTwo.epsilonEquals(simpleEuclideanTrajectoryPoint, 1e-10));
      
      assertTrue(simpleEuclideanTrajectoryPointTwo.getPosition().epsilonEquals(positionToPack, 1e-10));
      assertTrue(simpleEuclideanTrajectoryPointTwo.getLinearVelocity().epsilonEquals(linearVelocityToPack, 1e-10));

   }
}
