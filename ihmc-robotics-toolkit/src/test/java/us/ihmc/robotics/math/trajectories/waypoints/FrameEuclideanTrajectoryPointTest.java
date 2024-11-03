package us.ihmc.robotics.math.trajectories.waypoints;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.lang.reflect.Method;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.FrameTypeCopier;
import us.ihmc.euclid.referenceFrame.api.RandomFramelessTypeBuilder;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.math.trajectories.trajectorypoints.EuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.EuclideanTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.EuclideanTrajectoryPointReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameEuclideanTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameEuclideanTrajectoryPointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointBasics;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class FrameEuclideanTrajectoryPointTest
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

      FrameEuclideanTrajectoryPoint frameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(worldFrame);
      EuclideanTrajectoryPoint simpleTrajectoryPoint = new EuclideanTrajectoryPoint();

      double time = 3.4;
      Point3D position = new Point3D(1.0, 2.1, 3.7);
      Vector3D linearVelocity = new Vector3D(-0.4, 1.2, 3.3);

      simpleTrajectoryPoint.set(time, position, linearVelocity);
      frameEuclideanTrajectoryPoint.setIncludingFrame(worldFrame, simpleTrajectoryPoint);
      frameEuclideanTrajectoryPoint.changeFrame(poseFrame);

      // Do some checks:
      RigidBodyTransform transformToPoseFrame = worldFrame.getTransformToDesiredFrame(poseFrame);
      transformToPoseFrame.transform(position);
      transformToPoseFrame.transform(linearVelocity);

      FrameEuclideanTrajectoryPoint expectedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(poseFrame);

      expectedFrameEuclideanTrajectoryPoint.setTime(time);
      expectedFrameEuclideanTrajectoryPoint.getPosition().set(position);

      expectedFrameEuclideanTrajectoryPoint.getLinearVelocity().set(linearVelocity);

      assertEquals(3.4, frameEuclideanTrajectoryPoint.getTime(), 1e-7);
      assertEquals(3.4, expectedFrameEuclideanTrajectoryPoint.getTime(), 1e-7);
      assertTrue(expectedFrameEuclideanTrajectoryPoint.epsilonEquals(frameEuclideanTrajectoryPoint, 1e-10));
   }

   @Test
   public void testConstructors()
   {
      double epsilon = 1.0e-20;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame aFrame = EuclidFrameRandomTools.nextReferenceFrame("aFrame", random, worldFrame);

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FramePoint3D expectedPosition = new FramePoint3D(expectedFrame);
      FrameVector3D expectedLinearVelocity = new FrameVector3D(expectedFrame);

      FrameEuclideanTrajectoryPoint testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = aFrame;
      expectedTime = 0.0;
      expectedPosition = new FramePoint3D(expectedFrame);
      expectedLinearVelocity = new FrameVector3D(expectedFrame);
      testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedFrame);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      FrameEuclideanTrajectoryPoint expectedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedFrameEuclideanTrajectoryPoint);

      assertTrue(expectedFrameEuclideanTrajectoryPoint.epsilonEquals(testedFrameEuclideanTrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedFrameEuclideanTrajectoryPoint.getReferenceFrame(), expectedFrameEuclideanTrajectoryPoint.getTime(),
            expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      final ReferenceFrame expectedFinalFrame = aFrame;
      final double expectedFinalTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      final FramePoint3D expectedFinalPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFinalFrame, 10.0, 10.0, 10.0);
      final FrameVector3D expectedFinalLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFinalFrame);

      EuclideanTrajectoryPoint expectedEuclideanTrajectoryPoint = new EuclideanTrajectoryPoint();
      expectedEuclideanTrajectoryPoint.setTime(expectedFinalTime);
      expectedEuclideanTrajectoryPoint.getPosition().set(expectedFinalPosition);
      expectedEuclideanTrajectoryPoint.getLinearVelocity().set(expectedFinalLinearVelocity);

      testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedFinalFrame, expectedEuclideanTrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalFrame, expectedFinalTime, expectedFinalPosition, expectedFinalLinearVelocity,
            testedFrameEuclideanTrajectoryPoint, epsilon);

   }

   @Test
   public void testSetters()
   {
      double epsilon = 1.0e-20;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame aFrame = EuclidFrameRandomTools.nextReferenceFrame("aFrame", random, worldFrame);

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FramePoint3D expectedPosition = new FramePoint3D(expectedFrame);
      FrameVector3D expectedLinearVelocity = new FrameVector3D(expectedFrame);

      final FrameEuclideanTrajectoryPoint testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      testedFrameEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      testedFrameEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = aFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      testedFrameEuclideanTrajectoryPoint.setIncludingFrame(expectedTime, expectedPosition, expectedLinearVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      FrameEuclideanTrajectoryPoint expectedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      testedFrameEuclideanTrajectoryPoint.setIncludingFrame(expectedFrameEuclideanTrajectoryPoint);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      expectedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      testedFrameEuclideanTrajectoryPoint.set(expectedFrameEuclideanTrajectoryPoint);

      assertTrue(expectedFrameEuclideanTrajectoryPoint.epsilonEquals(testedFrameEuclideanTrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedFrameEuclideanTrajectoryPoint.getReferenceFrame(), expectedFrameEuclideanTrajectoryPoint.getTime(),
            expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      final ReferenceFrame expectedFinalFrame = aFrame;
      final double expectedFinalTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      final FramePoint3D expectedFinalPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFinalFrame, 10.0, 10.0, 10.0);
      final FrameVector3D expectedFinalLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFinalFrame);

      EuclideanTrajectoryPoint expectedEuclideanTrajectoryPoint = new EuclideanTrajectoryPoint();
      expectedEuclideanTrajectoryPoint.setTime(expectedFinalTime);
      expectedEuclideanTrajectoryPoint.getPosition().set(expectedFinalPosition);
      expectedEuclideanTrajectoryPoint.getLinearVelocity().set(expectedFinalLinearVelocity);

      testedFrameEuclideanTrajectoryPoint.setIncludingFrame(expectedFinalFrame, expectedEuclideanTrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalFrame, expectedFinalTime, expectedFinalPosition, expectedFinalLinearVelocity,
            testedFrameEuclideanTrajectoryPoint, epsilon);

   }

   @Test
   public void testChangeFrame() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      FramePoint3D expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameVector3D expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      FrameEuclideanTrajectoryPoint testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = EuclidFrameRandomTools.nextReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : expectedFrame);

         expectedPosition.changeFrame(expectedFrame);
         expectedLinearVelocity.changeFrame(expectedFrame);
         testedFrameEuclideanTrajectoryPoint.changeFrame(expectedFrame);

         assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      FramePoint3D expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameVector3D expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      FrameEuclideanTrajectoryPoint testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero();
      expectedLinearVelocity.setToZero();
      testedFrameEuclideanTrajectoryPoint.setToZero();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = EuclidFrameRandomTools.nextReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
      testedFrameEuclideanTrajectoryPoint.setIncludingFrame(expectedTime, expectedPosition, expectedLinearVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero(expectedFrame);
      expectedLinearVelocity.setToZero(expectedFrame);
      testedFrameEuclideanTrajectoryPoint.setToZero(expectedFrame);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      FramePoint3D expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameVector3D expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      FrameEuclideanTrajectoryPoint testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      testedFrameEuclideanTrajectoryPoint.setToNaN();
      assertTrue(Double.isNaN(testedFrameEuclideanTrajectoryPoint.getTime()));
      assertTrue(testedFrameEuclideanTrajectoryPoint.containsNaN());

      expectedFrame = EuclidFrameRandomTools.nextReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
      testedFrameEuclideanTrajectoryPoint.setIncludingFrame(expectedTime, expectedPosition, expectedLinearVelocity);

      testedFrameEuclideanTrajectoryPoint.setToNaN(expectedFrame);

      assertTrue(expectedFrame == testedFrameEuclideanTrajectoryPoint.getReferenceFrame());
      assertTrue(Double.isNaN(testedFrameEuclideanTrajectoryPoint.getTime()));
      assertTrue(testedFrameEuclideanTrajectoryPoint.containsNaN());
   }

   static void assertTrajectoryPointContainsExpectedData(ReferenceFrame expectedFrame, double expectedTime, FramePoint3DReadOnly expectedPosition,
         FrameVector3DReadOnly expectedLinearVelocity, FrameEuclideanTrajectoryPoint testedFrameEuclideanTrajectoryPoint, double epsilon)
   {
      assertTrue(expectedFrame == testedFrameEuclideanTrajectoryPoint.getReferenceFrame());
      assertEquals(expectedTime, testedFrameEuclideanTrajectoryPoint.getTime(), epsilon);
      assertTrue(expectedPosition.epsilonEquals(testedFrameEuclideanTrajectoryPoint.getPosition(), epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(testedFrameEuclideanTrajectoryPoint.getLinearVelocity(), epsilon));

      Point3D actualPosition = new Point3D();
      Vector3D actualLinearVelocity = new Vector3D();

      actualPosition.set(testedFrameEuclideanTrajectoryPoint.getPosition());
      actualLinearVelocity.set(testedFrameEuclideanTrajectoryPoint.getLinearVelocity());

      assertTrue(expectedPosition.epsilonEquals(actualPosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualLinearVelocity, epsilon));

      FramePoint3D actualFramePosition = new FramePoint3D();
      FrameVector3D actualFrameLinearVelocity = new FrameVector3D();

      actualFramePosition.setIncludingFrame(testedFrameEuclideanTrajectoryPoint.getPosition());
      actualFrameLinearVelocity.setIncludingFrame(testedFrameEuclideanTrajectoryPoint.getLinearVelocity());

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));

      actualFramePosition = new FramePoint3D(expectedFrame);
      actualFrameLinearVelocity = new FrameVector3D(expectedFrame);

      actualFramePosition.set(testedFrameEuclideanTrajectoryPoint.getPosition());
      actualFrameLinearVelocity.set(testedFrameEuclideanTrajectoryPoint.getLinearVelocity());

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
   }

   @Test
   public void testSomeSetsAngGets()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FrameEuclideanTrajectoryPoint frameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(worldFrame);
      frameEuclideanTrajectoryPoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      EuclideanTrajectoryPoint simpleTrajectoryPoint = new EuclideanTrajectoryPoint();

      double time = 3.4;
      Point3D position = new Point3D(1.0, 2.1, 3.7);
      Vector3D linearVelocity = new Vector3D(-0.4, 1.2, 3.3);

      simpleTrajectoryPoint.set(time, position, linearVelocity);
      frameEuclideanTrajectoryPoint.setIncludingFrame(worldFrame, simpleTrajectoryPoint);

      // Check some get calls:
      FramePoint3D pointForVerification = new FramePoint3D(worldFrame);
      FrameVector3D linearVelocityForVerification = new FrameVector3D(worldFrame);

      pointForVerification.set(frameEuclideanTrajectoryPoint.getPosition());
      linearVelocityForVerification.set(frameEuclideanTrajectoryPoint.getLinearVelocity());

      assertEquals(time, frameEuclideanTrajectoryPoint.getTime(), 1e-10);
      assertTrue(pointForVerification.epsilonEquals(position, 1e-10));
      assertTrue(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-10));

      // Check NaN calls:
      assertFalse(frameEuclideanTrajectoryPoint.containsNaN());
      frameEuclideanTrajectoryPoint.getPosition().setToNaN();
      assertTrue(frameEuclideanTrajectoryPoint.containsNaN());
      frameEuclideanTrajectoryPoint.getPosition().setToZero();

      assertFalse(frameEuclideanTrajectoryPoint.containsNaN());
      frameEuclideanTrajectoryPoint.getLinearVelocity().setToNaN();
      assertTrue(frameEuclideanTrajectoryPoint.containsNaN());
      frameEuclideanTrajectoryPoint.getLinearVelocity().setToZero();

      position.set(frameEuclideanTrajectoryPoint.getPosition());
      linearVelocity.set(frameEuclideanTrajectoryPoint.getLinearVelocity());

      // Make sure they are all equal to zero:
      assertTrue(position.epsilonEquals(new Point3D(), 1e-10));
      assertTrue(linearVelocity.epsilonEquals(new Vector3D(), 1e-10));

      time = 9.9;
      pointForVerification.set(3.9, 2.2, 1.1);
      linearVelocityForVerification.set(8.8, 1.4, 9.22);

      assertFalse(Math.abs(frameEuclideanTrajectoryPoint.getTime() - time) < 1e-7);
      assertFalse(pointForVerification.epsilonEquals(position, 1e-7));
      assertFalse(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-7));

      frameEuclideanTrajectoryPoint.set(time, pointForVerification, linearVelocityForVerification);

      position.set(frameEuclideanTrajectoryPoint.getPosition());
      linearVelocity.set(frameEuclideanTrajectoryPoint.getLinearVelocity());

      assertEquals(time, frameEuclideanTrajectoryPoint.getTime(), 1e-10);
      assertTrue(pointForVerification.epsilonEquals(position, 1e-10));
      assertTrue(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-10));

      FrameEuclideanTrajectoryPoint frameEuclideanTrajectoryPointTwo = new FrameEuclideanTrajectoryPoint(worldFrame);

      double positionDistance = frameEuclideanTrajectoryPoint.positionDistance(frameEuclideanTrajectoryPointTwo);
      assertEquals(4.610856753359402, positionDistance, 1e-7);
      assertFalse(frameEuclideanTrajectoryPoint.epsilonEquals(frameEuclideanTrajectoryPointTwo, 1e-7));

      frameEuclideanTrajectoryPointTwo.set(frameEuclideanTrajectoryPoint);
      positionDistance = frameEuclideanTrajectoryPoint.positionDistance(frameEuclideanTrajectoryPointTwo);
      assertEquals(0.0, positionDistance, 1e-7);
      assertTrue(frameEuclideanTrajectoryPoint.epsilonEquals(frameEuclideanTrajectoryPointTwo, 1e-7));

      EuclideanTrajectoryPoint simplePoint = new EuclideanTrajectoryPoint();
      simplePoint.set(frameEuclideanTrajectoryPoint);

      frameEuclideanTrajectoryPoint.setToNaN();
      assertTrue(frameEuclideanTrajectoryPoint.containsNaN());
      positionDistance = frameEuclideanTrajectoryPoint.positionDistance(frameEuclideanTrajectoryPointTwo);
      assertTrue(Double.isNaN(positionDistance));
      assertFalse(frameEuclideanTrajectoryPoint.epsilonEquals(frameEuclideanTrajectoryPointTwo, 1e-7));

      EuclideanTrajectoryPointBasics trajectoryPointAsInterface = simplePoint;
      frameEuclideanTrajectoryPoint.set(trajectoryPointAsInterface);

      positionDistance = frameEuclideanTrajectoryPoint.positionDistance(frameEuclideanTrajectoryPointTwo);
      assertEquals(0.0, positionDistance, 1e-7);
      assertTrue(frameEuclideanTrajectoryPoint.epsilonEquals(frameEuclideanTrajectoryPointTwo, 1e-7));
   }

   @Test
   public void testSomeMoreSettersAndGetters()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FrameEuclideanTrajectoryPoint frameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(worldFrame);
      frameEuclideanTrajectoryPoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      double time = 3.4;
      FramePoint3D position = new FramePoint3D(worldFrame, 1.0, 2.1, 3.7);
      FrameVector3D linearVelocity = new FrameVector3D(worldFrame, -0.4, 1.2, 3.3);

      frameEuclideanTrajectoryPoint.setTime(time);
      frameEuclideanTrajectoryPoint.getPosition().set((FramePoint3DReadOnly) position);
      frameEuclideanTrajectoryPoint.getLinearVelocity().set((FrameVector3DReadOnly) linearVelocity);

      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose3D(worldFrame));

      FramePoint3D poseFramePosition = new FramePoint3D(worldFrame, new Point3D(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameQuaternion poseOrientation = new FrameQuaternion(worldFrame, new AxisAngle(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      frameEuclideanTrajectoryPoint.changeFrame(poseFrame);

      assertFalse(position.epsilonEquals(frameEuclideanTrajectoryPoint.getPosition(), 1e-10));
      assertFalse(linearVelocity.epsilonEquals(frameEuclideanTrajectoryPoint.getLinearVelocity(), 1e-10));

      position.changeFrame(poseFrame);
      linearVelocity.changeFrame(poseFrame);

      assertTrue(position.epsilonEquals(frameEuclideanTrajectoryPoint.getPosition(), 1e-10));
      assertTrue(linearVelocity.epsilonEquals(frameEuclideanTrajectoryPoint.getLinearVelocity(), 1e-10));

      FrameEuclideanTrajectoryPoint frameEuclideanTrajectoryPointTwo = new FrameEuclideanTrajectoryPoint(poseFrame);
      frameEuclideanTrajectoryPointTwo.setTime(time);
      frameEuclideanTrajectoryPointTwo.getPosition().set((FramePoint3DReadOnly) position);
      frameEuclideanTrajectoryPointTwo.getLinearVelocity().set((FrameVector3DReadOnly) linearVelocity);
      assertTrue(frameEuclideanTrajectoryPointTwo.epsilonEquals(frameEuclideanTrajectoryPoint, 1e-10));

      frameEuclideanTrajectoryPointTwo = new FrameEuclideanTrajectoryPoint(worldFrame);
      frameEuclideanTrajectoryPointTwo.setIncludingFrame(poseFrame, time, new Point3D(position), linearVelocity);
      assertTrue(frameEuclideanTrajectoryPointTwo.epsilonEquals(frameEuclideanTrajectoryPoint, 1e-10));

      frameEuclideanTrajectoryPointTwo = new FrameEuclideanTrajectoryPoint(poseFrame);
      EuclideanWaypointBasics euclideanWaypoint = frameEuclideanTrajectoryPoint;
      frameEuclideanTrajectoryPointTwo.set(time, euclideanWaypoint);
      assertTrue(frameEuclideanTrajectoryPointTwo.epsilonEquals(frameEuclideanTrajectoryPoint, 1e-10));

      frameEuclideanTrajectoryPointTwo = new FrameEuclideanTrajectoryPoint(worldFrame);
      frameEuclideanTrajectoryPointTwo.setIncludingFrame(poseFrame, time, euclideanWaypoint);
      assertTrue(frameEuclideanTrajectoryPointTwo.epsilonEquals(frameEuclideanTrajectoryPoint, 1e-10));

      frameEuclideanTrajectoryPointTwo = new FrameEuclideanTrajectoryPoint(poseFrame);
      FrameEuclideanWaypointBasics frameEuclideanWaypoint = frameEuclideanTrajectoryPoint;
      frameEuclideanTrajectoryPointTwo.set(time, frameEuclideanWaypoint);
      assertTrue(frameEuclideanTrajectoryPointTwo.epsilonEquals(frameEuclideanTrajectoryPoint, 1e-10));

      frameEuclideanTrajectoryPointTwo = new FrameEuclideanTrajectoryPoint(worldFrame);
      frameEuclideanTrajectoryPointTwo.setIncludingFrame(time, frameEuclideanWaypoint);
      assertTrue(frameEuclideanTrajectoryPointTwo.epsilonEquals(frameEuclideanTrajectoryPoint, 1e-10));
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("equals") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new FrameTrajectoryPointAPIDefaultConfiguration());
      tester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(TrajectoryPointRandomTools::nextFrameEuclideanTrajectoryPoint,
                                                                    methodFilter,
                                                                    EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testConsistencyWithEuclideanTrajectoryPoint() throws Exception
   {
      FrameTypeCopier frameTypeBuilder = (frame, TrajectoryPoint) -> new FrameEuclideanTrajectoryPoint(frame, (EuclideanTrajectoryPointReadOnly) TrajectoryPoint);
      RandomFramelessTypeBuilder framelessTypeBuilber = TrajectoryPointRandomTools::nextEuclideanTrajectoryPoint;
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("toString");
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new FrameTrajectoryPointAPIDefaultConfiguration());
      tester.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder,
                                                                  framelessTypeBuilber,
                                                                  methodFilter,
                                                                  EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   @Test
   public void testOverloading() throws Exception
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new FrameTrajectoryPointAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(FrameEuclideanTrajectoryPointBasics.class, EuclideanTrajectoryPointBasics.class, false, 1);
      tester.assertOverloadingWithFrameObjects(FrameEuclideanTrajectoryPointReadOnly.class, EuclideanTrajectoryPointReadOnly.class, false, 1);
   }
}
