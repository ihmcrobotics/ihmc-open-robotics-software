package us.ihmc.robotics.math.trajectories.waypoints;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.math.trajectories.trajectorypoints.EuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.EuclideanTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointBasics;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class EuclideanTrajectoryPointTest
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

      EuclideanTrajectoryPoint simpleEuclideanTrajectoryPoint = new EuclideanTrajectoryPoint();
      EuclideanTrajectoryPoint simpleTrajectoryPoint = new EuclideanTrajectoryPoint();

      double time = 3.4;
      Point3D position = new Point3D(1.0, 2.1, 3.7);
      Vector3D linearVelocity = new Vector3D(-0.4, 1.2, 3.3);

      simpleTrajectoryPoint.set(time, position, linearVelocity);
      simpleEuclideanTrajectoryPoint.set(simpleTrajectoryPoint);
      simpleEuclideanTrajectoryPoint.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));

      // Do some checks:
      RigidBodyTransform transformToPoseFrame = worldFrame.getTransformToDesiredFrame(poseFrame);
      transformToPoseFrame.transform(position);
      transformToPoseFrame.transform(linearVelocity);

      EuclideanTrajectoryPoint expectedSimpleEuclideanTrajectoryPoint = new EuclideanTrajectoryPoint();

      expectedSimpleEuclideanTrajectoryPoint.setTime(time);
      expectedSimpleEuclideanTrajectoryPoint.getPosition().set(position);
      expectedSimpleEuclideanTrajectoryPoint.getLinearVelocity().set(linearVelocity);

      assertEquals(3.4, simpleEuclideanTrajectoryPoint.getTime(), 1e-7);
      assertEquals(3.4, expectedSimpleEuclideanTrajectoryPoint.getTime(), 1e-7);
      assertTrue(expectedSimpleEuclideanTrajectoryPoint.epsilonEquals(simpleEuclideanTrajectoryPoint, 1e-10));
   }

   @Test
   public void testConstructors()
   {
      double epsilon = 1.0e-14;
      Random random = new Random(21651016L);

      double expectedTime = 0.0;
      Point3D expectedPosition = new Point3D();
      Vector3D expectedLinearVelocity = new Vector3D();

      EuclideanTrajectoryPoint testedSimpleEuclideanTrajectoryPoint = new EuclideanTrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedLinearVelocity,
            testedSimpleEuclideanTrajectoryPoint, epsilon);

      expectedTime = 0.0;
      expectedPosition = new Point3D();
      expectedLinearVelocity = new Vector3D();
      testedSimpleEuclideanTrajectoryPoint = new EuclideanTrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedLinearVelocity,
            testedSimpleEuclideanTrajectoryPoint, epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      expectedLinearVelocity = RandomGeometry.nextVector3D(random);

      testedSimpleEuclideanTrajectoryPoint = new EuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedLinearVelocity,
            testedSimpleEuclideanTrajectoryPoint, epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      expectedLinearVelocity = RandomGeometry.nextVector3D(random);

      EuclideanTrajectoryPoint expectedSimpleEuclideanTrajectoryPoint = new EuclideanTrajectoryPoint(expectedTime, expectedPosition,
            expectedLinearVelocity);

      testedSimpleEuclideanTrajectoryPoint = new EuclideanTrajectoryPoint(expectedSimpleEuclideanTrajectoryPoint);

      assertTrue(expectedSimpleEuclideanTrajectoryPoint.epsilonEquals(testedSimpleEuclideanTrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedSimpleEuclideanTrajectoryPoint.getTime(),
            expectedPosition, expectedLinearVelocity, testedSimpleEuclideanTrajectoryPoint, epsilon);

      final double expectedFinalTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      final Point3D expectedFinalPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      final Vector3D expectedFinalLinearVelocity = RandomGeometry.nextVector3D(random);

      EuclideanTrajectoryPoint expectedEuclideanTrajectoryPoint = new EuclideanTrajectoryPoint();
      expectedEuclideanTrajectoryPoint.setTime(expectedFinalTime);
      expectedEuclideanTrajectoryPoint.getPosition().set(expectedFinalPosition);
      expectedEuclideanTrajectoryPoint.getLinearVelocity().set(expectedFinalLinearVelocity);

      testedSimpleEuclideanTrajectoryPoint = new EuclideanTrajectoryPoint(expectedEuclideanTrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalTime, expectedFinalPosition,
            expectedFinalLinearVelocity, testedSimpleEuclideanTrajectoryPoint, epsilon);

   }

   @Test
   public void testSetters()
   {
      double epsilon = 1.0e-14;
      Random random = new Random(21651016L);

      double expectedTime = 0.0;
      Point3D expectedPosition = new Point3D();
      Vector3D expectedLinearVelocity = new Vector3D();

      final EuclideanTrajectoryPoint testedSimpleEuclideanTrajectoryPoint = new EuclideanTrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedLinearVelocity,
            testedSimpleEuclideanTrajectoryPoint, epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      expectedLinearVelocity = RandomGeometry.nextVector3D(random);

      testedSimpleEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedLinearVelocity,
            testedSimpleEuclideanTrajectoryPoint, epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      expectedLinearVelocity = RandomGeometry.nextVector3D(random);

      testedSimpleEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedLinearVelocity,
            testedSimpleEuclideanTrajectoryPoint, epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      expectedLinearVelocity = RandomGeometry.nextVector3D(random);

      testedSimpleEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedLinearVelocity,
            testedSimpleEuclideanTrajectoryPoint, epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      expectedLinearVelocity = RandomGeometry.nextVector3D(random);

      EuclideanTrajectoryPoint expectedSimpleEuclideanTrajectoryPoint = new EuclideanTrajectoryPoint(expectedTime, expectedPosition,
            expectedLinearVelocity);

      testedSimpleEuclideanTrajectoryPoint.set(expectedSimpleEuclideanTrajectoryPoint);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      expectedLinearVelocity = RandomGeometry.nextVector3D(random);

      expectedSimpleEuclideanTrajectoryPoint = new EuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      testedSimpleEuclideanTrajectoryPoint.set(expectedSimpleEuclideanTrajectoryPoint);

      assertTrue(expectedSimpleEuclideanTrajectoryPoint.epsilonEquals(testedSimpleEuclideanTrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedSimpleEuclideanTrajectoryPoint.getTime(),
            expectedPosition, expectedLinearVelocity, testedSimpleEuclideanTrajectoryPoint, epsilon);

      final double expectedFinalTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      final Point3D expectedFinalPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      final Vector3D expectedFinalLinearVelocity = RandomGeometry.nextVector3D(random);

      EuclideanTrajectoryPoint expectedEuclideanTrajectoryPoint = new EuclideanTrajectoryPoint();
      expectedEuclideanTrajectoryPoint.setTime(expectedFinalTime);
      expectedEuclideanTrajectoryPoint.getPosition().set(expectedFinalPosition);
      expectedEuclideanTrajectoryPoint.getLinearVelocity().set(expectedFinalLinearVelocity);

      testedSimpleEuclideanTrajectoryPoint.set(expectedEuclideanTrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalTime, expectedFinalPosition,
            expectedFinalLinearVelocity, testedSimpleEuclideanTrajectoryPoint, epsilon);

   }

   @Test
   public void testChangeFrame() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      Point3D expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      Vector3D expectedLinearVelocity = new Vector3D(RandomGeometry.nextVector3D(random));
      EuclideanTrajectoryPoint testedSimpleEuclideanTrajectoryPoint = new EuclideanTrajectoryPoint(expectedTime, expectedPosition,
            expectedLinearVelocity);

      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = EuclidFrameRandomTools.nextReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : expectedFrame);

         expectedPosition.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));
         expectedLinearVelocity.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));
         testedSimpleEuclideanTrajectoryPoint.applyTransform(worldFrame.getTransformToDesiredFrame(expectedFrame));

         assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedLinearVelocity,
               testedSimpleEuclideanTrajectoryPoint, epsilon);
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);

      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      Point3D expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      Vector3D expectedLinearVelocity = RandomGeometry.nextVector3D(random);
      EuclideanTrajectoryPoint testedSimpleEuclideanTrajectoryPoint = new EuclideanTrajectoryPoint(expectedTime, expectedPosition,
            expectedLinearVelocity);

      expectedTime = 0.0;
      expectedPosition.set(0.0, 0.0, 0.0);
      expectedLinearVelocity.set(0.0, 0.0, 0.0);
      testedSimpleEuclideanTrajectoryPoint.setToZero();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedLinearVelocity,
            testedSimpleEuclideanTrajectoryPoint, epsilon);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      expectedLinearVelocity = RandomGeometry.nextVector3D(random);
      testedSimpleEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      expectedTime = 0.0;
      expectedPosition.set(0.0, 0.0, 0.0);
      expectedLinearVelocity.set(0.0, 0.0, 0.0);
      testedSimpleEuclideanTrajectoryPoint.setToZero();

      assertTrajectoryPointContainsExpectedData(expectedTime, expectedPosition, expectedLinearVelocity,
            testedSimpleEuclideanTrajectoryPoint, epsilon);
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(21651016L);

      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      Point3D expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      Vector3D expectedLinearVelocity = RandomGeometry.nextVector3D(random);
      EuclideanTrajectoryPoint testedSimpleEuclideanTrajectoryPoint = new EuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      testedSimpleEuclideanTrajectoryPoint.setToNaN();
      assertTrue(Double.isNaN(testedSimpleEuclideanTrajectoryPoint.getTime()));
      assertTrue(testedSimpleEuclideanTrajectoryPoint.containsNaN());

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
      expectedLinearVelocity = RandomGeometry.nextVector3D(random);
      testedSimpleEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      testedSimpleEuclideanTrajectoryPoint.setToNaN();

      assertTrue(Double.isNaN(testedSimpleEuclideanTrajectoryPoint.getTime()));
      assertTrue(testedSimpleEuclideanTrajectoryPoint.containsNaN());
   }

   static void assertTrajectoryPointContainsExpectedData(double expectedTime, Point3D expectedPosition,
         Vector3D expectedLinearVelocity,
         EuclideanTrajectoryPoint testedSimpleEuclideanTrajectoryPoint, double epsilon)
   {
      assertEquals(expectedTime, testedSimpleEuclideanTrajectoryPoint.getTime(), epsilon);
      assertTrue(expectedPosition.epsilonEquals(testedSimpleEuclideanTrajectoryPoint.getPosition(), epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(testedSimpleEuclideanTrajectoryPoint.getLinearVelocity(), epsilon));

      Point3D actualPosition = new Point3D();
      Vector3D actualLinearVelocity = new Vector3D();

      actualPosition.set(testedSimpleEuclideanTrajectoryPoint.getPosition());
      actualLinearVelocity.set(testedSimpleEuclideanTrajectoryPoint.getLinearVelocity());

      assertTrue(expectedPosition.epsilonEquals(actualPosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualLinearVelocity, epsilon));

      Point3D actualFramePosition = new Point3D();
      Vector3D actualFrameLinearVelocity = new Vector3D();

      actualFramePosition.set(testedSimpleEuclideanTrajectoryPoint.getPosition());
      actualFrameLinearVelocity.set(testedSimpleEuclideanTrajectoryPoint.getLinearVelocity());

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));

      actualFramePosition = new Point3D();
      actualFrameLinearVelocity = new Vector3D();

      actualFramePosition.set(testedSimpleEuclideanTrajectoryPoint.getPosition());
      actualFrameLinearVelocity.set(testedSimpleEuclideanTrajectoryPoint.getLinearVelocity());

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
   }

   @Test
   public void testSomeSetsAngGets()
   {
      EuclideanTrajectoryPoint simpleEuclideanTrajectoryPoint = new EuclideanTrajectoryPoint();

      EuclideanTrajectoryPoint simpleTrajectoryPoint = new EuclideanTrajectoryPoint();

      double time = 3.4;
      Point3D position = new Point3D(1.0, 2.1, 3.7);
      Vector3D linearVelocity = new Vector3D(-0.4, 1.2, 3.3);

      simpleTrajectoryPoint.set(time, position, linearVelocity);
      simpleEuclideanTrajectoryPoint.set(simpleTrajectoryPoint);

      // Check some get calls:
      Point3D pointForVerification = new Point3D();
      Vector3D linearVelocityForVerification = new Vector3D();

      pointForVerification.set(simpleEuclideanTrajectoryPoint.getPosition());
      linearVelocityForVerification.set(simpleEuclideanTrajectoryPoint.getLinearVelocity());

      assertEquals(time, simpleEuclideanTrajectoryPoint.getTime(), 1e-10);
      assertTrue(pointForVerification.epsilonEquals(position, 1e-10));
      assertTrue(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-10));

      // Check NaN calls:
      assertFalse(simpleEuclideanTrajectoryPoint.containsNaN());
      simpleEuclideanTrajectoryPoint.getPosition().setToNaN();
      assertTrue(simpleEuclideanTrajectoryPoint.containsNaN());
      simpleEuclideanTrajectoryPoint.getPosition().setToZero();

      assertFalse(simpleEuclideanTrajectoryPoint.containsNaN());
      simpleEuclideanTrajectoryPoint.getLinearVelocity().setToNaN();
      assertTrue(simpleEuclideanTrajectoryPoint.containsNaN());
      simpleEuclideanTrajectoryPoint.getLinearVelocity().setToZero();

      position.set(simpleEuclideanTrajectoryPoint.getPosition());
      linearVelocity.set(simpleEuclideanTrajectoryPoint.getLinearVelocity());

      // Make sure they are all equal to zero:
      assertTrue(position.epsilonEquals(new Point3D(), 1e-10));
      assertTrue(linearVelocity.epsilonEquals(new Vector3D(), 1e-10));

      time = 9.9;
      pointForVerification.set(3.9, 2.2, 1.1);
      linearVelocityForVerification.set(8.8, 1.4, 9.22);

      assertFalse(Math.abs(simpleEuclideanTrajectoryPoint.getTime() - time) < 1e-7);
      assertFalse(pointForVerification.epsilonEquals(position, 1e-7));
      assertFalse(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-7));

      simpleEuclideanTrajectoryPoint.set(time, pointForVerification, linearVelocityForVerification);

      position.set(simpleEuclideanTrajectoryPoint.getPosition());
      linearVelocity.set(simpleEuclideanTrajectoryPoint.getLinearVelocity());

      assertEquals(time, simpleEuclideanTrajectoryPoint.getTime(), 1e-10);
      assertTrue(pointForVerification.epsilonEquals(position, 1e-10));
      assertTrue(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-10));

      EuclideanTrajectoryPoint simpleEuclideanTrajectoryPointTwo = new EuclideanTrajectoryPoint();

      double positionDistance = simpleEuclideanTrajectoryPoint.positionDistance(simpleEuclideanTrajectoryPointTwo);
      assertEquals(4.610856753359402, positionDistance, 1e-7);
      assertFalse(simpleEuclideanTrajectoryPoint.epsilonEquals(simpleEuclideanTrajectoryPointTwo, 1e-7));

      simpleEuclideanTrajectoryPointTwo.set(simpleEuclideanTrajectoryPoint);
      positionDistance = simpleEuclideanTrajectoryPoint.positionDistance(simpleEuclideanTrajectoryPointTwo);
      assertEquals(0.0, positionDistance, 1e-7);
      assertTrue(simpleEuclideanTrajectoryPoint.epsilonEquals(simpleEuclideanTrajectoryPointTwo, 1e-7));

      EuclideanTrajectoryPoint simplePoint = new EuclideanTrajectoryPoint();
      simplePoint.set(simpleEuclideanTrajectoryPoint);

      simpleEuclideanTrajectoryPoint.setToNaN();
      assertTrue(simpleEuclideanTrajectoryPoint.containsNaN());
      positionDistance = simpleEuclideanTrajectoryPoint.positionDistance(simpleEuclideanTrajectoryPointTwo);
      assertTrue(Double.isNaN(positionDistance));
      assertFalse(simpleEuclideanTrajectoryPoint.epsilonEquals(simpleEuclideanTrajectoryPointTwo, 1e-7));

      EuclideanTrajectoryPointBasics trajectoryPointAsInterface = simplePoint;
      simpleEuclideanTrajectoryPoint.set(trajectoryPointAsInterface);

      positionDistance = simpleEuclideanTrajectoryPoint.positionDistance(simpleEuclideanTrajectoryPointTwo);
      assertEquals(0.0, positionDistance, 1e-7);

      assertTrue(simpleEuclideanTrajectoryPoint.epsilonEquals(simpleEuclideanTrajectoryPointTwo, 1e-7));

      String string = simpleEuclideanTrajectoryPoint.toString();
      String expectedString = "Euclidean trajectory point: [time= 9.900, position=( 3.900,  2.200,  1.100 ), linear velocity=( 8.800,  1.400,  9.220 )]";
      assertEquals(expectedString, string);
   }

   @Test
   public void testSomeMoreSettersAndGetters()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      EuclideanTrajectoryPoint simpleEuclideanTrajectoryPoint = new EuclideanTrajectoryPoint();

      double time = 3.4;
      Point3D position = new Point3D(1.0, 2.1, 3.7);
      Vector3D linearVelocity = new Vector3D(-0.4, 1.2, 3.3);

      simpleEuclideanTrajectoryPoint.setTime(time);
      simpleEuclideanTrajectoryPoint.getPosition().set(position);
      simpleEuclideanTrajectoryPoint.getLinearVelocity().set(linearVelocity);

      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose3D(worldFrame));

      FramePoint3D poseFramePosition = new FramePoint3D(worldFrame, new Point3D(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameQuaternion poseOrientation = new FrameQuaternion(worldFrame, new AxisAngle(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      simpleEuclideanTrajectoryPoint.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));

      assertFalse(position.epsilonEquals(simpleEuclideanTrajectoryPoint.getPosition(), 1e-10));
      assertFalse(linearVelocity.epsilonEquals(simpleEuclideanTrajectoryPoint.getLinearVelocity(), 1e-10));

      position.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));
      linearVelocity.applyTransform(worldFrame.getTransformToDesiredFrame(poseFrame));

      assertTrue(position.epsilonEquals(simpleEuclideanTrajectoryPoint.getPosition(), 1e-10));
      assertTrue(linearVelocity.epsilonEquals(simpleEuclideanTrajectoryPoint.getLinearVelocity(), 1e-10));


      EuclideanTrajectoryPoint simpleEuclideanTrajectoryPointTwo = new EuclideanTrajectoryPoint();
      simpleEuclideanTrajectoryPointTwo.setTime(time);
      simpleEuclideanTrajectoryPointTwo.getPosition().set(position);
      simpleEuclideanTrajectoryPointTwo.getLinearVelocity().set(linearVelocity);
      assertTrue(simpleEuclideanTrajectoryPointTwo.epsilonEquals(simpleEuclideanTrajectoryPoint, 1e-10));

      simpleEuclideanTrajectoryPointTwo = new EuclideanTrajectoryPoint();
      simpleEuclideanTrajectoryPointTwo.set(time, position, linearVelocity);
      assertTrue(simpleEuclideanTrajectoryPointTwo.epsilonEquals(simpleEuclideanTrajectoryPoint, 1e-10));

      simpleEuclideanTrajectoryPointTwo = new EuclideanTrajectoryPoint();
      EuclideanWaypoint simpleEuclideanWaypoint = new EuclideanWaypoint();
      simpleEuclideanWaypoint.set(simpleEuclideanTrajectoryPoint);
      simpleEuclideanTrajectoryPointTwo.set(time, simpleEuclideanWaypoint);
      assertTrue(simpleEuclideanTrajectoryPointTwo.epsilonEquals(simpleEuclideanTrajectoryPoint, 1e-10));

      simpleEuclideanTrajectoryPointTwo = new EuclideanTrajectoryPoint();
      simpleEuclideanTrajectoryPointTwo.set(time, simpleEuclideanWaypoint);
      assertTrue(simpleEuclideanTrajectoryPointTwo.epsilonEquals(simpleEuclideanTrajectoryPoint, 1e-10));


      simpleEuclideanTrajectoryPointTwo = new EuclideanTrajectoryPoint();
      EuclideanWaypointBasics euclideanWaypoint = simpleEuclideanTrajectoryPoint;

      simpleEuclideanTrajectoryPointTwo.set(time, euclideanWaypoint);
      assertTrue(simpleEuclideanTrajectoryPointTwo.epsilonEquals(simpleEuclideanTrajectoryPoint, 1e-10));




      simpleEuclideanTrajectoryPointTwo = new EuclideanTrajectoryPoint();
      euclideanWaypoint = new EuclideanWaypoint();
      euclideanWaypoint.set(simpleEuclideanTrajectoryPoint);

      simpleEuclideanTrajectoryPointTwo.set(time, euclideanWaypoint);
      assertTrue(simpleEuclideanTrajectoryPointTwo.epsilonEquals(simpleEuclideanTrajectoryPoint, 1e-10));


      Point3D positionToPack = new Point3D(simpleEuclideanTrajectoryPoint.getPosition());
      Vector3D linearVelocityToPack = new Vector3D(simpleEuclideanTrajectoryPoint.getLinearVelocity());

      simpleEuclideanTrajectoryPointTwo = new EuclideanTrajectoryPoint();
      simpleEuclideanTrajectoryPointTwo.set(time, positionToPack, linearVelocityToPack);
      assertTrue(simpleEuclideanTrajectoryPointTwo.epsilonEquals(simpleEuclideanTrajectoryPoint, 1e-10));




      positionToPack = new Point3D(simpleEuclideanTrajectoryPoint.getPosition());
      linearVelocityToPack = new Vector3D(simpleEuclideanTrajectoryPoint.getLinearVelocity());

      simpleEuclideanTrajectoryPointTwo = new EuclideanTrajectoryPoint();
      simpleEuclideanTrajectoryPointTwo.set(time, positionToPack, linearVelocityToPack);
      assertTrue(simpleEuclideanTrajectoryPointTwo.epsilonEquals(simpleEuclideanTrajectoryPoint, 1e-10));

      assertTrue(simpleEuclideanTrajectoryPointTwo.getPosition().epsilonEquals(positionToPack, 1e-10));
      assertTrue(simpleEuclideanTrajectoryPointTwo.getLinearVelocity().epsilonEquals(linearVelocityToPack, 1e-10));

   }
}
