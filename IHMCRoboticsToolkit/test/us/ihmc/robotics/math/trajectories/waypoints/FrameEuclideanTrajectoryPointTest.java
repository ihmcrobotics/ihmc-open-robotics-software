package us.ihmc.robotics.math.trajectories.waypoints;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.frameObjects.FrameEuclideanWaypoint;
import us.ihmc.robotics.geometry.transformables.EuclideanWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanTrajectoryPointInterface;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameEuclideanTrajectoryPointTest
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

      FrameEuclideanTrajectoryPoint frameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(worldFrame);
      SimpleEuclideanTrajectoryPoint simpleTrajectoryPoint = new SimpleEuclideanTrajectoryPoint();

      double time = 3.4;
      Point3d position = new Point3d(1.0, 2.1, 3.7);
      Vector3d linearVelocity = new Vector3d(-0.4, 1.2, 3.3);

      simpleTrajectoryPoint.set(time, position, linearVelocity);
      frameEuclideanTrajectoryPoint.setIncludingFrame(worldFrame, simpleTrajectoryPoint);
      frameEuclideanTrajectoryPoint.changeFrame(poseFrame);

      // Do some checks:
      RigidBodyTransform transformToPoseFrame = worldFrame.getTransformToDesiredFrame(poseFrame);
      transformToPoseFrame.transform(position);
      transformToPoseFrame.transform(linearVelocity);

      FrameEuclideanTrajectoryPoint expectedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(poseFrame);

      expectedFrameEuclideanTrajectoryPoint.setTime(time);
      expectedFrameEuclideanTrajectoryPoint.setPosition(position);

      expectedFrameEuclideanTrajectoryPoint.setLinearVelocity(linearVelocity);

      assertEquals(3.4, frameEuclideanTrajectoryPoint.getTime(), 1e-7);
      assertEquals(3.4, expectedFrameEuclideanTrajectoryPoint.getTime(), 1e-7);
      assertTrue(expectedFrameEuclideanTrajectoryPoint.epsilonEquals(frameEuclideanTrajectoryPoint, 1e-10));
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
      FramePoint expectedPosition = new FramePoint(expectedFrame);
      FrameVector expectedLinearVelocity = new FrameVector(expectedFrame);

      FrameEuclideanTrajectoryPoint testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = aFrame;
      expectedTime = 0.0;
      expectedPosition = new FramePoint(expectedFrame);
      expectedLinearVelocity = new FrameVector(expectedFrame);
      testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedFrame);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      FrameEuclideanTrajectoryPoint expectedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedFrameEuclideanTrajectoryPoint);

      assertTrue(expectedFrameEuclideanTrajectoryPoint.epsilonEquals(testedFrameEuclideanTrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedFrameEuclideanTrajectoryPoint.getReferenceFrame(), expectedFrameEuclideanTrajectoryPoint.getTime(),
            expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      final ReferenceFrame expectedFinalFrame = aFrame;
      final double expectedFinalTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      final FramePoint expectedFinalPosition = FramePoint.generateRandomFramePoint(random, expectedFinalFrame, 10.0, 10.0, 10.0);
      final FrameVector expectedFinalLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFinalFrame);

      SimpleEuclideanTrajectoryPoint expectedEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint();
      expectedEuclideanTrajectoryPoint.setTime(expectedFinalTime);
      expectedEuclideanTrajectoryPoint.setPosition(expectedFinalPosition.getPoint());
      expectedEuclideanTrajectoryPoint.setLinearVelocity(expectedFinalLinearVelocity.getVector());

      testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedFinalFrame, expectedEuclideanTrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalFrame, expectedFinalTime, expectedFinalPosition, expectedFinalLinearVelocity,
            testedFrameEuclideanTrajectoryPoint, epsilon);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetters()
   {
      double epsilon = 1.0e-20;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame aFrame = ReferenceFrame.generateRandomReferenceFrame("aFrame", random, worldFrame);

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FramePoint expectedPosition = new FramePoint(expectedFrame);
      FrameVector expectedLinearVelocity = new FrameVector(expectedFrame);

      final FrameEuclideanTrajectoryPoint testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameEuclideanTrajectoryPoint.set(expectedTime, expectedPosition.getPoint(), expectedLinearVelocity.getVector());

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = aFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameEuclideanTrajectoryPoint.setIncludingFrame(expectedTime, expectedPosition, expectedLinearVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      FrameEuclideanTrajectoryPoint expectedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      testedFrameEuclideanTrajectoryPoint.setIncludingFrame(expectedFrameEuclideanTrajectoryPoint);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      expectedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      testedFrameEuclideanTrajectoryPoint.set(expectedFrameEuclideanTrajectoryPoint);

      assertTrue(expectedFrameEuclideanTrajectoryPoint.epsilonEquals(testedFrameEuclideanTrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedFrameEuclideanTrajectoryPoint.getReferenceFrame(), expectedFrameEuclideanTrajectoryPoint.getTime(),
            expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      final ReferenceFrame expectedFinalFrame = aFrame;
      final double expectedFinalTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      final FramePoint expectedFinalPosition = FramePoint.generateRandomFramePoint(random, expectedFinalFrame, 10.0, 10.0, 10.0);
      final FrameVector expectedFinalLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFinalFrame);

      SimpleEuclideanTrajectoryPoint expectedEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint();
      expectedEuclideanTrajectoryPoint.setTime(expectedFinalTime);
      expectedEuclideanTrajectoryPoint.setPosition(expectedFinalPosition.getPoint());
      expectedEuclideanTrajectoryPoint.setLinearVelocity(expectedFinalLinearVelocity.getVector());

      testedFrameEuclideanTrajectoryPoint.setIncludingFrame(expectedFinalFrame, expectedEuclideanTrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalFrame, expectedFinalTime, expectedFinalPosition, expectedFinalLinearVelocity,
            testedFrameEuclideanTrajectoryPoint, epsilon);

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
      FramePoint expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameVector expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameEuclideanTrajectoryPoint testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = ReferenceFrame.generateRandomReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : expectedFrame);

         expectedPosition.changeFrame(expectedFrame);
         expectedLinearVelocity.changeFrame(expectedFrame);
         testedFrameEuclideanTrajectoryPoint.changeFrame(expectedFrame);

         assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);
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
      FramePoint expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameVector expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameEuclideanTrajectoryPoint testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero();
      expectedLinearVelocity.setToZero();
      testedFrameEuclideanTrajectoryPoint.setToZero();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedFrameEuclideanTrajectoryPoint.setIncludingFrame(expectedTime, expectedPosition, expectedLinearVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero(expectedFrame);
      expectedLinearVelocity.setToZero(expectedFrame);
      testedFrameEuclideanTrajectoryPoint.setToZero(expectedFrame);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      FramePoint expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameVector expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameEuclideanTrajectoryPoint testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      testedFrameEuclideanTrajectoryPoint.setToNaN();
      assertTrue(Double.isNaN(testedFrameEuclideanTrajectoryPoint.getTime()));
      assertTrue(testedFrameEuclideanTrajectoryPoint.containsNaN());

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedFrameEuclideanTrajectoryPoint.setIncludingFrame(expectedTime, expectedPosition, expectedLinearVelocity);

      testedFrameEuclideanTrajectoryPoint.setToNaN(expectedFrame);

      assertTrue(expectedFrame == testedFrameEuclideanTrajectoryPoint.getReferenceFrame());
      assertTrue(Double.isNaN(testedFrameEuclideanTrajectoryPoint.getTime()));
      assertTrue(testedFrameEuclideanTrajectoryPoint.containsNaN());
   }

   static void assertTrajectoryPointContainsExpectedData(ReferenceFrame expectedFrame, double expectedTime, FramePoint expectedPosition,
         FrameVector expectedLinearVelocity, FrameEuclideanTrajectoryPoint testedFrameEuclideanTrajectoryPoint, double epsilon)
   {
      assertTrue(expectedFrame == testedFrameEuclideanTrajectoryPoint.getReferenceFrame());
      assertEquals(expectedTime, testedFrameEuclideanTrajectoryPoint.getTime(), epsilon);
      assertTrue(expectedPosition.epsilonEquals(testedFrameEuclideanTrajectoryPoint.getGeometryObject().getPosition(), epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(testedFrameEuclideanTrajectoryPoint.getGeometryObject().getLinearVelocity(), epsilon));

      Point3d actualPosition = new Point3d();
      Vector3d actualLinearVelocity = new Vector3d();

      testedFrameEuclideanTrajectoryPoint.getPosition(actualPosition);
      testedFrameEuclideanTrajectoryPoint.getLinearVelocity(actualLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualPosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualLinearVelocity, epsilon));

      FramePoint actualFramePosition = new FramePoint();
      FrameVector actualFrameLinearVelocity = new FrameVector();

      testedFrameEuclideanTrajectoryPoint.getPositionIncludingFrame(actualFramePosition);
      testedFrameEuclideanTrajectoryPoint.getLinearVelocityIncludingFrame(actualFrameLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));

      actualFramePosition = new FramePoint(expectedFrame);
      actualFrameLinearVelocity = new FrameVector(expectedFrame);

      testedFrameEuclideanTrajectoryPoint.getPosition(actualFramePosition);
      testedFrameEuclideanTrajectoryPoint.getLinearVelocity(actualFrameLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSomeSetsAngGets()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FrameEuclideanTrajectoryPoint FrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(worldFrame);
      FrameEuclideanTrajectoryPoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      SimpleEuclideanTrajectoryPoint simpleTrajectoryPoint = new SimpleEuclideanTrajectoryPoint();

      double time = 3.4;
      Point3d position = new Point3d(1.0, 2.1, 3.7);
      Vector3d linearVelocity = new Vector3d(-0.4, 1.2, 3.3);

      simpleTrajectoryPoint.set(time, position, linearVelocity);
      FrameEuclideanTrajectoryPoint.setIncludingFrame(worldFrame, simpleTrajectoryPoint);

      // Check some get calls: 
      FramePoint pointForVerification = new FramePoint(worldFrame);
      FrameVector linearVelocityForVerification = new FrameVector(worldFrame);

      FrameEuclideanTrajectoryPoint.getPosition(pointForVerification);
      FrameEuclideanTrajectoryPoint.getLinearVelocity(linearVelocityForVerification);

      assertEquals(time, FrameEuclideanTrajectoryPoint.getTime(), 1e-10);
      assertTrue(pointForVerification.getPoint().epsilonEquals(position, 1e-10));
      assertTrue(linearVelocityForVerification.getVector().epsilonEquals(linearVelocity, 1e-10));

      // Check NaN calls:
      assertFalse(FrameEuclideanTrajectoryPoint.containsNaN());
      FrameEuclideanTrajectoryPoint.setPositionToNaN();
      assertTrue(FrameEuclideanTrajectoryPoint.containsNaN());
      FrameEuclideanTrajectoryPoint.setPositionToZero();

      assertFalse(FrameEuclideanTrajectoryPoint.containsNaN());
      FrameEuclideanTrajectoryPoint.setLinearVelocityToNaN();
      assertTrue(FrameEuclideanTrajectoryPoint.containsNaN());
      FrameEuclideanTrajectoryPoint.setLinearVelocityToZero();

      FrameEuclideanTrajectoryPoint.getPosition(position);
      FrameEuclideanTrajectoryPoint.getLinearVelocity(linearVelocity);

      // Make sure they are all equal to zero:
      assertTrue(position.epsilonEquals(new Point3d(), 1e-10));
      assertTrue(linearVelocity.epsilonEquals(new Vector3d(), 1e-10));

      time = 9.9;
      pointForVerification.set(3.9, 2.2, 1.1);
      linearVelocityForVerification.set(8.8, 1.4, 9.22);

      assertFalse(Math.abs(FrameEuclideanTrajectoryPoint.getTime() - time) < 1e-7);
      assertFalse(pointForVerification.getPoint().epsilonEquals(position, 1e-7));
      assertFalse(linearVelocityForVerification.getVector().epsilonEquals(linearVelocity, 1e-7));

      FrameEuclideanTrajectoryPoint.set(time, pointForVerification, linearVelocityForVerification);

      FrameEuclideanTrajectoryPoint.getPosition(position);
      FrameEuclideanTrajectoryPoint.getLinearVelocity(linearVelocity);

      assertEquals(time, FrameEuclideanTrajectoryPoint.getTime(), 1e-10);
      assertTrue(pointForVerification.getPoint().epsilonEquals(position, 1e-10));
      assertTrue(linearVelocityForVerification.getVector().epsilonEquals(linearVelocity, 1e-10));

      FrameEuclideanTrajectoryPoint FrameEuclideanTrajectoryPointTwo = new FrameEuclideanTrajectoryPoint(worldFrame);

      double positionDistance = FrameEuclideanTrajectoryPoint.positionDistance(FrameEuclideanTrajectoryPointTwo);
      assertEquals(4.610856753359402, positionDistance, 1e-7);
      assertFalse(FrameEuclideanTrajectoryPoint.epsilonEquals(FrameEuclideanTrajectoryPointTwo, 1e-7));

      FrameEuclideanTrajectoryPointTwo.set(FrameEuclideanTrajectoryPoint);
      positionDistance = FrameEuclideanTrajectoryPoint.positionDistance(FrameEuclideanTrajectoryPointTwo);
      assertEquals(0.0, positionDistance, 1e-7);
      assertTrue(FrameEuclideanTrajectoryPoint.epsilonEquals(FrameEuclideanTrajectoryPointTwo, 1e-7));

      SimpleEuclideanTrajectoryPoint simplePoint = new SimpleEuclideanTrajectoryPoint();
      FrameEuclideanTrajectoryPoint.get(simplePoint);

      FrameEuclideanTrajectoryPoint.setToNaN();
      assertTrue(FrameEuclideanTrajectoryPoint.containsNaN());
      positionDistance = FrameEuclideanTrajectoryPoint.positionDistance(FrameEuclideanTrajectoryPointTwo);
      assertTrue(Double.isNaN(positionDistance));
      assertFalse(FrameEuclideanTrajectoryPoint.epsilonEquals(FrameEuclideanTrajectoryPointTwo, 1e-7));

      EuclideanTrajectoryPointInterface<?> trajectoryPointAsInterface = simplePoint;
      FrameEuclideanTrajectoryPoint.set(trajectoryPointAsInterface);

      positionDistance = FrameEuclideanTrajectoryPoint.positionDistance(FrameEuclideanTrajectoryPointTwo);
      assertEquals(0.0, positionDistance, 1e-7);
      assertTrue(FrameEuclideanTrajectoryPoint.epsilonEquals(FrameEuclideanTrajectoryPointTwo, 1e-7));

      String string = FrameEuclideanTrajectoryPoint.toString();
      String expectedString = "Euclidean trajectory point: (time =  9.90, Euclidean trajectory point: (time =  9.90, Euclidean waypoint: [position = ( 3.90,  2.20,  1.10), linearVelocity = ( 8.80,  1.40,  9.22)].))";
      assertEquals(expectedString, string);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSomeMoreSettersAndGetters()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FrameEuclideanTrajectoryPoint frameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(worldFrame);
      frameEuclideanTrajectoryPoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      double time = 3.4;
      FramePoint position = new FramePoint(worldFrame, 1.0, 2.1, 3.7);
      FrameVector linearVelocity = new FrameVector(worldFrame, -0.4, 1.2, 3.3);

      frameEuclideanTrajectoryPoint.setTime(time);
      frameEuclideanTrajectoryPoint.setPosition(position);
      frameEuclideanTrajectoryPoint.setLinearVelocity(linearVelocity);

      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose(worldFrame));

      FramePoint poseFramePosition = new FramePoint(worldFrame, new Point3d(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameOrientation poseOrientation = new FrameOrientation(worldFrame, new AxisAngle4d(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      frameEuclideanTrajectoryPoint.changeFrame(poseFrame);

      assertFalse(position.epsilonEquals(frameEuclideanTrajectoryPoint.getPositionCopy(), 1e-10));
      assertFalse(linearVelocity.epsilonEquals(frameEuclideanTrajectoryPoint.getLinearVelocityCopy(), 1e-10));

      position.changeFrame(poseFrame);
      linearVelocity.changeFrame(poseFrame);

      assertTrue(position.epsilonEquals(frameEuclideanTrajectoryPoint.getPositionCopy(), 1e-10));
      assertTrue(linearVelocity.epsilonEquals(frameEuclideanTrajectoryPoint.getLinearVelocityCopy(), 1e-10));

      FrameEuclideanTrajectoryPoint frameEuclideanTrajectoryPointTwo = new FrameEuclideanTrajectoryPoint(poseFrame);
      frameEuclideanTrajectoryPointTwo.setTime(time);
      frameEuclideanTrajectoryPointTwo.setPosition(position);
      frameEuclideanTrajectoryPointTwo.setLinearVelocity(linearVelocity);
      assertTrue(frameEuclideanTrajectoryPointTwo.epsilonEquals(frameEuclideanTrajectoryPoint, 1e-10));

      frameEuclideanTrajectoryPointTwo = new FrameEuclideanTrajectoryPoint(worldFrame);
      frameEuclideanTrajectoryPointTwo.setIncludingFrame(poseFrame, time, position.getPointCopy(), linearVelocity.getVectorCopy());
      assertTrue(frameEuclideanTrajectoryPointTwo.epsilonEquals(frameEuclideanTrajectoryPoint, 1e-10));
   
      frameEuclideanTrajectoryPointTwo = new FrameEuclideanTrajectoryPoint(poseFrame);
      EuclideanWaypoint euclideanWaypoint = new EuclideanWaypoint();
      frameEuclideanTrajectoryPoint.getEuclideanWaypoint(euclideanWaypoint);
      frameEuclideanTrajectoryPointTwo.set(time, euclideanWaypoint);
      assertTrue(frameEuclideanTrajectoryPointTwo.epsilonEquals(frameEuclideanTrajectoryPoint, 1e-10));

      frameEuclideanTrajectoryPointTwo = new FrameEuclideanTrajectoryPoint(worldFrame);
      frameEuclideanTrajectoryPointTwo.setIncludingFrame(poseFrame, time, euclideanWaypoint);
      assertTrue(frameEuclideanTrajectoryPointTwo.epsilonEquals(frameEuclideanTrajectoryPoint, 1e-10));

      frameEuclideanTrajectoryPointTwo = new FrameEuclideanTrajectoryPoint(poseFrame);
      FrameEuclideanWaypoint frameEuclideanWaypoint = new FrameEuclideanWaypoint(poseFrame);
      frameEuclideanTrajectoryPoint.getFrameEuclideanWaypoint(frameEuclideanWaypoint);
      frameEuclideanTrajectoryPointTwo.set(time, frameEuclideanWaypoint);
      assertTrue(frameEuclideanTrajectoryPointTwo.epsilonEquals(frameEuclideanTrajectoryPoint, 1e-10));

      frameEuclideanTrajectoryPointTwo = new FrameEuclideanTrajectoryPoint(worldFrame);
      frameEuclideanTrajectoryPointTwo.setIncludingFrame(time, frameEuclideanWaypoint);
      assertTrue(frameEuclideanTrajectoryPointTwo.epsilonEquals(frameEuclideanTrajectoryPoint, 1e-10));

   }
}
