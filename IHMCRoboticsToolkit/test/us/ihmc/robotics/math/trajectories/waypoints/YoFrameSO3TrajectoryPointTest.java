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
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.transformables.TransformableQuat4d;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3TrajectoryPointInterface;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFrameSO3TrajectoryPointTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCommonUsageExample()
   {
      String namePrefix = "point";
      String nameSuffix = "toTest";
      YoVariableRegistry registry = new YoVariableRegistry("myRegistry");

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose(worldFrame));

      FramePoint poseFramePosition = new FramePoint(worldFrame, new Point3d(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameOrientation poseOrientation = new FrameOrientation(worldFrame, new AxisAngle4d(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      YoFrameSO3TrajectoryPoint yoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint(namePrefix, nameSuffix, registry, worldFrame, poseFrame);
      SimpleSO3TrajectoryPoint simpleTrajectoryPoint = new SimpleSO3TrajectoryPoint();

      double time = 3.4;
      TransformableQuat4d orientation = new TransformableQuat4d(new Quat4d(0.1, 0.22, 0.34, 0.56));
      orientation.normalize();

      Vector3d angularVelocity = new Vector3d(1.7, 8.4, 2.2);

      simpleTrajectoryPoint.set(time, orientation, angularVelocity);
      yoFrameSO3TrajectoryPoint.setIncludingFrame(worldFrame, simpleTrajectoryPoint);

      yoFrameSO3TrajectoryPoint.changeFrame(poseFrame);

      // Do some checks:
      RigidBodyTransform transformToPoseFrame = worldFrame.getTransformToDesiredFrame(poseFrame);
      orientation.applyTransform(transformToPoseFrame);
      transformToPoseFrame.transform(angularVelocity);

      namePrefix = "point";
      nameSuffix = "toVerify";
      YoFrameSO3TrajectoryPoint expectedYoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint(namePrefix, nameSuffix, registry, poseFrame);

      expectedYoFrameSO3TrajectoryPoint.setTime(time);
      expectedYoFrameSO3TrajectoryPoint.setOrientation(orientation);
      expectedYoFrameSO3TrajectoryPoint.setAngularVelocity(angularVelocity);

      assertEquals(3.4, yoFrameSO3TrajectoryPoint.getTime(), 1e-7);
      assertEquals(3.4, expectedYoFrameSO3TrajectoryPoint.getTime(), 1e-7);

      assertTrue(expectedYoFrameSO3TrajectoryPoint.epsilonEquals(yoFrameSO3TrajectoryPoint, 1e-10));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructor()
   {
      double epsilon = 1.0e-20;
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FrameOrientation expectedOrientation = new FrameOrientation(expectedFrame);
      FrameVector expectedAngularVelocity = new FrameVector(expectedFrame);

      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSO3TrajectoryPoint testedYoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint(expectedNamePrefix, expectedNameSuffix,
            new YoVariableRegistry("schnoop"), expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
            testedYoFrameSO3TrajectoryPoint, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetters()
   {
      double epsilon = 1.0e-20;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FrameOrientation expectedOrientation = new FrameOrientation(expectedFrame);
      FrameVector expectedAngularVelocity = new FrameVector(expectedFrame);

      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSO3TrajectoryPoint testedYoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint(expectedNamePrefix, expectedNameSuffix,
            new YoVariableRegistry("schnoop"), expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
            testedYoFrameSO3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedYoFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
            testedYoFrameSO3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedYoFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation.getQuaternion(), expectedAngularVelocity.getVector());

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
            testedYoFrameSO3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      YoFrameSO3TrajectoryPoint expectedYoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint("sdfsd", "asd", new YoVariableRegistry("asawe"), expectedFrame);

      testedYoFrameSO3TrajectoryPoint.set(expectedYoFrameSO3TrajectoryPoint);

      assertTrue(expectedYoFrameSO3TrajectoryPoint.epsilonEquals(testedYoFrameSO3TrajectoryPoint, epsilon));
      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, testedYoFrameSO3TrajectoryPoint.getReferenceFrame(),
            testedYoFrameSO3TrajectoryPoint.getTime(), testedYoFrameSO3TrajectoryPoint.getOrientation().getFrameOrientationCopy(),
            testedYoFrameSO3TrajectoryPoint.getAngularVelocity().getFrameVectorCopy(), testedYoFrameSO3TrajectoryPoint, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.2)
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
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSO3TrajectoryPoint testedYoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint(expectedNamePrefix, expectedNameSuffix,
            new YoVariableRegistry("schnoop"), expectedFrame);

      testedYoFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      ReferenceFrame[] randomFrames = new ReferenceFrame[10];
      randomFrames[0] = worldFrame;
      for (int i = 1; i < 10; i++)
         randomFrames[i] = ReferenceFrame.generateRandomReferenceFrame("randomFrame" + i, random,
               random.nextBoolean() ? worldFrame : randomFrames[random.nextInt(i)]);

      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = randomFrames[random.nextInt(10)];
         testedYoFrameSO3TrajectoryPoint.registerReferenceFrame(expectedFrame);

         expectedOrientation.changeFrame(expectedFrame);
         expectedAngularVelocity.changeFrame(expectedFrame);
         testedYoFrameSO3TrajectoryPoint.changeFrame(expectedFrame);

         assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
               testedYoFrameSO3TrajectoryPoint, epsilon);
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
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSO3TrajectoryPoint testedYoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint(expectedNamePrefix, expectedNameSuffix,
            new YoVariableRegistry("schnoop"), expectedFrame);
      testedYoFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedOrientation.setToZero();
      expectedAngularVelocity.setToZero();
      testedYoFrameSO3TrajectoryPoint.setToZero();

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
            testedYoFrameSO3TrajectoryPoint, epsilon);

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      testedYoFrameSO3TrajectoryPoint.registerReferenceFrame(expectedFrame);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedYoFrameSO3TrajectoryPoint.switchCurrentReferenceFrame(worldFrame);
      testedYoFrameSO3TrajectoryPoint.registerReferenceFrame(worldFrame);
      testedYoFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedOrientation.setToZero(expectedFrame);
      expectedAngularVelocity.setToZero(expectedFrame);
      testedYoFrameSO3TrajectoryPoint.switchCurrentReferenceFrame(expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
            testedYoFrameSO3TrajectoryPoint, epsilon);
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
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSO3TrajectoryPoint testedYoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint(expectedNamePrefix, expectedNameSuffix,
            new YoVariableRegistry("schnoop"), expectedFrame);
      testedYoFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedYoFrameSO3TrajectoryPoint.setToNaN();
      assertTrue(Double.isNaN(testedYoFrameSO3TrajectoryPoint.getTime()));
      assertTrue(testedYoFrameSO3TrajectoryPoint.getOrientation().containsNaN());
      assertTrue(testedYoFrameSO3TrajectoryPoint.getAngularVelocity().containsNaN());

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      testedYoFrameSO3TrajectoryPoint.registerReferenceFrame(expectedFrame);
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedYoFrameSO3TrajectoryPoint.switchCurrentReferenceFrame(worldFrame);
      testedYoFrameSO3TrajectoryPoint.registerReferenceFrame(worldFrame);
      testedYoFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedYoFrameSO3TrajectoryPoint.setToNaN(expectedFrame);

      assertTrue(expectedFrame == testedYoFrameSO3TrajectoryPoint.getReferenceFrame());
      assertTrue(Double.isNaN(testedYoFrameSO3TrajectoryPoint.getTime()));
      assertTrue(testedYoFrameSO3TrajectoryPoint.getOrientation().containsNaN());
      assertTrue(testedYoFrameSO3TrajectoryPoint.getAngularVelocity().containsNaN());
   }

   private void assertWaypointContainsExpectedData(String expectedNamePrefix, String expectedNameSuffix, ReferenceFrame expectedFrame, double expectedTime,
         FrameOrientation expectedOrientation, FrameVector expectedAngularVelocity, YoFrameSO3TrajectoryPoint testedYoFrameSO3TrajectoryPoint, double epsilon)
   {
      assertTrue(expectedFrame == testedYoFrameSO3TrajectoryPoint.getReferenceFrame());
      assertEquals(expectedTime, testedYoFrameSO3TrajectoryPoint.getTime(), epsilon);
      assertEquals(expectedNamePrefix, testedYoFrameSO3TrajectoryPoint.getNamePrefix());
      assertEquals(expectedNameSuffix, testedYoFrameSO3TrajectoryPoint.getNameSuffix());
      assertTrue(expectedOrientation.epsilonEquals(testedYoFrameSO3TrajectoryPoint.getOrientation().getFrameOrientationCopy(), epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(testedYoFrameSO3TrajectoryPoint.getAngularVelocity().getFrameVectorCopy(), epsilon));

      FrameSO3TrajectoryPoint actualFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint();
      testedYoFrameSO3TrajectoryPoint.getIncludingFrame(actualFrameSO3TrajectoryPoint);
      FrameSO3TrajectoryPointTest.assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
            actualFrameSO3TrajectoryPoint, epsilon);
      actualFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedFrame);
      testedYoFrameSO3TrajectoryPoint.get(actualFrameSO3TrajectoryPoint);
      FrameSO3TrajectoryPointTest.assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
            actualFrameSO3TrajectoryPoint, epsilon);

      Quat4d actualOrientation = new Quat4d();
      Vector3d actualAngularVelocity = new Vector3d();

      testedYoFrameSO3TrajectoryPoint.getOrientation(actualOrientation);
      testedYoFrameSO3TrajectoryPoint.getAngularVelocity(actualAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, epsilon));

      FrameOrientation actualFrameOrientation = new FrameOrientation(expectedFrame);
      FrameVector actualFrameAngularVelocity = new FrameVector(expectedFrame);

      testedYoFrameSO3TrajectoryPoint.getOrientation(actualFrameOrientation);
      testedYoFrameSO3TrajectoryPoint.getAngularVelocity(actualFrameAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));

      actualFrameOrientation = new FrameOrientation();
      actualFrameAngularVelocity = new FrameVector();

      testedYoFrameSO3TrajectoryPoint.getOrientationIncludingFrame(actualFrameOrientation);
      testedYoFrameSO3TrajectoryPoint.getAngularVelocityIncludingFrame(actualFrameAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSomeSetsAngGets()
   {
      String namePrefix = "point";
      String nameSuffix = "toTest";
      YoVariableRegistry registry = new YoVariableRegistry("myRegistry");

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoFrameSO3TrajectoryPoint yoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint(namePrefix, nameSuffix, registry, worldFrame);
      yoFrameSO3TrajectoryPoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      SimpleSO3TrajectoryPoint simpleTrajectoryPoint = new SimpleSO3TrajectoryPoint();

      double time = 3.4;
      TransformableQuat4d orientation = new TransformableQuat4d(new Quat4d(0.1, 0.22, 0.34, 0.56));
      orientation.normalize();

      Vector3d angularVelocity = new Vector3d(1.7, 8.4, 2.2);

      simpleTrajectoryPoint.set(time, orientation, angularVelocity);
      yoFrameSO3TrajectoryPoint.setIncludingFrame(worldFrame, simpleTrajectoryPoint);

      // Check some get calls: 
      YoFrameQuaternion quaternionForVerification = new YoFrameQuaternion("quaternionForVerification", worldFrame, registry);
      YoFrameVector angularVelocityForVerification = new YoFrameVector("angularVelocityForVerification", worldFrame, registry);

      yoFrameSO3TrajectoryPoint.getOrientation(quaternionForVerification);
      yoFrameSO3TrajectoryPoint.getAngularVelocity(angularVelocityForVerification);

      assertEquals(time, yoFrameSO3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(quaternionForVerification.getFrameOrientation().epsilonEquals(orientation, 1e-10));
      assertTrue(angularVelocityForVerification.getFrameTuple().epsilonEquals(angularVelocity, 1e-10));

      // Check NaN calls:
      assertFalse(yoFrameSO3TrajectoryPoint.containsNaN());
      yoFrameSO3TrajectoryPoint.setOrientationToNaN();
      assertTrue(yoFrameSO3TrajectoryPoint.containsNaN());
      yoFrameSO3TrajectoryPoint.setOrientationToZero();

      assertFalse(yoFrameSO3TrajectoryPoint.containsNaN());
      yoFrameSO3TrajectoryPoint.setAngularVelocityToNaN();
      assertTrue(yoFrameSO3TrajectoryPoint.containsNaN());
      yoFrameSO3TrajectoryPoint.setAngularVelocityToZero();
      assertFalse(yoFrameSO3TrajectoryPoint.containsNaN());

      yoFrameSO3TrajectoryPoint.getOrientation(orientation);
      yoFrameSO3TrajectoryPoint.getAngularVelocity(angularVelocity);

      // Make sure they are all equal to zero:
      assertTrue(orientation.epsilonEquals(new TransformableQuat4d(), 1e-10));
      assertTrue(angularVelocity.epsilonEquals(new Vector3d(), 1e-10));

      time = 9.9;
      quaternionForVerification.set(0.2, 0.6, 1.1);
      angularVelocityForVerification.set(7.1, 2.2, 3.33);

      assertFalse(Math.abs(yoFrameSO3TrajectoryPoint.getTime() - time) < 1e-7);
      assertFalse(quaternionForVerification.getFrameOrientation().epsilonEquals(orientation, 1e-7));
      assertFalse(angularVelocityForVerification.getFrameTuple().epsilonEquals(angularVelocity, 1e-7));

      yoFrameSO3TrajectoryPoint.set(time, quaternionForVerification, angularVelocityForVerification);

      yoFrameSO3TrajectoryPoint.getOrientation(orientation);
      yoFrameSO3TrajectoryPoint.getAngularVelocity(angularVelocity);

      assertEquals(time, yoFrameSO3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(quaternionForVerification.getFrameOrientation().epsilonEquals(orientation, 1e-10));
      assertTrue(angularVelocityForVerification.getFrameTuple().epsilonEquals(angularVelocity, 1e-10));

      YoFrameSO3TrajectoryPoint yoFrameSO3TrajectoryPointTwo = new YoFrameSO3TrajectoryPoint(namePrefix, nameSuffix + "Two", registry, worldFrame);

      assertFalse(yoFrameSO3TrajectoryPoint.epsilonEquals(yoFrameSO3TrajectoryPointTwo, 1e-7));

      yoFrameSO3TrajectoryPointTwo.set(yoFrameSO3TrajectoryPoint);
      assertTrue(yoFrameSO3TrajectoryPoint.epsilonEquals(yoFrameSO3TrajectoryPointTwo, 1e-7));

      SimpleSO3TrajectoryPoint simplePoint = new SimpleSO3TrajectoryPoint();
      yoFrameSO3TrajectoryPoint.get(simplePoint);
      
      yoFrameSO3TrajectoryPoint.setToNaN();
      assertTrue(yoFrameSO3TrajectoryPoint.containsNaN());
      assertFalse(yoFrameSO3TrajectoryPoint.epsilonEquals(yoFrameSO3TrajectoryPointTwo, 1e-7));
      
      SO3TrajectoryPointInterface<?> trajectoryPointAsInterface = simplePoint;
      yoFrameSO3TrajectoryPoint.set(trajectoryPointAsInterface);
      
      assertTrue(yoFrameSO3TrajectoryPoint.epsilonEquals(yoFrameSO3TrajectoryPointTwo, 1e-7));
      
      String string = yoFrameSO3TrajectoryPoint.toString();
      String expectedString = "SO3 trajectory point: (time =  9.90, SO3 trajectory point: (time =  9.90, SO3 waypoint: [orientation = ( 0.47,  0.30, -0.07,  0.83), angular velocity = ( 7.10,  2.20,  3.33)].)World)";
      
      assertEquals(expectedString, string);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSomeMoreSettersAndGetters()
   {
      String namePrefix = "point";
      String nameSuffix = "toTest";
      YoVariableRegistry registry = new YoVariableRegistry("myRegistry");

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoFrameSO3TrajectoryPoint yoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint(namePrefix, nameSuffix, registry, worldFrame);
      yoFrameSO3TrajectoryPoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      double time = 3.4;
      FrameOrientation orientation = new FrameOrientation(worldFrame, new Quat4d(0.1, 0.22, 0.34, 0.56));

      FrameVector angularVelocity = new FrameVector(worldFrame, 1.7, 8.4, 2.2);

      yoFrameSO3TrajectoryPoint.setTime(time);
      yoFrameSO3TrajectoryPoint.setOrientation(orientation);
      yoFrameSO3TrajectoryPoint.setAngularVelocity(angularVelocity);
    
      
      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose(worldFrame));

      FramePoint poseFramePosition = new FramePoint(worldFrame, new Point3d(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameOrientation poseOrientation = new FrameOrientation(worldFrame, new AxisAngle4d(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      yoFrameSO3TrajectoryPoint.registerReferenceFrame(poseFrame);
      
      yoFrameSO3TrajectoryPoint.changeFrame(poseFrame);
      
      assertFalse(orientation.epsilonEquals(yoFrameSO3TrajectoryPoint.getOrientation().getFrameOrientationCopy(), 1e-10));
      assertFalse(angularVelocity.epsilonEquals(yoFrameSO3TrajectoryPoint.getAngularVelocity().getFrameVectorCopy(), 1e-10));

      orientation.changeFrame(poseFrame);
      angularVelocity.changeFrame(poseFrame);

      assertTrue(orientation.epsilonEquals(yoFrameSO3TrajectoryPoint.getOrientation().getFrameOrientationCopy(), 1e-10));
      assertTrue(angularVelocity.epsilonEquals(yoFrameSO3TrajectoryPoint.getAngularVelocity().getFrameVectorCopy(), 1e-10));

      YoFrameSO3TrajectoryPoint yoFrameSO3TrajectoryPointTwo = new YoFrameSO3TrajectoryPoint(namePrefix, nameSuffix+"Two", registry, poseFrame);

      yoFrameSO3TrajectoryPointTwo.setTime(time);
      yoFrameSO3TrajectoryPointTwo.setOrientation(orientation);
      yoFrameSO3TrajectoryPointTwo.setAngularVelocity(angularVelocity);
      
      assertTrue(yoFrameSO3TrajectoryPointTwo.epsilonEquals(yoFrameSO3TrajectoryPointTwo, 1e-10));
   }
}
