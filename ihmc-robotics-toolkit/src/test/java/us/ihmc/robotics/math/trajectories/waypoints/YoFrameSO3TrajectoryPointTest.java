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
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3TrajectoryPointInterface;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

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
      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose3D(worldFrame));

      FramePoint3D poseFramePosition = new FramePoint3D(worldFrame, new Point3D(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameQuaternion poseOrientation = new FrameQuaternion(worldFrame, new AxisAngle(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      YoFrameSO3TrajectoryPoint yoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint(namePrefix, nameSuffix, registry, worldFrame, poseFrame);
      SimpleSO3TrajectoryPoint simpleTrajectoryPoint = new SimpleSO3TrajectoryPoint();

      double time = 3.4;
      Quaternion orientation = new Quaternion(new Quaternion(0.1, 0.22, 0.34, 0.56));
      orientation.normalize();

      Vector3D angularVelocity = new Vector3D(1.7, 8.4, 2.2);

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
      FrameQuaternion expectedOrientation = new FrameQuaternion(expectedFrame);
      FrameVector3D expectedAngularVelocity = new FrameVector3D(expectedFrame);

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
      FrameQuaternion expectedOrientation = new FrameQuaternion(expectedFrame);
      FrameVector3D expectedAngularVelocity = new FrameVector3D(expectedFrame);

      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSO3TrajectoryPoint testedYoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint(expectedNamePrefix, expectedNameSuffix,
                                                                                                new YoVariableRegistry("schnoop"), expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
                                         testedYoFrameSO3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      testedYoFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
                                         testedYoFrameSO3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      testedYoFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
                                         testedYoFrameSO3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      YoFrameSO3TrajectoryPoint expectedYoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint("sdfsd", "asd", new YoVariableRegistry("asawe"),
                                                                                                  expectedFrame);

      testedYoFrameSO3TrajectoryPoint.set(expectedYoFrameSO3TrajectoryPoint);

      assertTrue(expectedYoFrameSO3TrajectoryPoint.epsilonEquals(testedYoFrameSO3TrajectoryPoint, epsilon));
      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, testedYoFrameSO3TrajectoryPoint.getReferenceFrame(),
                                         testedYoFrameSO3TrajectoryPoint.getTime(), testedYoFrameSO3TrajectoryPoint.getOrientation(),
                                         testedYoFrameSO3TrajectoryPoint.getAngularVelocity(), testedYoFrameSO3TrajectoryPoint, epsilon);
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
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSO3TrajectoryPoint testedYoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint(expectedNamePrefix, expectedNameSuffix,
                                                                                                new YoVariableRegistry("schnoop"), expectedFrame);

      testedYoFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      ReferenceFrame[] randomFrames = new ReferenceFrame[10];
      randomFrames[0] = worldFrame;
      for (int i = 1; i < 10; i++)
         randomFrames[i] = EuclidFrameRandomTools.nextReferenceFrame("randomFrame" + i, random,
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
      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      FrameQuaternion expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      FrameVector3D expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
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

      expectedFrame = EuclidFrameRandomTools.nextReferenceFrame("blop", random, worldFrame);
      testedYoFrameSO3TrajectoryPoint.registerReferenceFrame(expectedFrame);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
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
      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      FrameQuaternion expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      FrameVector3D expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSO3TrajectoryPoint testedYoFrameSO3TrajectoryPoint = new YoFrameSO3TrajectoryPoint(expectedNamePrefix, expectedNameSuffix,
                                                                                                new YoVariableRegistry("schnoop"), expectedFrame);
      testedYoFrameSO3TrajectoryPoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedYoFrameSO3TrajectoryPoint.setToNaN();
      assertTrue(Double.isNaN(testedYoFrameSO3TrajectoryPoint.getTime()));
      assertTrue(testedYoFrameSO3TrajectoryPoint.getOrientation().containsNaN());
      assertTrue(testedYoFrameSO3TrajectoryPoint.getAngularVelocity().containsNaN());

      expectedFrame = EuclidFrameRandomTools.nextReferenceFrame("blop", random, worldFrame);
      testedYoFrameSO3TrajectoryPoint.registerReferenceFrame(expectedFrame);
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
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
                                                   FrameQuaternionReadOnly expectedOrientation, FrameVector3DReadOnly expectedAngularVelocity,
                                                   YoFrameSO3TrajectoryPoint testedYoFrameSO3TrajectoryPoint, double epsilon)
   {
      assertTrue(expectedFrame == testedYoFrameSO3TrajectoryPoint.getReferenceFrame());
      assertEquals(expectedTime, testedYoFrameSO3TrajectoryPoint.getTime(), epsilon);
      assertEquals(expectedNamePrefix, testedYoFrameSO3TrajectoryPoint.getNamePrefix());
      assertEquals(expectedNameSuffix, testedYoFrameSO3TrajectoryPoint.getNameSuffix());
      assertTrue(expectedOrientation.epsilonEquals(testedYoFrameSO3TrajectoryPoint.getOrientation(), epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(testedYoFrameSO3TrajectoryPoint.getAngularVelocity(), epsilon));

      FrameSO3TrajectoryPoint actualFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint();
      testedYoFrameSO3TrajectoryPoint.getIncludingFrame(actualFrameSO3TrajectoryPoint);
      FrameSO3TrajectoryPointTest.assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
                                                                            actualFrameSO3TrajectoryPoint, epsilon);
      actualFrameSO3TrajectoryPoint = new FrameSO3TrajectoryPoint(expectedFrame);
      testedYoFrameSO3TrajectoryPoint.get(actualFrameSO3TrajectoryPoint);
      FrameSO3TrajectoryPointTest.assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity,
                                                                            actualFrameSO3TrajectoryPoint, epsilon);

      Quaternion actualOrientation = new Quaternion();
      Vector3D actualAngularVelocity = new Vector3D();

      testedYoFrameSO3TrajectoryPoint.getOrientation(actualOrientation);
      testedYoFrameSO3TrajectoryPoint.getAngularVelocity(actualAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, epsilon));

      FrameQuaternion actualFrameOrientation = new FrameQuaternion(expectedFrame);
      FrameVector3D actualFrameAngularVelocity = new FrameVector3D(expectedFrame);

      testedYoFrameSO3TrajectoryPoint.getOrientation(actualFrameOrientation);
      testedYoFrameSO3TrajectoryPoint.getAngularVelocity(actualFrameAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));

      actualFrameOrientation = new FrameQuaternion();
      actualFrameAngularVelocity = new FrameVector3D();

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
      Quaternion orientation = new Quaternion(new Quaternion(0.1, 0.22, 0.34, 0.56));
      orientation.normalize();

      Vector3D angularVelocity = new Vector3D(1.7, 8.4, 2.2);

      simpleTrajectoryPoint.set(time, orientation, angularVelocity);
      yoFrameSO3TrajectoryPoint.setIncludingFrame(worldFrame, simpleTrajectoryPoint);

      // Check some get calls: 
      YoFrameQuaternion quaternionForVerification = new YoFrameQuaternion("quaternionForVerification", worldFrame, registry);
      YoFrameVector3D angularVelocityForVerification = new YoFrameVector3D("angularVelocityForVerification", worldFrame, registry);

      yoFrameSO3TrajectoryPoint.getOrientation(quaternionForVerification);
      yoFrameSO3TrajectoryPoint.getAngularVelocity(angularVelocityForVerification);

      assertEquals(time, yoFrameSO3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(quaternionForVerification.epsilonEquals(orientation, 1e-10));
      assertTrue(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-10));

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
      assertTrue(orientation.epsilonEquals(new Quaternion(), 1e-10));
      assertTrue(angularVelocity.epsilonEquals(new Vector3D(), 1e-10));

      time = 9.9;
      quaternionForVerification.setYawPitchRoll(0.2, 0.6, 1.1);
      angularVelocityForVerification.set(7.1, 2.2, 3.33);

      assertFalse(Math.abs(yoFrameSO3TrajectoryPoint.getTime() - time) < 1e-7);
      assertFalse(quaternionForVerification.epsilonEquals(orientation, 1e-7));
      assertFalse(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-7));

      yoFrameSO3TrajectoryPoint.set(time, quaternionForVerification, angularVelocityForVerification);

      yoFrameSO3TrajectoryPoint.getOrientation(orientation);
      yoFrameSO3TrajectoryPoint.getAngularVelocity(angularVelocity);

      assertEquals(time, yoFrameSO3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(quaternionForVerification.epsilonEquals(orientation, 1e-10));
      assertTrue(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-10));

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
      String expectedString = "SO3 trajectory point: (time =  9.90, SO3 waypoint: [orientation = ( 0.472,  0.301, -0.072,  0.826), angular velocity = ( 7.100,  2.200,  3.330)].)-World";

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
      FrameQuaternion orientation = new FrameQuaternion(worldFrame, new Quaternion(0.1, 0.22, 0.34, 0.56));

      FrameVector3D angularVelocity = new FrameVector3D(worldFrame, 1.7, 8.4, 2.2);

      yoFrameSO3TrajectoryPoint.setTime(time);
      yoFrameSO3TrajectoryPoint.setOrientation(orientation);
      yoFrameSO3TrajectoryPoint.setAngularVelocity(angularVelocity);

      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose3D(worldFrame));

      FramePoint3D poseFramePosition = new FramePoint3D(worldFrame, new Point3D(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameQuaternion poseOrientation = new FrameQuaternion(worldFrame, new AxisAngle(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      yoFrameSO3TrajectoryPoint.registerReferenceFrame(poseFrame);

      yoFrameSO3TrajectoryPoint.changeFrame(poseFrame);

      assertFalse(orientation.epsilonEquals(yoFrameSO3TrajectoryPoint.getOrientation(), 1e-10));
      assertFalse(angularVelocity.epsilonEquals(yoFrameSO3TrajectoryPoint.getAngularVelocity(), 1e-10));

      orientation.changeFrame(poseFrame);
      angularVelocity.changeFrame(poseFrame);

      assertTrue(orientation.epsilonEquals(yoFrameSO3TrajectoryPoint.getOrientation(), 1e-10));
      assertTrue(angularVelocity.epsilonEquals(yoFrameSO3TrajectoryPoint.getAngularVelocity(), 1e-10));

      YoFrameSO3TrajectoryPoint yoFrameSO3TrajectoryPointTwo = new YoFrameSO3TrajectoryPoint(namePrefix, nameSuffix + "Two", registry, poseFrame);

      yoFrameSO3TrajectoryPointTwo.setTime(time);
      yoFrameSO3TrajectoryPointTwo.setOrientation(orientation);
      yoFrameSO3TrajectoryPointTwo.setAngularVelocity(angularVelocity);

      assertTrue(yoFrameSO3TrajectoryPointTwo.epsilonEquals(yoFrameSO3TrajectoryPointTwo, 1e-10));
   }
}
