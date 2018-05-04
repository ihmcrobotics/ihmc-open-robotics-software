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
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3TrajectoryPointInterface;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class YoFrameSE3TrajectoryPointTest
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

      YoFrameSE3TrajectoryPoint yoFrameSE3TrajectoryPoint = new YoFrameSE3TrajectoryPoint(namePrefix, nameSuffix, registry, worldFrame, poseFrame);
      SimpleSE3TrajectoryPoint simpleTrajectoryPoint = new SimpleSE3TrajectoryPoint();

      double time = 3.4;
      Point3D position = new Point3D(1.0, 2.1, 3.7);
      Quaternion orientation = new Quaternion(new Quaternion(0.1, 0.22, 0.34, 0.56));
      orientation.normalize();

      Vector3D linearVelocity = new Vector3D(-0.4, 1.2, 3.3);
      Vector3D angularVelocity = new Vector3D(1.7, 8.4, 2.2);

      simpleTrajectoryPoint.set(time, position, orientation, linearVelocity, angularVelocity);
      yoFrameSE3TrajectoryPoint.setIncludingFrame(worldFrame, simpleTrajectoryPoint);

      yoFrameSE3TrajectoryPoint.changeFrame(poseFrame);

      // Do some checks:
      RigidBodyTransform transformToPoseFrame = worldFrame.getTransformToDesiredFrame(poseFrame);
      transformToPoseFrame.transform(position);
      orientation.applyTransform(transformToPoseFrame);
      transformToPoseFrame.transform(linearVelocity);
      transformToPoseFrame.transform(angularVelocity);

      namePrefix = "point";
      nameSuffix = "toVerify";
      YoFrameSE3TrajectoryPoint expectedYoFrameSE3TrajectoryPoint = new YoFrameSE3TrajectoryPoint(namePrefix, nameSuffix, registry, poseFrame);

      expectedYoFrameSE3TrajectoryPoint.setTime(time);
      expectedYoFrameSE3TrajectoryPoint.setPosition(position);
      expectedYoFrameSE3TrajectoryPoint.setOrientation(orientation);
      expectedYoFrameSE3TrajectoryPoint.setLinearVelocity(linearVelocity);
      expectedYoFrameSE3TrajectoryPoint.setAngularVelocity(angularVelocity);

      assertEquals(3.4, yoFrameSE3TrajectoryPoint.getTime(), 1e-7);
      assertEquals(3.4, expectedYoFrameSE3TrajectoryPoint.getTime(), 1e-7);

      assertTrue(expectedYoFrameSE3TrajectoryPoint.epsilonEquals(yoFrameSE3TrajectoryPoint, 1e-10));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructor()
   {
      double epsilon = 1.0e-20;
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FramePoint3D expectedPosition = new FramePoint3D(expectedFrame);
      FrameQuaternion expectedOrientation = new FrameQuaternion(expectedFrame);
      FrameVector3D expectedLinearVelocity = new FrameVector3D(expectedFrame);
      FrameVector3D expectedAngularVelocity = new FrameVector3D(expectedFrame);

      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSE3TrajectoryPoint testedYoFrameSE3TrajectoryPoint = new YoFrameSE3TrajectoryPoint(expectedNamePrefix, expectedNameSuffix,
                                                                                                new YoVariableRegistry("schnoop"), expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedOrientation,
                                         expectedLinearVelocity, expectedAngularVelocity, testedYoFrameSE3TrajectoryPoint, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetters()
   {
      double epsilon = 1.0e-14;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FramePoint3D expectedPosition = new FramePoint3D(expectedFrame);
      FrameQuaternion expectedOrientation = new FrameQuaternion(expectedFrame);
      FrameVector3D expectedLinearVelocity = new FrameVector3D(expectedFrame);
      FrameVector3D expectedAngularVelocity = new FrameVector3D(expectedFrame);

      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSE3TrajectoryPoint testedYoFrameSE3TrajectoryPoint = new YoFrameSE3TrajectoryPoint(expectedNamePrefix, expectedNameSuffix,
                                                                                                new YoVariableRegistry("schnoop"), expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedOrientation,
                                         expectedLinearVelocity, expectedAngularVelocity, testedYoFrameSE3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      testedYoFrameSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedOrientation,
                                         expectedLinearVelocity, expectedAngularVelocity, testedYoFrameSE3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      testedYoFrameSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
                                          expectedAngularVelocity);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedOrientation,
                                         expectedLinearVelocity, expectedAngularVelocity, testedYoFrameSE3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      YoFrameSE3TrajectoryPoint expectedYoFrameSE3TrajectoryPoint = new YoFrameSE3TrajectoryPoint("sdfsd", "asd", new YoVariableRegistry("asawe"),
                                                                                                  expectedFrame);

      testedYoFrameSE3TrajectoryPoint.set(expectedYoFrameSE3TrajectoryPoint);

      assertTrue(expectedYoFrameSE3TrajectoryPoint.epsilonEquals(testedYoFrameSE3TrajectoryPoint, epsilon));
      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, testedYoFrameSE3TrajectoryPoint.getReferenceFrame(),
                                         testedYoFrameSE3TrajectoryPoint.getTime(), testedYoFrameSE3TrajectoryPoint.getPosition(),
                                         testedYoFrameSE3TrajectoryPoint.getOrientation(), testedYoFrameSE3TrajectoryPoint.getLinearVelocity(),
                                         testedYoFrameSE3TrajectoryPoint.getAngularVelocity(), testedYoFrameSE3TrajectoryPoint, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testChangeFrame() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      FramePoint3D expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameQuaternion expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      FrameVector3D expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      FrameVector3D expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSE3TrajectoryPoint testedYoFrameSE3TrajectoryPoint = new YoFrameSE3TrajectoryPoint(expectedNamePrefix, expectedNameSuffix,
                                                                                                new YoVariableRegistry("schnoop"), expectedFrame);

      testedYoFrameSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      ReferenceFrame[] randomFrames = new ReferenceFrame[10];
      randomFrames[0] = worldFrame;
      for (int i = 1; i < 10; i++)
         randomFrames[i] = EuclidFrameRandomTools.nextReferenceFrame("randomFrame" + i, random,
                                                                     random.nextBoolean() ? worldFrame : randomFrames[random.nextInt(i)]);

      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = randomFrames[random.nextInt(10)];
         testedYoFrameSE3TrajectoryPoint.registerReferenceFrame(expectedFrame);

         expectedPosition.changeFrame(expectedFrame);
         expectedOrientation.changeFrame(expectedFrame);
         expectedLinearVelocity.changeFrame(expectedFrame);
         expectedAngularVelocity.changeFrame(expectedFrame);
         testedYoFrameSE3TrajectoryPoint.changeFrame(expectedFrame);

         assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedOrientation,
                                            expectedLinearVelocity, expectedAngularVelocity, testedYoFrameSE3TrajectoryPoint, epsilon);
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
      FramePoint3D expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameQuaternion expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      FrameVector3D expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      FrameVector3D expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSE3TrajectoryPoint testedYoFrameSE3TrajectoryPoint = new YoFrameSE3TrajectoryPoint(expectedNamePrefix, expectedNameSuffix,
                                                                                                new YoVariableRegistry("schnoop"), expectedFrame);
      testedYoFrameSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero();
      expectedOrientation.setToZero();
      expectedLinearVelocity.setToZero();
      expectedAngularVelocity.setToZero();
      testedYoFrameSE3TrajectoryPoint.setToZero();

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedOrientation,
                                         expectedLinearVelocity, expectedAngularVelocity, testedYoFrameSE3TrajectoryPoint, epsilon);

      expectedFrame = EuclidFrameRandomTools.nextReferenceFrame("blop", random, worldFrame);
      testedYoFrameSE3TrajectoryPoint.registerReferenceFrame(expectedFrame);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 10.0, 10.0, 10.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
      testedYoFrameSE3TrajectoryPoint.switchCurrentReferenceFrame(worldFrame);
      testedYoFrameSE3TrajectoryPoint.registerReferenceFrame(worldFrame);
      testedYoFrameSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero(expectedFrame);
      expectedOrientation.setToZero(expectedFrame);
      expectedLinearVelocity.setToZero(expectedFrame);
      expectedAngularVelocity.setToZero(expectedFrame);
      testedYoFrameSE3TrajectoryPoint.switchCurrentReferenceFrame(expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedOrientation,
                                         expectedLinearVelocity, expectedAngularVelocity, testedYoFrameSE3TrajectoryPoint, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      FramePoint3D expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameQuaternion expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      FrameVector3D expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      FrameVector3D expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSE3TrajectoryPoint testedYoFrameSE3TrajectoryPoint = new YoFrameSE3TrajectoryPoint(expectedNamePrefix, expectedNameSuffix,
                                                                                                new YoVariableRegistry("schnoop"), expectedFrame);
      testedYoFrameSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      testedYoFrameSE3TrajectoryPoint.setToNaN();
      assertTrue(Double.isNaN(testedYoFrameSE3TrajectoryPoint.getTime()));
      assertTrue(testedYoFrameSE3TrajectoryPoint.getPosition().containsNaN());
      assertTrue(testedYoFrameSE3TrajectoryPoint.getOrientation().containsNaN());
      assertTrue(testedYoFrameSE3TrajectoryPoint.getLinearVelocity().containsNaN());
      assertTrue(testedYoFrameSE3TrajectoryPoint.getAngularVelocity().containsNaN());

      expectedFrame = EuclidFrameRandomTools.nextReferenceFrame("blop", random, worldFrame);
      testedYoFrameSE3TrajectoryPoint.registerReferenceFrame(expectedFrame);
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 10.0, 10.0, 10.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
      testedYoFrameSE3TrajectoryPoint.switchCurrentReferenceFrame(worldFrame);
      testedYoFrameSE3TrajectoryPoint.registerReferenceFrame(worldFrame);
      testedYoFrameSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      testedYoFrameSE3TrajectoryPoint.setToNaN(expectedFrame);

      assertTrue(expectedFrame == testedYoFrameSE3TrajectoryPoint.getReferenceFrame());
      assertTrue(Double.isNaN(testedYoFrameSE3TrajectoryPoint.getTime()));
      assertTrue(testedYoFrameSE3TrajectoryPoint.getPosition().containsNaN());
      assertTrue(testedYoFrameSE3TrajectoryPoint.getOrientation().containsNaN());
      assertTrue(testedYoFrameSE3TrajectoryPoint.getLinearVelocity().containsNaN());
      assertTrue(testedYoFrameSE3TrajectoryPoint.getAngularVelocity().containsNaN());
   }

   private void assertWaypointContainsExpectedData(String expectedNamePrefix, String expectedNameSuffix, ReferenceFrame expectedFrame, double expectedTime,
                                                   FramePoint3DReadOnly expectedPosition, FrameQuaternionReadOnly expectedOrientation,
                                                   FrameVector3DReadOnly expectedLinearVelocity, FrameVector3DReadOnly expectedAngularVelocity,
                                                   YoFrameSE3TrajectoryPoint testedYoFrameSE3TrajectoryPoint, double epsilon)
   {
      assertTrue(expectedFrame == testedYoFrameSE3TrajectoryPoint.getReferenceFrame());
      assertEquals(expectedTime, testedYoFrameSE3TrajectoryPoint.getTime(), epsilon);
      assertEquals(expectedNamePrefix, testedYoFrameSE3TrajectoryPoint.getNamePrefix());
      assertEquals(expectedNameSuffix, testedYoFrameSE3TrajectoryPoint.getNameSuffix());
      assertTrue(expectedPosition.epsilonEquals(testedYoFrameSE3TrajectoryPoint.getPosition(), epsilon));
      Quaternion trajectoryPointQuaternion = new Quaternion(testedYoFrameSE3TrajectoryPoint.getOrientation());
      assertEquals(expectedOrientation.getReferenceFrame(), testedYoFrameSE3TrajectoryPoint.getOrientation().getReferenceFrame());
      EuclidCoreTestTools.assertQuaternionEquals(expectedOrientation, trajectoryPointQuaternion, epsilon);
      assertTrue(expectedLinearVelocity.epsilonEquals(testedYoFrameSE3TrajectoryPoint.getLinearVelocity(), epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(testedYoFrameSE3TrajectoryPoint.getAngularVelocity(), epsilon));

      FrameSE3TrajectoryPoint actualFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint();
      testedYoFrameSE3TrajectoryPoint.getIncludingFrame(actualFrameSE3TrajectoryPoint);
      FrameSE3TrajectoryPointTest.assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation,
                                                                            expectedLinearVelocity, expectedAngularVelocity, actualFrameSE3TrajectoryPoint,
                                                                            epsilon);
      actualFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedFrame);
      testedYoFrameSE3TrajectoryPoint.get(actualFrameSE3TrajectoryPoint);
      FrameSE3TrajectoryPointTest.assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation,
                                                                            expectedLinearVelocity, expectedAngularVelocity, actualFrameSE3TrajectoryPoint,
                                                                            epsilon);

      Point3D actualPosition = new Point3D();
      Quaternion actualOrientation = new Quaternion();
      Vector3D actualLinearVelocity = new Vector3D();
      Vector3D actualAngularVelocity = new Vector3D();

      testedYoFrameSE3TrajectoryPoint.getPosition(actualPosition);
      testedYoFrameSE3TrajectoryPoint.getOrientation(actualOrientation);
      testedYoFrameSE3TrajectoryPoint.getLinearVelocity(actualLinearVelocity);
      testedYoFrameSE3TrajectoryPoint.getAngularVelocity(actualAngularVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualPosition, epsilon));
      assertTrue(expectedOrientation.epsilonEquals(actualOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, epsilon));

      FramePoint3D actualFramePosition = new FramePoint3D(expectedFrame);
      FrameQuaternion actualFrameOrientation = new FrameQuaternion(expectedFrame);
      FrameVector3D actualFrameLinearVelocity = new FrameVector3D(expectedFrame);
      FrameVector3D actualFrameAngularVelocity = new FrameVector3D(expectedFrame);

      testedYoFrameSE3TrajectoryPoint.getPosition(actualFramePosition);
      testedYoFrameSE3TrajectoryPoint.getOrientation(actualFrameOrientation);
      testedYoFrameSE3TrajectoryPoint.getLinearVelocity(actualFrameLinearVelocity);
      testedYoFrameSE3TrajectoryPoint.getAngularVelocity(actualFrameAngularVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));

      actualFramePosition = new FramePoint3D();
      actualFrameOrientation = new FrameQuaternion();
      actualFrameLinearVelocity = new FrameVector3D();
      actualFrameAngularVelocity = new FrameVector3D();

      testedYoFrameSE3TrajectoryPoint.getPositionIncludingFrame(actualFramePosition);
      testedYoFrameSE3TrajectoryPoint.getOrientationIncludingFrame(actualFrameOrientation);
      testedYoFrameSE3TrajectoryPoint.getLinearVelocityIncludingFrame(actualFrameLinearVelocity);
      testedYoFrameSE3TrajectoryPoint.getAngularVelocityIncludingFrame(actualFrameAngularVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
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

      YoFrameSE3TrajectoryPoint yoFrameSE3TrajectoryPoint = new YoFrameSE3TrajectoryPoint(namePrefix, nameSuffix, registry, worldFrame);
      yoFrameSE3TrajectoryPoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      SimpleSE3TrajectoryPoint simpleTrajectoryPoint = new SimpleSE3TrajectoryPoint();

      double time = 3.4;
      Point3D position = new Point3D(1.0, 2.1, 3.7);
      Quaternion orientation = new Quaternion(new Quaternion(0.1, 0.22, 0.34, 0.56));
      orientation.normalize();

      Vector3D linearVelocity = new Vector3D(-0.4, 1.2, 3.3);
      Vector3D angularVelocity = new Vector3D(1.7, 8.4, 2.2);

      simpleTrajectoryPoint.set(time, position, orientation, linearVelocity, angularVelocity);
      yoFrameSE3TrajectoryPoint.setIncludingFrame(worldFrame, simpleTrajectoryPoint);

      // Check some get calls: 
      YoFramePoint3D pointForVerification = new YoFramePoint3D("pointForVerification", worldFrame, registry);
      YoFrameQuaternion quaternionForVerification = new YoFrameQuaternion("quaternionForVerification", worldFrame, registry);
      YoFrameVector3D linearVelocityForVerification = new YoFrameVector3D("linearVelocityForVerification", worldFrame, registry);
      YoFrameVector3D angularVelocityForVerification = new YoFrameVector3D("angularVelocityForVerification", worldFrame, registry);

      yoFrameSE3TrajectoryPoint.getPosition(pointForVerification);
      yoFrameSE3TrajectoryPoint.getOrientation(quaternionForVerification);
      yoFrameSE3TrajectoryPoint.getLinearVelocity(linearVelocityForVerification);
      yoFrameSE3TrajectoryPoint.getAngularVelocity(angularVelocityForVerification);

      assertEquals(time, yoFrameSE3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(pointForVerification.epsilonEquals(position, 1e-10));
      assertTrue(quaternionForVerification.epsilonEquals(orientation, 1e-10));
      assertTrue(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-10));
      assertTrue(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-10));

      // Check NaN calls:
      assertFalse(yoFrameSE3TrajectoryPoint.containsNaN());
      yoFrameSE3TrajectoryPoint.setPositionToNaN();
      assertTrue(yoFrameSE3TrajectoryPoint.containsNaN());
      yoFrameSE3TrajectoryPoint.setPositionToZero();

      assertFalse(yoFrameSE3TrajectoryPoint.containsNaN());
      yoFrameSE3TrajectoryPoint.setOrientationToNaN();
      assertTrue(yoFrameSE3TrajectoryPoint.containsNaN());
      yoFrameSE3TrajectoryPoint.setOrientationToZero();

      assertFalse(yoFrameSE3TrajectoryPoint.containsNaN());
      yoFrameSE3TrajectoryPoint.setLinearVelocityToNaN();
      assertTrue(yoFrameSE3TrajectoryPoint.containsNaN());
      yoFrameSE3TrajectoryPoint.setLinearVelocityToZero();

      assertFalse(yoFrameSE3TrajectoryPoint.containsNaN());
      yoFrameSE3TrajectoryPoint.setAngularVelocityToNaN();
      assertTrue(yoFrameSE3TrajectoryPoint.containsNaN());
      yoFrameSE3TrajectoryPoint.setAngularVelocityToZero();
      assertFalse(yoFrameSE3TrajectoryPoint.containsNaN());

      yoFrameSE3TrajectoryPoint.getPosition(position);
      yoFrameSE3TrajectoryPoint.getOrientation(orientation);
      yoFrameSE3TrajectoryPoint.getLinearVelocity(linearVelocity);
      yoFrameSE3TrajectoryPoint.getAngularVelocity(angularVelocity);

      // Make sure they are all equal to zero:
      assertTrue(position.epsilonEquals(new Point3D(), 1e-10));
      assertTrue(orientation.epsilonEquals(new Quaternion(), 1e-10));
      assertTrue(linearVelocity.epsilonEquals(new Vector3D(), 1e-10));
      assertTrue(angularVelocity.epsilonEquals(new Vector3D(), 1e-10));

      time = 9.9;
      pointForVerification.set(3.9, 2.2, 1.1);
      quaternionForVerification.setYawPitchRoll(0.2, 0.6, 1.1);
      linearVelocityForVerification.set(8.8, 1.4, 9.22);
      angularVelocityForVerification.set(7.1, 2.2, 3.33);

      assertFalse(Math.abs(yoFrameSE3TrajectoryPoint.getTime() - time) < 1e-7);
      assertFalse(pointForVerification.epsilonEquals(position, 1e-7));
      assertFalse(quaternionForVerification.epsilonEquals(orientation, 1e-7));
      assertFalse(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-7));
      assertFalse(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-7));

      yoFrameSE3TrajectoryPoint.set(time, pointForVerification, quaternionForVerification, linearVelocityForVerification, angularVelocityForVerification);

      yoFrameSE3TrajectoryPoint.getPosition(position);
      yoFrameSE3TrajectoryPoint.getOrientation(orientation);
      yoFrameSE3TrajectoryPoint.getLinearVelocity(linearVelocity);
      yoFrameSE3TrajectoryPoint.getAngularVelocity(angularVelocity);

      assertEquals(time, yoFrameSE3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(pointForVerification.epsilonEquals(position, 1e-10));
      assertTrue(quaternionForVerification.epsilonEquals(orientation, 1e-10));
      assertTrue(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-10));
      assertTrue(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-10));

      YoFrameSE3TrajectoryPoint yoFrameSE3TrajectoryPointTwo = new YoFrameSE3TrajectoryPoint(namePrefix, nameSuffix + "Two", registry, worldFrame);

      double positionDistance = yoFrameSE3TrajectoryPoint.positionDistance(yoFrameSE3TrajectoryPointTwo);
      assertEquals(4.610856753359402, positionDistance, 1e-7);
      assertFalse(yoFrameSE3TrajectoryPoint.epsilonEquals(yoFrameSE3TrajectoryPointTwo, 1e-7));

      yoFrameSE3TrajectoryPointTwo.set(yoFrameSE3TrajectoryPoint);
      positionDistance = yoFrameSE3TrajectoryPoint.positionDistance(yoFrameSE3TrajectoryPointTwo);
      assertEquals(0.0, positionDistance, 1e-7);
      assertTrue(yoFrameSE3TrajectoryPoint.epsilonEquals(yoFrameSE3TrajectoryPointTwo, 1e-7));

      SimpleSE3TrajectoryPoint simplePoint = new SimpleSE3TrajectoryPoint();
      yoFrameSE3TrajectoryPoint.get(simplePoint);

      yoFrameSE3TrajectoryPoint.setToNaN();
      assertTrue(yoFrameSE3TrajectoryPoint.containsNaN());
      positionDistance = yoFrameSE3TrajectoryPoint.positionDistance(yoFrameSE3TrajectoryPointTwo);
      assertTrue(Double.isNaN(positionDistance));
      assertFalse(yoFrameSE3TrajectoryPoint.epsilonEquals(yoFrameSE3TrajectoryPointTwo, 1e-7));

      SE3TrajectoryPointInterface<?> trajectoryPointAsInterface = simplePoint;
      yoFrameSE3TrajectoryPoint.set(trajectoryPointAsInterface);

      positionDistance = yoFrameSE3TrajectoryPoint.positionDistance(yoFrameSE3TrajectoryPointTwo);
      assertEquals(0.0, positionDistance, 1e-7);
      assertTrue(yoFrameSE3TrajectoryPoint.epsilonEquals(yoFrameSE3TrajectoryPointTwo, 1e-7));

      String string = yoFrameSE3TrajectoryPoint.toString();
      String expectedString = "SE3 trajectory point: (time =  9.90, SE3 waypoint: [position = ( 3.900,  2.200,  1.100), orientation = ( 0.472,  0.301, -0.072,  0.826), linearVelocity = ( 8.800,  1.400,  9.220), angular velocity = ( 7.100,  2.200,  3.330)].)-World";

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

      YoFrameSE3TrajectoryPoint yoFrameSE3TrajectoryPoint = new YoFrameSE3TrajectoryPoint(namePrefix, nameSuffix, registry, worldFrame);
      yoFrameSE3TrajectoryPoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      double time = 3.4;
      FramePoint3D position = new FramePoint3D(worldFrame, 1.0, 2.1, 3.7);
      FrameQuaternion orientation = new FrameQuaternion(worldFrame, new Quaternion(0.1, 0.22, 0.34, 0.56));

      FrameVector3D linearVelocity = new FrameVector3D(worldFrame, -0.4, 1.2, 3.3);
      FrameVector3D angularVelocity = new FrameVector3D(worldFrame, 1.7, 8.4, 2.2);

      yoFrameSE3TrajectoryPoint.setTime(time);
      yoFrameSE3TrajectoryPoint.setPosition(position);
      yoFrameSE3TrajectoryPoint.setOrientation(orientation);
      yoFrameSE3TrajectoryPoint.setLinearVelocity(linearVelocity);
      yoFrameSE3TrajectoryPoint.setAngularVelocity(angularVelocity);

      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose3D(worldFrame));

      FramePoint3D poseFramePosition = new FramePoint3D(worldFrame, new Point3D(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameQuaternion poseOrientation = new FrameQuaternion(worldFrame, new AxisAngle(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      yoFrameSE3TrajectoryPoint.registerReferenceFrame(poseFrame);

      yoFrameSE3TrajectoryPoint.changeFrame(poseFrame);

      assertFalse(position.epsilonEquals(yoFrameSE3TrajectoryPoint.getPosition(), 1e-10));
      assertFalse(orientation.epsilonEquals(yoFrameSE3TrajectoryPoint.getOrientation(), 1e-10));
      assertFalse(linearVelocity.epsilonEquals(yoFrameSE3TrajectoryPoint.getLinearVelocity(), 1e-10));
      assertFalse(angularVelocity.epsilonEquals(yoFrameSE3TrajectoryPoint.getAngularVelocity(), 1e-10));

      position.changeFrame(poseFrame);
      orientation.changeFrame(poseFrame);
      linearVelocity.changeFrame(poseFrame);
      angularVelocity.changeFrame(poseFrame);

      assertTrue(position.epsilonEquals(yoFrameSE3TrajectoryPoint.getPosition(), 1e-10));
      assertTrue(orientation.epsilonEquals(yoFrameSE3TrajectoryPoint.getOrientation(), 1e-10));
      assertTrue(linearVelocity.epsilonEquals(yoFrameSE3TrajectoryPoint.getLinearVelocity(), 1e-10));
      assertTrue(angularVelocity.epsilonEquals(yoFrameSE3TrajectoryPoint.getAngularVelocity(), 1e-10));

      YoFrameSE3TrajectoryPoint yoFrameSE3TrajectoryPointTwo = new YoFrameSE3TrajectoryPoint(namePrefix, nameSuffix + "Two", registry, poseFrame);

      yoFrameSE3TrajectoryPointTwo.setTime(time);
      yoFrameSE3TrajectoryPointTwo.setPosition(position);
      yoFrameSE3TrajectoryPointTwo.setOrientation(orientation);
      yoFrameSE3TrajectoryPointTwo.setLinearVelocity(linearVelocity);
      yoFrameSE3TrajectoryPointTwo.setAngularVelocity(angularVelocity);

      assertTrue(yoFrameSE3TrajectoryPointTwo.epsilonEquals(yoFrameSE3TrajectoryPointTwo, 1e-10));

   }

}
