package us.ihmc.robotics.math.trajectories.waypoints;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.YoFrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SE3TrajectoryPointBasics;
import us.ihmc.euclid.referenceFrame.PoseReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoFrameSE3TrajectoryPointTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testCommonUsageExample()
   {
      String namePrefix = "point";
      String nameSuffix = "toTest";
      YoRegistry registry = new YoRegistry("myRegistry");

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose3D(worldFrame));

      FramePoint3D poseFramePosition = new FramePoint3D(worldFrame, new Point3D(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameQuaternion poseOrientation = new FrameQuaternion(worldFrame, new AxisAngle(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      YoFrameSE3TrajectoryPoint yoFrameSE3TrajectoryPoint = new YoFrameSE3TrajectoryPoint(namePrefix, nameSuffix, registry);
      SE3TrajectoryPoint simpleTrajectoryPoint = new SE3TrajectoryPoint();

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
      expectedYoFrameSE3TrajectoryPoint.getPosition().set(position);
      expectedYoFrameSE3TrajectoryPoint.getOrientation().set((Orientation3DReadOnly) orientation);
      expectedYoFrameSE3TrajectoryPoint.getLinearVelocity().set(linearVelocity);
      expectedYoFrameSE3TrajectoryPoint.getAngularVelocity().set(angularVelocity);

      assertEquals(3.4, yoFrameSE3TrajectoryPoint.getTime(), 1e-7);
      assertEquals(3.4, expectedYoFrameSE3TrajectoryPoint.getTime(), 1e-7);

      assertTrue(expectedYoFrameSE3TrajectoryPoint.epsilonEquals(yoFrameSE3TrajectoryPoint, 1e-10));
   }

   @Test
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
                                                                                                new YoRegistry("schnoop"), expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedOrientation,
                                         expectedLinearVelocity, expectedAngularVelocity, testedYoFrameSE3TrajectoryPoint, epsilon);
   }

   @Test
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
                                                                                                new YoRegistry("schnoop"), expectedFrame);

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

      YoFrameSE3TrajectoryPoint expectedYoFrameSE3TrajectoryPoint = new YoFrameSE3TrajectoryPoint("sdfsd", "asd", new YoRegistry("asawe"),
                                                                                                  expectedFrame);

      testedYoFrameSE3TrajectoryPoint.set(expectedYoFrameSE3TrajectoryPoint);

      assertTrue(expectedYoFrameSE3TrajectoryPoint.epsilonEquals(testedYoFrameSE3TrajectoryPoint, epsilon));
      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, testedYoFrameSE3TrajectoryPoint.getReferenceFrame(),
                                         testedYoFrameSE3TrajectoryPoint.getTime(), testedYoFrameSE3TrajectoryPoint.getPosition(),
                                         testedYoFrameSE3TrajectoryPoint.getOrientation(), testedYoFrameSE3TrajectoryPoint.getLinearVelocity(),
                                         testedYoFrameSE3TrajectoryPoint.getAngularVelocity(), testedYoFrameSE3TrajectoryPoint, epsilon);
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
      FrameQuaternion expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      FrameVector3D expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      FrameVector3D expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSE3TrajectoryPoint testedYoFrameSE3TrajectoryPoint = new YoFrameSE3TrajectoryPoint(expectedNamePrefix, expectedNameSuffix,
                                                                                                new YoRegistry("schnoop"), expectedFrame);

      testedYoFrameSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      ReferenceFrame[] randomFrames = new ReferenceFrame[10];
      randomFrames[0] = worldFrame;
      for (int i = 1; i < 10; i++)
         randomFrames[i] = EuclidFrameRandomTools.nextReferenceFrame("randomFrame" + i, random,
                                                                     random.nextBoolean() ? worldFrame : randomFrames[random.nextInt(i)]);

      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = randomFrames[random.nextInt(10)];

         expectedPosition.changeFrame(expectedFrame);
         expectedOrientation.changeFrame(expectedFrame);
         expectedLinearVelocity.changeFrame(expectedFrame);
         expectedAngularVelocity.changeFrame(expectedFrame);
         testedYoFrameSE3TrajectoryPoint.changeFrame(expectedFrame);

         assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedOrientation,
                                            expectedLinearVelocity, expectedAngularVelocity, testedYoFrameSE3TrajectoryPoint, epsilon);
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
      FrameQuaternion expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      FrameVector3D expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      FrameVector3D expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSE3TrajectoryPoint testedYoFrameSE3TrajectoryPoint = new YoFrameSE3TrajectoryPoint(expectedNamePrefix, expectedNameSuffix,
                                                                                                new YoRegistry("schnoop"), expectedFrame);
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

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 10.0, 10.0, 10.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
      testedYoFrameSE3TrajectoryPoint.setToZero(worldFrame);
      testedYoFrameSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero(expectedFrame);
      expectedOrientation.setToZero(expectedFrame);
      expectedLinearVelocity.setToZero(expectedFrame);
      expectedAngularVelocity.setToZero(expectedFrame);
      testedYoFrameSE3TrajectoryPoint.setToZero(expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedOrientation,
                                         expectedLinearVelocity, expectedAngularVelocity, testedYoFrameSE3TrajectoryPoint, epsilon);
   }

   @Test
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
                                                                                                new YoRegistry("schnoop"), expectedFrame);
      testedYoFrameSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      testedYoFrameSE3TrajectoryPoint.setToNaN();
      assertTrue(Double.isNaN(testedYoFrameSE3TrajectoryPoint.getTime()));
      assertTrue(testedYoFrameSE3TrajectoryPoint.getPosition().containsNaN());
      assertTrue(testedYoFrameSE3TrajectoryPoint.getOrientation().containsNaN());
      assertTrue(testedYoFrameSE3TrajectoryPoint.getLinearVelocity().containsNaN());
      assertTrue(testedYoFrameSE3TrajectoryPoint.getAngularVelocity().containsNaN());

      expectedFrame = EuclidFrameRandomTools.nextReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 10.0, 10.0, 10.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
      testedYoFrameSE3TrajectoryPoint.setToZero(worldFrame);
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
      EuclidCoreTestTools.assertOrientation3DGeometricallyEquals(expectedOrientation, trajectoryPointQuaternion, epsilon);
      assertTrue(expectedLinearVelocity.epsilonEquals(testedYoFrameSE3TrajectoryPoint.getLinearVelocity(), epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(testedYoFrameSE3TrajectoryPoint.getAngularVelocity(), epsilon));

      FrameSE3TrajectoryPoint actualFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint();
      actualFrameSE3TrajectoryPoint.setIncludingFrame(testedYoFrameSE3TrajectoryPoint);
      FrameSE3TrajectoryPointTest.assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation,
                                                                            expectedLinearVelocity, expectedAngularVelocity, actualFrameSE3TrajectoryPoint,
                                                                            epsilon);
      actualFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedFrame);
      actualFrameSE3TrajectoryPoint.set(testedYoFrameSE3TrajectoryPoint);
      FrameSE3TrajectoryPointTest.assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation,
                                                                            expectedLinearVelocity, expectedAngularVelocity, actualFrameSE3TrajectoryPoint,
                                                                            epsilon);

      Point3D actualPosition = new Point3D();
      Quaternion actualOrientation = new Quaternion();
      Vector3D actualLinearVelocity = new Vector3D();
      Vector3D actualAngularVelocity = new Vector3D();

      actualPosition.set(testedYoFrameSE3TrajectoryPoint.getPosition());
      actualOrientation.set(testedYoFrameSE3TrajectoryPoint.getOrientation());
      actualLinearVelocity.set(testedYoFrameSE3TrajectoryPoint.getLinearVelocity());
      actualAngularVelocity.set(testedYoFrameSE3TrajectoryPoint.getAngularVelocity());

      assertTrue(expectedPosition.epsilonEquals(actualPosition, epsilon));
      assertTrue(expectedOrientation.geometricallyEquals(actualOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, epsilon));

      FramePoint3D actualFramePosition = new FramePoint3D(expectedFrame);
      FrameQuaternion actualFrameOrientation = new FrameQuaternion(expectedFrame);
      FrameVector3D actualFrameLinearVelocity = new FrameVector3D(expectedFrame);
      FrameVector3D actualFrameAngularVelocity = new FrameVector3D(expectedFrame);

      actualFramePosition.set(testedYoFrameSE3TrajectoryPoint.getPosition());
      actualFrameOrientation.set(testedYoFrameSE3TrajectoryPoint.getOrientation());
      actualFrameLinearVelocity.set(testedYoFrameSE3TrajectoryPoint.getLinearVelocity());
      actualFrameAngularVelocity.set(testedYoFrameSE3TrajectoryPoint.getAngularVelocity());

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedOrientation.geometricallyEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));

      actualFramePosition = new FramePoint3D();
      actualFrameOrientation = new FrameQuaternion();
      actualFrameLinearVelocity = new FrameVector3D();
      actualFrameAngularVelocity = new FrameVector3D();

      actualFramePosition.setIncludingFrame(testedYoFrameSE3TrajectoryPoint.getPosition());
      actualFrameOrientation.setIncludingFrame(testedYoFrameSE3TrajectoryPoint.getOrientation());
      actualFrameLinearVelocity.setIncludingFrame(testedYoFrameSE3TrajectoryPoint.getLinearVelocity());
      actualFrameAngularVelocity.setIncludingFrame(testedYoFrameSE3TrajectoryPoint.getAngularVelocity());

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedOrientation.geometricallyEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));
   }

   @Test
   public void testSomeSetsAngGets()
   {
      String namePrefix = "point";
      String nameSuffix = "toTest";
      YoRegistry registry = new YoRegistry("myRegistry");

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoFrameSE3TrajectoryPoint yoFrameSE3TrajectoryPoint = new YoFrameSE3TrajectoryPoint(namePrefix, nameSuffix, registry, worldFrame);
      yoFrameSE3TrajectoryPoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      SE3TrajectoryPoint simpleTrajectoryPoint = new SE3TrajectoryPoint();

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

      pointForVerification.set(yoFrameSE3TrajectoryPoint.getPosition());
      quaternionForVerification.set(yoFrameSE3TrajectoryPoint.getOrientation());
      linearVelocityForVerification.set(yoFrameSE3TrajectoryPoint.getLinearVelocity());
      angularVelocityForVerification.set(yoFrameSE3TrajectoryPoint.getAngularVelocity());

      assertEquals(time, yoFrameSE3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(pointForVerification.epsilonEquals(position, 1e-10));
      assertTrue(quaternionForVerification.epsilonEquals(orientation, 1e-10));
      assertTrue(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-10));
      assertTrue(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-10));

      // Check NaN calls:
      assertFalse(yoFrameSE3TrajectoryPoint.containsNaN());
      yoFrameSE3TrajectoryPoint.getPosition().setToNaN();
      assertTrue(yoFrameSE3TrajectoryPoint.containsNaN());
      yoFrameSE3TrajectoryPoint.getPosition().setToZero();

      assertFalse(yoFrameSE3TrajectoryPoint.containsNaN());
      yoFrameSE3TrajectoryPoint.getOrientation().setToNaN();
      assertTrue(yoFrameSE3TrajectoryPoint.containsNaN());
      yoFrameSE3TrajectoryPoint.getOrientation().setToZero();

      assertFalse(yoFrameSE3TrajectoryPoint.containsNaN());
      yoFrameSE3TrajectoryPoint.getLinearVelocity().setToNaN();
      assertTrue(yoFrameSE3TrajectoryPoint.containsNaN());
      yoFrameSE3TrajectoryPoint.getLinearVelocity().setToZero();

      assertFalse(yoFrameSE3TrajectoryPoint.containsNaN());
      yoFrameSE3TrajectoryPoint.getAngularVelocity().setToNaN();
      assertTrue(yoFrameSE3TrajectoryPoint.containsNaN());
      yoFrameSE3TrajectoryPoint.getAngularVelocity().setToZero();
      assertFalse(yoFrameSE3TrajectoryPoint.containsNaN());

      position.set(yoFrameSE3TrajectoryPoint.getPosition());
      orientation.set(yoFrameSE3TrajectoryPoint.getOrientation());
      linearVelocity.set(yoFrameSE3TrajectoryPoint.getLinearVelocity());
      angularVelocity.set(yoFrameSE3TrajectoryPoint.getAngularVelocity());

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

      position.set(yoFrameSE3TrajectoryPoint.getPosition());
      orientation.set(yoFrameSE3TrajectoryPoint.getOrientation());
      linearVelocity.set(yoFrameSE3TrajectoryPoint.getLinearVelocity());
      angularVelocity.set(yoFrameSE3TrajectoryPoint.getAngularVelocity());

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

      SE3TrajectoryPoint simplePoint = new SE3TrajectoryPoint();
      simplePoint.set(yoFrameSE3TrajectoryPoint);

      yoFrameSE3TrajectoryPoint.setToNaN();
      assertTrue(yoFrameSE3TrajectoryPoint.containsNaN());
      positionDistance = yoFrameSE3TrajectoryPoint.positionDistance(yoFrameSE3TrajectoryPointTwo);
      assertTrue(Double.isNaN(positionDistance));
      assertFalse(yoFrameSE3TrajectoryPoint.epsilonEquals(yoFrameSE3TrajectoryPointTwo, 1e-7));

      SE3TrajectoryPointBasics trajectoryPointAsInterface = simplePoint;
      yoFrameSE3TrajectoryPoint.set(trajectoryPointAsInterface);

      positionDistance = yoFrameSE3TrajectoryPoint.positionDistance(yoFrameSE3TrajectoryPointTwo);
      assertEquals(0.0, positionDistance, 1e-7);
      assertTrue(yoFrameSE3TrajectoryPoint.epsilonEquals(yoFrameSE3TrajectoryPointTwo, 1e-7));

      String string = yoFrameSE3TrajectoryPoint.toString();
      String expectedString = "SE3 trajectory point: [time= 9.900, position=( 3.900,  2.200,  1.100 ), orientation=( 0.472,  0.301, -0.072,  0.826 ), linear velocity=( 8.800,  1.400,  9.220 ), angular velocity=( 7.100,  2.200,  3.330 )] - World";
      assertEquals(expectedString, string);
   }

   @Test
   public void testSomeMoreSettersAndGetters()
   {
      String namePrefix = "point";
      String nameSuffix = "toTest";
      YoRegistry registry = new YoRegistry("myRegistry");

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoFrameSE3TrajectoryPoint yoFrameSE3TrajectoryPoint = new YoFrameSE3TrajectoryPoint(namePrefix, nameSuffix, registry, worldFrame);
      yoFrameSE3TrajectoryPoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      double time = 3.4;
      FramePoint3D position = new FramePoint3D(worldFrame, 1.0, 2.1, 3.7);
      FrameQuaternion orientation = new FrameQuaternion(worldFrame, new Quaternion(0.1, 0.22, 0.34, 0.56));

      FrameVector3D linearVelocity = new FrameVector3D(worldFrame, -0.4, 1.2, 3.3);
      FrameVector3D angularVelocity = new FrameVector3D(worldFrame, 1.7, 8.4, 2.2);

      yoFrameSE3TrajectoryPoint.setTime(time);
      yoFrameSE3TrajectoryPoint.getPosition().set((FramePoint3DReadOnly) position);
      yoFrameSE3TrajectoryPoint.getOrientation().set((FrameOrientation3DReadOnly) orientation);
      yoFrameSE3TrajectoryPoint.getLinearVelocity().set((FrameVector3DReadOnly) linearVelocity);
      yoFrameSE3TrajectoryPoint.getAngularVelocity().set((FrameVector3DReadOnly) angularVelocity);

      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose3D(worldFrame));

      FramePoint3D poseFramePosition = new FramePoint3D(worldFrame, new Point3D(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameQuaternion poseOrientation = new FrameQuaternion(worldFrame, new AxisAngle(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

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
      yoFrameSE3TrajectoryPointTwo.getPosition().set((FramePoint3DReadOnly) position);
      yoFrameSE3TrajectoryPointTwo.getOrientation().set((FrameOrientation3DReadOnly) orientation);
      yoFrameSE3TrajectoryPointTwo.getLinearVelocity().set((FrameVector3DReadOnly) linearVelocity);
      yoFrameSE3TrajectoryPointTwo.getAngularVelocity().set((FrameVector3DReadOnly) angularVelocity);

      assertTrue(yoFrameSE3TrajectoryPointTwo.epsilonEquals(yoFrameSE3TrajectoryPointTwo, 1e-10));

   }

}
