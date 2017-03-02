package us.ihmc.robotics.math.trajectories.waypoints;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanTrajectoryPointInterface;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFrameEuclideanTrajectoryPointTest
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

      FramePoint poseFramePosition = new FramePoint(worldFrame, new Point3D(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameOrientation poseOrientation = new FrameOrientation(worldFrame, new AxisAngle(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      YoFrameEuclideanTrajectoryPoint yoFrameEuclideanTrajectoryPoint = new YoFrameEuclideanTrajectoryPoint(namePrefix, nameSuffix, registry, worldFrame, poseFrame);
      SimpleEuclideanTrajectoryPoint simpleTrajectoryPoint = new SimpleEuclideanTrajectoryPoint();

      double time = 3.4;
      Point3D position = new Point3D(1.0, 2.1, 3.7);

      Vector3D linearVelocity = new Vector3D(-0.4, 1.2, 3.3);

      simpleTrajectoryPoint.set(time, position, linearVelocity);
      yoFrameEuclideanTrajectoryPoint.setIncludingFrame(worldFrame, simpleTrajectoryPoint);

      yoFrameEuclideanTrajectoryPoint.changeFrame(poseFrame);

      // Do some checks:
      RigidBodyTransform transformToPoseFrame = worldFrame.getTransformToDesiredFrame(poseFrame);
      transformToPoseFrame.transform(position);
      transformToPoseFrame.transform(linearVelocity);

      namePrefix = "point";
      nameSuffix = "toVerify";
      YoFrameEuclideanTrajectoryPoint expectedYoFrameEuclideanTrajectoryPoint = new YoFrameEuclideanTrajectoryPoint(namePrefix, nameSuffix, registry, poseFrame);

      expectedYoFrameEuclideanTrajectoryPoint.setTime(time);
      expectedYoFrameEuclideanTrajectoryPoint.setPosition(position);
      expectedYoFrameEuclideanTrajectoryPoint.setLinearVelocity(linearVelocity);

      assertEquals(3.4, yoFrameEuclideanTrajectoryPoint.getTime(), 1e-7);
      assertEquals(3.4, expectedYoFrameEuclideanTrajectoryPoint.getTime(), 1e-7);

      assertTrue(expectedYoFrameEuclideanTrajectoryPoint.epsilonEquals(yoFrameEuclideanTrajectoryPoint, 1e-10));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructor()
   {
      double epsilon = 1.0e-20;
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FramePoint expectedPosition = new FramePoint(expectedFrame);
      FrameVector expectedLinearVelocity = new FrameVector(expectedFrame);

      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameEuclideanTrajectoryPoint testedYoFrameEuclideanTrajectoryPoint = new YoFrameEuclideanTrajectoryPoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanTrajectoryPoint, epsilon);
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
      FramePoint expectedPosition = new FramePoint(expectedFrame);
      FrameVector expectedLinearVelocity = new FrameVector(expectedFrame);

      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameEuclideanTrajectoryPoint testedYoFrameEuclideanTrajectoryPoint = new YoFrameEuclideanTrajectoryPoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedYoFrameEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedYoFrameEuclideanTrajectoryPoint.set(expectedTime, expectedPosition.getPoint(), expectedLinearVelocity.getVector());

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      YoFrameEuclideanTrajectoryPoint expectedYoFrameEuclideanTrajectoryPoint = new YoFrameEuclideanTrajectoryPoint("sdfsd", "asd", new YoVariableRegistry("asawe"), expectedFrame);

      testedYoFrameEuclideanTrajectoryPoint.set(expectedYoFrameEuclideanTrajectoryPoint);

      assertTrue(expectedYoFrameEuclideanTrajectoryPoint.epsilonEquals(testedYoFrameEuclideanTrajectoryPoint, epsilon));
      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, testedYoFrameEuclideanTrajectoryPoint.getReferenceFrame(), testedYoFrameEuclideanTrajectoryPoint.getTime(),
            testedYoFrameEuclideanTrajectoryPoint.getPosition().getFramePointCopy(), testedYoFrameEuclideanTrajectoryPoint.getLinearVelocity().getFrameVectorCopy(), testedYoFrameEuclideanTrajectoryPoint,
            epsilon);
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
      FramePoint expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameVector expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameEuclideanTrajectoryPoint testedYoFrameEuclideanTrajectoryPoint = new YoFrameEuclideanTrajectoryPoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);
      
      testedYoFrameEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      ReferenceFrame[] randomFrames = new ReferenceFrame[10];
      randomFrames[0] = worldFrame;
      for (int i = 1; i < 10; i++)
         randomFrames[i] = ReferenceFrame.generateRandomReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : randomFrames[random.nextInt(i)]);
      
      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = randomFrames[random.nextInt(10)];
         testedYoFrameEuclideanTrajectoryPoint.registerReferenceFrame(expectedFrame);

         expectedPosition.changeFrame(expectedFrame);
         expectedLinearVelocity.changeFrame(expectedFrame);
         testedYoFrameEuclideanTrajectoryPoint.changeFrame(expectedFrame);

         assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanTrajectoryPoint, epsilon);
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
      FramePoint expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameVector expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameEuclideanTrajectoryPoint testedYoFrameEuclideanTrajectoryPoint = new YoFrameEuclideanTrajectoryPoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);
      testedYoFrameEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero();
      expectedLinearVelocity.setToZero();
      testedYoFrameEuclideanTrajectoryPoint.setToZero();
      
      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      testedYoFrameEuclideanTrajectoryPoint.registerReferenceFrame(expectedFrame);

      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedYoFrameEuclideanTrajectoryPoint.switchCurrentReferenceFrame(worldFrame);
      testedYoFrameEuclideanTrajectoryPoint.registerReferenceFrame(worldFrame);
      testedYoFrameEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero(expectedFrame);
      expectedLinearVelocity.setToZero(expectedFrame);
      testedYoFrameEuclideanTrajectoryPoint.switchCurrentReferenceFrame(expectedFrame);
      
      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedYoFrameEuclideanTrajectoryPoint, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      FramePoint expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameVector expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameEuclideanTrajectoryPoint testedYoFrameEuclideanTrajectoryPoint = new YoFrameEuclideanTrajectoryPoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);
      testedYoFrameEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      testedYoFrameEuclideanTrajectoryPoint.setToNaN();
      assertTrue(Double.isNaN(testedYoFrameEuclideanTrajectoryPoint.getTime()));
      assertTrue(testedYoFrameEuclideanTrajectoryPoint.getPosition().containsNaN());
      assertTrue(testedYoFrameEuclideanTrajectoryPoint.getLinearVelocity().containsNaN());

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      testedYoFrameEuclideanTrajectoryPoint.registerReferenceFrame(expectedFrame);
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedYoFrameEuclideanTrajectoryPoint.switchCurrentReferenceFrame(worldFrame);
      testedYoFrameEuclideanTrajectoryPoint.registerReferenceFrame(worldFrame);
      testedYoFrameEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      testedYoFrameEuclideanTrajectoryPoint.setToNaN(expectedFrame);

      assertTrue(expectedFrame == testedYoFrameEuclideanTrajectoryPoint.getReferenceFrame());
      assertTrue(Double.isNaN(testedYoFrameEuclideanTrajectoryPoint.getTime()));
      assertTrue(testedYoFrameEuclideanTrajectoryPoint.getPosition().containsNaN());
      assertTrue(testedYoFrameEuclideanTrajectoryPoint.getLinearVelocity().containsNaN());
   }

   private void assertWaypointContainsExpectedData(String expectedNamePrefix, String expectedNameSuffix, ReferenceFrame expectedFrame, double expectedTime, FramePoint expectedPosition,
         FrameVector expectedLinearVelocity, YoFrameEuclideanTrajectoryPoint testedYoFrameEuclideanTrajectoryPoint, double epsilon)
   {
      assertTrue(expectedFrame == testedYoFrameEuclideanTrajectoryPoint.getReferenceFrame());
      assertEquals(expectedTime, testedYoFrameEuclideanTrajectoryPoint.getTime(), epsilon);
      assertEquals(expectedNamePrefix, testedYoFrameEuclideanTrajectoryPoint.getNamePrefix());
      assertEquals(expectedNameSuffix, testedYoFrameEuclideanTrajectoryPoint.getNameSuffix());
      assertTrue(expectedPosition.epsilonEquals(testedYoFrameEuclideanTrajectoryPoint.getPosition().getFramePointCopy(), epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(testedYoFrameEuclideanTrajectoryPoint.getLinearVelocity().getFrameVectorCopy(), epsilon));

      FrameEuclideanTrajectoryPoint actualFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedFrame);
      testedYoFrameEuclideanTrajectoryPoint.get(actualFrameEuclideanTrajectoryPoint);
      FrameEuclideanTrajectoryPointTest.assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, actualFrameEuclideanTrajectoryPoint, epsilon);
      actualFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedFrame);
      testedYoFrameEuclideanTrajectoryPoint.get(actualFrameEuclideanTrajectoryPoint);
      FrameEuclideanTrajectoryPointTest.assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, actualFrameEuclideanTrajectoryPoint, epsilon);

      Point3D actualPosition = new Point3D();
      Vector3D actualLinearVelocity = new Vector3D();

      testedYoFrameEuclideanTrajectoryPoint.getPosition(actualPosition);
      testedYoFrameEuclideanTrajectoryPoint.getLinearVelocity(actualLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualPosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualLinearVelocity, epsilon));

      FramePoint actualFramePosition = new FramePoint(expectedFrame);
      FrameVector actualFrameLinearVelocity = new FrameVector(expectedFrame);

      testedYoFrameEuclideanTrajectoryPoint.getPosition(actualFramePosition);
      testedYoFrameEuclideanTrajectoryPoint.getLinearVelocity(actualFrameLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));

      actualFramePosition = new FramePoint();
      actualFrameLinearVelocity = new FrameVector();

      testedYoFrameEuclideanTrajectoryPoint.getPositionIncludingFrame(actualFramePosition);
      testedYoFrameEuclideanTrajectoryPoint.getLinearVelocityIncludingFrame(actualFrameLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
   }

   
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testSomeSetsAngGets()
   {
      String namePrefix = "point";
      String nameSuffix = "toTest";
      YoVariableRegistry registry = new YoVariableRegistry("myRegistry");

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoFrameEuclideanTrajectoryPoint yoFrameEuclideanTrajectoryPoint = new YoFrameEuclideanTrajectoryPoint(namePrefix, nameSuffix, registry, worldFrame);
      yoFrameEuclideanTrajectoryPoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      SimpleEuclideanTrajectoryPoint simpleTrajectoryPoint = new SimpleEuclideanTrajectoryPoint();

      double time = 3.4;
      Point3D position = new Point3D(1.0, 2.1, 3.7);
      Vector3D linearVelocity = new Vector3D(-0.4, 1.2, 3.3);

      simpleTrajectoryPoint.set(time, position, linearVelocity);
      yoFrameEuclideanTrajectoryPoint.setIncludingFrame(worldFrame, simpleTrajectoryPoint);

      // Check some get calls: 
      YoFramePoint pointForVerification = new YoFramePoint("pointForVerification", worldFrame, registry);
      YoFrameVector linearVelocityForVerification = new YoFrameVector("linearVelocityForVerification", worldFrame, registry);

      yoFrameEuclideanTrajectoryPoint.getPosition(pointForVerification);
      yoFrameEuclideanTrajectoryPoint.getLinearVelocity(linearVelocityForVerification);

      assertEquals(time, yoFrameEuclideanTrajectoryPoint.getTime(), 1e-10);
      assertTrue(pointForVerification.getFrameTuple().epsilonEquals(position, 1e-10));
      assertTrue(linearVelocityForVerification.getFrameTuple().epsilonEquals(linearVelocity, 1e-10));

      // Check NaN calls:
      assertFalse(yoFrameEuclideanTrajectoryPoint.containsNaN());
      yoFrameEuclideanTrajectoryPoint.setPositionToNaN();
      assertTrue(yoFrameEuclideanTrajectoryPoint.containsNaN());
      yoFrameEuclideanTrajectoryPoint.setPositionToZero();

      assertFalse(yoFrameEuclideanTrajectoryPoint.containsNaN());
      yoFrameEuclideanTrajectoryPoint.setLinearVelocityToNaN();
      assertTrue(yoFrameEuclideanTrajectoryPoint.containsNaN());
      yoFrameEuclideanTrajectoryPoint.setLinearVelocityToZero();

      yoFrameEuclideanTrajectoryPoint.getPosition(position);
      yoFrameEuclideanTrajectoryPoint.getLinearVelocity(linearVelocity);

      // Make sure they are all equal to zero:
      assertTrue(position.epsilonEquals(new Point3D(), 1e-10));
      assertTrue(linearVelocity.epsilonEquals(new Vector3D(), 1e-10));

      time = 9.9;
      pointForVerification.set(3.9, 2.2, 1.1);
      linearVelocityForVerification.set(8.8, 1.4, 9.22);

      assertFalse(Math.abs(yoFrameEuclideanTrajectoryPoint.getTime() - time) < 1e-7);
      assertFalse(pointForVerification.getFrameTuple().epsilonEquals(position, 1e-7));
      assertFalse(linearVelocityForVerification.getFrameTuple().epsilonEquals(linearVelocity, 1e-7));

      yoFrameEuclideanTrajectoryPoint.set(time, pointForVerification, linearVelocityForVerification);

      yoFrameEuclideanTrajectoryPoint.getPosition(position);
      yoFrameEuclideanTrajectoryPoint.getLinearVelocity(linearVelocity);

      assertEquals(time, yoFrameEuclideanTrajectoryPoint.getTime(), 1e-10);
      assertTrue(pointForVerification.getFrameTuple().epsilonEquals(position, 1e-10));
      assertTrue(linearVelocityForVerification.getFrameTuple().epsilonEquals(linearVelocity, 1e-10));

      YoFrameEuclideanTrajectoryPoint yoFrameEuclideanTrajectoryPointTwo = new YoFrameEuclideanTrajectoryPoint(namePrefix, nameSuffix + "Two", registry, worldFrame);

      double positionDistance = yoFrameEuclideanTrajectoryPoint.positionDistance(yoFrameEuclideanTrajectoryPointTwo);
      assertEquals(4.610856753359402, positionDistance, 1e-7);
      assertFalse(yoFrameEuclideanTrajectoryPoint.epsilonEquals(yoFrameEuclideanTrajectoryPointTwo, 1e-7));

      yoFrameEuclideanTrajectoryPointTwo.set(yoFrameEuclideanTrajectoryPoint);
      positionDistance = yoFrameEuclideanTrajectoryPoint.positionDistance(yoFrameEuclideanTrajectoryPointTwo);
      assertEquals(0.0, positionDistance, 1e-7);
      assertTrue(yoFrameEuclideanTrajectoryPoint.epsilonEquals(yoFrameEuclideanTrajectoryPointTwo, 1e-7));
      
      
      SimpleEuclideanTrajectoryPoint simplePoint = new SimpleEuclideanTrajectoryPoint();
      yoFrameEuclideanTrajectoryPoint.get(simplePoint);
      
      yoFrameEuclideanTrajectoryPoint.setToNaN();
      assertTrue(yoFrameEuclideanTrajectoryPoint.containsNaN());
      positionDistance = yoFrameEuclideanTrajectoryPoint.positionDistance(yoFrameEuclideanTrajectoryPointTwo);
      assertTrue(Double.isNaN(positionDistance));
      assertFalse(yoFrameEuclideanTrajectoryPoint.epsilonEquals(yoFrameEuclideanTrajectoryPointTwo, 1e-7));
      
      EuclideanTrajectoryPointInterface<?> trajectoryPointAsInterface = simplePoint;
      yoFrameEuclideanTrajectoryPoint.set(trajectoryPointAsInterface);
      
      positionDistance = yoFrameEuclideanTrajectoryPoint.positionDistance(yoFrameEuclideanTrajectoryPointTwo);
      assertEquals(0.0, positionDistance, 1e-7);
      assertTrue(yoFrameEuclideanTrajectoryPoint.epsilonEquals(yoFrameEuclideanTrajectoryPointTwo, 1e-7));
      
      String string = yoFrameEuclideanTrajectoryPoint.toString();
      String expectedString = "Euclidean trajectory point: (time =  9.90, Euclidean trajectory point: (time =  9.90, Euclidean waypoint: [position = ( 3.90,  2.20,  1.10), linearVelocity = ( 8.80,  1.40,  9.22)].))";      
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

      YoFrameEuclideanTrajectoryPoint yoFrameEuclideanTrajectoryPoint = new YoFrameEuclideanTrajectoryPoint(namePrefix, nameSuffix, registry, worldFrame);
      yoFrameEuclideanTrajectoryPoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      double time = 3.4;
      FramePoint position = new FramePoint(worldFrame, 1.0, 2.1, 3.7);
      FrameVector linearVelocity = new FrameVector(worldFrame, -0.4, 1.2, 3.3);

      yoFrameEuclideanTrajectoryPoint.setTime(time);
      yoFrameEuclideanTrajectoryPoint.setPosition(position);
      yoFrameEuclideanTrajectoryPoint.setLinearVelocity(linearVelocity);
      
      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose(worldFrame));

      FramePoint poseFramePosition = new FramePoint(worldFrame, new Point3D(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameOrientation poseOrientation = new FrameOrientation(worldFrame, new AxisAngle(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      yoFrameEuclideanTrajectoryPoint.registerReferenceFrame(poseFrame);
      
      yoFrameEuclideanTrajectoryPoint.changeFrame(poseFrame);
      
      assertFalse(position.epsilonEquals(yoFrameEuclideanTrajectoryPoint.getPosition().getFramePointCopy(), 1e-10));
      assertFalse(linearVelocity.epsilonEquals(yoFrameEuclideanTrajectoryPoint.getLinearVelocity().getFrameVectorCopy(), 1e-10));

      position.changeFrame(poseFrame);
      linearVelocity.changeFrame(poseFrame);

      assertTrue(position.epsilonEquals(yoFrameEuclideanTrajectoryPoint.getPosition().getFramePointCopy(), 1e-10));
      assertTrue(linearVelocity.epsilonEquals(yoFrameEuclideanTrajectoryPoint.getLinearVelocity().getFrameVectorCopy(), 1e-10));

      YoFrameEuclideanTrajectoryPoint yoFrameEuclideanTrajectoryPointTwo = new YoFrameEuclideanTrajectoryPoint(namePrefix, nameSuffix+"Two", registry, poseFrame);

      yoFrameEuclideanTrajectoryPointTwo.setTime(time);
      yoFrameEuclideanTrajectoryPointTwo.setPosition(position);
      yoFrameEuclideanTrajectoryPointTwo.setLinearVelocity(linearVelocity);
      
      assertTrue(yoFrameEuclideanTrajectoryPointTwo.epsilonEquals(yoFrameEuclideanTrajectoryPointTwo, 1e-10));
   }
}
