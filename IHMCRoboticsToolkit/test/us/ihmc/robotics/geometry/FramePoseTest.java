package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RotationTools.AxisAngleComparisonMode;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.JUnitTools;

public class FramePoseTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = ReferenceFrameMismatchException.class)
   public void testFrameMismatch()
   {
      RigidBodyTransform transform1 = new RigidBodyTransform();
      transform1.setIdentity();
      FramePose framePose1 = new FramePose(ReferenceFrame.getWorldFrame(), transform1);

      // Test frame mismatch
      FramePose framePose2 = new FramePose(ReferenceFrame.constructARootFrame("junk"), transform1);

      double distance = framePose1.getOrientationDistance(framePose2);

      distance = framePose1.getEffectiveDistanceToFramePose(framePose2, 10.0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetOrientationDistanceTrivial()
   {
      RigidBodyTransform transform1 = new RigidBodyTransform();
      transform1.setIdentity();
      FramePose framePose1 = new FramePose(ReferenceFrame.getWorldFrame(), transform1);

      RigidBodyTransform transform2 = new RigidBodyTransform();
      transform2.setIdentity();
      FramePose framePose2 = new FramePose(ReferenceFrame.getWorldFrame(), transform2);

      double distance = framePose1.getOrientationDistance(framePose2);
      assertEquals(0.0, distance, 1e-9);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetDistance()
   {
      FramePose framePose1 = new FramePose(ReferenceFrame.getWorldFrame());
      FramePose framePose2 = new FramePose(ReferenceFrame.getWorldFrame());

      Random random = new Random(100L);

      int numberOfTests = 1000;
      for (int i = 0; i < numberOfTests; i++)
      {
         {
            double angle = -Math.PI + 2 * Math.PI * random.nextDouble();
            Vector3d vector3d = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
            vector3d.normalize();

            AxisAngle4d axisAngle = new AxisAngle4d(vector3d, angle);

            double vectorScale = 10.0;
            Vector3d vector3dTranlation = new Vector3d(-vectorScale + 2.0 * vectorScale * random.nextDouble(),
                                             -vectorScale + 2.0 * vectorScale * random.nextDouble(), -vectorScale + 2.0 * vectorScale * random.nextDouble());
            RigidBodyTransform transform = new RigidBodyTransform();

            transform.set(axisAngle, vector3dTranlation);
            framePose1 = new FramePose(ReferenceFrame.getWorldFrame(), transform);
         }

         {
            double angle = -Math.PI + 2 * Math.PI * random.nextDouble();
            Vector3d vector3d = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
            vector3d.normalize();

            AxisAngle4d axisAngle = new AxisAngle4d(vector3d, angle);

            double vectorScale = 10.0;
            Vector3d vector3dTranlation = new Vector3d(-vectorScale + 2.0 * vectorScale * random.nextDouble(),
                                             -vectorScale + 2.0 * vectorScale * random.nextDouble(), -vectorScale + 2.0 * vectorScale * random.nextDouble());
            RigidBodyTransform transform = new RigidBodyTransform();

            transform.set(axisAngle, vector3dTranlation);
            framePose2 = new FramePose(ReferenceFrame.getWorldFrame(), transform);
         }

         double rotationDistance = framePose1.getOrientationDistance(framePose2);
         double positionDistance = framePose1.getPositionDistance(framePose2);
         
         double radiusForRotation =  random.nextDouble();
         
         RigidBodyTransform transformFromWorldToA1 = new RigidBodyTransform();
         framePose1.getPose(transformFromWorldToA1);
         transformFromWorldToA1.invert();
         
         RigidBodyTransform transformFromWorldToA2 = new RigidBodyTransform();
         framePose2.getPose(transformFromWorldToA2);
         transformFromWorldToA2.invert();
         
         //RigidBodyTransform transformA2toA1 = TransformTools.getTransformFromA2toA1(transformFromWorldToA1, transformFromWorldToA2);
         double distanceFromTansformTools = TransformTools.getSizeOfTransformBetweenTwoWithRotationScaled(transformFromWorldToA1, transformFromWorldToA2, radiusForRotation);
         
         double combinedDistance = framePose1.getEffectiveDistanceToFramePose(framePose2, radiusForRotation);
         
         double handCalculatedDistance = positionDistance + rotationDistance * radiusForRotation;
         assertEquals(combinedDistance, distanceFromTansformTools, 1e-9);
         
         assertEquals(combinedDistance, handCalculatedDistance, 1e-7);
         
         ReferenceFrame framePose1Frame ;
         {
            RigidBodyTransform transform = new RigidBodyTransform();
            framePose1.getPose(transform);
            transform.invert();
            framePose1Frame = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("frame1", ReferenceFrame.getWorldFrame(), transform);
         }
               
         framePose2.changeFrame(framePose1Frame);
         RigidBodyTransform transform = new RigidBodyTransform();
         framePose2.getPose(transform);
         AxisAngle4d axisAngle = new AxisAngle4d();
         transform.getRotation(axisAngle);
         double rotation = Math.abs(axisAngle.getAngle());

         assertEquals(rotation, rotationDistance, 1e-7);
         assertEquals(positionDistance, framePose2.getFramePointCopy().getPoint().distance(new Point3d()), 1e-7);
      }

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetTransform()
   {
      Random random = new Random(1179L);
      RigidBodyTransform transform = RigidBodyTransform.generateRandomTransform(random);
      FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame(), transform);

      RigidBodyTransform transformCheck = new RigidBodyTransform();

      framePose.getRigidBodyTransform(transformCheck);
      RigidBodyTransformTest.assertTransformEquals(transform, transformCheck, 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRotatePoseAboutOffsetAxisAndCheckTranslation()
   {
      Random random = new Random(1179L);
      double angleToRotate = RandomTools.generateRandomDouble(random, Math.toRadians(720.0));

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FramePose framePose = new FramePose(worldFrame);
      framePose.setPosition(1.0, 0.0, 1.0);
      framePose.setOrientation(RandomTools.generateRandomQuaternion(random));

      FrameVector rotationAxis = new FrameVector(worldFrame, 0.0, 0.0, 1.0);
      FramePoint rotationAxisOrigin = new FramePoint(worldFrame, 0.0, 0.0, 0.0);
      framePose.rotatePoseAboutAxis(rotationAxis, rotationAxisOrigin, angleToRotate);

      Point3d actualPosePositionAfterRotation = new Point3d();
      framePose.getPosition(actualPosePositionAfterRotation);

      Point3d desiredPosition = new Point3d(Math.cos(angleToRotate), Math.sin(angleToRotate), 1.0);

      Vector3d positionError = new Vector3d();
      positionError.sub(desiredPosition, actualPosePositionAfterRotation);

      assertTrue("Reference Frame shoud not have changed.  Actual frame: " + framePose.getReferenceFrame().getName() + ", Desired frame: "
            + worldFrame.getName(), framePose.getReferenceFrame() == worldFrame);
      assertEquals("FramePose Position after rotation is wrong.  Desired position: " + desiredPosition + ", actual position: "
            + actualPosePositionAfterRotation, 0.0, positionError.length(), 1e-3);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRotatePoseAboutCollinearAxisAndCheckTranslation()
   {
      Random random = new Random(1179L);
      double angleToRotate = RandomTools.generateRandomDouble(random, Math.toRadians(720.0));

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FramePose framePose = new FramePose(worldFrame);
      framePose.setPosition(1.0, 0.0, 1.0);
      framePose.setOrientation(RandomTools.generateRandomQuaternion(random));

      framePose.rotatePoseAboutAxis(framePose.getReferenceFrame(), Axis.Z, angleToRotate);

      Point3d actualPosePositionAfterRotation = new Point3d();
      framePose.getPosition(actualPosePositionAfterRotation);

      Point3d desiredPosition = new Point3d(Math.cos(angleToRotate), Math.sin(angleToRotate), 1.0);

      Vector3d positionError = new Vector3d();
      positionError.sub(desiredPosition, actualPosePositionAfterRotation);

      assertTrue("Reference Frame shoud not have changed.  Actual frame: " + framePose.getReferenceFrame().getName() + ", Desired frame: "
            + worldFrame.getName(), framePose.getReferenceFrame() == worldFrame);
      assertEquals("FramePose Position after rotation is wrong.  Desired position: " + desiredPosition + ", actual position: "
            + actualPosePositionAfterRotation, 0.0, positionError.length(), 1e-3);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRotatePoseAboutZAxisAndCheckOrientation()
   {
      Random random = new Random(1179L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FramePose initialPose = new FramePose(worldFrame);
      FramePose rotatedPose = new FramePose(worldFrame);

      double angleToRotate = Math.toRadians(-720.01);
      AxisAngle4d desiredRotationAxisAngle = new AxisAngle4d(0.0, 0.0, 1.0, angleToRotate);

      while (angleToRotate < Math.toRadians(720.0))
      {
         initialPose.setPosition(0.0, 0.0, RandomTools.generateRandomDouble(random, 10.0));
         rotatedPose.setPoseIncludingFrame(initialPose);

         desiredRotationAxisAngle.setAngle(angleToRotate);
         rotatedPose.rotatePoseAboutAxis(rotatedPose.getReferenceFrame(), Axis.Z, angleToRotate);

         PoseReferenceFrame initialPoseFrame = new PoseReferenceFrame("initialPoseFrame", initialPose);
         rotatedPose.changeFrame(initialPoseFrame);

         AxisAngle4d actualRotationAxisAngle = new AxisAngle4d();
         rotatedPose.getOrientation(actualRotationAxisAngle);

         assertTrue("Actual rotation: " + actualRotationAxisAngle + " does not match desired: " + desiredRotationAxisAngle,
               RotationTools.axisAngleEpsilonEquals(desiredRotationAxisAngle, actualRotationAxisAngle, 1e-5,
                     AxisAngleComparisonMode.IGNORE_FLIPPED_AXES_ROTATION_DIRECTION_AND_COMPLETE_ROTATIONS));

         angleToRotate += Math.toRadians(5.0);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRotatePoseAboutCollinearAxisIncrementally()
   {
      Random random = new Random(1179L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      double angleToRotate = RandomTools.generateRandomDouble(random, Math.toRadians(720.0));

      FramePose framePose = new FramePose(worldFrame);
      framePose.setPosition(0.0, 0.0, 1.0);
      FramePose framePoseCopy = new FramePose(framePose);

      framePose.rotatePoseAboutAxis(framePose.getReferenceFrame(), Axis.Z, 0.5 * angleToRotate);
      framePose.rotatePoseAboutAxis(framePose.getReferenceFrame(), Axis.Z, 0.5 * angleToRotate);

      double positionDistance = framePose.getPositionDistance(framePoseCopy);
      double orientationDistance = framePose.getOrientationDistance(framePoseCopy);

      double angleToRotate0to2PI = angleToRotate % (2.0 * Math.PI);
      double desiredOrientationDistance = Math.min(angleToRotate0to2PI, 2.0 * Math.PI - angleToRotate0to2PI);

      assertTrue("Reference Frame shoud not have changed.  Actual frame: " + framePose.getReferenceFrame().getName() + ", Desired frame: "
            + worldFrame.getName(), framePose.getReferenceFrame() == worldFrame);
      assertEquals("Change in FramePose Position after rotation is wrong.", 0.0, positionDistance, 1e-3);
      assertEquals("Change in FramePose Orientation after rotation is wrong.", desiredOrientationDistance, orientationDistance, Math.toRadians(0.1));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRotateAndUnrotatePoseAboutCollinearAxis()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      double angleToRotate = Math.toRadians(90.0);

      FramePose framePose = new FramePose(worldFrame);
      framePose.setPosition(0.0, 0.0, 1.0);
      FramePose framePoseCopy = new FramePose(framePose);

      framePose.rotatePoseAboutAxis(framePose.getReferenceFrame(), Axis.Z, 0.5 * angleToRotate);
      framePose.rotatePoseAboutAxis(framePose.getReferenceFrame(), Axis.Z, -0.5 * angleToRotate);

      double positionDistance = framePose.getPositionDistance(framePoseCopy);
      double orientationDistance = framePose.getOrientationDistance(framePoseCopy);

      assertTrue("Reference Frame shoud not have changed.  Actual frame: " + framePose.getReferenceFrame().getName() + ", Desired frame: "
            + worldFrame.getName(), framePose.getReferenceFrame() == worldFrame);
      assertEquals("Change in FramePose Position after rotation is wrong.", 0.0, positionDistance, 1e-3);
      assertEquals("Change in FramePose Orientation after rotation is wrong.", 0.0, orientationDistance, Math.toRadians(0.1));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRotatePoseLockOrientation()
   {
      boolean lockPosition = false;
      boolean lockOrientation = true;

      Random random = new Random(1179L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      double angleToRotate = Math.toRadians(-720.0);

      FramePose initialPose = new FramePose(worldFrame);
      FramePose rotatedPose = new FramePose(worldFrame);
      
      Point3d actualPosition = new Point3d();
      Point3d desiredPosition = new Point3d();
      Vector3d positionError = new Vector3d();

      while (angleToRotate < Math.toRadians(720.0))
      {
         initialPose.setPosition(1.0, 0.0, 1.0);
         initialPose.setOrientation(RandomTools.generateRandomQuaternion(random));
         rotatedPose.setPoseIncludingFrame(initialPose);
         
         rotatedPose.rotatePoseAboutAxis(worldFrame, Axis.Z, angleToRotate, lockPosition, lockOrientation);
         rotatedPose.getPosition(actualPosition);

         desiredPosition.set(Math.cos(angleToRotate), Math.sin(angleToRotate), 1.0);
         positionError.sub(desiredPosition, actualPosition);

         assertTrue("Reference Frame shoud not have changed.  Actual frame: " + rotatedPose.getReferenceFrame().getName() + ", Desired frame: "
               + worldFrame.getName(), rotatedPose.getReferenceFrame() == worldFrame);
         
         assertEquals("FramePose Position after rotation is wrong.  Desired position: " + desiredPosition + ", actual position: "
               + actualPosition, 0.0, positionError.length(), 1e-3);
         
         assertEquals("Change in FramePose Orientation after rotation is wrong.  Orientation should not have changed.", 0.0,
               rotatedPose.getOrientationDistance(initialPose), Math.toRadians(0.1));

         angleToRotate += Math.toRadians(1.0);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRotatePoseLockPosition()
   {
      boolean lockPosition = true;
      boolean lockOrientation = false;

      Random random = new Random(1179L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      double angleToRotate = RandomTools.generateRandomDouble(random, 0.0);

      FramePose rotatedPose = new FramePose(worldFrame);
      FramePose initialPose = new FramePose(worldFrame);

      while (angleToRotate < Math.toRadians(180.0))
      {
         rotatedPose.setPosition(1.0, 0.0, 1.0);
         rotatedPose.setOrientation(RandomTools.generateRandomQuaternion(random));

         initialPose.setPoseIncludingFrame(rotatedPose);

         Point3d actualpositionBeforeRotation = new Point3d();
         rotatedPose.getPosition(actualpositionBeforeRotation);

         rotatedPose.rotatePoseAboutAxis(rotatedPose.getReferenceFrame(), Axis.Z, angleToRotate, lockPosition, lockOrientation);

         Point3d actualPosePositionAfterRotation = new Point3d();
         rotatedPose.getPosition(actualPosePositionAfterRotation);

         Vector3d changeInPosition = new Vector3d();
         changeInPosition.sub(actualPosePositionAfterRotation, actualpositionBeforeRotation);

         assertTrue("Reference Frame shoud not have changed.  Actual frame: " + rotatedPose.getReferenceFrame().getName() + ", Desired frame: "
               + worldFrame.getName(), rotatedPose.getReferenceFrame() == worldFrame);
         assertEquals("FramePose Position after rotation is wrong.", 0.0, changeInPosition.length(), 1e-3);
         assertEquals("Change in FramePose Orientation after rotation is wrong.", angleToRotate, rotatedPose.getOrientationDistance(initialPose),
               Math.toRadians(0.1));

         angleToRotate += Math.toRadians(5.0);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetTranslationBetweenPoses()
   {
      Random random = new Random(100L);

      ReferenceFrame testFrame = ReferenceFrame.getWorldFrame();

      FramePose thisPose = new FramePose(testFrame);
      FramePose thatPose = new FramePose(testFrame);
      FrameVector actualTranslationNOTfromRotation = new FrameVector(testFrame);
      FrameVector shouldEqualTotalTranslation = new FrameVector(testFrame);

      double[] thetaToTest = new double[] { Math.toRadians(-90.0), Math.toRadians(-45.0), Math.toRadians(45.0), Math.toRadians(90.0) };

      for (double theta : thetaToTest)
      {
         thatPose.setPoseIncludingFrame(thisPose);
         thatPose.rotatePoseAboutAxis(thisPose.getReferenceFrame(), Axis.Z, theta);

         Vector3d randomTranslation = RandomTools.generateRandomVector(random);
         actualTranslationNOTfromRotation.setIncludingFrame(testFrame, randomTranslation);

         thatPose.translate(actualTranslationNOTfromRotation.tuple);

         FrameVector translationNOTfromRotation = thisPose.getTranslationNOTDueToRotationAboutFrame(thatPose);
         FrameVector translationDueToRotation = thisPose.getTranslationDueToRotationAboutFrame(thatPose);
         FrameVector totalTranslation = thisPose.getTranslationToOtherPoseTotal(thatPose);

         shouldEqualTotalTranslation.add(translationDueToRotation, translationNOTfromRotation);

         FrameVectorTest.assertFrameVectorEquals(actualTranslationNOTfromRotation, translationNOTfromRotation, 1e-15);
         FrameVectorTest.assertFrameVectorEquals(totalTranslation, shouldEqualTotalTranslation, 1e-15);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetOrientationTransformBetweenPoses()
   {
      Random random = new Random(100L);

      FramePose thisPose = new FramePose();
      FramePose thatPose = new FramePose(thisPose);

      Quat4d actualRotationFromThisToThat = RandomTools.generateRandomQuaternion(random);
      thatPose.setOrientation(actualRotationFromThisToThat);

      RigidBodyTransform transfromFromThisToThat = thisPose.getTransformFromThisToThat(thatPose);

      Quat4d rotationFromThisToThat = new Quat4d();
      transfromFromThisToThat.getRotation(rotationFromThisToThat);

      JUnitTools.assertQuaternionsEqual(actualRotationFromThisToThat, rotationFromThisToThat, 1e-15);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetRotationAngleBetweenRotatedPoses()
   {
      Random random = new Random(100L);
      double epsilonAssert = 1e-6;

      FramePose initialPose = new FramePose();
      initialPose.setPosition(RandomTools.generateRandomVector(random));
      initialPose.setYawPitchRoll(AngleTools.generateRandomAngle(random), AngleTools.generateRandomAngle(random), AngleTools.generateRandomAngle(random));

      FramePose rotatedPose = new FramePose();

      ReferenceFrame rotationAxisFrame = ReferenceFrame.getWorldFrame();
      FrameVector rotationAxis = new FrameVector(rotationAxisFrame, RandomTools.generateRandomVector(random));
      rotationAxis.normalize();
      FramePoint rotationAxisOrigin = new FramePoint(rotationAxisFrame, RandomTools.generateRandomVector(random));

      FrameVector computedAxis = new FrameVector();

      int numberOfPassedTests = 0;
      for (double rotationAngle : AngleTools.generateArrayOfTestAngles(1000, 1e-3, true, false))
      {
         rotatedPose.setPose(initialPose);
         rotatedPose.rotatePoseAboutAxis(rotationAxis, rotationAxisOrigin, rotationAngle);

         double computedRotationAngle = initialPose.getAxisAngleRotationToOtherPose(rotatedPose, computedAxis);
         double angleDifference = AngleTools.computeAngleDifferenceMinusPiToPi(rotationAngle, computedRotationAngle);

         if (Math.abs(angleDifference) > epsilonAssert)
         {
            boolean rotationAxisIsFlipped = MathTools.epsilonEquals(computedAxis.dot(rotationAxis), -1.0, epsilonAssert);

            String errorMsg = "Computed rotation angle, " + Math.toDegrees(computedRotationAngle) + " degrees, does not equal actual rotation angle, "
                  + Math.toDegrees(rotationAngle) + "\n and computed rotation axis: " + computedAxis + "\n is not a flipped version of actual axis: "
                  + rotationAxis + "\n Number of Passed Tests: " + numberOfPassedTests + "\n";

            assertTrue(errorMsg, rotationAxisIsFlipped);
         }
         numberOfPassedTests++;
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetRotationAxisBetweenRotatedPoses()
   {
      Random random = new Random(100L);

      FramePose initialPose = new FramePose();
      initialPose.setPosition(RandomTools.generateRandomVector(random));
      initialPose.setYawPitchRoll(AngleTools.generateRandomAngle(random), AngleTools.generateRandomAngle(random), AngleTools.generateRandomAngle(random));

      FramePose rotatedPose = new FramePose();

      ReferenceFrame rotationAxisFrame = ReferenceFrame.getWorldFrame();
      FrameVector rotationAxis = new FrameVector(rotationAxisFrame, RandomTools.generateRandomVector(random));
      rotationAxis.normalize();
      FramePoint rotationAxisOrigin = new FramePoint(rotationAxisFrame, RandomTools.generateRandomVector(random));

      FrameVector rotationAxisComputed = new FrameVector();
      FramePoint rotationAxisOriginComputed = new FramePoint();

      int numberOfPassedTests = 0;
      for (double rotationAngle : AngleTools.generateArrayOfTestAngles(100, 1e-5, false, true))
      {
         String rotationAngleMsg = "\n  Tested rotation angle = " + Math.toDegrees(rotationAngle) + " degrees \n";

         rotatedPose.setPose(initialPose);
         rotatedPose.rotatePoseAboutAxis(rotationAxis, rotationAxisOrigin, rotationAngle);

         initialPose.getAxisAngleRotationToOtherPose(rotatedPose, rotationAxisComputed);
         initialPose.getSpatialAxisOfRotationAndAngleToOtherPose(rotatedPose, rotationAxisComputed, rotationAxisOriginComputed);

         String errorMsg = "Computed rotation axis, \n" + rotationAxisComputed + "\n is not parallel with actual rotation axis, \n" + rotationAxis
               + rotationAngleMsg + "\n Number of Passed Tests: " + numberOfPassedTests + "\n";
         assertTrue(errorMsg, rotationAxisComputed.isEpsilonParallel(rotationAxis));

         numberOfPassedTests++;
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetRotationAxisOriginBetweenPoses()
   {
      Random random = new Random(100L);

      FramePose initialPose = new FramePose();
      FramePose rotatedPose = new FramePose();

      ReferenceFrame rotationAxisFrame = ReferenceFrame.getWorldFrame();
      FrameVector rotationAxis = new FrameVector(rotationAxisFrame, RandomTools.generateRandomVector(random));
      rotationAxis.normalize();
      FramePoint rotationAxisOrigin = new FramePoint(rotationAxisFrame, RandomTools.generateRandomVector(random));

      FrameVector rotationAxisComputed = new FrameVector();
      FramePoint rotationAxisOriginComputed = new FramePoint();
      FrameVector actualToComputedOrigin = new FrameVector();

      int numberOfPassedTests = 0;
      for (double rotationAngle : AngleTools.generateArrayOfTestAngles(100, 1e-1, false, false))
      {
         String rotationAngleMsg = "\n  Tested rotation angle = " + Math.toDegrees(rotationAngle) + " degrees \n";

         initialPose.setPosition(RandomTools.generateRandomVector(random));
         initialPose.setYawPitchRoll(AngleTools.generateRandomAngle(random), AngleTools.generateRandomAngle(random), AngleTools.generateRandomAngle(random));

         rotatedPose.setPose(initialPose);
         rotatedPose.rotatePoseAboutAxis(rotationAxis, rotationAxisOrigin, rotationAngle);

         initialPose.getSpatialAxisOfRotationAndAngleToOtherPose(rotatedPose, rotationAxisComputed, rotationAxisOriginComputed);

         actualToComputedOrigin.sub(rotationAxisOriginComputed, rotationAxisOrigin);
         String errorMsg = "Computed rotation axis origin must coincide with actual origin, or lie along rotation axis.\n Alignment error = "
               + Math.toDegrees(actualToComputedOrigin.angle(rotationAxis)) + " degrees" + rotationAngleMsg + "\n Number of Passed Tests: "
               + numberOfPassedTests + "\n";
         assertTrue(errorMsg, actualToComputedOrigin.isEpsilonParallel(rotationAxis));

         numberOfPassedTests++;
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRotatePoseGetAxisOfRotationAndUnrotatePose()
   {
      Random random = new Random(100L);

      FramePose initialPose = new FramePose();
      FramePose rotatedPose = new FramePose();

      ReferenceFrame rotationAxisFrame = ReferenceFrame.getWorldFrame();
      FrameVector rotationAxis = new FrameVector(rotationAxisFrame, RandomTools.generateRandomVector(random));
      rotationAxis.normalize();
      FramePoint rotationAxisOrigin = new FramePoint(rotationAxisFrame, RandomTools.generateRandomVector(random));

      FrameVector rotationAxisComputed = new FrameVector();
      FramePoint rotationAxisOriginComputed = new FramePoint();
      FramePose rotatedPoseComputed = new FramePose(initialPose);

      int numberOfPassedTests = 0;

      for (double rotationAngle : AngleTools.generateArrayOfTestAngles(100, 1e-5, true, true))
      {
         String rotationAngleMsg = "\n Tested rotation angle = " + Math.toDegrees(rotationAngle) + " degrees \n";

         initialPose.setPosition(RandomTools.generateRandomVector(random));
         initialPose.setYawPitchRoll(AngleTools.generateRandomAngle(random), AngleTools.generateRandomAngle(random), AngleTools.generateRandomAngle(random));

         rotatedPose.setPose(initialPose);
         rotatedPose.rotatePoseAboutAxis(rotationAxis, rotationAxisOrigin, rotationAngle);

         double rotationAngleComputed = initialPose.getSpatialAxisOfRotationAndAngleToOtherPose(rotatedPose, rotationAxisComputed, rotationAxisOriginComputed);

         rotatedPoseComputed.setPoseIncludingFrame(initialPose);
         rotatedPoseComputed.rotatePoseAboutAxis(rotationAxisComputed, rotationAxisOriginComputed, rotationAngleComputed);

         double orientationError = rotatedPoseComputed.getOrientationDistance(rotatedPose);
         assertEquals("FramePose Orientation after rotation is wrong.  Orientation error = " + orientationError + rotationAngleMsg, 0.0, orientationError, 1e-6);

         double positionError = rotatedPoseComputed.getPositionDistance(rotatedPose);
         String positionAssertErrorMsg = "FramePose Position after rotation is wrong.  Position error = " + positionError + rotationAngleMsg
               + "\n Number of Passed Tests: " + numberOfPassedTests + "\n";
         assertEquals(positionAssertErrorMsg, 0.0, positionError, 1e-6);

         String moreInfo = "Initial Pose:   " + initialPose + ".\n  Initial rotation axis: " + rotationAxis + ".\n  Initial rotation axis origin: "
               + rotationAxisOrigin + ".\n  Correct Rotated Position: " + rotatedPose.getFramePointCopy() + ",\n Actual Rotated Position: "
               + rotatedPoseComputed.getFramePointCopy() + "\n";

         numberOfPassedTests++;
      }
   }
}