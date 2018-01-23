package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.RotationTools.AxisAngleComparisonMode;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class FramePoseTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetOrientationDistanceTrivial()
   {
      RigidBodyTransform transform1 = new RigidBodyTransform();
      transform1.setIdentity();
      FramePose3D framePose1 = new FramePose3D(ReferenceFrame.getWorldFrame(), transform1);

      RigidBodyTransform transform2 = new RigidBodyTransform();
      transform2.setIdentity();
      FramePose3D framePose2 = new FramePose3D(ReferenceFrame.getWorldFrame(), transform2);

      double distance = framePose1.getOrientationDistance(framePose2);
      assertEquals(0.0, distance, 1e-9);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetTransform()
   {
      Random random = new Random(1179L);
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      FramePose3D framePose = new FramePose3D(ReferenceFrame.getWorldFrame(), transform);

      RigidBodyTransform transformCheck = new RigidBodyTransform();

      framePose.get(transformCheck);
      EuclidCoreTestTools.assertRigidBodyTransformEquals(transform, transformCheck, 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRotatePoseAboutOffsetAxisAndCheckTranslation()
   {
      Random random = new Random(1179L);
      double angleToRotate = RandomNumbers.nextDouble(random, Math.toRadians(720.0));

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FramePose3D framePose = new FramePose3D(worldFrame);
      framePose.setPosition(1.0, 0.0, 1.0);
      framePose.setOrientation(RandomGeometry.nextQuaternion(random));

      FrameVector3D rotationAxis = new FrameVector3D(worldFrame, 0.0, 0.0, 1.0);
      FramePoint3D rotationAxisOrigin = new FramePoint3D(worldFrame, 0.0, 0.0, 0.0);
      GeometryTools.rotatePoseAboutAxis(rotationAxis, rotationAxisOrigin, angleToRotate, framePose);

      Point3D actualPosePositionAfterRotation = new Point3D(framePose.getPosition());

      Point3D desiredPosition = new Point3D(Math.cos(angleToRotate), Math.sin(angleToRotate), 1.0);

      Vector3D positionError = new Vector3D();
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
      double angleToRotate = RandomNumbers.nextDouble(random, Math.toRadians(720.0));

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FramePose3D framePose = new FramePose3D(worldFrame);
      framePose.setPosition(1.0, 0.0, 1.0);
      framePose.setOrientation(RandomGeometry.nextQuaternion(random));

      GeometryTools.rotatePoseAboutAxis(framePose.getReferenceFrame(), Axis.Z, angleToRotate, framePose);

      Point3D actualPosePositionAfterRotation = new Point3D(framePose.getPosition());

      Point3D desiredPosition = new Point3D(Math.cos(angleToRotate), Math.sin(angleToRotate), 1.0);

      Vector3D positionError = new Vector3D();
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

      FramePose3D initialPose = new FramePose3D(worldFrame);
      FramePose3D rotatedPose = new FramePose3D(worldFrame);

      double angleToRotate = Math.toRadians(-720.01);
      AxisAngle desiredRotationAxisAngle = new AxisAngle(0.0, 0.0, 1.0, angleToRotate);

      while (angleToRotate < Math.toRadians(720.0))
      {
         initialPose.setPosition(0.0, 0.0, RandomNumbers.nextDouble(random, 10.0));
         rotatedPose.setIncludingFrame(initialPose);

         desiredRotationAxisAngle.setAngle(angleToRotate);
         GeometryTools.rotatePoseAboutAxis(rotatedPose.getReferenceFrame(), Axis.Z, angleToRotate, rotatedPose);

         PoseReferenceFrame initialPoseFrame = new PoseReferenceFrame("initialPoseFrame", initialPose);
         rotatedPose.changeFrame(initialPoseFrame);

         AxisAngle actualRotationAxisAngle = new AxisAngle(rotatedPose.getOrientation());

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
      double angleToRotate = RandomNumbers.nextDouble(random, Math.toRadians(720.0));

      FramePose3D framePose = new FramePose3D(worldFrame);
      framePose.setPosition(0.0, 0.0, 1.0);
      FramePose3D framePoseCopy = new FramePose3D(framePose);

      GeometryTools.rotatePoseAboutAxis(framePose.getReferenceFrame(), Axis.Z, 0.5 * angleToRotate, framePose);
      GeometryTools.rotatePoseAboutAxis(framePose.getReferenceFrame(), Axis.Z, 0.5 * angleToRotate, framePose);

      double positionDistance = framePose.getPositionDistance(framePoseCopy);
      double orientationDistance = AngleTools.trimAngleMinusPiToPi(framePose.getOrientationDistance(framePoseCopy));

      double desiredOrientationDistance = AngleTools.trimAngleMinusPiToPi(angleToRotate);

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

      FramePose3D framePose = new FramePose3D(worldFrame);
      framePose.setPosition(0.0, 0.0, 1.0);
      FramePose3D framePoseCopy = new FramePose3D(framePose);

      GeometryTools.rotatePoseAboutAxis(framePose.getReferenceFrame(), Axis.Z, 0.5 * angleToRotate , framePose);
      GeometryTools.rotatePoseAboutAxis(framePose.getReferenceFrame(), Axis.Z, -0.5 * angleToRotate, framePose);

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

      FramePose3D initialPose = new FramePose3D(worldFrame);
      FramePose3D rotatedPose = new FramePose3D(worldFrame);
      
      Point3D actualPosition = new Point3D();
      Point3D desiredPosition = new Point3D();
      Vector3D positionError = new Vector3D();

      while (angleToRotate < Math.toRadians(720.0))
      {
         initialPose.setPosition(1.0, 0.0, 1.0);
         initialPose.setOrientation(RandomGeometry.nextQuaternion(random));
         rotatedPose.setIncludingFrame(initialPose);
         
         GeometryTools.rotatePoseAboutAxis(worldFrame, Axis.Z, angleToRotate, lockPosition, lockOrientation, rotatedPose);
         actualPosition.set(rotatedPose.getPosition());

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
      double angleToRotate = RandomNumbers.nextDouble(random, 0.0);

      FramePose3D rotatedPose = new FramePose3D(worldFrame);
      FramePose3D initialPose = new FramePose3D(worldFrame);

      while (angleToRotate < Math.toRadians(180.0))
      {
         rotatedPose.setPosition(1.0, 0.0, 1.0);
         rotatedPose.setOrientation(RandomGeometry.nextQuaternion(random));

         initialPose.setIncludingFrame(rotatedPose);

         Point3D actualpositionBeforeRotation = new Point3D(rotatedPose.getPosition());

         GeometryTools.rotatePoseAboutAxis(rotatedPose.getReferenceFrame(), Axis.Z, angleToRotate, lockPosition, lockOrientation, rotatedPose);

         Point3D actualPosePositionAfterRotation = new Point3D(rotatedPose.getPosition());

         Vector3D changeInPosition = new Vector3D();
         changeInPosition.sub(actualPosePositionAfterRotation, actualpositionBeforeRotation);

         assertTrue("Reference Frame shoud not have changed.  Actual frame: " + rotatedPose.getReferenceFrame().getName() + ", Desired frame: "
               + worldFrame.getName(), rotatedPose.getReferenceFrame() == worldFrame);
         assertEquals("FramePose Position after rotation is wrong.", 0.0, changeInPosition.length(), 1e-3);
         assertEquals("Change in FramePose Orientation after rotation is wrong.", angleToRotate, rotatedPose.getOrientationDistance(initialPose),
               Math.toRadians(0.1));

         angleToRotate += Math.toRadians(5.0);
      }
   }
}