package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.communication.subscribers.TimeStampedTransformBuffer;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets={TestPlanTarget.Fast})
public class OutdatedPoseToUpToDateReferenceFrameUpdaterTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @DeployableTestMethod(estimatedDuration = 0.2)
   @Test(timeout = 3000)
   public void testGetUpToDateTimeStampedBufferNewestTimeStamp()
   {
      FramePose upToDatePoseInPresent = new FramePose(worldFrame);
      PoseReferenceFrame upToDateReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", upToDatePoseInPresent);
      int numberOfUpToDateTransforms = 20;
      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(
            numberOfUpToDateTransforms, upToDateReferenceFrameInPresent);

      Random random = new Random(42L);
      long timeStamp;
      RigidBodyTransform transform;

      for (int i = 0; i < 100; i++)
      {
         timeStamp = random.nextLong();
         transform = generateRandomUpToDateTransforms(random);
         outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(transform, timeStamp);
         assertTrue(timeStamp == outdatedPoseToUpToDateReferenceFrameUpdater.getStateEstimatorTimeStampedBufferNewestTimestamp());
      }
   }

   @DeployableTestMethod(estimatedDuration = 0.2)
   @Test(timeout = 3000)
   public void testGetUpToDateTimeStampedBufferOldestTimeStamp()
   {
      FramePose upToDatePoseInPresent = new FramePose(worldFrame);
      PoseReferenceFrame upToDateReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", upToDatePoseInPresent);
      int numberOfUpToDateTransforms = 20;
      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(
            numberOfUpToDateTransforms, upToDateReferenceFrameInPresent);

      Random random = new Random(42L);
      long timeStamp = 100;
      long firstTimeStamp = timeStamp;
      RigidBodyTransform transform;

      for (int i = 0; i < 100; i++)
      {
         transform = generateRandomUpToDateTransforms(random);
         outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(transform, timeStamp);
         if (i < 20)
            assertTrue(outdatedPoseToUpToDateReferenceFrameUpdater.getStateEstimatorTimeStampedBufferOldestTimestamp() == firstTimeStamp);
         else
            assertTrue((timeStamp - (numberOfUpToDateTransforms - 1)) == outdatedPoseToUpToDateReferenceFrameUpdater.getStateEstimatorTimeStampedBufferOldestTimestamp());
         timeStamp++;
      }
   }

   @DeployableTestMethod(estimatedDuration = 0.2)
   @Test(timeout = 3000)
   public void testUpdateOutdatedTransformWithKnownOffsets()
   {
      int numberOfUpToDateTransforms = 10;
      int numberOfOutdatedTransforms = 3;
      long firstTimeStamp = 1000;
      long lastTimeStamp = 2000;
      int numberOfTicksOfDelay = 100;
      Random random = new Random(1987L);

      Vector3d[] translationOffsets = new Vector3d[numberOfOutdatedTransforms];
      Quat4d[] orientationOffsets = new Quat4d[numberOfOutdatedTransforms];
      long[] outdatedTimeStamps = new long[numberOfOutdatedTransforms];

      FramePose upToDatePoseInPresent = new FramePose(worldFrame);
      PoseReferenceFrame upToDateReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", upToDatePoseInPresent);
      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(
            numberOfUpToDateTransforms, upToDateReferenceFrameInPresent);
      ReferenceFrame outdatedReferenceFrame_ToBeUpdated;
      outdatedReferenceFrame_ToBeUpdated = outdatedPoseToUpToDateReferenceFrameUpdater.getLocalizationReferenceFrameToBeUpdated();
      
      TimeStampedTransformBuffer upToDateTimeStampedTransformPoseBuffer = new TimeStampedTransformBuffer(numberOfUpToDateTransforms);
      TimeStampedTransformBuffer outdatedTimeStampedTransformBuffer = new TimeStampedTransformBuffer(numberOfOutdatedTransforms);

      //generate uptoDateTransforms used later in the test as waypoints
      for (int i = 0; i < numberOfUpToDateTransforms; i++)
      {
         long timeStamp = (long) (i * (lastTimeStamp - firstTimeStamp) / numberOfUpToDateTransforms + firstTimeStamp);
         RigidBodyTransform upToDateTransform = generateRandomUpToDateTransforms(random);
         upToDateTimeStampedTransformPoseBuffer.put(upToDateTransform, timeStamp);
         outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(upToDateTransform, timeStamp);
      }
      
      //generate outdatedTransforms offsets based on the upToDateTransforms
      for (int j = 0; j < numberOfOutdatedTransforms; j++)
      {
         long timeStamp = (long) (j * (lastTimeStamp * 0.8 - firstTimeStamp * 0.8) / numberOfOutdatedTransforms + firstTimeStamp * 1.2);
         outdatedTimeStamps[j] = timeStamp;

         translationOffsets[j] = RandomTools.generateRandomVector(random, -2.0, -2.0, 0.0, 2.0, 2.0, 2.0);
         orientationOffsets[j] = RandomTools.generateRandomQuaternion(random, Math.PI / 2.0);//RandomTools.generateRandomQuaternion(random, Math.PI);

         RigidBodyTransform outdatedTransform = generateOutdatedTransformWithTranslationAndOrientationOffset(upToDateTimeStampedTransformPoseBuffer, timeStamp,
               orientationOffsets[j], translationOffsets[j]);

         outdatedTimeStampedTransformBuffer.put(outdatedTransform, timeStamp);
      }

      int outdatedTimeStampsIndex = -1;
      for (long timeStamp = firstTimeStamp; timeStamp < lastTimeStamp; timeStamp++)
      {
         //////////////////  update the upToDate referenceFrame
         TimeStampedTransform3D upToDateTimeStampedTransform = new TimeStampedTransform3D();
         upToDateTimeStampedTransformPoseBuffer.findTransform(timeStamp, upToDateTimeStampedTransform);
         upToDatePoseInPresent.setPose(upToDateTimeStampedTransform.getTransform3D());
         upToDateReferenceFrameInPresent.setPoseAndUpdate(upToDatePoseInPresent);
         //////////////////
         if (outdatedTimeStamps[0] == timeStamp - numberOfTicksOfDelay)
            outdatedTimeStampsIndex++;
         if (outdatedTimeStampsIndex >= 0)
         {
            if (outdatedTimeStampsIndex < numberOfOutdatedTransforms && outdatedTimeStamps[outdatedTimeStampsIndex] == timeStamp - numberOfTicksOfDelay)
            {
               TimeStampedTransform3D outdatedTimeStampedTransform = new TimeStampedTransform3D();
               outdatedTimeStampedTransformBuffer.findTransform(outdatedTimeStamps[outdatedTimeStampsIndex], outdatedTimeStampedTransform);
               assertTrue(outdatedPoseToUpToDateReferenceFrameUpdater.stateEstimatorTimeStampedBufferIsInRange(outdatedTimeStampedTransform.getTimeStamp()));
               outdatedPoseToUpToDateReferenceFrameUpdater.updateLocalizationTransform(outdatedTimeStampedTransform);
               outdatedTimeStampsIndex++;
            }
         }
         outdatedReferenceFrame_ToBeUpdated.update();
         FramePose outdatedPoseUpdatedInWorldFrame = new FramePose(outdatedReferenceFrame_ToBeUpdated);
         outdatedPoseUpdatedInWorldFrame.changeFrame(worldFrame);

         Vector3d upToDateReferenceFrameInPresent_Translation = new Vector3d();
         upToDatePoseInPresent.getPosition(upToDateReferenceFrameInPresent_Translation);
         Vector3d outdatedPoseUpdatedInWorldFrame_Translation = new Vector3d();
         outdatedPoseUpdatedInWorldFrame.getPosition(outdatedPoseUpdatedInWorldFrame_Translation);
         
         FramePose testedPose = new FramePose(worldFrame);
         testedPose.setPose(outdatedPoseUpdatedInWorldFrame);
         testedPose.changeFrame(upToDateReferenceFrameInPresent);

         Vector3d testedTranslation = new Vector3d();
         testedTranslation.sub(outdatedPoseUpdatedInWorldFrame_Translation, upToDateReferenceFrameInPresent_Translation);
         Quat4d testedOrientation = new Quat4d();
         testedPose.getOrientation(testedOrientation);

         if (timeStamp < (int) (firstTimeStamp * 1.2 + numberOfTicksOfDelay))
         {
            assertTrue(testedOrientation.epsilonEquals(new Quat4d(0.0, 0.0, 0.0, 1.0), 1e-8));
            assertTrue(testedTranslation.epsilonEquals(new Vector3d(0.0, 0.0, 0.0), 1e-8));
         }
         else
         {
            assertTrue(testedOrientation.epsilonEquals(orientationOffsets[outdatedTimeStampsIndex - 1], 1e-8));
            assertTrue(testedTranslation.epsilonEquals(translationOffsets[outdatedTimeStampsIndex - 1], 1e-8));
         }
      }
   }

   private RigidBodyTransform generateOutdatedTransformWithTranslationAndOrientationOffset(TimeStampedTransformBuffer upToDateTimeStampedTransformPoseBuffer,
         long timeStamp, Quat4d orientationOffset, Vector3d translationOffset)
   {
      TimeStampedTransform3D upToDateTimeStampedTransformInPast = new TimeStampedTransform3D();
      upToDateTimeStampedTransformPoseBuffer.findTransform(timeStamp, upToDateTimeStampedTransformInPast);
      
      RigidBodyTransform upToDateTransformInPast_Translation = new RigidBodyTransform(upToDateTimeStampedTransformInPast.getTransform3D());
      RigidBodyTransform upToDateTransformInPast_Rotation = new RigidBodyTransform(upToDateTransformInPast_Translation);
      upToDateTransformInPast_Translation.setRotationToIdentity();
      upToDateTransformInPast_Rotation.zeroTranslation();
      
      RigidBodyTransform offsetRotationTransform = new RigidBodyTransform(orientationOffset, new Vector3d());
      RigidBodyTransform offsetTranslationTransform = new RigidBodyTransform(new Quat4d(), translationOffset);
      RigidBodyTransform transformedOutdatedTransform = new RigidBodyTransform();

      transformedOutdatedTransform.multiply(upToDateTransformInPast_Translation);
      transformedOutdatedTransform.multiply(offsetTranslationTransform);
      transformedOutdatedTransform.multiply(upToDateTransformInPast_Rotation);
      transformedOutdatedTransform.multiply(offsetRotationTransform);
      
      return transformedOutdatedTransform;
   }

   private RigidBodyTransform generateRandomUpToDateTransforms(Random random)
   {
      RigidBodyTransform upToDateTransform = new RigidBodyTransform();
      upToDateTransform.setTranslation(RandomTools.generateRandomVector(random));
      upToDateTransform.setRotation(RandomTools.generateRandomQuaternion(random));
      return upToDateTransform;
   }

}
