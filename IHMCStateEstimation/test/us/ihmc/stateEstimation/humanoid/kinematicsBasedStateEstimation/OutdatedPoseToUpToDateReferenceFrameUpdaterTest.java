package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.communication.subscribers.TimeStampedTransformBuffer;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;

public class OutdatedPoseToUpToDateReferenceFrameUpdaterTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final int numberOfUpToDateTransforms = 10;
   private final int numberOfOutdatedTransforms = 3;
   private final long firstTimeStamp = 1000;
   private final long lastTimeStamp = 2000;
   private final int numberOfTicksOfDelay = 100;

   @AverageDuration(duration = 0.1)
   @Test(timeout = 300000)
   public void testUpdateOutdatedTransformWithKnownOffsets()
   {
      Random random = new Random(1987L);

      Vector3d[] translationOffsets = new Vector3d[numberOfOutdatedTransforms];
      Quat4d[] orientationOffsets = new Quat4d[numberOfOutdatedTransforms];
      long[] outdatedTimeStamps = new long[numberOfOutdatedTransforms];

      FramePose upToDatePoseInPresent = new FramePose(worldFrame);
      PoseReferenceFrame upToDateReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", upToDatePoseInPresent);
      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(
            numberOfUpToDateTransforms, upToDateReferenceFrameInPresent);

      TimeStampedTransformBuffer upToDateTimeStampedTransformPoseBuffer = new TimeStampedTransformBuffer(numberOfUpToDateTransforms);
      TimeStampedTransformBuffer outdatedTimeStampedTransformBuffer = new TimeStampedTransformBuffer(numberOfOutdatedTransforms);

      for (int i = 0; i < numberOfUpToDateTransforms; i++)
      {
         long timeStamp = (long) (i * (lastTimeStamp - firstTimeStamp) / numberOfUpToDateTransforms + firstTimeStamp);
         RigidBodyTransform upToDateTransform = generateRandomUpToDateTransforms(random);
         upToDateTimeStampedTransformPoseBuffer.put(upToDateTransform, timeStamp);
         outdatedPoseToUpToDateReferenceFrameUpdater.putUpToDateTransformInBuffer(upToDateTransform, timeStamp);
      }

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

      FramePose lastUpdatedOfOutdatedPoseInPresent = new FramePose(upToDateReferenceFrameInPresent);
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
               lastUpdatedOfOutdatedPoseInPresent.changeFrame(upToDateReferenceFrameInPresent);
               outdatedPoseToUpToDateReferenceFrameUpdater.updateOutdatedTransform(outdatedTimeStampedTransform, lastUpdatedOfOutdatedPoseInPresent);
               outdatedTimeStampsIndex++;
            }
         }
         FramePose outdatedPoseExpressedInUpToDateReferenceFrame = new FramePose(upToDateReferenceFrameInPresent);
         outdatedPoseExpressedInUpToDateReferenceFrame.setPose(lastUpdatedOfOutdatedPoseInPresent);
         outdatedPoseExpressedInUpToDateReferenceFrame.changeFrame(worldFrame);

         FramePose testedPose = new FramePose(worldFrame);
         testedPose.setPose(outdatedPoseExpressedInUpToDateReferenceFrame);
         testedPose.changeFrame(upToDateReferenceFrameInPresent);

         Quat4d testedOrientation = new Quat4d();
         Tuple3d testedTranslation = new Vector3d();
         testedPose.getOrientation(testedOrientation);
         testedPose.getPosition(testedTranslation);

         System.out.println(outdatedTimeStampsIndex);

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
      RigidBodyTransform errorTransform = new RigidBodyTransform(orientationOffset, translationOffset);

      upToDateTimeStampedTransformPoseBuffer.findTransform(timeStamp, upToDateTimeStampedTransformInPast);

      RigidBodyTransform upToDateTransformInPast = upToDateTimeStampedTransformInPast.getTransform3D();
      FramePose upToDatePoseInPast = new FramePose(worldFrame, upToDateTransformInPast);
      PoseReferenceFrame currentOutdatedPoseReferenceFrame = new PoseReferenceFrame("currentOutdatedPoseReferenceFrame", upToDatePoseInPast);
      FramePose transformedOutdatedPose = new FramePose(currentOutdatedPoseReferenceFrame);
      transformedOutdatedPose.setPose(errorTransform);
      transformedOutdatedPose.changeFrame(worldFrame);

      RigidBodyTransform transformedOutdatedTransform = new RigidBodyTransform();
      transformedOutdatedPose.getPose(transformedOutdatedTransform);

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
