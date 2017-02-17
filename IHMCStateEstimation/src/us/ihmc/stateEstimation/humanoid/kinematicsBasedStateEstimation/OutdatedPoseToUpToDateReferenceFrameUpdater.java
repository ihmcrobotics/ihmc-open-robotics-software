package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.subscribers.TimeStampedTransformBuffer;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * @author Nicolas EYSSETTE
 * 
 * The aim of this class is to express the outdated localization transform in the updated stateEstimator reference.
 * It will compare the outdated transform to the updated transform at the same timeStamp in the past,
 * and express the result in the up to date referenceFrame in the present.
 *
 */

public class OutdatedPoseToUpToDateReferenceFrameUpdater
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final TimeStampedTransformBuffer stateEstimatorTimeStampedTransformBuffer;

   // upToDate in present
   private final ReferenceFrame stateEstimatorReferenceFrameInPresent;
   private final ReferenceFrame stateEstimatorReferenceFrameInPresent_Translation;
   private final ReferenceFrame stateEstimatorReferenceFrameInPresent_Rotation;

   // upToDate in the past
   private final TimeStampedTransform3D stateEstimatorTimeStampedTransformInPast;
   private final FramePose stateEstimatorPoseInThePast;
   private final PoseReferenceFrame stateEstimatorReferenceFrameInThePast;
   private final FramePoint stateEstimatorPositionInThePastInWorldFrame;

   // outdated in present
   private final ReferenceFrame localizationReferenceFrameInPresent_Translation;
   private final ReferenceFrame localizationReferenceFrameInPresent_Rotation;

   // outdated in the past
   private final FramePose localizationPoseInThePast;
   private final PoseReferenceFrame localizationReferenceFrameInThePast;
   private final RigidBodyTransform localizationPoseTransformInThePast_InStateEstimatorReferenceFrameInThePast_Translation = new RigidBodyTransform();
   private final RigidBodyTransform localizationPoseTransformInThePast_InStateEstimatorReferenceFrameInThePast_Rotation = new RigidBodyTransform();
   private final FramePoint localizationPositionInThePastInWorldFrame;

   private final FrameVector translationOffsetFrameVector = new FrameVector(worldFrame);
   private final Vector3D translationOffsetVector = new Vector3D();

   private final RigidBodyTransform totalErrorTransform = new RigidBodyTransform();

   /**
    * Constructor
    * @param stateEstimatorBufferSize is the size of the TimeStampedTransformBuffer that will be used to compare the outdated transforms
    * @param stateEsimatorReferenceFrameInPresent is the ReferenceFrame in which the outdated pose will be expressed
    */
   public OutdatedPoseToUpToDateReferenceFrameUpdater(int stateEstimatorBufferSize, ReferenceFrame stateEsimatorReferenceFrameInPresent)
   {
      this.stateEstimatorReferenceFrameInPresent = stateEsimatorReferenceFrameInPresent;

      stateEstimatorTimeStampedTransformBuffer = new TimeStampedTransformBuffer(stateEstimatorBufferSize);
      stateEstimatorTimeStampedTransformInPast = new TimeStampedTransform3D();
      stateEstimatorPoseInThePast = new FramePose(worldFrame);
      localizationPoseInThePast = new FramePose(worldFrame);

      stateEstimatorReferenceFrameInThePast = new PoseReferenceFrame("upToDateReferenceFrameInThePast", stateEstimatorPoseInThePast);
      stateEstimatorPositionInThePastInWorldFrame = new FramePoint(stateEstimatorReferenceFrameInThePast);
      localizationReferenceFrameInThePast = new PoseReferenceFrame("upToDateReferenceFrameInThePast", localizationPoseInThePast);
      localizationPositionInThePastInWorldFrame = new FramePoint(localizationReferenceFrameInThePast);

      ////////////////////////// In present ///////////
      stateEstimatorReferenceFrameInPresent_Translation = new ReferenceFrame("stateEstimatorReferenceFrameInPresent_Translation", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            OutdatedPoseToUpToDateReferenceFrameUpdater.this.stateEstimatorReferenceFrameInPresent.getTransformToDesiredFrame(transformToParent, worldFrame);
            transformToParent.setRotationToZero();
         }
      };

      localizationReferenceFrameInPresent_Translation = new ReferenceFrame("localizationReferenceFrameInPresent_Translation",
            stateEstimatorReferenceFrameInPresent_Translation)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(localizationPoseTransformInThePast_InStateEstimatorReferenceFrameInThePast_Translation);
         }

         @Override
         public void update()
         {
            super.update();
            stateEstimatorReferenceFrameInPresent_Translation.update();
         }

      };

      stateEstimatorReferenceFrameInPresent_Rotation = new ReferenceFrame("stateEstimatorReferenceFrameInPresent_Rotation",
            localizationReferenceFrameInPresent_Translation)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            OutdatedPoseToUpToDateReferenceFrameUpdater.this.stateEstimatorReferenceFrameInPresent.getTransformToDesiredFrame(transformToParent, worldFrame);
            transformToParent.setTranslationToZero();
         }

         @Override
         public void update()
         {
            super.update();
            localizationReferenceFrameInPresent_Translation.update();
         }
      };

      localizationReferenceFrameInPresent_Rotation = new ReferenceFrame("localizationReferenceFrameInPresent_Rotation", stateEstimatorReferenceFrameInPresent_Rotation)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(localizationPoseTransformInThePast_InStateEstimatorReferenceFrameInThePast_Rotation);
         }

         @Override
         public void update()
         {
            super.update();
            stateEstimatorReferenceFrameInPresent_Rotation.update();
         }
      };
      ///////////////////////////////////////
   }

   /**
    * Compare the localizationTimeStampedTransform with the stateEstimatorTimeStampedTransform having the same timeStamp
    * @param localizationTimeStampedTransformInWorld
    */
   public void updateLocalizationTransform(TimeStampedTransform3D localizationTimeStampedTransformInWorld)
   {
      //update the stateEstimator reference frame in the past
      stateEstimatorTimeStampedTransformBuffer.findTransform(localizationTimeStampedTransformInWorld.getTimeStamp(), stateEstimatorTimeStampedTransformInPast);
      stateEstimatorPoseInThePast.setPoseIncludingFrame(worldFrame, stateEstimatorTimeStampedTransformInPast.getTransform3D());
      stateEstimatorReferenceFrameInThePast.setPoseAndUpdate(stateEstimatorPoseInThePast);

      //update the localization Pose
      localizationPoseInThePast.setPoseIncludingFrame(worldFrame, localizationTimeStampedTransformInWorld.getTransform3D());
      localizationReferenceFrameInThePast.setPoseAndUpdate(localizationPoseInThePast);

      stateEstimatorPositionInThePastInWorldFrame.setToZero(stateEstimatorReferenceFrameInThePast);
      stateEstimatorPositionInThePastInWorldFrame.changeFrame(worldFrame);

      localizationPositionInThePastInWorldFrame.setToZero(localizationReferenceFrameInThePast);
      localizationPositionInThePastInWorldFrame.changeFrame(worldFrame);

      translationOffsetFrameVector.sub(localizationPositionInThePastInWorldFrame, stateEstimatorPositionInThePastInWorldFrame);
      translationOffsetFrameVector.get(translationOffsetVector);

      localizationPoseTransformInThePast_InStateEstimatorReferenceFrameInThePast_Translation.setTranslationAndIdentityRotation(translationOffsetVector);

      localizationPoseInThePast.changeFrame(stateEstimatorReferenceFrameInThePast);
      localizationPoseInThePast.getPose(localizationPoseTransformInThePast_InStateEstimatorReferenceFrameInThePast_Rotation);
      localizationPoseTransformInThePast_InStateEstimatorReferenceFrameInThePast_Rotation.setTranslationToZero();

      totalErrorTransform.set(localizationPoseTransformInThePast_InStateEstimatorReferenceFrameInThePast_Rotation);
      totalErrorTransform.setTranslation(translationOffsetVector);

      localizationReferenceFrameInPresent_Rotation.update();
   }

   /**
    * Puts the stateEstimatorTransform in the buffer with the corresponding timeStamp
    * @param stateEstimatorTransform 
    * @param timeStamp
    */
   public void putStateEstimatorTransformInBuffer(RigidBodyTransform stateEstimatorTransform, long timeStamp)
   {
      stateEstimatorTimeStampedTransformBuffer.put(stateEstimatorTransform, timeStamp);
   }
   
   public void getStateEstimatorTransform(long timestamp, TimeStampedTransform3D timeStampedTransform3DToPack)
   {
      stateEstimatorTimeStampedTransformBuffer.findTransform(timestamp, timeStampedTransform3DToPack);
   }

   /**
    * @param timeStamp
    * @return true if the timeStamp is in range of the stateEstimatorTimeStampedBuffer
    */
   public boolean stateEstimatorTimeStampedBufferIsInRange(long timeStamp)
   {
      return stateEstimatorTimeStampedTransformBuffer.isInRange(timeStamp);
   }

   public ReferenceFrame getLocalizationReferenceFrameToBeUpdated()
   {
      return localizationReferenceFrameInPresent_Rotation;
   }

   public long getStateEstimatorTimeStampedBufferNewestTimestamp()
   {
      return stateEstimatorTimeStampedTransformBuffer.getNewestTimestamp();
   }

   public long getStateEstimatorTimeStampedBufferOldestTimestamp()
   {
      return stateEstimatorTimeStampedTransformBuffer.getOldestTimestamp();
   }

   public void getTotalErrorTransform(RigidBodyTransform rigidBodyTransformToPack)
   {
      rigidBodyTransformToPack.set(totalErrorTransform);
   }
}
