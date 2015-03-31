package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.communication.subscribers.TimeStampedTransformBuffer;
import us.ihmc.utilities.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;

/**
 * @author Nicolas EYSSETTE
 * 
 * The aim of this class is to express an outdated transform in an updated reference.
 * It will compare the outdated transform to the updated transform at the same timeStamp in the past,
 * and express the result in the up to date referenceFrame in the present.
 *
 */

public class OutdatedPoseToUpToDateReferenceFrameUpdater
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final TimeStampedTransformBuffer upToDateTimeStampedTransformBuffer;
   private final ReferenceFrame upToDateReferenceFrameInPresent;
   private final ReferenceFrame upToDateReferenceFrameInPresent_Translation;
   private final ReferenceFrame upToDateReferenceFrameInPresent_Rotation;

   private final TimeStampedTransform3D upToDateTimeStampedTransformInPast;
   private final FramePose upToDatePoseInPast;
   
   private final FramePose outdatedPose;
   private final RigidBodyTransform outdatedTransform_InUpToDateReferenceFrameInPast = new RigidBodyTransform();
   private final ReferenceFrame outdatedReferenceFrame_Translation;
   private final ReferenceFrame outdatedReferenceFrame_Rotation;
   
   private final ReferenceFrame upToDateReferenceFrameInThePast_Translation;
   private final ReferenceFrame upToDateReferenceFrameInThePast_Rotation;
   
   
   
   /**
    * Constructor
    * @param upToDateBufferSize is the size of the TimeStampedTransformBuffer that will be used to compare the outdated transforms
    * @param upToDateReferenceFrameInPresent is the ReferenceFrame in which the outdated pose will be expressed
    */
   public OutdatedPoseToUpToDateReferenceFrameUpdater(int upToDateBufferSize, ReferenceFrame upToDateReferenceFrameInPresent)
   {
      this.upToDateReferenceFrameInPresent = upToDateReferenceFrameInPresent;

      
      /////////////////////////////// upToDate ReferenceFrame In Present
      upToDateReferenceFrameInPresent_Translation = new ReferenceFrame("upToDateReferenceFrameInPresent_Translation", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            OutdatedPoseToUpToDateReferenceFrameUpdater.this.upToDateReferenceFrameInPresent.getTransformToDesiredFrame(transformToParent, worldFrame);
            transformToParent.setRotationToIdentity();
         }
      };
      
      upToDateReferenceFrameInPresent_Rotation = new ReferenceFrame("upToDateReferenceFrameInPresent_Rotation", upToDateReferenceFrameInPresent_Translation)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            OutdatedPoseToUpToDateReferenceFrameUpdater.this.upToDateReferenceFrameInPresent.getTransformToDesiredFrame(transformToParent, worldFrame);
            transformToParent.zeroTranslation();
         }

         @Override
         public void update()
         {
            super.update();
            upToDateReferenceFrameInPresent_Translation.update();
         }
      };
      //////////////////////////////////////////////////////////////////
      
      upToDateTimeStampedTransformBuffer = new TimeStampedTransformBuffer(upToDateBufferSize);

      upToDateTimeStampedTransformInPast = new TimeStampedTransform3D();
      upToDatePoseInPast = new FramePose(worldFrame);

      
      //////////////////// upToDate ReferenceFrame In Past
      upToDateReferenceFrameInThePast_Translation = new ReferenceFrame("upToDateReferenceFrameInThePast_Translation", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            upToDatePoseInPast.getPose(transformToParent);
            transformToParent.setRotationToIdentity();
            
         }
      };
      
      upToDateReferenceFrameInThePast_Rotation = new ReferenceFrame("upToDateReferenceFrameInThePast_Rotation", upToDateReferenceFrameInThePast_Translation)
      {
         
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            upToDatePoseInPast.getPose(transformToParent);
            transformToParent.zeroTranslation();
         }
         
         @Override
         public void update()
         {
            super.update();
            upToDateReferenceFrameInThePast_Translation.update();
         }
      };
      //////////////////////////////////////////////////
      
      ////////////////////// outdated Pose reference frame
      outdatedPose = new FramePose(worldFrame);
      outdatedReferenceFrame_Translation = new ReferenceFrame("outdatedReferenceFrame_Translation", upToDateReferenceFrameInPresent_Rotation)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(outdatedTransform_InUpToDateReferenceFrameInPast);
            transformToParent.setRotationToIdentity();
         }
         
         @Override
         public void update()
         {
            super.update();
            upToDateReferenceFrameInPresent_Rotation.update();
         }
         
      };
      
      outdatedReferenceFrame_Rotation = new ReferenceFrame("outdatedReferenceFrame_Rotation", outdatedReferenceFrame_Translation)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(outdatedTransform_InUpToDateReferenceFrameInPast);
            transformToParent.zeroTranslation();
         }
         
         @Override
         public void update()
         {
            super.update();
            outdatedReferenceFrame_Translation.update();
         }
      };
      ///////////////////////////////////
   }

   /**
    * Compare the outdatedTimeStampedTransform with the upToDateTimeStampedTransform having the same timeStamp
    * @param outdatedTimeStampedTransformInWorld
    * @param outdatedPoseToBeUpdatedToPack
    */
   public void updateOutdatedTransform(TimeStampedTransform3D outdatedTimeStampedTransformInWorld)
   {
      //update the upToDate reference frame in the past
      upToDateTimeStampedTransformBuffer.findTransform(outdatedTimeStampedTransformInWorld.getTimeStamp(), upToDateTimeStampedTransformInPast);
      upToDatePoseInPast.setPoseIncludingFrame(worldFrame, upToDateTimeStampedTransformInPast.getTransform3D());
      upToDateReferenceFrameInThePast_Rotation.update();
      
      //update the outdated Pose
      outdatedPose.setPoseIncludingFrame(worldFrame, outdatedTimeStampedTransformInWorld.getTransform3D());
      outdatedPose.changeFrame(upToDateReferenceFrameInThePast_Rotation);
      outdatedPose.getPose(outdatedTransform_InUpToDateReferenceFrameInPast);
      outdatedReferenceFrame_Rotation.update();
   }

   /**
    * Puts the upToDateTransform in the buffer with the corresponding timeStamp
    * @param upToDateTransform 
    * @param timeStamp
    */
   public void putUpToDateTransformInBuffer(RigidBodyTransform upToDateTransform, long timeStamp)
   {
      upToDateTimeStampedTransformBuffer.put(upToDateTransform, timeStamp);
   }
   
   /**
    * @param timeStamp
    * @return true if the timeStamp is in range of the upToDateTimeStampedBuffer
    */
   public boolean upToDateTimeStampedBufferIsInRange(long timeStamp)
   {
      return upToDateTimeStampedTransformBuffer.isInRange(timeStamp);
   }
   
   public ReferenceFrame getOutdatedReferenceFrameToBeUpdated()
   {
      return outdatedReferenceFrame_Rotation;
   }
   
   public long getUpToDateTimeStampedBufferNewestTimestamp()
   {
      return upToDateTimeStampedTransformBuffer.getNewestTimestamp();
   }
   
   public long getUpToDateTimeStampedBufferOldestTimestamp()
   {
      return upToDateTimeStampedTransformBuffer.getOldestTimestamp();
   }
}
