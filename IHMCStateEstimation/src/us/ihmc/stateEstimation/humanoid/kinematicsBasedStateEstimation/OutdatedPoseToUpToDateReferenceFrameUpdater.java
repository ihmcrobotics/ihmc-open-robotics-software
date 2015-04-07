package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import javax.vecmath.Vector3d;

import us.ihmc.communication.subscribers.TimeStampedTransformBuffer;
import us.ihmc.utilities.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
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

   // upToDate in present
   private final ReferenceFrame upToDateReferenceFrameInPresent;
   private final ReferenceFrame upToDateReferenceFrameInPresent_Translation;
   private final ReferenceFrame upToDateReferenceFrameInPresent_Rotation;

   // upToDate in the past
   private final TimeStampedTransform3D upToDateTimeStampedTransformInPast;
   private final FramePose upToDatePoseInThePast;
   private final PoseReferenceFrame upToDateReferenceFrameInThePast;
   private final FramePoint upToDatePositionInThePastInWorldFrame;

   // outdated in present
   private final ReferenceFrame outdatedReferenceFrameInPresent_Translation;
   private final ReferenceFrame outdatedReferenceFrameInPresent_Rotation;
   
   // outdated in the past
   private final FramePose outdatedPoseInThePast;
   private final PoseReferenceFrame outdatedReferenceFrameInThePast;
   private final RigidBodyTransform outdatedPoseTransformInThePast_InUpToDateReferenceFrameInThePast_Translation = new RigidBodyTransform();
   private final RigidBodyTransform outdatedPoseTransformInThePast_InUpToDateReferenceFrameInThePast_Rotation = new RigidBodyTransform();
   private final FramePoint outdatedPositionInThePastInWorldFrame;
   
   private final FrameVector translationOffsetFrameVector = new FrameVector(worldFrame);
   
   /**
    * Constructor
    * @param upToDateBufferSize is the size of the TimeStampedTransformBuffer that will be used to compare the outdated transforms
    * @param upToDateReferenceFrameInPresent is the ReferenceFrame in which the outdated pose will be expressed
    */
   public OutdatedPoseToUpToDateReferenceFrameUpdater(int upToDateBufferSize, ReferenceFrame upToDateReferenceFrameInPresent)
   {
      this.upToDateReferenceFrameInPresent = upToDateReferenceFrameInPresent;

      upToDateTimeStampedTransformBuffer = new TimeStampedTransformBuffer(upToDateBufferSize);
      upToDateTimeStampedTransformInPast = new TimeStampedTransform3D();
      upToDatePoseInThePast = new FramePose(worldFrame);
      outdatedPoseInThePast = new FramePose(worldFrame);
      
      upToDateReferenceFrameInThePast = new PoseReferenceFrame("upToDateReferenceFrameInThePast", upToDatePoseInThePast);
      upToDatePositionInThePastInWorldFrame = new FramePoint(upToDateReferenceFrameInThePast);
      outdatedReferenceFrameInThePast = new PoseReferenceFrame("upToDateReferenceFrameInThePast", outdatedPoseInThePast);
      outdatedPositionInThePastInWorldFrame = new FramePoint(outdatedReferenceFrameInThePast);
      
      ////////////////////////// In present ///////////
      upToDateReferenceFrameInPresent_Translation = new ReferenceFrame("upToDateReferenceFrameInPresent_Translation", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            OutdatedPoseToUpToDateReferenceFrameUpdater.this.upToDateReferenceFrameInPresent.getTransformToDesiredFrame(transformToParent, worldFrame);
            transformToParent.setRotationToIdentity();
         }
      };

      outdatedReferenceFrameInPresent_Translation = new ReferenceFrame("outdatedReferenceFrameInPresent_Translation", upToDateReferenceFrameInPresent_Translation)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(outdatedPoseTransformInThePast_InUpToDateReferenceFrameInThePast_Translation);
         }
         
         @Override
         public void update()
         {
            super.update();
            upToDateReferenceFrameInPresent_Translation.update();
         }
         
      };
      
      upToDateReferenceFrameInPresent_Rotation = new ReferenceFrame("upToDateReferenceFrameInPresent_Rotation", outdatedReferenceFrameInPresent_Translation)
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
            outdatedReferenceFrameInPresent_Translation.update();
         }
      };
      
      outdatedReferenceFrameInPresent_Rotation = new ReferenceFrame("outdatedReferenceFrameInPresent_Rotation", upToDateReferenceFrameInPresent_Rotation)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(outdatedPoseTransformInThePast_InUpToDateReferenceFrameInThePast_Rotation);
         }
         
         @Override
         public void update()
         {
            super.update();
            upToDateReferenceFrameInPresent_Rotation.update();
         }
      };
      ///////////////////////////////////////
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
      upToDatePoseInThePast.setPoseIncludingFrame(worldFrame, upToDateTimeStampedTransformInPast.getTransform3D());
      upToDateReferenceFrameInThePast.setPoseAndUpdate(upToDatePoseInThePast);
      
      //update the outdated Pose
      outdatedPoseInThePast.setPoseIncludingFrame(worldFrame, outdatedTimeStampedTransformInWorld.getTransform3D());
      outdatedReferenceFrameInThePast.setPoseAndUpdate(outdatedPoseInThePast);
      
      upToDatePositionInThePastInWorldFrame.setToZero(upToDateReferenceFrameInThePast);
      upToDatePositionInThePastInWorldFrame.changeFrame(worldFrame);
      
      outdatedPositionInThePastInWorldFrame.setToZero(outdatedReferenceFrameInThePast);
      outdatedPositionInThePastInWorldFrame.changeFrame(worldFrame);
      
      translationOffsetFrameVector.sub(outdatedPositionInThePastInWorldFrame, upToDatePositionInThePastInWorldFrame);

      Vector3d translationOffsetVector = new Vector3d();
      translationOffsetFrameVector.get(translationOffsetVector);
      
      outdatedPoseTransformInThePast_InUpToDateReferenceFrameInThePast_Translation.setTranslationAndIdentityRotation(translationOffsetVector);
      
      outdatedPoseInThePast.changeFrame(upToDateReferenceFrameInThePast);
      outdatedPoseInThePast.getPose(outdatedPoseTransformInThePast_InUpToDateReferenceFrameInThePast_Rotation);
      outdatedPoseTransformInThePast_InUpToDateReferenceFrameInThePast_Rotation.zeroTranslation();
      
      outdatedReferenceFrameInPresent_Rotation.update();
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
      return outdatedReferenceFrameInPresent_Rotation;
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
