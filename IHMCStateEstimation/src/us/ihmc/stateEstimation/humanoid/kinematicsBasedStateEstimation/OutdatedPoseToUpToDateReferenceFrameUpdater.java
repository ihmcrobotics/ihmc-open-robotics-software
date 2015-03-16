package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.communication.subscribers.TimeStampedTransformBuffer;
import us.ihmc.utilities.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.math.geometry.FramePose;
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
   private final ReferenceFrame upToDateReferenceFrameInPresent;

   private final TimeStampedTransform3D upToDateTimeStampedTransformInPast;
   private final FramePose upToDatePoseInPast;
   private final PoseReferenceFrame upToDateReferenceFrameInPast;

   private final FramePose outdatedPose;
   private final RigidBodyTransform outdatedTransform_upToDateReferenceFrameInPast = new RigidBodyTransform();

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
      upToDatePoseInPast = new FramePose(worldFrame);
      upToDateReferenceFrameInPast = new PoseReferenceFrame("upToDateReferenceFrameInPast", upToDatePoseInPast);

      outdatedPose = new FramePose(worldFrame);
   }

   /**
    * Compare the outdatedTimeStampedTransform with the upToDateTimeStampedTransform having the same timeStamp
    * @param outdatedTimeStampedTransformInWorld
    * @param outdatedPoseToBeUpdatedToPack
    */
   public void updateOutdatedTransform(TimeStampedTransform3D outdatedTimeStampedTransformInWorld, FramePose outdatedPoseToBeUpdatedToPack)
   {
      outdatedPoseToBeUpdatedToPack.checkReferenceFrameMatch(upToDateReferenceFrameInPresent);

      outdatedPose.setPoseIncludingFrame(worldFrame, outdatedTimeStampedTransformInWorld.getTransform3D());
      upToDateTimeStampedTransformBuffer.findTransform(outdatedTimeStampedTransformInWorld.getTimeStamp(), upToDateTimeStampedTransformInPast);
      upToDatePoseInPast.setPoseIncludingFrame(worldFrame, upToDateTimeStampedTransformInPast.getTransform3D());
      upToDateReferenceFrameInPast.setPoseAndUpdate(upToDatePoseInPast);
      outdatedPose.changeFrame(upToDateReferenceFrameInPast);
      outdatedPose.getRigidBodyTransform(outdatedTransform_upToDateReferenceFrameInPast);

      outdatedPoseToBeUpdatedToPack.setPoseIncludingFrame(upToDateReferenceFrameInPresent, outdatedTransform_upToDateReferenceFrameInPast);
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
   
   public long getUpToDateTimeStampedBufferNewestTimestamp()
   {
      return upToDateTimeStampedTransformBuffer.getNewestTimestamp();
   }
}
