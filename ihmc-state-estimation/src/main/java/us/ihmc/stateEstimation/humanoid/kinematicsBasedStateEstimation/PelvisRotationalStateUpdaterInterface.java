package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public interface PelvisRotationalStateUpdaterInterface
{
   void initialize();

   void updateRootJointOrientationAndAngularVelocity();

   FrameOrientation3DReadOnly getEstimatedOrientation();

   FrameVector3DReadOnly getEstimatedAngularVelocity();

   /**
    * Returns the estimated yaw angle of the pelvis. The yaw angle is the angle around the z-axis of the pelvis frame.
    * <p>
    * This method is expected to return a yaw angle that is <b>not</b> clamped to the range [-<i>pi</i>, <i>pi</i>], such that the winding of the robot can be
    * tracked over time.
    * </p>
    *
    * @return the estimated yaw angle.
    */
   default double getEstimatedUnclampedYaw()
   {
      return getEstimatedOrientation().getYaw();
   }
}