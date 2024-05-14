package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.input;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;

/**
 * Interface for the state estimation of the different end-effectors desired state controlled by the user inputs.
 * <p>
 * The state estimator is responsible for estimating the pose and velocity of the end-effectors based on the user inputs.
 * It is responsible for smoothing the user inputs.
 * </p>
 */
public interface KSTInputStateEstimator
{
   void reset();

   void update(double time,
               boolean isNewInput,
               KinematicsStreamingToolboxInputCommand latestInputCommand,
               KinematicsStreamingToolboxInputCommand previousRawInputCommand);

   FramePose3DReadOnly getEstimatedPose(RigidBodyReadOnly endEffector);

   default SpatialVectorReadOnly getEstimatedVelocity(RigidBodyReadOnly endEffector)
   {
      return null;
   }

   default FramePoint3DReadOnly getEstimatedCoMPosition()
   {
      return null;
   }

   default FrameVector3DReadOnly getEstimatedCoMVelocity()
   {
      return null;
   }
}
