package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;

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
}
