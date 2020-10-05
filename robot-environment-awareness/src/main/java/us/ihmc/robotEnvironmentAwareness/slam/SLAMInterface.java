package us.ihmc.robotEnvironmentAwareness.slam;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public interface SLAMInterface
{
   abstract void addKeyFrame(StereoVisionPointCloudMessage pointCloudMessage, boolean insertMiss);

   abstract boolean addFrame(StereoVisionPointCloudMessage pointCloudMessage, boolean insertMiss);

   abstract void clear();

   /**
    * if this frame is detected as a key frame, return new RigidBodyTransform(); if this frame needs
    * drift correction, return optimized transform; if this frame should not be mergeable, return null;
    */
   default RigidBodyTransformReadOnly computeFrameCorrectionTransformer(SLAMFrame frame)
   {
      return new RigidBodyTransform();
   }
}
