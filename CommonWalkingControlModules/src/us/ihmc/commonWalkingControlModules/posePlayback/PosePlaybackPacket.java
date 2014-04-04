package us.ihmc.commonWalkingControlModules.posePlayback;

import java.util.Map;

import us.ihmc.utilities.screwTheory.OneDoFJoint;

public interface PosePlaybackPacket
{
   public abstract Map<OneDoFJoint, Double> getJointKps();
   public abstract Map<OneDoFJoint, Double> getJointKds();
   
   public abstract PlaybackPoseSequence getPlaybackPoseSequence();
   
   public abstract double getInitialGainScaling();
}
