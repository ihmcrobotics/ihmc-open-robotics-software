package us.ihmc.avatar.posePlayback;

import java.util.Map;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public interface PosePlaybackPacket
{
   public abstract Map<OneDoFJointBasics, Double> getJointKps();
   public abstract Map<OneDoFJointBasics, Double> getJointKds();
   
   public abstract PlaybackPoseSequence getPlaybackPoseSequence();
   
   public abstract double getInitialGainScaling();
}
