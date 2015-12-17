package us.ihmc.darpaRoboticsChallenge.posePlayback;

import us.ihmc.commonWalkingControlModules.posePlayback.PlaybackPose;

public interface PoseCheckerCallback
{
   public void checkPose(PlaybackPose pose, PlaybackPose previousPose);
}
