package us.ihmc.darpaRoboticsChallenge.posePlayback;

public interface PoseCheckerCallback
{
   public void checkPose(PlaybackPose pose, PlaybackPose previousPose);
}
