package us.ihmc.avatar.posePlayback;

import java.util.ArrayList;

import us.ihmc.robotModels.FullRobotModel;

public class PlaybackPoseSequence
{
   private final ArrayList<PlaybackPose> poseSequence = new ArrayList<PlaybackPose>();

   private final FullRobotModel fullRobotModel;

   public PlaybackPoseSequence(FullRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;
   }

   public void addPose(PlaybackPose playbackPose)
   {
      poseSequence.add(playbackPose);
   }

   public FullRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public ArrayList<PlaybackPose> getPoseSequence()
   {
      return poseSequence;
   }

   public PlaybackPose getPose(int index)
   {
      if (index < poseSequence.size())
         return poseSequence.get(index);
      else
         return poseSequence.get(poseSequence.size() - 1);
   }

   public PlaybackPose getFinalPose()
   {
      return getPose(getNumberOfPoses() - 1);
   }

   public int getNumberOfPoses()
   {
      return poseSequence.size();
   }

   public void clear()
   {
      poseSequence.clear();
   }

   public boolean epsilonEquals(PlaybackPoseSequence otherPoseSequence, double jointEpsilon, double timeEpsilon)
   {
      ArrayList<PlaybackPose> otherPoseArray = otherPoseSequence.getPoseSequence();

      if (otherPoseArray.size() != this.poseSequence.size())
         return false;

      for (int i = 0; i < poseSequence.size(); i++)
      {
         PlaybackPose thisPose = poseSequence.get(i);
         PlaybackPose otherPose = otherPoseArray.get(i);

         if (thisPose.epsilonEquals(otherPose, jointEpsilon, timeEpsilon))
            return false;
      }

      return true;
   }

   public PlaybackPoseSequence copy()
   {
      PlaybackPoseSequence playbackPoseSequence = new PlaybackPoseSequence(fullRobotModel);
      for (PlaybackPose pose : poseSequence)
      {
         playbackPoseSequence.addPose(pose.copy());
      }

      return playbackPoseSequence;
   }

   public int size()
   {
      return poseSequence.size();
   }


}
