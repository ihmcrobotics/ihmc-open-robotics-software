package us.ihmc.humanoidRobotics.footstep.footsepGenerator.overheadPath;

import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public abstract class OverheadPath extends ReferenceFrameHolder
{
   public abstract FramePose2d getPoseAtS(double pathVariableS);

   public abstract FramePose2d getExtrapolatedPoseAtS(double pathVariableS);

   public abstract OverheadPath changeFrameCopy(ReferenceFrame desiredFrame);
}
