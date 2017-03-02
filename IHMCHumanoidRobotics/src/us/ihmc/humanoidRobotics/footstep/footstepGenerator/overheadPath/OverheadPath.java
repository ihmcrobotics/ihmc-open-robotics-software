package us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath;

import us.ihmc.robotics.geometry.AbstractReferenceFrameHolder;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public abstract class OverheadPath extends AbstractReferenceFrameHolder
{
   public abstract FramePose2d getPoseAtS(double pathVariableS);

   public abstract FramePose2d getExtrapolatedPoseAtS(double pathVariableS);

   public abstract OverheadPath changeFrameCopy(ReferenceFrame desiredFrame);
}
