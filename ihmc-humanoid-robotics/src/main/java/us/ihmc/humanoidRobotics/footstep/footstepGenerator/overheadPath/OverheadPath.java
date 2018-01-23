package us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath;

import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;


public abstract class OverheadPath implements ReferenceFrameHolder
{
   public abstract FramePose2D getPoseAtS(double pathVariableS);

   public abstract FramePose2D getExtrapolatedPoseAtS(double pathVariableS);

   public abstract OverheadPath changeFrameCopy(ReferenceFrame desiredFrame);
}
