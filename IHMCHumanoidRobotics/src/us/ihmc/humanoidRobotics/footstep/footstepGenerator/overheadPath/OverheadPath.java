package us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.robotics.geometry.FramePose2d;


public abstract class OverheadPath implements ReferenceFrameHolder
{
   public abstract FramePose2d getPoseAtS(double pathVariableS);

   public abstract FramePose2d getExtrapolatedPoseAtS(double pathVariableS);

   public abstract OverheadPath changeFrameCopy(ReferenceFrame desiredFrame);
}
