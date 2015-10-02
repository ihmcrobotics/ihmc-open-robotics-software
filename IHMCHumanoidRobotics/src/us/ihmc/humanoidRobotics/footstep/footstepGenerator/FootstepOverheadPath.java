package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * Created by agrabertilton on 2/19/15.
 */
public abstract class FootstepOverheadPath extends ReferenceFrameHolder
{
   public abstract FramePose2d getPoseAtDistance(double distanceAlongPath);

   public abstract double getTotalDistance();

   public abstract FootstepOverheadPath changeFrameCopy(ReferenceFrame desiredFrame);
}
