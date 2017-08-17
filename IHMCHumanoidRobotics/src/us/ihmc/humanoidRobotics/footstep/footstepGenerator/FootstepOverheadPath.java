package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.robotics.geometry.FramePose2d;

/**
 * Created by agrabertilton on 2/19/15.
 */
public abstract class FootstepOverheadPath implements ReferenceFrameHolder
{
   public abstract FramePose2d getPoseAtDistance(double distanceAlongPath);

   public abstract double getTotalDistance();

   public abstract FootstepOverheadPath changeFrameCopy(ReferenceFrame desiredFrame);
}
