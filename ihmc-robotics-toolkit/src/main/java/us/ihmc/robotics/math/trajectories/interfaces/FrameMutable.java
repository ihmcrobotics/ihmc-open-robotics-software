package us.ihmc.robotics.math.trajectories.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;

/**
 * TODO: this should be upgraded to modify the underlying frame
 */
public interface FrameMutable extends ReferenceFrameHolder
{
   void setReferenceFrame(ReferenceFrame referenceFrame);
}
