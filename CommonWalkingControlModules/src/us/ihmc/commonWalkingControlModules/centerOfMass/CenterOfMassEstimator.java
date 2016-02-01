package us.ihmc.commonWalkingControlModules.centerOfMass;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface CenterOfMassEstimator
{
   public abstract FramePoint getCenterOfMassInFrame(ReferenceFrame frame);
}
