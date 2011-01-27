package us.ihmc.commonWalkingControlModules.centerOfMass;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public interface CenterOfMassEstimator
{
   public abstract FramePoint getCenterOfMassInFrame(ReferenceFrame frame);
}
