package us.ihmc.commonWalkingControlModules.kinematics;

import us.ihmc.utilities.math.geometry.FramePoint;

public interface BodyPositionEstimator
{
   public abstract void packBodyPosition(FramePoint bodyPositionToPack);

   public abstract void estimateBodyPosition();
}
