package us.ihmc.commonWalkingControlModules.kinematics;

import us.ihmc.utilities.math.geometry.FrameVector;

public interface BodyVelocityEstimator
{

   public abstract void packBodyVelocity(FrameVector bodyVelocityToPack);

   public abstract void estimateBodyVelocity();
}
