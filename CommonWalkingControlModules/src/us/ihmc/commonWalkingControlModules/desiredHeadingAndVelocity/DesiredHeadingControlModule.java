package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public interface DesiredHeadingControlModule
{  
   public abstract void updateDesiredHeadingFrame();

   public abstract ReferenceFrame getDesiredHeadingFrame();

   public abstract double getDesiredHeadingAngle();
   
   public abstract void getDesiredHeading(FrameVector2d desiredHeadingToPack);

   public abstract double getFinalHeadingTargetAngle();
   
   public abstract FrameVector2d getFinalHeadingTarget();

   public abstract void setFinalHeadingTargetAngle(double finalHeadingTargetAngle);
   
   public abstract void setFinalHeadingTarget(FrameVector2d finalHeadingTarget);

   public abstract void resetHeadingAngle(double newHeading);
}
