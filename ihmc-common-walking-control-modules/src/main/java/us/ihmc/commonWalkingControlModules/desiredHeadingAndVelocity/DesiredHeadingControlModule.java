package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public interface DesiredHeadingControlModule
{  
   public abstract void updateDesiredHeadingFrame();

   public abstract ReferenceFrame getDesiredHeadingFrame();
   
   public abstract ReferenceFrame getPredictedHeadingFrame(double timeFromNow);

   public abstract double getDesiredHeadingAngle();
   
   public abstract void getDesiredHeading(FrameVector2D desiredHeadingToPack, double timeFromNow);

   public abstract double getFinalHeadingTargetAngle();
   
   public abstract FrameVector2D getFinalHeadingTarget();

   public abstract void setFinalHeadingTargetAngle(double finalHeadingTargetAngle);
   
   public abstract void setFinalHeadingTarget(FrameVector2D finalHeadingTarget);

   public abstract void resetHeadingAngle(double newHeading);
}
