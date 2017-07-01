package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.FrameVector2d;

public interface DesiredHeadingControlModule
{  
   public abstract void updateDesiredHeadingFrame();

   public abstract ReferenceFrame getDesiredHeadingFrame();
   
   public abstract ReferenceFrame getPredictedHeadingFrame(double timeFromNow);

   public abstract double getDesiredHeadingAngle();
   
   public abstract void getDesiredHeading(FrameVector2d desiredHeadingToPack, double timeFromNow);

   public abstract double getFinalHeadingTargetAngle();
   
   public abstract FrameVector2d getFinalHeadingTarget();

   public abstract void setFinalHeadingTargetAngle(double finalHeadingTargetAngle);
   
   public abstract void setFinalHeadingTarget(FrameVector2d finalHeadingTarget);

   public abstract void resetHeadingAngle(double newHeading);
}
