package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public interface DesiredHeadingControlModule
{  
   public abstract void updateDesiredHeadingFrame();

   public abstract ReferenceFrame getDesiredHeadingFrame();

   public abstract FrameVector getFinalHeadingTarget();

   public abstract void setFinalDesiredHeading(double desiredHeading);

   public abstract double getDesiredHeading();
   
   public abstract void resetHeading(double newHeading);
}
