package us.ihmc.commonWalkingControlModules.couplingRegistry;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Wrench;

public interface CouplingRegistry
{
   public abstract void setSingleSupportDuration(double singleSupportDuration);
   public abstract double getSingleSupportDuration();

   public abstract void setDoubleSupportDuration(double doubleSupportDuration);
   public abstract double getDoubleSupportDuration();

   public abstract void setDesiredVelocity(FrameVector2d desiredVelocity);
   public abstract FrameVector2d getDesiredVelocity();

   public abstract void setCaptureRegion(FrameConvexPolygon2d captureRegion);
   public abstract FrameConvexPolygon2d getCaptureRegion();

   public abstract void setCapturePoint(FramePoint capturePoint);
   public abstract FramePoint getCapturePointInFrame(ReferenceFrame desiredFrame);

   public abstract void setDesiredFootstep(Footstep desiredStepLocation);
   public abstract Footstep getDesiredFootstep();

   public abstract void setSupportLeg(RobotSide supportLeg);
   public abstract RobotSide getSupportLeg();

   public abstract void setEstimatedSwingTimeRemaining(double estimatedSwingTimeRemaining);
   public abstract double getEstimatedSwingTimeRemaining();

   public abstract void setBipedSupportPolygons(BipedSupportPolygons bipedSupportPolygons);
   public abstract BipedSupportPolygons getBipedSupportPolygons();

   public abstract void setForceHindOnToes(boolean forceHindOnToes);
   public abstract boolean getForceHindOnToes();

   public abstract void setUpperBodyWrench(Wrench upperBodyWrench);
   public abstract Wrench getUpperBodyWrench();
   
   // TODO: Calculate and set desired CoP in doEveryTick controller
   // The stance sub controller calculates the desired CoP now, and is executed after
   // the swing controller. The swing sub controller might use the desired CoP to estimate the
   // body position at the end of the swing phase. So now, it is using data that is one tick old.
   public abstract void setDesiredCoP(FramePoint2d desiredCoP);
   public abstract FramePoint2d getDesiredCoP();

   public abstract void setIsLunging(boolean isLunging);
   public abstract boolean getIsLunging();
   
}
