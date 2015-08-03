package us.ihmc.commonWalkingControlModules.couplingRegistry;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;

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

   public abstract OldBipedSupportPolygons getOldBipedSupportPolygons();

   public abstract void setForceHindOnToes(boolean forceHindOnToes);
   public abstract boolean getForceHindOnToes();

   public abstract void setDesiredUpperBodyWrench(Wrench upperBodyWrench);
   public abstract Wrench getDesiredUpperBodyWrench();
   
   public abstract void setActualUpperBodyLungingWrench(Wrench wrenchOnPelvis);
   public abstract Wrench getActualUpperBodyLungingWrench();
   
   // TODO: Calculate and set desired CoP in doEveryTick controller
   // The stance sub controller calculates the desired CoP now, and is executed after
   // the swing controller. The swing sub controller might use the desired CoP to estimate the
   // body position at the end of the swing phase. So now, it is using data that is one tick old.
   public abstract void setDesiredCoP(FramePoint2d desiredCoP);
   public abstract FramePoint2d getDesiredCoP();
   
   public abstract void setLungeAxis(FrameVector2d lungeAxis);
   // returns null if not lunging
   public abstract FrameVector2d getLungeAxisInFrame(ReferenceFrame expressedInFrame);
   public abstract double getEstimatedDoubleSupportTimeRemaining();
   public abstract void setEstimatedDoubleSupportTimeRemaining(double estimatedDoubleSupportTimeRemaining);
   
   public abstract void setDesiredCMP(FramePoint2d desiredCMP);
   public abstract FramePoint2d getDesiredCMP();
   
   public abstract void setDesiredCapturePoint(FramePoint2d desiredCapturePoint);
   public abstract FramePoint2d getDesiredCapturePointInFrame(ReferenceFrame desiredFrame);
   
   public abstract void setUpcomingSupportLeg(RobotSide upcomingSupportLeg);
   public abstract RobotSide getUpcomingSupportLeg();
}
