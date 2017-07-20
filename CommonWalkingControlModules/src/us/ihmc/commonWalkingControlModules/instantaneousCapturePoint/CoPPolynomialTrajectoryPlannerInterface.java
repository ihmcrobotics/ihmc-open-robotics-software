package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;

public interface CoPPolynomialTrajectoryPlannerInterface
{
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing);
   public void setInitialCoPPosition(FramePoint2d initialCoPPosition);
   public void setInitialCoPPosition(FramePoint initialCoPPosition);
   public void setInitialCoPVelocity(FrameVector2d intialCoPVelocity);
   public void setInitialCoPVelocity(FrameVector intialCoPVelocity);
   public void setInitialCoPAcceleration(FrameVector2d initialCoPAcceleration);
   public void setInitialCoPAcceleration(FrameVector initialCoPAcceleration);
   public void setFinalCoPVelocity(FrameVector finalCoPVelocity);
   public void setFinalCoPVelocity(FrameVector2d finalCoPVelocity);
}
