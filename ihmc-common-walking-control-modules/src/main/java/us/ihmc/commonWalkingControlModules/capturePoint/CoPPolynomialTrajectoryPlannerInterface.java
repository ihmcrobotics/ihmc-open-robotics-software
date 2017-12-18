package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;

public interface CoPPolynomialTrajectoryPlannerInterface
{
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing);
}
