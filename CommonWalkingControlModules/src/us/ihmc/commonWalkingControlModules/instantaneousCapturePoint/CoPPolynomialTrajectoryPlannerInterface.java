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
}
