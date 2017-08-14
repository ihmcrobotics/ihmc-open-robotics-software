package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.FramePoint3D;
import us.ihmc.robotics.geometry.FramePoint2D;
import us.ihmc.robotics.geometry.FrameVector3D;
import us.ihmc.robotics.geometry.FrameVector2D;

public interface CoPPolynomialTrajectoryPlannerInterface
{
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing);
}
