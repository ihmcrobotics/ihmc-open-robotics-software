package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ExtendedCapturePointPlannerParameters;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.math.trajectories.YoPolynomial;

public interface CMPComponentPolynomialTrajectoryPlannerInterface
{
   public CMPComponentType getComponentType();
   public void initializeParameters(ExtendedCapturePointPlannerParameters icpPlannerParameters);
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing);
   public List<YoPolynomial> getPolynomialTrajectory();
}
