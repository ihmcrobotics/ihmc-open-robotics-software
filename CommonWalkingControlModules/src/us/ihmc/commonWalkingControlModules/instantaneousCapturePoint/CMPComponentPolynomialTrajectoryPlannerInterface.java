package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.ExtendedCapturePointPlannerParameters;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.trajectories.YoPolynomial3D;
import us.ihmc.robotics.robotSide.SideDependentList;

public interface CMPComponentPolynomialTrajectoryPlannerInterface
{
   public CMPComponentType getComponentType();
   public void initializeParameters(ExtendedCapturePointPlannerParameters icpPlannerParameters, BipedSupportPolygons bipedSupportPolygons,
                                    SideDependentList<? extends ContactablePlaneBody> contactableFeet, YoVariableRegistry parentRegistry, 
                                    double defaultFinalTransferDuration);
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing);
   public List<YoPolynomial3D> getPolynomialTrajectory();
}
