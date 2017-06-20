package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CenterOfPressurePlannerParameters;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.trajectories.YoPolynomial3D;
import us.ihmc.robotics.robotSide.SideDependentList;

public interface CMPComponentPolynomialTrajectoryPlannerInterface
{
   public CMPComponentType getComponentType();
   public void initializeParameters(CenterOfPressurePlannerParameters icpPlannerParameters, BipedSupportPolygons bipedSupportPolygons,
                                    SideDependentList<? extends ContactablePlaneBody> contactableFeet, YoVariableRegistry parentRegistry);
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing);
   public List<YoPolynomial3D> getPolynomialTrajectory();
   public void setInitialCoPPosition(FramePoint2d initialCoPPosition);
   public void setInitialCoPPosition(FramePoint initialCoPPosition);
   public void setInitialCoPVelocity(FrameVector2d intialCoPVelocity);
   public void setInitialCoPVelocity(FrameVector intialCoPVelocity);
}
