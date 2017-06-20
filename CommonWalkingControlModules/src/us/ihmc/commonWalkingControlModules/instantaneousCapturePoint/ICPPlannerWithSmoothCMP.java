package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;

import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

import us.ihmc.robotics.robotSide.SideDependentList;

public class ICPPlannerWithSmoothCMP extends AbstractICPPlanner
{
   ReferenceCenterOfPressureLocationsCalculator referenceCOPsCalculator;
   
   public ICPPlannerWithSmoothCMP(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                  CapturePointPlannerParameters icpPlannerParameters, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(bipedSupportPolygons, contactableFeet, icpPlannerParameters, parentRegistry, yoGraphicsListRegistry);
   }
   
   public void clearPlan()
   {
      referenceCOPsCalculator.clear();
   }
   
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      referenceCOPsCalculator.addFootstepToPlan(footstep, timing);
   }

}
