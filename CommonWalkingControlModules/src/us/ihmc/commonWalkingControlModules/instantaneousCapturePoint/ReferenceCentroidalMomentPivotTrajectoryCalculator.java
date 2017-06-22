package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class ReferenceCentroidalMomentPivotTrajectoryCalculator
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private YoVariableRegistry parentRegistry;
   
   private final String namePrefix; 
   private BipedSupportPolygons bipedSupportPolygons;
   private SideDependentList<ContactablePlaneBody> contactableFeet;
   private YoInteger maxNumberOfFootstepsToPlan;
  
   private CoPPolynomialTrajectoryPlannerInterface CoPPlanner;
   private AngularMomentumTrajectoryPlannerInterface angularMomentumPlanner;
   
   @SuppressWarnings("unchecked")
   public ReferenceCentroidalMomentPivotTrajectoryCalculator(String namePrefix, BipedSupportPolygons bipedSupportPolygons,
                                                             SideDependentList<? extends ContactablePlaneBody> contactableFeet, int numberFootstepsToConsider,
                                                             YoVariableRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.parentRegistry = parentRegistry;
      this.maxNumberOfFootstepsToPlan.set(numberFootstepsToConsider);
      this.contactableFeet = (SideDependentList<ContactablePlaneBody>) contactableFeet;
      
   }

}
