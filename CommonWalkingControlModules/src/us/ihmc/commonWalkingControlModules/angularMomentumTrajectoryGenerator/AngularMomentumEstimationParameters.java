package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;

public class AngularMomentumEstimationParameters
{
   double comHeight = 0.5;
   SmoothCMPPlannerParameters copPlannerParameters;
   public AngularMomentumEstimationParameters(SmoothCMPPlannerParameters cmpPlannerParameters)
   {
      this.copPlannerParameters = cmpPlannerParameters;
   }
   
   public CoPPointName getEntryCoPName()
   {
      return this.copPlannerParameters.getEntryCoPName();
   }
   
   public CoPPointName getExitCoPName()
   {
      return this.copPlannerParameters.getExitCoPName();
   }
   
   public CoPPointName getEndCoPName()
   {
      return this.copPlannerParameters.getEndCoPName();
   }
   
   public double getSwingLegMass()
   {
      return 0.5;
   }
   
   public double getBodyMass()
   {
      return 1.0;
   }
   
   public double getSupportLegMass()
   {
      return 0.5;
   }
   
   public CoPPointName getInitialCoPPoint()
   {
      return getEndCoPName();
   }

   public CoPPointName getEndCoPPoint()
   {
      return getEndCoPName();
   }

   public CoPPointName getInitialReferencePoint()
   {
      return getEntryCoPName();
   }
   
   public CoPPointName getFinalReferencePoint()
   {
      return getExitCoPName();
   }
   
   public int getNumberOfFootstepsToConsider()
   {
      return this.copPlannerParameters.getNumberOfFootstepsToConsider();
   }
   
   public int getMaximumNumberOfAngularMomentumPointsToPlan()
   {
      return 100;
   }
   
   public SmoothCMPPlannerParameters getCoPPlannerParameters()
   {
      return this.copPlannerParameters;
   }
   
   public CoPPointName[] getCoPPointList()
   {
      return copPlannerParameters.getCoPPointsToPlan();
   }
   
   public EnumMap<CoPPointName, Double> getSegmentTimes()
   {
      return copPlannerParameters.getSegmentTimes();
   }
}
