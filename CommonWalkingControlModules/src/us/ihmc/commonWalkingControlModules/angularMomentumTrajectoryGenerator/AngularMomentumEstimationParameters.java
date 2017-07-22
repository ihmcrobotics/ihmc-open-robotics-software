package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;

public class AngularMomentumEstimationParameters
{
   double comHeight = 0.5;
   private SmoothCMPPlannerParameters copPlannerParameters;
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
   
   public CoPPointName getInitialCoPPointName()
   {
      return getEndCoPName();
   }

   public CoPPointName getEndCoPPointName()
   {
      return getEndCoPName();
   }

   public CoPPointName getInitialDepartureReferenceName()
   {
      return getEntryCoPName();
   }
   
   public CoPPointName getFinalApproachReferenceName()
   {
      return getExitCoPName();
   }
   
   public int getNumberOfFootstepsToConsider()
   {
      return this.copPlannerParameters.getNumberOfFootstepsToConsider();
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
   
   public double getCoMHeight()
   {
      return 0.3;
   }

   public double getSwingFootMaxLift()
   {
      return 0.01;
   }
   
   public int getNumberOfPointsToSampleForTransfer()
   {
      return 20;
   }

   public int getNumberOfPointsToSampleForSwing()
   {
      return 20;
   }

   public AngularMomentumSplineType getSplineType()
   {
      return AngularMomentumSplineType.LINEAR;
   }
   
   public String getYoTimeVariableName()
   {
      return "t";
   }
}
