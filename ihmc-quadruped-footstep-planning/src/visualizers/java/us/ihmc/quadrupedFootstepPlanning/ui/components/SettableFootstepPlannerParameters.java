package us.ihmc.quadrupedFootstepPlanning.ui.components;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;

public class SettableFootstepPlannerParameters implements FootstepPlannerParameters
{
   private double maximumStepReach;
   private double maximumStepWidth;
   private double maximumStepCycleDistance;
   private double minimumStepLength;
   private double minimumStepWidth;
   private double minimumStepYaw;
   private double maximumStepYaw;

   private double maximumStepChangeZ;
   private double maximumStepCycleChangeZ;
   private double bodyGroundClearance;

   private double forwardWeight;
   private double lateralWeight;
   private double yawWeight;
   private double costPerStep;
   private double stepUpWeight;
   private double stepDownWeight;
   private double heuristicsWeight;

   private double minXClearanceFromFoot;
   private double minYClearanceFromFoot;
   private double minimumSurfaceInclineRadians;

   public SettableFootstepPlannerParameters(FootstepPlannerParameters footstepPlannerParameters)
   {
      set(footstepPlannerParameters);
   }

   public void set(FootstepPlannerParameters footstepPlannerParameters)
   {
      this.maximumStepReach = footstepPlannerParameters.getMaximumStepReach();
      this.maximumStepWidth = footstepPlannerParameters.getMaximumStepWidth();
      this.maximumStepCycleDistance = footstepPlannerParameters.getMaximumStepCycleDistance();
      this.minimumStepLength = footstepPlannerParameters.getMinimumStepLength();
      this.minimumStepWidth = footstepPlannerParameters.getMinimumStepWidth();
      this.maximumStepYaw = footstepPlannerParameters.getMaximumStepYaw();
      this.minimumStepYaw = footstepPlannerParameters.getMinimumStepYaw();
      this.maximumStepChangeZ = footstepPlannerParameters.getMaximumStepChangeZ();
      this.maximumStepCycleChangeZ = footstepPlannerParameters.getMaximumStepCycleChangeZ();
      this.bodyGroundClearance = footstepPlannerParameters.getBodyGroundClearance();

      this.forwardWeight = footstepPlannerParameters.getForwardWeight();
      this.lateralWeight = footstepPlannerParameters.getLateralWeight();
      this.yawWeight = footstepPlannerParameters.getYawWeight();
      this.costPerStep = footstepPlannerParameters.getCostPerStep();
      this.stepUpWeight = footstepPlannerParameters.getStepUpWeight();
      this.stepDownWeight = footstepPlannerParameters.getStepDownWeight();
      this.heuristicsWeight = footstepPlannerParameters.getHeuristicsWeight();

      this.minimumSurfaceInclineRadians = footstepPlannerParameters.getMinimumSurfaceInclineRadians();
      this.minXClearanceFromFoot = footstepPlannerParameters.getMinXClearanceFromFoot();
      this.minYClearanceFromFoot = footstepPlannerParameters.getMinYClearanceFromFoot();
   }

   public void setMaximumStepReach(double maximumStepReach)
   {
      this.maximumStepReach = maximumStepReach;
   }

   public void setMaximumStepWidth(double maximumStepWidth)
   {
      this.maximumStepWidth = maximumStepWidth;
   }

   public void setMaximumStepCycleDistance(double maximumStepCycleDistance)
   {
      this.maximumStepCycleDistance = maximumStepCycleDistance;
   }

   public void setMinimumStepLength(double minimumStepLength)
   {
      this.minimumStepLength = minimumStepLength;
   }

   public void setMinimumStepWidth(double minimumStepWidth)
   {
      this.minimumStepWidth = minimumStepWidth;
   }

   public void setMinimumStepYaw(double minimumStepYaw)
   {
      this.minimumStepYaw = minimumStepYaw;
   }

   public void setMaximumStepYaw(double maximumStepYaw)
   {
      this.maximumStepYaw = maximumStepYaw;
   }

   public void setMaximumStepChangeZ(double maximumStepChangeZ)
   {
      this.maximumStepChangeZ = maximumStepChangeZ;
   }

   public void setMaximumStepCycleChangeZ(double maximumStepCycleChangeZ)
   {
      this.maximumStepCycleChangeZ = maximumStepCycleChangeZ;
   }

   public void setBodyGroundClearance(double bodyGroundClearance)
   {
      this.bodyGroundClearance = bodyGroundClearance;
   }

   public void setForwardWeight(double forwardWeight)
   {
      this.forwardWeight = forwardWeight;
   }

   public void setLateralWeight(double lateralWeight)
   {
      this.lateralWeight = lateralWeight;
   }

   public void setYawWeight(double yawWeight)
   {
      this.yawWeight = yawWeight;
   }

   public void setCostPerStep(double costPerStep)
   {
      this.costPerStep = costPerStep;
   }

   public void setStepUpWeight(double stepUpWeight)
   {
      this.stepUpWeight = stepUpWeight;
   }

   public void setStepDownWeight(double stepDownWeight)
   {
      this.stepDownWeight = stepDownWeight;
   }

   public void setHeuristicsWeight(double heuristicsWeight)
   {
      this.heuristicsWeight = heuristicsWeight;
   }

   public void setMinXClearanceFromFoot(double minXClearanceFromFoot)
   {
      this.minXClearanceFromFoot = minXClearanceFromFoot;
   }

   public void setMinYClearanceFromFoot(double minYClearanceFromFoot)
   {
      this.minYClearanceFromFoot = minYClearanceFromFoot;
   }

   public void setMinimumSurfaceInclineRadians(double minimumSurfaceInclineRadians)
   {
      this.minimumSurfaceInclineRadians = minimumSurfaceInclineRadians;
   }

   public double getMaximumStepReach()
   {
      return maximumStepReach;
   }

   public double getMaximumStepWidth()
   {
      return maximumStepWidth;
   }

   public double getMaximumStepCycleDistance()
   {
      return maximumStepCycleDistance;
   }

   public double getMinimumStepLength()
   {
      return minimumStepLength;
   }

   public double getMinimumStepWidth()
   {
      return minimumStepWidth;
   }

   public double getMinimumStepYaw()
   {
      return minimumStepYaw;
   }

   public double getMaximumStepYaw()
   {
      return maximumStepYaw;
   }

   public double getMaximumStepChangeZ()
   {
      return maximumStepChangeZ;
   }

   public double getMaximumStepCycleChangeZ()
   {
      return maximumStepCycleChangeZ;
   }

   public double getBodyGroundClearance()
   {
      return bodyGroundClearance;
   }

   public double getForwardWeight()
   {
      return forwardWeight;
   }

   public double getLateralWeight()
   {
      return lateralWeight;
   }

   public double getYawWeight()
   {
      return yawWeight;
   }

   public double getCostPerStep()
   {
      return costPerStep;
   }

   public double getStepUpWeight()
   {
      return stepUpWeight;
   }

   public double getStepDownWeight()
   {
      return stepDownWeight;
   }

   public double getHeuristicsWeight()
   {
      return heuristicsWeight;
   }

   public double getMinXClearanceFromFoot()
   {
      return minXClearanceFromFoot;
   }

   public double getMinYClearanceFromFoot()
   {
      return minYClearanceFromFoot;
   }

   public double getMinimumSurfaceInclineRadians()
   {
      return minimumSurfaceInclineRadians;
   }
}
