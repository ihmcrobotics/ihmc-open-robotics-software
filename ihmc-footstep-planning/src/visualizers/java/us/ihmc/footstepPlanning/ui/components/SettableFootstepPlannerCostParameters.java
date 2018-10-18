package us.ihmc.footstepPlanning.ui.components;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;

public class SettableFootstepPlannerCostParameters implements FootstepPlannerCostParameters
{
   private double yawWeight;
   private double costPerStep;

   public SettableFootstepPlannerCostParameters(FootstepPlannerCostParameters parameters)
   {
      this.yawWeight = parameters.getYawWeight();
      this.costPerStep = parameters.getCostPerStep();
   }

   public void setYawWeight(double yawWeight)
   {
      this.yawWeight = yawWeight;
   }

   public void setCostPerStep(double costPerStep)
   {
      this.costPerStep = costPerStep;
   }

   @Override
   public double getYawWeight()
   {
      return yawWeight;
   }

   @Override
   public double getCostPerStep()
   {
      return costPerStep;
   }
}
