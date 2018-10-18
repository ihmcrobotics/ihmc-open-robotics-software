package us.ihmc.footstepPlanning.ui.components;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class SettableFootstepPlannerCostParameters implements FootstepPlannerCostParameters
{
   private double yawWeight;
   private double costPerStep;

   private double aStarHeuristicsWeight;
   private double visGraphWithAStarHeuristicsWeight;
   private double depthFirstHeuristicsWeight;
   private double bodyPathBasedHeuristicsWeight;

   private final DoubleProvider aStarHeuristicsProvider = () -> aStarHeuristicsWeight;
   private final DoubleProvider visGraphWithAStarHeuristicsProvider = () -> visGraphWithAStarHeuristicsWeight;
   private final DoubleProvider depthFirstHeuristicsProvider = () -> depthFirstHeuristicsWeight;
   private final DoubleProvider bodyPathBasedHeuristicsProvider = () -> bodyPathBasedHeuristicsWeight;

   public SettableFootstepPlannerCostParameters(FootstepPlannerCostParameters parameters)
   {
      this.yawWeight = parameters.getYawWeight();
      this.costPerStep = parameters.getCostPerStep();
      this.aStarHeuristicsWeight = parameters.getAStarHeuristicsWeight().getValue();
      this.visGraphWithAStarHeuristicsWeight = parameters.getVisGraphWithAStarHeuristicsWeight().getValue();
      this.depthFirstHeuristicsWeight = parameters.getDepthFirstHeuristicsWeight().getValue();
      this.bodyPathBasedHeuristicsWeight = parameters.getBodyPathBasedHeuristicsWeight().getValue();
   }

   public void setYawWeight(double yawWeight)
   {
      this.yawWeight = yawWeight;
   }

   public void setCostPerStep(double costPerStep)
   {
      this.costPerStep = costPerStep;
   }

   public void setAStarHeuristicsWeight(double heuristicsWeight)
   {
      aStarHeuristicsWeight = heuristicsWeight;
   }

   public void setVisGraphWithAStarHeuristicsWeight(double heuristicsWeight)
   {
      visGraphWithAStarHeuristicsWeight = heuristicsWeight;
   }

   public void setDepthFirstHeuristicsWeight(double heuristicsWeight)
   {
      depthFirstHeuristicsWeight = heuristicsWeight;
   }

   public void setBodyPathBasedHeuristicsWeight(double heuristicsWeight)
   {
      bodyPathBasedHeuristicsWeight = heuristicsWeight;
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

   @Override
   public DoubleProvider getAStarHeuristicsWeight()
   {
      return aStarHeuristicsProvider;
   }

   @Override
   public DoubleProvider getVisGraphWithAStarHeuristicsWeight()
   {
      return visGraphWithAStarHeuristicsProvider;
   }

   @Override
   public DoubleProvider getDepthFirstHeuristicsWeight()
   {
      return depthFirstHeuristicsProvider;
   }

   @Override
   public DoubleProvider getBodyPathBasedHeuristicsWeight()
   {
      return bodyPathBasedHeuristicsProvider;
   }
}
