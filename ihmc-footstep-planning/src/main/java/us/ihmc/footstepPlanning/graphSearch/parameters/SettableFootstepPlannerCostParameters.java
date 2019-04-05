package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class SettableFootstepPlannerCostParameters implements FootstepPlannerCostParameters
{
   private boolean useQuadraticDistanceCost;
   private boolean useQuadraticHeightCost;

   private double yawWeight;
   private double pitchWeight;
   private double rollWeight;
   private double costPerStep;
   private double forwardWeight;
   private double lateralWeight;
   private double stepUpWeight;
   private double stepDownWeight;

   private double aStarHeuristicsWeight;
   private double visGraphWithAStarHeuristicsWeight;
   private double depthFirstHeuristicsWeight;
   private double bodyPathBasedHeuristicsWeight;
   private double boundingBoxCost;
   private double maximum2dDistanceFromBoundingBoxToPenalize;

   private final DoubleProvider aStarHeuristicsProvider = () -> aStarHeuristicsWeight;
   private final DoubleProvider visGraphWithAStarHeuristicsProvider = () -> visGraphWithAStarHeuristicsWeight;
   private final DoubleProvider depthFirstHeuristicsProvider = () -> depthFirstHeuristicsWeight;
   private final DoubleProvider bodyPathBasedHeuristicsProvider = () -> bodyPathBasedHeuristicsWeight;

   public SettableFootstepPlannerCostParameters(FootstepPlannerCostParameters parameters)
   {
      set(parameters);
   }

   public void set(FootstepPlannerCostParameters parameters)
   {
      this.useQuadraticDistanceCost = parameters.useQuadraticDistanceCost();
      this.useQuadraticHeightCost = parameters.useQuadraticHeightCost();

      this.yawWeight = parameters.getYawWeight();
      this.pitchWeight = parameters.getPitchWeight();
      this.rollWeight = parameters.getRollWeight();
      this.costPerStep = parameters.getCostPerStep();
      this.forwardWeight = parameters.getForwardWeight();
      this.lateralWeight = parameters.getLateralWeight();
      this.stepUpWeight = parameters.getStepUpWeight();
      this.stepDownWeight = parameters.getStepDownWeight();

      this.aStarHeuristicsWeight = parameters.getAStarHeuristicsWeight().getValue();
      this.visGraphWithAStarHeuristicsWeight = parameters.getVisGraphWithAStarHeuristicsWeight().getValue();
      this.depthFirstHeuristicsWeight = parameters.getDepthFirstHeuristicsWeight().getValue();
      this.bodyPathBasedHeuristicsWeight = parameters.getBodyPathBasedHeuristicsWeight().getValue();
      this.boundingBoxCost = parameters.getBoundingBoxCost();
      this.maximum2dDistanceFromBoundingBoxToPenalize = parameters.getMaximum2dDistanceFromBoundingBoxToPenalize();
   }

   public void setUseQuadraticDistanceCost(boolean useQuadraticDistanceCost)
   {
      this.useQuadraticDistanceCost = useQuadraticDistanceCost;
   }

   public void setUseQuadraticHeightCost(boolean useQuadraticHeightCost)
   {
      this.useQuadraticHeightCost = useQuadraticHeightCost;
   }

   public void setYawWeight(double yawWeight)
   {
      this.yawWeight = yawWeight;
   }

   public void setPitchWeight(double pitchWeight)
   {
      this.pitchWeight = pitchWeight;
   }

   public void setRollWeight(double rollWeight)
   {
      this.rollWeight = rollWeight;
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

   public void setForwardWeight(double forwardWeight)
   {
      this.forwardWeight = forwardWeight;
   }

   public void setLateralWeight(double lateralWeight)
   {
      this.lateralWeight = lateralWeight;
   }

   public void setStepUpWeight(double stepUpWeight)
   {
      this.stepUpWeight = stepUpWeight;
   }

   public void setStepDownWeight(double stepDownWeight)
   {
      this.stepDownWeight = stepDownWeight;
   }

   public void setBoundingBoxCost(double boundingBoxCost)
   {
      this.boundingBoxCost = boundingBoxCost;
   }

   public void setMaximum2dDistanceFromBoundingBoxToPenalize(double maximum2dDistanceFromBoundingBoxToPenalize)
   {
      this.maximum2dDistanceFromBoundingBoxToPenalize = maximum2dDistanceFromBoundingBoxToPenalize;
   }

   @Override
   public boolean useQuadraticDistanceCost()
   {
      return useQuadraticDistanceCost;
   }

   @Override
   public boolean useQuadraticHeightCost()
   {
      return useQuadraticHeightCost;
   }

   @Override
   public double getYawWeight()
   {
      return yawWeight;
   }

   @Override
   public double getPitchWeight()
   {
      return pitchWeight;
   }

   @Override
   public double getRollWeight()
   {
      return rollWeight;
   }

   @Override
   public double getCostPerStep()
   {
      return costPerStep;
   }

   @Override
   public double getForwardWeight()
   {
      return forwardWeight;
   }

   @Override
   public double getLateralWeight()
   {
      return lateralWeight;
   }

   @Override
   public double getStepUpWeight()
   {
      return stepUpWeight;
   }

   @Override
   public double getStepDownWeight()
   {
      return stepDownWeight;
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

   @Override
   public double getMaximum2dDistanceFromBoundingBoxToPenalize()
   {
      return maximum2dDistanceFromBoundingBoxToPenalize;
   }

   @Override
   public double getBoundingBoxCost()
   {
      return boundingBoxCost;
   }
}
