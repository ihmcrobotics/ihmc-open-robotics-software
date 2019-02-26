package us.ihmc.footstepPlanning.graphSearch.parameters;

import controller_msgs.msg.dds.FootstepPlannerCostParametersPacket;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoFootstepPlannerCostParameters implements FootstepPlannerCostParameters
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoBoolean useQuadraticDistanceCost = new YoBoolean("useQuadraticDistanceCost", registry);
   private final YoBoolean useQuadraticHeightCost = new YoBoolean("useQuadraticHeightCost", registry);

   private final YoDouble aStarHeuristicsWeight = new YoDouble("aStarHeuristicsWeight", registry);
   private final YoDouble visGraphAStarHeuristicsWeight = new YoDouble("visGraphAStarHeuristicsWeight", registry);
   private final YoDouble depthFirstHeuristicsWeight = new YoDouble("depthFirstHeuristicsWeight", registry);
   private final YoDouble bodyPathBasedHeuristicsWeight = new YoDouble("bodyPathBasedHeuristicsWeight", registry);

   private final YoDouble yawWeight = new YoDouble("yawWeight", registry);
   private final YoDouble pitchWeight = new YoDouble("pitchWeight", registry);
   private final YoDouble rollWeight = new YoDouble("rollWeight", registry);
   private final YoDouble forwardWeight = new YoDouble("forwardWeight", registry);
   private final YoDouble lateralWeight = new YoDouble("lateralWeight", registry);
   private final YoDouble stepUpWeight = new YoDouble("stepUpWeight", registry);
   private final YoDouble stepDownWeight = new YoDouble("stepDownWeight", registry);
   private final YoDouble costPerStep = new YoDouble("costPerStep", registry);
   private final YoDouble maximum2dDistanceFromBoundingBoxToPenalize = new YoDouble("maximum2dDistanceFromBoundingBoxToPenalize", registry);
   private final YoDouble boundingBoxCost = new YoDouble("boundingBoxCost", registry);

   public YoFootstepPlannerCostParameters(YoVariableRegistry parentRegistry, FootstepPlannerCostParameters defaults)
   {
      parentRegistry.addChild(registry);
      set(defaults);
   }

   public void set(FootstepPlannerCostParameters defaults)
   {
      setUseQuadraticDistanceCost(defaults.useQuadraticDistanceCost());
      setUseQuadraticHeightCost(defaults.useQuadraticHeightCost());

      setAStarHeuristicsWeight(defaults.getAStarHeuristicsWeight().getValue());
      setVisGraphWithAStarHeuristicsWeight(defaults.getVisGraphWithAStarHeuristicsWeight().getValue());
      setDepthFirstHeuristicsWeight(defaults.getDepthFirstHeuristicsWeight().getValue());
      setBodyPathBasedHeuristicWeight(defaults.getBodyPathBasedHeuristicsWeight().getValue());

      setYawWeight(defaults.getYawWeight());
      setPitchWeight(defaults.getPitchWeight());
      setRollWeight(defaults.getRollWeight());
      setForwardWeight(defaults.getForwardWeight());
      setLateralWeight(defaults.getLateralWeight());
      setStepUpWeight(defaults.getStepUpWeight());
      setStepDownWeight(defaults.getStepDownWeight());
      setCostPerStep(defaults.getCostPerStep());
      setMaximum2dDistanceFromBoundingBoxToPenalize(defaults.getMaximum2dDistanceFromBoundingBoxToPenalize());
      setBoundingBoxCost(defaults.getBoundingBoxCost());
   }

   @Override
   public boolean useQuadraticDistanceCost()
   {
      return useQuadraticDistanceCost.getBooleanValue();
   }

   @Override
   public boolean useQuadraticHeightCost()
   {
      return useQuadraticHeightCost.getBooleanValue();
   }

   @Override
   public DoubleProvider getAStarHeuristicsWeight()
   {
      return aStarHeuristicsWeight;
   }

   @Override
   public DoubleProvider getVisGraphWithAStarHeuristicsWeight()
   {
      return visGraphAStarHeuristicsWeight;
   }

   @Override
   public DoubleProvider getDepthFirstHeuristicsWeight()
   {
      return depthFirstHeuristicsWeight;
   }

   @Override
   public DoubleProvider getBodyPathBasedHeuristicsWeight()
   {
      return bodyPathBasedHeuristicsWeight;
   }

   @Override
   public double getYawWeight()
   {
      return yawWeight.getDoubleValue();
   }

   @Override
   public double getPitchWeight()
   {
      return pitchWeight.getDoubleValue();
   }

   @Override
   public double getRollWeight()
   {
      return rollWeight.getDoubleValue();
   }

   @Override
   public double getForwardWeight()
   {
      return forwardWeight.getDoubleValue();
   }

   @Override
   public double getLateralWeight()
   {
      return lateralWeight.getDoubleValue();
   }

   @Override
   public double getStepUpWeight()
   {
      return stepUpWeight.getDoubleValue();
   }

   @Override
   public double getStepDownWeight()
   {
      return stepDownWeight.getDoubleValue();
   }

   @Override
   public double getCostPerStep()
   {
      return costPerStep.getDoubleValue();
   }

   @Override
   public double getMaximum2dDistanceFromBoundingBoxToPenalize()
   {
      return maximum2dDistanceFromBoundingBoxToPenalize.getDoubleValue();
   }

   @Override
   public double getBoundingBoxCost()
   {
      return boundingBoxCost.getDoubleValue();
   }

   public void set(FootstepPlannerCostParametersPacket parametersPacket)
   {
      setUseQuadraticDistanceCost(parametersPacket.getUseQuadraticDistanceCost());
      setUseQuadraticHeightCost(parametersPacket.getUseQuadraticHeightCost());

      if (parametersPacket.getAStarHeuristicsWeight() != -1.0)
         setAStarHeuristicsWeight(parametersPacket.getAStarHeuristicsWeight());
      if (parametersPacket.getVisGraphWithAStarHeuristicsWeight() != -1.0)
         setVisGraphWithAStarHeuristicsWeight(parametersPacket.getVisGraphWithAStarHeuristicsWeight());
      if (parametersPacket.getDepthFirstHeuristicsWeight() != -1.0)
         setDepthFirstHeuristicsWeight(parametersPacket.getDepthFirstHeuristicsWeight());
      if (parametersPacket.getBodyPathBasedHeuristicsWeight() != -1.0)
         setBodyPathBasedHeuristicWeight(parametersPacket.getBodyPathBasedHeuristicsWeight());

      if (parametersPacket.getYawWeight() != -1.0)
         setYawWeight(parametersPacket.getYawWeight());
      if (parametersPacket.getPitchWeight() != -1.0)
         setPitchWeight(parametersPacket.getPitchWeight());
      if (parametersPacket.getRollWeight() != -1.0)
         setRollWeight(parametersPacket.getRollWeight());
      if (parametersPacket.getForwardWeight() != -1.0)
         setForwardWeight(parametersPacket.getForwardWeight());
      if (parametersPacket.getLateralWeight() != -1.0)
         setLateralWeight(parametersPacket.getLateralWeight());
      if (parametersPacket.getStepUpWeight() != -1.0)
         setStepUpWeight(parametersPacket.getStepUpWeight());
      if (parametersPacket.getStepDownWeight() != -1.0)
         setStepDownWeight(parametersPacket.getStepDownWeight());
      if (parametersPacket.getCostPerStep() != -1.0)
         setCostPerStep(parametersPacket.getCostPerStep());
      if (parametersPacket.getMaximum2dDistanceFromBoundingBoxToPenalize() != -1.0)
         setMaximum2dDistanceFromBoundingBoxToPenalize(parametersPacket.getMaximum2dDistanceFromBoundingBoxToPenalize());
      if (parametersPacket.getBoundingBoxCost() != -1.0)
         setBoundingBoxCost(parametersPacket.getBoundingBoxCost());
   }

   public void setUseQuadraticDistanceCost(boolean useQuadraticDistanceCost)
   {
      this.useQuadraticDistanceCost.set(useQuadraticDistanceCost);
   }

   public void setUseQuadraticHeightCost(boolean useQuadraticHeightCost)
   {
      this.useQuadraticHeightCost.set(useQuadraticHeightCost);
   }

   public void setAStarHeuristicsWeight(double aStarHeuristicsWeight)
   {
      this.aStarHeuristicsWeight.set(aStarHeuristicsWeight);
   }

   public void setVisGraphWithAStarHeuristicsWeight(double visGraphWithAStarHeuristicsWeight)
   {
      this.visGraphAStarHeuristicsWeight.set(visGraphWithAStarHeuristicsWeight);
   }

   public void setDepthFirstHeuristicsWeight(double depthFirstHeuristicsWeight)
   {
      this.depthFirstHeuristicsWeight.set(depthFirstHeuristicsWeight);
   }

   public void setBodyPathBasedHeuristicWeight(double bodyPathBasedHeuristicWeight)
   {
      this.bodyPathBasedHeuristicsWeight.set(bodyPathBasedHeuristicWeight);
   }

   public void setYawWeight(double yawWeight)
   {
      this.yawWeight.set(yawWeight);
   }

   public void setPitchWeight(double pitchWeight)
   {
      this.pitchWeight.set(pitchWeight);
   }

   public void setRollWeight(double rollWeight)
   {
      this.rollWeight.set(rollWeight);
   }

   public void setStepUpWeight(double stepUpWeight)
   {
      this.stepUpWeight.set(stepUpWeight);
   }

   public void setStepDownWeight(double stepDownWeight)
   {
      this.stepDownWeight.set(stepDownWeight);
   }

   public void setForwardWeight(double forwardWeight)
   {
      this.forwardWeight.set(forwardWeight);
   }

   public void setLateralWeight(double lateralWeight)
   {
      this.lateralWeight.set(lateralWeight);
   }

   public void setCostPerStep(double costPerStep)
   {
      this.costPerStep.set(costPerStep);
   }

   public void setMaximum2dDistanceFromBoundingBoxToPenalize(double maximum2dDistanceFromBoundingBoxToPenalize)
   {
      this.maximum2dDistanceFromBoundingBoxToPenalize.set(maximum2dDistanceFromBoundingBoxToPenalize);
   }

   public void setBoundingBoxCost(double boundingBoxCost)
   {
      this.boundingBoxCost.set(boundingBoxCost);
   }
}
