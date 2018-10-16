package us.ihmc.footstepPlanning.graphSearch.parameters;

import controller_msgs.msg.dds.FootstepPlannerCostParametersPacket;
import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoFootstepPlannerCostParameters implements FootstepPlannerCostParameters
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoBoolean useQuadraticDistanceCost = new YoBoolean("useQuadraticDistancCost", registry);

   private final YoDouble yawWeight = new YoDouble("yawWeight", registry);
   private final YoDouble pitchWeight = new YoDouble("pitchWeight", registry);
   private final YoDouble rollWeight = new YoDouble("rollWeight", registry);
   private final YoDouble forwardWeight = new YoDouble("forwardWeight", registry);
   private final YoDouble lateralWeight  = new YoDouble("lateralWeight", registry);
   private final YoDouble stepUpWeight = new YoDouble("stepUpWeight", registry);
   private final YoDouble stepDownWeight = new YoDouble("stepDownWeight", registry);
   private final YoDouble costPerStep = new YoDouble("costPerStep", registry);

   public YoFootstepPlannerCostParameters(YoVariableRegistry parentRegistry, FootstepPlannerCostParameters defaults)
   {
      parentRegistry.addChild(registry);
      set(defaults);
   }

   public void set(FootstepPlannerCostParameters defaults)
   {
      setUseQuadraticDistanceCost(defaults.useQuadraticDistanceCost());

      setYawWeight(defaults.getYawWeight());
      setPitchWeight(defaults.getPitchWeight());
      setRollWeight(defaults.getRollWeight());
      setForwardWeight(defaults.getForwardWeight());
      setLateralWeight(defaults.getLateralWeight());
      setStepUpWeight(defaults.getStepUpWeight());
      setStepDownWeight(defaults.getStepDownWeight());
      setCostPerStep(defaults.getCostPerStep());
   }

   @Override
   public boolean useQuadraticDistanceCost()
   {
      return useQuadraticDistanceCost.getBooleanValue();
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

   public void set(FootstepPlannerCostParametersPacket parametersPacket)
   {
      setUseQuadraticDistanceCost(parametersPacket.getUseQuadraticDistanceCost());

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
   }

   public void setUseQuadraticDistanceCost(boolean useQuadraticDistanceCost)
   {
      this.useQuadraticDistanceCost.set(useQuadraticDistanceCost);
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
}
