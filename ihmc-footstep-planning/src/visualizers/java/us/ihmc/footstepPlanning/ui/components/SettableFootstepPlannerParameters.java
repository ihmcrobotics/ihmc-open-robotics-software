package us.ihmc.footstepPlanning.ui.components;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;

public class SettableFootstepPlannerParameters implements FootstepPlannerParameters
{
   private double idealFootstepWidth;
   private double idealFootstepLength;

   private double maxStepReach;
   private double maxStepYaw;
   private double minStepWidth;
   private double minStepLength;
   private double minStepYaw;
   private double maxStepZ;
   private double minFootholdPercent;
   private double minSurfaceIncline;
   private double maxStepWidth;

   private final SettableFootstepPlannerCostParameters costParameters;

   public SettableFootstepPlannerParameters(FootstepPlannerParameters footstepPlanningParameters)
   {
      this.idealFootstepWidth = footstepPlanningParameters.getIdealFootstepWidth();
      this.idealFootstepLength = footstepPlanningParameters.getIdealFootstepLength();
      this.maxStepReach = footstepPlanningParameters.getMaximumStepReach();
      this.maxStepYaw = footstepPlanningParameters.getMaximumStepYaw();
      this.minStepWidth = footstepPlanningParameters.getMinimumStepWidth();
      this.minStepLength = footstepPlanningParameters.getMinimumStepLength();
      this.minStepYaw = footstepPlanningParameters.getMinimumStepYaw();
      this.maxStepZ = footstepPlanningParameters.getMaximumStepZ();
      this.maxStepWidth = footstepPlanningParameters.getMaximumStepWidth();
      this.minFootholdPercent = footstepPlanningParameters.getMinimumFootholdPercent();
      this.minSurfaceIncline = footstepPlanningParameters.getMinimumSurfaceInclineRadians();

      this.costParameters = new SettableFootstepPlannerCostParameters(footstepPlanningParameters.getCostParameters());
   }

   public void setIdealFootstepWidth(double idealFootstepLength)
   {
      this.idealFootstepLength = idealFootstepLength;
   }

   public void setIdealFootstepLength(double idealFootstepLength)
   {
      this.idealFootstepLength = idealFootstepLength;
   }

   public void setMaximumStepReach(double maxStepReach)
   {
      this.maxStepReach = maxStepReach;
   }

   public void setMaximumStepYaw(double maxStepYaw)
   {
      this.maxStepYaw = maxStepYaw;
   }

   public void setMinimumStepWidth(double minStepWidth)
   {
      this.minStepWidth = minStepWidth;
   }

   public void setMinimumStepLength(double minStepLength)
   {
      this.minStepLength = minStepLength;
   }

   public void setMinimumStepYaw(double minStepYaw)
   {
      this.minStepYaw = minStepYaw;
   }

   public void setMaximumStepZ(double maxStepZ)
   {
      this.maxStepZ = maxStepZ;
   }

   public void setMaximumStepWidth(double maxStepWidth)
   {
      this.maxStepWidth = maxStepWidth;
   }

   public void setMinimumFootholdPercent(double minFootholdPercent)
   {
      this.minFootholdPercent = minFootholdPercent;
   }

   public void setMinimumSurfaceInclineRadians(double minSurfaceIncline)
   {
      this.minSurfaceIncline = minSurfaceIncline;
   }

   public void setYawWeight(double yawWeight)
   {
      costParameters.setYawWeight(yawWeight);
   }

   public void setPitchWeight(double pitchWeight)
   {
      costParameters.setPitchWeight(pitchWeight);
   }

   public void setRollWeight(double rollWeight)
   {
      costParameters.setRollWeight(rollWeight);
   }

   public void setForwardWeight(double forwardWeight)
   {
      costParameters.setForwardWeight(forwardWeight);
   }

   public void setLateralWeight(double lateralWeight)
   {
      costParameters.setLateralWeight(lateralWeight);
   }

   public void setStepUpWeight(double stepUpWeight)
   {
      costParameters.setStepUpWeight(stepUpWeight);
   }

   public void setStepDownWeight(double stepDownWeight)
   {
      costParameters.setStepDownWeight(stepDownWeight);
   }

   public void setUseQuadraticDistanceCost(boolean useQuadraticDistanceCost)
   {
      costParameters.setUseQuadraticDistanceCost(useQuadraticDistanceCost);
   }

   public void setUseQuadraticHeightCost(boolean useQuadraticHeightCost)
   {
      costParameters.setUseQuadraticHeightCost(useQuadraticHeightCost);
   }

   public void setCostPerStep(double costPerStep)
   {
      costParameters.setCostPerStep(costPerStep);
   }

   public void setAStarHeuristicsWeight(double heuristicsWeight)
   {
      costParameters.setAStarHeuristicsWeight(heuristicsWeight);
   }

   public void setVisGraphWithAStarHeuristicsWeight(double heuristicsWeight)
   {
      costParameters.setVisGraphWithAStarHeuristicsWeight(heuristicsWeight);
   }

   public void setDepthFirstHeuristicsWeight(double heuristicsWeight)
   {
      costParameters.setDepthFirstHeuristicsWeight(heuristicsWeight);
   }

   public void setBodyPathBasedHeuristicsWeight(double heuristicsWeight)
   {
      costParameters.setBodyPathBasedHeuristicsWeight(heuristicsWeight);
   }

   @Override
   public double getIdealFootstepWidth()
   {
      return idealFootstepWidth;
   }

   @Override
   public double getIdealFootstepLength()
   {
      return idealFootstepLength;
   }

   @Override
   public double getMaximumStepReach()
   {
      return maxStepReach;
   }

   @Override
   public double getMaximumStepYaw()
   {
      return maxStepYaw;
   }

   @Override
   public double getMinimumStepWidth()
   {
      return minStepWidth;
   }

   @Override
   public double getMinimumStepLength()
   {
      return minStepLength;
   }

   @Override
   public double getMinimumStepYaw()
   {
      return minStepYaw;
   }

   @Override
   public double getMaximumStepZ()
   {
      return maxStepZ;
   }

   @Override
   public double getMaximumStepWidth()
   {
      return maxStepWidth;
   }

   @Override
   public double getMinimumFootholdPercent()
   {
      return minFootholdPercent;
   }

   @Override
   public double getMinimumSurfaceInclineRadians()
   {
      return minSurfaceIncline;
   }

   public boolean useQuadraticDistanceCost()
   {
      return costParameters.useQuadraticDistanceCost();
   }

   public boolean useQuadraticHeightCost()
   {
      return costParameters.useQuadraticHeightCost();
   }

   public double getYawWeight()
   {
      return costParameters.getYawWeight();
   }

   public double getPitchWeight()
   {
      return costParameters.getPitchWeight();
   }

   public double getRollWeight()
   {
      return costParameters.getRollWeight();
   }

   public double getForwardWeight()
   {
      return costParameters.getForwardWeight();
   }

   public double getLateralWeight()
   {
      return costParameters.getLateralWeight();
   }

   public double getStepUpWeight()
   {
      return costParameters.getStepUpWeight();
   }

   public double getStepDownWeight()
   {
      return costParameters.getStepDownWeight();
   }

   public double getCostPerStep()
   {
      return costParameters.getCostPerStep();
   }

   public double getAStarHeuristicsWeight()
   {
      return costParameters.getAStarHeuristicsWeight().getValue();
   }

   public double getVisGraphWithAStarHeuristicsWeight()
   {
      return costParameters.getVisGraphWithAStarHeuristicsWeight().getValue();
   }

   public double getDepthFirstHeuristicsWeight()
   {
      return costParameters.getDepthFirstHeuristicsWeight().getValue();
   }

   public double getBodyPathBasedHeuristicsWeight()
   {
      return costParameters.getBodyPathBasedHeuristicsWeight().getValue();
   }

   @Override
   public SettableFootstepPlannerCostParameters getCostParameters()
   {
      return costParameters;
   }
}
