package us.ihmc.quadrupedFootstepPlanning.ui.components;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;

public class SettableFootstepPlannerParameters implements FootstepPlannerParameters
{
   private double maximumStepReach;
   private double maximumStepLength;
   private double maximumStepWidth;
   private double minimumStepLength;
   private double minimumStepWidth;
   private double minimumStepYaw;
   private double maximumStepYaw;

   private double maximumStepChangeZ;
   private double bodyGroundClearance;

   private double yawWeight;
   private double costPerStep;
   private double stepUpWeight;
   private double stepDownWeight;
   private double heuristicsWeight;
   private double distanceWeight;
   private double xGaitWeight;

   private double minXClearanceFromFoot;
   private double minYClearanceFromFoot;
   private double minimumSurfaceInclineRadians;
   private double projectInsideDistance;

   private double crawlSpeed;
   private double trotSpeed;
   private double paceSpeed;

   public SettableFootstepPlannerParameters(FootstepPlannerParameters footstepPlannerParameters)
   {
      set(footstepPlannerParameters);
   }

   public void set(FootstepPlannerParameters footstepPlannerParameters)
   {
      this.maximumStepReach = footstepPlannerParameters.getMaximumStepReach();
      this.maximumStepLength = footstepPlannerParameters.getMaximumStepLength();
      this.maximumStepWidth = footstepPlannerParameters.getMaximumStepWidth();
      this.minimumStepLength = footstepPlannerParameters.getMinimumStepLength();
      this.minimumStepWidth = footstepPlannerParameters.getMinimumStepWidth();
      this.maximumStepYaw = footstepPlannerParameters.getMaximumStepYaw();
      this.minimumStepYaw = footstepPlannerParameters.getMinimumStepYaw();
      this.maximumStepChangeZ = footstepPlannerParameters.getMaximumStepChangeZ();
      this.bodyGroundClearance = footstepPlannerParameters.getBodyGroundClearance();

      this.yawWeight = footstepPlannerParameters.getYawWeight();
      this.costPerStep = footstepPlannerParameters.getCostPerStep();
      this.stepUpWeight = footstepPlannerParameters.getStepUpWeight();
      this.stepDownWeight = footstepPlannerParameters.getStepDownWeight();
      this.heuristicsWeight = footstepPlannerParameters.getHeuristicsInflationWeight();
      this.distanceWeight = footstepPlannerParameters.getDistanceHeuristicWeight();
      this.xGaitWeight = footstepPlannerParameters.getXGaitWeight();

      this.minimumSurfaceInclineRadians = footstepPlannerParameters.getMinimumSurfaceInclineRadians();
      this.minXClearanceFromFoot = footstepPlannerParameters.getMinXClearanceFromFoot();
      this.minYClearanceFromFoot = footstepPlannerParameters.getMinYClearanceFromFoot();
      this.projectInsideDistance = footstepPlannerParameters.getProjectInsideDistance();

      this.crawlSpeed = footstepPlannerParameters.getCrawlSpeed();
      this.trotSpeed = footstepPlannerParameters.getTrotSpeed();
      this.paceSpeed = footstepPlannerParameters.getPaceSpeed();
   }

   public void setMaximumStepReach(double maximumStepReach)
   {
      this.maximumStepReach = maximumStepReach;
   }

   public void setMaximumStepLength(double maximumStepLength)
   {
      this.maximumStepLength = maximumStepLength;
   }

   public void setMaximumStepWidth(double maximumStepWidth)
   {
      this.maximumStepWidth = maximumStepWidth;
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

   public void setBodyGroundClearance(double bodyGroundClearance)
   {
      this.bodyGroundClearance = bodyGroundClearance;
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

   public void setProjectInsideDistance(double projectInsideDistance)
   {
      this.projectInsideDistance = projectInsideDistance;
   }

   public double getMaximumStepReach()
   {
      return maximumStepReach;
   }

   public double getMaximumStepLength()
   {
      return maximumStepLength;
   }

   public double getMaximumStepWidth()
   {
      return maximumStepWidth;
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

   public double getBodyGroundClearance()
   {
      return bodyGroundClearance;
   }

   public double getDistanceHeuristicWeight()
   {
      return distanceWeight;
   }

   public double getXGaitWeight()
   {
      return xGaitWeight;
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

   public double getHeuristicsInflationWeight()
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

   public double getProjectInsideDistance()
   {
      return projectInsideDistance;
   }

   public double getTrotSpeed()
   {
      return trotSpeed;
   }

   public double getCrawlSpeed()
   {
      return crawlSpeed;
   }

   public double getPaceSpeed()
   {
      return paceSpeed;
   }
}
