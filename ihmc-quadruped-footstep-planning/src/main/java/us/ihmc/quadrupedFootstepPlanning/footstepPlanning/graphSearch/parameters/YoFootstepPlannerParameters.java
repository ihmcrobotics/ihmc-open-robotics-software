package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoFootstepPlannerParameters implements FootstepPlannerParametersBasics
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble maximumFrontStepReach = new YoDouble("maximumFrontStepReach", registry);
   private final YoDouble maximumFrontStepLength = new YoDouble("maximumFrontStepLength", registry);
   private final YoDouble minimumFrontStepLength = new YoDouble("minimumFrontStepLength", registry);
   private final YoDouble maximumHindStepReach = new YoDouble("maximumHindStepReach", registry);
   private final YoDouble maximumHindStepLength = new YoDouble("maximumHindStepLength", registry);
   private final YoDouble minimumHindStepLength = new YoDouble("minimumHindStepLength", registry);
   private final YoDouble maximumStepWidth = new YoDouble("maximumStepWidth", registry);
   private final YoDouble minimumStepWidth = new YoDouble("minimumStepWidth", registry);
   private final YoDouble minimumStepYaw = new YoDouble("minimumStepYaw", registry);
   private final YoDouble maximumStepYaw = new YoDouble("maximumStepYaw", registry);
   private final YoDouble maximumStepChangeZ = new YoDouble("maximumStepChangeZ", registry);
   private final YoDouble bodyGroundClearance = new YoDouble("bodyGroundClearance", registry);
   private final YoDouble distanceHeuristicWeight = new YoDouble("distanceHeuristicWeight", registry);
   private final YoDouble yawWeight = new YoDouble("yawWeight", registry);
   private final YoDouble xGaitWeight = new YoDouble("xGaitWeight", registry);
   private final YoDouble costPerStep = new YoDouble("costPerStep", registry);
   private final YoDouble stepUpWeight = new YoDouble("stepUpWeight", registry);
   private final YoDouble stepDownWeight = new YoDouble("stepDownWeight", registry);
   private final YoDouble heuristicsInflationWeight = new YoDouble("heuristicsInflationWeight", registry);
   private final YoDouble minXClearanceFromFoot = new YoDouble("minXClearanceFromFoot", registry);
   private final YoDouble minYClearanceFromFoot = new YoDouble("minYClearanceFromFoot", registry);
   private final YoDouble maxWalkingSpeedMultiplier = new YoDouble("maxWalkingSpeedMultiplier", registry);
   private final YoDouble projectionInsideDistanceForExpansion = new YoDouble("projectionInsideDistanceForExpansion", registry);
   private final YoDouble projectionInsideDistanceForPostProcessing = new YoDouble("projectionInsideDistanceForPostProcessing", registry);
   private final YoDouble maximumXYWiggleDistance = new YoDouble("maximumXYWiggleDistance", registry);
   private final YoDouble minimumSurfaceInclineRadians = new YoDouble("minimumSurfaceInclineRadians", registry);
   private final YoDouble cliffHeightToAvoid = new YoDouble("cliffHeightToAvoid", registry);
   private final YoDouble minimumDistanceFromCliffBottoms = new YoDouble("minimumCliffHeightFromBottoms", registry);
   private final YoDouble minimumDistanceFromCliffTops = new YoDouble("minimumCliffHeightFromTops", registry);

   public YoFootstepPlannerParameters(FootstepPlannerParameters parameters, YoVariableRegistry parentRegistry)
   {
      set(parameters);
      parentRegistry.addChild(registry);
   }

   @Override
   public void setMaximumFrontStepReach(double maximumStepReach)
   {
      this.maximumFrontStepReach.set(maximumStepReach);
   }

   @Override
   public void setMaximumFrontStepLength(double maximumStepLength)
   {
      this.maximumFrontStepLength.set(maximumStepLength);
   }

   @Override
   public void setMinimumFrontStepLength(double minimumStepLength)
   {
      this.minimumFrontStepLength.set(minimumStepLength);
   }

   @Override
   public void setMaximumHindStepReach(double maximumStepReach)
   {
      this.maximumHindStepReach.set(maximumStepReach);
   }

   @Override
   public void setMaximumHindStepLength(double maximumStepLength)
   {
      this.maximumHindStepLength.set(maximumStepLength);
   }

   @Override
   public void setMinimumHindStepLength(double minimumStepLength)
   {
      this.minimumHindStepLength.set(minimumStepLength);
   }

   @Override
   public void setMaximumStepWidth(double maximumStepWidth)
   {
      this.maximumStepWidth.set(maximumStepWidth);
   }

   @Override
   public void setMinimumStepWidth(double minimumStepWidth)
   {
      this.minimumStepWidth.set(minimumStepWidth);
   }

   @Override
   public void setMinimumStepYaw(double minimumStepYaw)
   {
      this.minimumStepYaw.set(minimumStepYaw);
   }

   @Override
   public void setMaximumStepYaw(double maximumStepYaw)
   {
      this.maximumStepYaw.set(maximumStepYaw);
   }

   @Override
   public void setMaximumStepChangeZ(double maximumStepChangeZ)
   {
      this.maximumStepChangeZ.set(maximumStepChangeZ);
   }

   @Override
   public void setBodyGroundClearance(double bodyGroundClearance)
   {
      this.bodyGroundClearance.set(bodyGroundClearance);
   }

   @Override
   public void setMaxWalkingSpeedMultiplier(double multiplier)
   {
      maxWalkingSpeedMultiplier.set(multiplier);
   }

   @Override
   public void setDistanceHeuristicWeight(double distanceHeuristicWeight)
   {
      this.distanceHeuristicWeight.set(distanceHeuristicWeight);
   }

   @Override
   public void setYawWeight(double yawWeight)
   {
      this.yawWeight.set(yawWeight);
   }

   @Override
   public void setXGaitWeight(double xGaitWeight)
   {
      this.xGaitWeight.set(xGaitWeight);
   }

   @Override
   public void setCostPerStep(double costPerStep)
   {
      this.costPerStep.set(costPerStep);
   }

   @Override
   public void setStepUpWeight(double stepUpWeight)
   {
      this.stepUpWeight.set(stepUpWeight);
   }

   @Override
   public void setStepDownWeight(double stepDownWeight)
   {
      this.stepDownWeight.set(stepDownWeight);
   }

   @Override
   public void setHeuristicsInflationWeight(double heuristicsInflationWeight)
   {
      this.heuristicsInflationWeight.set(heuristicsInflationWeight);
   }

   @Override
   public void setMinXClearanceFromFoot(double minXClearanceFromFoot)
   {
      this.minXClearanceFromFoot.set(minXClearanceFromFoot);
   }

   @Override
   public void setMinYClearanceFromFoot(double minYClearanceFromFoot)
   {
      this.minYClearanceFromFoot.set(minYClearanceFromFoot);
   }

   @Override
   public void setProjectInsideDistanceForExpansion(double projectionInsideDistance)
   {
      this.projectionInsideDistanceForExpansion.set(projectionInsideDistance);
   }

   @Override
   public void setProjectInsideDistanceForPostProcessing(double projectionInsideDistance)
   {
      this.projectionInsideDistanceForPostProcessing.set(projectionInsideDistance);
   }

   @Override
   public void setMaximumXYWiggleDistance(double wiggleDistance)
   {
      this.maximumXYWiggleDistance.set(wiggleDistance);
   }

   @Override
   public void setMinimumSurfaceInclineRadians(double minimumSurfaceIncline)
   {
      this.minimumSurfaceInclineRadians.set(minimumSurfaceIncline);
   }

   @Override
   public void setCliffHeightToAvoid(double cliffHeightToAvoid)
   {
      this.cliffHeightToAvoid.set(cliffHeightToAvoid);
   }

   @Override
   public void setMinimumDistanceFromCliffBottoms(double distance)
   {
      minimumDistanceFromCliffBottoms.set(distance);
   }

   @Override
   public void setMinimumDistanceFromCliffTops(double distance)
   {
      minimumDistanceFromCliffTops.set(distance);
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumFrontStepReach()
   {
      return maximumFrontStepReach.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumFrontStepLength()
   {
      return maximumFrontStepLength.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumFrontStepLength()
   {
      return minimumFrontStepLength.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumHindStepReach()
   {
      return maximumHindStepReach.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumHindStepLength()
   {
      return maximumHindStepLength.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumHindStepLength()
   {
      return minimumHindStepLength.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepWidth()
   {
      return maximumStepWidth.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumStepWidth()
   {
      return minimumStepWidth.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumStepYaw()
   {
      return minimumStepYaw.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepYaw()
   {
      return maximumStepYaw.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepChangeZ()
   {
      return maximumStepChangeZ.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getBodyGroundClearance()
   {
      return bodyGroundClearance.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxWalkingSpeedMultiplier()
   {
      return maxWalkingSpeedMultiplier.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getDistanceHeuristicWeight()
   {
      return distanceHeuristicWeight.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getYawWeight()
   {
      return yawWeight.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getXGaitWeight()
   {
      return xGaitWeight.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getCostPerStep()
   {
      return costPerStep.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getStepUpWeight()
   {
      return stepUpWeight.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getStepDownWeight()
   {
      return stepDownWeight.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getHeuristicsInflationWeight()
   {
      return heuristicsInflationWeight.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMinXClearanceFromFoot()
   {
      return minXClearanceFromFoot.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMinYClearanceFromFoot()
   {
      return minYClearanceFromFoot.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getProjectInsideDistanceForExpansion()
   {
      return projectionInsideDistanceForExpansion.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getProjectInsideDistanceForPostProcessing()
   {
      return projectionInsideDistanceForPostProcessing.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumXYWiggleDistance()
   {
      return maximumXYWiggleDistance.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumSurfaceInclineRadians()
   {
      return minimumSurfaceInclineRadians.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getCliffHeightToAvoid()
   {
      return cliffHeightToAvoid.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumDistanceFromCliffBottoms()
   {
      return minimumDistanceFromCliffBottoms.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumDistanceFromCliffTops()
   {
      return minimumDistanceFromCliffTops.getDoubleValue();
   }
}
