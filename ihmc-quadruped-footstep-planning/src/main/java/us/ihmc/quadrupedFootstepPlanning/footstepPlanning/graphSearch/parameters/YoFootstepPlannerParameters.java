package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
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
   private final YoDouble maximumFrontStepLengthWhenSteppingUp = new YoDouble("maximumFrontStepLengthWhenSteppingUp", registry);
   private final YoDouble minimumFrontStepLengthWhenSteppingUp = new YoDouble("minimumFrontStepLengthWhenSteppingUp", registry);
   private final YoDouble maximumHindStepLengthWhenSteppingUp = new YoDouble("maximumHindStepLengthWhenSteppingUp", registry);
   private final YoDouble minimumHindStepLengthWhenSteppingUp = new YoDouble("minimumHindStepLengthWhenSteppingUp", registry);
   private final YoDouble maximumFrontStepLengthWhenSteppingDown = new YoDouble("maximumFrontStepLengthWhenSteppingDown", registry);
   private final YoDouble minimumFrontStepLengthWhenSteppingDown = new YoDouble("minimumFrontStepLengthWhenSteppingDown", registry);
   private final YoDouble maximumHindStepLengthWhenSteppingDown = new YoDouble("maximumHindStepLengthWhenSteppingDown", registry);
   private final YoDouble minimumHindStepLengthWhenSteppingDown = new YoDouble("minimumHindStepLengthWhenSteppingDown", registry);
   private final YoDouble stepZForSteppingUp = new YoDouble("stepZForSteppingUp", registry);
   private final YoDouble stepZForSteppingDown = new YoDouble("stepZForSteppingDown", registry);
   private final YoDouble maximumStepWidth = new YoDouble("maximumStepWidth", registry);
   private final YoDouble minimumStepWidth = new YoDouble("minimumStepWidth", registry);
   private final YoDouble minimumStepYaw = new YoDouble("minimumStepYaw", registry);
   private final YoDouble maximumStepYaw = new YoDouble("maximumStepYaw", registry);
   private final YoDouble maximumStepChangeZ = new YoDouble("maximumStepChangeZ", registry);
   private final YoDouble bodyGroundClearance = new YoDouble("bodyGroundClearance", registry);
   private final YoDouble distanceWeight = new YoDouble("distanceWeight", registry);
   private final YoDouble yawWeight = new YoDouble("yawWeight", registry);
   private final YoDouble xGaitWeight = new YoDouble("xGaitWeight", registry);
   private final YoDouble desiredVelocityWeight = new YoDouble("desiredVelocityWeight", registry);
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
   private final YoDouble minimumFrontEndForwardDistanceFromCliffBottoms = new YoDouble("minimumFrontEndForwardCliffHeightFromBottoms", registry);
   private final YoDouble minimumFrontEndBackwardDistanceFromCliffBottoms = new YoDouble("minimumFrontEndBackwardCliffHeightFromBottoms", registry);
   private final YoDouble minimumHindEndForwardDistanceFromCliffBottoms = new YoDouble("minimumHindEndForwardCliffHeightFromBottoms", registry);
   private final YoDouble minimumHindEndBackwardDistanceFromCliffBottoms = new YoDouble("minimumHindEndBackwardCliffHeightFromBottoms", registry);
   private final YoDouble minimumLateralDistanceFromCliffBottoms = new YoDouble("minimumLateralCliffHeightFromBottoms", registry);
   private final YoBoolean projectInsideUsingConvexHullDuringExpansion = new YoBoolean("projectInsideUsingConvexHullDuringExpansion", registry);
   private final YoBoolean projectInsideUsingConvexHullDuringPostProcessing = new YoBoolean("projectInsideUsingConvexHullDuringPostProcessing", registry);
   private final YoDouble finalTurnProximity = new YoDouble("finalTurnProximity", registry);
   private final YoDouble finalSlowDownProximity = new YoDouble("finalSlowDownProximity", registry);

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
   public void setMaximumFrontStepLengthWhenSteppingUp(double maximumStepLength)
   {
      this.maximumFrontStepLengthWhenSteppingUp.set(maximumStepLength);
   }

   @Override
   public void setMinimumFrontStepLengthWhenSteppingUp(double minimumStepLength)
   {
      this.minimumFrontStepLengthWhenSteppingUp.set(minimumStepLength);
   }

   @Override
   public void setMaximumHindStepLengthWhenSteppingUp(double maximumStepLength)
   {
      this.maximumHindStepLengthWhenSteppingUp.set(maximumStepLength);
   }

   @Override
   public void setMinimumHindStepLengthWhenSteppingUp(double minimumStepLength)
   {
      this.minimumHindStepLengthWhenSteppingUp.set(minimumStepLength);
   }

   @Override
   public void setStepZForSteppingUp(double stepZ)
   {
      this.stepZForSteppingUp.set(stepZ);
   }

   @Override
   public void setMaximumFrontStepLengthWhenSteppingDown(double maximumStepLength)
   {
      this.maximumFrontStepLengthWhenSteppingDown.set(maximumStepLength);
   }

   @Override
   public void setMinimumFrontStepLengthWhenSteppingDown(double minimumStepLength)
   {
      this.minimumFrontStepLengthWhenSteppingDown.set(minimumStepLength);
   }

   @Override
   public void setMaximumHindStepLengthWhenSteppingDown(double maximumStepLength)
   {
      this.maximumHindStepLengthWhenSteppingDown.set(maximumStepLength);
   }

   @Override
   public void setMinimumHindStepLengthWhenSteppingDown(double minimumStepLength)
   {
      this.minimumHindStepLengthWhenSteppingDown.set(minimumStepLength);
   }

   @Override
   public void setStepZForSteppingDown(double stepZ)
   {
      this.stepZForSteppingDown.set(stepZ);
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
   public void setDistanceWeight(double distanceWeight)
   {
      this.distanceWeight.set(distanceWeight);
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
   public void setDesiredVelocityWeight(double desiredVelocityWeight)
   {
      this.desiredVelocityWeight.set(desiredVelocityWeight);
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
   public void setProjectInsideUsingConvexHullDuringExpansion(boolean projectInsideUsingConvexHull)
   {
      this.projectInsideUsingConvexHullDuringExpansion.set(projectInsideUsingConvexHull);
   }

   @Override
   public void setProjectInsideUsingConvexHullDuringPostProcessing(boolean projectInsideUsingConvexHull)
   {
      this.projectInsideUsingConvexHullDuringPostProcessing.set(projectInsideUsingConvexHull);
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
   public void setMinimumFrontEndForwardDistanceFromCliffBottoms(double distance)
   {
      minimumFrontEndForwardDistanceFromCliffBottoms.set(distance);
   }

   @Override
   public void setMinimumFrontEndBackwardDistanceFromCliffBottoms(double distance)
   {
      minimumFrontEndBackwardDistanceFromCliffBottoms.set(distance);
   }

   @Override
   public void setMinimumHindEndForwardDistanceFromCliffBottoms(double distance)
   {
      minimumHindEndForwardDistanceFromCliffBottoms.set(distance);
   }

   @Override
   public void setMinimumHindEndBackwardDistanceFromCliffBottoms(double distance)
   {
      minimumHindEndBackwardDistanceFromCliffBottoms.set(distance);
   }

   @Override
   public void setMinimumLateralDistanceFromCliffBottoms(double distance)
   {
      minimumLateralDistanceFromCliffBottoms.set(distance);
   }

   @Override
   public void setFinalTurnProximity(double proximity)
   {
      this.finalTurnProximity.set(proximity);
   }

   @Override
   public void setFinalSlowDownProximity(double proximity)
   {
      this.finalSlowDownProximity.set(proximity);
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

   @Override
   public double getMaximumFrontStepLengthWhenSteppingUp()
   {
      return maximumFrontStepLengthWhenSteppingUp.getDoubleValue();
   }

   @Override
   public double getMinimumFrontStepLengthWhenSteppingUp()
   {
      return minimumFrontStepLengthWhenSteppingUp.getDoubleValue();
   }

   @Override
   public double getMaximumHindStepLengthWhenSteppingUp()
   {
      return maximumHindStepLengthWhenSteppingUp.getDoubleValue();
   }

   @Override
   public double getMinimumHindStepLengthWhenSteppingUp()
   {
      return minimumHindStepLengthWhenSteppingUp.getDoubleValue();
   }

   @Override
   public double getStepZForSteppingUp()
   {
      return stepZForSteppingUp.getDoubleValue();
   }

   @Override
   public double getMaximumFrontStepLengthWhenSteppingDown()
   {
      return maximumFrontStepLengthWhenSteppingDown.getDoubleValue();
   }

   @Override
   public double getMinimumFrontStepLengthWhenSteppingDown()
   {
      return minimumFrontStepLengthWhenSteppingDown.getDoubleValue();
   }

   @Override
   public double getMaximumHindStepLengthWhenSteppingDown()
   {
      return maximumHindStepLengthWhenSteppingDown.getDoubleValue();
   }

   @Override
   public double getMinimumHindStepLengthWhenSteppingDown()
   {
      return minimumHindStepLengthWhenSteppingDown.getDoubleValue();
   }

   @Override
   public double getStepZForSteppingDown()
   {
      return stepZForSteppingDown.getDoubleValue();
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
   public double getDistanceWeight()
   {
      return distanceWeight.getDoubleValue();
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

   @Override
   public double getDesiredVelocityWeight()
   {
      return desiredVelocityWeight.getDoubleValue();
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
   public boolean getProjectInsideUsingConvexHullDuringExpansion()
   {
      return projectInsideUsingConvexHullDuringExpansion.getBooleanValue();
   }

   @Override
   public boolean getProjectInsideUsingConvexHullDuringPostProcessing()
   {
      return projectInsideUsingConvexHullDuringPostProcessing.getBooleanValue();
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
   public double getMinimumFrontEndForwardDistanceFromCliffBottoms()
   {
      return minimumFrontEndForwardDistanceFromCliffBottoms.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumFrontEndBackwardDistanceFromCliffBottoms()
   {
      return minimumFrontEndBackwardDistanceFromCliffBottoms.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumHindEndForwardDistanceFromCliffBottoms()
   {
      return minimumHindEndForwardDistanceFromCliffBottoms.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumHindEndBackwardDistanceFromCliffBottoms()
   {
      return minimumHindEndBackwardDistanceFromCliffBottoms.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumLateralDistanceFromCliffBottoms()
   {
      return minimumLateralDistanceFromCliffBottoms.getDoubleValue();
   }

   @Override
   public double getFinalTurnProximity()
   {
      return finalTurnProximity.getDoubleValue();
   }

   @Override
   public double getFinalSlowDownProximity()
   {
      return finalSlowDownProximity.getDoubleValue();
   }

}
