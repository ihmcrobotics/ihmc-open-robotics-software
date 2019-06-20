package us.ihmc.quadrupedFootstepPlanning.ui.components;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;

public class SettableFootstepPlannerParameters implements FootstepPlannerParametersBasics
{
   private double maximumFrontStepReach;
   private double maximumFrontStepLength;
   private double minimumFrontStepLength;
   private double maximumHindStepReach;
   private double maximumHindStepLength;
   private double minimumHindStepLength;

   private double maximumFrontStepLengthWhenSteppingUp;
   private double minimumFrontStepLengthWhenSteppingUp;
   private double maximumHindStepLengthWhenSteppingUp;
   private double minimumHindStepLengthWhenSteppingUp;
   private double stepZForSteppingUp;

   private double maximumFrontStepLengthWhenSteppingDown;
   private double minimumFrontStepLengthWhenSteppingDown;
   private double maximumHindStepLengthWhenSteppingDown;
   private double minimumHindStepLengthWhenSteppingDown;
   private double stepZForSteppingDown;

   private double maximumStepOutward;
   private double maximumStepInward;
   private double maximumStepYawInward;
   private double maximumStepYawOutward;

   private double maximumStepChangeZ;
   private double bodyGroundClearance;
   private double maxWalkingSpeedMultiplier;

   private double yawWeight;
   private double costPerStep;
   private double stepUpWeight;
   private double stepDownWeight;
   private double distanceWeight;
   private double heuristicsWeight;
   private double xGaitWeight;
   private double desiredVelocityWeight;

   private double minXClearanceFromFoot;
   private double minYClearanceFromFoot;
   private double minimumSurfaceInclineRadians;
   private double projectInsideDistanceForExpansion;
   private double projectInsideDistanceForPostProcessing;
   private double maximumXYWiggleDistance;

   private double cliffHeightToAvoid;
   private double minimumFrontEndForwardDistanceFromCliffBottoms;
   private double minimumFrontEndBackwardDistanceFromCliffBottoms;
   private double minimumHindEndForwardDistanceFromCliffBottoms;
   private double minimumHindEndBackwardDistanceFromCliffBottoms;
   private double minimumLateralDistanceFromCliffBottoms;

   private boolean projectInsideUsingConvexHullDuringExpansion;
   private boolean projectInsideUsingConvexHullDuringPostProcessing;

   private double finalTurnProximity;
   private double finalSlowDownProximity;

   public SettableFootstepPlannerParameters(FootstepPlannerParameters footstepPlannerParameters)
   {
      set(footstepPlannerParameters);
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumFrontStepReach(double maximumStepReach)
   {
      this.maximumFrontStepReach = maximumStepReach;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumFrontStepLength(double maximumStepLength)
   {
      this.maximumFrontStepLength = maximumStepLength;
   }

   /** {@inheritDoc} */
   @Override
   public void setMinimumFrontStepLength(double minimumStepLength)
   {
      this.minimumFrontStepLength = minimumStepLength;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumHindStepReach(double maximumStepReach)
   {
      this.maximumHindStepReach = maximumStepReach;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumHindStepLength(double maximumStepLength)
   {
      this.maximumHindStepLength = maximumStepLength;
   }

   /** {@inheritDoc} */
   @Override
   public void setMinimumHindStepLength(double minimumStepLength)
   {
      this.minimumHindStepLength = minimumStepLength;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumFrontStepLengthWhenSteppingUp(double stepLength)
   {
      maximumFrontStepLengthWhenSteppingUp = stepLength;
   }

   /** {@inheritDoc} */
   @Override
   public void setMinimumFrontStepLengthWhenSteppingUp(double stepLength)
   {
      minimumFrontStepLengthWhenSteppingUp = stepLength;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumHindStepLengthWhenSteppingUp(double stepLength)
   {
      maximumHindStepLengthWhenSteppingUp = stepLength;
   }

   /** {@inheritDoc} */
   @Override
   public void setMinimumHindStepLengthWhenSteppingUp(double stepLength)
   {
      minimumHindStepLengthWhenSteppingUp = stepLength;
   }

   /** {@inheritDoc} */
   @Override
   public void setStepZForSteppingUp(double stepZ)
   {
      stepZForSteppingUp = stepZ;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumFrontStepLengthWhenSteppingDown(double stepLength)
   {
      maximumFrontStepLengthWhenSteppingDown = stepLength;
   }

   /** {@inheritDoc} */
   @Override
   public void setMinimumFrontStepLengthWhenSteppingDown(double stepLength)
   {
      minimumFrontStepLengthWhenSteppingDown = stepLength;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumHindStepLengthWhenSteppingDown(double stepLength)
   {
      maximumHindStepLengthWhenSteppingDown = stepLength;
   }

   /** {@inheritDoc} */
   @Override
   public void setMinimumHindStepLengthWhenSteppingDown(double stepLength)
   {
      minimumHindStepLengthWhenSteppingDown = stepLength;
   }

   /** {@inheritDoc} */
   @Override
   public void setStepZForSteppingDown(double stepZ)
   {
      stepZForSteppingDown = stepZ;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumStepOutward(double maximumStepOutward)
   {
      this.maximumStepOutward = maximumStepOutward;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumStepInward(double maximumStepInward)
   {
      this.maximumStepInward = maximumStepInward;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumStepYawInward(double maximumStepYawInward)
   {
      this.maximumStepYawInward = maximumStepYawInward;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumStepYawOutward(double maximumStepYawOutward)
   {
      this.maximumStepYawOutward = maximumStepYawOutward;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumStepChangeZ(double maximumStepChangeZ)
   {
      this.maximumStepChangeZ = maximumStepChangeZ;
   }

   /** {@inheritDoc} */
   @Override
   public void setBodyGroundClearance(double bodyGroundClearance)
   {
      this.bodyGroundClearance = bodyGroundClearance;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaxWalkingSpeedMultiplier(double maxWalkingSpeedMultiplier)
   {
      this.maxWalkingSpeedMultiplier = maxWalkingSpeedMultiplier;
   }

   @Override
   public void setDistanceWeight(double distanceHeuristicWeight)
   {
      this.distanceWeight = distanceHeuristicWeight;
   }

   /** {@inheritDoc} */
   @Override
   public void setYawWeight(double yawWeight)
   {
      this.yawWeight = yawWeight;
   }

   /** {@inheritDoc} */
   @Override
   public void setXGaitWeight(double xGaitWeight)
   {
      this.xGaitWeight = xGaitWeight;
   }

   /** {@inheritDoc} */
   @Override
   public void setDesiredVelocityWeight(double desiredVelocityWeight)
   {
      this.desiredVelocityWeight = desiredVelocityWeight;
   }

   /** {@inheritDoc} */
   @Override
   public void setCostPerStep(double costPerStep)
   {
      this.costPerStep = costPerStep;
   }

   /** {@inheritDoc} */
   @Override
   public void setStepUpWeight(double stepUpWeight)
   {
      this.stepUpWeight = stepUpWeight;
   }

   /** {@inheritDoc} */
   @Override
   public void setStepDownWeight(double stepDownWeight)
   {
      this.stepDownWeight = stepDownWeight;
   }

   /** {@inheritDoc} */
   @Override
   public void setHeuristicsInflationWeight(double heuristicsWeight)
   {
      this.heuristicsWeight = heuristicsWeight;
   }

   /** {@inheritDoc} */
   @Override
   public void setMinXClearanceFromFoot(double minXClearanceFromFoot)
   {
      this.minXClearanceFromFoot = minXClearanceFromFoot;
   }

   /** {@inheritDoc} */
   @Override
   public void setMinYClearanceFromFoot(double minYClearanceFromFoot)
   {
      this.minYClearanceFromFoot = minYClearanceFromFoot;
   }

   /** {@inheritDoc} */
   @Override
   public void setMinimumSurfaceInclineRadians(double minimumSurfaceInclineRadians)
   {
      this.minimumSurfaceInclineRadians = minimumSurfaceInclineRadians;
   }

   @Override
   public void setCliffHeightToAvoid(double cliffHeightToAvoid)
   {
      this.cliffHeightToAvoid = cliffHeightToAvoid;
   }

   @Override
   public void setMinimumFrontEndForwardDistanceFromCliffBottoms(double distance)
   {
      this.minimumFrontEndForwardDistanceFromCliffBottoms = distance;
   }

   @Override
   public void setMinimumFrontEndBackwardDistanceFromCliffBottoms(double distance)
   {
      this.minimumFrontEndBackwardDistanceFromCliffBottoms = distance;
   }

   @Override
   public void setMinimumHindEndForwardDistanceFromCliffBottoms(double distance)
   {
      this.minimumHindEndForwardDistanceFromCliffBottoms = distance;
   }

   @Override
   public void setMinimumHindEndBackwardDistanceFromCliffBottoms(double distance)
   {
      this.minimumHindEndBackwardDistanceFromCliffBottoms = distance;
   }

   @Override
   public void setMinimumLateralDistanceFromCliffBottoms(double distance)
   {
      this.minimumLateralDistanceFromCliffBottoms = distance;
   }

   @Override
   public void setFinalTurnProximity(double proximity)
   {
      this.finalTurnProximity = proximity;
   }

   @Override
   public void setFinalSlowDownProximity(double proximity)
   {
      this.finalSlowDownProximity = proximity;
   }

   /** {@inheritDoc} */
   @Override
   public void setProjectInsideDistanceForExpansion(double projectInsideDistance)
   {
      this.projectInsideDistanceForExpansion = projectInsideDistance;
   }

   /** {@inheritDoc} */
   @Override
   public void setProjectInsideDistanceForPostProcessing(double projectInsideDistance)
   {
      this.projectInsideDistanceForPostProcessing = projectInsideDistance;
   }

   /** {@inheritDoc} */
   @Override
   public void setProjectInsideUsingConvexHullDuringExpansion(boolean projectInsideUsingConvexHull)
   {
      this.projectInsideUsingConvexHullDuringExpansion = projectInsideUsingConvexHull;
   }

   /** {@inheritDoc} */
   @Override
   public void setProjectInsideUsingConvexHullDuringPostProcessing(boolean projectInsideUsingConvexHull)
   {
      this.projectInsideUsingConvexHullDuringPostProcessing = projectInsideUsingConvexHull;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumXYWiggleDistance(double maximumWiggleDistance)
   {
      this.maximumXYWiggleDistance = maximumWiggleDistance;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumFrontStepReach()
   {
      return maximumFrontStepReach;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumFrontStepLength()
   {
      return maximumFrontStepLength;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumFrontStepLength()
   {
      return minimumFrontStepLength;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumHindStepReach()
   {
      return maximumHindStepReach;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumHindStepLength()
   {
      return maximumHindStepLength;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumHindStepLength()
   {
      return minimumHindStepLength;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumFrontStepLengthWhenSteppingUp()
   {
      return maximumFrontStepLengthWhenSteppingUp;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumFrontStepLengthWhenSteppingUp()
   {
      return minimumFrontStepLengthWhenSteppingUp;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumHindStepLengthWhenSteppingUp()
   {
      return maximumHindStepLengthWhenSteppingUp;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumHindStepLengthWhenSteppingUp()
   {
      return minimumHindStepLengthWhenSteppingUp;
   }

   /** {@inheritDoc} */
   @Override
   public double getStepZForSteppingUp()
   {
      return stepZForSteppingUp;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumFrontStepLengthWhenSteppingDown()
   {
      return maximumFrontStepLengthWhenSteppingDown;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumFrontStepLengthWhenSteppingDown()
   {
      return minimumFrontStepLengthWhenSteppingDown;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumHindStepLengthWhenSteppingDown()
   {
      return maximumHindStepLengthWhenSteppingDown;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumHindStepLengthWhenSteppingDown()
   {
      return minimumHindStepLengthWhenSteppingDown;
   }

   /** {@inheritDoc} */
   @Override
   public double getStepZForSteppingDown()
   {
      return stepZForSteppingDown;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepOutward()
   {
      return maximumStepOutward;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepInward()
   {
      return maximumStepInward;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepYawInward()
   {
      return maximumStepYawInward;
   }

   /** {@inheritDoc} */
   public double getMaximumStepYawOutward()
   {
      return maximumStepYawOutward;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepChangeZ()
   {
      return maximumStepChangeZ;
   }

   /** {@inheritDoc} */
   @Override
   public double getBodyGroundClearance()
   {
      return bodyGroundClearance;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxWalkingSpeedMultiplier()
   {
      return maxWalkingSpeedMultiplier;
   }

   @Override
   public double getDistanceWeight()
   {
      return distanceWeight;
   }

   /** {@inheritDoc} */
   @Override
   public double getXGaitWeight()
   {
      return xGaitWeight;
   }

   /** {@inheritDoc} */
   @Override
   public double getDesiredVelocityWeight()
   {
      return desiredVelocityWeight;
   }

   /** {@inheritDoc} */
   @Override
   public double getYawWeight()
   {
      return yawWeight;
   }

   /** {@inheritDoc} */
   @Override
   public double getCostPerStep()
   {
      return costPerStep;
   }

   /** {@inheritDoc} */
   @Override
   public double getStepUpWeight()
   {
      return stepUpWeight;
   }

   /** {@inheritDoc} */
   @Override
   public double getStepDownWeight()
   {
      return stepDownWeight;
   }

   /** {@inheritDoc} */
   @Override
   public double getHeuristicsInflationWeight()
   {
      return heuristicsWeight;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinXClearanceFromFoot()
   {
      return minXClearanceFromFoot;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinYClearanceFromFoot()
   {
      return minYClearanceFromFoot;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumSurfaceInclineRadians()
   {
      return minimumSurfaceInclineRadians;
   }

   /** {@inheritDoc} */
   @Override
   public double getProjectInsideDistanceForExpansion()
   {
      return projectInsideDistanceForExpansion;
   }

   /** {@inheritDoc} */
   @Override
   public double getProjectInsideDistanceForPostProcessing()
   {
      return projectInsideDistanceForPostProcessing;
   }

   /** {@inheritDoc} */
   @Override
   public boolean getProjectInsideUsingConvexHullDuringExpansion()
   {
      return projectInsideUsingConvexHullDuringExpansion;
   }

   /** {@inheritDoc} */
   @Override
   public boolean getProjectInsideUsingConvexHullDuringPostProcessing()
   {
      return projectInsideUsingConvexHullDuringPostProcessing;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumXYWiggleDistance()
   {
      return maximumXYWiggleDistance;
   }

   /** {@inheritDoc} */
   @Override
   public double getCliffHeightToAvoid()
   {
      return cliffHeightToAvoid;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumFrontEndForwardDistanceFromCliffBottoms()
   {
      return minimumFrontEndForwardDistanceFromCliffBottoms;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumFrontEndBackwardDistanceFromCliffBottoms()
   {
      return minimumFrontEndBackwardDistanceFromCliffBottoms;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumHindEndForwardDistanceFromCliffBottoms()
   {
      return minimumHindEndForwardDistanceFromCliffBottoms;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumHindEndBackwardDistanceFromCliffBottoms()
   {
      return minimumHindEndBackwardDistanceFromCliffBottoms;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumLateralDistanceFromCliffBottoms()
   {
      return minimumLateralDistanceFromCliffBottoms;
   }

   /** {@inheritDoc} */
   @Override
   public double getFinalTurnProximity()
   {
      return finalTurnProximity;
   }

   /** {@inheritDoc} */
   @Override
   public double getFinalSlowDownProximity()
   {
      return finalSlowDownProximity;
   }
}
