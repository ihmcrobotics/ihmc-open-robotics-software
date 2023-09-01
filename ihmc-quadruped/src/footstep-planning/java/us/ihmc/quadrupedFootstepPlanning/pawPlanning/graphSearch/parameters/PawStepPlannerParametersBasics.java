
package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters;

import quadruped_msgs.msg.dds.PawStepPlannerParametersPacket;
import us.ihmc.tools.property.StoredPropertySetBasics;

import static us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParameterKeys.*;

public interface PawStepPlannerParametersBasics extends PawStepPlannerParametersReadOnly, StoredPropertySetBasics
{
   default void set(PawStepPlannerParametersReadOnly other)
   {
      setAll(other.getAll());
   }

   default void setMaximumFrontStepReach(double maximumStepReach)
   {
      set(maximumFrontStepReach, maximumStepReach);
   }

   default void setMaximumFrontStepLength(double maximumStepLength)
   {
      set(maximumFrontStepLength, maximumStepLength);
   }

   default void setMinimumFrontStepLength(double minimumStepLength)
   {
      set(minimumFrontStepLength, minimumStepLength);
   }

   default void setMaximumHindStepReach(double maximumStepReach)
   {
      set(maximumHindStepReach, maximumStepReach);
   }

   default void setMaximumHindStepLength(double maximumStepLength)
   {
      set(maximumHindStepLength, maximumStepLength);
   }

   default void setMinimumHindStepLength(double minimumStepLength)
   {
      set(minimumHindStepLength, minimumStepLength);
   }

   default void setMaximumStepOutward(double maximumStepOutward)
   {
      set(PawStepPlannerParameterKeys.maximumStepOutward, maximumStepOutward);
   }

   default void setMaximumStepInward(double maximumStepInward)
   {
      set(PawStepPlannerParameterKeys.maximumStepInward, maximumStepInward);
   }

   default void setMaximumFrontStepLengthWhenSteppingUp(double maximumStepLength)
   {
      set(maximumFrontStepLengthWhenSteppingUp, maximumStepLength);
   }

   default void setMinimumFrontStepLengthWhenSteppingUp(double minimumStepLength)
   {
      set(minimumFrontStepLengthWhenSteppingUp, minimumStepLength);
   }

   default void setMaximumHindStepLengthWhenSteppingUp(double maximumStepLength)
   {
      set(maximumHindStepLengthWhenSteppingUp, maximumStepLength);
   }

   default void setMinimumHindStepLengthWhenSteppingUp(double minimumStepLength)
   {
      set(minimumHindStepLengthWhenSteppingUp, minimumStepLength);
   }

   default void setStepZForSteppingUp(double stepZ)
   {
      set(stepZForSteppingUp, stepZ);
   }

   default void setMaximumFrontStepLengthWhenSteppingDown(double maximumStepLength)
   {
      set(maximumFrontStepLengthWhenSteppingDown, maximumStepLength);
   }

   default void setMinimumFrontStepLengthWhenSteppingDown(double minimumStepLength)
   {
      set(minimumFrontStepLengthWhenSteppingDown, minimumStepLength);
   }

   default void setMaximumHindStepLengthWhenSteppingDown(double maximumStepLength)
   {
      set(maximumHindStepLengthWhenSteppingDown, maximumStepLength);
   }

   default void setMinimumHindStepLengthWhenSteppingDown(double minimumStepLength)
   {
      set(minimumHindStepLengthWhenSteppingDown, minimumStepLength);
   }

   default void setStepZForSteppingDown(double stepZ)
   {
      set(stepZForSteppingDown, stepZ);
   }

   default void setMaximumStepYawInward(double maximumStepYawInward)
   {
      set(PawStepPlannerParameterKeys.maximumStepYawInward, maximumStepYawInward);
   }

   default void setMaximumStepYawOutward(double maximumStepYawOutward)
   {
      set(PawStepPlannerParameterKeys.maximumStepYawOutward, maximumStepYawOutward);
   }

   default void setMaximumStepChangeZ(double maximumStepChangeZ)
   {
      set(PawStepPlannerParameterKeys.maximumStepChangeZ, maximumStepChangeZ);
   }

   default void setBodyGroundClearance(double bodyGroundClearance)
   {
      set(PawStepPlannerParameterKeys.bodyGroundClearance, bodyGroundClearance);
   }

   default void setDistanceWeight(double distanceHeuristicWeight)
   {
      set(distanceWeight, distanceHeuristicWeight);
   }

   default void setYawWeight(double yawWeight)
   {
      set(PawStepPlannerParameterKeys.yawWeight, yawWeight);
   }

   default void setXGaitWeight(double xGaitWeight)
   {
      set(PawStepPlannerParameterKeys.xGaitWeight, xGaitWeight);
   }

   default void setDesiredVelocityWeight(double desiredVelocityWeight)
   {
      set(PawStepPlannerParameterKeys.desiredVelocityWeight, desiredVelocityWeight);
   }

   default void setCostPerStep(double costPerStep)
   {
      set(PawStepPlannerParameterKeys.costPerStep, costPerStep);
   }

   default void setStepUpWeight(double stepUpWeight)
   {
      set(PawStepPlannerParameterKeys.stepUpWeight, stepUpWeight);
   }

   default void setStepDownWeight(double stepDownWeight)
   {
      set(PawStepPlannerParameterKeys.stepDownWeight, stepDownWeight);
   }

   default void setHeuristicsInflationWeight(double heuristicsInflationWeight)
   {
      set(PawStepPlannerParameterKeys.heuristicsInflationWeight, heuristicsInflationWeight);
   }

   default void setMinXClearanceFromPaw(double minXClearanceFromFoot)
   {
      set(minXClearanceFromPaw, minXClearanceFromFoot);
   }

   default void setMinYClearanceFromPaw(double minYClearanceFromFoot)
   {
      set(minYClearanceFromPaw, minYClearanceFromFoot);
   }

   default void setMaxWalkingSpeedMultiplier(double multiplier)
   {
      set(maxWalkingSpeedMultiplier, multiplier);
   }

   default void setProjectInsideDistance(double projectionInsideDistanceForExpansion)
   {
      set(projectInsideDistance, projectionInsideDistanceForExpansion);
   }

   default void setMinimumProjectInsideDistance(double minimumProjectInsideDistance)
   {
      set(PawStepPlannerParameterKeys.minimumProjectInsideDistance, minimumProjectInsideDistance);
   }

   default void setProjectInsideUsingConvexHull(boolean projectInsideUsingConvexHull)
   {
      set(PawStepPlannerParameterKeys.projectInsideUsingConvexHull, projectInsideUsingConvexHull);
   }

   default void setMaximumXYWiggleDistance(double wiggleDistance)
   {
      set(maximumXYWiggleDistance, wiggleDistance);
   }

   default void setMinimumSurfaceInclineRadians(double minimumSurfaceIncline)
   {
      set(minimumSurfaceInclineRadians, minimumSurfaceIncline);
   }

   default void setCliffHeightToAvoid(double cliffHeightToAvoid)
   {
      set(PawStepPlannerParameterKeys.cliffHeightToAvoid, cliffHeightToAvoid);
   }

   default void setMinimumFrontEndForwardDistanceFromCliffBottoms(double distance)
   {
      set(minimumFrontEndForwardDistanceFromCliffBottoms, distance);
   }

   default void setMinimumFrontEndBackwardDistanceFromCliffBottoms(double distance)
   {
      set(minimumFrontEndBackwardDistanceFromCliffBottoms, distance);
   }

   default void setMinimumHindEndForwardDistanceFromCliffBottoms(double distance)
   {
      set(minimumHindEndForwardDistanceFromCliffBottoms, distance);
   }

   default void setMinimumHindEndBackwardDistanceFromCliffBottoms(double distance)
   {
      set(minimumHindEndBackwardDistanceFromCliffBottoms, distance);
   }

   default void setMinimumLateralDistanceFromCliffBottoms(double distance)
   {
      set(minimumLateralDistanceFromCliffBottoms, distance);
   }

   default void setFinalTurnProximity(double proximity)
   {
      set(finalTurnProximity, proximity);
   }

   default void setFinalSlowDownProximity(double proximity)
   {
      set(finalSlowDownProximity, proximity);
   }

   default void setMaximumDeviationFromXGaitDuringExpansion(double deviation)
   {
      set(maximumDeviationFromXGaitDuringExpansion, deviation);
   }

   default void setReturnBestEffortPlan(boolean returnBestEffortPlan)
   {
      set(PawStepPlannerParameterKeys.returnBestEffortPlan, returnBestEffortPlan);
   }

   default void setMinimumStepsForBestEffortPlan(int minimumStepsForBestEffortPlan)
   {
      set(PawStepPlannerParameterKeys.minStepsForBestEffortPlan, minimumStepsForBestEffortPlan);
   }

   default void setPerformGraphRepairingStep(boolean performGraphRepairingStep)
   {
      set(PawStepPlannerParameterKeys.performGraphRepairingStep, performGraphRepairingStep);
   }

   default void setRepairingHeuristicWeightScaling(double repairingHeuristicWeightScaling)
   {
      set(PawStepPlannerParameterKeys.repairingHeuristicWeightScaling, repairingHeuristicWeightScaling);
   }

   default void setMinimumHeuristicWeightReduction(double minimumHeuristicWeightReduction)
   {
      set(minHeuristicWeightReduction, minimumHeuristicWeightReduction);
   }

   default void set(PawStepPlannerParametersPacket other)
   {
      if (other.getMaximumFrontStepReach() != other.NO_VALUE_DOUBLE)
         setMaximumFrontStepReach(other.getMaximumFrontStepReach());
      if (other.getMaximumFrontStepLength() != other.NO_VALUE_DOUBLE)
         setMaximumFrontStepLength(other.getMaximumFrontStepLength());
      if (other.getMinimumFrontStepLength() != other.NO_VALUE_DOUBLE)
         setMinimumFrontStepLength(other.getMinimumFrontStepLength());
      if (other.getMaximumHindStepReach() != other.NO_VALUE_DOUBLE)
         setMaximumHindStepReach(other.getMaximumHindStepReach());
      if (other.getMaximumHindStepLength() != other.NO_VALUE_DOUBLE)
         setMaximumHindStepLength(other.getMaximumHindStepLength());
      if (other.getMinimumHindStepLength() != other.NO_VALUE_DOUBLE)
         setMinimumHindStepLength(other.getMinimumHindStepLength());
      if (other.getMaximumFrontStepLengthWhenSteppingUp() != other.NO_VALUE_DOUBLE)
         setMaximumFrontStepLengthWhenSteppingUp(other.getMaximumFrontStepLengthWhenSteppingUp());
      if (other.getMinimumFrontStepLengthWhenSteppingUp() != other.NO_VALUE_DOUBLE)
         setMinimumFrontStepLengthWhenSteppingUp(other.getMinimumFrontStepLengthWhenSteppingUp());
      if (other.getMaximumHindStepLengthWhenSteppingUp() != other.NO_VALUE_DOUBLE)
         setMaximumHindStepLengthWhenSteppingUp(other.getMaximumHindStepLengthWhenSteppingUp());
      if (other.getMinimumHindStepLengthWhenSteppingUp() != other.NO_VALUE_DOUBLE)
         setMinimumHindStepLengthWhenSteppingUp(other.getMinimumHindStepLengthWhenSteppingUp());
      if (other.getStepZForSteppingUp() != other.NO_VALUE_DOUBLE)
         setStepZForSteppingUp(other.getStepZForSteppingUp());
      if (other.getMaximumFrontStepLengthWhenSteppingDown() != other.NO_VALUE_DOUBLE)
         setMaximumFrontStepLengthWhenSteppingDown(other.getMaximumFrontStepLengthWhenSteppingDown());
      if (other.getMinimumFrontStepLengthWhenSteppingDown() != other.NO_VALUE_DOUBLE)
         setMinimumFrontStepLengthWhenSteppingDown(other.getMinimumFrontStepLengthWhenSteppingDown());
      if (other.getMaximumHindStepLengthWhenSteppingDown() != other.NO_VALUE_DOUBLE)
         setMaximumHindStepLengthWhenSteppingDown(other.getMaximumHindStepLengthWhenSteppingDown());
      if (other.getMinimumHindStepLengthWhenSteppingDown() != other.NO_VALUE_DOUBLE)
         setMinimumHindStepLengthWhenSteppingDown(other.getMinimumHindStepLengthWhenSteppingDown());
      if (other.getStepZForSteppingDown() != other.NO_VALUE_DOUBLE)
         setStepZForSteppingDown(other.getStepZForSteppingDown());
      if (other.getMaximumStepOutward() != other.NO_VALUE_DOUBLE)
         setMaximumStepOutward(other.getMaximumStepOutward());
      if (other.getMaximumStepInward() != other.NO_VALUE_DOUBLE)
         setMaximumStepInward(other.getMaximumStepInward());
      if (other.getMaximumStepYawInward() != other.NO_VALUE_DOUBLE)
         setMaximumStepYawInward(other.getMaximumStepYawInward());
      if (other.getMaximumStepYawOutward() != other.NO_VALUE_DOUBLE)
         setMaximumStepYawOutward(other.getMaximumStepYawOutward());
      if (other.getMaximumStepChangeZ() != other.NO_VALUE_DOUBLE)
         setMaximumStepChangeZ(other.getMaximumStepChangeZ());
      if (other.getBodyGroundClearance() != other.NO_VALUE_DOUBLE)
         setBodyGroundClearance(other.getBodyGroundClearance());
      if (other.getMaxWalkingSpeedMultiplier() != other.NO_VALUE_DOUBLE)
         setMaxWalkingSpeedMultiplier(other.getMaxWalkingSpeedMultiplier());
      if (other.getDistanceWeight() != other.NO_VALUE_DOUBLE)
         setDistanceWeight(other.getDistanceWeight());
      if (other.getYawWeight() != other.NO_VALUE_DOUBLE)
         setYawWeight(other.getYawWeight());
      if (other.getXGaitWeight() != other.NO_VALUE_DOUBLE)
         setXGaitWeight(other.getXGaitWeight());
      if (other.getDesiredVelocityWeight() != other.NO_VALUE_DOUBLE)
         setDesiredVelocityWeight(other.getDesiredVelocityWeight());
      if (other.getCostPerStep() != other.NO_VALUE_DOUBLE)
         setCostPerStep(other.getCostPerStep());
      if (other.getStepUpWeight() != other.NO_VALUE_DOUBLE)
         setStepUpWeight(other.getStepUpWeight());
      if (other.getStepDownWeight() != other.NO_VALUE_DOUBLE)
         setStepDownWeight(other.getStepDownWeight());
      if (other.getHeuristicsWeight() != other.NO_VALUE_DOUBLE)
         setHeuristicsInflationWeight(other.getHeuristicsWeight());
      if (other.getMinXClearanceFromPaw() != other.NO_VALUE_DOUBLE)
         setMinXClearanceFromPaw(other.getMinXClearanceFromPaw());
      if (other.getMinYClearanceFromPaw() != other.NO_VALUE_DOUBLE)
         setMinYClearanceFromPaw(other.getMinYClearanceFromPaw());
      if (other.getProjectionInsideDistance() != other.NO_VALUE_DOUBLE)
         setProjectInsideDistance(other.getProjectionInsideDistance());
      if (other.getMinimumProjectionInsideDistance() != other.NO_VALUE_DOUBLE)
         setMinimumProjectInsideDistance(other.getMinimumProjectionInsideDistance());
      if (other.getMaximumXyWiggleDistance() != other.NO_VALUE_DOUBLE)
         setMaximumXYWiggleDistance(other.getMaximumXyWiggleDistance());
      if (other.getMinimumSurfaceInclineRadians() != other.NO_VALUE_DOUBLE)
         setMinimumSurfaceInclineRadians(other.getMinimumSurfaceInclineRadians());
      if (other.getCliffHeightToAvoid() != other.NO_VALUE_DOUBLE)
         setCliffHeightToAvoid(other.getCliffHeightToAvoid());
      if (other.getMinimumFrontEndForwardDistanceFromCliffBottoms() != other.NO_VALUE_DOUBLE)
         setMinimumFrontEndForwardDistanceFromCliffBottoms(other.getMinimumFrontEndForwardDistanceFromCliffBottoms());
      if (other.getMinimumFrontEndBackwardDistanceFromCliffBottoms() != other.NO_VALUE_DOUBLE)
         setMinimumFrontEndBackwardDistanceFromCliffBottoms(other.getMinimumFrontEndBackwardDistanceFromCliffBottoms());
      if (other.getMinimumHindEndForwardDistanceFromCliffBottoms() != other.NO_VALUE_DOUBLE)
         setMinimumHindEndForwardDistanceFromCliffBottoms(other.getMinimumHindEndForwardDistanceFromCliffBottoms());
      if (other.getMinimumHindEndBackwardDistanceFromCliffBottoms() != other.NO_VALUE_DOUBLE)
         setMinimumHindEndBackwardDistanceFromCliffBottoms(other.getMinimumHindEndBackwardDistanceFromCliffBottoms());
      if (other.getMinimumLateralDistanceFromCliffBottoms() != other.NO_VALUE_DOUBLE)
         setMinimumLateralDistanceFromCliffBottoms(other.getMinimumLateralDistanceFromCliffBottoms());
      if (other.getFinalTurnProximity() != other.NO_VALUE_DOUBLE)
         setFinalTurnProximity(other.getFinalTurnProximity());
      if (other.getFinalSlowDownProximity() != other.NO_VALUE_DOUBLE)
         setFinalSlowDownProximity(other.getFinalSlowDownProximity());
      if (other.getMaximumDeviationFromXGaitDuringExpansion() != other.NO_VALUE_DOUBLE)
         setMaximumDeviationFromXGaitDuringExpansion(other.getMaximumDeviationFromXGaitDuringExpansion());
      if (other.getMinimumStepsForBestEffortPlan() != -1)
         setMinimumStepsForBestEffortPlan((int) other.getMinimumStepsForBestEffortPlan());
      if (other.getRepairingHeuristicWeightScaling() != -11.1)
         setRepairingHeuristicWeightScaling(other.getRepairingHeuristicWeightScaling());
      if (other.getMinimumHeuristicWeightReduction() != -11.1)
         setMinimumHeuristicWeightReduction(other.getMinimumHeuristicWeightReduction());

      setReturnBestEffortPlan(other.getReturnBestEffortPlan());
      setPerformGraphRepairingStep(other.getPerformGraphRepairingStep());
      setProjectInsideUsingConvexHull(other.getProjectInsideUsingConvexHull());
   }
}
