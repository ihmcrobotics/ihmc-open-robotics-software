package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters;

import controller_msgs.msg.dds.QuadrupedPawPlannerParametersPacket;
import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParameterKeys.*;

public interface PawPlannerParametersReadOnly extends StoredPropertySetReadOnly
{
   /**
    * The total maximum Euclidean distance length.
    */
   default double getMaximumFrontStepReach()
   {
      return get(maximumFrontStepReach);
   }

   default double getMaximumFrontStepLength()
   {
      return get(maximumFrontStepLength);
   }

   default double getMinimumFrontStepLength()
   {
      return get(minimumFrontStepLength);
   }

   default double getMaximumHindStepReach()
   {
      return get(maximumHindStepReach);
   }

   default double getMaximumHindStepLength()
   {
      return get(maximumHindStepLength);
   }

   default double getMinimumHindStepLength()
   {
      return get(minimumHindStepLength);
   }

   default double getMaximumStepOutward()
   {
      return get(maximumStepOutward);
   }

   default double getMaximumStepInward()
   {
      return get(maximumStepInward);
   }

   default double getMaximumFrontStepLengthWhenSteppingUp()
   {
      return get(maximumFrontStepLengthWhenSteppingUp);
   }

   default double getMinimumFrontStepLengthWhenSteppingUp()
   {
      return get(minimumFrontStepLengthWhenSteppingUp);
   }

   default double getMaximumHindStepLengthWhenSteppingUp()
   {
      return get(maximumHindStepLengthWhenSteppingUp);
   }

   default double getMinimumHindStepLengthWhenSteppingUp()
   {
      return get(minimumHindStepLengthWhenSteppingUp);
   }

   default double getStepZForSteppingUp()
   {
      return get(stepZForSteppingUp);
   }

   default double getMaximumFrontStepLengthWhenSteppingDown()
   {
      return get(maximumFrontStepLengthWhenSteppingDown);
   }

   default double getMinimumFrontStepLengthWhenSteppingDown()
   {
      return get(minimumFrontStepLengthWhenSteppingDown);
   }

   default double getMaximumHindStepLengthWhenSteppingDown()
   {
      return get(maximumHindStepLengthWhenSteppingDown);
   }

   default double getMinimumHindStepLengthWhenSteppingDown()
   {
      return get(minimumHindStepLengthWhenSteppingDown);
   }

   default double getStepZForSteppingDown()
   {
      return get(stepZForSteppingDown);
   }

   default double getMaximumStepYawInward()
   {
      return get(maximumStepYawInward);
   }

   default double getMaximumStepYawOutward()
   {
      return get(maximumStepYawOutward);
   }

   default double getMaximumStepChangeZ()
   {
      return get(maximumStepChangeZ);
   }

   default double getBodyGroundClearance()
   {
      return get(bodyGroundClearance);
   }

   default double getDistanceWeight()
   {
      return get(distanceWeight);
   }

   default double getYawWeight()
   {
      return get(yawWeight);
   }

   default double getXGaitWeight()
   {
      return get(xGaitWeight);
   }

   default double getDesiredVelocityWeight()
   {
      return get(desiredVelocityWeight);
   }

   default double getCostPerStep()
   {
      return get(costPerStep);
   }

   default double getStepUpWeight()
   {
      return get(stepUpWeight);
   }

   default double getStepDownWeight()
   {
      return get(stepDownWeight);
   }

   default double getHeuristicsInflationWeight()
   {
      return get(heuristicsInflationWeight);
   }

   default double getMinXClearanceFromPaw()
   {
      return get(minXClearanceFromPaw);
   }

   default double getMinYClearanceFromPaw()
   {
      return get(minYClearanceFromPaw);
   }

   default double getMaxWalkingSpeedMultiplier()
   {
      return get(maxWalkingSpeedMultiplier);
   }

   /**
    * Distance which a foothold is projected into planar region during expansion and node checking. Should be a positive value,
    * e.g. 0.02 means footholds are projected 2cm inside. If this is a non-positive value then no projection is performed.
    */
   default double getProjectInsideDistance()
   {
      return get(projectInsideDistance);
   }

   default boolean getProjectInsideUsingConvexHull()
   {
      return get(projectInsideUsingConvexHull);
   }

   /***
    * Maximum distance that the snap and wiggler is allowed to wiggle the footstep node.
    */
   default double getMaximumXYWiggleDistance()
   {
      return get(maximumXYWiggleDistance);
   }

   /**
    * The planner will ignore candidate footsteps if they are on a planar region with an incline that is higher
    * then the value specified here.
    *
    * <p>
    * More specifically, if a footstep has an associated planar region and that regions surface normal has a
    * z-value less than cos(minimumSurfaceInclineRadians), it will be rejected.
    * </p>
    */
   default double getMinimumSurfaceInclineRadians()
   {
      return get(minimumSurfaceInclineRadians);
   }

   /**
    * The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
    * nearby that is cliffHeightToShiftAwayFrom higher than the candidate footstep, it will move away from it
    * until it is minimumDistanceFromCliffBottoms away from it.
    *
    * <p>
    * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
    * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
    * generator is capable of swinging over.
    * </p>
    */
   default double getCliffHeightToAvoid()
   {
      return get(cliffHeightToAvoid);
   }

   /**
    * The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
    * nearby that is {@link #getCliffHeightToAvoid} higher than the candidate footstep, it will move away from it
    * until it is minimumDistanceFromCliffBottoms away from it.
    *
    * <p>
    * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
    * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
    * generator is capable of swinging over.
    * </p>
    */
   default double getMinimumFrontEndForwardDistanceFromCliffBottoms()
   {
      return get(minimumFrontEndForwardDistanceFromCliffBottoms);
   }

   default double getMinimumFrontEndBackwardDistanceFromCliffBottoms()
   {
      return get(minimumFrontEndBackwardDistanceFromCliffBottoms);
   }

   default double getMinimumHindEndForwardDistanceFromCliffBottoms()
   {
      return get(minimumHindEndForwardDistanceFromCliffBottoms);
   }

   default double getMinimumHindEndBackwardDistanceFromCliffBottoms()
   {
      return get(minimumHindEndBackwardDistanceFromCliffBottoms);
   }

   default double getMinimumLateralDistanceFromCliffBottoms()
   {
      return get(minimumLateralDistanceFromCliffBottoms);
   }

   default double getFinalTurnProximity()
   {
      return get(finalTurnProximity);
   }

   default double getFinalSlowDownProximity()
   {
      return get(finalSlowDownProximity);
   }

   default double getMaximumDeviationFromXGaitDuringExpansion()
   {
      return get(maximumDeviationFromXGaitDuringExpansion);
   }

   default boolean returnBestEffortPlan()
   {
      return get(returnBestEffortPlan);
   }

   default int getMinimumStepsForBestEffortPlan()
   {
      return get(minStepsForBestEffortPlan);
   }

   default boolean performGraphRepairingStep()
   {
      return get(performGraphRepairingStep);
   }

   default double getRepairingHeuristicWeightScaling()
   {
      return get(repairingHeuristicWeightScaling);
   }

   default double getMinimumHeuristicWeightReduction()
   {
      return get(minHeuristicWeightReduction);
   }

   default QuadrupedPawPlannerParametersPacket getAsPacket()
   {
      QuadrupedPawPlannerParametersPacket packet = new QuadrupedPawPlannerParametersPacket();
      packet.setMaximumFrontStepReach(getMaximumFrontStepReach());
      packet.setMaximumFrontStepLength(getMaximumFrontStepLength());
      packet.setMinimumFrontStepLength(getMinimumFrontStepLength());
      packet.setMaximumHindStepReach(getMaximumHindStepReach());
      packet.setMaximumHindStepLength(getMaximumHindStepLength());
      packet.setMinimumHindStepLength(getMinimumHindStepLength());
      packet.setMaximumFrontStepLengthWhenSteppingUp(getMaximumFrontStepLengthWhenSteppingUp());
      packet.setMinimumFrontStepLengthWhenSteppingUp(getMinimumFrontStepLengthWhenSteppingUp());
      packet.setMaximumHindStepLengthWhenSteppingUp(getMaximumHindStepLengthWhenSteppingUp());
      packet.setMinimumHindStepLengthWhenSteppingUp(getMinimumHindStepLengthWhenSteppingUp());
      packet.setStepZForSteppingUp(getStepZForSteppingUp());
      packet.setMaximumFrontStepLengthWhenSteppingDown(getMaximumFrontStepLengthWhenSteppingDown());
      packet.setMinimumFrontStepLengthWhenSteppingDown(getMinimumFrontStepLengthWhenSteppingDown());
      packet.setMaximumHindStepLengthWhenSteppingDown(getMaximumHindStepLengthWhenSteppingDown());
      packet.setMinimumHindStepLengthWhenSteppingDown(getMinimumHindStepLengthWhenSteppingDown());
      packet.setStepZForSteppingDown(getStepZForSteppingDown());
      packet.setMaximumStepOutward(getMaximumStepOutward());
      packet.setMaximumStepInward(getMaximumStepInward());
      packet.setMaximumStepYawInward(getMaximumStepYawInward());
      packet.setMaximumStepYawOutward(getMaximumStepYawOutward());
      packet.setMaximumStepChangeZ(getMaximumStepChangeZ());
      packet.setBodyGroundClearance(getBodyGroundClearance());
      packet.setMaxWalkingSpeedMultiplier(getMaxWalkingSpeedMultiplier());
      packet.setDistanceWeight(getDistanceWeight());
      packet.setYawWeight(getYawWeight());
      packet.setXGaitWeight(getXGaitWeight());
      packet.setDesiredVelocityWeight(getDesiredVelocityWeight());
      packet.setCostPerStep(getCostPerStep());
      packet.setStepUpWeight(getStepUpWeight());
      packet.setStepDownWeight(getStepDownWeight());
      packet.setHeuristicsWeight(getHeuristicsInflationWeight());
      packet.setMinXClearanceFromPaw(getMinXClearanceFromPaw());
      packet.setMinYClearanceFromPaw(getMinYClearanceFromPaw());
      packet.setProjectionInsideDistance(getProjectInsideDistance());
      packet.setProjectInsideUsingConvexHull(getProjectInsideUsingConvexHull());
      packet.setMaximumXyWiggleDistance(getMaximumXYWiggleDistance());
      packet.setMinimumSurfaceInclineRadians(getMinimumSurfaceInclineRadians());
      packet.setCliffHeightToAvoid(getCliffHeightToAvoid());
      packet.setMinimumFrontEndForwardDistanceFromCliffBottoms(getMinimumFrontEndForwardDistanceFromCliffBottoms());
      packet.setMinimumFrontEndBackwardDistanceFromCliffBottoms(getMinimumFrontEndBackwardDistanceFromCliffBottoms());
      packet.setMinimumHindEndForwardDistanceFromCliffBottoms(getMinimumHindEndForwardDistanceFromCliffBottoms());
      packet.setMinimumHindEndBackwardDistanceFromCliffBottoms(getMinimumHindEndBackwardDistanceFromCliffBottoms());
      packet.setMinimumLateralDistanceFromCliffBottoms(getMinimumLateralDistanceFromCliffBottoms());
      packet.setFinalTurnProximity(getFinalTurnProximity());
      packet.setFinalSlowDownProximity(getFinalSlowDownProximity());
      packet.setMaximumDeviationFromXGaitDuringExpansion(getMaximumDeviationFromXGaitDuringExpansion());
      packet.setReturnBestEffortPlan(returnBestEffortPlan());
      packet.setMinimumStepsForBestEffortPlan(getMinimumStepsForBestEffortPlan());
      packet.setPerformGraphRepairingStep(performGraphRepairingStep());
      packet.setRepairingHeuristicWeightScaling(getRepairingHeuristicWeightScaling());
      packet.setMinimumHeuristicWeightReduction(getMinimumHeuristicWeightReduction());

      return packet;
   }
}
