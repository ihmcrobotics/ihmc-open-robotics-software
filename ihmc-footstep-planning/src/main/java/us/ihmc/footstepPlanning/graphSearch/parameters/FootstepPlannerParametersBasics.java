package us.ihmc.footstepPlanning.graphSearch.parameters;

import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import us.ihmc.tools.property.StoredPropertySetBasics;

public interface FootstepPlannerParametersBasics extends FootstepPlannerParametersReadOnly, StoredPropertySetBasics
{
   default void set(FootstepPlannerParametersReadOnly footstepPlannerParameters)
   {
      setAll(footstepPlannerParameters.getAll());
   }

   default void setCheckForBodyBoxCollisions(boolean checkForBodyBoxCollisions)
   {
      set(FootstepPlannerParameterKeys.checkForBodyBoxCollisions, checkForBodyBoxCollisions);
   }

   default void setPerformHeuristicSearchPolicies(boolean performHeuristicSearchPolicies)
   {
      set(FootstepPlannerParameterKeys.performHeuristicSearchPolicies, performHeuristicSearchPolicies);
   }

   default void setMinimumDistanceFromCliffBottoms(double distance)
   {
      set(FootstepPlannerParameterKeys.minimumDistanceFromCliffBottoms, distance);
   }

   default void setCliffHeightToAvoid(double height)
   {
      set(FootstepPlannerParameterKeys.cliffHeightToAvoid, height);
   }

   default void setMaximumStepReach(double reach)
   {
      set(FootstepPlannerParameterKeys.maxStepReach, reach);
   }

   default void setMaximumStepWidth(double width)
   {
      set(FootstepPlannerParameterKeys.maxStepWidth, width);
   }

   default void setMaximumStepYaw(double yaw)
   {
      set(FootstepPlannerParameterKeys.maxStepYaw, yaw);
   }

   default void setMinimumStepYaw(double yaw)
   {
      set(FootstepPlannerParameterKeys.minStepYaw, yaw);
   }

   default void setMaximumStepZ(double stepZ)
   {
      set(FootstepPlannerParameterKeys.maxStepZ, stepZ);
   }

   default void setMaximumXYWiggleDistance(double wiggleDistance)
   {
      set(FootstepPlannerParameterKeys.maximumXYWiggleDistance, wiggleDistance);
   }

   default void setMaximumYawWiggle(double wiggleDistance)
   {
      set(FootstepPlannerParameterKeys.maximumYawWiggle, wiggleDistance);
   }

   default void setMinimumFootholdPercent(double footholdPercent)
   {
      set(FootstepPlannerParameterKeys.minFootholdPercent, footholdPercent);
   }

   default void setMinimumStepLength(double minimumStepLength)
   {
      set(FootstepPlannerParameterKeys.minStepLength, minimumStepLength);
   }

   default void setMinimumStepWidth(double minimumStepWidth)
   {
      set(FootstepPlannerParameterKeys.minStepWidth, minimumStepWidth);
   }

   default void setMinimumSurfaceInclineRadians(double surfaceInclineRadians)
   {
      set(FootstepPlannerParameterKeys.minSurfaceIncline, surfaceInclineRadians);
   }

   default void setMinXClearanceFromStance(double clearance)
   {
      set(FootstepPlannerParameterKeys.minXClearanceFromStance, clearance);
   }

   default void setMinYClearanceFromStance(double clearance)
   {
      set(FootstepPlannerParameterKeys.minYClearanceFromStance, clearance);
   }

   default void setWiggleInsideDelta(double wiggleInsideDelta)
   {
      set(FootstepPlannerParameterKeys.wiggleInsideDelta, wiggleInsideDelta);
   }

   default void setMaximumStepZWhenSteppingUp(double maxStepZ)
   {
      set(FootstepPlannerParameterKeys.maximumStepZWhenSteppingUp, maxStepZ);
   }

   default void setMaximumStepZWhenForwardAndDown(double maximumStepZWhenForwardAndDown)
   {
      set(FootstepPlannerParameterKeys.maximumStepZWhenForwardAndDown, maximumStepZWhenForwardAndDown);
   }

   default void setMaximumStepReachWhenSteppingUp(double maximumStepReachWhenSteppingUp)
   {
      set(FootstepPlannerParameterKeys.maximumStepReachWhenSteppingUp, maximumStepReachWhenSteppingUp);
   }

   default void setMaximumStepXWhenForwardAndDown(double maximumStepXWhenForwardAndDown)
   {
      set(FootstepPlannerParameterKeys.maximumStepXWhenForwardAndDown, maximumStepXWhenForwardAndDown);
   }

   default void setIdealFootstepWidth(double idealFootstepWidth)
   {
      set(FootstepPlannerParameterKeys.idealFootstepWidth, idealFootstepWidth);
   }

   default void setIdealFootstepLength(double idealFootstepLength)
   {
      set(FootstepPlannerParameterKeys.idealFootstepLength, idealFootstepLength);
   }

   default void setWiggleIntoConvexHullOfPlanarRegions(boolean wiggleIntoConvexHullOfPlanarRegions)
   {
      set(FootstepPlannerParameterKeys.wiggleIntoConvexHullOfPlanarRegions, wiggleIntoConvexHullOfPlanarRegions);
   }

   default void setRejectIfCannotFullyWiggleInside(boolean rejectIfCannotFullyWiggleInside)
   {
      set(FootstepPlannerParameterKeys.rejectIfCannotFullyWiggleInside, rejectIfCannotFullyWiggleInside);
   }

   default void setMaximumZPenetrationOnValleyRegions(double maximumZPenetrationOnValleyRegions)
   {
      set(FootstepPlannerParameterKeys.maximumZPenetrationOnValleyRegions, maximumZPenetrationOnValleyRegions);
   }

   default void setReturnBestEffortPlan(boolean returnBestEffortPlan)
   {
      set(FootstepPlannerParameterKeys.returnBestEffortPlan, returnBestEffortPlan);
   }

   default void setMinimumStepsForBestEffortPlan(int minimumStepForBestEffortPlan)
   {
      set(FootstepPlannerParameterKeys.minimumStepsForBestEffortPlan, minimumStepForBestEffortPlan);
   }

   default void setBodyGroundClearance(double bodyGroundClearance)
   {
      set(FootstepPlannerParameterKeys.bodyGroundClearance, bodyGroundClearance);
   }

   default void setBodyBoxHeight(double bodyBoxHeight)
   {
      set(FootstepPlannerParameterKeys.bodyBoxHeight, bodyBoxHeight);
   }

   default void setBodyBoxDepth(double bodyBoxDepth)
   {
      set(FootstepPlannerParameterKeys.bodyBoxDepth, bodyBoxDepth);
   }

   default void setBodyBoxWidth(double bodyBoxWidth)
   {
      set(FootstepPlannerParameterKeys.bodyBoxWidth, bodyBoxWidth);
   }

   default void setBodyBoxBaseX(double bodyBoxBaseX)
   {
      set(FootstepPlannerParameterKeys.bodyBoxBaseX, bodyBoxBaseX);
   }

   default void setBodyBoxBaseY(double bodyBoxBaseY)
   {
      set(FootstepPlannerParameterKeys.bodyBoxBaseY, bodyBoxBaseY);
   }

   default void setBodyBoxBaseZ(double bodyBoxBaseZ)
   {
      set(FootstepPlannerParameterKeys.bodyBoxBaseZ, bodyBoxBaseZ);
   }

   default void setUseQuadraticDistanceCost(boolean useQuadraticDistanceCost)
   {
      set(FootstepPlannerParameterKeys.useQuadraticDistanceCost, useQuadraticDistanceCost);
   }

   default void setUseQuadraticHeightCost(boolean useQuadraticHeightCost)
   {
      set(FootstepPlannerParameterKeys.useQuadraticHeightCost, useQuadraticHeightCost);
   }

   default void setAStarHeuristicsWeight(double aStarHeuristicsWeight)
   {
      set(FootstepPlannerParameterKeys.aStarHeuristicsWeight, aStarHeuristicsWeight);
   }

   default void setVisGraphWithAStarHeuristicsWeight(double visGraphWithAStarHeuristicsWeight)
   {
      set(FootstepPlannerParameterKeys.visGraphWithAStarHeuristicsWeight, visGraphWithAStarHeuristicsWeight);
   }

   default void setDepthFirstHeuristicsWeight(double depthFirstHeuristicsWeight)
   {
      set(FootstepPlannerParameterKeys.depthFirstHeuristicsWeight, depthFirstHeuristicsWeight);
   }

   default void setBodyPathBasedHeuristicWeight(double bodyPathBasedHeuristicWeight)
   {
      set(FootstepPlannerParameterKeys.bodyPathBasedHeuristicsWeight, bodyPathBasedHeuristicWeight);
   }

   default void setYawWeight(double yawWeight)
   {
      set(FootstepPlannerParameterKeys.yawWeight, yawWeight);
   }

   default void setPitchWeight(double pitchWeight)
   {
      set(FootstepPlannerParameterKeys.pitchWeight, pitchWeight);
   }

   default void setRollWeight(double rollWeight)
   {
      set(FootstepPlannerParameterKeys.rollWeight, rollWeight);
   }

   default void setForwardWeight(double forwardWeight)
   {
      set(FootstepPlannerParameterKeys.forwardWeight, forwardWeight);
   }

   default void setLateralWeight(double lateralWeight)
   {
      set(FootstepPlannerParameterKeys.lateralWeight, lateralWeight);
   }

   default void setStepUpWeight(double stepUpWeight)
   {
      set(FootstepPlannerParameterKeys.stepUpWeight, stepUpWeight);
   }

   default void setStepDownWeight(double stepDownWeight)
   {
      set(FootstepPlannerParameterKeys.stepDownWeight, stepDownWeight);
   }

   default void setCostPerStep(double costPerStep)
   {
      set(FootstepPlannerParameterKeys.costPerStep, costPerStep);
   }

   default void setMaximum2dDistanceFromBoundingBoxToPenalize(double maximum2dDistanceFromBoundingBoxToPenalize)
   {
      set(FootstepPlannerParameterKeys.maximum2dDistanceFromBoundingBoxToPenalize, maximum2dDistanceFromBoundingBoxToPenalize);
   }

   default void setBoundingBoxCost(double boundingBoxCost)
   {
      set(FootstepPlannerParameterKeys.boundingBoxCost, boundingBoxCost);
   }

   default void setFinalTurnProximity(double finalTurnProximity)
   {
      set(FootstepPlannerParameterKeys.finalTurnProximity, finalTurnProximity);
   }

   default void setFinalTurnProximityBlendFactor(double finalTurnProximityBlendFactor)
   {
      set(FootstepPlannerParameterKeys.finalTurnProximityBlendFactor, finalTurnProximityBlendFactor);
   }

   default void setFootholdAreaWeight(double footholdAreaWeight)
   {
      set(FootstepPlannerParameterKeys.footholdAreaWeight, footholdAreaWeight);
   }

   default void setLongStepWeight(double weight)
   {
      set(FootstepPlannerParameterKeys.longStepWeight, weight);
   }

   default void set(FootstepPlannerParametersPacket parametersPacket)
   {
      setCheckForBodyBoxCollisions(parametersPacket.getCheckForBodyBoxCollisions());
      setPerformHeuristicSearchPolicies(parametersPacket.getPerformHeuristicSearchPolicies());
      if (parametersPacket.getIdealFootstepWidth() != -1.0)
         setIdealFootstepWidth(parametersPacket.getIdealFootstepWidth());
      if (parametersPacket.getIdealFootstepLength() != -1.0)
         setIdealFootstepLength(parametersPacket.getIdealFootstepLength());
      if (parametersPacket.getWiggleInsideDelta() != -1.0)
         setWiggleInsideDelta(parametersPacket.getWiggleInsideDelta());
      if (parametersPacket.getMaximumStepReach() != -1.0)
         setMaximumStepReach(parametersPacket.getMaximumStepReach());
      if (parametersPacket.getMaximumStepYaw() != 1.0)
         setMaximumStepYaw(parametersPacket.getMaximumStepYaw());
      if (parametersPacket.getMinimumStepWidth() != 1.0)
         setMinimumStepWidth(parametersPacket.getMinimumStepWidth());
      if (parametersPacket.getMinimumStepLength() != -1.0)
         setMinimumStepLength(parametersPacket.getMinimumStepLength());
      if (parametersPacket.getMinimumStepYaw() != -1.0)
         setMinimumStepYaw(parametersPacket.getMinimumStepYaw());
      if (parametersPacket.getMaximumStepReachWhenSteppingUp() != -1.0)
         setMaximumStepReachWhenSteppingUp(parametersPacket.getMaximumStepReachWhenSteppingUp());
      if (parametersPacket.getMaximumStepZWhenSteppingUp() != -1.0)
         setMaximumStepZWhenSteppingUp(parametersPacket.getMaximumStepZWhenSteppingUp());
      if (parametersPacket.getMaximumStepXWhenForwardAndDown() != -1.0)
         setMaximumStepXWhenForwardAndDown(parametersPacket.getMaximumStepXWhenForwardAndDown());
      if (parametersPacket.getMaximumStepZWhenForwardAndDown() != -1.0)
         setMaximumStepZWhenForwardAndDown(parametersPacket.getMaximumStepZWhenForwardAndDown());
      if (parametersPacket.getMaximumStepZ() != -1.0)
         setMaximumStepZ(parametersPacket.getMaximumStepZ());
      if (parametersPacket.getMinimumFootholdPercent() != -1.0)
         setMinimumFootholdPercent(parametersPacket.getMinimumFootholdPercent());
      if (parametersPacket.getMinimumSurfaceInclineRadians() != -1.0)
         setMinimumSurfaceInclineRadians(parametersPacket.getMinimumSurfaceInclineRadians());
      setWiggleIntoConvexHullOfPlanarRegions(parametersPacket.getWiggleIntoConvexHullOfPlanarRegions());
      setRejectIfCannotFullyWiggleInside(parametersPacket.getRejectIfCannotFullyWiggleInside());
      if (parametersPacket.getMaximumXyWiggleDistance() != -1.0)
         setMaximumXYWiggleDistance(parametersPacket.getMaximumXyWiggleDistance());
      if (parametersPacket.getMaximumYawWiggle() != -1.0)
         setMaximumYawWiggle(parametersPacket.getMaximumYawWiggle());
      if (parametersPacket.getMaximumZPenetrationOnValleyRegions() != -1.0)
         setMaximumZPenetrationOnValleyRegions(parametersPacket.getMaximumZPenetrationOnValleyRegions());
      if (parametersPacket.getMaximumStepWidth() != -1.0)
         setMaximumStepWidth(parametersPacket.getMaximumStepWidth());
      if (parametersPacket.getCliffHeightToAvoid() != -1.0)
         setCliffHeightToAvoid(parametersPacket.getCliffHeightToAvoid());
      if (parametersPacket.getMinimumDistanceFromCliffBottoms() != -1.0)
         setMinimumDistanceFromCliffBottoms(parametersPacket.getMinimumDistanceFromCliffBottoms());
      setReturnBestEffortPlan(parametersPacket.getReturnBestEffortPlan());
      if (parametersPacket.getMinimumStepsForBestEffortPlan() > 0)
         setMinimumStepsForBestEffortPlan((int) parametersPacket.getMinimumStepsForBestEffortPlan());
      if (parametersPacket.getBodyGroundClearance() != -1.0)
         setBodyGroundClearance(parametersPacket.getBodyGroundClearance());
      if (parametersPacket.getBodyBoxHeight() != -1.0)
         setBodyBoxHeight(parametersPacket.getBodyBoxHeight());
      if (parametersPacket.getBodyBoxDepth() != -1.0)
         setBodyBoxDepth(parametersPacket.getBodyBoxDepth());
      if (parametersPacket.getBodyBoxWidth() != -1.0)
         setBodyBoxWidth(parametersPacket.getBodyBoxWidth());
      if (parametersPacket.getBodyBoxBaseX() != -1.0)
         setBodyBoxBaseX(parametersPacket.getBodyBoxBaseX());
      if (parametersPacket.getBodyBoxBaseY() != -1.0)
         setBodyBoxBaseY(parametersPacket.getBodyBoxBaseY());
      if (parametersPacket.getBodyBoxBaseZ() != -1.0)
         setBodyBoxBaseZ(parametersPacket.getBodyBoxBaseZ());
      if (parametersPacket.getMinXClearanceFromStance() != -1.0)
         setMinXClearanceFromStance(parametersPacket.getMinXClearanceFromStance());
      if (parametersPacket.getMinYClearanceFromStance() != -1.0)
         setMinYClearanceFromStance(parametersPacket.getMinYClearanceFromStance());
      if (parametersPacket.getFinalTurnProximity() != -1.0)
         setFinalTurnProximity(parametersPacket.getFinalTurnProximity());
      if (parametersPacket.getFinalTurnProximityBlendFactor() != -1.0)
         setFinalTurnProximityBlendFactor(parametersPacket.getFinalTurnProximityBlendFactor());

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

      if (parametersPacket.getFootholdAreaWeight() != -1.0)
         setFootholdAreaWeight(parametersPacket.getFootholdAreaWeight());
      if (parametersPacket.getLongStepWeight() != -1.0)
         setLongStepWeight(parametersPacket.getLongStepWeight());
   }
}