package us.ihmc.footstepPlanning.graphSearch.parameters;

import controller_msgs.msg.dds.FootstepPlannerParametersPacket;

public interface FootstepPlannerParametersBasics extends FootstepPlannerParametersReadOnly
{
   default void set(FootstepPlannerParametersReadOnly footstepPlannerParameters)
   {
      getStoredPropertySet().setAll(footstepPlannerParameters.getStoredPropertySet().getAll());
   }

   default void setCheckForBodyBoxCollisions(boolean checkForBodyBoxCollisions)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.checkForBodyBoxCollisions, checkForBodyBoxCollisions);
   }

   default void setPerformHeuristicSearchPolicies(boolean performHeuristicSearchPolicies)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.performHeuristicSearchPolicies, performHeuristicSearchPolicies);
   }

   default void setMinimumDistanceFromCliffBottoms(double distance)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.minimumDistanceFromCliffBottoms, distance);
   }

   default void setCliffHeightToAvoid(double height)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.cliffHeightToAvoid, height);
   }

   default void setMaximumStepReach(double reach)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maxStepReach, reach);
   }

   default void setMaximumStepWidth(double width)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maxStepWidth, width);
   }

   default void setMaximumStepYaw(double yaw)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maxStepYaw, yaw);
   }

   default void setMinimumStepYaw(double yaw)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.minStepYaw, yaw);
   }

   default void setMaximumStepZ(double stepZ)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maxStepZ, stepZ);
   }

   default void setMaximumXYWiggleDistance(double wiggleDistance)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maximumXYWiggleDistance, wiggleDistance);
   }

   default void setMaximumYawWiggle(double wiggleDistance)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maximumYawWiggle, wiggleDistance);
   }

   default void setMinimumFootholdPercent(double footholdPercent)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.minFootholdPercent, footholdPercent);
   }

   default void setMinimumStepLength(double minimumStepLength)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.minStepLength, minimumStepLength);
   }

   default void setMinimumStepWidth(double minimumStepWidth)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.minStepWidth, minimumStepWidth);
   }

   default void setMinimumSurfaceInclineRadians(double surfaceInclineRadians)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.minSurfaceIncline, surfaceInclineRadians);
   }

   default void setMinXClearanceFromStance(double clearance)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.minXClearanceFromStance, clearance);
   }

   default void setMinYClearanceFromStance(double clearance)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.minYClearanceFromStance, clearance);
   }

   default void setWiggleInsideDelta(double wiggleInsideDelta)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.wiggleInsideDelta, wiggleInsideDelta);
   }

   default void setMaximumStepZWhenSteppingUp(double maxStepZ)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maximumStepZWhenSteppingUp, maxStepZ);
   }

   default void setMaximumStepZWhenForwardAndDown(double maximumStepZWhenForwardAndDown)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maximumStepZWhenForwardAndDown, maximumStepZWhenForwardAndDown);
   }

   default void setMaximumStepReachWhenSteppingUp(double maximumStepReachWhenSteppingUp)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maximumStepReachWhenSteppingUp, maximumStepReachWhenSteppingUp);
   }

   default void setMaximumStepXWhenForwardAndDown(double maximumStepXWhenForwardAndDown)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maximumStepXWhenForwardAndDown, maximumStepXWhenForwardAndDown);
   }

   default void setIdealFootstepWidth(double idealFootstepWidth)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.idealFootstepWidth, idealFootstepWidth);
   }

   default void setIdealFootstepLength(double idealFootstepLength)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.idealFootstepLength, idealFootstepLength);
   }

   default void setWiggleIntoConvexHullOfPlanarRegions(boolean wiggleIntoConvexHullOfPlanarRegions)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.wiggleIntoConvexHullOfPlanarRegions, wiggleIntoConvexHullOfPlanarRegions);
   }

   default void setRejectIfCannotFullyWiggleInside(boolean rejectIfCannotFullyWiggleInside)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.rejectIfCannotFullyWiggleInside, rejectIfCannotFullyWiggleInside);
   }

   default void setMaximumZPenetrationOnValleyRegions(double maximumZPenetrationOnValleyRegions)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maximumZPenetrationOnValleyRegions, maximumZPenetrationOnValleyRegions);
   }

   default void setReturnBestEffortPlan(boolean returnBestEffortPlan)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.returnBestEffortPlan, returnBestEffortPlan);
   }

   default void setMinimumStepsForBestEffortPlan(int minimumStepForBestEffortPlan)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.minimumStepsForBestEffortPlan, minimumStepForBestEffortPlan);
   }

   default void setBodyGroundClearance(double bodyGroundClearance)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.bodyGroundClearance, bodyGroundClearance);
   }

   default void setBodyBoxHeight(double bodyBoxHeight)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.bodyBoxHeight, bodyBoxHeight);
   }

   default void setBodyBoxDepth(double bodyBoxDepth)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.bodyBoxDepth, bodyBoxDepth);
   }

   default void setBodyBoxWidth(double bodyBoxWidth)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.bodyBoxWidth, bodyBoxWidth);
   }

   default void setBodyBoxBaseX(double bodyBoxBaseX)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.bodyBoxBaseX, bodyBoxBaseX);
   }

   default void setBodyBoxBaseY(double bodyBoxBaseY)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.bodyBoxBaseY, bodyBoxBaseY);
   }

   default void setBodyBoxBaseZ(double bodyBoxBaseZ)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.bodyBoxBaseZ, bodyBoxBaseZ);
   }

   default void setUseQuadraticDistanceCost(boolean useQuadraticDistanceCost)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.useQuadraticDistanceCost, useQuadraticDistanceCost);
   }

   default void setUseQuadraticHeightCost(boolean useQuadraticHeightCost)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.useQuadraticHeightCost, useQuadraticHeightCost);
   }

   default void setAStarHeuristicsWeight(double aStarHeuristicsWeight)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.aStarHeuristicsWeight, aStarHeuristicsWeight);
   }

   default void setVisGraphWithAStarHeuristicsWeight(double visGraphWithAStarHeuristicsWeight)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.visGraphWithAStarHeuristicsWeight, visGraphWithAStarHeuristicsWeight);
   }

   default void setDepthFirstHeuristicsWeight(double depthFirstHeuristicsWeight)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.depthFirstHeuristicsWeight, depthFirstHeuristicsWeight);
   }

   default void setBodyPathBasedHeuristicWeight(double bodyPathBasedHeuristicWeight)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.bodyPathBasedHeuristicsWeight, bodyPathBasedHeuristicWeight);
   }

   default void setYawWeight(double yawWeight)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.yawWeight, yawWeight);
   }

   default void setPitchWeight(double pitchWeight)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.pitchWeight, pitchWeight);
   }

   default void setRollWeight(double rollWeight)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.rollWeight, rollWeight);
   }

   default void setForwardWeight(double forwardWeight)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.forwardWeight, forwardWeight);
   }

   default void setLateralWeight(double lateralWeight)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.lateralWeight, lateralWeight);
   }

   default void setStepUpWeight(double stepUpWeight)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.stepUpWeight, stepUpWeight);
   }

   default void setStepDownWeight(double stepDownWeight)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.stepDownWeight, stepDownWeight);
   }

   default void setCostPerStep(double costPerStep)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.costPerStep, costPerStep);
   }

   default void setMaximum2dDistanceFromBoundingBoxToPenalize(double maximum2dDistanceFromBoundingBoxToPenalize)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.maximum2dDistanceFromBoundingBoxToPenalize, maximum2dDistanceFromBoundingBoxToPenalize);
   }

   default void setBoundingBoxCost(double boundingBoxCost)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.boundingBoxCost, boundingBoxCost);
   }

   default void setFinalTurnProximity(double finalTurnProximity)
   {
      getStoredPropertySet().set(FootstepPlannerParameterKeys.finalTurnProximity, finalTurnProximity);
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
}