package us.ihmc.footstepPlanning.graphSearch.parameters;

import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import us.ihmc.tools.property.StoredPropertySetBasics;

import static us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys.bodyPathViolationWeight;
import static us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys.deltaYawFromReferenceTolerance;
import static us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys.distanceFromPathTolerance;

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

   default void setCheckForPathCollisions(boolean checkForPathCollisions)
   {
      set(FootstepPlannerParameterKeys.checkForPathCollisions, checkForPathCollisions);
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

   default void setStepYawReductionFactorAtMaxReach(double factor)
   {
      set(FootstepPlannerParameterKeys.stepYawReductionFactorAtMaxReach, factor);
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

   default void setMaximumStepReachWhenSteppingUp(double maximumStepReachWhenSteppingUp)
   {
      set(FootstepPlannerParameterKeys.maximumStepReachWhenSteppingUp, maximumStepReachWhenSteppingUp);
   }

   default void setMaximumStepWidthWhenSteppingUp(double maximumStepWidthWhenSteppingUp)
   {
      set(FootstepPlannerParameterKeys.maximumStepWidthWhenSteppingUp, maximumStepWidthWhenSteppingUp);
   }

   default void setMaximumStepZWhenForwardAndDown(double maximumStepZWhenForwardAndDown)
   {
      set(FootstepPlannerParameterKeys.maximumStepZWhenForwardAndDown, maximumStepZWhenForwardAndDown);
   }

   default void setMaximumStepXWhenForwardAndDown(double maximumStepXWhenForwardAndDown)
   {
      set(FootstepPlannerParameterKeys.maximumStepXWhenForwardAndDown, maximumStepXWhenForwardAndDown);
   }

   default void setMaximumStepYWhenForwardAndDown(double maximumStepYWhenForwardAndDown)
   {
      set(FootstepPlannerParameterKeys.maximumStepYWhenForwardAndDown, maximumStepYWhenForwardAndDown);
   }

   default void setTranslationScaleFromGrandparentNode(double translationScaleFromGrandparentNode)
   {
      set(FootstepPlannerParameterKeys.translationScaleFromGrandparentNode, translationScaleFromGrandparentNode);
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

   default void setNumberOfBoundingBoxChecks(int numberOfBoundingBoxChecks)
   {
      set(FootstepPlannerParameterKeys.numberOfBoundingBoxChecks, numberOfBoundingBoxChecks);
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

   default void setFinalTurnBodyPathProximity(double finalTurnProximity)
   {
      set(FootstepPlannerParameterKeys.finalTurnBodyPathProximity, finalTurnProximity);
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

   default void setBodyPathViolationWeight(double weight)
   {
      set(FootstepPlannerParameterKeys.bodyPathViolationWeight, weight);
   }

   default void setDistanceFromPathTolerance(double tolerance)
   {
      set(distanceFromPathTolerance, tolerance);
   }

   default void setDeltaYawFromReferenceTolerance(double tolerance)
   {
      set(deltaYawFromReferenceTolerance, tolerance);
   }

   default void set(FootstepPlannerParametersPacket parametersPacket)
   {
      double noValue = FootstepPlannerParametersPacket.DEFAULT_NO_VALUE;
      setCheckForBodyBoxCollisions(parametersPacket.getCheckForBodyBoxCollisions());
      setCheckForPathCollisions(parametersPacket.getCheckForPathCollisions());
      setNumberOfBoundingBoxChecks((int) parametersPacket.getNumberOfBoundingBoxChecks());
      setPerformHeuristicSearchPolicies(parametersPacket.getPerformHeuristicSearchPolicies());
      if (parametersPacket.getIdealFootstepWidth() != noValue)
         setIdealFootstepWidth(parametersPacket.getIdealFootstepWidth());
      if (parametersPacket.getIdealFootstepLength() != noValue)
         setIdealFootstepLength(parametersPacket.getIdealFootstepLength());
      if (parametersPacket.getWiggleInsideDelta() != noValue)
         setWiggleInsideDelta(parametersPacket.getWiggleInsideDelta());
      if (parametersPacket.getMaximumStepReach() != noValue)
         setMaximumStepReach(parametersPacket.getMaximumStepReach());
      if (parametersPacket.getMaximumStepYaw() != noValue)
         setMaximumStepYaw(parametersPacket.getMaximumStepYaw());
      if (parametersPacket.getMinimumStepWidth() != noValue)
         setMinimumStepWidth(parametersPacket.getMinimumStepWidth());
      if (parametersPacket.getMinimumStepLength() != noValue)
         setMinimumStepLength(parametersPacket.getMinimumStepLength());
      if (parametersPacket.getMinimumStepYaw() != noValue)
         setMinimumStepYaw(parametersPacket.getMinimumStepYaw());
      if (parametersPacket.getMaximumStepReachWhenSteppingUp() != noValue)
         setMaximumStepReachWhenSteppingUp(parametersPacket.getMaximumStepReachWhenSteppingUp());
      if (parametersPacket.getMaximumStepWidthWhenSteppingUp() != noValue)
         setMaximumStepWidthWhenSteppingUp(parametersPacket.getMaximumStepWidthWhenSteppingUp());
      if (parametersPacket.getMaximumStepZWhenSteppingUp() != noValue)
         setMaximumStepZWhenSteppingUp(parametersPacket.getMaximumStepZWhenSteppingUp());
      if (parametersPacket.getMaximumStepXWhenForwardAndDown() != noValue)
         setMaximumStepXWhenForwardAndDown(parametersPacket.getMaximumStepXWhenForwardAndDown());
      if (parametersPacket.getMaximumStepYWhenForwardAndDown() != noValue)
         setMaximumStepYWhenForwardAndDown(parametersPacket.getMaximumStepYWhenForwardAndDown());
      if (parametersPacket.getMaximumStepZWhenForwardAndDown() != noValue)
         setMaximumStepZWhenForwardAndDown(parametersPacket.getMaximumStepZWhenForwardAndDown());
      if (parametersPacket.getTranslationScaleFromGrandparentNode() != noValue)
         setTranslationScaleFromGrandparentNode(parametersPacket.getTranslationScaleFromGrandparentNode());
      if (parametersPacket.getMaximumStepZ() != noValue)
         setMaximumStepZ(parametersPacket.getMaximumStepZ());
      if (parametersPacket.getStepYawReductionFactorAtMaxReach() != noValue)
         setStepYawReductionFactorAtMaxReach(parametersPacket.getStepYawReductionFactorAtMaxReach());
      if (parametersPacket.getMinimumFootholdPercent() != noValue)
         setMinimumFootholdPercent(parametersPacket.getMinimumFootholdPercent());
      if (parametersPacket.getMinimumSurfaceInclineRadians() != noValue)
         setMinimumSurfaceInclineRadians(parametersPacket.getMinimumSurfaceInclineRadians());
      setWiggleIntoConvexHullOfPlanarRegions(parametersPacket.getWiggleIntoConvexHullOfPlanarRegions());
      setRejectIfCannotFullyWiggleInside(parametersPacket.getRejectIfCannotFullyWiggleInside());
      if (parametersPacket.getMaximumXyWiggleDistance() != noValue)
         setMaximumXYWiggleDistance(parametersPacket.getMaximumXyWiggleDistance());
      if (parametersPacket.getMaximumYawWiggle() != noValue)
         setMaximumYawWiggle(parametersPacket.getMaximumYawWiggle());
      if (parametersPacket.getMaximumZPenetrationOnValleyRegions() != noValue)
         setMaximumZPenetrationOnValleyRegions(parametersPacket.getMaximumZPenetrationOnValleyRegions());
      if (parametersPacket.getMaximumStepWidth() != noValue)
         setMaximumStepWidth(parametersPacket.getMaximumStepWidth());
      if (parametersPacket.getCliffHeightToAvoid() != noValue)
         setCliffHeightToAvoid(parametersPacket.getCliffHeightToAvoid());
      if (parametersPacket.getMinimumDistanceFromCliffBottoms() != noValue)
         setMinimumDistanceFromCliffBottoms(parametersPacket.getMinimumDistanceFromCliffBottoms());
      setReturnBestEffortPlan(parametersPacket.getReturnBestEffortPlan());
      if (parametersPacket.getMinimumStepsForBestEffortPlan() > 0)
         setMinimumStepsForBestEffortPlan((int) parametersPacket.getMinimumStepsForBestEffortPlan());
      if (parametersPacket.getBodyBoxHeight() != noValue)
         setBodyBoxHeight(parametersPacket.getBodyBoxHeight());
      if (parametersPacket.getBodyBoxDepth() != noValue)
         setBodyBoxDepth(parametersPacket.getBodyBoxDepth());
      if (parametersPacket.getBodyBoxWidth() != noValue)
         setBodyBoxWidth(parametersPacket.getBodyBoxWidth());
      if (parametersPacket.getBodyBoxBaseX() != noValue)
         setBodyBoxBaseX(parametersPacket.getBodyBoxBaseX());
      if (parametersPacket.getBodyBoxBaseY() != noValue)
         setBodyBoxBaseY(parametersPacket.getBodyBoxBaseY());
      if (parametersPacket.getBodyBoxBaseZ() != noValue)
         setBodyBoxBaseZ(parametersPacket.getBodyBoxBaseZ());
      if (parametersPacket.getMinXClearanceFromStance() != noValue)
         setMinXClearanceFromStance(parametersPacket.getMinXClearanceFromStance());
      if (parametersPacket.getMinYClearanceFromStance() != noValue)
         setMinYClearanceFromStance(parametersPacket.getMinYClearanceFromStance());
      if (parametersPacket.getFinalTurnProximity() != noValue)
         setFinalTurnProximity(parametersPacket.getFinalTurnProximity());
      if (parametersPacket.getFinalTurnBodyPathProximity() != noValue)
         setFinalTurnBodyPathProximity(parametersPacket.getFinalTurnBodyPathProximity());
      if (parametersPacket.getFinalTurnProximityBlendFactor() != noValue)
         setFinalTurnProximityBlendFactor(parametersPacket.getFinalTurnProximityBlendFactor());

      setUseQuadraticDistanceCost(parametersPacket.getUseQuadraticDistanceCost());
      setUseQuadraticHeightCost(parametersPacket.getUseQuadraticHeightCost());

      if (parametersPacket.getAStarHeuristicsWeight() != noValue)
         setAStarHeuristicsWeight(parametersPacket.getAStarHeuristicsWeight());
      if (parametersPacket.getVisGraphWithAStarHeuristicsWeight() != noValue)
         setVisGraphWithAStarHeuristicsWeight(parametersPacket.getVisGraphWithAStarHeuristicsWeight());
      if (parametersPacket.getDepthFirstHeuristicsWeight() != noValue)
         setDepthFirstHeuristicsWeight(parametersPacket.getDepthFirstHeuristicsWeight());
      if (parametersPacket.getBodyPathBasedHeuristicsWeight() != noValue)
         setBodyPathBasedHeuristicWeight(parametersPacket.getBodyPathBasedHeuristicsWeight());

      if (parametersPacket.getYawWeight() != noValue)
         setYawWeight(parametersPacket.getYawWeight());
      if (parametersPacket.getPitchWeight() != noValue)
         setPitchWeight(parametersPacket.getPitchWeight());
      if (parametersPacket.getRollWeight() != noValue)
         setRollWeight(parametersPacket.getRollWeight());
      if (parametersPacket.getForwardWeight() != noValue)
         setForwardWeight(parametersPacket.getForwardWeight());
      if (parametersPacket.getLateralWeight() != noValue)
         setLateralWeight(parametersPacket.getLateralWeight());
      if (parametersPacket.getStepUpWeight() != noValue)
         setStepUpWeight(parametersPacket.getStepUpWeight());
      if (parametersPacket.getStepDownWeight() != noValue)
         setStepDownWeight(parametersPacket.getStepDownWeight());
      if (parametersPacket.getCostPerStep() != noValue)
         setCostPerStep(parametersPacket.getCostPerStep());
      if (parametersPacket.getMaximum2dDistanceFromBoundingBoxToPenalize() != noValue)
         setMaximum2dDistanceFromBoundingBoxToPenalize(parametersPacket.getMaximum2dDistanceFromBoundingBoxToPenalize());
      if (parametersPacket.getBoundingBoxCost() != noValue)
         setBoundingBoxCost(parametersPacket.getBoundingBoxCost());

      if (parametersPacket.getFootholdAreaWeight() != noValue)
         setFootholdAreaWeight(parametersPacket.getFootholdAreaWeight());
      if (parametersPacket.getLongStepWeight() != noValue)
         setLongStepWeight(parametersPacket.getLongStepWeight());
      if (parametersPacket.getBodyPathViolationWeight() != noValue)
         setBodyPathViolationWeight(parametersPacket.getBodyPathViolationWeight());
      if (parametersPacket.getDistanceFromPathTolerance() != noValue)
         setDistanceFromPathTolerance(parametersPacket.getDistanceFromPathTolerance());
      if (parametersPacket.getDeltaYawFromReferenceTolerance() != noValue)
         setDeltaYawFromReferenceTolerance(parametersPacket.getDeltaYawFromReferenceTolerance());
   }
}