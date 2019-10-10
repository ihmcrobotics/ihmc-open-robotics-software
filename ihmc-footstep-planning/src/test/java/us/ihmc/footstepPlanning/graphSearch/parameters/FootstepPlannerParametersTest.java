package us.ihmc.footstepPlanning.graphSearch.parameters;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class FootstepPlannerParametersTest
{
   private static final int iters = 1000;
   private static final double epsilon = 1e-5;

   @Test
   public void testSettingParameters()
   {
      Random random = new Random(1738L);
      FootstepPlannerParametersBasics parameters = new DefaultFootstepPlannerParameters();

      for (int iter = 0; iter < iters; iter++)
      {
         testSettingParameters(random, parameters);
      }
   }

   private static void testSettingParameters(Random random, FootstepPlannerParametersBasics parameters)
   {
      boolean checkForBodyBoxCollisions = RandomNumbers.nextBoolean(random, 0.5);
      parameters.setCheckForBodyBoxCollisions(checkForBodyBoxCollisions);
      assertEquals(checkForBodyBoxCollisions, parameters.checkForBodyBoxCollisions());

      boolean checkForPathCollisions = RandomNumbers.nextBoolean(random, 0.5);
      parameters.setCheckForPathCollisions(checkForPathCollisions);
      assertEquals(checkForPathCollisions, parameters.checkForPathCollisions());

      boolean performheuristicSearchPolicies = RandomNumbers.nextBoolean(random, 0.5);
      parameters.setPerformHeuristicSearchPolicies(performheuristicSearchPolicies);
      assertEquals(performheuristicSearchPolicies, parameters.performHeuristicSearchPolicies());

      double idealFootstepWidth = RandomNumbers.nextDouble(random, 10.0);
      parameters.setIdealFootstepWidth(idealFootstepWidth);
      assertEquals(idealFootstepWidth, parameters.getIdealFootstepWidth(), epsilon);

      double idealFootstepLength = RandomNumbers.nextDouble(random, 10.0);
      parameters.setIdealFootstepLength(idealFootstepLength);
      assertEquals(idealFootstepLength, parameters.getIdealFootstepLength(), epsilon);

      double maxStepReach = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaximumStepReach(maxStepReach);
      assertEquals(maxStepReach, parameters.getMaximumStepReach(), epsilon);

      double minStepLength = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMinimumStepLength(minStepLength);
      assertEquals(minStepLength, parameters.getMinimumStepLength(), epsilon);

      double minSTepYaw = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMinimumStepYaw(minSTepYaw);
      assertEquals(minSTepYaw, parameters.getMinimumStepYaw(), epsilon);

      double minStepWidth = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMinimumStepWidth(minStepWidth);
      assertEquals(minStepWidth, parameters.getMinimumStepWidth(), epsilon);

      double maxStepWidth = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaximumStepWidth(maxStepWidth);
      assertEquals(maxStepWidth, parameters.getMaximumStepWidth(), epsilon);

      double maxStepZ = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaximumStepZ(maxStepZ);
      assertEquals(maxStepZ, parameters.getMaximumStepZ(), epsilon);

      double maxStepXWhenForwardAndDown = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaximumStepXWhenForwardAndDown(maxStepXWhenForwardAndDown);
      assertEquals(maxStepXWhenForwardAndDown, parameters.getMaximumStepXWhenForwardAndDown(), epsilon);

      double maxStepZWhenForwardAndDown = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaximumStepZWhenForwardAndDown(maxStepZWhenForwardAndDown);
      assertEquals(maxStepZWhenForwardAndDown, parameters.getMaximumStepZWhenForwardAndDown(), epsilon);

      double wiggleInsideDelta = RandomNumbers.nextDouble(random, 10.0);
      parameters.setWiggleInsideDelta(wiggleInsideDelta);
      assertEquals(wiggleInsideDelta, parameters.getWiggleInsideDelta(), epsilon);

      double maximumStepReachWhenSteppingUp = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMaximumStepReachWhenSteppingUp(maximumStepReachWhenSteppingUp);
      assertEquals(maximumStepReachWhenSteppingUp, parameters.getMaximumStepReachWhenSteppingUp(), epsilon);

      double maximumStepZWhenSteppingUp = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMaximumStepZWhenSteppingUp(maximumStepZWhenSteppingUp);
      assertEquals(maximumStepZWhenSteppingUp, parameters.getMaximumStepZWhenSteppingUp(), epsilon);

      double minFootholdPercent = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMinimumFootholdPercent(minFootholdPercent);
      assertEquals(minFootholdPercent, parameters.getMinimumFootholdPercent(), epsilon);

      double minSurfaceIncline = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMinimumSurfaceInclineRadians(minSurfaceIncline);
      assertEquals(minSurfaceIncline, parameters.getMinimumSurfaceInclineRadians(), epsilon);

      boolean wiggleIntoConvexHull = RandomNumbers.nextBoolean(random, 0.5);
      parameters.setWiggleIntoConvexHullOfPlanarRegions(wiggleIntoConvexHull);
      assertEquals(wiggleIntoConvexHull, parameters.getWiggleIntoConvexHullOfPlanarRegions());

      boolean rejectIfCannotWiggleInside = RandomNumbers.nextBoolean(random, 0.5);
      parameters.setRejectIfCannotFullyWiggleInside(rejectIfCannotWiggleInside);
      assertEquals(rejectIfCannotWiggleInside, parameters.getRejectIfCannotFullyWiggleInside());

      double maximumXYWiggleDistance = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMaximumXYWiggleDistance(maximumXYWiggleDistance);
      assertEquals(maximumXYWiggleDistance, parameters.getMaximumXYWiggleDistance(), epsilon);

      double maximumYawWiggle = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMaximumYawWiggle(maximumYawWiggle);
      assertEquals(maximumYawWiggle, parameters.getMaximumYawWiggle(), epsilon);

      double maxZpenetration = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaximumZPenetrationOnValleyRegions(maxZpenetration);
      assertEquals(maxZpenetration, parameters.getMaximumZPenetrationOnValleyRegions(), epsilon);

      double cliffHeightToAvoid = RandomNumbers.nextDouble(random, 10.00);
      parameters.setCliffHeightToAvoid(cliffHeightToAvoid);
      assertEquals(cliffHeightToAvoid, parameters.getCliffHeightToAvoid());

      double minimumDistanceFromCliff = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMinimumDistanceFromCliffBottoms(minimumDistanceFromCliff);
      assertEquals(minimumDistanceFromCliff, parameters.getMinimumDistanceFromCliffBottoms(), epsilon);

      boolean returnBestEffortPlan = RandomNumbers.nextBoolean(random, 0.5);
      parameters.setReturnBestEffortPlan(returnBestEffortPlan);
      assertEquals(returnBestEffortPlan, parameters.getReturnBestEffortPlan());

      int minStepsForBestEffort = RandomNumbers.nextInt(random, -10, 10);
      parameters.setMinimumStepsForBestEffortPlan(minStepsForBestEffort);
      assertEquals(minStepsForBestEffort, parameters.getMinimumStepsForBestEffortPlan());

      double minXClearanceFromStance = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMinXClearanceFromStance(minXClearanceFromStance);
      assertEquals(minXClearanceFromStance, parameters.getMinXClearanceFromStance(), epsilon);

      double minYClearanceFromStance = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMinYClearanceFromStance(minYClearanceFromStance);
      assertEquals(minYClearanceFromStance, parameters.getMinYClearanceFromStance(), epsilon);

      double bodyBoxWidth = RandomNumbers.nextDouble(random, 10.0);
      parameters.setBodyBoxWidth(bodyBoxWidth);
      assertEquals(bodyBoxWidth, parameters.getBodyBoxWidth(), epsilon);

      double bodyBoxHeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setBodyBoxHeight(bodyBoxHeight);
      assertEquals(bodyBoxHeight, parameters.getBodyBoxHeight(), 10.0);

      double bodyBoxDepth = RandomNumbers.nextDouble(random, 10.0);
      parameters.setBodyBoxDepth(bodyBoxDepth);
      assertEquals(bodyBoxDepth, parameters.getBodyBoxDepth(), epsilon);

      double bodyBoxBaseX = RandomNumbers.nextDouble(random, 10.0);
      parameters.setBodyBoxBaseX(bodyBoxBaseX);
      assertEquals(bodyBoxBaseX, parameters.getBodyBoxBaseX(), epsilon);

      double bodyBOxBaseY = RandomNumbers.nextDouble(random, 10.0);
      parameters.setBodyBoxBaseY(bodyBOxBaseY);
      assertEquals(bodyBOxBaseY, parameters.getBodyBoxBaseY(), epsilon);

      double bodyBoxBaseZ = RandomNumbers.nextDouble(random, 10.0);
      parameters.setBodyBoxBaseZ(bodyBoxBaseZ);
      assertEquals(bodyBoxBaseZ, parameters.getBodyBoxBaseZ(), epsilon);

      double finalTurnProximity = RandomNumbers.nextDouble(random, 10.0);
      parameters.setFinalTurnProximity(finalTurnProximity);
      assertEquals(finalTurnProximity, parameters.getFinalTurnProximity(), epsilon);

      double finalTurnBodyPathProximity = RandomNumbers.nextDouble(random, 10.0);
      parameters.setFinalTurnBodyPathProximity(finalTurnBodyPathProximity);
      assertEquals(finalTurnBodyPathProximity, parameters.getFinalTurnBodyPathProximity(), epsilon);

      double finalTurnProximityBlendFactor = RandomNumbers.nextDouble(random, 10.0);
      parameters.setFinalTurnProximityBlendFactor(finalTurnProximityBlendFactor);
      assertEquals(finalTurnProximityBlendFactor, parameters.getFinalTurnProximityBlendFactor(), epsilon);

      int numberOfBoundingBoxChecks = RandomNumbers.nextInt(random, -10, 10);
      parameters.setNumberOfBoundingBoxChecks(numberOfBoundingBoxChecks);
      assertEquals(numberOfBoundingBoxChecks, parameters.getNumberOfBoundingBoxChecks());

      boolean useQuadraticDistanceCost = RandomNumbers.nextBoolean(random, 0.5);
      parameters.setUseQuadraticDistanceCost(useQuadraticDistanceCost);
      assertEquals(useQuadraticDistanceCost, parameters.useQuadraticDistanceCost());

      boolean useQuadraticHeightCost = RandomNumbers.nextBoolean(random, 0.5);
      parameters.setUseQuadraticHeightCost(useQuadraticHeightCost);
      assertEquals(useQuadraticHeightCost, parameters.useQuadraticHeightCost());

      double aStarHeuristicWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setAStarHeuristicsWeight(aStarHeuristicWeight);
      assertEquals(aStarHeuristicWeight, parameters.getAStarHeuristicsWeight().getValue(), epsilon);

      double visGrpahWithAStarHeuristicWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setVisGraphWithAStarHeuristicsWeight(visGrpahWithAStarHeuristicWeight);
      assertEquals(visGrpahWithAStarHeuristicWeight, parameters.getVisGraphWithAStarHeuristicsWeight().getValue(), epsilon);

      double depthFirstWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setDepthFirstHeuristicsWeight(depthFirstWeight);
      assertEquals(depthFirstWeight, parameters.getDepthFirstHeuristicsWeight().getValue(), epsilon);

      double bodyPathWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setBodyPathBasedHeuristicWeight(bodyPathWeight);
      assertEquals(bodyPathWeight, parameters.getBodyPathBasedHeuristicsWeight().getValue(), epsilon);

      double yawWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setYawWeight(yawWeight);
      assertEquals(yawWeight, parameters.getYawWeight(), epsilon);

      double forwardWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setForwardWeight(forwardWeight);
      assertEquals(forwardWeight, parameters.getForwardWeight(), epsilon);

      double laterlaWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setLateralWeight(laterlaWeight);
      assertEquals(laterlaWeight, parameters.getLateralWeight(), epsilon);

      double costPerStep = RandomNumbers.nextDouble(random, 10.0);
      parameters.setCostPerStep(costPerStep);
      assertEquals(costPerStep, parameters.getCostPerStep(), epsilon);

      double stepUpWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setStepUpWeight(stepUpWeight);
      assertEquals(stepUpWeight, parameters.getStepUpWeight(), epsilon);

      double stepDownWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setStepDownWeight(stepDownWeight);
      assertEquals(stepDownWeight, parameters.getStepDownWeight(), epsilon);

      double rollWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setRollWeight(rollWeight);
      assertEquals(rollWeight, parameters.getRollWeight(), epsilon);

      double pitchWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setPitchWeight(pitchWeight);
      assertEquals(pitchWeight, parameters.getPitchWeight(), epsilon);

      double maximumDistanceFromBoundingBox = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaximum2dDistanceFromBoundingBoxToPenalize(maximumDistanceFromBoundingBox);
      assertEquals(maximumDistanceFromBoundingBox, parameters.getMaximum2dDistanceFromBoundingBoxToPenalize(), epsilon);

      double boundingBoxCost=  RandomNumbers.nextDouble(random, 10.00);
      parameters.setBoundingBoxCost(boundingBoxCost);
      assertEquals(boundingBoxCost, parameters.getBoundingBoxCost(), epsilon);

      double footholdAreaWeight = RandomNumbers.nextDouble(random, 10.00);
      parameters.setFootholdAreaWeight(footholdAreaWeight);
      assertEquals(footholdAreaWeight, parameters.getFootholdAreaWeight(), epsilon);

      double longStepWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setLongStepWeight(longStepWeight);
      assertEquals(longStepWeight, parameters.getLongStepWeight(), epsilon);

      double bodyPathViolationWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setBodyPathViolationWeight(bodyPathViolationWeight);
      assertEquals(bodyPathViolationWeight, parameters.getBodyPathViolationWeight(), epsilon);

      double distanceFrompathTolerance = RandomNumbers.nextDouble(random, 10.0);
      parameters.setDistanceFromPathTolerance(distanceFrompathTolerance);
      assertEquals(distanceFrompathTolerance, parameters.getDistanceFromPathTolerance(), epsilon);

      double deltaYawFromReferenceTolerance = RandomNumbers.nextDouble(random, 10.0);
      parameters.setDeltaYawFromReferenceTolerance(deltaYawFromReferenceTolerance);
      assertEquals(deltaYawFromReferenceTolerance, parameters.getDeltaYawFromReferenceTolerance(), epsilon);
   }
}
