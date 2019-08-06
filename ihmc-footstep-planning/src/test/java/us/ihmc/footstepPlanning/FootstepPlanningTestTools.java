package us.ihmc.footstepPlanning;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.footstepPlanning.graphSearch.parameters.*;
import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKey;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class FootstepPlanningTestTools
{
   private final static double epsilon = 1e-7;

   public static FootstepPlannerParametersReadOnly createRandomParameters(Random random)
   {

      FootstepPlannerParametersBasics parameters = new DefaultFootstepPlannerParameters();
      for (int i = 0; i < FootstepPlannerParameterKeys.keys.keys().size(); i++)
      {
         StoredPropertyKey<?> key = FootstepPlannerParameterKeys.keys.keys().get(i);
         if (key.getType() == Double.class)
            parameters.set((DoubleStoredPropertyKey) key, RandomNumbers.nextDouble(random, 0.01, 1.0));
         else if (key.getType() == Integer.class)
            parameters.set((IntegerStoredPropertyKey) key, RandomNumbers.nextInt(random, 1, 10));
         else if (key.getType() == Boolean.class)
            parameters.set((BooleanStoredPropertyKey) key, RandomNumbers.nextBoolean(random, 0.5));
      }


      return parameters;
   }


   public static void assertParametersEqual(FootstepPlannerParametersReadOnly parameters, FootstepPlannerParametersReadOnly other)
   {
      assertEquals(parameters.checkForBodyBoxCollisions(), other.checkForBodyBoxCollisions());
      assertEquals(parameters.performHeuristicSearchPolicies(), other.performHeuristicSearchPolicies());
      assertEquals(parameters.getIdealFootstepWidth(), other.getIdealFootstepWidth(), epsilon);
      assertEquals(parameters.getIdealFootstepLength(), other.getIdealFootstepLength(), epsilon);
      assertEquals(parameters.getWiggleInsideDelta(), other.getWiggleInsideDelta(), epsilon);
      assertEquals(parameters.getMaximumStepReach(), other.getMaximumStepReach(), epsilon);
      assertEquals(parameters.getMaximumStepYaw(), other.getMaximumStepYaw(), epsilon);
      assertEquals(parameters.getMinimumStepWidth(), other.getMinimumStepWidth(), epsilon);
      assertEquals(parameters.getMinimumStepLength(), other.getMinimumStepLength(), epsilon);
      assertEquals(parameters.getMinimumStepYaw(), other.getMinimumStepYaw(), epsilon);
      assertEquals(parameters.getMaximumStepReachWhenSteppingUp(), other.getMaximumStepReachWhenSteppingUp(), epsilon);
      assertEquals(parameters.getMaximumStepZWhenSteppingUp(), other.getMaximumStepZWhenSteppingUp(), epsilon);
      assertEquals(parameters.getMaximumStepXWhenForwardAndDown(), other.getMaximumStepXWhenForwardAndDown(), epsilon);
      assertEquals(parameters.getMaximumStepZWhenForwardAndDown(), other.getMaximumStepZWhenForwardAndDown(), epsilon);
      assertEquals(parameters.getMaximumStepZ(), other.getMaximumStepZ(), epsilon);
      assertEquals(parameters.getMinimumFootholdPercent(), other.getMinimumFootholdPercent(), epsilon);
      assertEquals(parameters.getMinimumSurfaceInclineRadians(), other.getMinimumSurfaceInclineRadians(), epsilon);
      assertEquals(parameters.getWiggleIntoConvexHullOfPlanarRegions(), other.getWiggleIntoConvexHullOfPlanarRegions());
      assertEquals(parameters.getRejectIfCannotFullyWiggleInside(), other.getRejectIfCannotFullyWiggleInside());
      assertEquals(parameters.getMaximumXYWiggleDistance(), other.getMaximumXYWiggleDistance(), epsilon);
      assertEquals(parameters.getMaximumYawWiggle(), other.getMaximumYawWiggle(), epsilon);
      assertEquals(parameters.getMaximumZPenetrationOnValleyRegions(), other.getMaximumZPenetrationOnValleyRegions(), epsilon);
      assertEquals(parameters.getMaximumStepWidth(), other.getMaximumStepWidth(), epsilon);
      assertEquals(parameters.getCliffHeightToAvoid(), other.getCliffHeightToAvoid(), epsilon);
      assertEquals(parameters.getMinimumDistanceFromCliffBottoms(), other.getMinimumDistanceFromCliffBottoms(), epsilon);
      assertEquals(parameters.getReturnBestEffortPlan(), other.getReturnBestEffortPlan());
      assertEquals(parameters.getMinimumStepsForBestEffortPlan(), other.getMinimumStepsForBestEffortPlan());
      assertEquals(parameters.getBodyGroundClearance(), other.getBodyGroundClearance(), epsilon);
      assertEquals(parameters.getBodyBoxHeight(), other.getBodyBoxHeight(), epsilon);
      assertEquals(parameters.getBodyBoxDepth(), other.getBodyBoxDepth(), epsilon);
      assertEquals(parameters.getBodyBoxWidth(), other.getBodyBoxWidth(), epsilon);
      assertEquals(parameters.getBodyBoxBaseX(), other.getBodyBoxBaseX(), epsilon);
      assertEquals(parameters.getBodyBoxBaseY(), other.getBodyBoxBaseY(), epsilon);
      assertEquals(parameters.getBodyBoxBaseZ(), other.getBodyBoxBaseZ(), epsilon);
      assertEquals(parameters.getMinXClearanceFromStance(), other.getMinXClearanceFromStance(), epsilon);
      assertEquals(parameters.getMinYClearanceFromStance(), other.getMinYClearanceFromStance(), epsilon);

      assertEquals(parameters.useQuadraticDistanceCost(), other.useQuadraticDistanceCost());
      assertEquals(parameters.useQuadraticHeightCost(), other.useQuadraticHeightCost());
      assertEquals(parameters.getAStarHeuristicsWeight().getValue(), other.getAStarHeuristicsWeight().getValue(), epsilon);
      assertEquals(parameters.getVisGraphWithAStarHeuristicsWeight().getValue(), other.getVisGraphWithAStarHeuristicsWeight().getValue(), epsilon);
      assertEquals(parameters.getDepthFirstHeuristicsWeight().getValue(), other.getDepthFirstHeuristicsWeight().getValue(), epsilon);
      assertEquals(parameters.getBodyPathBasedHeuristicsWeight().getValue(), other.getBodyPathBasedHeuristicsWeight().getValue(), epsilon);
      assertEquals(parameters.getYawWeight(), other.getYawWeight(), epsilon);
      assertEquals(parameters.getForwardWeight(), other.getForwardWeight(), epsilon);
      assertEquals(parameters.getLateralWeight(), other.getLateralWeight(), epsilon);
      assertEquals(parameters.getCostPerStep(), other.getCostPerStep(), epsilon);
      assertEquals(parameters.getStepUpWeight(), other.getStepUpWeight(), epsilon);
      assertEquals(parameters.getStepDownWeight(), other.getStepDownWeight(), epsilon);
      assertEquals(parameters.getRollWeight(), other.getRollWeight(), epsilon);
      assertEquals(parameters.getPitchWeight(), other.getPitchWeight(), epsilon);
   }
}
