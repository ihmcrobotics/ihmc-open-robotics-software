package us.ihmc.footstepPlanning.ui.components;

import org.junit.jupiter.api.Test;
import us.ihmc.footstepPlanning.FootstepPlanningTestTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.SettableFootstepPlannerParameters;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class SettableFootstepPlannerParametersTest
{
   private final static int iters = 10;
   private final static double epsilon = 1e-7;

   @Test
   public void test()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         FootstepPlannerParameters parameters = FootstepPlanningTestTools.createRandomParameters(random);
         SettableFootstepPlannerParameters settableParameters = new SettableFootstepPlannerParameters(parameters);

         assertParametersEqual(parameters, settableParameters);

         parameters = FootstepPlanningTestTools.createRandomParameters(random);
         settableParameters.set(parameters);

         assertParametersEqual(parameters, settableParameters);

         parameters = FootstepPlanningTestTools.createRandomParameters(random);

         settableParameters.setCheckForBodyBoxCollisions(parameters.checkForBodyBoxCollisions());
         settableParameters.setPerformHeuristicSearchPolicies(parameters.performHeuristicSearchPolicies());
         settableParameters.setIdealFootstepWidth(parameters.getIdealFootstepWidth());
         settableParameters.setIdealFootstepLength(parameters.getIdealFootstepLength());
         settableParameters.setWiggleInsideDelta(parameters.getWiggleInsideDelta());
         settableParameters.setMaximumStepReach(parameters.getMaximumStepReach());
         settableParameters.setMaximumStepYaw(parameters.getMaximumStepYaw());
         settableParameters.setMinimumStepWidth(parameters.getMinimumStepWidth());
         settableParameters.setMinimumStepLength(parameters.getMinimumStepLength());
         settableParameters.setMinimumStepYaw(parameters.getMinimumStepYaw());
         settableParameters.setMaximumStepReachWhenSteppingUp(parameters.getMaximumStepReachWhenSteppingUp());
         settableParameters.setMaximumStepZWhenSteppingUp(parameters.getMaximumStepZWhenSteppingUp());
         settableParameters.setMaximumStepXWhenForwardAndDown(parameters.getMaximumStepXWhenForwardAndDown());
         settableParameters.setMaximumStepZWhenForwardAndDown(parameters.getMaximumStepZWhenForwardAndDown());
         settableParameters.setMaximumStepZ(parameters.getMaximumStepZ());
         settableParameters.setMinimumFootholdPercent(parameters.getMinimumFootholdPercent());
         settableParameters.setMinimumSurfaceInclineRadians(parameters.getMinimumSurfaceInclineRadians());
         settableParameters.setWiggleIntoConvexHullOfPlanarRegions(parameters.getWiggleIntoConvexHullOfPlanarRegions());
         settableParameters.setRejectIfCannotFullyWiggleInside(parameters.getRejectIfCannotFullyWiggleInside());
         settableParameters.setMaximumXYWiggleDistance(parameters.getMaximumXYWiggleDistance());
         settableParameters.setMaximumYawWiggle(parameters.getMaximumYawWiggle());
         settableParameters.setMaximumZPenetrationOnValleyRegions(parameters.getMaximumZPenetrationOnValleyRegions());
         settableParameters.setMaximumStepWidth(parameters.getMaximumStepWidth());
         settableParameters.setCliffHeightToAvoid(parameters.getCliffHeightToAvoid());
         settableParameters.setMinimumDistanceFromCliffBottoms(parameters.getMinimumDistanceFromCliffBottoms());
         settableParameters.setReturnBestEffortPlan(parameters.getReturnBestEffortPlan());
         settableParameters.setMinimumStepsForBestEffortPlan(parameters.getMinimumStepsForBestEffortPlan());
         settableParameters.setBodyGroundClearance(parameters.getBodyGroundClearance());
         settableParameters.setBodyBoxHeight(parameters.getBodyBoxHeight());
         settableParameters.setBodyBoxDepth(parameters.getBodyBoxDepth());
         settableParameters.setBodyBoxWidth(parameters.getBodyBoxWidth());
         settableParameters.setBodyBoxBaseX(parameters.getBodyBoxBaseX());
         settableParameters.setBodyBoxBaseY(parameters.getBodyBoxBaseY());
         settableParameters.setBodyBoxBaseZ(parameters.getBodyBoxBaseZ());
         settableParameters.setMinXClearanceFromStance(parameters.getMinXClearanceFromStance());
         settableParameters.setMinYClearanceFromStance(parameters.getMinYClearanceFromStance());


         settableParameters.setUseQuadraticDistanceCost(parameters.getCostParameters().useQuadraticDistanceCost());
         settableParameters.setUseQuadraticHeightCost(parameters.getCostParameters().useQuadraticHeightCost());
         settableParameters.setAStarHeuristicsWeight(parameters.getCostParameters().getAStarHeuristicsWeight().getValue());
         settableParameters.setVisGraphWithAStarHeuristicsWeight(parameters.getCostParameters().getVisGraphWithAStarHeuristicsWeight().getValue());
         settableParameters.setDepthFirstHeuristicsWeight(parameters.getCostParameters().getDepthFirstHeuristicsWeight().getValue());
         settableParameters.setBodyPathBasedHeuristicsWeight(parameters.getCostParameters().getBodyPathBasedHeuristicsWeight().getValue());
         settableParameters.setYawWeight(parameters.getCostParameters().getYawWeight());
         settableParameters.setForwardWeight(parameters.getCostParameters().getForwardWeight());
         settableParameters.setLateralWeight(parameters.getCostParameters().getLateralWeight());
         settableParameters.setCostPerStep(parameters.getCostParameters().getCostPerStep());
         settableParameters.setStepUpWeight(parameters.getCostParameters().getStepUpWeight());
         settableParameters.setStepDownWeight(parameters.getCostParameters().getStepDownWeight());
         settableParameters.setRollWeight(parameters.getCostParameters().getRollWeight());
         settableParameters.setPitchWeight(parameters.getCostParameters().getPitchWeight());


         assertParametersEqual(parameters, settableParameters);
      }
   }

   private void assertParametersEqual(FootstepPlannerParameters parameters, SettableFootstepPlannerParameters settableParameters)
   {
      FootstepPlanningTestTools.assertParametersEqual(parameters, settableParameters);


      assertEquals(parameters.getCostParameters().useQuadraticDistanceCost(), settableParameters.useQuadraticDistanceCost());
      assertEquals(parameters.getCostParameters().useQuadraticHeightCost(), settableParameters.useQuadraticHeightCost());
      assertEquals(parameters.getCostParameters().getAStarHeuristicsWeight().getValue(), settableParameters.getAStarHeuristicsWeight(), epsilon);
      assertEquals(parameters.getCostParameters().getVisGraphWithAStarHeuristicsWeight().getValue(), settableParameters.getVisGraphWithAStarHeuristicsWeight(), epsilon);
      assertEquals(parameters.getCostParameters().getDepthFirstHeuristicsWeight().getValue(), settableParameters.getDepthFirstHeuristicsWeight(), epsilon);
      assertEquals(parameters.getCostParameters().getBodyPathBasedHeuristicsWeight().getValue(), settableParameters.getBodyPathBasedHeuristicsWeight(), epsilon);
      assertEquals(parameters.getCostParameters().getYawWeight(), settableParameters.getYawWeight(), epsilon);
      assertEquals(parameters.getCostParameters().getForwardWeight(), settableParameters.getForwardWeight(), epsilon);
      assertEquals(parameters.getCostParameters().getLateralWeight(), settableParameters.getLateralWeight(), epsilon);
      assertEquals(parameters.getCostParameters().getCostPerStep(), settableParameters.getCostPerStep(), epsilon);
      assertEquals(parameters.getCostParameters().getStepUpWeight(), settableParameters.getStepUpWeight(), epsilon);
      assertEquals(parameters.getCostParameters().getStepDownWeight(), settableParameters.getStepDownWeight(), epsilon);
      assertEquals(parameters.getCostParameters().getRollWeight(), settableParameters.getRollWeight(), epsilon);
      assertEquals(parameters.getCostParameters().getPitchWeight(), settableParameters.getPitchWeight(), epsilon);
   }
}
