package us.ihmc.footstepPlanning.ui.components;

import org.junit.Test;
import us.ihmc.footstepPlanning.FootstepPlanningTestTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;
import us.ihmc.footstepPlanning.graphSearch.stepCost.LinearHeightCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.QuadraticDistanceAndYawCost;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class SettableFootstepPlannerCostParametersTest
{
   private final static int iters = 10;
   private final static double epsilon = 1e-7;

   @Test(timeout = 30000)
   public void test()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         FootstepPlannerCostParameters costParameters = FootstepPlanningTestTools.getRandomCostParameters(random);
         SettableFootstepPlannerCostParameters settableParameters = new SettableFootstepPlannerCostParameters(costParameters);

         assertParametersEqual(costParameters, settableParameters);

         costParameters = FootstepPlanningTestTools.getRandomCostParameters(random);
         settableParameters.set(costParameters);

         assertParametersEqual(costParameters, settableParameters);

         costParameters = FootstepPlanningTestTools.getRandomCostParameters(random);
         settableParameters.setUseQuadraticDistanceCost(costParameters.useQuadraticDistanceCost());
         settableParameters.setUseQuadraticHeightCost(costParameters.useQuadraticHeightCost());
         settableParameters.setAStarHeuristicsWeight(costParameters.getAStarHeuristicsWeight().getValue());
         settableParameters.setVisGraphWithAStarHeuristicsWeight(costParameters.getVisGraphWithAStarHeuristicsWeight().getValue());
         settableParameters.setDepthFirstHeuristicsWeight(costParameters.getDepthFirstHeuristicsWeight().getValue());
         settableParameters.setBodyPathBasedHeuristicsWeight(costParameters.getBodyPathBasedHeuristicsWeight().getValue());
         settableParameters.setYawWeight(costParameters.getYawWeight());
         settableParameters.setForwardWeight(costParameters.getForwardWeight());
         settableParameters.setLateralWeight(costParameters.getLateralWeight());
         settableParameters.setCostPerStep(costParameters.getCostPerStep());
         settableParameters.setStepUpWeight(costParameters.getStepUpWeight());
         settableParameters.setStepDownWeight(costParameters.getStepDownWeight());
         settableParameters.setRollWeight(costParameters.getRollWeight());
         settableParameters.setPitchWeight(costParameters.getPitchWeight());


         assertParametersEqual(costParameters, settableParameters);
      }
   }

   private void assertParametersEqual(FootstepPlannerCostParameters parameters, SettableFootstepPlannerCostParameters settableParameters)
   {
      assertEquals(parameters.useQuadraticDistanceCost(), settableParameters.useQuadraticDistanceCost());
      assertEquals(parameters.useQuadraticHeightCost(), settableParameters.useQuadraticHeightCost());
      assertEquals(parameters.getAStarHeuristicsWeight().getValue(), settableParameters.getAStarHeuristicsWeight().getValue(), epsilon);
      assertEquals(parameters.getVisGraphWithAStarHeuristicsWeight().getValue(), settableParameters.getVisGraphWithAStarHeuristicsWeight().getValue(), epsilon);
      assertEquals(parameters.getDepthFirstHeuristicsWeight().getValue(), settableParameters.getDepthFirstHeuristicsWeight().getValue(), epsilon);
      assertEquals(parameters.getBodyPathBasedHeuristicsWeight().getValue(), settableParameters.getBodyPathBasedHeuristicsWeight().getValue(), epsilon);
      assertEquals(parameters.getYawWeight(), settableParameters.getYawWeight(), epsilon);
      assertEquals(parameters.getForwardWeight(), settableParameters.getForwardWeight(), epsilon);
      assertEquals(parameters.getLateralWeight(), settableParameters.getLateralWeight(), epsilon);
      assertEquals(parameters.getCostPerStep(), settableParameters.getCostPerStep(), epsilon);
      assertEquals(parameters.getStepUpWeight(), settableParameters.getStepUpWeight(), epsilon);
      assertEquals(parameters.getStepDownWeight(), settableParameters.getStepDownWeight(), epsilon);
      assertEquals(parameters.getRollWeight(), settableParameters.getRollWeight(), epsilon);
      assertEquals(parameters.getPitchWeight(), settableParameters.getPitchWeight(), epsilon);
   }
}
