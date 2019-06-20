package us.ihmc.footstepPlanning.ui.components;

import org.junit.jupiter.api.Test;
import us.ihmc.footstepPlanning.FootstepPlanningTestTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.SettableFootstepPlannerCostParameters;

import java.util.Random;

public class SettableFootstepPlannerCostParametersTest
{
   private final static int iters = 10;

   @Test
   public void test()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         FootstepPlannerCostParameters costParameters = FootstepPlanningTestTools.getRandomCostParameters(random);
         SettableFootstepPlannerCostParameters settableParameters = new SettableFootstepPlannerCostParameters(costParameters);

         FootstepPlanningTestTools.assertCostParametersEqual(costParameters, settableParameters);

         costParameters = FootstepPlanningTestTools.getRandomCostParameters(random);
         settableParameters.set(costParameters);

         FootstepPlanningTestTools.assertCostParametersEqual(costParameters, settableParameters);

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

         FootstepPlanningTestTools.assertCostParametersEqual(costParameters, settableParameters);
      }
   }

}
