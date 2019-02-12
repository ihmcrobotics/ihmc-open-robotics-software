package us.ihmc.quadrupedFootstepPlanning.footstepPlanning;

import org.junit.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedAStarFootstepPlanner;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedAStarFootstepPlannerDataSetTest extends FootstepPlannerDataSetTest
{
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.A_STAR;
   }

   public QuadrupedBodyPathAndFootstepPlanner createPlanner()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();
      xGaitSettings.setStanceLength(1.0);
      xGaitSettings.setStanceWidth(0.5);
      FootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters, xGaitSettings);
      return QuadrupedAStarFootstepPlanner.createPlanner(parameters, xGaitSettings, null, expansion, registry);
   }

   @Test
   public void testDatasetsWithoutOcclusion()
   {
      super.testDatasetsWithoutOcclusion();
   }

   public static void main(String[] args) throws Exception
   {
      QuadrupedAStarFootstepPlannerDataSetTest test = new QuadrupedAStarFootstepPlannerDataSetTest();
      String prefix = "unitTestDataSets/test/";
      VISUALIZE = true;
      test.setup();
      test.runAssertionsOnDataset(dataset -> test.runAssertions(dataset), prefix + "20171215_214730_CinderBlockField");
      ThreadTools.sleepForever();
      test.tearDown();

   }
}
