package us.ihmc.quadrupedFootstepPlanning.footstepPlanning;

import org.junit.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedAStarFootstepPlanner;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.listeners.QuadrupedFootstepPlannerListener;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.visualization.AStarMessagerListener;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedAStarToolboxDataSetTest extends FootstepPlannerToolboxDataSetTest
{
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.A_STAR;
   }

   public QuadrupedXGaitSettingsReadOnly getXGaitSettings()
   {
      QuadrupedXGaitSettings settings = new QuadrupedXGaitSettings();
      settings.setStanceLength(1.0);
      settings.setStanceWidth(0.5);
      settings.setEndDoubleSupportDuration(0.25);
      settings.setStepDuration(0.5);
      return settings;
   }

   @Override
   @Test
   public void testDatasetsWithoutOcclusion()
   {
      super.testDatasetsWithoutOcclusion();
   }

   public static void main(String[] args) throws Exception
   {
      QuadrupedAStarToolboxDataSetTest test = new QuadrupedAStarToolboxDataSetTest();
      String prefix = "unitTestDataSets/test/";
      VISUALIZE = true;
      test.setup();
      test.runAssertionsOnDataset(dataset -> test.runAssertions(dataset), prefix + "20171215_220523_SteppingStones");
      ThreadTools.sleepForever();
      test.tearDown();

   }
}
