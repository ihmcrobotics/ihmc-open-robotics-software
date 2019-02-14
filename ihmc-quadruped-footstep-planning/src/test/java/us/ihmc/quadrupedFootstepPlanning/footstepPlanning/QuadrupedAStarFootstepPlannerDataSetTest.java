package us.ihmc.quadrupedFootstepPlanning.footstepPlanning;

import org.junit.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedAStarFootstepPlanner;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.listeners.QuadrupedFootstepPlannerListener;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.visualization.AStarMessagerListener;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.visualization.QuadrupedAStarFootstepPlannerVisualizer;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedAStarFootstepPlannerDataSetTest extends FootstepPlannerDataSetTest
{
   private static final boolean activelyVisualize = false;
   private QuadrupedFootstepPlannerListener visualizer;

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

   public QuadrupedBodyPathAndFootstepPlanner createPlanner()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      QuadrupedXGaitSettingsReadOnly xGaitSettings = getXGaitSettings();
      FootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters, xGaitSettings);
      if (activelyVisualize)
         visualizer = new QuadrupedAStarFootstepPlannerVisualizer(null);
      else
         visualizer = new AStarMessagerListener(messager);

      return QuadrupedAStarFootstepPlanner.createPlanner(parameters, xGaitSettings, visualizer, expansion, registry);
   }

   @Override
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
      test.runAssertionsOnDataset(dataset -> test.runAssertions(dataset), prefix + "20171115_171243_SimplePlaneAndWall");
//      if (activelyVisualize)
//         test.visualizer.showAndSleep(true);
      ThreadTools.sleepForever();
      test.tearDown();

   }
}
