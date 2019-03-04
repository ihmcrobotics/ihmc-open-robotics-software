package us.ihmc.quadrupedFootstepPlanning.footstepPlanning;

import org.junit.jupiter.api.Test;
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
   public QuadrupedBodyPathAndFootstepPlanner createPlanner()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      QuadrupedXGaitSettingsReadOnly xGaitSettings = getXGaitSettings();
      FootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters()
      {
         @Override
         public double getMaximumStepReach()
         {
            return 0.7;
         }

         @Override
         public double getMaximumStepCycleDistance()
         {
            return 0.65;
         }

         @Override
         public double getMinimumStepLength()
         {
            return -0.3;
         }

         @Override
         public double getMinimumStepWidth()
         {
            return -0.3;
         }

         @Override
         public double getMaximumStepWidth()
         {
            return 0.35;
         }
      };
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters, xGaitSettings);

      return QuadrupedAStarFootstepPlanner.createPlanner(parameters, xGaitSettings, null, expansion, registry);
   }

   @Override
   @Test
   public void testDatasetsWithoutOcclusion()
   {
      super.testDatasetsWithoutOcclusion();
   }

   public static void main(String[] args)
   {
      QuadrupedAStarFootstepPlannerDataSetTest test = new QuadrupedAStarFootstepPlannerDataSetTest();
      VISUALIZE = true;
      test.setup();
      test.runAssertionsOnDataset(dataset -> test.runAssertions(dataset), "20171215_220208_SimpleStairs");
//      if (activelyVisualize)
//         test.visualizer.showAndSleep(true);
      ThreadTools.sleepForever();
   }
}
