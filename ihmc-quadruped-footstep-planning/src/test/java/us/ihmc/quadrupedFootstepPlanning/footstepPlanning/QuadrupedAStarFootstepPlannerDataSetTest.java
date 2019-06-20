package us.ihmc.quadrupedFootstepPlanning.footstepPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedAStarFootstepPlanner;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.visualization.AStarMessagerListener;
import us.ihmc.quadrupedPlanning.QuadrupedGait;
import us.ihmc.quadrupedPlanning.QuadrupedSpeed;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import static us.ihmc.robotics.Assert.assertTrue;

public class QuadrupedAStarFootstepPlannerDataSetTest extends FootstepPlannerDataSetTest
{
   public QuadrupedXGaitSettings getXGaitSettings()
   {
      QuadrupedXGaitSettings settings = new QuadrupedXGaitSettings();
      settings.setStanceLength(1.0);
      settings.setStanceWidth(0.5);
      settings.setQuadrupedSpeed(QuadrupedSpeed.MEDIUM);
      settings.setEndPhaseShift(QuadrupedGait.AMBLE.getEndPhaseShift());
      settings.getAmbleMediumTimings().setEndDoubleSupportDuration(0.25);
      settings.getAmbleMediumTimings().setStepDuration(0.5);
      settings.getAmbleMediumTimings().setMaxSpeed(0.3);
      return settings;
   }

   @Override
   public QuadrupedBodyPathAndFootstepPlanner createPlanner()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      xGaitSettings = getXGaitSettings();
      FootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters()
      {
         @Override
         public double getXGaitWeight()
         {
            return 2.0;
         }

         @Override
         public double getDesiredVelocityWeight()
         {
            return 0.5;
         }
      };
      AStarMessagerListener listener;
      if (VISUALIZE)
         listener = new AStarMessagerListener(messager);
      else
         listener = null;

      return QuadrupedAStarFootstepPlanner.createPlanner(parameters, xGaitSettings, listener, registry);
   }

   @Override
   @Test
   public void testDataSets()
   {
      super.testDataSets();
   }

   public static void main(String[] args)
   {
      QuadrupedAStarFootstepPlannerDataSetTest test = new QuadrupedAStarFootstepPlannerDataSetTest();
      VISUALIZE = true;
      test.setup();
      String errorMessage = test.runAssertions(DataSetName._20171115_171243_SimplePlaneAndWall);
//      String errorMessage = test.runAssertions(DataSetName._20171218_204953_FlatGroundWithWall);
      assertTrue(errorMessage, errorMessage.isEmpty());
//      if (activelyVisualize)
//         test.visualizer.showAndSleep(true);
      ThreadTools.sleepForever();
   }
}
