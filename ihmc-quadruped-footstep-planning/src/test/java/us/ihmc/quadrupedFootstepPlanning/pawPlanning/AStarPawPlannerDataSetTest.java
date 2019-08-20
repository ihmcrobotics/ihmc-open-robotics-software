package us.ihmc.quadrupedFootstepPlanning.pawPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.AStarPawPlanner;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.DefaultPawPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParametersReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.visualization.PawAStarMessagerListener;
import us.ihmc.quadrupedPlanning.QuadrupedGait;
import us.ihmc.quadrupedPlanning.QuadrupedSpeed;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import static us.ihmc.robotics.Assert.assertTrue;

public class AStarPawPlannerDataSetTest extends PawPlannerDataSetTest
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
   public BodyPathAndPawPlanner createPlanner()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      xGaitSettings = getXGaitSettings();
      PawPlannerParametersReadOnly parameters = new DefaultPawPlannerParameters()
      {
         @Override
         public double getXGaitWeight()
         {
            return 0.0;
         }

         @Override
         public double getDesiredVelocityWeight()
         {
            return 0.0;
         }
      };
      PawAStarMessagerListener listener;
      if (VISUALIZE)
         listener = new PawAStarMessagerListener(messager);
      else
         listener = null;

      return AStarPawPlanner.createPlanner(parameters, xGaitSettings, listener, registry);
   }

   @Override
   @Test
   public void testDataSets()
   {
      super.testDataSets();
   }

   public static void main(String[] args)
   {
      AStarPawPlannerDataSetTest test = new AStarPawPlannerDataSetTest();
      VISUALIZE = true;
      test.setup();
//      String errorMessage = test.runAssertions(DataSetName._20171115_171243_SimplePlaneAndWall);
      String errorMessage = test.runAssertions(DataSetName._20171218_204953_FlatGroundWithWall);
      assertTrue(errorMessage, errorMessage.isEmpty());
//      if (activelyVisualize)
//         test.visualizer.showAndSleep(true);
      ThreadTools.sleepForever();
   }
}
