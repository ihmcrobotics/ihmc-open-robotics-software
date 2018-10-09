package us.ihmc.footstepPlanning.roughTerrainPlanning;

import javafx.application.Application;
import javafx.stage.Stage;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.graphSearch.aStar.FootstepNodeVisualization;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
import us.ihmc.footstepPlanning.ui.ApplicationRunner;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.footstepPlanning.communication.FootstepPlannerSharedMemoryAPI;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class AStarOnRoughTerrainTest extends FootstepPlannerOnRoughTerrainTest
{
   private static final boolean visualizePlanner = false;
   private AStarFootstepPlanner planner;
   private FootstepNodeVisualization visualization = null;
   private FootstepPlannerUI ui;

   private static boolean visualize = true;
   private static boolean keepUp = false;

   @Before
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      if (visualize)
      {
         ApplicationRunner.runApplication(new Application()
         {
            @Override
            public void start(Stage stage) throws Exception
            {
               messager = new SharedMemoryJavaFXMessager(FootstepPlannerSharedMemoryAPI.API);
               messager.startMessager();

               ui = FootstepPlannerUI.createMessagerUI(stage, messager);
               ui.show();
            }

            @Override
            public void stop()
            {
               ui.stop();
            }
         });

         while (ui == null)
            ThreadTools.sleep(100);
      }
   }

   @After
   public void tearDown()
   {
      if (ui != null)
         ui.stop();
   }

   @Override
   public boolean assertPlannerReturnedResult()
   {
      return true;
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 10.2, categoriesOverride = {IntegrationCategory.EXCLUDE})
   @Test(timeout = 51000)
   public void testPartialGaps()
   {
      super.testPartialGaps();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 10.2, categoriesOverride = {IntegrationCategory.EXCLUDE})
   @Test(timeout = 51000)
   public void testSpiralStaircase()
   {
      super.testSpiralStaircase();
   }

   @Before
   public void createPlanner()
   {
      if (visualizePlanner)
         visualization = new FootstepNodeVisualization(1000, 1.0, null);
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      ParameterBasedNodeExpansion expansion = new ParameterBasedNodeExpansion(getPlannerParameters());
      planner = AStarFootstepPlanner
            .createRoughTerrainPlanner(getPlannerParameters(), visualization, footPolygons, expansion, new YoVariableRegistry("TestRegistry"));
   }

   @After
   public void destroyPlanner()
   {
      planner = null;

      if (visualizePlanner)
      {
         for (int i = 0; i < 1000; i++)
            visualization.tickAndUpdate();
         visualization.showAndSleep(true);
      }
   }

   @Override
   public FootstepPlanner getPlanner()
   {
      return planner;
   }

   @Override
   public boolean visualize()
   {
      if (visualizePlanner)
         return false;
      return visualize;
   }

   @Override
   public boolean keepUp()
   {
      return keepUp;
   }
}
