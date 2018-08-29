package us.ihmc.footstepPlanning.roughTerrainPlanning;

import javafx.stage.Stage;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.PlannerTools;
import us.ihmc.footstepPlanning.simplePlanners.PlanThenSnapPlanner;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.footstepPlanning.ui.StandaloneFootstepPlannerUI;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class PlanThenSnapPlannerTest extends FootstepPlannerOnRoughTerrainTest
{
   private final PlanThenSnapPlanner planner = new PlanThenSnapPlanner(new TurnWalkTurnPlanner(), PlannerTools.createDefaultFootPolygons());

   private static boolean visualize = true;

   @Before
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      if (visualize)
      {
         // Did not find a better solution for starting JavaFX and still be able to move on.
         new Thread(() -> launch()).start();

         while (ui == null)
            ThreadTools.sleep(200);
      }
   }

   @After
   public void tearDown()
   {
      if (visualize())
      {
         stop();
      }
      ui = null;
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      if (visualize())
      {
         ui = new FootstepPlannerUI(primaryStage);
         ui.show();
      }
   }

   @Override
   public void stop()
   {
      if (visualize())
      {
         ui.stop();
      }
   }

   @Override
   public boolean assertPlannerReturnedResult()
   {
      return true;
   }

   @Override
   public FootstepPlanner getPlanner()
   {
      return planner;
   }

   @Override
   public boolean visualize()
   {
      return visualize;
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.2, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 30000)
   public void testSteppingStones()
   {
      super.testSteppingStones();
   }

}
