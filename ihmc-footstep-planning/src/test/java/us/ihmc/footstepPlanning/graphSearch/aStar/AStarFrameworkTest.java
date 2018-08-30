package us.ihmc.footstepPlanning.graphSearch.aStar;

import javafx.stage.Stage;
import org.junit.After;
import org.junit.Before;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.roughTerrainPlanning.FootstepPlannerFrameworkTest;
import us.ihmc.footstepPlanning.roughTerrainPlanning.StandaloneUIFootstepPlannerOnRoughTerrainTest;
import us.ihmc.footstepPlanning.ui.StandaloneFootstepPlannerUI;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class AStarFrameworkTest extends FootstepPlannerFrameworkTest
{
   private static boolean visualize = false;

   @Before
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      // Did not find a better solution for starting JavaFX and still be able to move on.
      new Thread(() -> launch()).start();

      while (ui == null)
         ThreadTools.sleep(200);
   }

   @After
   public void tearDown()
   {
      stop();
      ui = null;
   }

   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.A_STAR;
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      ui = new StandaloneFootstepPlannerUI(primaryStage);

      if (visualize)
      {
         ui.show();
      }
   }

   @Override
   public void stop()
   {
      ui.stop();
   }


}
