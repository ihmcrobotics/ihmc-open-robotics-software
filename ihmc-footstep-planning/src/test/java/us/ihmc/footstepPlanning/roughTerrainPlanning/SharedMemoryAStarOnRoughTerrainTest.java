package us.ihmc.footstepPlanning.roughTerrainPlanning;

import org.junit.After;
import org.junit.Before;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.ui.ApplicationRunner;
import us.ihmc.footstepPlanning.ui.SharedMemoryFootstepPlannerUI;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class SharedMemoryAStarOnRoughTerrainTest extends SharedMemoryFootstepPlannerOnRoughTerrainTest
{
   private static boolean visualize = false;
   private SharedMemoryFootstepPlannerUI launcher;


   @Before
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      launcher = new SharedMemoryFootstepPlannerUI(visualize);
      ApplicationRunner.runApplication(launcher);

      messager = launcher.getMessager();
   }

   @After
   public void tearDown() throws Exception
   {
      launcher.stop();
      messager = null;
      launcher = null;
   }

   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.A_STAR;
   }

}
