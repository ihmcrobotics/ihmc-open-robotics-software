package us.ihmc.footstepPlanning.remoteStandaloneDataSet;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.ui.ApplicationRunner;
import us.ihmc.footstepPlanning.ui.SharedMemoryFootstepPlannerUI;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class RemoteStandalonePlanThenSnapDataSetTest extends RemoteStandalonePlannerDataSetTest
{
   private SharedMemoryFootstepPlannerUI launcher;
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.PLAN_THEN_SNAP;
   }

   @Test(timeout = 500000)
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 90.0)
   public void testDatasetsWithoutOcclusion()
   {
      runAssertionsOnAllDatasets(dataset -> runAssertionsWithoutOcclusion(dataset));
   }


   @Before
   public void setup()
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();


      launcher = new SharedMemoryFootstepPlannerUI(VISUALIZE);
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


   public static void main(String[] args) throws Exception
   {
      RemoteStandalonePlanThenSnapDataSetTest test = new RemoteStandalonePlanThenSnapDataSetTest();
      String prefix = "unitTestData/testable/";
      test.setup();
      test.runAssertionsOnDataset(dataset -> test.runAssertionsWithoutOcclusion(dataset), prefix + "20171218_205040_SimpleMaze");
      test.tearDown();

   }
}
