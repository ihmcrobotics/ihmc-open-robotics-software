package us.ihmc.footstepPlanning.frameworkTests;

import com.sun.javafx.application.PlatformImpl;
import javafx.application.Platform;
import javafx.stage.Stage;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.ui.StandaloneFootstepPlannerUILauncher;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class SimpleBodyPathFrameworkTest extends FootstepPlannerFrameworkTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.SIMPLE_BODY_PATH;
   }

   @Test(timeout = 500000)
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 30.0)
   public void testDatasetsWithoutOcclusion()
   {
      runAssertionsOnAllDatasets(dataset -> runAssertionsWithoutOcclusion(dataset));
   }


   @Before
   public void setup()
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();


      StandaloneFootstepPlannerUILauncher launcher = new StandaloneFootstepPlannerUILauncher(VISUALIZE);
      PlatformImpl.startup(() -> {
         Platform.runLater(new Runnable()
         {
            @Override
            public void run()
            {
               try
               {
                  launcher.start(new Stage());
               }
               catch (Exception e)
               {
                  e.printStackTrace();
               }
            }
         });
      });

      while (launcher.getUI() == null)
         ThreadTools.sleep(100);

      ui = launcher.getUI();
   }

   @After
   public void tearDown() throws Exception
   {
      launcher.stop();
      launcher = null;
      ui = null;
   }


   public static void main(String[] args) throws Exception
   {
      SimpleBodyPathFrameworkTest test = new SimpleBodyPathFrameworkTest();
      String prefix = "unitTestData/testable/";
      test.setup();
      test.runAssertionsOnDataset(dataset -> test.runAssertionsWithoutOcclusion(dataset), prefix + "20171218_204953_FlatGroundWithWall");
      test.tearDown();

   }
}
