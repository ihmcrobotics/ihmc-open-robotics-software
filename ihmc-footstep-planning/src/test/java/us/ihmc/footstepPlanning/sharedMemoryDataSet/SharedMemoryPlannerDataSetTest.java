package us.ihmc.footstepPlanning.sharedMemoryDataSet;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import org.junit.After;
import org.junit.Before;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerDataSetTest;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.communication.FootstepPlannerSharedMemoryAPI;
import us.ihmc.footstepPlanning.tools.FootstepPlannerIOTools.FootstepPlannerUnitTestDataset;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.ui.ApplicationRunner;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.footstepPlanning.ui.components.FootstepPathCalculatorModule;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryMessager;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerSharedMemoryAPI.*;

public abstract class SharedMemoryPlannerDataSetTest extends FootstepPlannerDataSetTest
{
   protected SharedMemoryMessager messager = null;
   protected FootstepPathCalculatorModule module = null;
   protected FootstepPlannerUI ui = null;

   private AtomicReference<Boolean> receivedPlan = new AtomicReference<>(false);
   private AtomicReference<Boolean> receivedResult = new AtomicReference<>(false);

   private AtomicReference<FootstepPlan> footstepPlanReference;
   private AtomicReference<FootstepPlanningResult> footstepPlanningResult;

   @Before
   public void setup()
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      if (VISUALIZE)
         messager = new SharedMemoryJavaFXMessager(FootstepPlannerSharedMemoryAPI.API);
      else
         messager = new SharedMemoryMessager(FootstepPlannerSharedMemoryAPI.API);

      try
      {
         messager.startMessager();
      }
      catch (Exception e)
      {
         throw new RuntimeException("Failed to start message.");
      }

      module = FootstepPathCalculatorModule.createMessagerModule(messager);
      module.start();

      receivedPlan = new AtomicReference<>(false);
      receivedResult = new AtomicReference<>(false);

      messager.registerTopicListener(FootstepPlanTopic, request -> receivedPlan.set(true));
      messager.registerTopicListener(PlanningResultTopic, request -> receivedResult.set(true));

      footstepPlanReference = messager.createInput(FootstepPlanTopic);
      footstepPlanningResult = messager.createInput(PlanningResultTopic);

      if (VISUALIZE)
      {

         ApplicationRunner.runApplication(new Application()
         {
            @Override
            public void start(Stage stage) throws Exception
            {
               ui = FootstepPlannerUI.createMessagerUI(stage, (SharedMemoryJavaFXMessager) messager);
               ui.show();
            }

            @Override
            public void stop() throws Exception
            {
               ui.stop();
               Platform.exit();
            }
         });

         double maxWaitTime = 5.0;
         double totalTime = 0.0;
         long sleepDuration = 100;

         while (ui == null)
         {
            if (totalTime > maxWaitTime)
               throw new RuntimeException("Timed out waiting for the UI to start.");
            ThreadTools.sleep(sleepDuration);
            totalTime += Conversions.millisecondsToSeconds(sleepDuration);
         }

      }
   }

   @After
   public void tearDown()
   {
      module.stop();
      messager.closeMessager();
      if (ui != null)
         ui.stop();

      receivedPlan = null;
      receivedResult = null;

      footstepPlanReference = null;
      footstepPlanningResult = null;

      module = null;
      messager = null;
      ui = null;
   }

   @Override
   public void submitDataSet(FootstepPlannerUnitTestDataset dataset)
   {
      packPlanningRequest(dataset, messager);
   }

   @Override
   public String findPlanAndAssertGoodResult(FootstepPlannerUnitTestDataset dataset)
   {
      String errorMessage = "";

      double timeMultiplier = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      double timeout = 2.0 * timeMultiplier * dataset.getTimeout(getPlannerType());
      double totalTimeTaken = 0.0;
      long sleepDuration = 10;
      while (!receivedResult.get() || footstepPlanningResult.get() == null)
      {
         if (totalTimeTaken > timeout)
         {
            errorMessage += dataset.getDatasetName() + " timed out waiting for a result.\n";
            return errorMessage;
         }

         ThreadTools.sleep(sleepDuration);
         totalTimeTaken += Conversions.millisecondsToSeconds(sleepDuration);
      }

      if (!footstepPlanningResult.get().validForExecution())
      {
         errorMessage += "Dataset " + dataset.getDatasetName() + " failed to find a valid result. Result : " + footstepPlanningResult.get() + "\n";
         return errorMessage;
      }

      while (!receivedPlan.get() || footstepPlanningResult.get() == null )
      {
         if (totalTimeTaken > timeout)
         {
            errorMessage += "Timed out waiting for a result with dataset " + dataset.getDatasetName() + ".\n";
            return errorMessage;
         }

         ThreadTools.sleep(sleepDuration);
         totalTimeTaken += Conversions.millisecondsToSeconds(sleepDuration);
      }

      String datasetName = dataset.getDatasetName();

      FootstepPlanningResult result = footstepPlanningResult.getAndSet(null);
      FootstepPlan plan = footstepPlanReference.getAndSet(null);
      receivedPlan.getAndSet(false);
      receivedResult.getAndSet(false);

      errorMessage += assertPlanIsValid(datasetName, result, plan, dataset.getGoal());

      return errorMessage;
   }
}
