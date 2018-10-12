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
   private static final double bambooTimeScaling = 4.0;

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
   public void tearDown() throws Exception
   {
      module.stop();
      messager.closeMessager();
      if (ui != null)
         ui.stop();

      module = null;
      messager = null;
      ui = null;
   }

   @Override
   public void submitDataSet(FootstepPlannerUnitTestDataset dataset)
   {
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.PlanarRegionDataTopic, dataset.getPlanarRegionsList());
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.StartPositionTopic, dataset.getStart());
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.GoalPositionTopic, dataset.getGoal());

      if (DEBUG)
      {
         PrintTools.info("Dataset " + dataset.getDatasetName());
         PrintTools.info("          Planning From : " );
         PrintTools.info("             " + dataset.getStart());
         PrintTools.info("          To : ");
         PrintTools.info("             " + dataset.getGoal());

      }

      if (dataset.hasGoalOrientation())
         messager.submitMessage(FootstepPlannerSharedMemoryAPI.GoalOrientationTopic, dataset.getGoalOrientation());
      if (dataset.hasStartOrientation())
         messager.submitMessage(FootstepPlannerSharedMemoryAPI.StartOrientationTopic, dataset.getStartOrientation());

      messager.submitMessage(PlannerTypeTopic, getPlannerType());

      double timeMultiplier = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.PlannerTimeoutTopic, timeMultiplier * dataset.getTimeout(getPlannerType()));
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.PlannerHorizonLengthTopic, Double.MAX_VALUE);
   }

   @Override
   public String findPlanAndAssertGoodResult(FootstepPlannerUnitTestDataset dataset)
   {
      AtomicReference<Boolean> receivedPlan = new AtomicReference<>(false);
      AtomicReference<Boolean> receivedResult = new AtomicReference<>(false);
      messager.registerTopicListener(FootstepPlanTopic, request -> receivedPlan.set(true));
      messager.registerTopicListener(PlanningResultTopic, request -> receivedResult.set(true));

      AtomicReference<FootstepPlan> footstepPlanReference = messager.createInput(FootstepPlanTopic);
      AtomicReference<FootstepPlanningResult> footstepPlanningResult = messager.createInput(PlanningResultTopic);

      messager.submitMessage(FootstepPlannerSharedMemoryAPI.ComputePathTopic, true);

      String errorMessage = "";

      double timeMultiplier = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      double timeout = 2.0 * timeMultiplier * dataset.getTimeout(getPlannerType());
      double totalTimeTaken = 0.0;
      long sleepDuration = 10;
      while (!receivedResult.get() || footstepPlanningResult.get() == null)
      {
         if (totalTimeTaken > timeout)
         {
            errorMessage += "Timed out waiting for a result with dataset " + dataset.getDatasetName() + ".\n";
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

      errorMessage += assertTrue(datasetName, "Planning result for " + datasetName + " is invalid, result was " + footstepPlanningResult.get(),
                                 footstepPlanningResult.get().validForExecution());

      if (footstepPlanningResult.get().validForExecution())
         errorMessage += assertTrue(datasetName, datasetName + " did not reach goal.",
                                    PlannerTools.isGoalNextToLastStep(dataset.getGoal(), footstepPlanReference.get()));

      return errorMessage;
   }

   private String assertTrue(String datasetName, String message, boolean condition)
   {
      if (VISUALIZE || DEBUG)
      {
         if (!condition)
            PrintTools.error(datasetName + ": " + message);
      }
      return !condition ? "\n" + message : "";
   }

}
