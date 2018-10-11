package us.ihmc.footstepPlanning.sharedMemoryDataSet;

import org.junit.After;
import org.junit.Before;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerDataSetTest;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.tools.FootstepPlannerIOTools.FootstepPlannerUnitTestDataset;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.ui.ApplicationRunner;
import us.ihmc.footstepPlanning.communication.FootstepPlannerSharedMemoryAPI;
import us.ihmc.footstepPlanning.ui.SharedMemoryStandaloneFootstepPlannerUI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerSharedMemoryAPI.*;

public abstract class SharedMemoryPlannerDataSetTest extends FootstepPlannerDataSetTest
{
   private SharedMemoryStandaloneFootstepPlannerUI launcher;
   protected JavaFXMessager messager;


   @Before
   public void setup()
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      launcher = new SharedMemoryStandaloneFootstepPlannerUI(VISUALIZE);
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
   public void submitDataSet(FootstepPlannerUnitTestDataset dataset)
   {
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.PlanarRegionDataTopic, dataset.getPlanarRegionsList());
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.StartPositionTopic, dataset.getStart());
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.GoalPositionTopic, dataset.getGoal());

      if (dataset.hasGoalOrientation())
         messager.submitMessage(FootstepPlannerSharedMemoryAPI.GoalOrientationTopic, dataset.getGoalOrientation());
      if (dataset.hasStartOrientation())
         messager.submitMessage(FootstepPlannerSharedMemoryAPI.StartOrientationTopic, dataset.getStartOrientation());

      messager.submitMessage(PlannerTypeTopic, getPlannerType());

      messager.submitMessage(FootstepPlannerSharedMemoryAPI.PlannerTimeoutTopic, dataset.getTimeout(getPlannerType()));
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

      double timeout = dataset.getTimeout(getPlannerType());
      double totalTimeTaken = 0.0;
      long sleepDuration = 10;
      while (!receivedPlan.get() && !receivedResult.get())
      {
         if (totalTimeTaken > timeout + 5.0)
            throw new RuntimeException("Timed out waiting for a result.");

         ThreadTools.sleep(sleepDuration);
         totalTimeTaken += Conversions.millisecondsToSeconds(sleepDuration);
      }
      String datasetName = dataset.getDatasetName();

      String errorMessage = "";
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
