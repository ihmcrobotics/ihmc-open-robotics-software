package us.ihmc.footstepPlanning.roughTerrainPlanning;

import javafx.application.Application;
import javafx.stage.Stage;
import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.PlannerTools;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI;
import us.ihmc.footstepPlanning.ui.StandaloneFootstepPlannerUI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityGraphsIOTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityGraphsIOTools.VisibilityGraphsUnitTestDataset;
import us.ihmc.pathPlanning.visibilityGraphs.ui.VisibilityGraphsDataExporter;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI.FootstepPlanTopic;
import static us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI.PlannerTypeTopic;
import static us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI.PlanningResultTopic;

public abstract class FootstepPlannerFrameworkTest extends Application
{
   // Set that to MAX_VALUE when visualizing. Before pushing, it has to be reset to a reasonable value.
   private static final long TIMEOUT = 100000; // Long.MAX_VALUE; // 

   // Whether to start the UI or not.
   private static boolean VISUALIZE = false;
   // For enabling helpful prints.
   private static boolean DEBUG = true;

   // Because JavaFX will create a fresh new instance of VisibilityGraphsFrameworkTest, the ui has to be static so there is only one instance and we can refer to it in the test part.
   protected static StandaloneFootstepPlannerUI ui;

   public abstract FootstepPlannerType getPlannerType();


   @Test(timeout = TIMEOUT)
   @ContinuousIntegrationTest(estimatedDuration = 13.0)
   public void testDatasetsWithoutOcclusion()
   {
      runAssertionsOnAllDatasets(dataset -> runAssertionsWithoutOcclusion(dataset));
   }


   private void runAssertionsOnAllDatasets(DatasetTestRunner datasetTestRunner)
   {
      List<VisibilityGraphsUnitTestDataset> allDatasets = VisibilityGraphsIOTools.loadAllDatasets(VisibilityGraphsDataExporter.class);

      if (DEBUG)
      {
         PrintTools.info("Unit test files found: " + allDatasets.size());
      }

      int numberOfFailingDatasets = 0;
      String errorMessages = "";

      int currentDatasetIndex = 0;
      if (allDatasets.isEmpty())
         Assert.fail("Did not find any datasets to test.");

      // Randomizing the regionIds so the viz is better
      Random random = new Random(324);
      allDatasets.stream().map(VisibilityGraphsUnitTestDataset::getPlanarRegionsList).map(PlanarRegionsList::getPlanarRegionsAsList)
                 .forEach(regionsList -> regionsList.forEach(region -> region.setRegionId(random.nextInt())));

      VisibilityGraphsUnitTestDataset dataset = allDatasets.get(currentDatasetIndex);

      while (dataset != null)
      {

         if (DEBUG)
         {
            PrintTools.info("Processing file: " + dataset.getDatasetName());
         }

         String errorMessagesForCurrentFile = datasetTestRunner.testDataset(dataset);
         if (!errorMessagesForCurrentFile.isEmpty())
            numberOfFailingDatasets++;
         errorMessages += errorMessagesForCurrentFile;

            currentDatasetIndex++;
            if (currentDatasetIndex < allDatasets.size())
               dataset = allDatasets.get(currentDatasetIndex);
            else
               dataset = null;

         ThreadTools.sleep(100); // Apparently need to give some time for the prints to appear in the right order.
      }

      Assert.assertTrue("Number of failing datasets: " + numberOfFailingDatasets + " out of " + allDatasets.size() + ". Errors:" + errorMessages,
                        errorMessages.isEmpty());
   }

   private String runAssertionsWithoutOcclusion(VisibilityGraphsUnitTestDataset dataset)
   {
      String datasetName = dataset.getDatasetName();

      JavaFXMessager messager = ui.getMessager();

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlanarRegionDataTopic, dataset.getPlanarRegionsList());
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.StartPositionTopic, dataset.getStart());
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.GoalPositionTopic, dataset.getGoal());

      messager.submitMessage(PlannerTypeTopic, getPlannerType());


      AtomicReference<Boolean> receivedPlan = new AtomicReference<>(false);
      AtomicReference<Boolean> receivedResult = new AtomicReference<>(false);
      messager.registerTopicListener(FootstepPlanTopic, request -> receivedPlan.set(true));
      messager.registerTopicListener(PlanningResultTopic, request -> receivedResult.set(true));
      AtomicReference<FootstepPlan> footstepPlanReference = messager.createInput(FootstepPlanTopic);
      AtomicReference<FootstepPlanningResult> footstepPlanningResult = messager.createInput(PlanningResultTopic);

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.ComputePathTopic, true);

      while (!receivedPlan.get() || !receivedResult.get())
      {
         ThreadTools.sleep(10);
      }

      String errorMessage = "";
      errorMessage += assertTrue(datasetName, "Planning result for is invalid, result was " + footstepPlanningResult.get(), footstepPlanningResult.get().validForExecution());
      errorMessage += assertTrue(datasetName, "Did not reach goal.", PlannerTools.isGoalNextToLastStep(dataset.getGoal(), footstepPlanReference.get()));

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



   private static interface DatasetTestRunner
   {
      String testDataset(VisibilityGraphsUnitTestDataset dataset);
   }
}
