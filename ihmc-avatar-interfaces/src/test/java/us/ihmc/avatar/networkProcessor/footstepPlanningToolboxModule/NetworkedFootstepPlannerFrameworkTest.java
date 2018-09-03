package us.ihmc.footstepPlanning.frameworkTests;

import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.tools.FootstepPlannerIOTools.FootstepPlannerUnitTestDataset;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI;
import us.ihmc.footstepPlanning.ui.StandaloneFootstepPlannerUI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI.*;

public abstract class NetworkedFootstepPlannerFrameworkTest extends DataSetFrameworkTest
{

   protected StandaloneFootstepPlannerUI ui;

   private void setup()
   {
      tryToStartModule(() -> setupFootstepPlanningToolboxModule(robotModel, params));
   }

   private void tryToStartModule(ModuleStarter runnable)
   {
      try
      {
         runnable.startModule();
      }
      catch (RuntimeException | IOException e)
      {
         PrintTools.error(this, "Failed to start a module in the network processor, stack trace:");
         e.printStackTrace();
      }
   }

   private interface ModuleStarter
   {
      void startModule() throws IOException;
   }

   private void setupFootstepPlanningToolboxModule(DRCNetworkModuleParameters params) throws IOException
   {
      new FootstepPlanningToolboxModule(null, null, null, true);
   }


   @Override
   public void submitDataSet(FootstepPlannerUnitTestDataset dataset)
   {
      JavaFXMessager messager = ui.getMessager();

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlanarRegionDataTopic, dataset.getPlanarRegionsList());
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.StartPositionTopic, dataset.getStart());
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.GoalPositionTopic, dataset.getGoal());

      messager.submitMessage(PlannerTypeTopic, getPlannerType());

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlannerTimeoutTopic, dataset.getTimeout(getPlannerType()));
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlannerHorizonLengthTopic, Double.MAX_VALUE);
   }

   @Override
   public String findPlanAndAssertGoodResult(FootstepPlannerUnitTestDataset dataset)
   {
      JavaFXMessager messager = ui.getMessager();

      AtomicReference<Boolean> receivedPlan = new AtomicReference<>(false);
      AtomicReference<Boolean> receivedResult = new AtomicReference<>(false);
      messager.registerTopicListener(FootstepPlanTopic, request -> receivedPlan.set(true));
      messager.registerTopicListener(PlanningResultTopic, request -> receivedResult.set(true));

      AtomicReference<FootstepPlan> footstepPlanReference = messager.createInput(FootstepPlanTopic);
      AtomicReference<FootstepPlanningResult> footstepPlanningResult = messager.createInput(PlanningResultTopic);

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.ComputePathTopic, true);

      while (!receivedPlan.get() && !receivedResult.get())
      {
         ThreadTools.sleep(10);
      }
      String datasetName = dataset.getDatasetName();

      int ticksToWait = 100;
      int tick = 0;
      if (receivedResult.get() && footstepPlanningResult.get().validForExecution())
      { // we know there's a valid plan, so wait until we've received it
         while (!receivedPlan.get())
         {
            if (tick > ticksToWait)
               return "Supposedly found a solution, but never received a plan out.";
            ThreadTools.sleep(10);
            tick++;
         }
      }

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
