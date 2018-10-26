package us.ihmc.footstepPlanning.messager;

import org.junit.After;
import org.junit.Before;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerDataSetTest;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.tools.FootstepPlannerIOTools.FootstepPlannerUnitTestDataset;
import us.ihmc.footstepPlanning.ui.components.FootstepPathCalculatorModule;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.FootstepPlanTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlanningResultTopic;

public abstract class MessagerPlannerDataSetTest extends FootstepPlannerDataSetTest
{
   protected FootstepPathCalculatorModule module = null;


   @Override
   @Before
   public void setup()
   {
      super.setup();

      messager.registerTopicListener(FootstepPlanTopic, request -> uiReceivedPlan.set(true));
      messager.registerTopicListener(PlanningResultTopic, request -> uiReceivedResult.set(true));

      uiFootstepPlanReference = messager.createInput(FootstepPlanTopic);
      uiPlanningResultReference = messager.createInput(PlanningResultTopic);
   }

   @Override
   public void setupInternal()
   {
      module = new FootstepPathCalculatorModule(messager);
      module.start();
   }

   @After
   public void tearDown() throws Exception
   {
      module.stop();
      messager.closeMessager();
      if (ui != null)
         ui.stop();
      ui = null;

      uiFootstepPlanReference = null;
      uiPlanningResultReference = null;

      module = null;
      messager = null;
   }

   @Override
   public void submitDataSet(FootstepPlannerUnitTestDataset dataset)
   {
      for (int i = 0; i < 10; i++)
         ThreadTools.sleep(100);

      packPlanningRequest(dataset, messager);
   }

   @Override
   public String findPlanAndAssertGoodResult(FootstepPlannerUnitTestDataset dataset)
   {
      totalTimeTaken = 0.0;
      double timeoutMultiplier = ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      double maxTimeToWait = 2.0 * timeoutMultiplier * dataset.getTimeout(getPlannerType());
      String datasetName = dataset.getDatasetName();

      queryUIResults();

      String errorMessage = "";

      errorMessage += waitForResult(() -> actualResult.get() == null, maxTimeToWait, datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      errorMessage += validateResult(() -> actualResult.get().validForExecution(), actualResult.get(), datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      errorMessage += waitForPlan(() -> actualPlan.get() == null, maxTimeToWait, datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      FootstepPlanningResult result = actualResult.getAndSet(null);
      FootstepPlan plan = actualPlan.getAndSet(null);

      uiReceivedPlan.getAndSet(false);
      uiReceivedResult.getAndSet(false);

      errorMessage += assertPlanIsValid(datasetName, result, plan, dataset.getGoal());

      for (int i = 0; i < 10; i++)
         ThreadTools.sleep(100);

      return errorMessage;
   }
}
