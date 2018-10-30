package us.ihmc.footstepPlanning.remoteStandaloneDataSet;

import controller_msgs.msg.dds.*;
import org.junit.After;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerDataSetTest;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.footstepPlanning.tools.FootstepPlannerIOTools.FootstepPlannerUnitTestDataset;
import us.ihmc.footstepPlanning.ui.RemotePlannerMessageConverter;
import us.ihmc.footstepPlanning.ui.components.FootstepPathCalculatorModule;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.RealtimeRos2Node;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.FootstepPlanTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlanningResultTopic;

public abstract class RemoteStandalonePlannerDataSetTest extends FootstepPlannerDataSetTest
{
   private static final String robotName = "testBot";

   private RealtimeRos2Node ros2Node;

   private IHMCRealtimeROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher;

   protected DomainFactory.PubSubImplementation pubSubImplementation;

   private RemotePlannerMessageConverter messageConverter = null;
   private FootstepPathCalculatorModule module = null;

   @Override
   public void setupInternal()
   {
      messageConverter = RemotePlannerMessageConverter.createConverter(messager, robotName, DomainFactory.PubSubImplementation.INTRAPROCESS);

      module = new FootstepPathCalculatorModule(messager);
      module.start();

      messager.registerTopicListener(FootstepPlanTopic, request -> receivedPlan());
      messager.registerTopicListener(PlanningResultTopic, request -> receivedResult());

      uiFootstepPlanReference = messager.createInput(FootstepPlanTopic);
      uiPlanningResultReference = messager.createInput(PlanningResultTopic);

      ros2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, "ihmc_footstep_planner_test");

      footstepPlanningRequestPublisher = ROS2Tools
            .createPublisher(ros2Node, FootstepPlanningRequestPacket.class, FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName));

      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlanningToolboxOutputStatus.class,
                                           FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName),
                                           s -> processFootstepPlanningOutputStatus(s.takeNextData()));

      ros2Node.spin();
   }

   private void receivedPlan()
   {
      uiReceivedPlan.set(true);
      if (DEBUG)
         PrintTools.info("Received a plan over Java FX.");
   }

   private void receivedResult()
   {
      uiReceivedResult.set(true);
      if (DEBUG)
         PrintTools.info("Received a result over Java FX.");
   }

   @After
   public void tearDown() throws Exception
   {
      ros2Node.destroy();

      module.stop();
      messager.closeMessager();
      messageConverter.destroy();

      if (ui != null)
         ui.stop();
      ui = null;

      pubSubImplementation = null;

      uiFootstepPlanReference = null;
      uiPlanningResultReference = null;

      ros2Node = null;
      footstepPlanningRequestPublisher = null;

      module = null;
      messageConverter = null;
      messager = null;
   }

   @Override
   public void submitDataSet(FootstepPlannerUnitTestDataset dataset)
   {
      for (int i = 0; i < 100; i++)
         ThreadTools.sleep(10);

      FootstepPlanningRequestPacket planningRequestPacket = new FootstepPlanningRequestPacket();

      packPlanningRequest(dataset, planningRequestPacket);

      footstepPlanningRequestPublisher.publish(planningRequestPacket);
   }

   @Override
   public String findPlanAndAssertGoodResult(FootstepPlannerUnitTestDataset dataset)
   {
      totalTimeTaken = 0;
      double timeoutMultiplier = ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      double maxTimeToWait = 2.0 * timeoutMultiplier * dataset.getTimeout(getPlannerType());
      String datasetName = dataset.getDatasetName();

      queryUIResults();
      queryPlannerResults();

      String errorMessage = "";

      if (DEBUG)
         PrintTools.info("Waiting for result.");

      errorMessage += waitForResult(() -> actualResult.get() == null || expectedResult.get() == null, maxTimeToWait, datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      if (DEBUG)
         PrintTools.info("Received a result (actual = " + actualResult.get() + " expected = " + expectedResult.get() + "), checking its validity.");

      errorMessage += validateResult(() -> actualResult.get().validForExecution() && expectedResult.get().validForExecution(), actualResult.get(), datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      if (DEBUG)
         PrintTools.info("Results are valid, waiting for plan.");

      errorMessage += waitForPlan(() -> expectedPlan.get() == null || actualPlan.get() == null, maxTimeToWait, datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      if (DEBUG)
         PrintTools.info("Received a plan, checking its validity.");

      FootstepPlanningResult expectedResult = this.expectedResult.getAndSet(null);
      FootstepPlanningResult actualResult = this.actualResult.getAndSet(null);
      FootstepPlan expectedPlan = this.expectedPlan.getAndSet(null);
      FootstepPlan actualPlan = this.actualPlan.getAndSet(null);

      uiReceivedResult.set(false);
      uiReceivedPlan.set(false);
      plannerReceivedPlan.set(false);
      plannerReceivedResult.set(false);

      errorMessage += assertPlansAreValid(datasetName, expectedResult, actualResult, expectedPlan, actualPlan, dataset.getGoal());

      for (int i = 0; i < 100; i++)
         ThreadTools.sleep(10);

      return errorMessage;
   }
}
