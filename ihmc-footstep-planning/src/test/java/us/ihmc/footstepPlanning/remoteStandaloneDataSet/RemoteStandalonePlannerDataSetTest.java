package us.ihmc.footstepPlanning.remoteStandaloneDataSet;

import controller_msgs.msg.dds.*;
import org.junit.After;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerDataSetTest;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.footstepPlanning.tools.FootstepPlannerIOTools.FootstepPlannerUnitTestDataset;
import us.ihmc.footstepPlanning.ui.RemotePlannerMessageConverter;
import us.ihmc.footstepPlanning.ui.components.FootstepPathCalculatorModule;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.RealtimeRos2Node;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerSharedMemoryAPI.FootstepPlanTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerSharedMemoryAPI.PlanningResultTopic;

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

      uiReceivedPlan = new AtomicReference<>(false);
      uiReceivedResult = new AtomicReference<>(false);

      messager.registerTopicListener(FootstepPlanTopic, request -> uiReceivedPlan.set(true));
      messager.registerTopicListener(PlanningResultTopic, request -> uiReceivedResult.set(true));

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

   @After
   public void tearDown()
   {
      ros2Node.destroy();

      module.stop();
      messager.closeMessager();
      messageConverter.destroy();

      if (ui != null)
         ui.stop();
      ui = null;

      uiReceivedPlan = null;
      uiReceivedResult = null;

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
      for (int i = 0; i < 10; i++)
         ThreadTools.sleep(100);

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

      errorMessage += waitForResult(() -> actualResult.get() == null || expectedResult.get() == null, maxTimeToWait, datasetName);

      errorMessage += validateResult(() -> actualResult.get().validForExecution() && expectedResult.get().validForExecution(), actualResult.get(), datasetName);

      errorMessage += waitForPlan(() -> expectedPlan.get() == null || actualPlan.get() == null, maxTimeToWait, datasetName);

      FootstepPlanningResult expectedResult = this.expectedResult.getAndSet(null);
      FootstepPlanningResult actualResult = this.actualResult.getAndSet(null);
      FootstepPlan expectedPlan = this.expectedPlan.getAndSet(null);
      FootstepPlan actualPlan = this.actualPlan.getAndSet(null);

      uiReceivedResult.set(false);
      uiReceivedPlan.set(false);
      plannerReceivedPlan.set(false);

      errorMessage += assertPlansAreValid(datasetName, expectedResult, actualResult, expectedPlan, actualPlan, dataset.getGoal());

      for (int i = 0; i < 10; i++)
         ThreadTools.sleep(100);

      return errorMessage;
   }

   private void processFootstepPlanningOutputStatus(FootstepPlanningToolboxOutputStatus packet)
   {
      plannerResultReference.set(FootstepPlanningResult.fromByte(packet.getFootstepPlanningResult()));
      plannerPlanReference.set(convertToFootstepPlan(packet.getFootstepDataList()));
      plannerReceivedPlan.set(true);
   }

   private static FootstepPlan convertToFootstepPlan(FootstepDataListMessage footstepDataListMessage)
   {
      FootstepPlan footstepPlan = new FootstepPlan();

      for (FootstepDataMessage footstepMessage : footstepDataListMessage.getFootstepDataList())
      {
         FramePose3D stepPose = new FramePose3D();
         stepPose.setPosition(footstepMessage.getLocation());
         stepPose.setOrientation(footstepMessage.getOrientation());
         SimpleFootstep footstep = footstepPlan.addFootstep(RobotSide.fromByte(footstepMessage.getRobotSide()), stepPose);

         ConvexPolygon2D foothold = new ConvexPolygon2D();
         for (int i = 0; i < footstepMessage.getPredictedContactPoints2d().size(); i++)
            foothold.addVertex(footstepMessage.getPredictedContactPoints2d().get(i));
         foothold.update();
         footstep.setFoothold(foothold);
      }

      return footstepPlan;
   }
}
