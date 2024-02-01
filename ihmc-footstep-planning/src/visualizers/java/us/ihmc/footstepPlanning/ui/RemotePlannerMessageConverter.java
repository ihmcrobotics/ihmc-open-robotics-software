package us.ihmc.footstepPlanning.ui;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.FootstepDataListMessage;
import perception_msgs.msg.dds.HeightMapMessage;
import toolbox_msgs.msg.dds.FootstepPlanningRequestPacket;
import toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.communication.FootstepPlannerAPI;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;

public class RemotePlannerMessageConverter
{
   private static final boolean verbose = false;

   private final RealtimeROS2Node ros2Node;

   private final Messager messager;
   private final String robotName;

   private IHMCRealtimeROS2Publisher<FootstepPlanningToolboxOutputStatus> outputStatusPublisher;

   private Optional<HeightMapMessage> heightMapData = Optional.empty();

   private final AtomicReference<FootstepPlanningResult> resultReference;
   private final AtomicReference<FootstepDataListMessage> footstepPlanReference;
   private final AtomicReference<Integer> plannerRequestIdReference;
   private final AtomicReference<Integer> receivedPlanIdReference;
   private final AtomicReference<Boolean> acceptNewPlanarRegionsReference;

   public static RemotePlannerMessageConverter createRemoteConverter(Messager messager, String robotName)
   {
      return createConverter(messager, robotName, DomainFactory.PubSubImplementation.FAST_RTPS);
   }

   public static RemotePlannerMessageConverter createIntraprocessConverter(Messager messager, String robotName)
   {
      return createConverter(messager, robotName, DomainFactory.PubSubImplementation.INTRAPROCESS);
   }

   public static RemotePlannerMessageConverter createConverter(Messager messager, String robotName, DomainFactory.PubSubImplementation implementation)
   {
      RealtimeROS2Node ros2Node = ROS2Tools.createRealtimeROS2Node(implementation, "ihmc_footstep_planner_ui");
      return new RemotePlannerMessageConverter(ros2Node, messager, robotName);
   }

   public RemotePlannerMessageConverter(RealtimeROS2Node ros2Node, Messager messager, String robotName)
   {
      this.messager = messager;
      this.robotName = robotName;
      this.ros2Node = ros2Node;

      resultReference = messager.createInput(FootstepPlannerMessagerAPI.FootstepPlanningResultTopic);
      footstepPlanReference = messager.createInput(FootstepPlannerMessagerAPI.FootstepPlanResponse);
      plannerRequestIdReference = messager.createInput(FootstepPlannerMessagerAPI.PlannerRequestId, 0);
      receivedPlanIdReference = messager.createInput(FootstepPlannerMessagerAPI.ReceivedPlanId, 0);
      acceptNewPlanarRegionsReference = messager.createInput(FootstepPlannerMessagerAPI.AcceptNewPlanarRegions, true);

      registerPubSubs(ros2Node);

      ros2Node.spin();
   }

   public void destroy()
   {
      ros2Node.destroy();
   }

   private void registerPubSubs(RealtimeROS2Node ros2Node)
   {
      /* subscribers */
      // we want to listen to the incoming request
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepPlanningRequestPacket.class,
                                                    FootstepPlannerAPI.inputTopic(robotName),
                                           s -> processFootstepPlanningRequestPacket(s.takeNextData()));
      // we want to also listen to incoming REA planar region data.
      // TODO replace with height map
//      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, PlanarRegionsListMessage.class, REACommunicationProperties.outputTopic,
//                                           s -> processIncomingPlanarRegionMessage(s.takeNextData()));

      // publishers
      outputStatusPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, FootstepPlanningToolboxOutputStatus.class,
                                                                 FootstepPlannerAPI.outputTopic(robotName));

      messager.addTopicListener(FootstepPlannerMessagerAPI.FootstepPlanningResultTopic, request -> checkAndPublishIfInvalidResult());
      messager.addTopicListener(FootstepPlannerMessagerAPI.FootstepPlanResponse, request -> publishResultingPlan());
   }

   private void processFootstepPlanningRequestPacket(FootstepPlanningRequestPacket packet)
   {
      if (verbose)
         PrintTools.info("Received planning request over the network.");

      HeightMapMessage heightMapMessage = packet.getHeightMapMessage();
      if (heightMapMessage == null)
      {
         this.heightMapData = Optional.empty();
      }
      else
      {
         this.heightMapData = Optional.of(heightMapMessage);
      }

      RobotSide initialSupportSide = RobotSide.fromByte(packet.getRequestedInitialStanceSide());
      int plannerRequestId = packet.getPlannerRequestId();

      double timeout = packet.getTimeout();
      double horizonLength = packet.getHorizonLength();

      this.heightMapData.ifPresent(regions -> messager.submitMessage(FootstepPlannerMessagerAPI.HeightMapData, regions));
      messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootPose, packet.getStartLeftFootPose());
      messager.submitMessage(FootstepPlannerMessagerAPI.RightFootPose, packet.getStartRightFootPose());
      messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootGoalPose, packet.getGoalLeftFootPose());
      messager.submitMessage(FootstepPlannerMessagerAPI.RightFootGoalPose, packet.getGoalRightFootPose());

      messager.submitMessage(FootstepPlannerMessagerAPI.PlanBodyPath, packet.getPlanBodyPath());
      messager.submitMessage(FootstepPlannerMessagerAPI.PerformAStarSearch, packet.getPerformAStarSearch());

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTimeout, timeout);
      messager.submitMessage(FootstepPlannerMessagerAPI.InitialSupportSide, initialSupportSide);
      messager.submitMessage(FootstepPlannerMessagerAPI.MaxIterations, packet.getMaxIterations());
      messager.submitMessage(FootstepPlannerMessagerAPI.SnapGoalSteps, packet.getSnapGoalSteps());
      messager.submitMessage(FootstepPlannerMessagerAPI.AbortIfGoalStepSnapFails, packet.getAbortIfGoalStepSnappingFails());

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerRequestId, plannerRequestId);

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerHorizonLength, horizonLength);

      messager.submitMessage(FootstepPlannerMessagerAPI.ComputePath, true);
   }

   private void processIncomingHeightMapMessage(HeightMapMessage packet)
   {
      this.heightMapData = Optional.of(packet);

      if (acceptNewPlanarRegionsReference.get())
         messager.submitMessage(FootstepPlannerMessagerAPI.HeightMapData, packet);
   }

   private void publishResultingPlan()
   {
      double totalWaitTime = 0.0;
      double maxWaitTime = 10.0;
      long sleepDuration = 1;

      while (resultReference.get() == null)
      {
         if (totalWaitTime > maxWaitTime)
         {
            if (verbose)
               PrintTools.info("Timed out waiting for a plan when we received a valid result for execution.");
            return;
         }
         ThreadTools.sleep(sleepDuration);
         totalWaitTime += Conversions.millisecondsToSeconds(sleepDuration);
      }

      if (verbose)
         PrintTools.info("Finished planning, publishing the result on the network.");

      FootstepPlanningToolboxOutputStatus result = new FootstepPlanningToolboxOutputStatus();

      FootstepPlanningResult planningResult = resultReference.getAndSet(null);
      if (!planningResult.validForExecution())
         throw new RuntimeException("The result is completely invalid, which is a problem.");

      heightMapData.ifPresent(result.getHeightMapMessage()::set);
      result.setFootstepPlanningResult(planningResult.toByte());
      result.getFootstepDataList().set(footstepPlanReference.getAndSet(null));
      result.setPlanId(plannerRequestIdReference.get());
      receivedPlanIdReference.set(plannerRequestIdReference.get());;

      outputStatusPublisher.publish(result);
   }

   private void checkAndPublishIfInvalidResult()
   {
      if (resultReference.get().validForExecution())
         return;

      FootstepPlanningToolboxOutputStatus result = new FootstepPlanningToolboxOutputStatus();

      FootstepPlanningResult planningResult = resultReference.getAndSet(null);

      heightMapData.ifPresent(result.getHeightMapMessage()::set);
      result.setFootstepPlanningResult(planningResult.toByte());
      result.setPlanId(plannerRequestIdReference.get());

      outputStatusPublisher.publish(result);

      if (verbose)
         PrintTools.info("Finished planning, but result isn't valid, so publishing blank result on the network.");
   }
}
