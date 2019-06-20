package us.ihmc.footstepPlanning.ui;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.RealtimeRos2Node;

public class RemotePlannerMessageConverter
{
   private static final boolean verbose = false;

   private final RealtimeRos2Node ros2Node;

   private final Messager messager;
   private final String robotName;

   private IHMCRealtimeROS2Publisher<FootstepPlanningToolboxOutputStatus> outputStatusPublisher;

   private Optional<PlanarRegionsList> planarRegionsList = Optional.empty();

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
      RealtimeRos2Node ros2Node = ROS2Tools.createRealtimeRos2Node(implementation, "ihmc_footstep_planner_ui");
      return new RemotePlannerMessageConverter(ros2Node, messager, robotName);
   }

   public RemotePlannerMessageConverter(RealtimeRos2Node ros2Node, Messager messager, String robotName)
   {
      this.messager = messager;
      this.robotName = robotName;
      this.ros2Node = ros2Node;

      resultReference = messager.createInput(FootstepPlannerMessagerAPI.PlanningResultTopic);
      footstepPlanReference = messager.createInput(FootstepPlannerMessagerAPI.FootstepPlanResponseTopic);
      plannerRequestIdReference = messager.createInput(FootstepPlannerMessagerAPI.PlannerRequestIdTopic, 0);
      receivedPlanIdReference = messager.createInput(FootstepPlannerMessagerAPI.ReceivedPlanIdTopic, 0);
      acceptNewPlanarRegionsReference = messager.createInput(FootstepPlannerMessagerAPI.AcceptNewPlanarRegions, true);

      registerPubSubs(ros2Node);

      ros2Node.spin();
   }

   public void destroy()
   {
      ros2Node.destroy();
   }

   private void registerPubSubs(RealtimeRos2Node ros2Node)
   {
      /* subscribers */
      // we want to listen to the incoming request
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlanningRequestPacket.class,
                                           FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName),
                                           s -> processFootstepPlanningRequestPacket(s.takeNextData()));
      // we want to also listen to incoming REA planar region data.
      ROS2Tools.createCallbackSubscription(ros2Node, PlanarRegionsListMessage.class, REACommunicationProperties.publisherTopicNameGenerator,
                                           s -> processIncomingPlanarRegionMessage(s.takeNextData()));

      // publishers
      outputStatusPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPlanningToolboxOutputStatus.class,
                                                        FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName));

      messager.registerTopicListener(FootstepPlannerMessagerAPI.PlanningResultTopic, request -> checkAndPublishIfInvalidResult());
      messager.registerTopicListener(FootstepPlannerMessagerAPI.FootstepPlanResponseTopic, request -> publishResultingPlan());
   }

   private void processFootstepPlanningRequestPacket(FootstepPlanningRequestPacket packet)
   {
      if (verbose)
         PrintTools.info("Received planning request over the network.");

      PlanarRegionsListMessage planarRegionsListMessage = packet.getPlanarRegionsListMessage();
      if (planarRegionsListMessage == null)
      {
         this.planarRegionsList = Optional.empty();
      }
      else
      {
         PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
         this.planarRegionsList = Optional.of(planarRegionsList);
      }

      Point3D goalPosition = packet.getGoalPositionInWorld();
      Quaternion goalOrientation = packet.getGoalOrientationInWorld();
      Point3D startPosition = packet.getStanceFootPositionInWorld();
      Quaternion startOrientation = packet.getStanceFootOrientationInWorld();
      FootstepPlannerType plannerType = FootstepPlannerType.fromByte(packet.getRequestedFootstepPlannerType());
      RobotSide initialSupportSide = RobotSide.fromByte(packet.getInitialStanceRobotSide());
      int plannerRequestId = packet.getPlannerRequestId();

      double timeout = packet.getTimeout();
      double horizonLength = packet.getHorizonLength();

      this.planarRegionsList.ifPresent(regions -> messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionDataTopic, regions));
      messager.submitMessage(FootstepPlannerMessagerAPI.StartPositionTopic, startPosition);
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalPositionTopic, goalPosition);

      messager.submitMessage(FootstepPlannerMessagerAPI.StartOrientationTopic, startOrientation);
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalOrientationTopic, goalOrientation);

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTypeTopic, plannerType);

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTimeoutTopic, timeout);
      messager.submitMessage(FootstepPlannerMessagerAPI.InitialSupportSideTopic, initialSupportSide);

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerRequestIdTopic, plannerRequestId);

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerHorizonLengthTopic, horizonLength);

      messager.submitMessage(FootstepPlannerMessagerAPI.ComputePathTopic, true);
   }

   private void processIncomingPlanarRegionMessage(PlanarRegionsListMessage packet)
   {
      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(packet);
      this.planarRegionsList = Optional.of(planarRegionsList);

      if (acceptNewPlanarRegionsReference.get())
         messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionDataTopic, planarRegionsList);
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

      planarRegionsList.ifPresent(regions -> result.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regions)));
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

      planarRegionsList.ifPresent(regions -> result.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regions)));
      result.setFootstepPlanningResult(planningResult.toByte());
      result.setPlanId(plannerRequestIdReference.get());

      outputStatusPublisher.publish(result);

      if (verbose)
         PrintTools.info("Finished planning, but result isn't valid, so publishing blank result on the network.");
   }
}
