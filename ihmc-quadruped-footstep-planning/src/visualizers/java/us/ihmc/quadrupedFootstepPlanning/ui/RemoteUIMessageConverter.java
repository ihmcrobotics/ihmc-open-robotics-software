package us.ihmc.quadrupedFootstepPlanning.ui;

import controller_msgs.msg.dds.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraphMessagesConverter;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapWithNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.*;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawPlannerCommunicationProperties;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.tools.PawPlannerMessageTools;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.ros2.RealtimeRos2Node;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * This class is required when using a local version of the Footstep Planner UI and the footstep planning algorithms
 * located in the Footstep Planner Toolbox. It allows users to view the resulting plans calculated by the toolbox. It
 * also allows the user to tune the planner parameters, and request a new plan from the planning toolbox.
 *
 * This class is used to convert the shared memory messages from Java FX used by the Footstep Planner UI
 * to RTPS messages, and vice-versa. It specifically listens to the planning request messages and the output
 * status messages sent to and from the toolbox, and visualizes those. It also allows the user to request a
 * new plan using the local version of the edit footstep planning parameters and a footstep request, requiring
 * conversion from the Java FX messages to the RTPS messages.
 */
public class RemoteUIMessageConverter
{
   private static final boolean verbose = false;

   private final RealtimeRos2Node ros2Node;

   private final Messager messager;

   private final String robotName;

   private final AtomicReference<PawPlannerParameters> plannerParametersReference;
   private final AtomicReference<VisibilityGraphsParameters> visibilityGraphParametersReference;
   private final AtomicReference<Point3D> plannerStartPositionReference;
   private final AtomicReference<Quaternion> plannerStartOrientationReference;
   private final AtomicReference<Point3D> plannerGoalPositionReference;
   private final AtomicReference<Quaternion> plannerGoalOrientationReference;
   private final AtomicReference<PawPlannerTargetType> plannerStartTargetTypeReference;
   private final AtomicReference<QuadrantDependentList<Point3D>> plannerStartFeetPositionsReference;
   private final AtomicReference<PlanarRegionsList> plannerPlanarRegionReference;
   private final AtomicReference<PawPlannerType> plannerTypeReference;
   private final AtomicReference<Double> plannerTimeoutReference;
   private final AtomicReference<Double> plannerBestEffortTimeoutReference;
   private final AtomicReference<RobotQuadrant> plannerInitialSupportQuadrantReference;
   private final AtomicReference<Integer> plannerRequestIdReference;
   private final AtomicReference<Double> plannerHorizonLengthReference;
   private final AtomicReference<Boolean> acceptNewPlanarRegionsReference;
   private final AtomicReference<Integer> currentPlanRequestId;
   private final AtomicReference<Boolean> assumeFlatGround;


   private IHMCRealtimeROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private IHMCRealtimeROS2Publisher<QuadrupedPawPlannerParametersPacket> plannerParametersPublisher;
   private IHMCRealtimeROS2Publisher<VisibilityGraphsParametersPacket> visibilityGraphsParametersPublisher;
   private IHMCRealtimeROS2Publisher<PawPlanningRequestPacket> pawPlanningRequestPublisher;
   private IHMCRealtimeROS2Publisher<PlanningStatisticsRequestMessage> plannerStatisticsRequestPublisher;
   private IHMCRealtimeROS2Publisher<QuadrupedTimedStepListMessage> footstepDataListPublisher;
//   private IHMCRealtimeROS2Publisher<ToolboxStateMessage> walkingPreviewToolboxStatePublisher;
//   private IHMCRealtimeROS2Publisher<WalkingControllerPreviewInputMessage> walkingPreviewRequestPublisher;

   public static RemoteUIMessageConverter createRemoteConverter(Messager messager, String robotName)
   {
      return createConverter(messager, robotName, DomainFactory.PubSubImplementation.FAST_RTPS);
   }

   public static RemoteUIMessageConverter createIntraprocessConverter(Messager messager, String robotName)
   {
      return createConverter(messager, robotName, DomainFactory.PubSubImplementation.INTRAPROCESS);
   }

   public static RemoteUIMessageConverter createConverter(Messager messager, String robotName, DomainFactory.PubSubImplementation implementation)
   {
      RealtimeRos2Node ros2Node = ROS2Tools.createRealtimeRos2Node(implementation, "ihmc_footstep_planner_ui");
      return new RemoteUIMessageConverter(ros2Node, messager, robotName);
   }

   public RemoteUIMessageConverter(RealtimeRos2Node ros2Node, Messager messager, String robotName)
   {
      this.messager = messager;
      this.robotName = robotName;
      this.ros2Node = ros2Node;

      plannerParametersReference = messager.createInput(PawPlannerMessagerAPI.PlannerParametersTopic, null);
      visibilityGraphParametersReference = messager.createInput(PawPlannerMessagerAPI.VisibilityGraphsParametersTopic, null);
      plannerStartPositionReference = messager.createInput(PawPlannerMessagerAPI.StartPositionTopic);
      plannerStartOrientationReference = messager.createInput(PawPlannerMessagerAPI.StartOrientationTopic, new Quaternion());
      plannerStartTargetTypeReference = messager.createInput(PawPlannerMessagerAPI.StartTargetTypeTopic, PawPlannerTargetType.POSE_BETWEEN_FEET);
      plannerStartFeetPositionsReference = messager.createInput(PawPlannerMessagerAPI.StartFeetPositionTopic);
      plannerGoalPositionReference = messager.createInput(PawPlannerMessagerAPI.GoalPositionTopic);
      plannerGoalOrientationReference = messager.createInput(PawPlannerMessagerAPI.GoalOrientationTopic, new Quaternion());
      plannerPlanarRegionReference = messager.createInput(PawPlannerMessagerAPI.PlanarRegionDataTopic);
      plannerTypeReference = messager.createInput(PawPlannerMessagerAPI.PlannerTypeTopic, PawPlannerType.A_STAR);
      plannerTimeoutReference = messager.createInput(PawPlannerMessagerAPI.PlannerTimeoutTopic, 5.0);
      plannerBestEffortTimeoutReference = messager.createInput(PawPlannerMessagerAPI.PlannerBestEffortTimeoutTopic, 1.0);
      plannerInitialSupportQuadrantReference = messager.createInput(PawPlannerMessagerAPI.InitialSupportQuadrantTopic, RobotQuadrant.FRONT_LEFT);
      plannerRequestIdReference = messager.createInput(PawPlannerMessagerAPI.PlannerRequestIdTopic);
      plannerHorizonLengthReference = messager.createInput(PawPlannerMessagerAPI.PlannerHorizonLengthTopic);
      acceptNewPlanarRegionsReference = messager.createInput(PawPlannerMessagerAPI.AcceptNewPlanarRegionsTopic, true);
      currentPlanRequestId = messager.createInput(PawPlannerMessagerAPI.PlannerRequestIdTopic, 0);
      assumeFlatGround = messager.createInput(PawPlannerMessagerAPI.AssumeFlatGroundTopic, false);

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
      // we want to listen to the incoming request to the planning toolbox
      ROS2Tools.createCallbackSubscription(ros2Node, PawPlanningRequestPacket.class,
                                           PawPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName),
                                           s -> processPawPlanningRequestPacket(s.takeNextData()));
      // we want to listen to the resulting body path plan from the toolbox
      ROS2Tools.createCallbackSubscription(ros2Node, BodyPathPlanMessage.class, PawPlannerCommunicationProperties.publisherTopicNameGenerator(robotName),
                                           s -> processBodyPathPlanMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, BodyPathPlanStatisticsMessage.class,
                                           PawPlannerCommunicationProperties.publisherTopicNameGenerator(robotName),
                                           s -> processBodyPathPlanStatistics(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlannerStatusMessage.class,
                                           PawPlannerCommunicationProperties.publisherTopicNameGenerator(robotName),
                                           s -> processFootstepPlannerStatus(s.takeNextData()));
      // we want to listen to the resulting footstep plan from the toolbox
      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedFootstepPlanningToolboxOutputStatus.class,
                                           PawPlannerCommunicationProperties.publisherTopicNameGenerator(robotName),
                                           s -> processFootstepPlanningOutputStatus(s.takeNextData()));
      // we want to also listen to incoming REA planar region data.
      ROS2Tools.createCallbackSubscription(ros2Node, PlanarRegionsListMessage.class, REACommunicationProperties.publisherTopicNameGenerator,
                                           s -> processIncomingPlanarRegionMessage(s.takeNextData()));

      /*
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepNodeDataListMessage.class,
                                           FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName),
                                           s -> messager.submitMessage(FootstepPlannerMessagerAPI.NodeDataTopic, s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlannerOccupancyMapMessage.class,
                                           FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName),
                                           s -> messager.submitMessage(FootstepPlannerMessagerAPI.OccupancyMapTopic, s.takeNextData()));
                                           */

      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                           s -> messager.submitMessage(PawPlannerMessagerAPI.RobotConfigurationDataTopic, s.takeNextData()));

      MessageTopicNameGenerator controllerPreviewOutputTopicNameGenerator = ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.WALKING_PREVIEW_TOOLBOX, ROS2TopicQualifier.OUTPUT);
      ROS2Tools.createCallbackSubscription(ros2Node, WalkingControllerPreviewOutputMessage.class, controllerPreviewOutputTopicNameGenerator, s -> messager.submitMessage(
            PawPlannerMessagerAPI.WalkingPreviewOutput, s.takeNextData()));

      // publishers
      plannerParametersPublisher = ROS2Tools
            .createPublisher(ros2Node, QuadrupedPawPlannerParametersPacket.class, PawPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName));
      visibilityGraphsParametersPublisher = ROS2Tools
            .createPublisher(ros2Node, VisibilityGraphsParametersPacket.class, PawPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName));
      toolboxStatePublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class,
                                                        PawPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName));
      pawPlanningRequestPublisher = ROS2Tools
            .createPublisher(ros2Node, PawPlanningRequestPacket.class, PawPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName));
      plannerStatisticsRequestPublisher = ROS2Tools
            .createPublisher(ros2Node, PlanningStatisticsRequestMessage.class, PawPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName));
      footstepDataListPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedTimedStepListMessage.class, ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName));

//      MessageTopicNameGenerator controllerPreviewInputTopicNameGenerator = ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.WALKING_PREVIEW_TOOLBOX, ROS2TopicQualifier.INPUT);
//      walkingPreviewToolboxStatePublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, controllerPreviewInputTopicNameGenerator);
//      walkingPreviewRequestPublisher = ROS2Tools.createPublisher(ros2Node, WalkingControllerPreviewInputMessage.class, controllerPreviewInputTopicNameGenerator);

      messager.registerTopicListener(PawPlannerMessagerAPI.ComputePathTopic, request -> requestNewPlan());
      messager.registerTopicListener(PawPlannerMessagerAPI.RequestPlannerStatistics, request -> requestPlannerStatistics());
      messager.registerTopicListener(PawPlannerMessagerAPI.AbortPlanningTopic, request -> requestAbortPlanning());
      messager.registerTopicListener(PawPlannerMessagerAPI.FootstepDataListTopic, footstepDataListPublisher::publish);
      /*
      messager.registerTopicListener(FootstepPlannerMessagerAPI.RequestWalkingPreview, request ->
      {
         ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
         toolboxStateMessage.setRequestedToolboxState(ToolboxState.WAKE_UP.toByte());
         walkingPreviewToolboxStatePublisher.publish(toolboxStateMessage);
         walkingPreviewRequestPublisher.publish(request);
      });
      */
   }

   private void processPawPlanningRequestPacket(PawPlanningRequestPacket packet)
   {
      if (verbose)
         PrintTools.info("Received a planning request.");

      Point3D goalPosition = packet.getGoalPositionInWorld();
      Quaternion goalOrientation = packet.getGoalOrientationInWorld();
      Point3D startPosition = packet.getBodyPositionInWorld();
      Quaternion startOrientation = packet.getBodyOrientationInWorld();
      PawPlannerType plannerType = PawPlannerType.fromByte(packet.getRequestedPawPlannerType());
      RobotQuadrant initialSupportSide = RobotQuadrant.fromByte(packet.getInitialStepRobotQuadrant());
      int plannerRequestId = packet.getPlannerRequestId();

      double timeout = packet.getTimeout();
      double horizonLength = packet.getHorizonLength();

      messager.submitMessage(PawPlannerMessagerAPI.StartPositionTopic, startPosition);
      messager.submitMessage(PawPlannerMessagerAPI.GoalPositionTopic, goalPosition);

      messager.submitMessage(PawPlannerMessagerAPI.StartOrientationTopic, startOrientation);
      messager.submitMessage(PawPlannerMessagerAPI.GoalOrientationTopic, goalOrientation);

      messager.submitMessage(PawPlannerMessagerAPI.PlannerTypeTopic, plannerType);

      messager.submitMessage(PawPlannerMessagerAPI.PlannerTimeoutTopic, timeout);
      messager.submitMessage(PawPlannerMessagerAPI.InitialSupportQuadrantTopic, initialSupportSide);

      messager.submitMessage(PawPlannerMessagerAPI.PlannerRequestIdTopic, plannerRequestId);

      messager.submitMessage(PawPlannerMessagerAPI.PlannerHorizonLengthTopic, horizonLength);
   }

   private void processBodyPathPlanMessage(BodyPathPlanMessage packet)
   {
      PlanarRegionsListMessage planarRegionsListMessage = packet.getPlanarRegionsList();
      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
      PawPlanningResult result = PawPlanningResult.fromByte(packet.getFootstepPlanningResult());
      List<? extends Point3DReadOnly> bodyPath = packet.getBodyPath();

      messager.submitMessage(PawPlannerMessagerAPI.PlanarRegionDataTopic, planarRegionsList);
      messager.submitMessage(PawPlannerMessagerAPI.PlanningResultTopic, result);
      messager.submitMessage(PawPlannerMessagerAPI.BodyPathDataTopic, bodyPath);

      if (verbose)
         PrintTools.info("Received a body path planning result from the toolbox.");
   }

   private void processBodyPathPlanStatistics(BodyPathPlanStatisticsMessage packet)
   {
      VisibilityMapHolder startVisibilityMap = VisibilityGraphMessagesConverter.convertToSingleSourceVisibilityMap(packet.getStartVisibilityMap());
      VisibilityMapHolder goalVisibilityMap = VisibilityGraphMessagesConverter.convertToSingleSourceVisibilityMap(packet.getGoalVisibilityMap());
      VisibilityMapHolder interRegionVisibilityMap = VisibilityGraphMessagesConverter.convertToInterRegionsVisibilityMap(packet.getInterRegionsMap());

      List<VisibilityMapWithNavigableRegion> navigableRegionList = VisibilityGraphMessagesConverter.convertToNavigableRegionsList(packet.getNavigableRegions());

      messager.submitMessage(PawPlannerMessagerAPI.StartVisibilityMap, startVisibilityMap);
      messager.submitMessage(PawPlannerMessagerAPI.GoalVisibilityMap, goalVisibilityMap);
      messager.submitMessage(PawPlannerMessagerAPI.VisibilityMapWithNavigableRegionData, navigableRegionList);
      messager.submitMessage(PawPlannerMessagerAPI.InterRegionVisibilityMap, interRegionVisibilityMap);
   }

   private void processFootstepPlannerStatus(FootstepPlannerStatusMessage packet)
   {
      messager.submitMessage(PawPlannerMessagerAPI.PlannerStatusTopic, PawPlannerStatus.fromByte(packet.getFootstepPlannerStatus()));
   }

   private void processFootstepPlanningOutputStatus(QuadrupedFootstepPlanningToolboxOutputStatus packet)
   {
      QuadrupedTimedStepListMessage footstepDataListMessage = packet.getFootstepDataList();
      int plannerRequestId = packet.getPlanId();
      PawPlanningResult result = PawPlanningResult.fromByte(packet.getFootstepPlanningResult());
      PawPlan pawPlan = convertToFootstepPlan(footstepDataListMessage);
      List<? extends Point3DReadOnly> bodyPath = packet.getBodyPath();
      Pose3D lowLevelGoal = packet.getLowLevelPlannerGoal();

      if (plannerRequestId > currentPlanRequestId.get())
         messager.submitMessage(PawPlannerMessagerAPI.PlannerRequestIdTopic, plannerRequestId);
     
      ThreadTools.sleep(100);

      messager.submitMessage(PawPlannerMessagerAPI.FootstepPlanTopic, pawPlan);
      messager.submitMessage(PawPlannerMessagerAPI.ReceivedPlanIdTopic, plannerRequestId);
      messager.submitMessage(PawPlannerMessagerAPI.PlanningResultTopic, result);
      messager.submitMessage(PawPlannerMessagerAPI.PlannerTimeTakenTopic, packet.getTimeTaken());
      messager.submitMessage(PawPlannerMessagerAPI.BodyPathDataTopic, bodyPath);
      if (lowLevelGoal != null)
      {
         messager.submitMessage(PawPlannerMessagerAPI.LowLevelGoalPositionTopic, lowLevelGoal.getPosition());
         messager.submitMessage(PawPlannerMessagerAPI.LowLevelGoalOrientationTopic, lowLevelGoal.getOrientation());
      }

      if (verbose)
         PrintTools.info("Received a footstep planning result from the toolbox.");
   }

   private void processIncomingPlanarRegionMessage(PlanarRegionsListMessage packet)
   {
      if (acceptNewPlanarRegionsReference.get())
      {
         messager.submitMessage(PawPlannerMessagerAPI.PlanarRegionDataTopic, PlanarRegionMessageConverter.convertToPlanarRegionsList(packet));

         if (verbose)
            PrintTools.info("Received updated planner regions.");
      }
   }

   private void requestNewPlan()
   {
      if (!checkRequireds())
      {
         return;
      }

      toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));

      if (verbose)
         PrintTools.info("Told the toolbox to wake up.");
      
      plannerParametersPublisher.publish(plannerParametersReference.get().getAsPacket());

      VisibilityGraphsParametersPacket visibilityGraphsParametersPacket = new VisibilityGraphsParametersPacket();
      VisibilityGraphsParameters visibilityGraphsParameters = visibilityGraphParametersReference.get();

      PawPlannerMessageTools.copyParametersToPacket(visibilityGraphsParametersPacket, visibilityGraphsParameters);
      visibilityGraphsParametersPublisher.publish(visibilityGraphsParametersPacket);
      
      if (verbose)
         PrintTools.info("Sent out some parameters");

      submitFootstepPlanningRequestPacket();
   }

   private boolean checkRequireds()
   {
      if (plannerStartPositionReference.get() == null && plannerStartTargetTypeReference.get() == PawPlannerTargetType.POSE_BETWEEN_FEET)
      {
         PrintTools.warn("Need to set start position.");
         return false;
      }
      if (plannerStartFeetPositionsReference.get() == null && plannerStartTargetTypeReference.get() == PawPlannerTargetType.FOOTSTEPS)
      {
         PrintTools.warn("Need to set start position.");
         return false;
      }
      if (plannerGoalPositionReference.get() == null)
      {
         PrintTools.warn("Need to set goal position.");
         return false;
      }
      return true;
   }

   private void requestPlannerStatistics()
   {
      plannerStatisticsRequestPublisher.publish(new PlanningStatisticsRequestMessage());
   }

   private void requestAbortPlanning()
   {
      if (verbose)
         PrintTools.info("Sending out a sleep request.");
      toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.SLEEP));
   }

   private void submitFootstepPlanningRequestPacket()
   {
      PawPlanningRequestPacket packet = new PawPlanningRequestPacket();
      if (plannerStartTargetTypeReference.get() == PawPlannerTargetType.POSE_BETWEEN_FEET)
      {
         packet.getBodyPositionInWorld().set(plannerStartPositionReference.get());
         packet.getBodyOrientationInWorld().set(plannerStartOrientationReference.get());
      }
      else
      {
         packet.getFrontLeftPositionInWorld().set(plannerStartFeetPositionsReference.get().get(RobotQuadrant.FRONT_LEFT));
         packet.getFrontRightPositionInWorld().set(plannerStartFeetPositionsReference.get().get(RobotQuadrant.FRONT_RIGHT));
         packet.getHindLeftPositionInWorld().set(plannerStartFeetPositionsReference.get().get(RobotQuadrant.HIND_LEFT));
         packet.getHindRightPositionInWorld().set(plannerStartFeetPositionsReference.get().get(RobotQuadrant.HIND_RIGHT));
      }
      packet.setStartTargetType(plannerStartTargetTypeReference.get().toByte());
      packet.getGoalPositionInWorld().set(plannerGoalPositionReference.get());
      packet.getGoalOrientationInWorld().set(plannerGoalOrientationReference.get());
      if (plannerInitialSupportQuadrantReference.get() != null)
         packet.setInitialStepRobotQuadrant(plannerInitialSupportQuadrantReference.get().toByte());
      if (plannerTimeoutReference.get() != null)
         packet.setTimeout(plannerTimeoutReference.get());
      if (plannerBestEffortTimeoutReference.get() != null)
         packet.setBestEffortTimeout(plannerBestEffortTimeoutReference.get());
      if (plannerTypeReference.get() != null)
         packet.setRequestedPawPlannerType(plannerTypeReference.get().toByte());
      if (plannerRequestIdReference.get() != null)
         packet.setPlannerRequestId(plannerRequestIdReference.get());
      if (plannerHorizonLengthReference.get() != null)
         packet.setHorizonLength(plannerHorizonLengthReference.get());
      if (plannerPlanarRegionReference.get() != null)
         packet.getPlanarRegionsListMessage().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(plannerPlanarRegionReference.get()));
      packet.setAssumeFlatGround(assumeFlatGround.get());

      pawPlanningRequestPublisher.publish(packet);
   }

   private static PawPlan convertToFootstepPlan(QuadrupedTimedStepListMessage footstepDataListMessage)
   {
      PawPlan pawPlan = new PawPlan();

      for (QuadrupedTimedStepMessage timedStepMessage : footstepDataListMessage.getQuadrupedStepList())
      {
         QuadrupedStepMessage stepMessage = timedStepMessage.getQuadrupedStepMessage();
         TimeIntervalMessage timeInterval = timedStepMessage.getTimeInterval();
         FramePoint3D stepPosition = new FramePoint3D();
         stepPosition.set(stepMessage.getGoalPosition());
         pawPlan.addPawStep(RobotQuadrant.fromByte(stepMessage.getRobotQuadrant()), stepPosition, stepMessage.getGroundClearance(),
                            timeInterval.getStartTime(), timeInterval.getEndTime());
      }

      return pawPlan;
   }
}
