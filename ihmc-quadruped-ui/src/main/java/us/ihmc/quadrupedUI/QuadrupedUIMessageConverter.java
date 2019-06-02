package us.ihmc.quadrupedUI;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraphMessagesConverter;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapWithNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.*;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.ros2.RealtimeRos2Node;

import us.ihmc.messager.Messager;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;

public class QuadrupedUIMessageConverter
{
   private static final boolean verbose = false;

   private final RealtimeRos2Node ros2Node;

   private final Messager messager;
   private final String robotName;

   private final AtomicReference<FootstepPlannerParameters> footstepPlannerParametersReference;
   private final AtomicReference<VisibilityGraphsParameters> visibilityGraphParametersReference;
   private final AtomicReference<QuadrupedXGaitSettingsReadOnly> xGaitSettingsReference;
   private final AtomicReference<Point3D> plannerStartPositionReference;
   private final AtomicReference<FootstepPlannerTargetType> plannerStartTargetTypeReference;
   private final AtomicReference<QuadrantDependentList<Point3D>> plannerStartFeetPositionsReference;
   private final AtomicReference<Quaternion> plannerStartOrientationReference;
   private final AtomicReference<Point3D> plannerGoalPositionReference;
   private final AtomicReference<Quaternion> plannerGoalOrientationReference;
   private final AtomicReference<PlanarRegionsList> plannerPlanarRegionReference;
   private final AtomicReference<FootstepPlannerType> plannerTypeReference;
   private final AtomicReference<Double> plannerTimeoutReference;
   private final AtomicReference<RobotQuadrant> plannerInitialSupportQuadrantReference;
   private final AtomicReference<Integer> plannerRequestIdReference;
   private final AtomicReference<Double> plannerHorizonLengthReference;
   private final AtomicReference<Boolean> acceptNewPlanarRegionsReference;
   private final AtomicReference<Integer> currentPlanRequestId;
   private final AtomicReference<Boolean> assumeFlatGround;

   private IHMCRealtimeROS2Publisher<HighLevelStateMessage> desiredHighLevelStatePublisher;
   private IHMCRealtimeROS2Publisher<QuadrupedBodyHeightMessage> bodyHeightPublisher;
   private IHMCRealtimeROS2Publisher<QuadrupedTeleopDesiredVelocity> desiredTeleopVelocityPublisher;
   private IHMCRealtimeROS2Publisher<QuadrupedTeleopDesiredPose> desiredTeleopBodyPosePublisher;
   private IHMCRealtimeROS2Publisher<ToolboxStateMessage> enableStepTeleopPublisher;
   private IHMCRealtimeROS2Publisher<ToolboxStateMessage> enableBodyTeleopPublisher;
   private IHMCRealtimeROS2Publisher<ToolboxStateMessage> enableHeightTeleopPublisher;
   private IHMCRealtimeROS2Publisher<ToolboxStateMessage> enableJoystickPublisher;
   private IHMCRealtimeROS2Publisher<ToolboxStateMessage> enableFootstepPlanningPublisher;
   private IHMCRealtimeROS2Publisher<QuadrupedXGaitSettingsPacket> stepTeleopXGaitSettingsPublisher;
   private IHMCRealtimeROS2Publisher<QuadrupedXGaitSettingsPacket> xboxXGaitSettingsPublisher;
   private IHMCRealtimeROS2Publisher<QuadrupedXGaitSettingsPacket> footstepPlanningXGaitSettingsPublisher;

   private IHMCRealtimeROS2Publisher<QuadrupedFootstepPlannerParametersPacket> footstepPlannerParametersPublisher;
   private IHMCRealtimeROS2Publisher<VisibilityGraphsParametersPacket> visibilityGraphsParametersPublisher;
   private IHMCRealtimeROS2Publisher<PlanningStatisticsRequestMessage> plannerStatisticsRequestPublisher;
   private IHMCRealtimeROS2Publisher<QuadrupedFootstepPlanningRequestPacket> footstepPlanningRequestPublisher;

   private IHMCRealtimeROS2Publisher<QuadrupedTimedStepListMessage> stepListMessagePublisher;
   private IHMCRealtimeROS2Publisher<SoleTrajectoryMessage> soleTrajectoryMessagePublisher;
   private IHMCRealtimeROS2Publisher<QuadrupedRequestedSteppingStateMessage> desiredSteppingStatePublisher;


   public QuadrupedUIMessageConverter(RealtimeRos2Node ros2Node, Messager messager, String robotName)
   {
      this.messager = messager;
      this.robotName = robotName;
      this.ros2Node = ros2Node;

      footstepPlannerParametersReference = messager.createInput(QuadrupedUIMessagerAPI.FootstepPlannerParametersTopic, null);
      visibilityGraphParametersReference = messager.createInput(QuadrupedUIMessagerAPI.VisibilityGraphsParametersTopic, null);
      xGaitSettingsReference = messager.createInput(QuadrupedUIMessagerAPI.XGaitSettingsTopic, null);
      plannerStartPositionReference = messager.createInput(QuadrupedUIMessagerAPI.StartPositionTopic);
      plannerStartTargetTypeReference = messager.createInput(QuadrupedUIMessagerAPI.StartTargetTypeTopic, FootstepPlannerTargetType.POSE_BETWEEN_FEET);
      plannerStartFeetPositionsReference = messager.createInput(QuadrupedUIMessagerAPI.StartFeetPositionTopic);
      plannerStartOrientationReference = messager.createInput(QuadrupedUIMessagerAPI.StartOrientationTopic, new Quaternion());
      plannerGoalPositionReference = messager.createInput(QuadrupedUIMessagerAPI.GoalPositionTopic);
      plannerGoalOrientationReference = messager.createInput(QuadrupedUIMessagerAPI.GoalOrientationTopic, new Quaternion());
      plannerPlanarRegionReference = messager.createInput(QuadrupedUIMessagerAPI.PlanarRegionDataTopic);
      plannerTypeReference = messager.createInput(QuadrupedUIMessagerAPI.PlannerTypeTopic, FootstepPlannerType.A_STAR);
      plannerTimeoutReference = messager.createInput(QuadrupedUIMessagerAPI.PlannerTimeoutTopic, 5.0);
      plannerInitialSupportQuadrantReference = messager.createInput(QuadrupedUIMessagerAPI.InitialSupportQuadrantTopic, RobotQuadrant.FRONT_LEFT);
      plannerRequestIdReference = messager.createInput(QuadrupedUIMessagerAPI.PlannerRequestIdTopic);
      plannerHorizonLengthReference = messager.createInput(QuadrupedUIMessagerAPI.PlannerHorizonLengthTopic);
      acceptNewPlanarRegionsReference = messager.createInput(QuadrupedUIMessagerAPI.AcceptNewPlanarRegionsTopic, true);
      currentPlanRequestId = messager.createInput(QuadrupedUIMessagerAPI.PlannerRequestIdTopic, 0);
      assumeFlatGround = messager.createInput(QuadrupedUIMessagerAPI.AssumeFlatGroundTopic, false);

      registerPubSubs();

      ros2Node.spin();
   }

   public void destroy()
   {
      ros2Node.destroy();
   }

   private void registerPubSubs()
   {
      /* subscribers */
      MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      MessageTopicNameGenerator footstepPlannerPubGenerator = FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName);
      MessageTopicNameGenerator footstepPlannerInputTopicGenerator = FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName);

      ROS2Tools
            .createCallbackSubscription(ros2Node, RobotConfigurationData.class, controllerPubGenerator, s -> processRobotConfigurationData(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, HighLevelStateChangeStatusMessage.class, controllerPubGenerator,
                                           s -> processControllerStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedFootstepStatusMessage.class, controllerPubGenerator,
                                           s -> messager.submitMessage(QuadrupedUIMessagerAPI.FootstepStatusMessageTopic, s.takeNextData()));

      // we want to listen to the incoming request to the planning toolbox
      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedFootstepPlanningRequestPacket.class, footstepPlannerInputTopicGenerator,
                                           s -> processFootstepPlanningRequestPacket(s.takeNextData()));
      // we want to listen to the resulting body path plan from the toolbox
      ROS2Tools.createCallbackSubscription(ros2Node, BodyPathPlanMessage.class, footstepPlannerPubGenerator,
                                           s -> processBodyPathPlanMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, BodyPathPlanStatisticsMessage.class, footstepPlannerPubGenerator,
                                           s -> processBodyPathPlanStatistics(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlannerStatusMessage.class, footstepPlannerPubGenerator,
                                           s -> processFootstepPlannerStatus(s.takeNextData()));
      // we want to listen to the resulting footstep plan from the toolbox
      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedFootstepPlanningToolboxOutputStatus.class, footstepPlannerPubGenerator,
                                           s -> processFootstepPlanningOutputStatus(s.takeNextData()));
      // we want to also listen to incoming REA planar region data.
      ROS2Tools.createCallbackSubscription(ros2Node, PlanarRegionsListMessage.class, REACommunicationProperties.publisherTopicNameGenerator,
                                           s -> processIncomingPlanarRegionMessage(s.takeNextData()));

      ROS2Tools.createCallbackSubscription(ros2Node, VideoPacket.class, ROS2Tools.getDefaultTopicNameGenerator(),
                                           s -> messager.submitMessage(QuadrupedUIMessagerAPI.LeftCameraVideo, s.takeNextData()));

      /* TODO
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepNodeDataListMessage.class,
                                           FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName),
                                           s -> messager.submitMessage(FootstepPlannerMessagerAPI.NodeDataTopic, s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlannerOccupancyMapMessage.class,
                                           FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName),
                                           s -> messager.submitMessage(FootstepPlannerMessagerAPI.OccupancyMapTopic, s.takeNextData()));
                                           */
      // TODO
//      MessageTopicNameGenerator controllerPreviewOutputTopicNameGenerator = ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.WALKING_PREVIEW_TOOLBOX, ROS2TopicQualifier.OUTPUT);
//      ROS2Tools.createCallbackSubscription(ros2Node, WalkingControllerPreviewOutputMessage.class, controllerPreviewOutputTopicNameGenerator, s -> messager.submitMessage(QuadrupedUIMessagerAPI.WalkingPreviewOutput, s.takeNextData()));




      /* publishers */
      MessageTopicNameGenerator controllerSubGenerator = QuadrupedControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);

      MessageTopicNameGenerator bodyTeleopInputTopicGenerator = getTopicNameGenerator(robotName, ROS2Tools.BODY_TELEOP_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT);
      MessageTopicNameGenerator stepTeleopInputTopicGenerator = getTopicNameGenerator(robotName, ROS2Tools.STEP_TELEOP_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT);
      MessageTopicNameGenerator heightTeleopInputTopicGenerator = getTopicNameGenerator(robotName, ROS2Tools.HEIGHT_TELEOP_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT);
      MessageTopicNameGenerator xBoxTeleopInputTopicGenerator = getTopicNameGenerator(robotName, ROS2Tools.XBOX_TELEOP_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT);

      desiredHighLevelStatePublisher = ROS2Tools.createPublisher(ros2Node, HighLevelStateMessage.class, controllerSubGenerator);
      desiredSteppingStatePublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedRequestedSteppingStateMessage.class, controllerSubGenerator);
      soleTrajectoryMessagePublisher = ROS2Tools.createPublisher(ros2Node, SoleTrajectoryMessage.class, controllerSubGenerator);
      bodyHeightPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedBodyHeightMessage.class, controllerSubGenerator);

      enableBodyTeleopPublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, bodyTeleopInputTopicGenerator);
      enableStepTeleopPublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, stepTeleopInputTopicGenerator);
      enableHeightTeleopPublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, heightTeleopInputTopicGenerator);
      enableJoystickPublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, xBoxTeleopInputTopicGenerator);
      enableFootstepPlanningPublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, footstepPlannerInputTopicGenerator);

      stepTeleopXGaitSettingsPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedXGaitSettingsPacket.class, stepTeleopInputTopicGenerator);
      xboxXGaitSettingsPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedXGaitSettingsPacket.class, xBoxTeleopInputTopicGenerator);
      footstepPlanningXGaitSettingsPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedXGaitSettingsPacket.class, footstepPlannerInputTopicGenerator);

      desiredTeleopVelocityPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedTeleopDesiredVelocity.class, stepTeleopInputTopicGenerator);
      desiredTeleopBodyPosePublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedTeleopDesiredPose.class, bodyTeleopInputTopicGenerator);

      footstepPlannerParametersPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedFootstepPlannerParametersPacket.class, footstepPlannerInputTopicGenerator);
      visibilityGraphsParametersPublisher = ROS2Tools.createPublisher(ros2Node, VisibilityGraphsParametersPacket.class, footstepPlannerInputTopicGenerator);
      plannerStatisticsRequestPublisher = ROS2Tools.createPublisher(ros2Node, PlanningStatisticsRequestMessage.class, footstepPlannerInputTopicGenerator);
      footstepPlanningRequestPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedFootstepPlanningRequestPacket.class, footstepPlannerInputTopicGenerator);

      stepListMessagePublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedTimedStepListMessage.class, controllerSubGenerator);

      messager.registerTopicListener(QuadrupedUIMessagerAPI.DesiredControllerNameTopic, this::publishDesiredHighLevelControllerState);
      messager.registerTopicListener(QuadrupedUIMessagerAPI.DesiredSteppingStateNameTopic, this::publishDesiredQuadrupedSteppigState);
      messager.registerTopicListener(QuadrupedUIMessagerAPI.DesiredBodyHeightTopic, this::publishDesiredBodyHeight);

      messager.registerTopicListener(QuadrupedUIMessagerAPI.EnableStepTeleopTopic, this::publishEnableStepTeleop);
      messager.registerTopicListener(QuadrupedUIMessagerAPI.EnableBodyTeleopTopic, this::publishEnableBodyTeleop);
      messager.registerTopicListener(QuadrupedUIMessagerAPI.EnableHeightTeleopTopic, this::publishEnableHeightTeleop);
      messager.registerTopicListener(QuadrupedUIMessagerAPI.EnableJoystickTopic, this::publishEnableJoystick);
      messager.registerTopicListener(QuadrupedUIMessagerAPI.XGaitSettingsTopic, this::publishQuadrupedXGaitSettings);

      messager.registerTopicListener(QuadrupedUIMessagerAPI.ManualStepsListMessageTopic, this::publishStepListMessage);
      messager.registerTopicListener(QuadrupedUIMessagerAPI.SoleTrajectoryMessageTopic, this::publishSoleTrajectoryMessage);
      messager.registerTopicListener(QuadrupedUIMessagerAPI.FootstepPlannerTimedStepsTopic, this::publishStepListMessage);

      messager.registerTopicListener(QuadrupedUIMessagerAPI.ComputePathTopic, request -> requestNewPlan());
      messager.registerTopicListener(QuadrupedUIMessagerAPI.AbortPlanningTopic, request -> requestAbortPlanning());

      messager.registerTopicListener(QuadrupedUIMessagerAPI.DesiredTeleopVelocity, this::publishDesiredVelocity);
      messager.registerTopicListener(QuadrupedUIMessagerAPI.DesiredTeleopBodyPoseTopic, this::publishDesiredBodyPose);
   }

   private void processRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      messager.submitMessage(QuadrupedUIMessagerAPI.RobotConfigurationDataTopic, robotConfigurationData);
   }

   private void processControllerStateChangeMessage(HighLevelStateChangeStatusMessage stateChangeStatusMessage)
   {
      HighLevelControllerName currentState = HighLevelControllerName.fromByte(stateChangeStatusMessage.getEndHighLevelControllerName());
      messager.submitMessage(QuadrupedUIMessagerAPI.CurrentControllerNameTopic, currentState);
   }

   private void processFootstepPlanningRequestPacket(QuadrupedFootstepPlanningRequestPacket packet)
   {
      if (verbose)
         PrintTools.info("Received a planning request.");

      Point3D goalPosition = packet.getGoalPositionInWorld();
      Quaternion goalOrientation = packet.getGoalOrientationInWorld();
      Point3D startPosition = packet.getBodyPositionInWorld();
      Quaternion startOrientation = packet.getBodyOrientationInWorld();
      FootstepPlannerType plannerType = FootstepPlannerType.fromByte(packet.getRequestedFootstepPlannerType());
      RobotQuadrant initialSupportSide = RobotQuadrant.fromByte(packet.getInitialStepRobotQuadrant());
      int plannerRequestId = packet.getPlannerRequestId();

      double timeout = packet.getTimeout();
      double horizonLength = packet.getHorizonLength();

      messager.submitMessage(QuadrupedUIMessagerAPI.StartPositionTopic, startPosition);
      messager.submitMessage(QuadrupedUIMessagerAPI.GoalPositionTopic, goalPosition);

      messager.submitMessage(QuadrupedUIMessagerAPI.StartOrientationTopic, startOrientation);
      messager.submitMessage(QuadrupedUIMessagerAPI.GoalOrientationTopic, goalOrientation);

      messager.submitMessage(QuadrupedUIMessagerAPI.PlannerTypeTopic, plannerType);

      messager.submitMessage(QuadrupedUIMessagerAPI.PlannerTimeoutTopic, timeout);
      messager.submitMessage(QuadrupedUIMessagerAPI.InitialSupportQuadrantTopic, initialSupportSide);

      messager.submitMessage(QuadrupedUIMessagerAPI.PlannerRequestIdTopic, plannerRequestId);

      messager.submitMessage(QuadrupedUIMessagerAPI.PlannerHorizonLengthTopic, horizonLength);
   }

   private void processBodyPathPlanMessage(BodyPathPlanMessage packet)
   {
      PlanarRegionsListMessage planarRegionsListMessage = packet.getPlanarRegionsList();
      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
      FootstepPlanningResult result = FootstepPlanningResult.fromByte(packet.getFootstepPlanningResult());
      List<? extends Point3DReadOnly> bodyPath = packet.getBodyPath();

      messager.submitMessage(QuadrupedUIMessagerAPI.PlanarRegionDataTopic, planarRegionsList);
      messager.submitMessage(QuadrupedUIMessagerAPI.PlanningResultTopic, result);
      messager.submitMessage(QuadrupedUIMessagerAPI.BodyPathDataTopic, bodyPath);

      if (verbose)
         PrintTools.info("Received a body path planning result from the toolbox.");
   }

   private void processBodyPathPlanStatistics(BodyPathPlanStatisticsMessage packet)
   {
      VisibilityMapHolder startVisibilityMap = VisibilityGraphMessagesConverter.convertToSingleSourceVisibilityMap(packet.getStartVisibilityMap());
      VisibilityMapHolder goalVisibilityMap = VisibilityGraphMessagesConverter.convertToSingleSourceVisibilityMap(packet.getGoalVisibilityMap());
      VisibilityMapHolder interRegionVisibilityMap = VisibilityGraphMessagesConverter.convertToInterRegionsVisibilityMap(packet.getInterRegionsMap());

      List<VisibilityMapWithNavigableRegion> navigableRegionList = VisibilityGraphMessagesConverter.convertToNavigableRegionsList(packet.getNavigableRegions());

      /* TODO
      messager.submitMessage(QuadrupedUIMessagerAPI.StartVisibilityMap, startVisibilityMap);
      messager.submitMessage(QuadrupedUIMessagerAPI.GoalVisibilityMap, goalVisibilityMap);
      messager.submitMessage(QuadrupedUIMessagerAPI.VisibilityMapWithNavigableRegionData, navigableRegionList);
      messager.submitMessage(QuadrupedUIMessagerAPI.InterRegionVisibilityMap, interRegionVisibilityMap);
      */
   }

   private void processFootstepPlannerStatus(FootstepPlannerStatusMessage packet)
   {
      messager.submitMessage(QuadrupedUIMessagerAPI.PlannerStatusTopic, FootstepPlannerStatus.fromByte(packet.getFootstepPlannerStatus()));
   }

   private void processFootstepPlanningOutputStatus(QuadrupedFootstepPlanningToolboxOutputStatus packet)
   {
      QuadrupedTimedStepListMessage footstepDataListMessage = packet.getFootstepDataList();
      int plannerRequestId = packet.getPlanId();
      FootstepPlanningResult result = FootstepPlanningResult.fromByte(packet.getFootstepPlanningResult());
      FootstepPlan footstepPlan = convertToFootstepPlan(footstepDataListMessage);
      List<? extends Point3DReadOnly> bodyPath = packet.getBodyPath();
      Pose3D lowLevelGoal = packet.getLowLevelPlannerGoal();

      if (plannerRequestId > currentPlanRequestId.get())
         messager.submitMessage(QuadrupedUIMessagerAPI.PlannerRequestIdTopic, plannerRequestId);

      ThreadTools.sleep(100);

      messager.submitMessage(QuadrupedUIMessagerAPI.FootstepPlanTopic, footstepPlan);
      messager.submitMessage(QuadrupedUIMessagerAPI.ReceivedPlanIdTopic, plannerRequestId);
      messager.submitMessage(QuadrupedUIMessagerAPI.PlanningResultTopic, result);
      messager.submitMessage(QuadrupedUIMessagerAPI.PlannerTimeTakenTopic, packet.getTimeTaken());
      messager.submitMessage(QuadrupedUIMessagerAPI.BodyPathDataTopic, bodyPath);
      if (lowLevelGoal != null)
      {
         messager.submitMessage(QuadrupedUIMessagerAPI.LowLevelGoalPositionTopic, lowLevelGoal.getPosition());
         messager.submitMessage(QuadrupedUIMessagerAPI.LowLevelGoalOrientationTopic, lowLevelGoal.getOrientation());
      }

      if (verbose)
         PrintTools.info("Received a footstep planning result from the toolbox.");
   }

   private void processIncomingPlanarRegionMessage(PlanarRegionsListMessage packet)
   {
      if (acceptNewPlanarRegionsReference.get())
      {
         messager.submitMessage(QuadrupedUIMessagerAPI.PlanarRegionDataTopic, PlanarRegionMessageConverter.convertToPlanarRegionsList(packet));

         if (verbose)
            PrintTools.info("Received updated planner regions.");
      }
   }


   private void publishDesiredHighLevelControllerState(HighLevelControllerName controllerName)
   {
      desiredHighLevelStatePublisher.publish(HumanoidMessageTools.createHighLevelStateMessage(controllerName));
   }

   private void publishDesiredQuadrupedSteppigState(QuadrupedSteppingStateEnum steppingStateName)
   {
      QuadrupedRequestedSteppingStateMessage message = new QuadrupedRequestedSteppingStateMessage();
      message.setQuadrupedSteppingRequestedEvent(steppingStateName.toByte());
      desiredSteppingStatePublisher.publish(message);
   }

   private void publishSoleTrajectoryMessage(SoleTrajectoryMessage soleTrajectoryMessage)
   {
      soleTrajectoryMessagePublisher.publish(soleTrajectoryMessage);
   }

   public void publishDesiredBodyHeight(double desiredBodyHeight)
   {
      QuadrupedBodyHeightMessage bodyHeightMessage = QuadrupedMessageTools.createQuadrupedBodyHeightMessage(0.0, desiredBodyHeight);
      bodyHeightMessage.setControlBodyHeight(true);
      bodyHeightMessage.setIsExpressedInAbsoluteTime(false);
      bodyHeightPublisher.publish(bodyHeightMessage);
   }

   public void publishDesiredBodyPose(QuadrupedTeleopDesiredPose desiredBodyPose)
   {
      desiredTeleopBodyPosePublisher.publish(desiredBodyPose);
   }

   public void publishDesiredVelocity(QuadrupedTeleopDesiredVelocity desiredVelocity)
   {
      desiredTeleopVelocityPublisher.publish(desiredVelocity);
   }

   public void publishEnableStepTeleop(boolean enable)
   {
      if (enable)
         enableStepTeleopPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));
      else
         enableStepTeleopPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.SLEEP));
   }

   public void publishEnableBodyTeleop(boolean enable)
   {
      if (enable)
         enableBodyTeleopPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));
      else
         enableBodyTeleopPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.SLEEP));
   }

   public void publishEnableHeightTeleop(boolean enable)
   {
      if (enable)
         enableHeightTeleopPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));
      else
         enableHeightTeleopPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.SLEEP));
   }

   public void publishEnableJoystick(boolean enable)
   {
      if (enable)
         enableJoystickPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));
      else
         enableJoystickPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.SLEEP));
   }

   public void publishQuadrupedXGaitSettings(QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      QuadrupedXGaitSettingsPacket packet = xGaitSettings.getAsPacket();

      stepTeleopXGaitSettingsPublisher.publish(packet);
      xboxXGaitSettingsPublisher.publish(packet);
      footstepPlanningXGaitSettingsPublisher.publish(packet);
   }

   public void publishStepListMessage(QuadrupedTimedStepListMessage stepListMessage)
   {
      if (verbose)
         PrintTools.info("Sending out footsteps to the robot.");
      stepListMessagePublisher.publish(stepListMessage);
   }

   private void requestNewPlan()
   {
      if (!checkRequireds())
      {
         return;
      }

      enableFootstepPlanningPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));

      if (verbose)
         PrintTools.info("Told the footstep planner toolbox to wake up.");

      footstepPlannerParametersPublisher.publish(footstepPlannerParametersReference.get().getAsPacket());
      footstepPlanningXGaitSettingsPublisher.publish(xGaitSettingsReference.get().getAsPacket());

      VisibilityGraphsParametersPacket visibilityGraphsParametersPacket = new VisibilityGraphsParametersPacket();
      VisibilityGraphsParameters visibilityGraphsParameters = visibilityGraphParametersReference.get();

      FootstepPlannerMessageTools.copyParametersToPacket(visibilityGraphsParametersPacket, visibilityGraphsParameters);
      visibilityGraphsParametersPublisher.publish(visibilityGraphsParametersPacket);

      if (verbose)
         PrintTools.info("Sent out some parameters");

      submitFootstepPlanningRequestPacket();
   }

   private boolean checkRequireds()
   {
      if (plannerStartPositionReference.get() == null && plannerStartTargetTypeReference.get() == FootstepPlannerTargetType.POSE_BETWEEN_FEET)
      {
         PrintTools.warn("Need to set start position.");
         return false;
      }
      if (plannerStartFeetPositionsReference.get() == null && plannerStartTargetTypeReference.get() == FootstepPlannerTargetType.FOOTSTEPS)
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

   private void submitFootstepPlanningRequestPacket()
   {
      QuadrupedFootstepPlanningRequestPacket packet = new QuadrupedFootstepPlanningRequestPacket();
      if (plannerStartTargetTypeReference.get() == FootstepPlannerTargetType.POSE_BETWEEN_FEET)
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
      if (plannerTypeReference.get() != null)
         packet.setRequestedFootstepPlannerType(plannerTypeReference.get().toByte());
      if (plannerRequestIdReference.get() != null)
         packet.setPlannerRequestId(plannerRequestIdReference.get());
      if (plannerHorizonLengthReference.get() != null)
         packet.setHorizonLength(plannerHorizonLengthReference.get());
      if (plannerPlanarRegionReference.get() != null)
         packet.getPlanarRegionsListMessage().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(plannerPlanarRegionReference.get()));
      packet.setAssumeFlatGround(assumeFlatGround.get());

      footstepPlanningRequestPublisher.publish(packet);
   }

   private void requestAbortPlanning()
   {
      if (verbose)
         PrintTools.info("Sending out a sleep request to the footstep planner.");
      enableFootstepPlanningPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.SLEEP));
   }


   private static FootstepPlan convertToFootstepPlan(QuadrupedTimedStepListMessage footstepDataListMessage)
   {
      FootstepPlan footstepPlan = new FootstepPlan();

      for (QuadrupedTimedStepMessage timedStepMessage : footstepDataListMessage.getQuadrupedStepList())
      {
         QuadrupedStepMessage stepMessage = timedStepMessage.getQuadrupedStepMessage();
         TimeIntervalMessage timeInterval = timedStepMessage.getTimeInterval();
         FramePoint3D stepPosition = new FramePoint3D();
         stepPosition.set(stepMessage.getGoalPosition());
         footstepPlan.addFootstep(RobotQuadrant.fromByte(stepMessage.getRobotQuadrant()), stepPosition, stepMessage.getGroundClearance(), timeInterval.getStartTime(), timeInterval.getEndTime());
      }

      return footstepPlan;
   }

   public static QuadrupedUIMessageConverter createConverter(Messager messager, String robotName, DomainFactory.PubSubImplementation implementation)
   {
      RealtimeRos2Node ros2Node = ROS2Tools.createRealtimeRos2Node(implementation, "ihmc_quadruped_ui");
      return createConverter(ros2Node, messager, robotName);
   }

   public static QuadrupedUIMessageConverter createConverter(RealtimeRos2Node ros2Node, Messager messager, String robotName)
   {
      return new QuadrupedUIMessageConverter(ros2Node, messager, robotName);
   }
}
