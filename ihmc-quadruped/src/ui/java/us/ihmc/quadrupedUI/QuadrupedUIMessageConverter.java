package us.ihmc.quadrupedUI;

import controller_msgs.msg.dds.*;
import ihmc_common_msgs.msg.dds.TimeIntervalMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.REAStateRequestMessage;
import perception_msgs.msg.dds.VideoPacket;
import toolbox_msgs.msg.dds.BodyPathPlanMessage;
import toolbox_msgs.msg.dds.FootstepPlannerStatusMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import quadruped_msgs.msg.dds.*;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.QuadrupedAPI;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.*;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerCommunicationProperties;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.tools.PawStepPlannerMessageTools;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.ros2.RealtimeROS2Node;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedUIMessageConverter
{
   private static final boolean verbose = false;

   private final RealtimeROS2Node ros2Node;

   private final Messager messager;
   private final String robotName;

   private final AtomicReference<PawStepPlannerParametersReadOnly> footstepPlannerParametersReference;
   private final AtomicReference<VisibilityGraphsParametersReadOnly> visibilityGraphParametersReference;
   private final AtomicReference<QuadrupedXGaitSettingsReadOnly> xGaitSettingsReference;
   private final AtomicReference<Point3D> plannerStartPositionReference;
   private final AtomicReference<PawStepPlannerTargetType> plannerStartTargetTypeReference;
   private final AtomicReference<QuadrantDependentList<Point3D>> plannerStartFeetPositionsReference;
   private final AtomicReference<Quaternion> plannerStartOrientationReference;
   private final AtomicReference<Point3D> plannerGoalPositionReference;
   private final AtomicReference<Quaternion> plannerGoalOrientationReference;
   private final AtomicReference<PlanarRegionsList> plannerPlanarRegionReference;
   private final AtomicReference<PawStepPlannerType> plannerTypeReference;
   private final AtomicReference<Double> plannerTimeoutReference;
   private final AtomicReference<RobotQuadrant> plannerInitialSupportQuadrantReference;
   private final AtomicReference<Integer> plannerRequestIdReference;
   private final AtomicReference<Double> plannerHorizonLengthReference;
   private final AtomicReference<Boolean> acceptNewPlanarRegionsReference;
   private final AtomicReference<Integer> currentPlanRequestId;
   private final AtomicReference<Boolean> assumeFlatGround;

   private ROS2PublisherBasics<HighLevelStateMessage> desiredHighLevelStatePublisher;
   private ROS2PublisherBasics<QuadrupedBodyHeightMessage> bodyHeightPublisher;
   private ROS2PublisherBasics<QuadrupedTeleopDesiredVelocity> desiredTeleopVelocityPublisher;
   private ROS2PublisherBasics<QuadrupedBodyTrajectoryMessage> desiredBodyPosePublisher;
   private ROS2PublisherBasics<ToolboxStateMessage> enableStepTeleopPublisher;
   private ROS2PublisherBasics<ToolboxStateMessage> enableFootstepPlanningPublisher;
   private ROS2PublisherBasics<QuadrupedXGaitSettingsPacket> stepTeleopXGaitSettingsPublisher;
   private ROS2PublisherBasics<QuadrupedXGaitSettingsPacket> footstepPlanningXGaitSettingsPublisher;

   private ROS2PublisherBasics<PawStepPlannerParametersPacket> footstepPlannerParametersPublisher;
   private ROS2PublisherBasics<VisibilityGraphsParametersPacket> visibilityGraphsParametersPublisher;
   private ROS2PublisherBasics<PawStepPlanningRequestPacket> pawPlanningRequestPublisher;

   private ROS2PublisherBasics<QuadrupedTimedStepListMessage> stepListMessagePublisher;
   private ROS2PublisherBasics<SoleTrajectoryMessage> soleTrajectoryMessagePublisher;
   private ROS2PublisherBasics<QuadrupedRequestedSteppingStateMessage> desiredSteppingStatePublisher;
   private ROS2PublisherBasics<PauseWalkingMessage> pauseWalkingPublisher;
   private ROS2PublisherBasics<AbortWalkingMessage> abortWalkingPublisher;
   private ROS2PublisherBasics<QuadrupedFootLoadBearingMessage> loadBearingRequestPublisher;

   private ROS2PublisherBasics<REAStateRequestMessage> reaStateRequestPublisher;

   public QuadrupedUIMessageConverter(RealtimeROS2Node ros2Node, Messager messager, String robotName)
   {
      this.messager = messager;
      this.robotName = robotName;
      this.ros2Node = ros2Node;

      footstepPlannerParametersReference = messager.createInput(QuadrupedUIMessagerAPI.FootstepPlannerParametersTopic, null);
      visibilityGraphParametersReference = messager.createInput(QuadrupedUIMessagerAPI.VisibilityGraphsParametersTopic, null);
      xGaitSettingsReference = messager.createInput(QuadrupedUIMessagerAPI.XGaitSettingsTopic, null);
      plannerStartPositionReference = messager.createInput(QuadrupedUIMessagerAPI.StartPositionTopic);
      plannerStartTargetTypeReference = messager.createInput(QuadrupedUIMessagerAPI.StartTargetTypeTopic, PawStepPlannerTargetType.POSE_BETWEEN_FEET);
      plannerStartFeetPositionsReference = messager.createInput(QuadrupedUIMessagerAPI.StartFeetPositionTopic);
      plannerStartOrientationReference = messager.createInput(QuadrupedUIMessagerAPI.StartOrientationTopic, new Quaternion());
      plannerGoalPositionReference = messager.createInput(QuadrupedUIMessagerAPI.GoalPositionTopic);
      plannerGoalOrientationReference = messager.createInput(QuadrupedUIMessagerAPI.GoalOrientationTopic, new Quaternion());
      plannerPlanarRegionReference = messager.createInput(QuadrupedUIMessagerAPI.PlanarRegionDataTopic);
      plannerTypeReference = messager.createInput(QuadrupedUIMessagerAPI.PlannerTypeTopic, PawStepPlannerType.A_STAR);
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
      ROS2Topic controllerOutputTopic = QuadrupedAPI.getQuadrupedControllerOutputTopic(robotName);
      ROS2Topic footstepPlannerOutputTopic = PawStepPlannerCommunicationProperties.outputTopic(robotName);
      ROS2Topic footstepPlannerInputTopicGenerator = PawStepPlannerCommunicationProperties.inputTopic(robotName);

      ROS2Tools
            .createCallbackSubscriptionTypeNamed(ros2Node, RobotConfigurationData.class, controllerOutputTopic, s -> processRobotConfigurationData(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, HighLevelStateChangeStatusMessage.class, controllerOutputTopic,
                                           s -> processHighLevelStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, QuadrupedSteppingStateChangeMessage.class, controllerOutputTopic,
                                           s -> processSteppingStateStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, QuadrupedFootstepStatusMessage.class, controllerOutputTopic,
                                           s -> messager.submitMessage(QuadrupedUIMessagerAPI.FootstepStatusMessageTopic, s.takeNextData()));

      // we want to listen to the incoming request to the planning toolbox
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, PawStepPlanningRequestPacket.class, footstepPlannerInputTopicGenerator,
                                           s -> processPawPlanningRequestPacket(s.takeNextData()));
      // we want to listen to the resulting body path plan from the toolbox
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, BodyPathPlanMessage.class, footstepPlannerOutputTopic,
                                           s -> processBodyPathPlanMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepPlannerStatusMessage.class, footstepPlannerOutputTopic,
                                           s -> processFootstepPlannerStatus(s.takeNextData()));
      // we want to listen to the resulting footstep plan from the toolbox
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, PawStepPlanningToolboxOutputStatus.class, footstepPlannerOutputTopic,
                                           s -> processFootstepPlanningOutputStatus(s.takeNextData()));
      // we want to also listen to incoming REA planar region data.
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, PlanarRegionsListMessage.class, REACommunicationProperties.outputTopic,
                                           s -> processIncomingPlanarRegionMessage(s.takeNextData()));

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, VideoPacket.class, ROS2Tools.IHMC_ROOT,
                                           s -> messager.submitMessage(QuadrupedUIMessagerAPI.LeftCameraVideo, s.takeNextData()));

      /* TODO
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepNodeDataListMessage.class,
                                           FootstepPlannerCommunicationProperties.outputTopic(robotName),
                                           s -> messager.submitMessage(FootstepPlannerMessagerAPI.NodeDataTopic, s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepPlannerOccupancyMapMessage.class,
                                           FootstepPlannerCommunicationProperties.outputTopic(robotName),
                                           s -> messager.submitMessage(FootstepPlannerMessagerAPI.OccupancyMapTopic, s.takeNextData()));
                                           */
      // TODO
//      MessageTopicName controllerPreviewOutputTopic = ROS2Tools.getTopicName(robotName, ROS2Tools.WALKING_PREVIEW_TOOLBOX, ROS2TopicQualifier.OUTPUT);
//      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, WalkingControllerPreviewOutputMessage.class, controllerPreviewOutputTopic, s -> messager.submitMessage(QuadrupedUIMessagerAPI.WalkingPreviewOutput, s.takeNextData()));




      /* publishers */
      ROS2Topic controllerInputTopic = QuadrupedAPI.getQuadrupedControllerInputTopic(robotName);

      ROS2Topic stepTeleopInputTopicGenerator = ToolboxAPIs.STEP_TELEOP_TOOLBOX.withRobot(robotName)
                                                                               .withInput();

      desiredHighLevelStatePublisher = ros2Node.createPublisher(controllerInputTopic.withTypeName(HighLevelStateMessage.class));
      desiredSteppingStatePublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(QuadrupedRequestedSteppingStateMessage.class)
                                                                        .withTopic(controllerInputTopic));
      soleTrajectoryMessagePublisher = ros2Node.createPublisher(controllerInputTopic.withTypeName(SoleTrajectoryMessage.class));
      pauseWalkingPublisher = ros2Node.createPublisher(controllerInputTopic.withTypeName(PauseWalkingMessage.class));
      abortWalkingPublisher = ros2Node.createPublisher(controllerInputTopic.withTypeName(AbortWalkingMessage.class));
      loadBearingRequestPublisher = ros2Node.createPublisher(controllerInputTopic.withTypeName(QuadrupedFootLoadBearingMessage.class));
      bodyHeightPublisher = ros2Node.createPublisher(controllerInputTopic.withTypeName(QuadrupedBodyHeightMessage.class));
      desiredBodyPosePublisher = ros2Node.createPublisher(controllerInputTopic.withTypeName(QuadrupedBodyTrajectoryMessage.class));

      enableStepTeleopPublisher = ros2Node.createPublisher(stepTeleopInputTopicGenerator.withTypeName(ToolboxStateMessage.class));
      enableFootstepPlanningPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(ToolboxStateMessage.class)
                                                                          .withTopic(footstepPlannerInputTopicGenerator));

      stepTeleopXGaitSettingsPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(QuadrupedXGaitSettingsPacket.class)
                                                                           .withTopic(stepTeleopInputTopicGenerator));
      footstepPlanningXGaitSettingsPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(QuadrupedXGaitSettingsPacket.class)
                                                                                 .withTopic(footstepPlannerInputTopicGenerator));

      desiredTeleopVelocityPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(QuadrupedTeleopDesiredVelocity.class)
                                                                         .withTopic(stepTeleopInputTopicGenerator));

      footstepPlannerParametersPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(PawStepPlannerParametersPacket.class)
                                                                             .withTopic(footstepPlannerInputTopicGenerator));
      visibilityGraphsParametersPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(VisibilityGraphsParametersPacket.class)
                                                                              .withTopic(footstepPlannerInputTopicGenerator));
      pawPlanningRequestPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(PawStepPlanningRequestPacket.class)
                                                                      .withTopic(footstepPlannerInputTopicGenerator));

      stepListMessagePublisher = ros2Node.createPublisher(controllerInputTopic.withTypeName(QuadrupedTimedStepListMessage.class));

      reaStateRequestPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(REAStateRequestMessage.class)
                                                                   .withTopic(REACommunicationProperties.inputTopic));

      messager.addTopicListener(QuadrupedUIMessagerAPI.DesiredControllerNameTopic, this::publishDesiredHighLevelControllerState);
      messager.addTopicListener(QuadrupedUIMessagerAPI.DesiredSteppingStateNameTopic, this::publishDesiredQuadrupedSteppigState);
      messager.addTopicListener(QuadrupedUIMessagerAPI.DesiredBodyHeightTopic, this::publishDesiredBodyHeight);

      messager.addTopicListener(QuadrupedUIMessagerAPI.EnableStepTeleopTopic, this::publishEnableStepTeleop);
      messager.addTopicListener(QuadrupedUIMessagerAPI.XGaitSettingsTopic, this::publishQuadrupedXGaitSettings);

      messager.addTopicListener(QuadrupedUIMessagerAPI.ManualStepsListMessageTopic, this::publishStepListMessage);
      messager.addTopicListener(QuadrupedUIMessagerAPI.SoleTrajectoryMessageTopic, this::publishSoleTrajectoryMessage);
      messager.addTopicListener(QuadrupedUIMessagerAPI.FootstepPlannerTimedStepsTopic, this::publishStepListMessage);

      messager.addTopicListener(QuadrupedUIMessagerAPI.ComputePathTopic, request -> requestNewPlan());
      messager.addTopicListener(QuadrupedUIMessagerAPI.AbortPlanningTopic, request -> requestAbortPlanning());

      messager.addTopicListener(QuadrupedUIMessagerAPI.DesiredTeleopVelocity, this::publishDesiredVelocity);
      messager.addTopicListener(QuadrupedUIMessagerAPI.DesiredBodyTrajectoryTopic, desiredBodyPosePublisher::publish);
      messager.addTopicListener(QuadrupedUIMessagerAPI.AbortWalkingTopic, m -> abortWalkingPublisher.publish(new AbortWalkingMessage()));
      messager.addTopicListener(QuadrupedUIMessagerAPI.PauseWalkingTopic, request ->
      {
         PauseWalkingMessage pauseWalkingMessage = new PauseWalkingMessage();
         pauseWalkingMessage.setPause(request);
         pauseWalkingPublisher.publish(pauseWalkingMessage);
      });
      messager.addTopicListener(QuadrupedUIMessagerAPI.LoadBearingRequestTopic, quadrant ->
      {
         QuadrupedFootLoadBearingMessage loadBearingMessage = new QuadrupedFootLoadBearingMessage();
         loadBearingMessage.setRobotQuadrant(quadrant.toByte());
         loadBearingRequestPublisher.publish(loadBearingMessage);
      });

      messager.addTopicListener(QuadrupedUIMessagerAPI.PlanarRegionDataClearTopic, m ->
      {
         REAStateRequestMessage clearRequest = new REAStateRequestMessage();
         clearRequest.setRequestClear(true);
         reaStateRequestPublisher.publish(clearRequest);
      });
   }

   private void processRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      messager.submitMessage(QuadrupedUIMessagerAPI.RobotConfigurationDataTopic, robotConfigurationData);
   }

   private void processHighLevelStateChangeMessage(HighLevelStateChangeStatusMessage stateChangeStatusMessage)
   {
      HighLevelControllerName currentState = HighLevelControllerName.fromByte(stateChangeStatusMessage.getEndHighLevelControllerName());
      messager.submitMessage(QuadrupedUIMessagerAPI.CurrentControllerNameTopic, currentState);
   }

   private void processSteppingStateStateChangeMessage(QuadrupedSteppingStateChangeMessage stateChangeStatusMessage)
   {
      QuadrupedSteppingStateEnum currentState = QuadrupedSteppingStateEnum.fromByte(stateChangeStatusMessage.getEndQuadrupedSteppingStateEnum());
      messager.submitMessage(QuadrupedUIMessagerAPI.CurrentSteppingStateNameTopic, currentState);
   }

   private void processPawPlanningRequestPacket(PawStepPlanningRequestPacket packet)
   {
      if (verbose)
         PrintTools.info("Received a planning request.");

      Point3D goalPosition = packet.getGoalPositionInWorld();
      Quaternion goalOrientation = packet.getGoalOrientationInWorld();
      Point3D startPosition = packet.getBodyPositionInWorld();
      Quaternion startOrientation = packet.getBodyOrientationInWorld();
      PawStepPlannerType plannerType = PawStepPlannerType.fromByte(packet.getRequestedPawPlannerType());
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
      PawStepPlanningResult result = PawStepPlanningResult.fromByte(packet.getFootstepPlanningResult());
      List<? extends Pose3DReadOnly> bodyPath = packet.getBodyPath();

      messager.submitMessage(QuadrupedUIMessagerAPI.PlanarRegionDataTopic, planarRegionsList);
      messager.submitMessage(QuadrupedUIMessagerAPI.PlanningResultTopic, result);
      messager.submitMessage(QuadrupedUIMessagerAPI.BodyPathDataTopic, bodyPath);

      if (verbose)
         PrintTools.info("Received a body path planning result from the toolbox.");
   }

   private void processFootstepPlannerStatus(FootstepPlannerStatusMessage packet)
   {
      messager.submitMessage(QuadrupedUIMessagerAPI.PlannerStatusTopic, PawStepPlannerStatus.fromByte(packet.getFootstepPlannerStatus()));
   }

   private void processFootstepPlanningOutputStatus(PawStepPlanningToolboxOutputStatus packet)
   {
      QuadrupedTimedStepListMessage footstepDataListMessage = packet.getFootstepDataList();
      int plannerRequestId = packet.getPlanId();
      PawStepPlanningResult result = PawStepPlanningResult.fromByte(packet.getFootstepPlanningResult());
      PawStepPlan pawStepPlan = convertToFootstepPlan(footstepDataListMessage);
      List<? extends Pose3DReadOnly> bodyPath = packet.getBodyPath();
      Pose3D lowLevelGoal = packet.getLowLevelPlannerGoal();

      if (plannerRequestId > currentPlanRequestId.get())
         messager.submitMessage(QuadrupedUIMessagerAPI.PlannerRequestIdTopic, plannerRequestId);

      ThreadTools.sleep(100);

      messager.submitMessage(QuadrupedUIMessagerAPI.FootstepPlanTopic, pawStepPlan);
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

   public void publishQuadrupedXGaitSettings(QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      QuadrupedXGaitSettingsPacket packet = xGaitSettings.getAsPacket();

      stepTeleopXGaitSettingsPublisher.publish(packet);
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
      VisibilityGraphsParametersReadOnly visibilityGraphsParameters = visibilityGraphParametersReference.get();

      PawStepPlannerMessageTools.copyParametersToPacket(visibilityGraphsParametersPacket, visibilityGraphsParameters);
      visibilityGraphsParametersPublisher.publish(visibilityGraphsParametersPacket);

      if (verbose)
         PrintTools.info("Sent out some parameters");

      submitFootstepPlanningRequestPacket();
   }

   private boolean checkRequireds()
   {
      if (plannerStartPositionReference.get() == null && plannerStartTargetTypeReference.get() == PawStepPlannerTargetType.POSE_BETWEEN_FEET)
      {
         PrintTools.warn("Need to set start position.");
         return false;
      }
      if (plannerStartFeetPositionsReference.get() == null && plannerStartTargetTypeReference.get() == PawStepPlannerTargetType.FOOTSTEPS)
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
      PawStepPlanningRequestPacket packet = new PawStepPlanningRequestPacket();
      if (plannerStartTargetTypeReference.get() == PawStepPlannerTargetType.POSE_BETWEEN_FEET)
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

   private void requestAbortPlanning()
   {
      if (verbose)
         PrintTools.info("Sending out a sleep request to the footstep planner.");
      enableFootstepPlanningPublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.SLEEP));
   }


   private static PawStepPlan convertToFootstepPlan(QuadrupedTimedStepListMessage footstepDataListMessage)
   {
      PawStepPlan pawStepPlan = new PawStepPlan();

      for (QuadrupedTimedStepMessage timedStepMessage : footstepDataListMessage.getQuadrupedStepList())
      {
         QuadrupedStepMessage stepMessage = timedStepMessage.getQuadrupedStepMessage();
         TimeIntervalMessage timeInterval = timedStepMessage.getTimeInterval();
         FramePoint3D stepPosition = new FramePoint3D();
         stepPosition.set(stepMessage.getGoalPosition());
         pawStepPlan.addPawStep(RobotQuadrant.fromByte(stepMessage.getRobotQuadrant()), stepPosition, stepMessage.getGroundClearance(), timeInterval.getStartTime(), timeInterval.getEndTime());
      }

      return pawStepPlan;
   }

   public static QuadrupedUIMessageConverter createConverter(Messager messager, String robotName, DomainFactory.PubSubImplementation implementation)
   {
      RealtimeROS2Node ros2Node = ROS2Tools.createRealtimeROS2Node(implementation, "ihmc_quadruped_ui");
      return createConverter(ros2Node, messager, robotName);
   }

   public static QuadrupedUIMessageConverter createConverter(RealtimeROS2Node ros2Node, Messager messager, String robotName)
   {
      return new QuadrupedUIMessageConverter(ros2Node, messager, robotName);
   }
}
