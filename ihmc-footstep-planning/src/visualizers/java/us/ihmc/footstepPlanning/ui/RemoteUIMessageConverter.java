package us.ihmc.footstepPlanning.ui;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.footstepPlanning.BodyPathPlanningResult;
import us.ihmc.footstepPlanning.FootstepPlannerRequestedAction;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerOccupancyMap;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;

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

   private final RealtimeROS2Node ros2Node;

   private final Messager messager;

   private final String robotName;

   private final AtomicReference<VisibilityGraphsParametersReadOnly> visibilityGraphParametersReference;
   private final AtomicReference<FootstepPlannerParametersReadOnly> plannerParametersReference;
   private final AtomicReference<SwingPlannerParametersReadOnly> swingPlannerParametersReference;
   private final AtomicReference<Pose3DReadOnly> leftFootPose;
   private final AtomicReference<Pose3DReadOnly> rightFootPose;
   private final AtomicReference<Pose3DReadOnly> goalLeftFootPose;
   private final AtomicReference<Pose3DReadOnly> goalRightFootPose;
   private final AtomicReference<Boolean> snapGoalSteps;
   private final AtomicReference<Boolean> abortIfGoalStepSnapFails;
   private final AtomicReference<PlanarRegionsList> plannerPlanarRegionReference;
   private final AtomicReference<Boolean> planBodyPath;
   private final AtomicReference<Boolean> performAStarSearch;
   private final AtomicReference<SwingPlannerType> requestedSwingPlanner;
   private final AtomicReference<Double> plannerTimeoutReference;
   private final AtomicReference<Integer> maxIterations;
   private final AtomicReference<RobotSide> plannerInitialSupportSideReference;
   private final AtomicReference<Double> pathHeadingReference;
   private final AtomicReference<Integer> plannerRequestIdReference;
   private final AtomicReference<Double> plannerHorizonLengthReference;
   private final AtomicReference<Boolean> acceptNewPlanarRegionsReference;
   private final AtomicReference<Integer> currentPlanRequestId;
   private final AtomicReference<Boolean> assumeFlatGround;
   private final AtomicReference<Boolean> ignorePartialFootholds;
   private final AtomicReference<Double> goalDistanceProximity;
   private final AtomicReference<Double> goalYawProximity;

   private final AtomicReference<ConvexPolygon2D> postProcessingLeftFootSupportPolygonReference;
   private final AtomicReference<ConvexPolygon2D> postProcessingRightFootSupportPolygonReference;
   private final AtomicReference<FootstepDataListMessage> footstepPlanResponseReference;
   private final AtomicReference<PlanarRegionsList> planarRegionListReference;

   private IHMCRealtimeROS2Publisher<FootstepPlannerActionMessage> plannerActionPublisher;
   private IHMCRealtimeROS2Publisher<VisibilityGraphsParametersPacket> visibilityGraphsParametersPublisher;
   private IHMCRealtimeROS2Publisher<FootstepPlannerParametersPacket> plannerParametersPublisher;
   private IHMCRealtimeROS2Publisher<SwingPlannerParametersPacket> swingPlannerParametersPublisher;

   private IHMCRealtimeROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher;
   private IHMCRealtimeROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private IHMCRealtimeROS2Publisher<GoHomeMessage> goHomePublisher;
   private IHMCRealtimeROS2Publisher<ToolboxStateMessage> walkingPreviewToolboxStatePublisher;
   private IHMCRealtimeROS2Publisher<WalkingControllerPreviewInputMessage> walkingPreviewRequestPublisher;

   private IHMCRealtimeROS2Publisher<ArmTrajectoryMessage> armTrajectoryMessagePublisher;
   private IHMCRealtimeROS2Publisher<HandTrajectoryMessage> handTrajectoryMessagePublisher;
   private IHMCRealtimeROS2Publisher<FootTrajectoryMessage> footTrajectoryMessagePublisher;
   private IHMCRealtimeROS2Publisher<ChestTrajectoryMessage> chestTrajectoryMessagePublisher;
   private IHMCRealtimeROS2Publisher<SpineTrajectoryMessage> spineTrajectoryMessagePublisher;
   private IHMCRealtimeROS2Publisher<HeadTrajectoryMessage> headTrajectoryMessagePublisher;
   private IHMCRealtimeROS2Publisher<NeckTrajectoryMessage> neckTrajectoryMessagePublisher;

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
      RealtimeROS2Node ros2Node = ROS2Tools.createRealtimeROS2Node(implementation, "ihmc_footstep_planner_ui");
      return new RemoteUIMessageConverter(ros2Node, messager, robotName);
   }

   public RemoteUIMessageConverter(RealtimeROS2Node ros2Node, Messager messager, String robotName)
   {
      this.messager = messager;
      this.robotName = robotName;
      this.ros2Node = ros2Node;

      plannerParametersReference = messager.createInput(FootstepPlannerMessagerAPI.PlannerParameters, null);
      visibilityGraphParametersReference = messager.createInput(FootstepPlannerMessagerAPI.VisibilityGraphsParameters, null);
      swingPlannerParametersReference = messager.createInput(FootstepPlannerMessagerAPI.SwingPlannerParameters, null);

      leftFootPose = messager.createInput(FootstepPlannerMessagerAPI.LeftFootPose);
      rightFootPose = messager.createInput(FootstepPlannerMessagerAPI.RightFootPose);
      goalLeftFootPose = messager.createInput(FootstepPlannerMessagerAPI.LeftFootGoalPose);
      goalRightFootPose = messager.createInput(FootstepPlannerMessagerAPI.RightFootGoalPose);
      snapGoalSteps = messager.createInput(FootstepPlannerMessagerAPI.SnapGoalSteps);
      abortIfGoalStepSnapFails = messager.createInput(FootstepPlannerMessagerAPI.AbortIfGoalStepSnapFails);
      plannerPlanarRegionReference = messager.createInput(FootstepPlannerMessagerAPI.PlanarRegionData);
      planBodyPath = messager.createInput(FootstepPlannerMessagerAPI.PlanBodyPath, false);
      performAStarSearch = messager.createInput(FootstepPlannerMessagerAPI.PerformAStarSearch, true);
      requestedSwingPlanner = messager.createInput(FootstepPlannerMessagerAPI.RequestedSwingPlannerType, SwingPlannerType.NONE);
      plannerTimeoutReference = messager.createInput(FootstepPlannerMessagerAPI.PlannerTimeout, 5.0);
      maxIterations = messager.createInput(FootstepPlannerMessagerAPI.MaxIterations, -1);
      plannerInitialSupportSideReference = messager.createInput(FootstepPlannerMessagerAPI.InitialSupportSide, RobotSide.LEFT);
      pathHeadingReference = messager.createInput(FootstepPlannerMessagerAPI.RequestedFootstepPlanHeading, 0.0);
      plannerRequestIdReference = messager.createInput(FootstepPlannerMessagerAPI.PlannerRequestId);
      plannerHorizonLengthReference = messager.createInput(FootstepPlannerMessagerAPI.PlannerHorizonLength);
      acceptNewPlanarRegionsReference = messager.createInput(FootstepPlannerMessagerAPI.AcceptNewPlanarRegions, true);
      currentPlanRequestId = messager.createInput(FootstepPlannerMessagerAPI.PlannerRequestId, 0);
      assumeFlatGround = messager.createInput(FootstepPlannerMessagerAPI.AssumeFlatGround, false);
      ignorePartialFootholds = messager.createInput(FootstepPlannerMessagerAPI.IgnorePartialFootholds, false);
      goalDistanceProximity = messager.createInput(FootstepPlannerMessagerAPI.GoalDistanceProximity, 0.0);
      goalYawProximity = messager.createInput(FootstepPlannerMessagerAPI.GoalYawProximity, 0.0);

      postProcessingLeftFootSupportPolygonReference = messager.createInput(FootstepPlannerMessagerAPI.LeftFootStartSupportPolygon, null);
      postProcessingRightFootSupportPolygonReference = messager.createInput(FootstepPlannerMessagerAPI.RightFootStartSupportPolygon, null);
      footstepPlanResponseReference = messager.createInput(FootstepPlannerMessagerAPI.FootstepPlanResponse, null);
      planarRegionListReference = messager.createInput(FootstepPlannerMessagerAPI.PlanarRegionData, null);

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
      // we want to listen to the incoming request to the planning toolbox
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepPlanningRequestPacket.class,
                                                    FootstepPlannerCommunicationProperties.inputTopic(robotName),
                                           s -> processFootstepPlanningRequestPacket(s.takeNextData()));
      // we want to listen to the resulting body path plan from the toolbox
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, BodyPathPlanMessage.class, FootstepPlannerCommunicationProperties.outputTopic(robotName),
                                           s -> processBodyPathPlanMessage(s.takeNextData()));
      // we want to listen to the resulting footstep plan from the toolbox
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepPlanningToolboxOutputStatus.class,
                                                    FootstepPlannerCommunicationProperties.outputTopic(robotName),
                                           s -> processFootstepPlanningOutputStatus(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepPlannerOccupancyMapMessage.class,
                                                    FootstepPlannerCommunicationProperties.outputTopic(robotName),
                                           s -> messager.submitMessage(FootstepPlannerMessagerAPI.OccupancyMap, new PlannerOccupancyMap(s.takeNextData())));
      // we want to also listen to incoming REA planar region data.
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, PlanarRegionsListMessage.class, REACommunicationProperties.outputTopic,
                                           s -> processIncomingPlanarRegionMessage(s.takeNextData()));

      // things from the controller
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, RobotConfigurationData.class, ROS2Tools.getControllerOutputTopic(robotName),
                                           s -> messager.submitMessage(FootstepPlannerMessagerAPI.RobotConfigurationData, s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, CapturabilityBasedStatus.class, ROS2Tools.getControllerOutputTopic(robotName),
                                           s -> processCapturabilityStatus(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepStatusMessage.class, ROS2Tools.getControllerOutputTopic(robotName),
                                           s -> messager.submitMessage(FootstepPlannerMessagerAPI.FootstepStatusMessage, s.takeNextData()));

      ROS2Topic controllerPreviewOutputTopic = ROS2Tools.WALKING_PREVIEW_TOOLBOX.withRobot(robotName)
                                                                                    .withOutput();
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, WalkingControllerPreviewOutputMessage.class, controllerPreviewOutputTopic, s -> messager.submitMessage(FootstepPlannerMessagerAPI.WalkingPreviewOutput, s.takeNextData()));

      // publishers
      plannerParametersPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                      FootstepPlannerParametersPacket.class,
                                                                      FootstepPlannerCommunicationProperties.inputTopic(robotName));
      visibilityGraphsParametersPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                               VisibilityGraphsParametersPacket.class,
                                                                               FootstepPlannerCommunicationProperties.inputTopic(robotName));
      plannerActionPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                  FootstepPlannerActionMessage.class,
                                                                  FootstepPlannerCommunicationProperties.inputTopic(robotName));
      swingPlannerParametersPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                           SwingPlannerParametersPacket.class,
                                                                           ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName).withInput());

      footstepPlanningRequestPublisher = ROS2Tools
            .createPublisherTypeNamed(ros2Node, FootstepPlanningRequestPacket.class, FootstepPlannerCommunicationProperties.inputTopic(robotName));
      footstepDataListPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, FootstepDataListMessage.class, ROS2Tools.getControllerInputTopic(robotName));
      goHomePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, GoHomeMessage.class, ROS2Tools.getControllerInputTopic(robotName));

      ROS2Topic controllerPreviewInputTopic = ROS2Tools.WALKING_PREVIEW_TOOLBOX.withRobot(robotName)
                                                                                   .withInput();
      walkingPreviewToolboxStatePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, ToolboxStateMessage.class, controllerPreviewInputTopic);
      walkingPreviewRequestPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, WalkingControllerPreviewInputMessage.class, controllerPreviewInputTopic);
      armTrajectoryMessagePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, ArmTrajectoryMessage.class, ROS2Tools.getControllerInputTopic(robotName));
      handTrajectoryMessagePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, HandTrajectoryMessage.class, ROS2Tools.getControllerInputTopic(robotName));
      footTrajectoryMessagePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, FootTrajectoryMessage.class, ROS2Tools.getControllerInputTopic(robotName));
      chestTrajectoryMessagePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, ChestTrajectoryMessage.class, ROS2Tools.getControllerInputTopic(robotName));
      spineTrajectoryMessagePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, SpineTrajectoryMessage.class, ROS2Tools.getControllerInputTopic(robotName));
      headTrajectoryMessagePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, HeadTrajectoryMessage.class, ROS2Tools.getControllerInputTopic(robotName));
      neckTrajectoryMessagePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, NeckTrajectoryMessage.class, ROS2Tools.getControllerInputTopic(robotName));

      messager.registerTopicListener(FootstepPlannerMessagerAPI.ComputePath, request -> requestNewPlan());
      messager.registerTopicListener(FootstepPlannerMessagerAPI.HaltPlanning, request -> requestHaltPlanning());
      messager.registerTopicListener(FootstepPlannerMessagerAPI.GoHomeTopic, goHomePublisher::publish);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.RequestWalkingPreview, request ->
      {
         ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
         toolboxStateMessage.setRequestedToolboxState(ToolboxState.WAKE_UP.toByte());
         walkingPreviewToolboxStatePublisher.publish(toolboxStateMessage);
         walkingPreviewRequestPublisher.publish(request);
      });

      messager.registerTopicListener(FootstepPlannerMessagerAPI.FootstepPlanToRobot, footstepDataListPublisher::publish);

      IHMCRealtimeROS2Publisher<BipedalSupportPlanarRegionParametersMessage> supportRegionsParametersPublisher = ROS2Tools
            .createPublisherTypeNamed(ros2Node, BipedalSupportPlanarRegionParametersMessage.class,
                                      ROS2Tools.BIPED_SUPPORT_REGION_PUBLISHER.withRobot(robotName)
                                                .withInput());
      messager.registerTopicListener(FootstepPlannerMessagerAPI.BipedalSupportRegionsParameters, supportRegionsParametersPublisher::publish);

      messager.registerTopicListener(FootstepPlannerMessagerAPI.RequestedArmJointAngles, request ->
      {
         RobotSide robotSide = request.getKey();
         double[] jointAngles = request.getValue();
         double trajectoryTime = 4.0;

         ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryTime, jointAngles);
         armTrajectoryMessagePublisher.publish(armTrajectoryMessage);

      });

      messager.registerTopicListener(FootstepPlannerMessagerAPI.ArmTrajectoryMessageTopic, armTrajectoryMessagePublisher::publish);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.HandTrajectoryMessageTopic, handTrajectoryMessagePublisher::publish);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.FootTrajectoryMessageTopic, footTrajectoryMessagePublisher::publish);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ChestTrajectoryMessageTopic, chestTrajectoryMessagePublisher::publish);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.SpineTrajectoryMessageTopic, spineTrajectoryMessagePublisher::publish);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.HeadTrajectoryMessageTopic, headTrajectoryMessagePublisher::publish);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.NeckTrajectoryMessageTopic, neckTrajectoryMessagePublisher::publish);
   }

   private void processFootstepPlanningRequestPacket(FootstepPlanningRequestPacket packet)
   {
      if (verbose)
         LogTools.info("Received a planning request.");

      messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootPose, packet.getStartLeftFootPose());
      messager.submitMessage(FootstepPlannerMessagerAPI.RightFootPose, packet.getStartRightFootPose());
      messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootGoalPose, packet.getGoalLeftFootPose());
      messager.submitMessage(FootstepPlannerMessagerAPI.RightFootGoalPose, packet.getGoalRightFootPose());
      RobotSide initialSupportSide = RobotSide.fromByte(packet.getRequestedInitialStanceSide());
      int plannerRequestId = packet.getPlannerRequestId();

      double timeout = packet.getTimeout();
      double horizonLength = packet.getHorizonLength();

      messager.submitMessage(FootstepPlannerMessagerAPI.PerformAStarSearch, packet.getPerformAStarSearch());
      messager.submitMessage(FootstepPlannerMessagerAPI.PlanBodyPath, packet.getPlanBodyPath());
      messager.submitMessage(FootstepPlannerMessagerAPI.RequestedSwingPlannerType, SwingPlannerType.fromByte(packet.getRequestedSwingPlanner()));

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTimeout, timeout);
      messager.submitMessage(FootstepPlannerMessagerAPI.MaxIterations, packet.getMaxIterations());
      messager.submitMessage(FootstepPlannerMessagerAPI.InitialSupportSide, initialSupportSide);
      messager.submitMessage(FootstepPlannerMessagerAPI.SnapGoalSteps, packet.getSnapGoalSteps());
      messager.submitMessage(FootstepPlannerMessagerAPI.AbortIfGoalStepSnapFails, packet.getAbortIfGoalStepSnappingFails());

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerRequestId, plannerRequestId);

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerHorizonLength, horizonLength);
   }

   private void processBodyPathPlanMessage(BodyPathPlanMessage packet)
   {
      PlanarRegionsListMessage planarRegionsListMessage = packet.getPlanarRegionsList();
      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
      FootstepPlanningResult result = FootstepPlanningResult.fromByte(packet.getFootstepPlanningResult());
      List<? extends Pose3DReadOnly> bodyPath = packet.getBodyPath();

      messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionData, planarRegionsList);
      messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanningResultTopic, result);
      messager.submitMessage(FootstepPlannerMessagerAPI.BodyPathData, bodyPath);

      if (verbose)
         LogTools.info("Received a body path planning result from the toolbox.");
   }

   private void processFootstepPlanningOutputStatus(FootstepPlanningToolboxOutputStatus packet)
   {
      FootstepDataListMessage footstepDataListMessage = packet.getFootstepDataList();
      int plannerRequestId = packet.getPlanId();
      BodyPathPlanningResult bodyPathPlanningResult = BodyPathPlanningResult.fromByte(packet.getBodyPathPlanningResult());
      FootstepPlanningResult footstepPlanningResult = FootstepPlanningResult.fromByte(packet.getFootstepPlanningResult());
      List<? extends Pose3DReadOnly> bodyPath = packet.getBodyPath();
      Pose3D lowLevelGoal = packet.getGoalPose();

      if (plannerRequestId > currentPlanRequestId.get())
         messager.submitMessage(FootstepPlannerMessagerAPI.PlannerRequestId, plannerRequestId);

      ThreadTools.sleep(100);

      boolean broadcastFootstepPlan = footstepPlanningResult.terminalResult() || !footstepDataListMessage.getFootstepDataList().isEmpty();
      if (broadcastFootstepPlan)
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanResponse, footstepDataListMessage);         
      }
      
      messager.submitMessage(FootstepPlannerMessagerAPI.ReceivedPlanId, plannerRequestId);
      messager.submitMessage(FootstepPlannerMessagerAPI.BodyPathPlanningResultTopic, bodyPathPlanningResult);
      messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanningResultTopic, footstepPlanningResult);
      messager.submitMessage(FootstepPlannerMessagerAPI.BodyPathData, bodyPath);
      if (lowLevelGoal != null)
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.LowLevelGoalPosition, lowLevelGoal.getPosition());
         messager.submitMessage(FootstepPlannerMessagerAPI.LowLevelGoalOrientation, lowLevelGoal.getOrientation());
      }
      if (footstepPlanningResult == FootstepPlanningResult.EXCEPTION)
      {
         StringBuilder stackTrace = new StringBuilder();
         stackTrace.append(packet.getExceptionMessage()).append("\n");
         for (int i = 0; i < packet.getStacktrace().size(); i++)
         {
            stackTrace.append(packet.getStacktrace().get(i)).append("\n");
         }
         messager.submitMessage(FootstepPlannerMessagerAPI.PlannerExceptionStackTrace, stackTrace.toString());
      }
      else
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.PlannerExceptionStackTrace,
                                "No stack trace available, planner status wasn't " + FootstepPlanningResult.EXCEPTION + ", it was: " + footstepPlanningResult);
      }

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTimings, packet.getPlannerTimings());

      if (verbose)
         LogTools.info("Received a footstep planning result from planner module.");
   }

   private void processIncomingPlanarRegionMessage(PlanarRegionsListMessage packet)
   {
      if (acceptNewPlanarRegionsReference.get())
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionData, PlanarRegionMessageConverter.convertToPlanarRegionsList(packet));

         if (verbose)
            LogTools.info("Received updated planner regions.");
      }
   }

   private void processCapturabilityStatus(CapturabilityBasedStatus packet)
   {
      ConvexPolygon2D leftFootPolygon = new ConvexPolygon2D();
      ConvexPolygon2D rightFootPolygon = new ConvexPolygon2D();
      packet.getLeftFootSupportPolygon3d().forEach(leftFootPolygon::addVertex);
      packet.getRightFootSupportPolygon3d().forEach(rightFootPolygon::addVertex);
      leftFootPolygon.update();
      rightFootPolygon.update();

      messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootStartSupportPolygon, leftFootPolygon);
      messager.submitMessage(FootstepPlannerMessagerAPI.RightFootStartSupportPolygon, rightFootPolygon);
   }

   private void requestNewPlan()
   {
      if (!checkRequireds())
      {
         return;
      }

      if (verbose)
         LogTools.info("Told the toolbox to wake up.");

      VisibilityGraphsParametersReadOnly visibilityGraphsParameters = visibilityGraphParametersReference.get();
      if(visibilityGraphsParameters != null)
      {
         VisibilityGraphsParametersPacket visibilityGraphsParametersPacket = new VisibilityGraphsParametersPacket();
         FootstepPlannerMessageTools.copyParametersToPacket(visibilityGraphsParametersPacket, visibilityGraphsParameters);
         visibilityGraphsParametersPublisher.publish(visibilityGraphsParametersPacket);
      }

      FootstepPlannerParametersReadOnly footstepPlannerParameters = plannerParametersReference.get();
      if(footstepPlannerParameters != null)
      {
         FootstepPlannerParametersPacket plannerParametersPacket = new FootstepPlannerParametersPacket();
         FootstepPlannerMessageTools.copyParametersToPacket(plannerParametersPacket, footstepPlannerParameters);
         plannerParametersPublisher.publish(plannerParametersPacket);
      }

      SwingPlannerParametersReadOnly swingPlannerParameters = swingPlannerParametersReference.get();
      if (swingPlannerParameters != null)
      {
         swingPlannerParametersPublisher.publish(swingPlannerParameters.getAsPacket());
      }

      if (verbose)
         LogTools.info("Sent out some parameters");

      submitFootstepPlanningRequestPacket();
   }

   private boolean checkRequireds()
   {
      String errorMessage = "";

      if (leftFootPose.get() == null || rightFootPose.get() == null)
         errorMessage += "Need to set start poses.\n";
      if (goalLeftFootPose.get() == null || goalRightFootPose.get() == null)
         errorMessage += "Need to set goal poses.";

      if (!errorMessage.isEmpty())
      {
         LogTools.warn(errorMessage);
         return false;
      }

      return true;
   }

   private boolean checkPostProcessingRequireds()
   {
      String errorMessage = "";

      if (leftFootPose.get() == null || rightFootPose.get() == null)
         errorMessage += "Need to set foot poses.\n";
      if (postProcessingLeftFootSupportPolygonReference.get() == null || postProcessingRightFootSupportPolygonReference.get() == null)
         errorMessage += "Need to set foot polygons.\n";

      if (footstepPlanResponseReference.get() == null)
         errorMessage += "Need a footstep plan to post process.\n";
      if (planarRegionListReference.get() == null)
         errorMessage += "Need planar regions.\n";

      if (!errorMessage.isEmpty())
      {
         LogTools.warn(errorMessage);
         return false;
      }

      return true;
   }

   private void requestHaltPlanning()
   {
      if (verbose)
         LogTools.info("Sending out a sleep request.");
      FootstepPlannerActionMessage footstepPlannerActionMessage = new FootstepPlannerActionMessage();
      footstepPlannerActionMessage.setRequestedAction(FootstepPlannerRequestedAction.HALT.toByte());
      plannerActionPublisher.publish(footstepPlannerActionMessage);
   }

   private void submitFootstepPlanningRequestPacket()
   {
      FootstepPlanningRequestPacket packet = new FootstepPlanningRequestPacket();
      packet.getStartLeftFootPose().set(leftFootPose.get());
      packet.getStartRightFootPose().set(rightFootPose.get());
      packet.getGoalLeftFootPose().set(goalLeftFootPose.get());
      packet.getGoalRightFootPose().set(goalRightFootPose.get());

      if (plannerInitialSupportSideReference.get() != null)
         packet.setRequestedInitialStanceSide(plannerInitialSupportSideReference.get().toByte());
      if (plannerTimeoutReference.get() != null)
         packet.setTimeout(plannerTimeoutReference.get());

      packet.setPlanBodyPath(planBodyPath.get());
      packet.setPerformAStarSearch(performAStarSearch.get());
      packet.setRequestedSwingPlanner(requestedSwingPlanner.get().toByte());

      if (plannerRequestIdReference.get() != null)
         packet.setPlannerRequestId(plannerRequestIdReference.get());
      if (plannerHorizonLengthReference.get() != null)
         packet.setHorizonLength(plannerHorizonLengthReference.get());
      if (plannerPlanarRegionReference.get() != null)
         packet.getPlanarRegionsListMessage().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(plannerPlanarRegionReference.get()));
      packet.setAssumeFlatGround(assumeFlatGround.get());
      packet.setGoalDistanceProximity(goalDistanceProximity.get());
      packet.setGoalYawProximity(goalYawProximity.get());
      if (pathHeadingReference.get() != null)
         packet.setRequestedPathHeading(AngleTools.trimAngleMinusPiToPi(Math.toRadians(pathHeadingReference.get())));
      packet.setSnapGoalSteps(snapGoalSteps.get());
      packet.setAbortIfGoalStepSnappingFails(abortIfGoalStepSnapFails.get());
      packet.setMaxIterations(maxIterations.get());

      footstepPlanningRequestPublisher.publish(packet);
   }
}
