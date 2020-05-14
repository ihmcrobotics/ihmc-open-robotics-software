package us.ihmc.footstepPlanning.ui;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.footstepPlanning.BodyPathPlanningResult;
import us.ihmc.footstepPlanning.FootstepPlanHeading;
import us.ihmc.footstepPlanning.FootstepPlannerRequestedAction;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlannerOccupancyMap;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersReadOnly;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.RealtimeRos2Node;

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

   private final AtomicReference<FootstepPlannerParametersReadOnly> plannerParametersReference;
   private final AtomicReference<FootstepPostProcessingParametersReadOnly> postProcessingParametersReference;
   private final AtomicReference<VisibilityGraphsParametersReadOnly> visibilityGraphParametersReference;
   private final AtomicReference<Pose3DReadOnly> leftFootPose;
   private final AtomicReference<Pose3DReadOnly> rightFootPose;
   private final AtomicReference<Pose3DReadOnly> goalLeftFootPose;
   private final AtomicReference<Pose3DReadOnly> goalRightFootPose;
   private final AtomicReference<Boolean> snapGoalSteps;
   private final AtomicReference<Boolean> abortIfGoalStepSnapFails;
   private final AtomicReference<PlanarRegionsList> plannerPlanarRegionReference;
   private final AtomicReference<Boolean> planBodyPath;
   private final AtomicReference<Boolean> performAStarSearch;
   private final AtomicReference<Double> plannerTimeoutReference;
   private final AtomicReference<Integer> maxIterations;
   private final AtomicReference<RobotSide> plannerInitialSupportSideReference;
   private final AtomicReference<FootstepPlanHeading> pathHeadingReference;
   private final AtomicReference<Integer> plannerRequestIdReference;
   private final AtomicReference<Double> plannerHorizonLengthReference;
   private final AtomicReference<Boolean> acceptNewPlanarRegionsReference;
   private final AtomicReference<Integer> currentPlanRequestId;
   private final AtomicReference<Boolean> assumeFlatGround;
   private final AtomicReference<Boolean> ignorePartialFootholds;
   private final AtomicReference<Boolean> autoPostProcess;
   private final AtomicReference<Double> goalDistanceProximity;
   private final AtomicReference<Double> goalYawProximity;

   private final AtomicReference<ConvexPolygon2D> postProcessingLeftFootSupportPolygonReference;
   private final AtomicReference<ConvexPolygon2D> postProcessingRightFootSupportPolygonReference;
   private final AtomicReference<FootstepDataListMessage> footstepPlanResponseReference;
   private final AtomicReference<PlanarRegionsList> planarRegionListReference;

   private IHMCRealtimeROS2Publisher<FootstepPlannerActionMessage> plannerActionPublisher;
   private IHMCRealtimeROS2Publisher<FootstepPlannerParametersPacket> plannerParametersPublisher;
   private IHMCRealtimeROS2Publisher<FootstepPostProcessingParametersPacket> postProcessingParametersPublisher;
   private IHMCRealtimeROS2Publisher<VisibilityGraphsParametersPacket> visibilityGraphsParametersPublisher;
   private IHMCRealtimeROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher;
   private IHMCRealtimeROS2Publisher<FootstepPostProcessingPacket> footstepPostProcessingRequestPublisher;
   private IHMCRealtimeROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private IHMCRealtimeROS2Publisher<GoHomeMessage> goHomePublisher;
   private IHMCRealtimeROS2Publisher<ToolboxStateMessage> walkingPreviewToolboxStatePublisher;
   private IHMCRealtimeROS2Publisher<WalkingControllerPreviewInputMessage> walkingPreviewRequestPublisher;

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

      plannerParametersReference = messager.createInput(FootstepPlannerMessagerAPI.PlannerParameters, null);
      postProcessingParametersReference = messager.createInput(FootstepPlannerMessagerAPI.PostProcessingParametersTopic, null);
      visibilityGraphParametersReference = messager.createInput(FootstepPlannerMessagerAPI.VisibilityGraphsParameters, null);
      leftFootPose = messager.createInput(FootstepPlannerMessagerAPI.LeftFootPose);
      rightFootPose = messager.createInput(FootstepPlannerMessagerAPI.RightFootPose);
      goalLeftFootPose = messager.createInput(FootstepPlannerMessagerAPI.LeftFootGoalPose);
      goalRightFootPose = messager.createInput(FootstepPlannerMessagerAPI.RightFootGoalPose);
      snapGoalSteps = messager.createInput(FootstepPlannerMessagerAPI.SnapGoalSteps);
      abortIfGoalStepSnapFails = messager.createInput(FootstepPlannerMessagerAPI.AbortIfGoalStepSnapFails);
      plannerPlanarRegionReference = messager.createInput(FootstepPlannerMessagerAPI.PlanarRegionData);
      planBodyPath = messager.createInput(FootstepPlannerMessagerAPI.PlanBodyPath, false);
      performAStarSearch = messager.createInput(FootstepPlannerMessagerAPI.PerformAStarSearch, true);
      plannerTimeoutReference = messager.createInput(FootstepPlannerMessagerAPI.PlannerTimeout, 5.0);
      maxIterations = messager.createInput(FootstepPlannerMessagerAPI.MaxIterations, -1);
      plannerInitialSupportSideReference = messager.createInput(FootstepPlannerMessagerAPI.InitialSupportSide, RobotSide.LEFT);
      pathHeadingReference = messager.createInput(FootstepPlannerMessagerAPI.RequestedFootstepPlanHeading, FootstepPlanHeading.FORWARD);
      plannerRequestIdReference = messager.createInput(FootstepPlannerMessagerAPI.PlannerRequestId);
      plannerHorizonLengthReference = messager.createInput(FootstepPlannerMessagerAPI.PlannerHorizonLength);
      acceptNewPlanarRegionsReference = messager.createInput(FootstepPlannerMessagerAPI.AcceptNewPlanarRegions, true);
      currentPlanRequestId = messager.createInput(FootstepPlannerMessagerAPI.PlannerRequestId, 0);
      assumeFlatGround = messager.createInput(FootstepPlannerMessagerAPI.AssumeFlatGround, false);
      ignorePartialFootholds = messager.createInput(FootstepPlannerMessagerAPI.IgnorePartialFootholds, false);
      autoPostProcess = messager.createInput(FootstepPlannerMessagerAPI.AutoPostProcess, false);
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

   private void registerPubSubs(RealtimeRos2Node ros2Node)
   {
      /* subscribers */
      // we want to listen to the incoming request to the planning toolbox
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlanningRequestPacket.class,
                                           FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName),
                                           s -> processFootstepPlanningRequestPacket(s.takeNextData()));
      // we want to listen to the resulting body path plan from the toolbox
      ROS2Tools.createCallbackSubscription(ros2Node, BodyPathPlanMessage.class, FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName),
                                           s -> processBodyPathPlanMessage(s.takeNextData()));
      // we want to listen to the resulting footstep plan from the toolbox
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlanningToolboxOutputStatus.class,
                                           FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName),
                                           s -> processFootstepPlanningOutputStatus(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlannerOccupancyMapMessage.class,
                                           FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName),
                                           s -> messager.submitMessage(FootstepPlannerMessagerAPI.OccupancyMap, new PlannerOccupancyMap(s.takeNextData())));
      // we want to list to the footstep plan post processing result from the toolbox
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPostProcessingPacket.class,
                                           getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_POSTPROCESSING_TOOLBOX, ROS2TopicQualifier.OUTPUT),
                                           s -> processFootstepPostProcessingResult(s.takeNextData()));
      // we want to also listen to incoming REA planar region data.
      ROS2Tools.createCallbackSubscription(ros2Node, PlanarRegionsListMessage.class, REACommunicationProperties.publisherTopicNameGenerator,
                                           s -> processIncomingPlanarRegionMessage(s.takeNextData()));

      // things from the controller
      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                           s -> messager.submitMessage(FootstepPlannerMessagerAPI.RobotConfigurationData, s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, CapturabilityBasedStatus.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                           s -> processCapturabilityStatus(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepStatusMessage.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                           s ->
                                           {
                                              messager.submitMessage(FootstepPlannerMessagerAPI.FootstepStatusMessage, s.takeNextData());
                                              System.out.println("fdsjklsdf");
                                           });

      MessageTopicNameGenerator controllerPreviewOutputTopicNameGenerator = getTopicNameGenerator(robotName, ROS2Tools.WALKING_PREVIEW_TOOLBOX, ROS2TopicQualifier.OUTPUT);
      ROS2Tools.createCallbackSubscription(ros2Node, WalkingControllerPreviewOutputMessage.class, controllerPreviewOutputTopicNameGenerator, s -> messager.submitMessage(FootstepPlannerMessagerAPI.WalkingPreviewOutput, s.takeNextData()));

      // publishers
      plannerParametersPublisher = ROS2Tools.createPublisher(ros2Node,
                                                             FootstepPlannerParametersPacket.class,
                                                             FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName));
      visibilityGraphsParametersPublisher = ROS2Tools.createPublisher(ros2Node,
                                                                      VisibilityGraphsParametersPacket.class,
                                                                      FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName));
      plannerActionPublisher = ROS2Tools.createPublisher(ros2Node,
                                                         FootstepPlannerActionMessage.class,
                                                         FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName));
      postProcessingParametersPublisher = ROS2Tools.createPublisher(ros2Node,
                                                                    FootstepPostProcessingParametersPacket.class,
                                                                    getTopicNameGenerator(robotName,
                                                                                          ROS2Tools.FOOTSTEP_POSTPROCESSING_TOOLBOX,
                                                                                          ROS2TopicQualifier.INPUT));
      footstepPlanningRequestPublisher = ROS2Tools
            .createPublisher(ros2Node, FootstepPlanningRequestPacket.class, FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName));
      footstepPostProcessingRequestPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPostProcessingPacket.class,
                                                                    getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_POSTPROCESSING_TOOLBOX, ROS2TopicQualifier.INPUT));
      footstepDataListPublisher = ROS2Tools.createPublisher(ros2Node, FootstepDataListMessage.class, ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName));
      goHomePublisher = ROS2Tools.createPublisher(ros2Node, GoHomeMessage.class, ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName));

      MessageTopicNameGenerator controllerPreviewInputTopicNameGenerator = getTopicNameGenerator(robotName, ROS2Tools.WALKING_PREVIEW_TOOLBOX, ROS2TopicQualifier.INPUT);
      walkingPreviewToolboxStatePublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, controllerPreviewInputTopicNameGenerator);
      walkingPreviewRequestPublisher = ROS2Tools.createPublisher(ros2Node, WalkingControllerPreviewInputMessage.class, controllerPreviewInputTopicNameGenerator);

      messager.registerTopicListener(FootstepPlannerMessagerAPI.ComputePath, request -> requestNewPlan());
      messager.registerTopicListener(FootstepPlannerMessagerAPI.PostProcessPlan, request -> requestPostProcessing());
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
            .createPublisher(ros2Node, BipedalSupportPlanarRegionParametersMessage.class,
                             getTopicNameGenerator(robotName, ROS2Tools.BIPED_SUPPORT_REGION_PUBLISHER, ROS2TopicQualifier.INPUT));
      messager.registerTopicListener(FootstepPlannerMessagerAPI.BipedalSupportRegionsParameters, supportRegionsParametersPublisher::publish);
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

      messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanResponse, footstepDataListMessage);
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
         LogTools.info("Received a footstep planning result from the toolbox.");

      if (autoPostProcess.get())
         requestPostProcessing();
   }

   private void processFootstepPostProcessingResult(FootstepPostProcessingPacket packet)
   {
      FootstepDataListMessage footstepDataListMessage = packet.getFootstepDataList();

      ThreadTools.sleep(100);

      messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanResponse, footstepDataListMessage);

      if (verbose)
         LogTools.info("Received a footstep post processing result from the toolbox.");
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
      packet.getLeftFootSupportPolygon2d().forEach(leftFootPolygon::addVertex);
      packet.getRightFootSupportPolygon2d().forEach(rightFootPolygon::addVertex);
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

      FootstepPlannerParametersReadOnly footstepPlannerParameters = plannerParametersReference.get();
      if(footstepPlannerParameters != null)
      {
         FootstepPlannerParametersPacket plannerParametersPacket = new FootstepPlannerParametersPacket();
         FootstepPlannerMessageTools.copyParametersToPacket(plannerParametersPacket, footstepPlannerParameters);
         plannerParametersPublisher.publish(plannerParametersPacket);
      }

      FootstepPostProcessingParametersReadOnly postProcessingParameters = postProcessingParametersReference.get();
      if (postProcessingParameters != null)
         postProcessingParametersPublisher.publish(postProcessingParameters.getAsPacket());

      VisibilityGraphsParametersReadOnly visibilityGraphsParameters = visibilityGraphParametersReference.get();
      if(visibilityGraphsParameters != null)
      {
         VisibilityGraphsParametersPacket visibilityGraphsParametersPacket = new VisibilityGraphsParametersPacket();
         FootstepPlannerMessageTools.copyParametersToPacket(visibilityGraphsParametersPacket, visibilityGraphsParameters);
         visibilityGraphsParametersPublisher.publish(visibilityGraphsParametersPacket);
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

   private void requestPostProcessing()
   {
      if (!checkPostProcessingRequireds())
         return;

      FootstepPostProcessingParametersReadOnly postProcessingParameters = postProcessingParametersReference.get();
      if (postProcessingParameters != null)
         postProcessingParametersPublisher.publish(postProcessingParameters.getAsPacket());

      submitFootstepPostProcessingRequestPacket();
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
         packet.setRequestedPathHeading(pathHeadingReference.get().toByte());
      packet.setSnapGoalSteps(snapGoalSteps.get());
      packet.setAbortIfGoalStepSnappingFails(abortIfGoalStepSnapFails.get());
      packet.setMaxIterations(maxIterations.get());

      footstepPlanningRequestPublisher.publish(packet);
   }

   private void submitFootstepPostProcessingRequestPacket()
   {
      FootstepPostProcessingPacket packet = new FootstepPostProcessingPacket();

      packet.getLeftFootPositionInWorld().set(leftFootPose.get().getPosition());
      packet.getLeftFootOrientationInWorld().set(leftFootPose.get().getOrientation());
      postProcessingLeftFootSupportPolygonReference.get().getVertexBufferView().forEach(point -> packet.getLeftFootContactPoints2d().add().set(point));
      packet.getRightFootPositionInWorld().set(rightFootPose.get().getPosition());
      packet.getRightFootOrientationInWorld().set(rightFootPose.get().getOrientation());
      postProcessingRightFootSupportPolygonReference.get().getVertexBufferView().forEach(point -> packet.getRightFootContactPoints2d().add().set(point));

      packet.getFootstepDataList().set(footstepPlanResponseReference.get());
      packet.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionListReference.get()));

      footstepPostProcessingRequestPublisher.publish(packet);
   }
}
