package us.ihmc.quadrupedFootstepPlanning.ui;

import ihmc_common_msgs.msg.dds.TimeIntervalMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import toolbox_msgs.msg.dds.*;
import controller_msgs.msg.dds.RobotConfigurationData;
import quadruped_msgs.msg.dds.*;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.HumanoidControllerAPI;
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
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.*;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerCommunicationProperties;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.tools.PawStepPlannerMessageTools;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
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

   private final AtomicReference<PawStepPlannerParametersReadOnly> plannerParametersReference;
   private final AtomicReference<VisibilityGraphsParametersReadOnly> visibilityGraphParametersReference;
   private final AtomicReference<Point3D> plannerStartPositionReference;
   private final AtomicReference<Quaternion> plannerStartOrientationReference;
   private final AtomicReference<Point3D> plannerGoalPositionReference;
   private final AtomicReference<Quaternion> plannerGoalOrientationReference;
   private final AtomicReference<PawStepPlannerTargetType> plannerStartTargetTypeReference;
   private final AtomicReference<QuadrantDependentList<Point3D>> plannerStartFeetPositionsReference;
   private final AtomicReference<PlanarRegionsList> plannerPlanarRegionReference;
   private final AtomicReference<PawStepPlannerType> plannerTypeReference;
   private final AtomicReference<Double> plannerTimeoutReference;
   private final AtomicReference<Double> plannerBestEffortTimeoutReference;
   private final AtomicReference<RobotQuadrant> plannerInitialSupportQuadrantReference;
   private final AtomicReference<Integer> plannerRequestIdReference;
   private final AtomicReference<Double> plannerHorizonLengthReference;
   private final AtomicReference<Boolean> acceptNewPlanarRegionsReference;
   private final AtomicReference<Integer> currentPlanRequestId;
   private final AtomicReference<Boolean> assumeFlatGround;


   private ROS2PublisherBasics<ToolboxStateMessage> toolboxStatePublisher;
   private ROS2PublisherBasics<PawStepPlannerParametersPacket> plannerParametersPublisher;
   private ROS2PublisherBasics<VisibilityGraphsParametersPacket> visibilityGraphsParametersPublisher;
   private ROS2PublisherBasics<PawStepPlanningRequestPacket> pawPlanningRequestPublisher;
   private ROS2PublisherBasics<QuadrupedTimedStepListMessage> footstepDataListPublisher;
//   private ROS2PublisherBasics<ToolboxStateMessage> walkingPreviewToolboxStatePublisher;
//   private ROS2PublisherBasics<WalkingControllerPreviewInputMessage> walkingPreviewRequestPublisher;

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

      plannerParametersReference = messager.createInput(PawStepPlannerMessagerAPI.PlannerParametersTopic, null);
      visibilityGraphParametersReference = messager.createInput(PawStepPlannerMessagerAPI.VisibilityGraphsParametersTopic, null);
      plannerStartPositionReference = messager.createInput(PawStepPlannerMessagerAPI.StartPositionTopic);
      plannerStartOrientationReference = messager.createInput(PawStepPlannerMessagerAPI.StartOrientationTopic, new Quaternion());
      plannerStartTargetTypeReference = messager.createInput(PawStepPlannerMessagerAPI.StartTargetTypeTopic, PawStepPlannerTargetType.POSE_BETWEEN_FEET);
      plannerStartFeetPositionsReference = messager.createInput(PawStepPlannerMessagerAPI.StartFeetPositionTopic);
      plannerGoalPositionReference = messager.createInput(PawStepPlannerMessagerAPI.GoalPositionTopic);
      plannerGoalOrientationReference = messager.createInput(PawStepPlannerMessagerAPI.GoalOrientationTopic, new Quaternion());
      plannerPlanarRegionReference = messager.createInput(PawStepPlannerMessagerAPI.PlanarRegionDataTopic);
      plannerTypeReference = messager.createInput(PawStepPlannerMessagerAPI.PlannerTypeTopic, PawStepPlannerType.A_STAR);
      plannerTimeoutReference = messager.createInput(PawStepPlannerMessagerAPI.PlannerTimeoutTopic, 5.0);
      plannerBestEffortTimeoutReference = messager.createInput(PawStepPlannerMessagerAPI.PlannerBestEffortTimeoutTopic, 1.0);
      plannerInitialSupportQuadrantReference = messager.createInput(PawStepPlannerMessagerAPI.InitialSupportQuadrantTopic, RobotQuadrant.FRONT_LEFT);
      plannerRequestIdReference = messager.createInput(PawStepPlannerMessagerAPI.PlannerRequestIdTopic);
      plannerHorizonLengthReference = messager.createInput(PawStepPlannerMessagerAPI.PlannerHorizonLengthTopic);
      acceptNewPlanarRegionsReference = messager.createInput(PawStepPlannerMessagerAPI.AcceptNewPlanarRegionsTopic, true);
      currentPlanRequestId = messager.createInput(PawStepPlannerMessagerAPI.PlannerRequestIdTopic, 0);
      assumeFlatGround = messager.createInput(PawStepPlannerMessagerAPI.AssumeFlatGroundTopic, false);

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
      ros2Node.createSubscription(PawStepPlannerCommunicationProperties.inputTopic(robotName).withTypeName(PawStepPlanningRequestPacket.class), s -> processPawPlanningRequestPacket(s.takeNextData()));
      // we want to listen to the resulting body path plan from the toolbox
      ros2Node.createSubscription(PawStepPlannerCommunicationProperties.outputTopic(robotName).withTypeName(BodyPathPlanMessage.class), s -> processBodyPathPlanMessage(s.takeNextData()));
      ros2Node.createSubscription(PawStepPlannerCommunicationProperties.outputTopic(robotName).withTypeName(FootstepPlannerStatusMessage.class), s -> processFootstepPlannerStatus(s.takeNextData()));
      // we want to listen to the resulting footstep plan from the toolbox
      ros2Node.createSubscription(PawStepPlannerCommunicationProperties.outputTopic(robotName).withTypeName(PawStepPlanningToolboxOutputStatus.class),
                                  s -> processFootstepPlanningOutputStatus(s.takeNextData()));
      // we want to also listen to incoming REA planar region data.
      ros2Node.createSubscription(REACommunicationProperties.outputTopic.withTypeName(PlanarRegionsListMessage.class),
                                  s -> processIncomingPlanarRegionMessage(s.takeNextData()));

      /*
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepNodeDataListMessage.class,
                                           FootstepPlannerCommunicationProperties.outputTopic(robotName),
                                           s -> messager.submitMessage(FootstepPlannerMessagerAPI.NodeDataTopic, s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepPlannerOccupancyMapMessage.class,
                                           FootstepPlannerCommunicationProperties.outputTopic(robotName),
                                           s -> messager.submitMessage(FootstepPlannerMessagerAPI.OccupancyMapTopic, s.takeNextData()));
                                           */

      ros2Node.createSubscription(HumanoidControllerAPI.getOutputTopic(robotName).withTypeName(RobotConfigurationData.class),
                                  s -> messager.submitMessage(PawStepPlannerMessagerAPI.RobotConfigurationDataTopic, s.takeNextData()));

      ROS2Topic<?> controllerPreviewOutputTopic = ToolboxAPIs.WALKING_PREVIEW_TOOLBOX.withRobot(robotName).withOutput();
      ros2Node.createSubscription(controllerPreviewOutputTopic.withTypeName(WalkingControllerPreviewOutputMessage.class), s -> messager.submitMessage(
            PawStepPlannerMessagerAPI.WalkingPreviewOutput, s.takeNextData()));

      // publishers
      plannerParametersPublisher = ros2Node.createPublisher(PawStepPlannerCommunicationProperties.inputTopic(robotName).withTypeName(PawStepPlannerParametersPacket.class));
      visibilityGraphsParametersPublisher = ros2Node.createPublisher(PawStepPlannerCommunicationProperties.inputTopic(robotName).withTypeName(VisibilityGraphsParametersPacket.class));
      toolboxStatePublisher = ros2Node.createPublisher(PawStepPlannerCommunicationProperties.inputTopic(robotName).withTypeName(ToolboxStateMessage.class));
      pawPlanningRequestPublisher = ros2Node.createPublisher(PawStepPlannerCommunicationProperties.inputTopic(robotName).withTypeName(PawStepPlanningRequestPacket.class));
      footstepDataListPublisher = ros2Node.createPublisher(HumanoidControllerAPI.getInputTopic(robotName).withTypeName(QuadrupedTimedStepListMessage.class));

//      MessageTopicName controllerPreviewInputTopic = ROS2Tools.getTopicName(robotName, ROS2Tools.WALKING_PREVIEW_TOOLBOX, ROS2TopicQualifier.INPUT);
//      walkingPreviewToolboxStatePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, ToolboxStateMessage.class, controllerPreviewInputTopic);
//      walkingPreviewRequestPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, WalkingControllerPreviewInputMessage.class, controllerPreviewInputTopic);

      messager.addTopicListener(PawStepPlannerMessagerAPI.ComputePathTopic, request -> requestNewPlan());
      messager.addTopicListener(PawStepPlannerMessagerAPI.AbortPlanningTopic, request -> requestAbortPlanning());
      messager.addTopicListener(PawStepPlannerMessagerAPI.FootstepDataListTopic, footstepDataListPublisher::publish);
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

      messager.submitMessage(PawStepPlannerMessagerAPI.StartPositionTopic, startPosition);
      messager.submitMessage(PawStepPlannerMessagerAPI.GoalPositionTopic, goalPosition);

      messager.submitMessage(PawStepPlannerMessagerAPI.StartOrientationTopic, startOrientation);
      messager.submitMessage(PawStepPlannerMessagerAPI.GoalOrientationTopic, goalOrientation);

      messager.submitMessage(PawStepPlannerMessagerAPI.PlannerTypeTopic, plannerType);

      messager.submitMessage(PawStepPlannerMessagerAPI.PlannerTimeoutTopic, timeout);
      messager.submitMessage(PawStepPlannerMessagerAPI.InitialSupportQuadrantTopic, initialSupportSide);

      messager.submitMessage(PawStepPlannerMessagerAPI.PlannerRequestIdTopic, plannerRequestId);

      messager.submitMessage(PawStepPlannerMessagerAPI.PlannerHorizonLengthTopic, horizonLength);
   }

   private void processBodyPathPlanMessage(BodyPathPlanMessage packet)
   {
      PlanarRegionsListMessage planarRegionsListMessage = packet.getPlanarRegionsList();
      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
      PawStepPlanningResult result = PawStepPlanningResult.fromByte(packet.getFootstepPlanningResult());
      List<? extends Pose3DReadOnly> bodyPath = packet.getBodyPath();

      messager.submitMessage(PawStepPlannerMessagerAPI.PlanarRegionDataTopic, planarRegionsList);
      messager.submitMessage(PawStepPlannerMessagerAPI.PlanningResultTopic, result);
      messager.submitMessage(PawStepPlannerMessagerAPI.BodyPathDataTopic, bodyPath);

      if (verbose)
         PrintTools.info("Received a body path planning result from the toolbox.");
   }

   private void processFootstepPlannerStatus(FootstepPlannerStatusMessage packet)
   {
      messager.submitMessage(PawStepPlannerMessagerAPI.PlannerStatusTopic, PawStepPlannerStatus.fromByte(packet.getFootstepPlannerStatus()));
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
         messager.submitMessage(PawStepPlannerMessagerAPI.PlannerRequestIdTopic, plannerRequestId);

      ThreadTools.sleep(100);

      messager.submitMessage(PawStepPlannerMessagerAPI.FootstepPlanTopic, pawStepPlan);
      messager.submitMessage(PawStepPlannerMessagerAPI.ReceivedPlanIdTopic, plannerRequestId);
      messager.submitMessage(PawStepPlannerMessagerAPI.PlanningResultTopic, result);
      messager.submitMessage(PawStepPlannerMessagerAPI.PlannerTimeTakenTopic, packet.getTimeTaken());
      messager.submitMessage(PawStepPlannerMessagerAPI.BodyPathDataTopic, bodyPath);
      if (lowLevelGoal != null)
      {
         messager.submitMessage(PawStepPlannerMessagerAPI.LowLevelGoalPositionTopic, lowLevelGoal.getPosition());
         messager.submitMessage(PawStepPlannerMessagerAPI.LowLevelGoalOrientationTopic, lowLevelGoal.getOrientation());
      }

      if (verbose)
         PrintTools.info("Received a footstep planning result from the toolbox.");
   }

   private void processIncomingPlanarRegionMessage(PlanarRegionsListMessage packet)
   {
      if (acceptNewPlanarRegionsReference.get())
      {
         messager.submitMessage(PawStepPlannerMessagerAPI.PlanarRegionDataTopic, PlanarRegionMessageConverter.convertToPlanarRegionsList(packet));

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

   private void requestAbortPlanning()
   {
      if (verbose)
         PrintTools.info("Sending out a sleep request.");
      toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.SLEEP));
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

   private static PawStepPlan convertToFootstepPlan(QuadrupedTimedStepListMessage footstepDataListMessage)
   {
      PawStepPlan pawStepPlan = new PawStepPlan();

      for (QuadrupedTimedStepMessage timedStepMessage : footstepDataListMessage.getQuadrupedStepList())
      {
         QuadrupedStepMessage stepMessage = timedStepMessage.getQuadrupedStepMessage();
         TimeIntervalMessage timeInterval = timedStepMessage.getTimeInterval();
         FramePoint3D stepPosition = new FramePoint3D();
         stepPosition.set(stepMessage.getGoalPosition());
         pawStepPlan.addPawStep(RobotQuadrant.fromByte(stepMessage.getRobotQuadrant()), stepPosition, stepMessage.getGroundClearance(),
                                timeInterval.getStartTime(), timeInterval.getEndTime());
      }

      return pawStepPlan;
   }
}
