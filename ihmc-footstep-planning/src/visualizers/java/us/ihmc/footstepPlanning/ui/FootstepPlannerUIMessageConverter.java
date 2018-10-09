package us.ihmc.footstepPlanning.ui;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.RealtimeRos2Node;

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
public class FootstepPlannerUIMessageConverter
{
   // TODO make a local thing of planar regions

   private final RealtimeRos2Node ros2Node;

   private final JavaFXMessager messager;

   private final String robotName;

   private final AtomicReference<FootstepPlannerParameters> plannerParametersReference;
   private final AtomicReference<Point3D> plannerStartPositionReference;
   private final AtomicReference<Quaternion> plannerStartOrientationReference;
   private final AtomicReference<Point3D> plannerGoalPositionReference;
   private final AtomicReference<Quaternion> plannerGoalOrientationReference;
   private final AtomicReference<PlanarRegionsList> plannerPlanarRegionReference;
   private final AtomicReference<FootstepPlannerType> plannerTypeReference;
   private final AtomicReference<Double> plannerTimeoutReference;
   private final AtomicReference<RobotSide> plannerInitialSupportSideReference;
   private final AtomicReference<Integer> plannerSequenceIdReference;
   private final AtomicReference<Integer> plannerRequestIdReference;
   private final AtomicReference<Double> plannerHorizonLengthReference;

   private IHMCRealtimeROS2Publisher<FootstepPlannerParametersPacket> plannerParametersPublisher;
   private IHMCRealtimeROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher;

   public static FootstepPlannerUIMessageConverter createRemoteConverter(JavaFXMessager messager, String robotName)
   {
      return createConverter(messager, robotName, DomainFactory.PubSubImplementation.FAST_RTPS);
   }

   public static FootstepPlannerUIMessageConverter createIntraprocessConverter(JavaFXMessager messager, String robotName)
   {
      return createConverter(messager, robotName, DomainFactory.PubSubImplementation.INTRAPROCESS);
   }

   public static FootstepPlannerUIMessageConverter createConverter(JavaFXMessager messager, String robotName, DomainFactory.PubSubImplementation implementation)
   {
      RealtimeRos2Node ros2Node = ROS2Tools.createRealtimeRos2Node(implementation, "ihmc_footstep_planner_ui");
      return new FootstepPlannerUIMessageConverter(ros2Node, messager, robotName);
   }

   public FootstepPlannerUIMessageConverter(RealtimeRos2Node ros2Node, JavaFXMessager messager, String robotName)
   {
      this.messager = messager;
      this.robotName = robotName;
      this.ros2Node = ros2Node;

      plannerParametersReference = messager.createInput(FootstepPlannerUserInterfaceAPI.PlannerParametersTopic, null);
      plannerStartPositionReference = messager.createInput(FootstepPlannerUserInterfaceAPI.StartPositionTopic);
      plannerStartOrientationReference = messager.createInput(FootstepPlannerUserInterfaceAPI.StartOrientationTopic, new Quaternion());
      plannerGoalPositionReference = messager.createInput(FootstepPlannerUserInterfaceAPI.GoalPositionTopic);
      plannerGoalOrientationReference = messager.createInput(FootstepPlannerUserInterfaceAPI.GoalOrientationTopic, new Quaternion());
      plannerPlanarRegionReference = messager.createInput(FootstepPlannerUserInterfaceAPI.PlanarRegionDataTopic);
      plannerTypeReference = messager.createInput(FootstepPlannerUserInterfaceAPI.PlannerTypeTopic, FootstepPlannerType.A_STAR);
      plannerTimeoutReference = messager.createInput(FootstepPlannerUserInterfaceAPI.PlannerTimeoutTopic, 5.0);
      plannerInitialSupportSideReference = messager.createInput(FootstepPlannerUserInterfaceAPI.InitialSupportSideTopic, RobotSide.LEFT);
      plannerSequenceIdReference = messager.createInput(FootstepPlannerUserInterfaceAPI.SequenceIdTopic);
      plannerRequestIdReference = messager.createInput(FootstepPlannerUserInterfaceAPI.PlannerRequestIdTopic);
      plannerHorizonLengthReference = messager.createInput(FootstepPlannerUserInterfaceAPI.PlannerHorizonLengthTopic);

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
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlanningRequestPacket.class, getPlanningToolboxSubscriberNameGenerator(),
                                           s -> processFootstepPlanningRequestPacket(s.takeNextData()));
      // we want to listen to the resulting plan from the toolbox
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlanningToolboxOutputStatus.class, getPlanningToolboxPublisherNameGenerator(),
                                           s -> processFootstepPlanningOutputStatus(s.takeNextData()));

      // publishers
      plannerParametersPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPlannerParametersPacket.class, getPlanningToolboxSubscriberNameGenerator());
      footstepPlanningRequestPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPlanningRequestPacket.class, getPlanningToolboxSubscriberNameGenerator());

      messager.registerTopicListener(FootstepPlannerUserInterfaceAPI.ComputePathTopic, request -> requestNewPlan());
   }

   private void processFootstepPlanningRequestPacket(FootstepPlanningRequestPacket packet)
   {
      PlanarRegionsListMessage planarRegionsListMessage = packet.getPlanarRegionsListMessage();

      Point3D goalPosition = packet.getGoalPositionInWorld();
      Quaternion goalOrientation = packet.getGoalOrientationInWorld();
      Point3D startPosition = packet.getStanceFootPositionInWorld();
      Quaternion startOrientation = packet.getStanceFootOrientationInWorld();
      FootstepPlannerType plannerType = FootstepPlannerType.fromByte(packet.getRequestedFootstepPlannerType());
      RobotSide initialSupportSide = RobotSide.fromByte(packet.getInitialStanceRobotSide());
      int plannerRequestId = packet.getPlannerRequestId();
      int sequenceId = (int) packet.getSequenceId();

      double timeout = packet.getTimeout();
      double horizonLength = packet.getHorizonLength();

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlanarRegionDataTopic,
                             PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.StartPositionTopic, startPosition);
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.GoalPositionTopic, goalPosition);

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.StartOrientationTopic, startOrientation);
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.GoalOrientationTopic, goalOrientation);

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlannerTypeTopic, plannerType);

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlannerTimeoutTopic, timeout);
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.InitialSupportSideTopic, initialSupportSide);

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlannerRequestIdTopic, plannerRequestId);
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.SequenceIdTopic, sequenceId);

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlannerHorizonLengthTopic, horizonLength);
   }

   private void processFootstepPlanningOutputStatus(FootstepPlanningToolboxOutputStatus packet)
   {
      PlanarRegionsListMessage planarRegionsListMessage = packet.getPlanarRegionsList();
      FootstepDataListMessage footstepDataListMessage = packet.getFootstepDataList();
      int plannerRequestId = packet.getPlanId();
      int sequenceId = (int) packet.getSequenceId();
      FootstepPlanningResult result = FootstepPlanningResult.fromByte(packet.getFootstepPlanningResult());

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlanarRegionDataTopic,
                             PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.FootstepPlanTopic, convertToFootstepPlan(footstepDataListMessage));
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlannerRequestIdTopic, plannerRequestId);
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.SequenceIdTopic, sequenceId);
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlanningResultTopic, result);

      // Goal pose
      // TODO visualize body path
   }

   private void requestNewPlan()
   {
      submitFootstepPlannerParametersPacket();
      submitFootstepPlanningRequestPacket();
   }

   private void submitFootstepPlannerParametersPacket()
   {
      FootstepPlannerParametersPacket packet = new FootstepPlannerParametersPacket();
      FootstepPlannerParameters parameters = plannerParametersReference.get();

      if (parameters == null)
      {
         return;
      }

      packet.setIdealFootstepWidth(parameters.getIdealFootstepWidth());
      packet.setIdealFootstepLength(parameters.getIdealFootstepLength());
      packet.setWiggleInsideDelta(parameters.getWiggleInsideDelta());
      packet.setMaximumStepReach(parameters.getMaximumStepReach());
      packet.setMaximumStepYaw(parameters.getMaximumStepYaw());
      packet.setMinimumStepWidth(parameters.getMinimumStepWidth());
      packet.setMinimumStepLength(parameters.getMinimumStepLength());
      packet.setMinimumStepYaw(parameters.getMinimumStepYaw());
      packet.setMaximumStepXWhenForwardAndDown(parameters.getMaximumStepXWhenForwardAndDown());
      packet.setMaximumStepZWhenForwardAndDown(parameters.getMaximumStepZWhenForwardAndDown());
      packet.setMaximumStepZ(parameters.getMaximumStepZ());
      packet.setMinimumFootholdPercent(parameters.getMinimumFootholdPercent());
      packet.setMinimumSurfaceInclineRadians(parameters.getMinimumSurfaceInclineRadians());
      packet.setWiggleIntoConvexHullOfPlanarRegions(parameters.getWiggleIntoConvexHullOfPlanarRegions());
      packet.setRejectIfCannotFullyWiggleInside(parameters.getRejectIfCannotFullyWiggleInside());
      packet.setMaximumXyWiggleDistance(parameters.getMaximumXYWiggleDistance());
      packet.setMaximumYawWiggle(parameters.getMaximumYawWiggle());
      packet.setMaximumZPenetrationOnValleyRegions(parameters.getMaximumZPenetrationOnValleyRegions());
      packet.setMaximumStepWidth(parameters.getMaximumStepWidth());
      packet.setMinimumDistanceFromCliffBottoms(parameters.getMinimumDistanceFromCliffBottoms());
      packet.setCliffHeightToAvoid(parameters.getCliffHeightToAvoid());
      packet.setReturnBestEffortPlan(parameters.getReturnBestEffortPlan());
      packet.setMinimumStepsForBestEffortPlan(parameters.getMinimumStepsForBestEffortPlan());
      packet.setYawWeight(parameters.getYawWeight());
      packet.setCostPerStep(parameters.getCostPerStep());
      packet.setBodyGroundClearance(parameters.getBodyGroundClearance());
      packet.setMinXClearanceFromStance(parameters.getMinXClearanceFromStance());
      packet.setMinYClearanceFromStance(parameters.getMinYClearanceFromStance());

      plannerParametersPublisher.publish(packet);
   }

   private void submitFootstepPlanningRequestPacket()
   {
      FootstepPlanningRequestPacket packet = new FootstepPlanningRequestPacket();
      packet.getStanceFootPositionInWorld().set(plannerStartPositionReference.get());
      packet.getStanceFootOrientationInWorld().set(plannerStartOrientationReference.get());
      packet.getGoalPositionInWorld().set(plannerGoalPositionReference.get());
      packet.getGoalOrientationInWorld().set(plannerGoalOrientationReference.get());
      if (plannerInitialSupportSideReference.get() != null)
         packet.setInitialStanceRobotSide(plannerInitialSupportSideReference.get().toByte());
      if (plannerTimeoutReference.get() != null)
         packet.setTimeout(plannerTimeoutReference.get());
      if (plannerTypeReference.get() != null)
         packet.setRequestedFootstepPlannerType(plannerTypeReference.get().toByte());
      if (plannerSequenceIdReference.get() != null)
         packet.setSequenceId(plannerSequenceIdReference.get());
      if (plannerRequestIdReference.get() != null)
         packet.setPlannerRequestId(plannerRequestIdReference.get());
      if (plannerHorizonLengthReference.get() != null)
         packet.setHorizonLength(plannerHorizonLengthReference.get());
      if (plannerPlanarRegionReference.get() != null)
         packet.getPlanarRegionsListMessage()
               .set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(plannerPlanarRegionReference.get())); // TODO use a local copy

      footstepPlanningRequestPublisher.publish(packet);
   }

   private ROS2Tools.MessageTopicNameGenerator getPlanningToolboxSubscriberNameGenerator()
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT);
   }

   private ROS2Tools.MessageTopicNameGenerator getPlanningToolboxPublisherNameGenerator()
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2Tools.ROS2TopicQualifier.OUTPUT);
   }

   private static FootstepPlan convertToFootstepPlan(FootstepDataListMessage footstepDataListMessage)
   {
      FootstepPlan footstepPlan = new FootstepPlan();

      for (FootstepDataMessage footstepMessage : footstepDataListMessage.getFootstepDataList())
      {
         FramePose3D stepPose = new FramePose3D();
         stepPose.setPosition(footstepMessage.getLocation());
         stepPose.setOrientation(footstepMessage.getOrientation());
         footstepPlan.addFootstep(RobotSide.fromByte(footstepMessage.getRobotSide()), stepPose);
      }

      return footstepPlan;
   }
}
