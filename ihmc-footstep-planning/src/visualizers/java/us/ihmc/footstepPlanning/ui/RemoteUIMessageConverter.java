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
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.footstepPlanning.communication.FootstepPlannerSharedMemoryAPI;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
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
public class RemoteUIMessageConverter
{
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

   public static RemoteUIMessageConverter createRemoteConverter(JavaFXMessager messager, String robotName)
   {
      return createConverter(messager, robotName, DomainFactory.PubSubImplementation.FAST_RTPS);
   }

   public static RemoteUIMessageConverter createIntraprocessConverter(JavaFXMessager messager, String robotName)
   {
      return createConverter(messager, robotName, DomainFactory.PubSubImplementation.INTRAPROCESS);
   }

   public static RemoteUIMessageConverter createConverter(JavaFXMessager messager, String robotName, DomainFactory.PubSubImplementation implementation)
   {
      RealtimeRos2Node ros2Node = ROS2Tools.createRealtimeRos2Node(implementation, "ihmc_footstep_planner_ui");
      return new RemoteUIMessageConverter(ros2Node, messager, robotName);
   }

   public RemoteUIMessageConverter(RealtimeRos2Node ros2Node, JavaFXMessager messager, String robotName)
   {
      this.messager = messager;
      this.robotName = robotName;
      this.ros2Node = ros2Node;

      plannerParametersReference = messager.createInput(FootstepPlannerSharedMemoryAPI.PlannerParametersTopic, null);
      plannerStartPositionReference = messager.createInput(FootstepPlannerSharedMemoryAPI.StartPositionTopic);
      plannerStartOrientationReference = messager.createInput(FootstepPlannerSharedMemoryAPI.StartOrientationTopic, new Quaternion());
      plannerGoalPositionReference = messager.createInput(FootstepPlannerSharedMemoryAPI.GoalPositionTopic);
      plannerGoalOrientationReference = messager.createInput(FootstepPlannerSharedMemoryAPI.GoalOrientationTopic, new Quaternion());
      plannerPlanarRegionReference = messager.createInput(FootstepPlannerSharedMemoryAPI.PlanarRegionDataTopic);
      plannerTypeReference = messager.createInput(FootstepPlannerSharedMemoryAPI.PlannerTypeTopic, FootstepPlannerType.A_STAR);
      plannerTimeoutReference = messager.createInput(FootstepPlannerSharedMemoryAPI.PlannerTimeoutTopic, 5.0);
      plannerInitialSupportSideReference = messager.createInput(FootstepPlannerSharedMemoryAPI.InitialSupportSideTopic, RobotSide.LEFT);
      plannerSequenceIdReference = messager.createInput(FootstepPlannerSharedMemoryAPI.SequenceIdTopic);
      plannerRequestIdReference = messager.createInput(FootstepPlannerSharedMemoryAPI.PlannerRequestIdTopic);
      plannerHorizonLengthReference = messager.createInput(FootstepPlannerSharedMemoryAPI.PlannerHorizonLengthTopic);

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
      // we want to listen to the resulting plan from the toolbox
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlanningToolboxOutputStatus.class,
                                           FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName),
                                           s -> processFootstepPlanningOutputStatus(s.takeNextData()));
      // we want to also listen to incoming REA planar region data.
      ROS2Tools.createCallbackSubscription(ros2Node, PlanarRegionsListMessage.class, REACommunicationProperties.publisherTopicNameGenerator,
                                           s -> processIncomingPlanarRegionMessage(s.takeNextData()));

      // publishers
      plannerParametersPublisher = ROS2Tools
            .createPublisher(ros2Node, FootstepPlannerParametersPacket.class, FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName));
      footstepPlanningRequestPublisher = ROS2Tools
            .createPublisher(ros2Node, FootstepPlanningRequestPacket.class, FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName));

      messager.registerTopicListener(FootstepPlannerSharedMemoryAPI.ComputePathTopic, request -> requestNewPlan());
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

      messager.submitMessage(FootstepPlannerSharedMemoryAPI.PlanarRegionDataTopic,
                             PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.StartPositionTopic, startPosition);
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.GoalPositionTopic, goalPosition);

      messager.submitMessage(FootstepPlannerSharedMemoryAPI.StartOrientationTopic, startOrientation);
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.GoalOrientationTopic, goalOrientation);

      messager.submitMessage(FootstepPlannerSharedMemoryAPI.PlannerTypeTopic, plannerType);

      messager.submitMessage(FootstepPlannerSharedMemoryAPI.PlannerTimeoutTopic, timeout);
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.InitialSupportSideTopic, initialSupportSide);

      messager.submitMessage(FootstepPlannerSharedMemoryAPI.PlannerRequestIdTopic, plannerRequestId);
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.SequenceIdTopic, sequenceId);

      messager.submitMessage(FootstepPlannerSharedMemoryAPI.PlannerHorizonLengthTopic, horizonLength);
   }

   private void processFootstepPlanningOutputStatus(FootstepPlanningToolboxOutputStatus packet)
   {
      PlanarRegionsListMessage planarRegionsListMessage = packet.getPlanarRegionsList();
      FootstepDataListMessage footstepDataListMessage = packet.getFootstepDataList();
      int plannerRequestId = packet.getPlanId();
      int sequenceId = (int) packet.getSequenceId();
      FootstepPlanningResult result = FootstepPlanningResult.fromByte(packet.getFootstepPlanningResult());

      messager.submitMessage(FootstepPlannerSharedMemoryAPI.PlanarRegionDataTopic,
                             PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.FootstepPlanTopic, convertToFootstepPlan(footstepDataListMessage));
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.PlannerRequestIdTopic, plannerRequestId);
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.SequenceIdTopic, sequenceId);
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.PlanningResultTopic, result);

      // Goal pose
      // TODO visualize body path
   }

   private void processIncomingPlanarRegionMessage(PlanarRegionsListMessage packet)
   {
      messager.submitMessage(FootstepPlannerSharedMemoryAPI.PlanarRegionDataTopic, PlanarRegionMessageConverter.convertToPlanarRegionsList(packet));
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
               .set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(plannerPlanarRegionReference.get()));

      footstepPlanningRequestPublisher.publish(packet);
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
