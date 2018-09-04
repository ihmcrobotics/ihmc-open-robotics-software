package us.ihmc.footstepPlanning.ui;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.RealtimeRos2Node;

import java.util.concurrent.atomic.AtomicReference;

public class FootstepPlannerUIRosNode
{
   // TODO make a local thing of planar regions
   private final RealtimeRos2Node ros2Node = ROS2Tools.createRealtimeRos2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ihmc_footstep_planner_ui");

   private final FootstepPlannerUI ui;
   private final JavaFXMessager messager;

   private final String robotName;

   private final AtomicReference<FootstepPlannerParameters> plannerParametersReference;
   private final AtomicReference<Point3D> plannerStartPositionReference;
   private final AtomicReference<Double> plannerStartOrientationReference;
   private final AtomicReference<Point3D> plannerGoalPositionReference;
   private final AtomicReference<Double> plannerGoalOrientationReference;
   private final AtomicReference<PlanarRegionsList> plannerPlanarRegionReference;
   private final AtomicReference<FootstepPlannerType> plannerTypeReference;

   private IHMCRealtimeROS2Publisher<FootstepPlannerParametersPacket> plannerParametersPublisher;
   private IHMCRealtimeROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher;

   public FootstepPlannerUIRosNode(String robotName)
   {
      this.robotName = robotName;

      FootstepPlannerUILauncher launcher = new FootstepPlannerUILauncher();
      ui = launcher.getUI();
      messager = ui.getMessager();

      plannerParametersReference = messager.createInput(FootstepPlannerUserInterfaceAPI.PlannerParametersTopic, null);
      plannerStartPositionReference = messager.createInput(FootstepPlannerUserInterfaceAPI.StartPositionTopic);
      plannerStartOrientationReference= messager.createInput(FootstepPlannerUserInterfaceAPI.StartOrientationTopic);
      plannerGoalPositionReference = messager.createInput(FootstepPlannerUserInterfaceAPI.GoalPositionTopic);
      plannerGoalOrientationReference = messager.createInput(FootstepPlannerUserInterfaceAPI.GoalOrientationTopic);
      plannerPlanarRegionReference = messager.createInput(FootstepPlannerUserInterfaceAPI.PlanarRegionDataTopic);
      plannerTypeReference = messager.createInput(FootstepPlannerUserInterfaceAPI.PlannerTypeTopic);

      registerPubSubs(ros2Node);
      ros2Node.spin();
   }

   public void destroy()
   {
      ui.stop();
      ros2Node.stopSpinning();
   }

   private void registerPubSubs(RealtimeRos2Node ros2Node)
   {
      registerSubscribers(ros2Node);
      registerPublishers(ros2Node);
   }

   private void registerSubscribers(RealtimeRos2Node ros2Node)
   {
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlanningRequestPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> processFootstepPlanningRequestPacket(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlanningToolboxOutputStatus.class, getSubscriberTopicNameGenerator(),
                                           s -> processFootstepPlanningOutputStatus(s.takeNextData()));
      // TODO visualize node status

   }

   private void registerPublishers(RealtimeRos2Node ros2Node)
   {
      plannerParametersPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPlannerParametersPacket.class, ROS2Tools
            .getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2Tools.ROS2TopicQualifier.OUTPUT));
      footstepPlanningRequestPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPlanningRequestPacket.class, ROS2Tools
            .getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2Tools.ROS2TopicQualifier.OUTPUT));

      messager.registerTopicListener(FootstepPlannerUserInterfaceAPI.ComputePathTopic, request -> requestNewPlan());
   }

   private void processFootstepPlanningRequestPacket(FootstepPlanningRequestPacket packet)
   {
      PlanarRegionsListMessage planarRegionsListMessage = packet.getPlanarRegionsListMessage();

      Point3D goalPosition = packet.getGoalPositionInWorld();
      Point3D startPosition = packet.getStanceFootPositionInWorld();
      FootstepPlannerType plannerType = FootstepPlannerType.fromByte(packet.getRequestedFootstepPlannerType());

      double timeout = packet.getTimeout();

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlanarRegionDataTopic,
                             PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.StartPositionTopic, startPosition);
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.GoalPositionTopic, goalPosition);

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlannerTypeTopic, plannerType);

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlannerTimeoutTopic, timeout);
      // TODO add horizon length
   }

   private void processFootstepPlanningOutputStatus(FootstepPlanningToolboxOutputStatus packet)
   {
      PlanarRegionsListMessage planarRegionsListMessage = packet.getPlanarRegionsList();
      FootstepDataListMessage footstepDataListMessage = packet.getFootstepDataList();

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlanarRegionDataTopic,
                             PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.FootstepPlanTopic, convertToFootstepPlan(footstepDataListMessage));
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
      packet.setWiggleIntoConvexHullOfPlanarRegions(parameters.getWiggleIntoConvexHullOfPlanarRegions());
      packet.setRejectIfCannotFullyWiggleInside(parameters.getRejectIfCannotFullyWiggleInside());
      packet.setMaximumXyWiggleDistance(parameters.getMaximumXYWiggleDistance());
      packet.setMaximumYawWiggle(parameters.getMaximumYawWiggle());
      packet.setMaximumZPenetrationOnValleyRegions(parameters.getMaximumZPenetrationOnValleyRegions());
      packet.setMaximumStepWidth(parameters.getMaximumStepWidth());
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
      packet.getStanceFootOrientationInWorld().setYawPitchRoll(plannerStartOrientationReference.get(), 0.0, 0.0); // TODO add pitch roll
      packet.getGoalPositionInWorld().set(plannerGoalPositionReference.get());
      packet.getGoalOrientationInWorld().setYawPitchRoll(plannerGoalOrientationReference.get(), 0.0, 0.0); // TODO add pitch roll
      // TODO add robot side
      packet.setRequestedFootstepPlannerType(plannerTypeReference.get().toByte());
      packet.getPlanarRegionsListMessage().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(plannerPlanarRegionReference.get())); // TODO use a local copy

      footstepPlanningRequestPublisher.publish(packet);
   }

   private ROS2Tools.MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return getSubscriberTopicNameGenerator(robotName);
   }

   private static ROS2Tools.MessageTopicNameGenerator getSubscriberTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT);
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
