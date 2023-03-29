package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class ActiveMappingLocomotionModule
{
   private final AtomicReference<FramePlanarRegionsListMessage> planarRegionsListMessage = new AtomicReference<>(null);
   private final AtomicReference<WalkingStatusMessage> walkingStatusMessage = new AtomicReference<>();
   private final PlanarRegionsList planarRegionsInWorldFrame = new PlanarRegionsList();

   private final FootstepPlanningModule footstepPlanner;
   private final ROS2PublisherMap publisherMap;
   private final DRCRobotModel robotModel;
   private final ROS2Helper ros2Helper;
   private final ROS2Node ros2Node;

   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.createNamedThreadFactory(
         "ActiveMappingRunner"));
   private final ROS2Topic controllerFootstepDataTopic;

   private FootstepPlannerRequest request;

   private Pose3D goalPose = new Pose3D(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   ;

   public ActiveMappingLocomotionModule(DRCRobotModel robotModel)
   {
      this.controllerFootstepDataTopic = ControllerAPIDefinition.getTopic(FootstepDataListMessage.class, robotModel.getSimpleRobotName());
      this.robotModel = robotModel;
      ros2Node = ROS2Tools.createROS2Node(CommunicationMode.INTERPROCESS.getPubSubImplementation(), "active_mapping_locomotion");
      ros2Helper = new ROS2Helper(ros2Node);
      publisherMap = new ROS2PublisherMap(ros2Node);

      ros2Helper.subscribeViaCallback(ROS2Tools.PERSPECTIVE_RAPID_REGIONS_WITH_POSE, this::onPlanarRegionsReceived);
      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(WalkingStatusMessage.class, robotModel.getSimpleRobotName()), walkingStatusMessage::set);


      publisherMap.getOrCreatePublisher(controllerFootstepDataTopic);

      footstepPlanner = FootstepPlanningModuleLauncher.createModule(ros2Node, robotModel);

      executorService.scheduleAtFixedRate(this::update, 0, 500, TimeUnit.MILLISECONDS);
   }

   public void onPlanarRegionsReceived(FramePlanarRegionsListMessage message)
   {
      LogTools.info("Received Planar Regions");

      if (planarRegionsListMessage.get() == null)
      {
         LogTools.info("Setting Planar Regions Received");

         planarRegionsListMessage.set(message);

         FramePlanarRegionsList framePlanarRegions = PlanarRegionMessageConverter.convertToFramePlanarRegionsList(message);
         PlanarRegionsList planarRegions = framePlanarRegions.getPlanarRegionsList();
         planarRegionsInWorldFrame.clear();
         planarRegionsInWorldFrame.addPlanarRegionsList(planarRegions);

      }
   }

   public void update()
   {
      if (planarRegionsListMessage.get() == null)
      {
         LogTools.warn("No Planar Regions Available");
         return;
      }

      LogTools.info("Setting Plan Request: Regions: {}", planarRegionsInWorldFrame.getNumberOfPlanarRegions());

      request = new FootstepPlannerRequest();
      request.setTimeout(3.5);
      Pose3D initialMidFootPose = new Pose3D(new Point3D(), new Quaternion());
      request.setStartFootPoses(robotModel.getFootstepPlannerParameters().getIdealFootstepWidth(), initialMidFootPose);
      request.setRequestedInitialStanceSide(RobotSide.LEFT);
      request.setPlanarRegionsList(planarRegionsInWorldFrame);
      request.setPlanBodyPath(false);
      request.setGoalFootPoses(robotModel.getFootstepPlannerParameters().getIdealFootstepWidth(), goalPose);

      FootstepPlannerOutput plannerOutput = footstepPlanner.handleRequest(request);

      LogTools.info("Footstep Plan Result: {}", plannerOutput.getFootstepPlanningResult().validForExecution());
      //FootstepDataMessageConverter.createFootstepDataListFromPlan(plannerOutput.getFootstepPlan(), 1.3, 0.4);
      //
      //LogTools.info("------------------------ Run -----------------------------");
      //LogTools.info(String.format("Planar Regions: %d\t Plan Length: %d\t Walking Status: %s",
      //                            planarRegionsInWorldFrame.getNumberOfPlanarRegions(),
      //                            footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps(),
      //                            WalkingStatus.fromByte(walkingStatusMessage.get().getWalkingStatus())));

      //LogTools.info("Publishing Plan Result");
      //publisherMap.publish(controllerFootstepDataTopic, new FootstepDataListMessage());
   }
}
