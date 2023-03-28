package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
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
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.log.LogTools;
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
   private final DRCRobotModel robotModel;
   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;
   private final ROS2PublisherMap publisherMap;

   private final AtomicReference<PlanarRegionsListMessage> planarRegionsListMessage = new AtomicReference<>();
   private final AtomicReference<WalkingStatusMessage> walkingStatusMessage = new AtomicReference<>();

   private Pose3D goalPose = new Pose3D(10.0, 0.0, 0.0, 0.0, 0.0, 0.0);;

   private FootstepPlannerRequest request;
   private ROS2Topic controllerFootstepDataTopic;

   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.createNamedThreadFactory("ActiveMappingRunner"));

   private FootstepPlanningModule footstepPlanner;
   private PlanarRegionsList planarRegionsInWorldFrame;

   public ActiveMappingLocomotionModule(DRCRobotModel robotModel)
   {
      this.controllerFootstepDataTopic = ControllerAPIDefinition.getTopic(FootstepDataListMessage.class, robotModel.getSimpleRobotName());
      this.robotModel = robotModel;
      ros2Node = ROS2Tools.createROS2Node(CommunicationMode.INTERPROCESS.getPubSubImplementation(), "active_mapping_locomotion");
      ros2Helper = new ROS2Helper(ros2Node);
      publisherMap = new ROS2PublisherMap(ros2Node);

      ros2Helper.subscribeViaCallback(ROS2Tools.PERSPECTIVE_RAPID_REGIONS, planarRegionsListMessage::set);
      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(WalkingStatusMessage.class, robotModel.getSimpleRobotName()), walkingStatusMessage::set);

      executorService.scheduleAtFixedRate(this::run, 0, 500, TimeUnit.MILLISECONDS);

      publisherMap.getOrCreatePublisher(controllerFootstepDataTopic);

      FootstepPlanningModule footstepPlanningModule = FootstepPlanningModuleLauncher.createModule(ros2Node, robotModel);
   }

   public void onPlanarRegionsReceived(PlanarRegionsListMessage message)
   {

      synchronized (planarRegionsListMessage)
      {
         planarRegionsListMessage.getAndSet(message);
         PlanarRegionsList planarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(message);
         planarRegionsInWorldFrame.clear();
         planarRegionsInWorldFrame.addPlanarRegionsList(planarRegions);

      }
   }

   public void run()
   {
      if (planarRegionsListMessage.get() == null)
         return;

      request = new FootstepPlannerRequest();
      request.setTimeout(3.5);
      Pose3D initialMidFootPose = new Pose3D(new Point3D(), new Quaternion());
      request.setStartFootPoses(robotModel.getFootstepPlannerParameters().getIdealFootstepWidth(), initialMidFootPose);
      request.setRequestedInitialStanceSide(RobotSide.LEFT);
      request.setPlanarRegionsList(planarRegionsInWorldFrame);
      request.setPlanBodyPath(false);
      request.setGoalFootPoses(robotModel.getFootstepPlannerParameters().getIdealFootstepWidth(), goalPose);

      FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlanner.getOutput().getFootstepPlan(), 1.3, 0.4);

      publisherMap.publish(controllerFootstepDataTopic, new FootstepDataListMessage());

      LogTools.info("------------------------ Run -----------------------------");
      LogTools.info("Total Planar Regions Received: {}", planarRegionsInWorldFrame.getNumberOfPlanarRegions());
      LogTools.info("Footstep Plan Length: {}", footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps());
      LogTools.info("Walking Status: {}\n", walkingStatusMessage.get().getWalkingStatus());
   }
}
