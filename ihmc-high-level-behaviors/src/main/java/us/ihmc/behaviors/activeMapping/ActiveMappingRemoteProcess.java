package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StepGeneratorAPIDefinition;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class ActiveMappingRemoteProcess
{
   public enum ActiveMappingMode
   {
      EXECUTE_AND_PAUSE, CONTINUOUS_MAPPING_STRAIGHT, CONTINUOUS_MAPPING_COVERAGE, CONTINUOUS_MAPPING_SEARCH
   }

   public ActiveMappingMode activeMappingMode = ActiveMappingMode.EXECUTE_AND_PAUSE;

   private final static long STATISTICS_COLLECTION_PERIOD_MS = 500;

   private final AtomicReference<FramePlanarRegionsListMessage> planarRegionsListMessage = new AtomicReference<>(null);
   private final AtomicReference<WalkingStatusMessage> walkingStatusMessage = new AtomicReference<>();

   private final ROS2PublisherMap publisherMap;
   private final ROS2Helper ros2Helper;
   private final HumanoidReferenceFrames referenceFrames;

   private ActiveMappingModule activeMappingModule;
   private IHMCROS2Publisher<PlanarRegionsListMessage> controllerRegionsPublisher;
   private IHMCROS2Publisher<PlanarRegionsListMessage> slamOutputRegionsPublisher;

   private final AtomicReference<FramePlanarRegionsListMessage> latestIncomingRegions = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> latestPlanarRegionsForPublishing = new AtomicReference<>(null);

   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.createNamedThreadFactory(
         "ActiveMappingRunner"));
   private final ROS2Topic controllerFootstepDataTopic;

   private FootstepPlannerRequest request;
   private boolean enableLiveMode = true;

   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;

   private ROS2Topic<FramePlanarRegionsListMessage> terrainRegionsTopic;

   private final PerceptionConfigurationParameters configurationParameters = new PerceptionConfigurationParameters();

   public ActiveMappingRemoteProcess(String simpleRobotName, DRCRobotModel robotModel, HumanoidReferenceFrames referenceFrames , ROS2Topic<FramePlanarRegionsListMessage> terrainRegionsTopic, ROS2Node ros2Node)
   {
      this.referenceFrames = referenceFrames;
      this.terrainRegionsTopic = terrainRegionsTopic;
      this.controllerFootstepDataTopic = ControllerAPIDefinition.getTopic(FootstepDataListMessage.class, robotModel.getSimpleRobotName());

      activeMappingModule = new ActiveMappingModule(robotModel, referenceFrames);

      ros2Helper = new ROS2Helper(ros2Node);
      publisherMap = new ROS2PublisherMap(ros2Node);
      publisherMap.getOrCreatePublisher(controllerFootstepDataTopic);
      ros2Helper.subscribeViaCallback(terrainRegionsTopic, this::onPlanarRegionsReceived);
      controllerRegionsPublisher = ROS2Tools.createPublisher(ros2Node, StepGeneratorAPIDefinition.getTopic(PlanarRegionsListMessage.class, simpleRobotName));
      slamOutputRegionsPublisher = ROS2Tools.createPublisher(ros2Node, PerceptionAPI.SLAM_OUTPUT_RAPID_REGIONS);

      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(WalkingControllerFailureStatusMessage.class, simpleRobotName), message ->
      {
         setEnableLiveMode(false);
         resetMap();
      });

      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);
      ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERCEPTION_CONFIGURATION_PARAMETERS, configurationParameters);

      ros2Helper.subscribeViaCallback(PerceptionAPI.PERSPECTIVE_RAPID_REGIONS_WITH_POSE, this::onPlanarRegionsReceived);
      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(WalkingStatusMessage.class, robotModel.getSimpleRobotName()), walkingStatusMessage::set);

      executorService.scheduleAtFixedRate(this::updateActiveMappingPlan, 0, 500, TimeUnit.MILLISECONDS);
      executorService.scheduleAtFixedRate(this::generalUpdate, 0, STATISTICS_COLLECTION_PERIOD_MS, TimeUnit.MILLISECONDS);
   }

   private void generalUpdate()
   {
      ros2PropertySetGroup.update();
      //activeMappingModule.getPlanarRegionMap().printStatistics(true);
   }

   public void onPlanarRegionsReceived(FramePlanarRegionsListMessage message)
   {
      if (latestIncomingRegions.get() == null)
         latestIncomingRegions.set(message);

      executorService.submit(this::updateMap);
   }

   public synchronized void updateMap()
   {
      if (latestIncomingRegions.get() == null)
      {
         LogTools.warn("No regions received");
         return;
      }

      FramePlanarRegionsList framePlanarRegionsList = PlanarRegionMessageConverter.convertToFramePlanarRegionsList(latestIncomingRegions.getAndSet(null));

      if (enableLiveMode)
      {
         updateMapWithNewRegions(framePlanarRegionsList);
      }

      PlanarRegionsList regionsToPublish = latestPlanarRegionsForPublishing.getAndSet(null);
      if (regionsToPublish != null)
      {
         PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regionsToPublish);
         slamOutputRegionsPublisher.publish(planarRegionsListMessage);
      }

      if (configurationParameters.getSLAMReset())
      {
         resetMap();
         configurationParameters.setSLAMReset(false);
      }
   }

   public void updateMapWithNewRegions(FramePlanarRegionsList regions)
   {
      activeMappingModule.updateMap(regions);
      latestPlanarRegionsForPublishing.set(activeMappingModule.getPlanarRegionMap().getMapRegions().copy());
   }

   public void updateActiveMappingPlan()
   {
      LogTools.info("Updating Active Mapping Plan: " + configurationParameters.getActiveMapping());
      if (configurationParameters.getActiveMapping())
      {
         activeMappingModule.updateFootstepPlan();
         sendActiveMappingPlanToController();
         configurationParameters.setActiveMapping(false);
      }
   }

   public void sendActiveMappingPlanToController()
   {
      LogTools.info("Publishing Plan Result");
      FootstepDataListMessage footstepDataList = activeMappingModule.getFootstepDataListMessage();
      publisherMap.publish(controllerFootstepDataTopic, footstepDataList);
   }

   public void resetMap()
   {
      activeMappingModule.reset();
   }

   public void setEnableLiveMode(boolean enableLiveMode)
   {
      this.enableLiveMode = enableLiveMode;
   }
}
