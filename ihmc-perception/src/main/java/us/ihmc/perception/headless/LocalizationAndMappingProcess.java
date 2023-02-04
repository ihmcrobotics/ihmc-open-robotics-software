package us.ihmc.perception.headless;

import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StepGeneratorAPIDefinition;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.mapping.PlanarRegionMap;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

/**
 * LocalizationAndMappingProcess is a headless process that operates as the SLAM-backend by optimizing a factor graph generated from incoming landmark
 * and odometry measurements from ROS2 topics. It asynchronously updates the mapping and localization estimates as more measurements are received. However,
 * it publishes the most recent results of the optimization at a fixed rate. This class may be extended to include visual keypoint landmarks and visual
 * odometry from the visual perception process.
 *
 * Primary responsibilities include (but are not limited to):
 * 1. Receive planar regions from terrain perception process
 * 2. Receive planar regions from structural perception process
 * 3. Insert all landmark and odometry measurements (received in form of FramePlanarRegionsList objects)
 * 4. Perform factor graph optimization
 * 5. Publish optimized results for both map and localization estimates
 */

public class LocalizationAndMappingProcess
{
   private final static long PUBLISH_PERIOD_MILLISECONDS = 100;

   private ROS2Node ros2Node;
   private ROS2Helper ros2Helper;
   private PlanarRegionMap planarRegionMap;
   private IHMCROS2Publisher<PlanarRegionsListMessage> controllerRegionsPublisher;

   private final AtomicReference<FramePlanarRegionsListMessage> latestIncomingRegions = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> latestPlanarRegionsForRendering = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> latestPlanarRegionsForPublishing = new AtomicReference<>(null);

   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;

   private final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                        getClass(),
                                                                                                        ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private ScheduledFuture<?> updateMapFuture;
   private boolean enableLiveMode = false;

   private ROS2Topic<FramePlanarRegionsListMessage> terrainRegionsTopic;
   private ROS2Topic<FramePlanarRegionsListMessage> structuralRegionsTopic;

   public LocalizationAndMappingProcess(String simpleRobotName, ROS2Topic<FramePlanarRegionsListMessage> terrainRegionsTopic, ROS2Topic<FramePlanarRegionsListMessage> structuralRegionsTopic, ROS2Node ros2Node, boolean smoothing)
   {
      planarRegionMap = new PlanarRegionMap(true);

      this.terrainRegionsTopic = terrainRegionsTopic;
      this.structuralRegionsTopic = structuralRegionsTopic;

      this.ros2Node = ros2Node;
      this.ros2Helper = new ROS2Helper(ros2Node);

      launchMapper();
      controllerRegionsPublisher = ROS2Tools.createPublisher(ros2Node, StepGeneratorAPIDefinition.getTopic(PlanarRegionsListMessage.class, simpleRobotName));
      ros2Helper.subscribeViaCallback(terrainRegionsTopic, latestIncomingRegions::set);

      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(WalkingControllerFailureStatusMessage.class, simpleRobotName), message ->
      {
         setEnableLiveMode(false);
         resetMap();
      });

      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);
      ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERSPECTIVE_PLANAR_REGION_MAPPING_PARAMETERS, planarRegionMap.getParameters());
   }

   private void launchMapper()
   {
      updateMapFuture = executorService.scheduleAtFixedRate(this::updateMap, 0, PUBLISH_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
   }

   public synchronized void updateMap()
   {
      if (latestIncomingRegions.get() == null)
         return;

      FramePlanarRegionsList framePlanarRegionsList = PlanarRegionMessageConverter.convertToFramePlanarRegionsList(latestIncomingRegions.getAndSet(null));

      if (enableLiveMode)
      {
         LogTools.debug("Registering Regions");
         updateMapWithNewRegions(framePlanarRegionsList);
      }

      PlanarRegionsList regionsToPublish = latestPlanarRegionsForPublishing.getAndSet(null);
      if (regionsToPublish != null)
      {
         PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regionsToPublish);
         controllerRegionsPublisher.publish(planarRegionsListMessage);
      }
   }

   public void updateMapWithNewRegions(FramePlanarRegionsList regions)
   {
      planarRegionMap.submitRegionsUsingIterativeReduction(regions);
      latestPlanarRegionsForRendering.set(planarRegionMap.getMapRegions().copy());
      latestPlanarRegionsForPublishing.set(planarRegionMap.getMapRegions().copy());
   }

   public void resetMap()
   {
      planarRegionMap.reset();
      latestPlanarRegionsForRendering.set(new PlanarRegionsList());
      planarRegionMap.setModified(true);
      if (updateMapFuture.isCancelled() || updateMapFuture.isDone())
         launchMapper();
   }

   public void setEnableLiveMode(boolean enableLiveMode)
   {
      this.enableLiveMode = enableLiveMode;
   }

   public static void main(String[] args)
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(CommunicationMode.INTERPROCESS.getPubSubImplementation(), "slam_node");
      new LocalizationAndMappingProcess("Nadia", ROS2Tools.PERSPECTIVE_RAPID_REGIONS_WITH_POSE,
                                        ROS2Tools.SPHERICAL_RAPID_REGIONS_WITH_POSE, ros2Node, true);
   }
}
