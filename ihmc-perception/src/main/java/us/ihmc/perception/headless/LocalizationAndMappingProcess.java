package us.ihmc.perception.headless;

import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StepGeneratorAPIDefinition;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.mapping.PlanarRegionMap;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegion;
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
 * <p>
 * Primary responsibilities include (but are not limited to):
 * 1. Receive planar regions from terrain perception process
 * 2. Receive planar regions from structural perception process
 * 3. Insert all landmark and odometry measurements (received in form of FramePlanarRegionsList objects)
 * 4. Perform factor graph optimization
 * 5. Publish optimized results for both map and localization estimates
 */
public class LocalizationAndMappingProcess
{
   private final static long STATISTICS_COLLECTION_PERIOD_MS = 100;

   private static final double maxAngleFromNormalToFilterAsShadow = 10.0;

   private ROS2Node ros2Node;
   private ROS2Helper ros2Helper;
   private PlanarRegionMap planarRegionMap;
   private IHMCROS2Publisher<PlanarRegionsListMessage> controllerRegionsPublisher;
   private final IHMCROS2Publisher<PlanarRegionsListMessage> slamOutputRegionsPublisher;

   private final AtomicReference<FramePlanarRegionsListMessage> latestIncomingRegions = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> latestPlanarRegionsForPublishing = new AtomicReference<>(null);

   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;

   private final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                        getClass(),
                                                                                                        ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   /**
    * Live Mode refers to being active and accepting new planar regions for updating the map. It can be overridden by the PerceptionConfigurationParameters
    * parameter for enabling SLAM.
    */
   private boolean enableLiveMode = true;

   private ROS2Topic<FramePlanarRegionsListMessage> terrainRegionsTopic;
   private ROS2Topic<FramePlanarRegionsListMessage> structuralRegionsTopic;
   private ScheduledFuture<?> updateMapFuture;

   private final PerceptionConfigurationParameters configurationParameters = new PerceptionConfigurationParameters();

   public LocalizationAndMappingProcess(String simpleRobotName,
                                        ROS2Topic<FramePlanarRegionsListMessage> terrainRegionsTopic,
                                        ROS2Topic<FramePlanarRegionsListMessage> structuralRegionsTopic,
                                        ROS2Node ros2Node,
                                        boolean smoothing)
   {
      planarRegionMap = new PlanarRegionMap(smoothing);

      this.terrainRegionsTopic = terrainRegionsTopic;
      this.structuralRegionsTopic = structuralRegionsTopic;

      this.ros2Node = ros2Node;
      this.ros2Helper = new ROS2Helper(ros2Node);

      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);
      ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERSPECTIVE_PLANAR_REGION_MAPPING_PARAMETERS, planarRegionMap.getParameters());
      ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERCEPTION_CONFIGURATION_PARAMETERS, configurationParameters);

      controllerRegionsPublisher = ROS2Tools.createPublisher(ros2Node, StepGeneratorAPIDefinition.getTopic(PlanarRegionsListMessage.class, simpleRobotName));
      slamOutputRegionsPublisher = ROS2Tools.createPublisher(ros2Node, PerceptionAPI.SLAM_OUTPUT_RAPID_REGIONS);
      ros2Helper.subscribeViaCallback(terrainRegionsTopic, this::onPlanarRegionsReceived);

      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(WalkingControllerFailureStatusMessage.class, simpleRobotName), message ->
      {
         LogTools.warn("Resetting Map (Walking Failure Detected)");
         setEnableLiveMode(false);
         resetMap();
      });

      updateMapFuture = executorService.scheduleAtFixedRate(this::statisticsCollectionThread, 0, STATISTICS_COLLECTION_PERIOD_MS, TimeUnit.MILLISECONDS);
   }

   private void statisticsCollectionThread()
   {
   }

   public void onPlanarRegionsReceived(FramePlanarRegionsListMessage message)
   {
      if (latestIncomingRegions.get() == null)
         latestIncomingRegions.set(message);

      executorService.submit(this::updateMap);
   }

   public synchronized void updateMap()
   {
      ros2PropertySetGroup.update();

      if (latestIncomingRegions.get() == null)
      {
         LogTools.debug("No regions received");
         return;
      }

      FramePlanarRegionsList framePlanarRegionsList = PlanarRegionMessageConverter.convertToFramePlanarRegionsList(latestIncomingRegions.getAndSet(null));
      filterShadowRegions(framePlanarRegionsList);

      if (enableLiveMode)
      {
         updateMapWithNewRegions(framePlanarRegionsList);
      }

      PlanarRegionsList regionsToPublish = latestPlanarRegionsForPublishing.getAndSet(null);
      if (regionsToPublish != null)
      {
         PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regionsToPublish);
         controllerRegionsPublisher.publish(planarRegionsListMessage);
         slamOutputRegionsPublisher.publish(planarRegionsListMessage);
      }

      if (configurationParameters.getSLAMReset())
      {
         resetMap();
         configurationParameters.setSLAMReset(false);
      }
   }

   private static void filterShadowRegions(FramePlanarRegionsList framePlanarRegionsList)
   {
      int i = 0;
      double angleFromNormal = Math.toRadians(maxAngleFromNormalToFilterAsShadow);
      double minDot = Math.cos(Math.PI / 2.0 - angleFromNormal);
      while (i < framePlanarRegionsList.getPlanarRegionsList().getNumberOfPlanarRegions())
      {
         PlanarRegion regionInSensorFrame = framePlanarRegionsList.getPlanarRegionsList().getPlanarRegion(i);
         Vector3D vectorToRegion = new Vector3D(regionInSensorFrame.getPoint());
         vectorToRegion.normalize();

         if (Math.abs(vectorToRegion.dot(regionInSensorFrame.getNormal())) < minDot)
         {
            framePlanarRegionsList.getPlanarRegionsList().getPlanarRegionsAsList().remove(i);
         }
         else
         {
            i++;
         }
      }
   }

   public void updateMapWithNewRegions(FramePlanarRegionsList regions)
   {
      RigidBodyTransform keyframePose = planarRegionMap.registerRegions(regions.getPlanarRegionsList(), regions.getSensorToWorldFrameTransform());
      PlanarRegionsList resultMap = planarRegionMap.getMapRegions();

      synchronized (resultMap)
      {
         latestPlanarRegionsForPublishing.set(resultMap.copy());
      }
   }

   public void resetMap()
   {
      planarRegionMap.reset();
      planarRegionMap.setModified(true);
   }

   public void destroy()
   {
      if (updateMapFuture != null)
         updateMapFuture.cancel(true);
      executorService.shutdownNow();
      planarRegionMap.destroy();
   }

   public void setEnableLiveMode(boolean enableLiveMode)
   {
      this.enableLiveMode = enableLiveMode;
   }

   public static void main(String[] args)
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(CommunicationMode.INTERPROCESS.getPubSubImplementation(), "slam_node");
      new LocalizationAndMappingProcess("Nadia",
                                        PerceptionAPI.PERSPECTIVE_RAPID_REGIONS,
                                        PerceptionAPI.SPHERICAL_RAPID_REGIONS_WITH_POSE,
                                        ros2Node,
                                        true);
   }
}
