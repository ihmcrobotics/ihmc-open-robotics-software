package us.ihmc.perception.headless;

import controller_msgs.msg.dds.HighLevelStateMessage;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StepGeneratorAPIDefinition;
import us.ihmc.commons.thread.Notification;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.mapping.PlanarRegionMap;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.perception.tools.PerceptionFilterTools;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
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
 * <p>
 * Primary responsibilities include (but are not limited to):
 * 1. Receive planar regions from terrain perception process
 * 2. Receive planar regions from structural perception process
 * 3. Insert all landmark and odometry measurements (received in form of FramePlanarRegionsList objects)
 * 4. Perform factor graph optimization
 * 5. Publish optimized results for both map and localization estimates
 */
public class LocalizationAndMappingTask
{
   private final static long SCHEDULED_UPDATE_PERIOD_MS = 100;

   private static final double maxAngleFromNormalToFilterAsShadow = 10.0;

   protected ROS2Helper ros2Helper;
   protected ROS2Node ros2Node;
   protected PlanarRegionMap planarRegionMap;

   private ROS2PublisherBasics<PlanarRegionsListMessage> controllerRegionsPublisher;
   private ROS2PublisherBasics<PlanarRegionsListMessage> slamOutputRegionsPublisher;

   private final AtomicReference<HighLevelStateMessage> highLevelState = new AtomicReference<>();
   private final AtomicReference<WalkingControllerFailureStatusMessage> walkingFailureStatus = new AtomicReference<>();
   private final AtomicReference<ImageMessage> latestDepthMessage = new AtomicReference<>(null);
   private final AtomicReference<FramePlanarRegionsListMessage> latestIncomingRegions = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> latestPlanarRegionsForPublishing = new AtomicReference<>(null);
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters("ForGPURegions");

   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;

   protected final PerceptionConfigurationParameters configurationParameters = new PerceptionConfigurationParameters();
   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                        getClass(),
                                                                                                        ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   /**
    * Live Mode refers to being active and accepting new planar regions for updating the map. It can be overridden by the PerceptionConfigurationParameters
    * parameter for enabling SLAM.
    */
   private boolean enableLiveMode = true;
   private boolean smoothingEnabled = false;

   private HumanoidReferenceFrames referenceFrames;
   private Runnable referenceFramesUpdater;
   private ROS2Topic<FramePlanarRegionsListMessage> terrainRegionsTopic;
   private ROS2Topic<FramePlanarRegionsListMessage> structuralRegionsTopic;
   private ScheduledFuture<?> updateMapFuture;

   private final Notification shouldUpdateMap = new Notification();

   public LocalizationAndMappingTask(String simpleRobotName,
                                     ROS2Topic<FramePlanarRegionsListMessage> terrainRegionsTopic,
                                     ROS2Topic<FramePlanarRegionsListMessage> structuralRegionsTopic,
                                     ROS2Node ros2Node,
                                     HumanoidReferenceFrames referenceFrames,
                                     Runnable referenceFramesUpdater,
                                     boolean smoothing)
   {
      this.referenceFramesUpdater = referenceFramesUpdater;
      this.terrainRegionsTopic = terrainRegionsTopic;
      this.structuralRegionsTopic = structuralRegionsTopic;
      this.referenceFrames = referenceFrames;
      this.smoothingEnabled = smoothing;

      this.planarRegionMap = new PlanarRegionMap(smoothing, "Fast");
      planarRegionMap.setInitialSupportSquareEnabled(configurationParameters.getSupportSquareEnabled());

      this.ros2Node = ros2Node;
      this.ros2Helper = new ROS2Helper(ros2Node);

      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);
      ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERSPECTIVE_PLANAR_REGION_MAPPING_PARAMETERS, planarRegionMap.getParameters());
      ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERCEPTION_CONFIGURATION_PARAMETERS, configurationParameters);

      controllerRegionsPublisher = ros2Node.createPublisher(StepGeneratorAPIDefinition.getTopic(PlanarRegionsListMessage.class, simpleRobotName));
      slamOutputRegionsPublisher = ros2Node.createPublisher(PerceptionAPI.SLAM_OUTPUT_RAPID_REGIONS);
      ros2Helper.subscribeViaCallback(terrainRegionsTopic, this::onPlanarRegionsReceived);

      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(WalkingControllerFailureStatusMessage.class, simpleRobotName), message ->
      {
         LogTools.warn("Resetting Map (Walking Failure Detected)");
         setEnableLiveMode(false);
      });

      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(HighLevelStateMessage.class, simpleRobotName), highLevelState::set);

      updateMapFuture = executorService.scheduleAtFixedRate(this::scheduledUpdate, 0, SCHEDULED_UPDATE_PERIOD_MS, TimeUnit.MILLISECONDS);
   }

   private void scheduledUpdate()
   {
      planarRegionMap.setInitialSupportSquareEnabled(configurationParameters.getSupportSquareEnabled());

      ros2PropertySetGroup.update();

      HighLevelStateMessage highLevelState = this.highLevelState.getAndSet(null);
      if (highLevelState != null)
         if (highLevelState.getHighLevelControllerName() != HighLevelControllerName.WALKING.toByte())
            resetMap();

      if (shouldUpdateMap.poll())
      {
         updateMap();
      }
   }

   public void onOusterDepthReceived(ImageMessage depthMessage)
   {
      if (latestDepthMessage.get() == null)
         latestDepthMessage.set(depthMessage);
   }

   public void onPlanarRegionsReceived(FramePlanarRegionsListMessage message)
   {
      if (latestIncomingRegions.get() == null)
         latestIncomingRegions.set(message);

      executorService.submit(this::updateMap);
   }

   public synchronized void updateMap()
   {
      referenceFramesUpdater.run();

      if (latestIncomingRegions.get() == null)
      {
         LogTools.debug("No regions received");
         return;
      }

      RigidBodyTransform midFootTransform = referenceFrames.getMidFeetZUpFrame().getTransformToWorldFrame();

      FramePlanarRegionsList framePlanarRegionsList = PlanarRegionMessageConverter.convertToFramePlanarRegionsList(latestIncomingRegions.getAndSet(null));

      if (configurationParameters.getShadowFilter())
      {
         PerceptionFilterTools.filterShadowRegions(framePlanarRegionsList, maxAngleFromNormalToFilterAsShadow);
      }

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

      PlanarRegionsList resultMap = planarRegionMap.getMapRegions().copy();

      if (configurationParameters.getBoundingBoxFilter())
      {
         BoundingBox3D boundingBox = new BoundingBox3D(midFootTransform.getTranslationX() - 2.0f,
                                                       midFootTransform.getTranslationY() - 2.0f,
                                                       -2.0f,
                                                       midFootTransform.getTranslationX() + 2.0f,
                                                       midFootTransform.getTranslationY() + 2.0f,
                                                       2.0f);
         PerceptionFilterTools.filterByBoundingBox(resultMap, boundingBox);
      }

      if (configurationParameters.getConcaveHullFilters())
      {
         PerceptionFilterTools.applyConcaveHullReduction(resultMap, polygonizerParameters);
      }

      if (configurationParameters.getSLAMReset())
      {
         resetMap();
         configurationParameters.setSLAMReset(false);
      }
   }

   public void updateMapWithNewRegions(FramePlanarRegionsList regions)
   {
      referenceFramesUpdater.run();

      RigidBodyTransform midFootTransform = referenceFrames.getMidFeetZUpFrame().getTransformToWorldFrame();
      planarRegionMap.registerRegions(regions.getPlanarRegionsList(), regions.getSensorToWorldFrameTransform(), midFootTransform);

      PlanarRegionsList resultMap = planarRegionMap.getMapRegions().copy();

      if (configurationParameters.getBoundingBoxFilter())
      {
         BoundingBox3D boundingBox = new BoundingBox3D(midFootTransform.getTranslationX() - 2.0f,
                                                       midFootTransform.getTranslationY() - 2.0f,
                                                       -2.0f,
                                                       midFootTransform.getTranslationX() + 2.0f,
                                                       midFootTransform.getTranslationY() + 2.0f,
                                                       2.0f);
         PerceptionFilterTools.filterByBoundingBox(resultMap, boundingBox);
      }

      if (configurationParameters.getConcaveHullFilters())
      {
         PerceptionFilterTools.applyConcaveHullReduction(resultMap, polygonizerParameters);
      }

      latestPlanarRegionsForPublishing.set(resultMap);
   }

   public void resetMap()
   {
      latestIncomingRegions.set(null);
      latestPlanarRegionsForPublishing.set(null);

      planarRegionMap.destroy();
      planarRegionMap = new PlanarRegionMap(this.smoothingEnabled, "Fast");
      planarRegionMap.setInitialSupportSquareEnabled(configurationParameters.getSupportSquareEnabled());
      enableLiveMode = true;
   }

   public void destroy()
   {
      if (updateMapFuture != null)
         updateMapFuture.cancel(true);
      planarRegionMap.destroy();
      executorService.shutdown();
   }

   public void setEnableLiveMode(boolean enableLiveMode)
   {
      this.enableLiveMode = enableLiveMode;
   }
}
