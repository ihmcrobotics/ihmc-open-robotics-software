package us.ihmc.perception.headless;

import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.PlanarRegionsListWithPoseMessage;
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
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListWithPose;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class LocalizationAndMappingProcess
{
   private final static long PUBLISH_MILLISECONDS = 100;

   private ROS2Node ros2Node;
   private ROS2Helper ros2Helper;
   private PlanarRegionMap planarRegionMap;
   private IHMCROS2Publisher<PlanarRegionsListMessage> controllerRegionsPublisher;

   private final AtomicReference<PlanarRegionsListWithPoseMessage> latestIncomingRegions = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> latestPlanarRegionsForRendering = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> latestPlanarRegionsForPublishing = new AtomicReference<>(null);

   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;

   private final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                        getClass(),
                                                                                                        ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private ScheduledFuture<?> updateMapFuture;
   private boolean enableLiveMode = false;

   public LocalizationAndMappingProcess(String simpleRobotName, ROS2Node ros2Node, boolean smoothing)
   {
      planarRegionMap = new PlanarRegionMap(true);

      this.ros2Node = ros2Node;
      this.ros2Helper = new ROS2Helper(ros2Node);

      launchMapper();
      controllerRegionsPublisher = ROS2Tools.createPublisher(ros2Node, StepGeneratorAPIDefinition.getTopic(PlanarRegionsListMessage.class, simpleRobotName));
      ros2Helper.subscribeViaCallback(ROS2Tools.MAPSENSE_REGIONS_WITH_POSE, latestIncomingRegions::set);

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
      updateMapFuture = executorService.scheduleAtFixedRate(this::updateMap, 0, PUBLISH_MILLISECONDS, TimeUnit.MILLISECONDS);
   }

   public synchronized void updateMap()
   {
      if (latestIncomingRegions.get() == null)
         return;

      PlanarRegionsListWithPose planarRegionsWithPose = PlanarRegionMessageConverter.convertToPlanarRegionsListWithPose(latestIncomingRegions.getAndSet(null));

      if (enableLiveMode)
      {
         LogTools.debug("Registering Regions");
         updateMapWithNewRegions(planarRegionsWithPose);
      }

      PlanarRegionsList regionsToPublish = latestPlanarRegionsForPublishing.getAndSet(null);
      if (regionsToPublish != null)
      {
         PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regionsToPublish);
         controllerRegionsPublisher.publish(planarRegionsListMessage);
      }
   }

   public void updateMapWithNewRegions(PlanarRegionsListWithPose regions)
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
      new LocalizationAndMappingProcess("Nadia", ros2Node, true);
   }
}
