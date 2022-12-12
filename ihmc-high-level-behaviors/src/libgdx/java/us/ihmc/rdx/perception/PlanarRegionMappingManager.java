package us.ihmc.rdx.perception;

import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.logging.PlanarRegionsListBuffer;
import us.ihmc.avatar.logging.PlanarRegionsListLogger;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StepGeneratorAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.mapping.PlanarRegionFilteredMap;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.io.File;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class PlanarRegionMappingManager
{
   private final static long PUBLISH_MILLISECONDS = 100;
   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;
   private IHMCROS2Publisher<PlanarRegionsListMessage> controllerRegionsPublisher;

   private PlanarRegionFilteredMap filteredMap;
   private PlanarRegionsListLogger logger;

   private final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                        getClass(),
                                                                                                        ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);
   private ScheduledFuture<?> updateMapFuture;

   private final AtomicReference<PlanarRegionsListMessage> latestIncomingRegions = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> latestPlanarRegionsForRendering = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> latestPlanarRegionsForPublishing = new AtomicReference<>(null);

   private boolean enableCapture = false;
   private boolean enableLiveMode = false;

   private static final File logDirectory = new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator);

   private PlanarRegionsListBuffer planarRegionsListBuffer = null;

   private int planarRegionListIndex = 0;

   public PlanarRegionMappingManager(String simpleRobotName, ROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;
      this.ros2Helper = new ROS2Helper(ros2Node);

      if(ros2Node != null)
      {
         launchMapper();
         controllerRegionsPublisher = ROS2Tools.createPublisher(ros2Node, StepGeneratorAPIDefinition.getTopic(PlanarRegionsListMessage.class, simpleRobotName));
         ros2Helper.subscribeViaCallback(ROS2Tools.MAPSENSE_REGIONS, latestIncomingRegions::set);
      }

      filteredMap = new PlanarRegionFilteredMap();

//      for (File file : logDirectory.listFiles())
//      {
//         if (file.getName().toUpperCase().endsWith(".PRLLOG"))
//         {
//            try
//            {
//               planarRegionsListBuffer = new PlanarRegionsListBuffer(file);
//            }
//            catch (IOException ex)
//            {
//               LogTools.error(ex.getStackTrace());
//            }
//            break;
//         }
//      }
   }

   private void launchMapper()
   {
      updateMapFuture = executorService.scheduleAtFixedRate(this::updateMap, 0, PUBLISH_MILLISECONDS, TimeUnit.MILLISECONDS);
   }

   public synchronized void updateMap()
   {
      if (latestIncomingRegions.get() == null)
         return;

      PlanarRegionsList planarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(latestIncomingRegions.getAndSet(null));

      if (enableLiveMode)
      {
         LogTools.debug("Registering Regions");
         updateMapWithNewRegions(planarRegions);
      }

      PlanarRegionsList regionsToPublish = latestPlanarRegionsForPublishing.getAndSet(null);
      if (regionsToPublish != null)
      {
         PlanarRegionsListMessage planarRegionsToPublish = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regionsToPublish);
         controllerRegionsPublisher.publish(planarRegionsToPublish);
      }
   }

   public void nextButtonCallback()
   {
      if (planarRegionListIndex < planarRegionsListBuffer.getBufferLength())
      {
         PlanarRegionsList planarRegions = planarRegionsListBuffer.get(planarRegionListIndex);
         updateMapWithNewRegions(planarRegions);
         planarRegionListIndex++;
      }
   }

   public PlanarRegionsList pollMapRegions()
   {
      return latestPlanarRegionsForRendering.getAndSet(null);
   }

   public boolean pollIsModified()
   {
      boolean modified = filteredMap.isModified();
      filteredMap.setModified(false);
      return modified;
   }

   public boolean hasPlanarRegionsToRender()
   {
      return latestPlanarRegionsForRendering.get() != null;
   }

   public void updateMapWithNewRegions(PlanarRegionsList regions)
   {
      filteredMap.submitRegionsUsingIterativeReduction(regions);
      latestPlanarRegionsForRendering.set(filteredMap.getMapRegions().copy());
      latestPlanarRegionsForPublishing.set(filteredMap.getMapRegions().copy());
   }

   public boolean isCaptured()
   {
      return enableCapture;
   }

   public void setCaptured(boolean enableCapture)
   {
      this.enableCapture = enableCapture;
   }

   public boolean isEnabled()
   {
      return enableLiveMode;
   }

   public void resetMap()
   {
      filteredMap.reset();
      latestPlanarRegionsForRendering.set(new PlanarRegionsList());
      filteredMap.setModified(true);
      if (updateMapFuture.isCancelled() || updateMapFuture.isDone())
         launchMapper();
   }

   public void hardResetTheMap()
   {
      updateMapFuture.cancel(true);
      resetMap();
   }

   public void setEnableLiveMode(boolean enableLiveMode)
   {
      this.enableLiveMode = enableLiveMode;
   }
}
