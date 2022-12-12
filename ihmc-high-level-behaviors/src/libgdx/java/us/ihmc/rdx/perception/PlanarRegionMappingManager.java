package us.ihmc.rdx.perception;

import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.logging.PlanarRegionsListBuffer;
import us.ihmc.avatar.logging.PlanarRegionsListLogger;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.mapping.PlanarRegionFilteredMap;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.io.File;
import java.io.IOException;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class PlanarRegionMappingManager
{
   private final static long PUBLISH_MILLISECONDS = 100;
   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;

   private PlanarRegionFilteredMap filteredMap;
   private PlanarRegionsListLogger logger;

   private final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                        getClass(),
                                                                                                        ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private final AtomicReference<PlanarRegionsList> latestPlanarRegionsForRendering = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> latestPlanarRegionsForPublishing = new AtomicReference<>(null);

   private boolean enableCapture = false;
   private boolean enableLiveMode = false;

   private static final File logDirectory = new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator);

   private PlanarRegionsListBuffer planarRegionsListBuffer = null;

   private int planarRegionListIndex = 0;

   public PlanarRegionMappingManager(ROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;
      this.ros2Helper = new ROS2Helper(ros2Node);

      if(ros2Node != null)
      {
         executorService.scheduleAtFixedRate(this::publishLatestRegions, 0, PUBLISH_MILLISECONDS, TimeUnit.MILLISECONDS)
         ros2Helper.subscribeViaCallback(ROS2Tools.MAPSENSE_REGIONS, this::planarRegionCallback);
      }

      filteredMap = new PlanarRegionFilteredMap();

      for (File file : logDirectory.listFiles())
      {
         if (file.getName().toUpperCase().endsWith(".PRLLOG"))
         {
            try
            {
               planarRegionsListBuffer = new PlanarRegionsListBuffer(file);
            }
            catch (IOException ex)
            {
               LogTools.error(ex.getStackTrace());
            }
            break;
         }
      }
   }

   public synchronized void publishLatestRegions()
   {
      PlanarRegionsList regionsToPublish = latestPlanarRegionsForPublishing.getAndSet(null);
      if (regionsToPublish != null)
      {
         PlanarRegionsListMessage planarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regionsToPublish);
      }

   }

   public void planarRegionCallback(PlanarRegionsListMessage planarRegionsListMessage)
   {
      PlanarRegionsList planarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);

      if (enableCapture)
      {
         if (logger == null)
         {
            logger = new PlanarRegionsListLogger("planar-region-logger", 1);
            logger.start();
         }

         logger.update(System.currentTimeMillis(), planarRegions);
         enableCapture = false;
      }

      LogTools.debug("Received Planar Regions ROS2");
      if (enableLiveMode)
      {
         LogTools.debug("Registering Regions");
         updateMapWithNewRegions(planarRegions);
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
      filteredMap.setModified(modified);
      return modified;
   }

   public boolean hasPlanarRegionsToRender()
   {
      if (latestPlanarRegionsForRendering.get() == null)
         return false;

      return latestPlanarRegionsForRendering.get().getNumberOfPlanarRegions() > 0;
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

   public void setEnableLiveMode(boolean enableLiveMode)
   {
      this.enableLiveMode = enableLiveMode;
   }
}
