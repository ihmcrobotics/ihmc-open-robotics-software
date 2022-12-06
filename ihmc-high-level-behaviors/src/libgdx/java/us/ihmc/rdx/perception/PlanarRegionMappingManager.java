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

import java.io.File;
import java.io.IOException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

public class PlanarRegionMappingManager
{
   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;

   private PlanarRegionsList planarRegions;
   private PlanarRegionFilteredMap filteredMap;
   private PlanarRegionsListLogger logger;

   private ScheduledExecutorService executorService = Executors.newScheduledThreadPool(2);

   private boolean enableCapture = false;
   private boolean enableLiveMode = false;

   private static final File logDirectory = new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator);

   private PlanarRegionsListBuffer planarRegionsListBuffer = null;

   private int planarRegionListIndex = 0;

   public PlanarRegionMappingManager(ROS2Node ros2Node)
   {
      //executorService.scheduleAtFixedRate(this::scheduledUpdate, 0, 100, TimeUnit.MILLISECONDS);
      this.ros2Node = ros2Node;
      this.ros2Helper = new ROS2Helper(ros2Node);

      if(ros2Node != null)
      {
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

   private void scheduledUpdate()
   {
      if (enableLiveMode)
      {
         filteredMap.submitRegionsUsingIterativeReduction(planarRegions);
      }
   }

   public void planarRegionCallback(PlanarRegionsListMessage planarRegionsListMessage)
   {
      if (enableCapture)
      {
         if (logger == null)
         {
            logger = new PlanarRegionsListLogger("planar-region-logger", 1);
            logger.start();
         }
         planarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);

         logger.update(System.currentTimeMillis(), planarRegions);
         enableCapture = false;
      }

      LogTools.debug("Received Planar Regions ROS2");
      if (enableLiveMode)
      {
         LogTools.debug("Registering Regions");
         planarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
         filteredMap.submitRegionsUsingIterativeReduction(planarRegions);
      }
   }

   public void nextButtonCallback()
   {
      if (planarRegionListIndex < planarRegionsListBuffer.getBufferLength())
      {
         planarRegions = planarRegionsListBuffer.get(planarRegionListIndex);
         filteredMap.submitRegionsUsingIterativeReduction(planarRegions);
         planarRegionListIndex++;
      }
   }

   public PlanarRegionsList getMapRegions()
   {
      return filteredMap.getMapRegions().copy();
   }

   public PlanarRegionFilteredMap getFilteredMap()
   {
      return filteredMap;
   }

   public void submitRegions(PlanarRegionsList regions)
   {
      if (enableLiveMode)
      {
         filteredMap.submitRegionsUsingIterativeReduction(regions);
      }
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
