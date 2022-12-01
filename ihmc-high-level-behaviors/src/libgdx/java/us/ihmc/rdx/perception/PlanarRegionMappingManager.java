package us.ihmc.rdx.perception;

import imgui.ImGui;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.logging.PlanarRegionsListBuffer;
import us.ihmc.avatar.logging.PlanarRegionsListLogger;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.mapping.PlanarRegionFilteredMap;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.UnitConversions;

import java.io.File;
import java.io.IOException;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

public class PlanarRegionMappingManager
{
   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;

   private PlanarRegionsList planarRegions;
   private PlanarRegionFilteredMap filteredMap;
   private PlanarRegionsListLogger logger;

   private ScheduledExecutorService executorService = new ScheduledThreadPoolExecutor(2);

   private boolean enableCapture = false;
   private boolean enableLiveMode = false;

   private static final File logDirectory = new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator);

   private PlanarRegionsListBuffer prlBuffer = null;

   private int prlIndex = 0;

   public PlanarRegionMappingManager()
   {
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "filtered_map_node");
      ros2Helper = new ROS2Helper(ros2Node);
      ros2Helper.subscribeViaCallback(ROS2Tools.MAPSENSE_REGIONS, this::planarRegionCallback);
      filteredMap = new PlanarRegionFilteredMap();
      executorService.scheduleAtFixedRate(this::scheduledUpdate, 0L, 100L, TimeUnit.MILLISECONDS);

      for (File f : logDirectory.listFiles())
      {
         if (f.getName().toUpperCase().endsWith(".PRLLOG"))
         {
            try
            {
               prlBuffer = new PlanarRegionsListBuffer(f);
            }
            catch (IOException ex)
            {
               LogTools.error(ex.getStackTrace());
            }
            break;
         }
      }
   }

   public void scheduledUpdate()
   {
      synchronized (this)
      {
         filteredMap.submitRegions(planarRegions);
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
         LogTools.info("Regions Captured: {}", planarRegions.getNumberOfPlanarRegions());

         logger.update(System.currentTimeMillis(), planarRegions);
         enableCapture = false;
      }

      if (enableLiveMode)
      {
         synchronized (this)
         {
            planarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
         }
      }
   }

   public void nextButtonCallback()
   {
      if (prlIndex < prlBuffer.getBufferLength())
      {
         planarRegions = prlBuffer.get(prlIndex);
         filteredMap.submitRegions(planarRegions);
         prlIndex++;
      }
   }

   public PlanarRegionsList getMapRegions()
   {
      return filteredMap.getMapRegions();
   }

   public PlanarRegionFilteredMap getFilteredMap()
   {
      return filteredMap;
   }

   public boolean isCaptured()
   {
      return enableCapture;
   }

   public void setCaptured(boolean enableCapture)
   {
      this.enableCapture = enableCapture;
   }

   public void setEnableLiveMode(boolean enableLiveMode)
   {
      this.enableLiveMode = enableLiveMode;
   }
}
