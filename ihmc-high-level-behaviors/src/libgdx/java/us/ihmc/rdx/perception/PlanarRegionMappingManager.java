package us.ihmc.rdx.perception;

import perception_msgs.msg.dds.PlanarRegionsListWithPoseMessage;
import us.ihmc.avatar.logging.PlanarRegionsReplayBuffer;
import us.ihmc.avatar.logging.PlanarRegionsListLogger;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.mapping.PlanarRegionMap;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListWithPose;
import us.ihmc.ros2.ROS2Node;

import java.io.File;
import java.io.IOException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

public class PlanarRegionMappingManager
{
   private static final boolean ROS2_ENABLED = false;
   private static final boolean DATASET_MODE_ENABLED = !ROS2_ENABLED;

   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "filtered_map_node");;
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

   private PlanarRegionsListWithPose planarRegionsListWithPose;
   private PlanarRegionMap filteredMap;
   private PlanarRegionsListLogger logger;

   private ScheduledExecutorService executorService = Executors.newScheduledThreadPool(2);

   private boolean enableCapture = true;
   private boolean enableLiveMode = false;

   private static final File logDirectory = new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator);

   private PlanarRegionsReplayBuffer prlBuffer = null;

   private int prlIndex = 0;

   public PlanarRegionMappingManager(boolean smoothing)
   {
      //executorService.scheduleAtFixedRate(this::scheduledUpdate, 0, 100, TimeUnit.MILLISECONDS);

      if(ROS2_ENABLED)
      {
         ros2Helper.subscribeViaCallback(ROS2Tools.MAPSENSE_REGIONS_WITH_POSE, this::planarRegionCallback);
      }

      filteredMap = new PlanarRegionMap(smoothing);

      if(DATASET_MODE_ENABLED)
      {
         for (File f : logDirectory.listFiles())
         {
            if (f.getName().toUpperCase().endsWith(".PRLLOG"))
            {
               try
               {
                  prlBuffer = new PlanarRegionsReplayBuffer(f, PlanarRegionsListWithPose.class);
               }
               catch (IOException ex)
               {
                  LogTools.error(ex.getStackTrace());
               }
               break;
            }
         }
      }
   }

   private void scheduledUpdate()
   {
      if(enableLiveMode)
      {
         filteredMap.submitRegionsUsingIterativeReduction(planarRegionsListWithPose);
      }
   }

   public void planarRegionCallback(PlanarRegionsListWithPoseMessage planarRegionsListWithPoseMessage)
   {
      if (enableCapture)
      {
         if (logger == null)
         {
            logger = new PlanarRegionsListLogger("planar-region-logger", 1);
            logger.start();
         }
         planarRegionsListWithPose = PlanarRegionMessageConverter.convertToPlanarRegionsListWithPose(planarRegionsListWithPoseMessage);
         LogTools.info("Regions Captured: {}", planarRegionsListWithPose.getPlanarRegionsList().getNumberOfPlanarRegions());

         logger.update(System.currentTimeMillis(), planarRegionsListWithPose);
         enableCapture = false;
      }

      LogTools.info("Callback: Regions Received!");
      if (enableLiveMode)
      {
            LogTools.info("Callback: Fusing Regions in Live Mode!");
            planarRegionsListWithPose = PlanarRegionMessageConverter.convertToPlanarRegionsListWithPose(planarRegionsListWithPoseMessage);
            filteredMap.submitRegionsUsingIterativeReduction(planarRegionsListWithPose);
      }
   }

   public void nextButtonCallback()
   {
      if (prlIndex < prlBuffer.getBufferLength())
      {
         planarRegionsListWithPose = (PlanarRegionsListWithPose) prlBuffer.get(prlIndex);
         LogTools.info("Transform: {}", planarRegionsListWithPose.getSensorToWorldFrameTransform());
         filteredMap.submitRegionsUsingIterativeReduction(planarRegionsListWithPose);
         prlIndex++;
      }
   }

   public PlanarRegionsList getMapRegions()
   {
      return filteredMap.getMapRegions().copy();
   }

   public PlanarRegionMap getFilteredMap()
   {
      return filteredMap;
   }

   public void submitRegions(PlanarRegionsListWithPose regionsWithPose)
   {
      if(enableLiveMode)
      {
         filteredMap.submitRegionsUsingIterativeReduction(regionsWithPose);
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
