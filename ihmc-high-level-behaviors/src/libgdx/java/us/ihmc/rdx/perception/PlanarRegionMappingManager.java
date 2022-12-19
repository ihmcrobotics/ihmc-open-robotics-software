package us.ihmc.rdx.perception;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.PlanarRegionsListWithPoseMessage;
import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtraction;
import us.ihmc.avatar.logging.PlanarRegionsListLogger;
import us.ihmc.avatar.logging.PlanarRegionsReplayBuffer;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.mapping.PlanarRegionMap;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListWithPose;
import us.ihmc.ros2.ROS2Node;

import java.io.File;
import java.io.IOException;

public class PlanarRegionMappingManager
{
   private enum DataSource
   {
      ROS2_CALLBACK, PLANAR_REGIONS_LOG, PERCEPTION_LOG
   }

   private final DataSource source;

   private RDXGPUPlanarRegionExtractionUI gpuPlanarRegionExtractionUI;
   private PlanarRegionsListWithPose planarRegionsListWithPose;
   private PlanarRegionMap planarRegionMap;
   private PlanarRegionsListLogger planarRegionsListLogger;

   private boolean enableCapture = true;
   private boolean enableLiveMode = false;
   private int planarRegionListIndex = 0;
   private int perceptionLogIndex = 0;
   private float depthToMetersScalar = 2.500000118743628E-4f;

   private ROS2Node ros2Node;
   private ROS2Helper ros2Helper;

   private final Mat depthImage = new Mat(768, 1024, opencv_core.CV_16UC1);
   private BytedecoImage depth32FC1Image;

   private PlanarRegionsReplayBuffer planarRegionsListBuffer = null;
   private PerceptionDataLoader perceptionDataLoader;

   private final String defaultLogDirectory =
         System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator + "perception" + File.separator + "Good" + File.separator;
   private static final File regionLogDirectory = new File(
         System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator);
   private final String perceptionLogDirectory = System.getProperty("perception.log.directory", defaultLogDirectory);
   private final String logFileName = "20221216_141954_PerceptionLog.hdf5";

   public PlanarRegionMappingManager(RDXGPUPlanarRegionExtractionUI planarRegionExtraction, BytedecoImage depth32FC1Image, boolean smoothing)
   {
      source = DataSource.PERCEPTION_LOG;
      planarRegionMap = new PlanarRegionMap(smoothing);

      gpuPlanarRegionExtractionUI = planarRegionExtraction;
      perceptionDataLoader = new PerceptionDataLoader();
      perceptionDataLoader.openLogFile(perceptionLogDirectory + logFileName);

      this.depth32FC1Image = depth32FC1Image;
   }

   public PlanarRegionMappingManager(boolean smoothing)
   {
      source = DataSource.PLANAR_REGIONS_LOG;
      planarRegionMap = new PlanarRegionMap(smoothing);

      for (File file : regionLogDirectory.listFiles())
      {
         if (file.getName().toUpperCase().endsWith(".PRLLOG"))
         {
            try
            {
               planarRegionsListBuffer = new PlanarRegionsReplayBuffer(file, PlanarRegionsListWithPose.class);
            }
            catch (IOException ex)
            {
               LogTools.error(ex.getStackTrace());
            }
            break;
         }
      }
   }

   public PlanarRegionMappingManager(ROS2Node ros2Node, boolean smoothing)
   {
      source = DataSource.ROS2_CALLBACK;
      planarRegionMap = new PlanarRegionMap(smoothing);

      this.ros2Node = ros2Node;
      this.ros2Helper = new ROS2Helper(ros2Node);
      ros2Helper.subscribeViaCallback(ROS2Tools.MAPSENSE_REGIONS_WITH_POSE, this::planarRegionCallback);
   }

   private void scheduledUpdate()
   {
      if (enableLiveMode)
      {
         planarRegionMap.submitRegionsUsingIterativeReduction(planarRegionsListWithPose);
      }
   }

   public void planarRegionCallback(PlanarRegionsListWithPoseMessage planarRegionsListWithPoseMessage)
   {
      if (enableCapture)
      {
         if (planarRegionsListLogger == null)
         {
            planarRegionsListLogger = new PlanarRegionsListLogger("planar-region-logger", 1);
            planarRegionsListLogger.start();
         }
         planarRegionsListWithPose = PlanarRegionMessageConverter.convertToPlanarRegionsListWithPose(planarRegionsListWithPoseMessage);
         LogTools.info("Regions Captured: {}", planarRegionsListWithPose.getPlanarRegionsList().getNumberOfPlanarRegions());

         planarRegionsListLogger.update(System.currentTimeMillis(), planarRegionsListWithPose);
         enableCapture = false;
      }

      LogTools.debug("Received Planar Regions ROS2");
      if (enableLiveMode)
      {
         LogTools.info("Callback: Fusing Regions in Live Mode!");
         planarRegionsListWithPose = PlanarRegionMessageConverter.convertToPlanarRegionsListWithPose(planarRegionsListWithPoseMessage);
         planarRegionMap.submitRegionsUsingIterativeReduction(planarRegionsListWithPose);
      }
   }

   public void nextButtonCallback()
   {
      if (source == DataSource.PLANAR_REGIONS_LOG && (planarRegionListIndex < planarRegionsListBuffer.getBufferLength()))
      {
         planarRegionsListWithPose = (PlanarRegionsListWithPose) planarRegionsListBuffer.get(planarRegionListIndex);
         LogTools.info("Transform: {}", planarRegionsListWithPose.getSensorToWorldFrameTransform());
         planarRegionMap.submitRegionsUsingIterativeReduction(planarRegionsListWithPose);
         planarRegionListIndex++;
      }

      if (source == DataSource.PERCEPTION_LOG)
      {
         LogTools.info("Loading Perception Log: {}", perceptionLogIndex);

         planarRegionsListWithPose = getRegionsFromPerceptionLog(perceptionDataLoader, perceptionLogIndex);
         if(planarRegionsListWithPose.getPlanarRegionsList().getNumberOfPlanarRegions() > 0)
         {
            planarRegionMap.submitRegionsUsingIterativeReduction(planarRegionsListWithPose);
         }
         perceptionLogIndex++;
      }
   }

   private PlanarRegionsListWithPose getRegionsFromPerceptionLog(PerceptionDataLoader loader, int index)
   {
      loader.loadCompressedDepth("/l515/depth/", index, depthImage);

      depthImage.convertTo(depth32FC1Image.getBytedecoOpenCVMat(), opencv_core.CV_32FC1, depthToMetersScalar, 0.0);

      PlanarRegionsListWithPose planarRegionsListWithPose = new PlanarRegionsListWithPose();
      gpuPlanarRegionExtractionUI.extractPlanarRegions();
      planarRegionsListWithPose.setPlanarRegionsList(gpuPlanarRegionExtractionUI.getPlanarRegionsList().copy());
      return planarRegionsListWithPose;
   }

   public PlanarRegionsList getMapRegions()
   {
      return planarRegionMap.getMapRegions().copy();
   }

   public PlanarRegionMap getPlanarRegionMap()
   {
      return planarRegionMap;
   }

   public void submitRegions(PlanarRegionsListWithPose regionsWithPose)
   {
      if (enableLiveMode)
      {
         planarRegionMap.submitRegionsUsingIterativeReduction(regionsWithPose);
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
