package us.ihmc.rdx.perception;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.PlanarRegionsListWithPoseMessage;
import us.ihmc.avatar.logging.PlanarRegionsListLogger;
import us.ihmc.avatar.logging.PlanarRegionsReplayBuffer;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StepGeneratorAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.mapping.PlanarRegionMap;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListWithPose;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.io.File;
import java.io.IOException;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class PlanarRegionMappingManager
{
   private enum DataSource
   {
      ROS2_CALLBACK, PLANAR_REGIONS_LOG, PERCEPTION_LOG
   }

   private final DataSource source;

   private final static long PUBLISH_MILLISECONDS = 100;

   private ROS2Node ros2Node = null;
   private ROS2Helper ros2Helper = null;
   private IHMCROS2Publisher<PlanarRegionsListMessage> controllerRegionsPublisher;


   private final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                        getClass(),
                                                                                                        ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);
   private ScheduledFuture<?> updateMapFuture;

   private final AtomicReference<PlanarRegionsListWithPoseMessage> latestIncomingRegions = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> latestPlanarRegionsForRendering = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> latestPlanarRegionsForPublishing = new AtomicReference<>(null);

   private boolean enableCapture = false;
   private boolean enableLiveMode = false;

   private static final File logDirectory = new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator);

   private PlanarRegionsReplayBuffer planarRegionsListBuffer = null;

   private RDXGPUPlanarRegionExtractionUI gpuPlanarRegionExtractionUI;
   private PlanarRegionsListWithPose planarRegionsListWithPose;
   private PlanarRegionMap planarRegionMap;
   private PlanarRegionsListLogger planarRegionsListLogger;

   private int planarRegionListIndex = 0;
   private int perceptionLogIndex = 0;

   private float depthToMetersScalar = 2.500000118743628E-4f;

   private final Mat depthImage = new Mat(768, 1024, opencv_core.CV_16UC1);
   private BytedecoImage depth32FC1Image;

   private PerceptionDataLoader perceptionDataLoader;

   private static final File regionLogDirectory = new File(
         System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator);

   private final String defaultLogDirectory =
         System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator + "perception" + File.separator + "Good" + File.separator;

   private final String perceptionLogDirectory = System.getProperty("perception.log.directory", defaultLogDirectory);
   private final String logFileName = "20221216_141954_PerceptionLog.hdf5";

   public PlanarRegionMappingManager(String simpleRobotName, ROS2Node ros2Node, boolean smoothing)
   {
      source = DataSource.ROS2_CALLBACK;
      planarRegionMap = new PlanarRegionMap(smoothing);

      if(ros2Node != null)
      {
         this.ros2Node = ros2Node;
         this.ros2Helper = new ROS2Helper(ros2Node);

         launchMapper();
         controllerRegionsPublisher = ROS2Tools.createPublisher(ros2Node, StepGeneratorAPIDefinition.getTopic(PlanarRegionsListMessage.class, simpleRobotName));
         ros2Helper.subscribeViaCallback(ROS2Tools.MAPSENSE_REGIONS_WITH_POSE, latestIncomingRegions::set);
      }
   }

   public PlanarRegionMappingManager(RDXGPUPlanarRegionExtractionUI gpuPlanarRegionExtractionUI, BytedecoImage depth32FC1Image, boolean smoothing)
   {
      source = DataSource.PERCEPTION_LOG;
      planarRegionMap = new PlanarRegionMap(smoothing);

      this.gpuPlanarRegionExtractionUI = gpuPlanarRegionExtractionUI;
      this.depth32FC1Image = depth32FC1Image;

      perceptionDataLoader = new PerceptionDataLoader();
      perceptionDataLoader.openLogFile(perceptionLogDirectory + logFileName);

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

   public void nextButtonCallback()
   {
      if (source == DataSource.PLANAR_REGIONS_LOG && (planarRegionListIndex < planarRegionsListBuffer.getBufferLength()))
      {
         planarRegionsListWithPose = (PlanarRegionsListWithPose) planarRegionsListBuffer.get(planarRegionListIndex);
         LogTools.info("Transform: {}", planarRegionsListWithPose.getSensorToWorldFrameTransform());

         updateMapWithNewRegions(planarRegionsListWithPose);
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

   public PlanarRegionsList pollMapRegions()
   {
      return latestPlanarRegionsForRendering.getAndSet(null);
   }

   public boolean pollIsModified()
   {
      boolean modified = planarRegionMap.isModified();
      planarRegionMap.setModified(false);
      return modified;
   }

   public boolean hasPlanarRegionsToRender()
   {
      return latestPlanarRegionsForRendering.get() != null;
   }

   public void updateMapWithNewRegions(PlanarRegionsListWithPose regions)
   {
      planarRegionMap.submitRegionsUsingIterativeReduction(regions);
      latestPlanarRegionsForRendering.set(planarRegionMap.getMapRegions().copy());
      latestPlanarRegionsForPublishing.set(planarRegionMap.getMapRegions().copy());
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
      planarRegionMap.reset();
      latestPlanarRegionsForRendering.set(new PlanarRegionsList());
      planarRegionMap.setModified(true);
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
