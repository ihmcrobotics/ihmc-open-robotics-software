package us.ihmc.rdx.perception;

import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_core;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.PlanarRegionsListWithPoseMessage;
import us.ihmc.avatar.logging.PlanarRegionsListLogger;
import us.ihmc.avatar.logging.PlanarRegionsReplayBuffer;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StepGeneratorAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.mapping.PlanarRegionMap;
import us.ihmc.perception.mapping.PlanarRegionMappingParameters;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListWithPose;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class PlanarRegionMappingManager
{
   private enum DataSource
   {
      ROS2_CALLBACK, PLANAR_REGIONS_LOG, PERCEPTION_LOG, SUBMIT_API
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
   private boolean modified = false;

   private static final File logDirectory = new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator);

   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private RapidPlanarRegionsExtractor rapidRegionsExtractor;
   private RDXRapidRegionsUIPanel rapidRegionsUIPanel;

   private PlanarRegionsReplayBuffer planarRegionsListBuffer = null;

   private PlanarRegionsListWithPose planarRegionsListWithPose;
   private PlanarRegionMap planarRegionMap;
   private PlanarRegionsListLogger planarRegionsListLogger;

   private int planarRegionListIndex = 0;
   private int perceptionLogIndex = 0;

   private BytedecoImage depth16UC1Image;

   private final RigidBodyTransform sensorTransformToWorld = new RigidBodyTransform();
   private final ReferenceFrame cameraFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("l515ReferenceFrame",
                                                                                                              ReferenceFrame.getWorldFrame(),
                                                                                                              sensorTransformToWorld);
   private final ArrayList<Point3D> sensorPositionBuffer = new ArrayList<>();
   private final ArrayList<Quaternion> sensorOrientationBuffer = new ArrayList<>();

   private PerceptionDataLoader perceptionDataLoader;

   public PlanarRegionMappingManager(boolean smoothing)
   {
      source = DataSource.SUBMIT_API;
      planarRegionMap = new PlanarRegionMap(smoothing);
   }

   public PlanarRegionMappingManager(String simpleRobotName, ROS2Node ros2Node, boolean smoothing)
   {
      source = DataSource.ROS2_CALLBACK;
      planarRegionMap = new PlanarRegionMap(smoothing);

      if (ros2Node != null)
      {
         this.ros2Node = ros2Node;
         this.ros2Helper = new ROS2Helper(ros2Node);

         launchMapper();
         controllerRegionsPublisher = ROS2Tools.createPublisher(ros2Node, StepGeneratorAPIDefinition.getTopic(PlanarRegionsListMessage.class, simpleRobotName));
         ros2Helper.subscribeViaCallback(ROS2Tools.PERSPECTIVE_RAPID_REGIONS_WITH_POSE, latestIncomingRegions::set);

         ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(WalkingControllerFailureStatusMessage.class, simpleRobotName), message ->
         {
            setEnableLiveMode(false);
            resetMap();
         });
      }
   }

   public PlanarRegionMappingManager(String logFile, boolean smoothing)
   {
      source = DataSource.PERCEPTION_LOG;

      /* L515 Parameters
            Color: [fx:901.3026, fy:901.8400, cx:635.2337, cy:350.9427, h:720, w:1280]
            Depth: [fx:730.7891, fy:731.0859, cx:528.6094, cy:408.1602, h:768, w:1024]
       */
      openCLManager = new OpenCLManager();
      openCLManager.create();
      openCLProgram = openCLManager.loadProgram("RapidRegionsExtractor");

      rapidRegionsExtractor = new RapidPlanarRegionsExtractor();

      rapidRegionsExtractor.create(openCLManager, openCLProgram, 1024, 768, 730.7891, 731.0859, 528.6094, 408.1602);

      this.depth16UC1Image = new BytedecoImage(rapidRegionsExtractor.getImageWidth(), rapidRegionsExtractor.getImageHeight(), opencv_core.CV_16UC1);

      //rapidRegionsUIPanel = new RDXRapidRegionsUIPanel();
      //rapidRegionsUIPanel.create(rapidRegionsExtractor);
      //baseUI.getImGuiPanelManager().addPanel(rapidRegionsUIPanel.getPanel());
      //baseUI.getPrimaryScene().addRenderableProvider(rapidRegionsUIPanel, RDXSceneLevel.VIRTUAL);

      planarRegionMap = new PlanarRegionMap(smoothing);

      perceptionDataLoader = new PerceptionDataLoader();
      perceptionDataLoader.openLogFile(logFile);

      perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.L515_SENSOR_POSITION, sensorPositionBuffer);
      perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.L515_SENSOR_ORIENTATION, sensorOrientationBuffer);
   }

   public PlanarRegionMappingManager(File planarRegionLogDirectory, boolean smoothing)
   {
      source = DataSource.PLANAR_REGIONS_LOG;
      planarRegionMap = new PlanarRegionMap(smoothing);

      for (File file : planarRegionLogDirectory.listFiles())
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
         LogTools.info("Regions Found: {}", planarRegionsListWithPose.getPlanarRegionsList().getNumberOfPlanarRegions());

         if (planarRegionsListWithPose.getPlanarRegionsList().getNumberOfPlanarRegions() > 0)
         {
            modified = true;
            updateMapWithNewRegions(planarRegionsListWithPose);
         }
         perceptionLogIndex++;
         rapidRegionsExtractor.setProcessing(false);
      }
   }

   private PlanarRegionsListWithPose getRegionsFromPerceptionLog(PerceptionDataLoader loader, int index)
   {
      PlanarRegionsListWithPose regionsToReturn = new PlanarRegionsListWithPose();

      loader.loadCompressedDepth(PerceptionLoggerConstants.L515_DEPTH_NAME, index, depth16UC1Image.getBytedecoOpenCVMat());
      sensorTransformToWorld.getTranslation().set(sensorPositionBuffer.get(index));
      sensorTransformToWorld.getRotation().set(sensorOrientationBuffer.get(index));
      cameraFrame.update();

      rapidRegionsExtractor.update(depth16UC1Image, cameraFrame, regionsToReturn);
      return regionsToReturn;
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
      LogTools.info("Adding Regions to Map.");
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

   public PlanarRegionMappingParameters getParameters()
   {
      return planarRegionMap.getParameters();
   }

   public void setEnableLiveMode(boolean enableLiveMode)
   {
      this.enableLiveMode = enableLiveMode;
   }

   public PlanarRegionsListWithPose getPlanarRegionsListWithPose()
   {
      return planarRegionsListWithPose;
   }
}
