package us.ihmc.perception;

import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_core;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import us.ihmc.avatar.logging.PlanarRegionsListLogger;
import us.ihmc.avatar.logging.PlanarRegionsReplayBuffer;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StepGeneratorAPIDefinition;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.mapping.PlanarRegionMap;
import us.ihmc.perception.mapping.PlanarRegionMappingParameters;
import us.ihmc.perception.odometry.RapidPatchesBasedICP;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.perception.tools.PlaneRegistrationTools;
import us.ihmc.robotics.geometry.PlanarLandmarkList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class PlanarRegionMappingHandler
{
   private enum DataSource
   {
      ROS2_CALLBACK, PLANAR_REGIONS_LOG, PERCEPTION_LOG, SUBMIT_API
   }

   private final DataSource source;

   private final static long PUBLISH_MILLISECONDS = 100;

   private int delayMilliseconds = 100;

   private ROS2Node ros2Node = null;
   private ROS2Helper ros2Helper = null;
   private IHMCROS2Publisher<PlanarRegionsListMessage> controllerRegionsPublisher;

   private final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                        getClass(),
                                                                                                        ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);
   private ScheduledFuture<?> updateMapFuture;

   private final AtomicReference<FramePlanarRegionsListMessage> latestIncomingRegions = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> latestPlanarRegionsForRendering = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> latestPlanarRegionsForPublishing = new AtomicReference<>(null);
   private final AtomicReference<RigidBodyTransform> latestKeyframePoseForRendering = new AtomicReference<>(new RigidBodyTransform());

   private boolean enableCapture = false;
   private boolean enableLiveMode = true;

   private static final File logDirectory = new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator);

   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private RapidPlanarRegionsExtractor rapidRegionsExtractor;

   private PlanarRegionsReplayBuffer planarRegionsListBuffer = null;

   private FramePlanarRegionsList framePlanarRegionsList;
   private FramePlanarRegionsList previousRegions;
   private FramePlanarRegionsList currentRegions;

   private PlanarRegionMap planarRegionMap;
   private PlanarRegionsListLogger planarRegionsListLogger;

   private final ArrayList<Point3D> mocapPositionBuffer = new ArrayList<>();
   private final ArrayList<Quaternion> mocapOrientationBuffer = new ArrayList<>();

   private int planarRegionListIndex = 0;
   private int perceptionLogIndex = 0;
   private int totalDepthCount = 0;

   private String sensorLogChannelName;

   private final BytePointer depthPointer = new BytePointer(1000000);
   private BytedecoImage depth16UC1Image;

   private final Notification dataAvailable = new Notification();
   private final ResettableExceptionHandlingExecutorService loadAndDecompressThreadExecutor = MissingThreadTools.newSingleThreadExecutor("LoadAndDecompress",
                                                                                                                                         true,
                                                                                                                                         1);

   private final RigidBodyTransform sensorTransformToWorld = new RigidBodyTransform();
   private final ReferenceFrame cameraFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("l515ReferenceFrame",
                                                                                                              ReferenceFrame.getWorldFrame(),
                                                                                                              sensorTransformToWorld);
   private final ArrayList<Point3D> sensorPositionBuffer = new ArrayList<>();
   private final ArrayList<Quaternion> sensorOrientationBuffer = new ArrayList<>();

   private final RapidPatchesBasedICP rapidPatchesBasedICP = new RapidPatchesBasedICP();

   private PerceptionDataLoader perceptionDataLoader;

   public PlanarRegionMappingHandler()
   {
      source = DataSource.SUBMIT_API;
      planarRegionMap = new PlanarRegionMap(true);
   }

   public PlanarRegionMappingHandler(String simpleRobotName, ROS2Node ros2Node)
   {
      source = DataSource.ROS2_CALLBACK;
      planarRegionMap = new PlanarRegionMap(true);

      if (ros2Node != null)
      {
         this.ros2Node = ros2Node;
         this.ros2Helper = new ROS2Helper(ros2Node);

         launchMapper();
         controllerRegionsPublisher = ROS2Tools.createPublisher(ros2Node, StepGeneratorAPIDefinition.getTopic(PlanarRegionsListMessage.class, simpleRobotName));
         ros2Helper.subscribeViaCallback(PerceptionAPI.PERSPECTIVE_RAPID_REGIONS, latestIncomingRegions::set);

         ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(WalkingControllerFailureStatusMessage.class, simpleRobotName), message ->
         {
            setEnableLiveMode(false);
            resetMap();
         });
      }
   }

   public PlanarRegionMappingHandler(String logFile, boolean smoothing)
   {
      source = DataSource.PERCEPTION_LOG;

      /* L515 Parameters
            Color: [fx:901.3026, fy:901.8400, cx:635.2337, cy:350.9427, h:720, w:1280]
            Depth: [fx:730.7891, fy:731.0859, cx:528.6094, cy:408.1602, h:768, w:1024]
       */

      openCLManager = new OpenCLManager();
      openCLProgram = openCLManager.loadProgram("RapidRegionsExtractor");

      perceptionDataLoader = new PerceptionDataLoader();
      perceptionDataLoader.openLogFile(logFile);

//      perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION, mocapPositionBuffer);
//      perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.MOCAP_RIGID_BODY_ORIENTATION, mocapOrientationBuffer);

      //createOuster(128, 1024, smoothing);
      createTerrain(720, 1280, false);
   }

   private void createTerrain(int depthHeight, int depthWidth, boolean simulation)
   {
      planarRegionMap = new PlanarRegionMap(true, "Fast");
      sensorLogChannelName = PerceptionLoggerConstants.L515_DEPTH_NAME;

      String version = simulation ? "Simulation" : "";

      rapidRegionsExtractor = new RapidPlanarRegionsExtractor(openCLManager, openCLProgram, depthHeight, depthWidth, 654.29, 654.29, 651.14, 361.89, version);
      rapidPatchesBasedICP.create(openCLManager, openCLProgram, depthHeight, depthWidth);
      depth16UC1Image = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);

      perceptionDataLoader.loadCompressedDepth(sensorLogChannelName, perceptionLogIndex, depthPointer, depth16UC1Image.getBytedecoOpenCVMat());
      perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.L515_SENSOR_POSITION, sensorPositionBuffer);
      perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.L515_SENSOR_ORIENTATION, sensorOrientationBuffer);

      totalDepthCount = perceptionDataLoader.getHDF5Manager().getCount(sensorLogChannelName);
   }

   private void createOuster(int depthHeight, int depthWidth)
   {
      planarRegionMap = new PlanarRegionMap(true, "Spherical");
      sensorLogChannelName = PerceptionLoggerConstants.OUSTER_DEPTH_NAME;
      rapidRegionsExtractor = new RapidPlanarRegionsExtractor(openCLManager, openCLProgram, depthHeight, depthWidth);
      rapidPatchesBasedICP.create(openCLManager, openCLProgram, depthHeight, depthWidth);
      depth16UC1Image = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);

      perceptionDataLoader.loadCompressedDepth(sensorLogChannelName, perceptionLogIndex, depthPointer, depth16UC1Image.getBytedecoOpenCVMat());
      perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.OUSTER_SENSOR_POSITION, sensorPositionBuffer);
      perceptionDataLoader.loadQuaternionList(PerceptionLoggerConstants.OUSTER_SENSOR_ORIENTATION, sensorOrientationBuffer);

      totalDepthCount = perceptionDataLoader.getHDF5Manager().getCount(sensorLogChannelName);
   }

   public PlanarRegionMappingHandler(File planarRegionLogDirectory)
   {
      source = DataSource.PLANAR_REGIONS_LOG;
      planarRegionMap = new PlanarRegionMap(true);

      for (File file : planarRegionLogDirectory.listFiles())
      {
         if (file.getName().toUpperCase().endsWith(".PRLLOG"))
         {
            try
            {
               planarRegionsListBuffer = new PlanarRegionsReplayBuffer(file, FramePlanarRegionsList.class);
            }
            catch (IOException ioException)
            {
               LogTools.error(ioException.getStackTrace());
            }
            break;
         }
      }
   }

   public void planarRegionCallback(FramePlanarRegionsListMessage framePlanarRegionsListMessage)
   {
      if (enableCapture)
      {
         if (planarRegionsListLogger == null)
         {
            planarRegionsListLogger = new PlanarRegionsListLogger("planar-region-logger", 1);
            planarRegionsListLogger.start();
         }
         framePlanarRegionsList = PlanarRegionMessageConverter.convertToFramePlanarRegionsList(framePlanarRegionsListMessage);
         LogTools.debug("Regions Captured: {}", framePlanarRegionsList.getPlanarRegionsList().getNumberOfPlanarRegions());

         planarRegionsListLogger.update(System.currentTimeMillis(), framePlanarRegionsList);
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

      FramePlanarRegionsList framePlanarRegionsList = PlanarRegionMessageConverter.convertToFramePlanarRegionsList(latestIncomingRegions.getAndSet(null));

      if (enableLiveMode)
      {
         LogTools.debug("Registering Regions");
         updateMapWithNewRegions(framePlanarRegionsList);
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
         framePlanarRegionsList = (FramePlanarRegionsList) planarRegionsListBuffer.get(planarRegionListIndex);
         LogTools.debug("Transform: {}", framePlanarRegionsList.getSensorToWorldFrameTransform());

         updateMapWithNewRegions(framePlanarRegionsList);
         planarRegionListIndex++;
      }

      if (source == DataSource.PERCEPTION_LOG)
      {
         LogTools.debug("Loading Perception Log: {}", perceptionLogIndex);

         loadDataFromPerceptionLog(perceptionDataLoader, perceptionLogIndex);

         while (!dataAvailable.poll())
         {
            try
            {
               Thread.sleep(10);
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }
         }

         framePlanarRegionsList = new FramePlanarRegionsList();
         rapidRegionsExtractor.update(depth16UC1Image, cameraFrame, framePlanarRegionsList);

         LogTools.debug("Regions Found: {}", framePlanarRegionsList.getPlanarRegionsList().getNumberOfPlanarRegions());

         //rapidPatchesBasedICP.update(rapidRegionsExtractor.getPreviousFeatureGrid(), rapidRegionsExtractor.getCurrentFeatureGrid());
         //rapidRegionsExtractor.copyFeatureGridMapUsingOpenCL();

         if (framePlanarRegionsList.getPlanarRegionsList().getNumberOfPlanarRegions() > 0)
         {
            planarRegionMap.setModified(true);
            updateMapWithNewRegions(framePlanarRegionsList);
         }

         perceptionLogIndex += 1;
         rapidRegionsExtractor.setProcessing(false);
      }
   }

   public void performMapCleanUp()
   {
      planarRegionMap.performMapCleanUp(true, true);
      latestPlanarRegionsForRendering.set(planarRegionMap.getMapRegions().copy());
      planarRegionMap.setModified(true);
   }

   private void loadDataFromPerceptionLog(PerceptionDataLoader loader, int index)
   {
      loadAndDecompressThreadExecutor.clearQueueAndExecute(() ->
                                                           {
                                                              loader.loadCompressedDepth(sensorLogChannelName,
                                                                                         index,
                                                                                         depthPointer,
                                                                                         depth16UC1Image.getBytedecoOpenCVMat());
                                                              sensorTransformToWorld.getTranslation().set(sensorPositionBuffer.get(index));
                                                              sensorTransformToWorld.getRotation().set(sensorOrientationBuffer.get(index));
                                                              cameraFrame.update();
                                                              dataAvailable.set();
                                                           });
   }

   public RigidBodyTransform pollKeyframePose()
   {
      LogTools.debug("Polling Keyframe Pose: {}", latestKeyframePoseForRendering.get());
      return latestKeyframePoseForRendering.getAndSet(null);
   }

   public boolean pollIsModified()
   {
      boolean modified = planarRegionMap.isModified();
      planarRegionMap.setModified(false);
      return modified;
   }

   public void updateMapWithNewRegions(FramePlanarRegionsList regions)
   {
      planarRegionMap.setModified(true);

      LogTools.debug("Updating Map with {} regions", regions.getPlanarRegionsList().getNumberOfPlanarRegions());

      RigidBodyTransform keyframePose = planarRegionMap.registerRegions(regions.getPlanarRegionsList(), regions.getSensorToWorldFrameTransform(), null);

      if (keyframePose != null)
         latestKeyframePoseForRendering.set(keyframePose);

      latestPlanarRegionsForRendering.set(planarRegionMap.getMapRegions().copy());
      latestPlanarRegionsForPublishing.set(planarRegionMap.getMapRegions().copy());

      LogTools.debug("Total Regions in Map: {}", planarRegionMap.getMapRegions().getNumberOfPlanarRegions());
   }

   public void resetMap()
   {
      planarRegionMap.destroy();
      latestKeyframePoseForRendering.set(new RigidBodyTransform());
      latestPlanarRegionsForRendering.set(new PlanarRegionsList());
      planarRegionMap.setModified(true);
      perceptionLogIndex = 0;

      if (updateMapFuture != null)
      {
         if (updateMapFuture.isCancelled() || updateMapFuture.isDone())
            launchMapper();
      }
   }

   public void loadRegionsFromLogIntoPrevious(int index)
   {
      LogTools.debug("[Previous] Loading Perception Log: {}", index);
      loadDataFromPerceptionLog(perceptionDataLoader, index);

      previousRegions = new FramePlanarRegionsList();
      rapidRegionsExtractor.update(depth16UC1Image, cameraFrame, previousRegions);
      rapidRegionsExtractor.setProcessing(false);
      LogTools.debug("[Previous] Regions Found: {}", previousRegions.getPlanarRegionsList().getNumberOfPlanarRegions());
   }

   public void loadRegionsFromLogIntoCurrent(int index)
   {
      LogTools.debug("[Current] Loading Perception Log: {}", index);
      loadDataFromPerceptionLog(perceptionDataLoader, index);

      currentRegions = new FramePlanarRegionsList();
      rapidRegionsExtractor.update(depth16UC1Image, cameraFrame, currentRegions);
      rapidRegionsExtractor.setProcessing(false);
      LogTools.debug("[Current] Regions Found: {}", currentRegions.getPlanarRegionsList().getNumberOfPlanarRegions());
   }

   public void computeICP()
   {
      RigidBodyTransform currentToPreviousTransform = new RigidBodyTransform();
      boolean valid = PlaneRegistrationTools.computeIterativeQuaternionAveragingBasedRegistration(new PlanarLandmarkList(previousRegions.getPlanarRegionsList()),
                                                                                                  new PlanarLandmarkList(currentRegions.getPlanarRegionsList()),
                                                                                                  currentToPreviousTransform,
                                                                                                  getParameters());

      if (valid)
         currentRegions.getPlanarRegionsList().applyTransform(currentToPreviousTransform);

      PerceptionDebugTools.printTransform("ComputeICP", currentToPreviousTransform, true);
   }

   public void autoIncrementButtonCallback()
   {
      updateMapFuture = executorService.scheduleAtFixedRate(this::nextButtonCallback, 0, 120, TimeUnit.MILLISECONDS);
   }

   public void pauseButtonCallback()
   {
      updateMapFuture.cancel(true);
   }

   public PlanarRegionsList pollMapRegions()
   {
      return latestPlanarRegionsForRendering.getAndSet(null);
   }

   public boolean hasPlanarRegionsToRender()
   {
      return latestPlanarRegionsForRendering.get() != null;
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

   public void hardResetTheMap()
   {
      updateMapFuture.cancel(true);
      resetMap();
   }

   public PlanarRegionMappingParameters getParameters()
   {
      return planarRegionMap.getParameters();
   }

   public ArrayList<Point3D> getMocapPositionBuffer()
   {
      return mocapPositionBuffer;
   }

   public ArrayList<Quaternion> getMocapOrientationBuffer()
   {
      return mocapOrientationBuffer;
   }

   public ArrayList<Point3D> getSensorPositionBuffer()
   {
      return sensorPositionBuffer;
   }

   public ArrayList<Quaternion> getSensorOrientationBuffer()
   {
      return sensorOrientationBuffer;
   }

   public void setEnableLiveMode(boolean enableLiveMode)
   {
      this.enableLiveMode = enableLiveMode;
   }

   public FramePlanarRegionsList getFramePlanarRegionsList()
   {
      return framePlanarRegionsList;
   }

   public RapidPlanarRegionsExtractor getRapidRegionsExtractor()
   {
      return rapidRegionsExtractor;
   }

   public void destroy()
   {
      if (updateMapFuture != null)
         updateMapFuture.cancel(true);
      executorService.shutdownNow();
      planarRegionMap.destroy();
   }

   public FramePlanarRegionsList getPreviousRegions()
   {
      return previousRegions;
   }

   public FramePlanarRegionsList getCurrentRegions()
   {
      return currentRegions;
   }

   public int getTotalDepthCount()
   {
      return totalDepthCount;
   }

   public PlanarRegionMap getPlanarRegionMap()
   {
      return planarRegionMap;
   }
}
