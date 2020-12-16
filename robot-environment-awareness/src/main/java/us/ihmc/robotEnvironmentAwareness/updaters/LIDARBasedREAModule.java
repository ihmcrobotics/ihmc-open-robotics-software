package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.HashMap;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import com.google.common.util.concurrent.AtomicDouble;

import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.REASensorDataFilterParametersMessage;
import controller_msgs.msg.dds.REAStateRequestMessage;
import controller_msgs.msg.dds.RequestPlanarRegionsListMessage;
import controller_msgs.msg.dds.StampedPosePacket;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsRequestType;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.tools.JOctoMapTools;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.perceptionSuite.PerceptionModule;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.SurfaceNormalFilterParameters;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.tools.thread.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class LIDARBasedREAModule implements PerceptionModule
{
   private static final String ocTreeTimeReport = "OcTree update took: ";
   private static final String reportOcTreeStateTimeReport = "Reporting OcTree state took: ";
   private static final String planarRegionsTimeReport = "OcTreePlanarRegion update took: ";
   private static final String reportPlanarRegionsStateTimeReport = "Reporting Planar Regions state took: ";

   private final TimeReporter timeReporter = new TimeReporter();

   private static final int THREAD_PERIOD_MILLISECONDS = 200;
   private static final int BUFFER_THREAD_PERIOD_MILLISECONDS = 10;
   private static final double DEFAULT_OCTREE_RESOLUTION = 0.02;

   protected static final boolean DEBUG = true;

   private final boolean manageRosNode;

   private final AtomicReference<Double> octreeResolution;

   private final REAOcTreeBuffer lidarBufferUpdater;
   private final REAOcTreeBuffer stereoVisionBufferUpdater;
   private final REAOcTreeBuffer depthCloudBufferUpdater;
   private final AtomicReference<Pose3D> latestLidarPoseReference = new AtomicReference<>(null);
   private final AtomicReference<Pose3D> latestStereoVisionPoseReference = new AtomicReference<>(null);
   private final AtomicReference<Pose3D> latestDepthPoseReference = new AtomicReference<>(null);
   private final REAOcTreeUpdater mainUpdater;
   private final REAPlanarRegionFeatureUpdater planarRegionFeatureUpdater;

   private final REAModuleStateReporter moduleStateReporter;
   private final AtomicReference<Boolean> clearOcTree;
   private final AtomicReference<Boolean> enableStereoBuffer;
   private final AtomicReference<Boolean> enableDepthCloudBuffer;
   private final AtomicReference<Boolean> enableLidarBuffer;

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(3, getClass(), ExceptionHandling.CATCH_AND_REPORT);
   private ScheduledFuture<?> scheduled;
   private final Messager reaMessager;

   private final AtomicReference<Boolean> preserveStereoOcTreeHistory;
   private final AtomicReference<Boolean> preserveDepthOcTreeHistory;
   private final FilePropertyHelper filePropertyHelper;

   private final REANetworkProvider networkProvider;

   public LIDARBasedREAModule(Messager reaMessager, FilePropertyHelper filePropertyHelper, REANetworkProvider networkProvider)
   {
      this(reaMessager, filePropertyHelper, networkProvider, true);
   }

   private LIDARBasedREAModule(Messager reaMessager, FilePropertyHelper filePropertyHelper, REANetworkProvider networkProvider, boolean manageRosNode)
   {
      this.reaMessager = reaMessager;
      this.filePropertyHelper = filePropertyHelper;
      this.manageRosNode = manageRosNode;
      this.networkProvider = networkProvider;

      moduleStateReporter = new REAModuleStateReporter(reaMessager);
      lidarBufferUpdater = new REAOcTreeBuffer(DEFAULT_OCTREE_RESOLUTION,
                                               reaMessager,
                                               REAModuleAPI.LidarBufferEnable,
                                               true,
                                               REAModuleAPI.LidarBufferOcTreeCapacity,
                                               10000,
                                               REAModuleAPI.LidarBufferMessageCapacity,
                                               500,
                                               REAModuleAPI.RequestLidarBuffer,
                                               REAModuleAPI.LidarBufferState);
      stereoVisionBufferUpdater = new REAOcTreeBuffer(DEFAULT_OCTREE_RESOLUTION,
                                                      reaMessager,
                                                      REAModuleAPI.StereoVisionBufferEnable,
                                                      false,
                                                      REAModuleAPI.StereoVisionBufferOcTreeCapacity,
                                                      1000000,
                                                      REAModuleAPI.StereoVisionBufferMessageCapacity,
                                                      1,
                                                      REAModuleAPI.RequestStereoVisionBuffer,
                                                      REAModuleAPI.StereoVisionBufferState);
      depthCloudBufferUpdater = new REAOcTreeBuffer(DEFAULT_OCTREE_RESOLUTION,
                                                    reaMessager,
                                                    REAModuleAPI.DepthCloudBufferEnable,
                                                    false,
                                                    REAModuleAPI.DepthCloudBufferOcTreeCapacity,
                                                    1000000,
                                                    REAModuleAPI.DepthCloudBufferMessageCapacity,
                                                    1,
                                                    REAModuleAPI.RequestDepthCloudBuffer,
                                                    REAModuleAPI.DepthCloudBufferState);
      REAOcTreeBuffer[] bufferUpdaters = new REAOcTreeBuffer[] {lidarBufferUpdater, stereoVisionBufferUpdater, depthCloudBufferUpdater};
      HashMap<REAOcTreeBuffer, AtomicReference<Pose3D>> latestSensorPosePositions = new HashMap<>();
      latestSensorPosePositions.put(lidarBufferUpdater, latestLidarPoseReference);
      latestSensorPosePositions.put(stereoVisionBufferUpdater, latestStereoVisionPoseReference);
      latestSensorPosePositions.put(depthCloudBufferUpdater, latestDepthPoseReference);
      mainUpdater = new REAOcTreeUpdater(DEFAULT_OCTREE_RESOLUTION, bufferUpdaters, latestSensorPosePositions, reaMessager);
      planarRegionFeatureUpdater = new REAPlanarRegionFeatureUpdater(reaMessager);
      planarRegionFeatureUpdater.bindControls();

      loadConfigurationFile(filePropertyHelper);

      reaMessager.registerTopicListener(REAModuleAPI.SaveBufferConfiguration, (content) -> lidarBufferUpdater.saveConfiguration(filePropertyHelper));
      reaMessager.registerTopicListener(REAModuleAPI.SaveBufferConfiguration, (content) -> stereoVisionBufferUpdater.saveConfiguration(filePropertyHelper));
      reaMessager.registerTopicListener(REAModuleAPI.SaveBufferConfiguration, (content) -> depthCloudBufferUpdater.saveConfiguration(filePropertyHelper));
      reaMessager.registerTopicListener(REAModuleAPI.SaveMainUpdaterConfiguration, (content) -> mainUpdater.saveConfiguration(filePropertyHelper));
      reaMessager.registerTopicListener(REAModuleAPI.SaveRegionUpdaterConfiguration,
                                        (content) -> planarRegionFeatureUpdater.saveConfiguration(filePropertyHelper));

      clearOcTree = reaMessager.createInput(REAModuleAPI.OcTreeClear, false);

      preserveStereoOcTreeHistory = reaMessager.createInput(REAModuleAPI.StereoVisionBufferPreservingEnable, false);
      preserveDepthOcTreeHistory = reaMessager.createInput(REAModuleAPI.DepthCloudBufferPreservingEnable, false);
      enableStereoBuffer = reaMessager.createInput(REAModuleAPI.StereoVisionBufferEnable, false);
      enableLidarBuffer = reaMessager.createInput(REAModuleAPI.LidarBufferEnable, true);
      enableDepthCloudBuffer = reaMessager.createInput(REAModuleAPI.DepthCloudBufferEnable, false);
      octreeResolution = reaMessager.createInput(REAModuleAPI.OcTreeResolution, mainUpdater.getMainOctree().getResolution());

      networkProvider.registerMessager(reaMessager);
      networkProvider.registerLidarScanHandler(reaMessager, this::dispatchLidarScanMessage);
      networkProvider.registerDepthPointCloudHandler(reaMessager, this::dispatchDepthPointCloudMessage);
      networkProvider.registerStereoVisionPointCloudHandler(reaMessager, this::dispatchStereoVisionPointCloudMessage);
      networkProvider.registerStampedPosePacketHandler(reaMessager, this::dispatchStampedPosePacket);
      networkProvider.registerCustomRegionsHandler(this::dispatchCustomPlanarRegion);
      networkProvider.registerPlanarRegionsListRequestHandler(this::handleRequestPlanarRegionsListMessage);
      networkProvider.registerREAStateRequestHandler(this::handleREAStateRequestMessage);
      networkProvider.registerREASensorDataFilterParametersHandler(this::handleREASensorDataFilterParametersMessage);

      // At the very end, we force the modules to submit their state so duplicate inputs have consistent values.
      reaMessager.submitMessage(REAModuleAPI.RequestEntireModuleState, true);
   }

   private void dispatchLidarScanMessage(Subscriber<LidarScanMessage> subscriber)
   {
      if (!enableLidarBuffer.get())
         return;

      LidarScanMessage message = subscriber.takeNextData();

      moduleStateReporter.registerLidarScanMessage(message);
      lidarBufferUpdater.handleLidarScanMessage(message);
      latestLidarPoseReference.set(new Pose3D(message.getLidarPosition(), message.getLidarOrientation()));
   }

   private void dispatchStereoVisionPointCloudMessage(Subscriber<StereoVisionPointCloudMessage> subscriber)
   {
      if (!enableStereoBuffer.get())
         return;

      StereoVisionPointCloudMessage message = subscriber.takeNextData();
      moduleStateReporter.registerStereoVisionPointCloudMessage(message);
      stereoVisionBufferUpdater.handleStereoVisionPointCloudMessage(message);
      latestStereoVisionPoseReference.set(new Pose3D(message.getSensorPosition(), message.getSensorOrientation()));
   }

   private void dispatchDepthPointCloudMessage(Subscriber<StereoVisionPointCloudMessage> subscriber)
   {
      if (!enableDepthCloudBuffer.get())
         return;

      StereoVisionPointCloudMessage message = subscriber.takeNextData();
      moduleStateReporter.registerDepthCloudMessage(message);
      depthCloudBufferUpdater.handleDepthCloudPointCloudMessage(message);
      latestDepthPoseReference.set(new Pose3D(message.getSensorPosition(), message.getSensorOrientation()));
   }

   private void dispatchStampedPosePacket(Subscriber<StampedPosePacket> subscriber)
   {
      StampedPosePacket message = subscriber.takeNextData();
      reaMessager.submitMessage(REAModuleAPI.TrackingCameraMessageState, new StampedPosePacket(message));
   }

   private void dispatchCustomPlanarRegion(Subscriber<PlanarRegionsListMessage> subscriber)
   {
      PlanarRegionsListMessage message = subscriber.takeNextData();
      PlanarRegionsList customPlanarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(message);
      customPlanarRegions.getPlanarRegionsAsList().forEach(planarRegionFeatureUpdater::registerCustomPlanarRegion);
   }

   private void handleRequestPlanarRegionsListMessage(Subscriber<RequestPlanarRegionsListMessage> subscriber)
   {
      RequestPlanarRegionsListMessage newMessage = subscriber.takeNextData();
      PlanarRegionsRequestType requestType = PlanarRegionsRequestType.fromByte(newMessage.getPlanarRegionsRequestType());
      if (requestType == PlanarRegionsRequestType.CLEAR)
         clearOcTree.set(true);
   }

   private void handleREAStateRequestMessage(Subscriber<REAStateRequestMessage> subscriber)
   {
      REAStateRequestMessage newMessage = subscriber.takeNextData();

      if (newMessage.getRequestResume())
         reaMessager.submitMessage(REAModuleAPI.OcTreeEnable, true);
      else if (newMessage.getRequestPause()) // We guarantee to resume if requested, regardless of the pause request.
         reaMessager.submitMessage(REAModuleAPI.OcTreeEnable, false);
      if (newMessage.getRequestClear())
         clearOcTree.set(true);
   }

   private void handleREASensorDataFilterParametersMessage(Subscriber<REASensorDataFilterParametersMessage> subscriber)
   {
      REASensorDataFilterParametersMessage newMessage = subscriber.takeNextData();

      if (!newMessage.getBoundingBoxMin().containsNaN() && !newMessage.getBoundingBoxMax().containsNaN())
      {
         BoundingBoxParametersMessage boundingBox = new BoundingBoxParametersMessage();
         boundingBox.getMin().set(newMessage.getBoundingBoxMin());
         boundingBox.getMax().set(newMessage.getBoundingBoxMax());
         reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxParameters, boundingBox);
      }
      if (newMessage.getSensorMinRange() >= 0.0)
         reaMessager.submitMessage(REAModuleAPI.LidarMinRange, newMessage.getSensorMinRange());
      if (newMessage.getSensorMaxRange() >= 0.0)
         reaMessager.submitMessage(REAModuleAPI.LidarMaxRange, newMessage.getSensorMaxRange());
   }

   private void loadConfigurationFile(FilePropertyHelper filePropertyHelper)
   {
      lidarBufferUpdater.loadConfiguration(filePropertyHelper);
      stereoVisionBufferUpdater.loadConfiguration(filePropertyHelper);
      depthCloudBufferUpdater.loadConfiguration(filePropertyHelper);
      mainUpdater.loadConfiguration(filePropertyHelper);
      planarRegionFeatureUpdater.loadConfiguration(filePropertyHelper);
   }

   private final AtomicDouble lastCompleteUpdate = new AtomicDouble(Double.NaN);

   private void mainUpdate()
   {
      if (isThreadInterrupted())
         return;

      double currentTime = JOctoMapTools.nanoSecondsToSeconds(System.nanoTime());

      boolean ocTreeUpdateSuccess = true;

      try
      {
         NormalOcTree mainOctree = mainUpdater.getMainOctree();
         Pose3DReadOnly sensorPose = mainUpdater.getSensorPose();
         if (clearOcTree.getAndSet(false))
         {
            lidarBufferUpdater.clearBuffer();
            stereoVisionBufferUpdater.clearBuffer();
            depthCloudBufferUpdater.clearBuffer();
            mainUpdater.clearOcTree();
            planarRegionFeatureUpdater.clearOcTree();

            Double latestOctreeResolution = octreeResolution.get();
            if (mainOctree.getResolution() != latestOctreeResolution)
            {
               lidarBufferUpdater.setOctreeResolution(latestOctreeResolution);
               stereoVisionBufferUpdater.setOctreeResolution(latestOctreeResolution);
               depthCloudBufferUpdater.setOctreeResolution(latestOctreeResolution);
               mainUpdater.initializeReferenceOctree(latestOctreeResolution);
            }
         }
         else
         {
            if (enableStereoBuffer.get() && !preserveStereoOcTreeHistory.get())
               mainUpdater.clearOcTreeOnNextUpdate(stereoVisionBufferUpdater);
            else if (enableDepthCloudBuffer.get() && !preserveDepthOcTreeHistory.get())
               mainUpdater.clearOcTreeOnNextUpdate(depthCloudBufferUpdater);

            timeReporter.run(mainUpdater::update, ocTreeTimeReport);
            timeReporter.run(() -> moduleStateReporter.reportOcTreeState(mainOctree), reportOcTreeStateTimeReport);
            moduleStateReporter.reportSensorPose(sensorPose);

            if (isThreadInterrupted())
               return;

            timeReporter.run(() -> planarRegionFeatureUpdater.update(mainOctree, sensorPose.getPosition()), planarRegionsTimeReport);
            timeReporter.run(() -> moduleStateReporter.reportPlanarRegionsState(planarRegionFeatureUpdater), reportPlanarRegionsStateTimeReport);

            networkProvider.update(planarRegionFeatureUpdater, ocTreeUpdateSuccess);
            networkProvider.publishCurrentState();
         }

         if (isThreadInterrupted())
            return;
      }
      catch (Exception e)
      {
         if (DEBUG)
         {
            e.printStackTrace();
         }
         else
         {
            LogTools.error(e.getClass().getSimpleName());
         }
      }

      currentTime = JOctoMapTools.nanoSecondsToSeconds(System.nanoTime());

      if (ocTreeUpdateSuccess)
         lastCompleteUpdate.set(currentTime);
   }

   private boolean isThreadInterrupted()
   {
      return Thread.interrupted() || scheduled == null || scheduled.isCancelled();
   }

   public void start()
   {
      if (scheduled == null)
      {
         scheduled = executorService.scheduleAtFixedRate(this::mainUpdate, 0, THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
         executorService.scheduleAtFixedRate(lidarBufferUpdater.createBufferThread(), 0, BUFFER_THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
         executorService.scheduleAtFixedRate(stereoVisionBufferUpdater.createBufferThread(), 0, BUFFER_THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
         executorService.scheduleAtFixedRate(depthCloudBufferUpdater.createBufferThread(), 0, BUFFER_THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }
   }

   public void setParametersForStereo()
   {
      BoundingBoxParametersMessage boundingBoxMessage = new BoundingBoxParametersMessage();
      boundingBoxMessage.setMaxX(1.0f);
      boundingBoxMessage.setMinX(0.0f);
      boundingBoxMessage.setMaxY(1.0f);
      boundingBoxMessage.setMinY(-1.0f);
      boundingBoxMessage.setMaxZ(1.0f);
      boundingBoxMessage.setMinZ(-2.0f);

      reaMessager.submitMessage(REAModuleAPI.LidarBufferEnable, false);
      reaMessager.submitMessage(REAModuleAPI.StereoVisionBufferEnable, true);
      reaMessager.submitMessage(REAModuleAPI.DepthCloudBufferEnable, false);
      reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxEnable, false);
      reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxParameters, boundingBoxMessage);

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      reaMessager.submitMessage(REAModuleAPI.NormalEstimationParameters, normalEstimationParameters);

      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
      planarRegionSegmentationParameters.setMaxDistanceFromPlane(0.03);
      planarRegionSegmentationParameters.setMaxAngleFromPlane(Math.toRadians(10.0));
      planarRegionSegmentationParameters.setMinRegionSize(150);
      reaMessager.submitMessage(REAModuleAPI.PlanarRegionsSegmentationParameters, planarRegionSegmentationParameters);

      SurfaceNormalFilterParameters surfaceNormalFilterParameters = new SurfaceNormalFilterParameters();
      surfaceNormalFilterParameters.setUseSurfaceNormalFilter(true);
      surfaceNormalFilterParameters.setSurfaceNormalLowerBound(Math.toRadians(-40.0));
      surfaceNormalFilterParameters.setSurfaceNormalUpperBound(Math.toRadians(40.0));
      reaMessager.submitMessage(REAModuleAPI.SurfaceNormalFilterParameters, surfaceNormalFilterParameters);

      PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
      polygonizerParameters.setConcaveHullThreshold(0.15);
      reaMessager.submitMessage(REAModuleAPI.PlanarRegionsPolygonizerParameters, polygonizerParameters);
   }

   public void setParametersForDepth()
   {
      BoundingBoxParametersMessage boundingBoxMessage = new BoundingBoxParametersMessage();
      boundingBoxMessage.setMaxX(1.0f);
      boundingBoxMessage.setMinX(0.0f);
      boundingBoxMessage.setMaxY(1.0f);
      boundingBoxMessage.setMinY(-1.0f);
      boundingBoxMessage.setMaxZ(1.0f);
      boundingBoxMessage.setMinZ(-2.0f);

      reaMessager.submitMessage(REAModuleAPI.LidarBufferEnable, false);
      reaMessager.submitMessage(REAModuleAPI.StereoVisionBufferEnable, false);
      reaMessager.submitMessage(REAModuleAPI.DepthCloudBufferEnable, true);
      reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxEnable, false);
      reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxParameters, boundingBoxMessage);
   }

   public void loadConfigurationsFromFile()
   {
      loadConfigurationFile(filePropertyHelper);
   }

   public void stop()
   {
      LogTools.info("REA Module is going down.");

      try
      {
         reaMessager.closeMessager();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
      if (manageRosNode)
         networkProvider.stop();

      if (scheduled != null)
      {
         scheduled.cancel(true);
         scheduled = null;
      }

      if (executorService != null)
      {
         executorService.shutdownNow();
         executorService = null;
      }
   }

   public static LIDARBasedREAModule createRemoteModule(FilePropertyHelper filePropertyHelper, REANetworkProvider networkProvider) throws Exception
   {
      KryoMessager server = createKryoMessager(NetworkPorts.REA_MODULE_UI_PORT);
      return new LIDARBasedREAModule(server, filePropertyHelper, networkProvider);
   }

   public static LIDARBasedREAModule createIntraprocessModule(FilePropertyHelper filePropertyHelper, REANetworkProvider networkProvider) throws Exception
   {
      return createIntraprocessModule(filePropertyHelper, networkProvider, NetworkPorts.REA_MODULE_UI_PORT);
   }

   public static LIDARBasedREAModule createIntraprocessModule(FilePropertyHelper filePropertyHelper, REANetworkProvider networkProvider, NetworkPorts networkPorts)
         throws Exception
   {
      KryoMessager messager = createKryoMessager(networkPorts);

      return new LIDARBasedREAModule(messager, filePropertyHelper, networkProvider);
   }

   private static KryoMessager createKryoMessager(NetworkPorts networkPorts) throws Exception
   {
      KryoMessager messager = KryoMessager.createIntraprocess(REAModuleAPI.API, networkPorts, REACommunicationProperties.getPrivateNetClassList());
      messager.setAllowSelfSubmit(true);
      messager.startMessager();
      return messager;
   }
}
