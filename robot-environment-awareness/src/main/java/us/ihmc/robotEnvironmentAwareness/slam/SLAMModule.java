package us.ihmc.robotEnvironmentAwareness.slam;

import java.io.File;
import java.nio.file.Paths;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.REAStateRequestMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import org.apache.commons.math3.stat.descriptive.moment.Mean;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.converters.BoundingBoxMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.perceptionSuite.PerceptionModule;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.tools.thread.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.updaters.OcTreeConsumer;
import us.ihmc.ros2.ROS2Node;

public class SLAMModule implements PerceptionModule
{
   protected final Messager reaMessager;

   private static final double DEFAULT_OCTREE_RESOLUTION = 0.02;

   protected final AtomicReference<Boolean> enable;

   private final AtomicReference<StereoVisionPointCloudMessage> newPointCloud = new AtomicReference<>(null);
   protected final CopyOnWriteArrayList<StereoVisionPointCloudMessage> pointCloudQueue = new CopyOnWriteArrayList<>();

   private final AtomicLong mostRecentTimestampProcessed = new AtomicLong(-1);
   protected final AtomicReference<SurfaceElementICPSLAMParameters> slamParameters;
   private final AtomicReference<NormalEstimationParameters> normalEstimationParameters;
   private final AtomicReference<NormalEstimationParameters> frameNormalEstimationParameters;
   private final AtomicReference<Boolean> enableNormalEstimation;
   private final AtomicReference<Boolean> clearNormals;

   private final AtomicReference<Boolean> isOcTreeBoundingBoxRequested;
   private final AtomicReference<BoundingBoxParametersMessage> atomicBoundingBoxParameters;
   private final AtomicReference<Boolean> useBoundingBox;
   private final AtomicBoolean clearSlam = new AtomicBoolean(false);

   protected final SurfaceElementICPSLAM slam;

   private static final int SLAM_PERIOD_MILLISECONDS = 250;
   private static final int QUEUE_PERIOD_MILLISECONDS = 5;
   private static final int MAIN_PERIOD_MILLISECONDS = 200;

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(3, getClass(), ExceptionHandling.CATCH_AND_REPORT);
   private ScheduledFuture<?> scheduledQueue;
   private ScheduledFuture<?> scheduledMain;
   private ScheduledFuture<?> scheduledSLAM;

   private final Mean average = new Mean();
   private final Stopwatch totalStopWatch = new Stopwatch();
   private final Stopwatch updateStopwatch = new Stopwatch();

   private final boolean manageRosNode;
   protected final ROS2Node ros2Node;

   private final List<OcTreeConsumer> ocTreeConsumers = new ArrayList<>();

   private final SLAMHistory history = new SLAMHistory();
   private final AtomicReference<String> slamDataExportPath;

   public SLAMModule(Messager messager)
   {
      this(messager, null);
   }

   public SLAMModule(ROS2Node ros2Node, Messager messager)
   {
      this(ros2Node, messager, new RigidBodyTransform());
   }

   public SLAMModule(ROS2Node ros2Node, Messager messager, RigidBodyTransformReadOnly transformFromLocalFrameToSensor)
   {
      this(ros2Node, messager, transformFromLocalFrameToSensor, null);
   }

   public SLAMModule(Messager messager, File configurationFile)
   {
      this(ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME), messager, configurationFile, new RigidBodyTransform(), true);
   }

   public SLAMModule(ROS2Node ros2Node, Messager messager, File configurationFile)
   {
      this(ros2Node, messager, configurationFile, new RigidBodyTransform(), false);
   }

   public SLAMModule(ROS2Node ros2Node, Messager messager, RigidBodyTransformReadOnly transformFromLocalFrameToSensor, File configurationFile)
   {
      this(ros2Node, messager, configurationFile, transformFromLocalFrameToSensor, false);
   }

   public SLAMModule(ROS2Node ros2Node, Messager messager, File configurationFile, RigidBodyTransformReadOnly transformFromLocalFrameToSensor, boolean manageRosNode)
   {
      this.ros2Node = ros2Node;
      this.reaMessager = messager;
      this.manageRosNode = manageRosNode;

      slam = new SurfaceElementICPSLAM(DEFAULT_OCTREE_RESOLUTION, transformFromLocalFrameToSensor);

      // TODO: Check name space and fix. Suspected atlas sensor suite and publisher.
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           ROS2Tools.MULTISENSE_STEREO_POINT_CLOUD,
                                           this::handlePointCloud);
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           ROS2Tools.D435_POINT_CLOUD,
                                           this::handlePointCloud);
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           REAStateRequestMessage.class,
                                           REACommunicationProperties.stereoInputTopic,
                                           this::handleREAStateRequestMessage);
      new IHMCROS2Callback<>(ros2Node, SLAMModuleAPI.CLEAR, message -> clearSLAM());
      new IHMCROS2Callback<>(ros2Node, SLAMModuleAPI.SHUTDOWN, message ->
      {
         LogTools.info("Received SHUTDOWN. Shutting down...");
         stop();
      });

      reaMessager.submitMessage(SLAMModuleAPI.UISensorPoseHistoryFrames, 1000);

      enable = reaMessager.createInput(SLAMModuleAPI.SLAMEnable, true);

      slamParameters = reaMessager.createInput(SLAMModuleAPI.SLAMParameters, new SurfaceElementICPSLAMParameters());

      enableNormalEstimation = reaMessager.createInput(SLAMModuleAPI.NormalEstimationEnable, true);
      clearNormals = reaMessager.createInput(SLAMModuleAPI.NormalEstimationClear, false);

      reaMessager.registerTopicListener(SLAMModuleAPI.SLAMClear, (content) -> clearSLAM());
      reaMessager.registerTopicListener(SLAMModuleAPI.RequestEntireModuleState, update -> sendCurrentState());

      NormalEstimationParameters normalEstimationParametersLocal = new NormalEstimationParameters();
      normalEstimationParametersLocal.setNumberOfIterations(1);
      normalEstimationParameters = reaMessager.createInput(SLAMModuleAPI.NormalEstimationParameters, normalEstimationParametersLocal);

      NormalEstimationParameters frameNormalEstimationParametersLocal = new NormalEstimationParameters();
      frameNormalEstimationParametersLocal.setNumberOfIterations(10);

      frameNormalEstimationParameters = reaMessager.createInput(SLAMModuleAPI.FrameNormalEstimationParameters, frameNormalEstimationParametersLocal);


      isOcTreeBoundingBoxRequested = reaMessager.createInput(SLAMModuleAPI.RequestBoundingBox, false);

      useBoundingBox = reaMessager.createInput(SLAMModuleAPI.OcTreeBoundingBoxEnable, true);
      atomicBoundingBoxParameters = reaMessager.createInput(SLAMModuleAPI.OcTreeBoundingBoxParameters,
                                                            BoundingBoxMessageConverter.createBoundingBoxParametersMessage(-1.0f,
                                                                                                                           -1.5f,
                                                                                                                           -1.5f,
                                                                                                                           2.0f,
                                                                                                                           1.5f,
                                                                                                                           0.5f));

      reaMessager.submitMessage(SLAMModuleAPI.NormalEstimationParameters, normalEstimationParametersLocal);
      reaMessager.submitMessage(SLAMModuleAPI.FrameNormalEstimationParameters, frameNormalEstimationParametersLocal);

      if (configurationFile != null)
      {
         FilePropertyHelper filePropertyHelper = new FilePropertyHelper(configurationFile);
         loadConfiguration(filePropertyHelper);

         reaMessager.registerTopicListener(SLAMModuleAPI.SaveConfiguration, content -> saveConfiguration(filePropertyHelper));
      }
      slamDataExportPath = reaMessager.createInput(SLAMModuleAPI.UISLAMDataExportDirectory);
      reaMessager.registerTopicListener(SLAMModuleAPI.UISLAMDataExportRequest, content -> exportSLAMHistory());

      sendCurrentState();
   }

   private void exportSLAMHistory()
   {
      history.export(Paths.get(slamDataExportPath.get()));
   }

   public void attachOcTreeConsumer(OcTreeConsumer ocTreeConsumer)
   {
      this.ocTreeConsumers.add(ocTreeConsumer);
   }

   public void removeOcTreeConsumer(OcTreeConsumer ocTreeConsumer)
   {
      this.ocTreeConsumers.remove(ocTreeConsumer);
   }

   private void handleREAStateRequestMessage(Subscriber<REAStateRequestMessage> subscriber)
   {
      REAStateRequestMessage newMessage = subscriber.takeNextData();

      if (newMessage.getRequestResume())
         reaMessager.submitMessage(SLAMModuleAPI.SLAMEnable, true);
      else if (newMessage.getRequestPause()) // We guarantee to resume if requested, regardless of the pause request.
         reaMessager.submitMessage(SLAMModuleAPI.SLAMEnable, false);
      if (newMessage.getRequestClear())
         clearSlam.set(true);
   }

   @Override
   public void start()
   {
      LogTools.info("Starting SLAM Module");
      totalStopWatch.start();
      updateStopwatch.start();
      average.clear();
      if (scheduledMain == null)
         scheduledMain = executorService.scheduleAtFixedRate(this::updateMain, 0, MAIN_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      if (scheduledQueue == null)
         scheduledQueue = executorService.scheduleAtFixedRate(this::updateQueue, 0, QUEUE_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      if (scheduledSLAM == null)
         scheduledSLAM = executorService.scheduleAtFixedRate(this::updateSLAM, 0, SLAM_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
   }

   @Override
   public void stop()
   {
      LogTools.info("SLAM Module is going down");

      try
      {
         reaMessager.closeMessager();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      if (scheduledQueue != null)
      {
         scheduledQueue.cancel(true);
         scheduledQueue = null;
      }

      if (scheduledMain != null)
      {
         scheduledMain.cancel(true);
         scheduledMain = null;
      }

      if (manageRosNode)
         ros2Node.destroy();

      if (scheduledSLAM != null)
      {
         scheduledSLAM.cancel(true);
         scheduledSLAM = null;
      }

      if (executorService != null)
      {
         executorService.shutdownNow();
         executorService = null;
      }

      LogTools.info("Shutdown complete");
   }

   private boolean isMainThreadInterrupted()
   {
      return Thread.interrupted() || scheduledMain == null || scheduledMain.isCancelled();
   }

   private boolean isQueueThreadInterrupted()
   {
      return Thread.interrupted() || scheduledQueue == null || scheduledQueue.isCancelled();
   }

   private boolean isSLAMThreadInterrupted()
   {
      return Thread.interrupted() || scheduledSLAM == null || scheduledSLAM.isCancelled();
   }

   public void sendCurrentState()
   {
      reaMessager.submitMessage(SLAMModuleAPI.SLAMEnable, enable.get());
      reaMessager.submitMessage(SLAMModuleAPI.SLAMParameters, slamParameters.get());

      reaMessager.submitMessage(SLAMModuleAPI.NormalEstimationEnable, enableNormalEstimation.get());
      reaMessager.submitMessage(SLAMModuleAPI.NormalEstimationParameters, normalEstimationParameters.get());
      reaMessager.submitMessage(SLAMModuleAPI.FrameNormalEstimationParameters, frameNormalEstimationParameters.get());

      reaMessager.submitMessage(SLAMModuleAPI.OcTreeBoundingBoxEnable, useBoundingBox.get());
      reaMessager.submitMessage(SLAMModuleAPI.OcTreeBoundingBoxParameters, atomicBoundingBoxParameters.get());
   }

   public void updateMain()
   {
      if (isMainThreadInterrupted())
         return;

      if (clearSlam.getAndSet(false))
         reaMessager.submitMessage(SLAMModuleAPI.SLAMClear, true);

      slam.setNormalEstimationParameters(normalEstimationParameters.get());
      if (clearNormals.getAndSet(false))
         slam.clearNormals();
      if (enableNormalEstimation.get())
         slam.updateSurfaceNormals();
   }

   private void updateSLAM()
   {
      totalStopWatch.lap();
      updateStopwatch.lap();
      if (updateSLAMInternal())
      {
         publishResults();
      }

      if (isOcTreeBoundingBoxRequested.getAndSet(false))
         reaMessager.submitMessage(SLAMModuleAPI.OcTreeBoundingBoxState,
                                   BoundingBoxMessageConverter.convertToMessage(slam.getMapOcTree().getBoundingBox()));
   }

   private boolean updateSLAMInternal()
   {
      if (isSLAMThreadInterrupted())
         return false;

      if (pointCloudQueue.size() == 0)
         return false;

      updateAndCompressQueue();

      updateSLAMParameters();
      StereoVisionPointCloudMessage pointCloudToCompute = pointCloudQueue.get(0);
      slam.handleBoundingBox(pointCloudToCompute.getSensorPosition(), pointCloudToCompute.getSensorOrientation(), atomicBoundingBoxParameters.get(), useBoundingBox.get());
      mostRecentTimestampProcessed.set(pointCloudToCompute.getTimestamp());

      slam.setFrameNormalEstimationParameters(frameNormalEstimationParameters.get());

      boolean success;
      slam.setComputeInParallel(slamParameters.get().getComputeFramesInParallel());
      if (slam.isEmpty())
      {
         LogTools.debug("addKeyFrame queueSize: {} pointCloudSize: {} timestamp: {}",
                       pointCloudQueue.size(),
                       pointCloudToCompute.getNumberOfPoints(),
                       pointCloudToCompute.getTimestamp());
         slam.addKeyFrame(pointCloudToCompute, slamParameters.get().getInsertMissInOcTree());
         success = true;
      }
      else
      {
         LogTools.debug("addFrame queueSize: {} pointCloudSize: {}, timestamp: {}",
                       pointCloudQueue.size(),
                       pointCloudToCompute.getNumberOfPoints(),
                       pointCloudToCompute.getTimestamp());
         success = addFrame(pointCloudToCompute);
         LogTools.debug("success: {} getComputationTimeForLatestFrame: {}", success, slam.getComputationTimeForLatestFrame());
      }

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters(this.normalEstimationParameters.get());
      normalEstimationParameters.setNumberOfIterations(2 * this.normalEstimationParameters.get().getNumberOfIterations());
      if (enableNormalEstimation.get())
         slam.updateSurfaceNormalsInBoundingBox(normalEstimationParameters);
         
      dequeue();

      return success;
   }

   protected boolean addFrame(StereoVisionPointCloudMessage pointCloudToCompute)
   {
      return slam.addFrame(pointCloudToCompute, slamParameters.get().getInsertMissInOcTree());
   }

   protected void queue(StereoVisionPointCloudMessage pointCloud)
   {
      pointCloudQueue.add(pointCloud);
   }

   private void updateAndCompressQueue()
   {
      StereoVisionPointCloudMessage lastPointCloud = pointCloudQueue.get(pointCloudQueue.size() - 1);

      if (mostRecentTimestampProcessed.get() > -1 &&
          Conversions.nanosecondsToSeconds(lastPointCloud.getTimestamp() - mostRecentTimestampProcessed.get()) > slamParameters.get().getLongestTimeToLag())
      {
         compressQueueToSize(1);
      }

      compressQueueToSize(slamParameters.get().getMaximumQueueSize());
   }

   private void compressQueueToSize(int desiredSize)
   {
      long timestamp = mostRecentTimestampProcessed.get();
      int index = 0;
      while (index < pointCloudQueue.size() - 1 && pointCloudQueue.size() > desiredSize - 1)
      {
         StereoVisionPointCloudMessage pointCloudMessage = pointCloudQueue.get(index);
         StereoVisionPointCloudMessage nextPointCloudMessage = pointCloudQueue.get(index + 1);
         if (Conversions.nanosecondsToSeconds(nextPointCloudMessage.getTimestamp() - timestamp) < slamParameters.get().getMaximumTimeBetweenFrames())
         {
            pointCloudQueue.remove(index);
         }
         else
         {
            timestamp = pointCloudMessage.getTimestamp();
            index++;
         }
      }
   }

   protected void dequeue()
   {
      if (!pointCloudQueue.isEmpty())
         pointCloudQueue.remove(0);
   }

   private final DecimalFormat df = new DecimalFormat("#.##");

   protected void publishResults()
   {
      reaMessager.submitMessage(SLAMModuleAPI.QueuedBuffers, pointCloudQueue.size() + " [" + slam.getMapSize() + "]");

      String stringToReport = slam.getComputationTimeForLatestFrame() + " (sec) ";
      reaMessager.submitMessage(SLAMModuleAPI.FrameComputationTime, stringToReport);
      reaMessager.submitMessage(SLAMModuleAPI.SLAMComputationTime, df.format(updateStopwatch.lapElapsed()) + "(sec)");
      average.increment(updateStopwatch.lapElapsed());
      reaMessager.submitMessage(SLAMModuleAPI.AverageComputationTime, df.format(average.getResult()) + " (sec)");


      NormalOcTree octreeMap = slam.getMapOcTree();
      SLAMFrame latestFrame = slam.getLatestFrame();

      if (latestFrame == null)
      {
         LogTools.warn("Latest frame is null. Skipping publish results");
         return;
      }

      reaMessager.submitMessage(SLAMModuleAPI.SLAMOctreeMapState, OcTreeMessageConverter.convertToMessage(slam.getMapOcTree()));


      long startTime = System.nanoTime();
      Pose3D pose = new Pose3D(latestFrame.getCorrectedSensorPoseInWorld());

      for (OcTreeConsumer ocTreeConsumer : ocTreeConsumers)
      {
         ocTreeConsumer.reportOcTree(octreeMap, pose);
      }
      long endTime = System.nanoTime();
      stringToReport = df.format(Conversions.nanosecondsToSeconds(endTime - startTime)) + " (sec)";
      reaMessager.submitMessage(SLAMModuleAPI.ListenerComputationTime, stringToReport);

      Point3DReadOnly[] originalPointCloud = latestFrame.getUncorrectedPointCloudInWorld();
      List<? extends Point3DReadOnly> correctedPointCloud = latestFrame.getCorrectedPointCloudInWorld();
      Point3DReadOnly[] sourcePointsToWorld = slam.getSourcePoints();
      if (originalPointCloud == null || sourcePointsToWorld == null || correctedPointCloud == null)
         return;
      SLAMFrameState frameState = new SLAMFrameState();
      frameState.setUncorrectedPointCloudInWorld(originalPointCloud);
      frameState.setCorrectedPointCloudInWorld(correctedPointCloud);
      frameState.setCorrespondingPointsInWorld(sourcePointsToWorld);
      RigidBodyTransformReadOnly sensorPose = latestFrame.getCorrectedSensorPoseInWorld();
      frameState.getSensorPosition().set(sensorPose.getTranslation());
      frameState.getSensorOrientation().set(sensorPose.getRotation());
      reaMessager.submitMessage(SLAMModuleAPI.IhmcSLAMFrameState, frameState);
      reaMessager.submitMessage(SLAMModuleAPI.LatestFrameConfidenceFactor, latestFrame.getConfidenceFactor());
      history.addLatestFrameHistory(latestFrame);
      history.addDriftCorrectionHistory(slam.getDriftCorrectionResult());



      LogTools.debug("Took: {} ocTree size: {}", totalStopWatch.lapElapsed(), octreeMap.size());

      stringToReport = df.format(totalStopWatch.lapElapsed()) + " (sec)";
      reaMessager.submitMessage(SLAMModuleAPI.TotalComputationTime, stringToReport);
   }


   public void updateQueue()
   {
      if (isQueueThreadInterrupted())
         return;

      if (enable.get())
      {
         StereoVisionPointCloudMessage pointCloud = newPointCloud.getAndSet(null);
         if (pointCloud == null)
            return;

         queue(pointCloud);
      }
   }

   private void updateSLAMParameters()
   {
      slam.updateParameters(slamParameters.get());
   }

   public void clearSLAM()
   {
      LogTools.info("Clearing");
      newPointCloud.set(null);
      pointCloudQueue.clear();
      slam.clear();
      history.clearHistory();
   }

   public void loadConfiguration(FilePropertyHelper filePropertyHelper)
   {
      Boolean enableFile = filePropertyHelper.loadBooleanProperty(SLAMModuleAPI.SLAMEnable.getName());
      if (enableFile != null)
         enable.set(enableFile);
      Boolean enableNormalEstimationFile = filePropertyHelper.loadBooleanProperty(SLAMModuleAPI.NormalEstimationEnable.getName());
      if (enableNormalEstimationFile != null)
         enableNormalEstimation.set(enableNormalEstimationFile);
      String slamParametersFile = filePropertyHelper.loadProperty(SLAMModuleAPI.SLAMParameters.getName());
      if (slamParametersFile != null)
         slamParameters.set(SurfaceElementICPSLAMParameters.parse(slamParametersFile));
      String normalEstimationParametersFile = filePropertyHelper.loadProperty(SLAMModuleAPI.NormalEstimationParameters.getName());
      if (normalEstimationParametersFile != null)
         normalEstimationParameters.set(NormalEstimationParameters.parse(normalEstimationParametersFile));
      String frameNormalEstimationParametersFile = filePropertyHelper.loadProperty(SLAMModuleAPI.FrameNormalEstimationParameters.getName());
      if (frameNormalEstimationParametersFile != null)
         frameNormalEstimationParameters.set(NormalEstimationParameters.parse(frameNormalEstimationParametersFile));

      Boolean useBoundingBoxFile = filePropertyHelper.loadBooleanProperty(SLAMModuleAPI.OcTreeBoundingBoxEnable.getName());
      if (useBoundingBoxFile != null)
         useBoundingBox.set(useBoundingBoxFile);
      String boundingBoxParametersFile = filePropertyHelper.loadProperty(SLAMModuleAPI.OcTreeBoundingBoxParameters.getName());
      if (boundingBoxParametersFile != null)
         atomicBoundingBoxParameters.set(BoundingBoxMessageConverter.parse(boundingBoxParametersFile));
   }

   public void saveConfiguration(FilePropertyHelper filePropertyHelper)
   {
      filePropertyHelper.saveProperty(SLAMModuleAPI.SLAMEnable.getName(), enable.get());
      filePropertyHelper.saveProperty(SLAMModuleAPI.NormalEstimationEnable.getName(), enableNormalEstimation.get());

      filePropertyHelper.saveProperty(SLAMModuleAPI.SLAMParameters.getName(), slamParameters.get().toString());
      filePropertyHelper.saveProperty(SLAMModuleAPI.NormalEstimationParameters.getName(), normalEstimationParameters.get().toString());
      filePropertyHelper.saveProperty(SLAMModuleAPI.FrameNormalEstimationParameters.getName(), frameNormalEstimationParameters.get().toString());

      filePropertyHelper.saveProperty(SLAMModuleAPI.OcTreeBoundingBoxEnable.getName(), useBoundingBox.get());
      filePropertyHelper.saveProperty(SLAMModuleAPI.OcTreeBoundingBoxParameters.getName(), atomicBoundingBoxParameters.get().toString());
   }

   private void handlePointCloud(Subscriber<StereoVisionPointCloudMessage> subscriber)
   {
      StereoVisionPointCloudMessage message = subscriber.takeNextData();
      LogTools.trace("Received point cloud. numberOfPoints: {} timestamp: {}", message.getNumberOfPoints(), message.getTimestamp());
      newPointCloud.set(message);
      reaMessager.submitMessage(SLAMModuleAPI.DepthPointCloudState, new StereoVisionPointCloudMessage(message));
   }

   public static SLAMModule createIntraprocessModule(ROS2Node ros2Node, Messager messager)
   {
      return new SLAMModule(ros2Node, messager);
   }

   public static SLAMModule createIntraprocessModule(Messager messager)
   {
      return new SLAMModule(messager);
   }

}
