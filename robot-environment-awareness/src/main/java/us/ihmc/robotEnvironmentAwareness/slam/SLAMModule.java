package us.ihmc.robotEnvironmentAwareness.slam;

import java.io.File;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.paint.Color;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudCompression;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.perceptionSuite.PerceptionModule;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.StereoVisionPointCloudViewer;
import us.ihmc.robotEnvironmentAwareness.updaters.OcTreeConsumer;
import us.ihmc.ros2.Ros2Node;

public class SLAMModule implements PerceptionModule
{
   protected final Messager reaMessager;

   private static final double DEFAULT_OCTREE_RESOLUTION = 0.02;

   private static final Color LATEST_ORIGINAL_POINT_CLOUD_COLOR = Color.BEIGE;
   private static final Color SOURCE_POINT_CLOUD_COLOR = Color.BLACK;
   private static final Color LATEST_POINT_CLOUD_COLOR = Color.LIME;

   protected final AtomicReference<Boolean> enable;

   private final AtomicReference<StereoVisionPointCloudMessage> newPointCloud = new AtomicReference<>(null);
   protected final LinkedList<StereoVisionPointCloudMessage> pointCloudQueue = new LinkedList<>();

   private final AtomicReference<RandomICPSLAMParameters> slamParameters;
   private final AtomicReference<NormalEstimationParameters> normalEstimationParameters;
   private final AtomicReference<Boolean> enableNormalEstimation;
   private final AtomicReference<Boolean> clearNormals;
   protected final RandomICPSLAM slam = new RandomICPSLAM(DEFAULT_OCTREE_RESOLUTION);

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(2, getClass(), ExceptionHandling.CATCH_AND_REPORT);
   private static final int THREAD_PERIOD_MILLISECONDS = 1;
   private ScheduledFuture<?> scheduledMain;
   private ScheduledFuture<?> scheduledSLAM;

   private final boolean manageRosNode;
   protected final Ros2Node ros2Node;

   private final List<OcTreeConsumer> ocTreeConsumers = new ArrayList<>();

   public SLAMModule(Messager messager)
   {
      this(messager, null);
   }

   public SLAMModule(Ros2Node ros2Node, Messager messager)
   {
      this(ros2Node, messager, null);
   }

   public SLAMModule(Messager messager, File configurationFile)
   {
      this(ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME), messager, configurationFile, true);
   }

   public SLAMModule(Ros2Node ros2Node, Messager messager, File configurationFile)
   {
      this(ros2Node, messager, configurationFile, false);
   }

   public SLAMModule(Ros2Node ros2Node, Messager messager, File configurationFile, boolean manageRosNode)
   {
      this.ros2Node = ros2Node;
      this.reaMessager = messager;
      this.manageRosNode = manageRosNode;

      // TODO: Check name space and fix. Suspected atlas sensor suite and publisher.
      ROS2Tools.createCallbackSubscription(ros2Node, StereoVisionPointCloudMessage.class, "/ihmc/stereo_vision_point_cloud", this::handlePointCloud);
      ROS2Tools.createCallbackSubscription(ros2Node, StereoVisionPointCloudMessage.class, "/ihmc/stereo_vision_point_cloud_D435", this::handlePointCloud);

      reaMessager.submitMessage(SLAMModuleAPI.UISensorPoseHistoryFrames, 1000);

      enable = reaMessager.createInput(SLAMModuleAPI.SLAMEnable, true);

      slamParameters = reaMessager.createInput(SLAMModuleAPI.SLAMParameters, new RandomICPSLAMParameters());

      enableNormalEstimation = reaMessager.createInput(SLAMModuleAPI.NormalEstimationEnable, true);
      clearNormals = reaMessager.createInput(SLAMModuleAPI.NormalEstimationClear, false);

      reaMessager.registerTopicListener(SLAMModuleAPI.SLAMClear, (content) -> clearSLAM());

      reaMessager.registerTopicListener(SLAMModuleAPI.RequestEntireModuleState, update -> sendCurrentState());

      NormalEstimationParameters normalEstimationParametersLocal = new NormalEstimationParameters();
      normalEstimationParametersLocal.setNumberOfIterations(10);
      normalEstimationParameters = reaMessager.createInput(SLAMModuleAPI.NormalEstimationParameters, normalEstimationParametersLocal);

      reaMessager.submitMessage(SLAMModuleAPI.NormalEstimationParameters, normalEstimationParametersLocal);

      if (configurationFile != null)
      {
         FilePropertyHelper filePropertyHelper = new FilePropertyHelper(configurationFile);
         loadConfiguration(filePropertyHelper);

         reaMessager.registerTopicListener(SLAMModuleAPI.SaveConfiguration, content -> saveConfiguration(filePropertyHelper));
      }

      sendCurrentState();
   }

   public void attachOcTreeConsumer(OcTreeConsumer ocTreeConsumer)
   {
      this.ocTreeConsumers.add(ocTreeConsumer);
   }

   public void removeOcTreeConsumer(OcTreeConsumer ocTreeConsumer)
   {
      this.ocTreeConsumers.remove(ocTreeConsumer);
   }

   @Override
   public void start()
   {
      LogTools.info("Starting SLAM Module");
      if (scheduledMain == null)
      {
         scheduledMain = executorService.scheduleAtFixedRate(this::updateMain, 0, THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }

      if (scheduledSLAM == null)
      {
         scheduledSLAM = executorService.scheduleAtFixedRate(this::updateSLAM, 0, THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }
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
   }

   private boolean isMainThreadInterrupted()
   {
      return Thread.interrupted() || scheduledMain == null || scheduledMain.isCancelled();
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
   }

   public void updateSLAM()
   {
      if (updateSLAMInternal())
      {
         publishResults();
      }
   }

   private boolean updateSLAMInternal()
   {
      if (isSLAMThreadInterrupted())
         return false;

      if (pointCloudQueue.size() == 0)
         return false;

      updateSLAMParameters();
      StereoVisionPointCloudMessage pointCloudToCompute = pointCloudQueue.getFirst();

      boolean success;
      if (slam.isEmpty())
      {
         slam.addKeyFrame(pointCloudToCompute);
         success = true;
      }
      else
      {
         success = addFrame(pointCloudToCompute);
      }

      slam.setNormalEstimationParameters(normalEstimationParameters.get());
      if (clearNormals.getAndSet(false))
         slam.clearNormals();
      if (enableNormalEstimation.get())
         slam.computeOcTreeNormals();

      dequeue();

      return success;
   }

   protected boolean addFrame(StereoVisionPointCloudMessage pointCloudToCompute)
   {
      return slam.addFrame(pointCloudToCompute);
   }

   protected void queue(StereoVisionPointCloudMessage pointCloud)
   {
      pointCloudQueue.add(pointCloud);
   }

   protected void dequeue()
   {
      pointCloudQueue.removeFirst();
   }

   protected void publishResults()
   {     
      String stringToReport = "";
      reaMessager.submitMessage(SLAMModuleAPI.QueuedBuffers, pointCloudQueue.size() + " [" + slam.getMapSize() + "]");
      stringToReport = stringToReport + " " + slam.getMapSize() + " " + slam.getComputationTimeForLatestFrame() + " (sec) ";
      reaMessager.submitMessage(SLAMModuleAPI.SLAMStatus, stringToReport);

      NormalOcTree octreeMap = slam.getOctree();
      NormalOcTreeMessage octreeMessage = OcTreeMessageConverter.convertToMessage(octreeMap);

      reaMessager.submitMessage(SLAMModuleAPI.SLAMOctreeMapState, octreeMessage);
      
      for (OcTreeConsumer ocTreeConsumer : ocTreeConsumers)
      {
         ocTreeConsumer.reportOcTree(octreeMap, slam.getLatestFrame().getSensorPose().getTranslation());
      }

      SLAMFrame latestFrame = slam.getLatestFrame();
      Point3DReadOnly[] originalPointCloud = latestFrame.getOriginalPointCloud();
      Point3DReadOnly[] correctedPointCloud = latestFrame.getPointCloud();
      Point3DReadOnly[] sourcePointsToWorld = slam.getSourcePointsToWorldLatestFrame();
      if (originalPointCloud == null || sourcePointsToWorld == null || correctedPointCloud == null)
         return;
      StereoVisionPointCloudMessage latestStereoMessage = createLatestFrameStereoVisionPointCloudMessage(originalPointCloud,
                                                                                                         sourcePointsToWorld,
                                                                                                         correctedPointCloud);
      RigidBodyTransformReadOnly sensorPose = latestFrame.getSensorPose();
      latestStereoMessage.getSensorPosition().set(sensorPose.getTranslation());
      latestStereoMessage.getSensorOrientation().set(sensorPose.getRotation());
      reaMessager.submitMessage(SLAMModuleAPI.IhmcSLAMFrameState, latestStereoMessage);
      reaMessager.submitMessage(SLAMModuleAPI.LatestFrameConfidenceFactor, latestFrame.getConfidenceFactor());
   }

   private StereoVisionPointCloudMessage createLatestFrameStereoVisionPointCloudMessage(Point3DReadOnly[] originalPointCloud,
                                                                                        Point3DReadOnly[] sourcePointsToWorld,
                                                                                        Point3DReadOnly[] correctedPointCloud)
   {
      int numberOfPointsToPack = originalPointCloud.length + sourcePointsToWorld.length + correctedPointCloud.length;

      Point3D[] pointCloudBuffer = new Point3D[numberOfPointsToPack];
      int[] colorBuffer = new int[numberOfPointsToPack];
      for (int i = 0; i < originalPointCloud.length; i++)
      {
         pointCloudBuffer[i] = new Point3D(originalPointCloud[i]);
         colorBuffer[i] = StereoVisionPointCloudViewer.colorToInt(LATEST_ORIGINAL_POINT_CLOUD_COLOR);
      }
      for (int i = originalPointCloud.length; i < originalPointCloud.length + sourcePointsToWorld.length; i++)
      {
         pointCloudBuffer[i] = new Point3D(sourcePointsToWorld[i - originalPointCloud.length]);
         colorBuffer[i] = StereoVisionPointCloudViewer.colorToInt(SOURCE_POINT_CLOUD_COLOR);
      }
      for (int i = originalPointCloud.length + sourcePointsToWorld.length; i < numberOfPointsToPack; i++)
      {
         pointCloudBuffer[i] = new Point3D(correctedPointCloud[i - originalPointCloud.length - sourcePointsToWorld.length]);
         colorBuffer[i] = StereoVisionPointCloudViewer.colorToInt(LATEST_POINT_CLOUD_COLOR);
      }
      return PointCloudCompression.compressPointCloud(19870612L, pointCloudBuffer, colorBuffer, numberOfPointsToPack, 0.001, null);
   }

   public void updateMain()
   {
      if (isMainThreadInterrupted())
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
      RandomICPSLAMParameters parameters = slamParameters.get();
      slam.updateParameters(parameters);
   }

   public void clearSLAM()
   {
      newPointCloud.set(null);
      pointCloudQueue.clear();
      slam.clear();
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
         slamParameters.set(RandomICPSLAMParameters.parse(slamParametersFile));
      String normalEstimationParametersFile = filePropertyHelper.loadProperty(SLAMModuleAPI.NormalEstimationParameters.getName());
      if (normalEstimationParametersFile != null)
         normalEstimationParameters.set(NormalEstimationParameters.parse(normalEstimationParametersFile));
   }

   public void saveConfiguration(FilePropertyHelper filePropertyHelper)
   {
      filePropertyHelper.saveProperty(SLAMModuleAPI.SLAMEnable.getName(), enable.get());
      filePropertyHelper.saveProperty(SLAMModuleAPI.NormalEstimationEnable.getName(), enableNormalEstimation.get());

      filePropertyHelper.saveProperty(SLAMModuleAPI.SLAMParameters.getName(), slamParameters.get().toString());
      filePropertyHelper.saveProperty(SLAMModuleAPI.NormalEstimationParameters.getName(), normalEstimationParameters.get().toString());
   }

   private void handlePointCloud(Subscriber<StereoVisionPointCloudMessage> subscriber)
   {
      StereoVisionPointCloudMessage message = subscriber.takeNextData();
      newPointCloud.set(message);
      reaMessager.submitMessage(SLAMModuleAPI.DepthPointCloudState, new StereoVisionPointCloudMessage(message));
   }

   public static SLAMModule createIntraprocessModule(Ros2Node ros2Node, Messager messager)
   {
      return new SLAMModule(ros2Node, messager);
   }

   public static SLAMModule createIntraprocessModule(Messager messager)
   {
      return new SLAMModule(messager);
   }

}
