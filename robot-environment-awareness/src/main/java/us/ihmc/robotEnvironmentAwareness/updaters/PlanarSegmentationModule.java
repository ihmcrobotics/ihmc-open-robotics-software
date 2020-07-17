package us.ihmc.robotEnvironmentAwareness.updaters;

import com.google.common.util.concurrent.AtomicDouble;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.REAStateRequestMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.tools.JOctoMapTools;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.SegmentationModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.SurfaceNormalFilterParameters;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.Ros2Node;

import java.io.File;
import java.io.IOException;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class PlanarSegmentationModule implements OcTreeConsumer
{
   private static final String planarRegionsTimeReport = "OcTreePlanarRegion update took: ";
   private static final String reportPlanarRegionsStateTimeReport = "Reporting Planar Regions state took: ";

   private final TimeReporter timeReporter = new TimeReporter();

   private static final int THREAD_PERIOD_MILLISECONDS = 200;

   protected static final boolean DEBUG = true;

   private final Ros2Node ros2Node;

   private final REAPlanarRegionFeatureUpdater planarRegionFeatureUpdater;

   private final SegmentationModuleStateReporter moduleStateReporter;
   private final IHMCROS2Publisher<PlanarRegionsListMessage> planarRegionPublisher;

   private final AtomicReference<Boolean> clearOcTree;
   private final AtomicReference<NormalOcTree> ocTree = new AtomicReference<>(null);
   private final AtomicReference<Tuple3DReadOnly> sensorPosition;

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(3, getClass(), ExceptionHandling.CATCH_AND_REPORT);
   private ScheduledFuture<?> scheduled;
   private final Messager reaMessager;

   private PlanarSegmentationModule(Messager reaMessager, File configurationFile) throws Exception
   {
      this(ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME),
           REACommunicationProperties.inputTopic,
           REACommunicationProperties.subscriberCustomRegionsTopicName,
           ROS2Tools.REALSENSE_SLAM_MAP,
           reaMessager,
           configurationFile);
   }

   private PlanarSegmentationModule(ROS2Topic<?> inputTopic,
                                    ROS2Topic<?> customRegionTopic,
                                    ROS2Topic<?> outputTopic,
                                    Messager reaMessager,
                                    File configurationFile) throws Exception
   {
     this(ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME),
          inputTopic,
          customRegionTopic,
          outputTopic,
          reaMessager,
          configurationFile);
   }

   private PlanarSegmentationModule(Ros2Node ros2Node,
                                    ROS2Topic<?> inputTopic,
                                    ROS2Topic<?> customRegionTopic,
                                    ROS2Topic<?> outputTopic,
                                    Messager reaMessager,
                                    File configurationFile) throws Exception
   {
      this.ros2Node = ros2Node;
      this.reaMessager = reaMessager;
      if (!reaMessager.isMessagerOpen())
         reaMessager.startMessager();

      moduleStateReporter = new SegmentationModuleStateReporter(reaMessager);

      planarRegionFeatureUpdater = new REAPlanarRegionFeatureUpdater(reaMessager, SegmentationModuleAPI.RequestEntireModuleState);
      planarRegionFeatureUpdater.setOcTreeEnableTopic(SegmentationModuleAPI.OcTreeEnable);
      planarRegionFeatureUpdater.setPlanarRegionSegmentationEnableTopic(SegmentationModuleAPI.PlanarRegionsSegmentationEnable);
      planarRegionFeatureUpdater.setPlanarRegionSegmentationClearTopic(SegmentationModuleAPI.PlanarRegionsSegmentationClear);
      planarRegionFeatureUpdater.setCustomRegionsMergingEnableTopic(SegmentationModuleAPI.CustomRegionsMergingEnable);
      planarRegionFeatureUpdater.setCustomRegionsClearTopic(SegmentationModuleAPI.CustomRegionsClear);
      planarRegionFeatureUpdater.setPlanarRegionsPolygonizerEnableTopic(SegmentationModuleAPI.PlanarRegionsPolygonizerEnable);
      planarRegionFeatureUpdater.setPlanarRegionsPolygonizerClearTopic(SegmentationModuleAPI.PlanarRegionsPolygonizerClear);
      planarRegionFeatureUpdater.setPlanarRegionsIntersectionEnableTopic(SegmentationModuleAPI.PlanarRegionsIntersectionEnable);
      planarRegionFeatureUpdater.setPlanarRegionSegmentationParametersTopic(SegmentationModuleAPI.PlanarRegionsSegmentationParameters);
      planarRegionFeatureUpdater.setCustomRegionMergeParametersTopic(SegmentationModuleAPI.CustomRegionsMergingParameters);
      planarRegionFeatureUpdater.setPlanarRegionsConcaveHullFactoryParametersTopic(SegmentationModuleAPI.PlanarRegionsConcaveHullParameters);
      planarRegionFeatureUpdater.setPlanarRegionsPolygonizerParametersTopic(SegmentationModuleAPI.PlanarRegionsPolygonizerParameters);
      planarRegionFeatureUpdater.setPlanarRegionsIntersectionParametersTopic(SegmentationModuleAPI.PlanarRegionsIntersectionParameters);
      planarRegionFeatureUpdater.setSurfaceNormalFilterParametersTopic(SegmentationModuleAPI.SurfaceNormalFilterParameters);
      planarRegionFeatureUpdater.bindControls();

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    PlanarRegionsListMessage.class,
                                                    customRegionTopic,
                                                    this::dispatchCustomPlanarRegion);
      // TODO what the heck is this used for?
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, REAStateRequestMessage.class, inputTopic, this::handleREAStateRequestMessage);

      planarRegionPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, PlanarRegionsListMessage.class, outputTopic);

      FilePropertyHelper filePropertyHelper = new FilePropertyHelper(configurationFile);
      loadConfigurationFile(filePropertyHelper);

      reaMessager.registerTopicListener(SegmentationModuleAPI.SaveUpdaterConfiguration,
                                        (content) -> planarRegionFeatureUpdater.saveConfiguration(filePropertyHelper));

      clearOcTree = reaMessager.createInput(SegmentationModuleAPI.OcTreeClear, false);
      sensorPosition = reaMessager.createInput(SegmentationModuleAPI.SensorPosition, null);

      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
      planarRegionSegmentationParameters.setMaxDistanceFromPlane(0.03);
      planarRegionSegmentationParameters.setMinRegionSize(150);
      planarRegionFeatureUpdater.setPlanarRegionSegmentationParameters(planarRegionSegmentationParameters);

      SurfaceNormalFilterParameters surfaceNormalFilterParameters = new SurfaceNormalFilterParameters();
      surfaceNormalFilterParameters.setUseSurfaceNormalFilter(true);
      surfaceNormalFilterParameters.setSurfaceNormalLowerBound(Math.toRadians(-40.0));
      surfaceNormalFilterParameters.setSurfaceNormalUpperBound(Math.toRadians(40.0));
      planarRegionFeatureUpdater.setSurfaceNormalFilterParameters(surfaceNormalFilterParameters);

      PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
      polygonizerParameters.setConcaveHullThreshold(0.15);
      planarRegionFeatureUpdater.setPolygonizerParameters(polygonizerParameters);

      reaMessager.submitMessage(SegmentationModuleAPI.UIOcTreeDisplayType, OcTreeMeshBuilder.DisplayType.PLANE);
      reaMessager.submitMessage(SegmentationModuleAPI.UIOcTreeColoringMode, OcTreeMeshBuilder.ColoringType.REGION);

      // At the very end, we force the modules to submit their state so duplicate inputs have consistent values.
      reaMessager.submitMessage(SegmentationModuleAPI.RequestEntireModuleState, true);
   }

   public void reportOcTree(NormalOcTree ocTree, Tuple3DReadOnly sensorPosition)
   {
      this.ocTree.set(ocTree);

      reaMessager.submitMessage(SegmentationModuleAPI.OcTreeState, OcTreeMessageConverter.convertToMessage(ocTree));
      reaMessager.submitMessage(SegmentationModuleAPI.SensorPosition, sensorPosition);
   }

   private void dispatchCustomPlanarRegion(Subscriber<PlanarRegionsListMessage> subscriber)
   {
      PlanarRegionsListMessage message = subscriber.takeNextData();
      PlanarRegionsList customPlanarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(message);
      customPlanarRegions.getPlanarRegionsAsList().forEach(planarRegionFeatureUpdater::registerCustomPlanarRegion);
   }

   private void handleREAStateRequestMessage(Subscriber<REAStateRequestMessage> subscriber)
   {
      REAStateRequestMessage newMessage = subscriber.takeNextData();

      if (newMessage.getRequestResume())
         reaMessager.submitMessage(SegmentationModuleAPI.OcTreeEnable, true);
      else if (newMessage.getRequestPause()) // We guarantee to resume if requested, regardless of the pause request.
         reaMessager.submitMessage(SegmentationModuleAPI.OcTreeEnable, false);
      if (newMessage.getRequestClear())
         clearOcTree.set(true);
   }

   private void loadConfigurationFile(FilePropertyHelper filePropertyHelper)
   {
      planarRegionFeatureUpdater.loadConfiguration(filePropertyHelper);
   }

   private final AtomicDouble lastCompleteUpdate = new AtomicDouble(Double.NaN);

   private void mainUpdate()
   {
      if (isThreadInterrupted())
         return;

      try
      {
         NormalOcTree latestOcTree = ocTree.getAndSet(null);
         Tuple3DReadOnly latestSensorPose = this.sensorPosition.getAndSet(null);
         if (clearOcTree.getAndSet(false))
         {
            planarRegionFeatureUpdater.clearOcTree();
         }
         else if (latestOcTree != null)
         {
            if (isThreadInterrupted())
               return;

            Point3D sensorPose = new Point3D(latestSensorPose);
            NormalOcTree mainOcTree = new NormalOcTree(latestOcTree);

            timeReporter.run(() -> planarRegionFeatureUpdater.update(mainOcTree, sensorPose), planarRegionsTimeReport);
            reaMessager.submitMessage(SegmentationModuleAPI.UISegmentationDuration, timeReporter.getStringToReport());

            timeReporter.run(() -> moduleStateReporter.reportPlanarRegionsState(planarRegionFeatureUpdater), reportPlanarRegionsStateTimeReport);


            planarRegionPublisher.publish(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionFeatureUpdater.getPlanarRegionsList()));
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

      double currentTime = JOctoMapTools.nanoSecondsToSeconds(System.nanoTime());
      lastCompleteUpdate.set(currentTime);
   }

   private boolean isThreadInterrupted()
   {
      return Thread.interrupted() || scheduled == null || scheduled.isCancelled();
   }

   public void start()
   {
      LogTools.info("Planar segmentation is starting.");

      if (scheduled == null)
      {
         scheduled = executorService.scheduleAtFixedRate(this::mainUpdate, 0, THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }
   }

   public void stop()
   {
      LogTools.info("Planar segmentation is going down.");

      try
      {
         reaMessager.closeMessager();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
      ros2Node.destroy();

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

   public static PlanarSegmentationModule createRemoteModule(String configurationFilePath) throws Exception
   {
      KryoMessager messager = KryoMessager.createTCPServer(SegmentationModuleAPI.API,
                                                                    NetworkPorts.PLANAR_SEGMENTATION_UI_PORT,
                                                                    REACommunicationProperties.getPrivateNetClassList());
      messager.setAllowSelfSubmit(true);
      return new PlanarSegmentationModule(messager, new File(configurationFilePath));
   }

   public static PlanarSegmentationModule createIntraprocessModule(ROS2Topic<?> inputTopic,
                                                                   ROS2Topic<?> customRegionTopic,
                                                                   ROS2Topic<?> outputTopic,
                                                                   String configurationFilePath) throws Exception
   {
      KryoMessager moduleMessager = KryoMessager.createIntraprocess(SegmentationModuleAPI.API,
                                                                    NetworkPorts.PLANAR_SEGMENTATION_UI_PORT,
                                                                    REACommunicationProperties.getPrivateNetClassList());
      moduleMessager.setAllowSelfSubmit(true);

      File configurationFile = new File(configurationFilePath);
      try
      {
         configurationFile.getParentFile().mkdirs();
         configurationFile.createNewFile();
      }
      catch (IOException e)
      {
         System.out.println(configurationFile.getAbsolutePath());
         e.printStackTrace();
      }

      return new PlanarSegmentationModule(inputTopic, customRegionTopic, outputTopic, moduleMessager, configurationFile);
   }
}
