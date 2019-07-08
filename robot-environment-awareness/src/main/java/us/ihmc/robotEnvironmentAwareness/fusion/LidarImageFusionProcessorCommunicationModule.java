package us.ihmc.robotEnvironmentAwareness.fusion;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.subscriberCustomRegionsTopicNameGenerator;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import boofcv.struct.calib.IntrinsicParameters;
import controller_msgs.msg.dds.ImageMessage;
import controller_msgs.msg.dds.IntrinsicParametersMessage;
import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import controller_msgs.msg.dds.VideoPacket;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.fusion.objectDetection.FusionSensorObjectDetectionManager;
import us.ihmc.robotEnvironmentAwareness.fusion.objectDetection.ObjectType;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.ImageVisualizationHelper;
import us.ihmc.robotEnvironmentAwareness.updaters.REAModuleStateReporter;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

public class LidarImageFusionProcessorCommunicationModule
{
   private final Messager messager;

   private final Ros2Node ros2Node;
   private final REAModuleStateReporter moduleStateReporter;
   private final StereoREAModule stereoREAModule;

   private final FusionSensorObjectDetectionManager objectDetectionManager;

   private final AtomicReference<String> socketHostIPAddress;
   private final AtomicReference<BufferedImage> latestBufferedImage = new AtomicReference<>(null);
   private final AtomicReference<List<ObjectType>> selectedObjecTypes;

   private final JPEGDecompressor jpegDecompressor = new JPEGDecompressor();

   private final ExceptionHandlingThreadScheduler scheduler;
   private static final int BUFFER_THREAD_PERIOD_MILLISECONDS = 1500;

   private LidarImageFusionProcessorCommunicationModule(Ros2Node ros2Node, Messager reaMessager, SharedMemoryJavaFXMessager messager)
   {
      this.messager = messager;
      this.ros2Node = ros2Node;

      moduleStateReporter = new REAModuleStateReporter(reaMessager);
      stereoREAModule = new StereoREAModule(ros2Node, reaMessager, messager);

      new ROS2Callback<>(ros2Node, LidarScanMessage.class, this::dispatchLidarScanMessage);
      new ROS2Callback<>(ros2Node, StereoVisionPointCloudMessage.class, this::dispatchStereoVisionPointCloudMessage);
      new ROS2Callback<>(ros2Node, ImageMessage.class, this::dispatchImageMessage);
      new ROS2Callback<>(ros2Node, VideoPacket.class, this::dispatchVideoPacket);
      new ROS2Callback<>(ros2Node, PlanarRegionsListMessage.class, this::dispatchCustomPlanarRegion);

      ROS2Tools.createCallbackSubscription(ros2Node, PlanarRegionsListMessage.class, subscriberCustomRegionsTopicNameGenerator,
                                           this::dispatchCustomPlanarRegion);

      objectDetectionManager = new FusionSensorObjectDetectionManager(ros2Node, messager);

      messager.registerTopicListener(LidarImageFusionAPI.RequestSocketConnection, (content) -> connect());
      messager.registerTopicListener(LidarImageFusionAPI.RequestObjectDetection, (content) -> request());
      selectedObjecTypes = messager.createInput(LidarImageFusionAPI.SelectedObjecTypes, new ArrayList<ObjectType>());
      socketHostIPAddress = messager.createInput(LidarImageFusionAPI.ObjectDetectionModuleAddress);

      messager.registerTopicListener(LidarImageFusionAPI.RunStereoREA, (content) -> stereoREAModule.singleRun());

      scheduler = new ExceptionHandlingThreadScheduler(this.getClass().getSimpleName(), t -> {
         LogTools.error(t.getMessage());
         t.printStackTrace();
         LogTools.error("{} is crashing due to an exception.", Thread.currentThread().getName());
      });
   }

   private void connect()
   {
      objectDetectionManager.connectExternalModule(socketHostIPAddress.get());
   }

   private void request()
   {
      objectDetectionManager.requestObjectDetection(latestBufferedImage.getAndSet(null), selectedObjecTypes.get());
   }

   private void dispatchCustomPlanarRegion(Subscriber<PlanarRegionsListMessage> subscriber)
   {
      PlanarRegionsListMessage message = subscriber.takeNextData();
      PlanarRegionsList customPlanarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(message);
      customPlanarRegions.getPlanarRegionsAsList().forEach(stereoREAModule::registerCustomPlanarRegion);
   }

   private void dispatchCustomPlanarRegion(PlanarRegionsListMessage message)
   {
      stereoREAModule.dispatchCustomPlanarRegion(message);
   }

   private void dispatchLidarScanMessage(LidarScanMessage message)
   {
      moduleStateReporter.registerLidarScanMessage(message);
   }

   private void dispatchStereoVisionPointCloudMessage(StereoVisionPointCloudMessage message)
   {
      moduleStateReporter.registerStereoVisionPointCloudMessage(message);
      objectDetectionManager.updateLatestStereoVisionPointCloudMessage(message);
      stereoREAModule.updateLatestStereoVisionPointCloudMessage(message);
   }

   private void dispatchImageMessage(ImageMessage message)
   {
      if (messager.isMessagerOpen())
         messager.submitMessage(LidarImageFusionAPI.ImageState, new ImageMessage(message));

      latestBufferedImage.set(ImageVisualizationHelper.convertImageMessageToBufferedImage(message));
      stereoREAModule.updateLatestBufferedImage(latestBufferedImage.get());
   }

   private void dispatchVideoPacket(VideoPacket message)
   {
      BufferedImage bufferedImage = jpegDecompressor.decompressJPEGDataToBufferedImage(message.getData().toArray());
      stereoREAModule.updateLatestBufferedImage(bufferedImage);
      latestBufferedImage.set(bufferedImage);
      messager.submitMessage(LidarImageFusionAPI.CameraPositionState, message.getPosition());
      messager.submitMessage(LidarImageFusionAPI.CameraOrientationState, message.getOrientation());
      messager.submitMessage(LidarImageFusionAPI.CameraIntrinsicParametersState, toIntrinsicParameters(message.getIntrinsicParameters()));
   }

   public void start() throws IOException
   {
      scheduler.schedule(stereoREAModule, BUFFER_THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
   }

   public void stop() throws Exception
   {
      messager.closeMessager();
      ros2Node.destroy();
      objectDetectionManager.close();
      scheduler.shutdown();
   }

   public static LidarImageFusionProcessorCommunicationModule createIntraprocessModule(Ros2Node ros2Node, SharedMemoryJavaFXMessager messager)
         throws IOException
   {
      KryoMessager kryoMessager = KryoMessager.createIntraprocess(REAModuleAPI.API, NetworkPorts.REA_MODULE_UI_PORT,
                                                                  REACommunicationProperties.getPrivateNetClassList());
      kryoMessager.setAllowSelfSubmit(true);
      kryoMessager.startMessager();

      return new LidarImageFusionProcessorCommunicationModule(ros2Node, kryoMessager, messager);
   }

   private static IntrinsicParameters toIntrinsicParameters(IntrinsicParametersMessage message)
   {
      IntrinsicParameters intrinsicParameters = new IntrinsicParameters();
      intrinsicParameters.width = message.getWidth();
      intrinsicParameters.height = message.getHeight();
      intrinsicParameters.fx = message.getFx();
      intrinsicParameters.fy = message.getFy();
      intrinsicParameters.skew = message.getSkew();
      intrinsicParameters.cx = message.getCx();
      intrinsicParameters.cy = message.getCy();
      if (!message.getRadial().isEmpty())
         intrinsicParameters.radial = message.getRadial().toArray();
      intrinsicParameters.t1 = message.getT1();
      intrinsicParameters.t2 = message.getT2();
      return intrinsicParameters;
   }
}