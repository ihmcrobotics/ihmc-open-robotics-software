package us.ihmc.robotEnvironmentAwareness.fusion;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.depthOutputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.lidarOutputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.outputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.stereoOutputTopic;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import boofcv.struct.calib.CameraPinholeBrown;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import perception_msgs.msg.dds.Image32;
import perception_msgs.msg.dds.IntrinsicParametersMessage;
import perception_msgs.msg.dds.LidarScanMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.VideoPacket;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.fusion.objectDetection.FusionSensorObjectDetectionManager;
import us.ihmc.robotEnvironmentAwareness.fusion.objectDetection.ObjectType;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.ImageVisualizationHelper;
import us.ihmc.robotEnvironmentAwareness.updaters.REAModuleStateReporter;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionPublicNetworkProvider;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

public class LidarImageFusionProcessorCommunicationModule
{
   private final Messager messager;

   private final ROS2Node ros2Node;
   private final REAModuleStateReporter moduleStateReporter;
   private final StereoREAModule stereoREAModule;

   private final FusionSensorObjectDetectionManager objectDetectionManager;

   private final AtomicReference<String> socketHostIPAddress;
   private final AtomicReference<BufferedImage> latestBufferedImage = new AtomicReference<>(null);
   private final AtomicReference<List<ObjectType>> selectedObjecTypes;

   private final JPEGDecompressor jpegDecompressor = new JPEGDecompressor();

   private final ExceptionHandlingThreadScheduler scheduler;
   private static final int BUFFER_THREAD_PERIOD_MILLISECONDS = 1500;

   private LidarImageFusionProcessorCommunicationModule(ROS2Node ros2Node, REANetworkProvider networkProvider, Messager reaMessager, SharedMemoryJavaFXMessager messager)
   {
      this.messager = messager;
      this.ros2Node = ros2Node;

      moduleStateReporter = new REAModuleStateReporter(reaMessager);
      stereoREAModule = new StereoREAModule(networkProvider, reaMessager, messager);

      networkProvider.registerMessager(reaMessager);
      networkProvider.registerLidarScanHandler(this::dispatchLidarScanMessage);
      networkProvider.registerStereoVisionPointCloudHandler(this::dispatchStereoVisionPointCloudMessage);
      networkProvider.registerCustomRegionsHandler(this::dispatchCustomPlanarRegion);

      new ROS2Callback<>(ros2Node, Image32.class, ROS2Tools.IHMC_ROOT, this::dispatchImage32);
      new ROS2Callback<>(ros2Node, VideoPacket.class, ROS2Tools.IHMC_ROOT, this::dispatchVideoPacket);

      objectDetectionManager = new FusionSensorObjectDetectionManager(ros2Node, messager);

      messager.addTopicListener(LidarImageFusionAPI.RequestSocketConnection, (content) -> connect());
      messager.addTopicListener(LidarImageFusionAPI.RequestObjectDetection, (content) -> request());
      selectedObjecTypes = messager.createInput(LidarImageFusionAPI.SelectedObjecTypes, new ArrayList<ObjectType>());
      socketHostIPAddress = messager.createInput(LidarImageFusionAPI.ObjectDetectionModuleAddress);

      messager.addTopicListener(LidarImageFusionAPI.RunStereoREA, (content) -> stereoREAModule.singleRun());

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
      stereoREAModule.dispatchCustomPlanarRegion(message);
   }

   private void dispatchLidarScanMessage(Subscriber<LidarScanMessage> message)
   {
      moduleStateReporter.registerLidarScanMessage(message.takeNextData());
   }

   private void dispatchStereoVisionPointCloudMessage(Subscriber<StereoVisionPointCloudMessage> subscriber)
   {
      StereoVisionPointCloudMessage message = subscriber.takeNextData();
      moduleStateReporter.registerStereoVisionPointCloudMessage(message);
      objectDetectionManager.updateLatestStereoVisionPointCloudMessage(message);
      stereoREAModule.updateLatestStereoVisionPointCloudMessage(message);
   }

   private void dispatchImage32(Image32 message)
   {
      if (messager.isMessagerOpen())
         messager.submitMessage(LidarImageFusionAPI.ImageState, new Image32(message));

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

   public static LidarImageFusionProcessorCommunicationModule createIntraprocessModule(ROS2Node ros2Node, SharedMemoryJavaFXMessager messager)
         throws IOException
   {
      KryoMessager kryoMessager = KryoMessager.createIntraprocess(REAModuleAPI.API, NetworkPorts.REA_MODULE_UI_PORT,
                                                                  REACommunicationProperties.getPrivateNetClassList());
      kryoMessager.setAllowSelfSubmit(true);
      kryoMessager.startMessager();

      REANetworkProvider networkProvider = new REAPlanarRegionPublicNetworkProvider(ros2Node,
                                                                                    outputTopic,
                                                                                    lidarOutputTopic,
                                                                                    stereoOutputTopic,
                                                                                    depthOutputTopic);
      return new LidarImageFusionProcessorCommunicationModule(ros2Node, networkProvider, kryoMessager, messager);
   }

   private static CameraPinholeBrown toIntrinsicParameters(IntrinsicParametersMessage message)
   {
      CameraPinholeBrown intrinsicParameters = new CameraPinholeBrown();
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