package us.ihmc.robotEnvironmentAwareness.fusion;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.ImageMessage;
import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import controller_msgs.msg.dds.VideoPacket;
import org.opencv.video.Video;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
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
import us.ihmc.ros2.Ros2Node;

public class LidarImageFusionProcessorCommunicationModule
{
   private final Messager messager;

   private final Ros2Node ros2Node;
   private final REAModuleStateReporter moduleStateReporter;
   private final StereoREAModule stereoREAModule; // TODO: realize online runner with enable button.

   private final FusionSensorObjectDetectionManager objectDetectionManager;

   private final AtomicReference<String> socketHostIPAddress;
   private final AtomicReference<BufferedImage> latestBufferedImage = new AtomicReference<>(null);
   private final AtomicReference<List<ObjectType>> selectedObjecTypes;

   private LidarImageFusionProcessorCommunicationModule(Ros2Node ros2Node, Messager reaMessager, SharedMemoryJavaFXMessager messager)
   {
      this.messager = messager;
      this.ros2Node = ros2Node;

      moduleStateReporter = new REAModuleStateReporter(reaMessager);
      stereoREAModule = new StereoREAModule(reaMessager, messager);

      new ROS2Callback<>(ros2Node, LidarScanMessage.class, this::dispatchLidarScanMessage);
      new ROS2Callback<>(ros2Node, StereoVisionPointCloudMessage.class, this::dispatchStereoVisionPointCloudMessage);
      new ROS2Callback<>(ros2Node, ImageMessage.class, this::dispatchImageMessage);
//      new ROS2Callback<>(ros2Node, VideoPacket.class, this::dispatchVideoPacket);

      objectDetectionManager = new FusionSensorObjectDetectionManager(ros2Node, messager);

      messager.registerTopicListener(LidarImageFusionAPI.RequestSocketConnection, (content) -> connect());
      messager.registerTopicListener(LidarImageFusionAPI.RequestObjectDetection, (content) -> request());
      selectedObjecTypes = messager.createInput(LidarImageFusionAPI.SelectedObjecTypes, new ArrayList<ObjectType>());
      socketHostIPAddress = messager.createInput(LidarImageFusionAPI.ObjectDetectionModuleAddress);

      //TODO: will be replaced by LidarImageFusionAPI.EnableREA.
      //messager.registerTopicListener(LidarImageFusionAPI.EnableREA, (content) -> stereoREAModule.enable());
      messager.registerTopicListener(LidarImageFusionAPI.RunStereoREA, (content) -> stereoREAModule.run());
   }

   private void connect()
   {
      objectDetectionManager.connectExternalModule(socketHostIPAddress.get());
   }

   private void request()
   {
      objectDetectionManager.requestObjectDetection(latestBufferedImage.getAndSet(null), selectedObjecTypes.get());
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

   public void start() throws IOException
   {

   }

   public void stop() throws Exception
   {
      messager.closeMessager();
      ros2Node.destroy();
      objectDetectionManager.close();
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
}