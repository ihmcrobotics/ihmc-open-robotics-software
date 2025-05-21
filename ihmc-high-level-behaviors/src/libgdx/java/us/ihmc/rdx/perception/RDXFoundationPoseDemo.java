package us.ihmc.rdx.perception;

import geometry_msgs.msg.dds.PoseStamped;
import imgui.ImGui;
import perception_msgs.msg.dds.FoundationPoseRequest;
import sensor_msgs.msg.dds.CameraInfo;
import sensor_msgs.msg.dds.Image;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.ImageSensorPublishThread;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionThread;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.yolo.RDXROS2YOLOv8Visualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import static us.ihmc.zed.global.zed.*;

class RDXFoundationPoseDemo
{
   private static final String OBJECT_NAME = "bottle";
   private static final ROS2Topic<?> RELIABLE_TOPIC = new ROS2Topic<>().withQoS(ROS2QosProfile.RELIABLE());
   private static final ROS2Topic<Image> COLOR_TOPIC = RELIABLE_TOPIC.withModule("foundation_pose/camera/color/image_raw").withType(Image.class);
   private static final ROS2Topic<Image> DEPTH_TOPIC = RELIABLE_TOPIC.withModule("foundation_pose/camera/aligned_depth_to_color/image_raw").withType(Image.class);
   private static final ROS2Topic<CameraInfo> CAMERA_INFO_TOPIC = RELIABLE_TOPIC.withModule("foundation_pose/camera/color/camera_info").withType(CameraInfo.class);
   private static final ROS2Topic<FoundationPoseRequest> REQUEST_TOPIC = RELIABLE_TOPIC.withModule("foundation_pose/request").withType(FoundationPoseRequest.class);
   private static final ROS2Topic<PoseStamped> POSE_TOPIC = new ROS2Topic<>().withModule("pose").withType(PoseStamped.class);

   private final ROS2Node ros2Node;
   private final ROS2PeerClockOffsetEstimator robotClockOffsetEstimator;
   private final ROS2PeerClockOffsetEstimator uiClockOffsetEstimator;
   private final ROS2Publisher<FoundationPoseRequest> requestPublisher;
   private final FoundationPoseRequest requestMessage;
   private boolean requestSent;

   private final ImageSensor zed;
   private final ImageSensorPublishThread imagePublishThread;
   private final YOLOv8DetectionThread yoloThread;
   private final RepeatingTaskThread zedImageConsumerThread;

   // UI Stuff
   private final RDXBaseUI baseUI;
   private final RDXRawImagePointCloudVisualizer pointCloudVisualizer;
   private final RDXROS2YOLOv8Visualizer yoloSettings;
   private RDXPose3DGizmo zedPoseGizmo;

   private final AtomicBoolean destroyed;

   private RDXFoundationPoseDemo()
   {
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy));

      ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName());
      robotClockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);
      uiClockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);
      requestPublisher = ros2Node.createPublisher(REQUEST_TOPIC);
      requestMessage = new FoundationPoseRequest();
      requestSent = false;

      ros2Node.createSubscription2(POSE_TOPIC, this::receivePose);

      boolean enableNeuralMode = CUDATools.hasCUDADeviceOfAtLeast(CUDATools.getDeviceName(0), "RTX 3080");
      zed = new ZEDImageSensor(0, ZEDModelData.ZED_2, SL_INPUT_TYPE_USB, enableNeuralMode ? SL_DEPTH_MODE_NEURAL : SL_DEPTH_MODE_PERFORMANCE);

      imagePublishThread = new ImageSensorPublishThread(ros2Node, zed);
      imagePublishThread.addTopic(COLOR_TOPIC, ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      imagePublishThread.addTopic(DEPTH_TOPIC, ZEDImageSensor.DEPTH_IMAGE_KEY);
      imagePublishThread.addTopic(CAMERA_INFO_TOPIC, ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      imagePublishThread.enableROS2Frames(true);

      yoloThread = new YOLOv8DetectionThread(robotClockOffsetEstimator, () -> true);
      yoloThread.setImageSensor(zed, ZEDImageSensor.LEFT_COLOR_IMAGE_KEY, ZEDImageSensor.DEPTH_IMAGE_KEY);
      yoloThread.addDetectionConsumerCallback(this::publishRequest);

      zedImageConsumerThread = new RepeatingTaskThread("ZEDImageConsumer", this::consumeZEDImage);

      baseUI = new RDXBaseUI();
      pointCloudVisualizer = new RDXRawImagePointCloudVisualizer("ZED Point Cloud");
      yoloSettings = new RDXROS2YOLOv8Visualizer("YOLO Results", ros2Node, uiClockOffsetEstimator, PerceptionAPI.YOLO_ANNOTATED_IMAGE);

      destroyed = new AtomicBoolean(false);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            yoloSettings.create();
            yoloSettings.setActive(true);
            zedPoseGizmo = new RDXPose3DGizmo();

            baseUI.getImGuiPanelManager().addPanel(yoloSettings.getPanel());
            baseUI.getImGuiPanelManager().addPanel("YOLO Settings", yoloSettings::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel("Options", this::renderOptions);
            baseUI.getPrimaryScene().addRenderableProvider(yoloSettings);
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudVisualizer);
            baseUI.getPrimaryScene().addRenderableProvider(zedPoseGizmo);
            baseUI.create();

            zedPoseGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());

            zed.setSensorFrame(zedPoseGizmo.getGizmoFrame());
            zed.run(true);
            imagePublishThread.startRepeating();
            yoloThread.startRepeating();
            zedImageConsumerThread.startRepeating();
         }

         private void renderOptions()
         {
            if (ImGui.button("Send request"))
               requestSent = false;
         }

         @Override
         public void render()
         {
            pointCloudVisualizer.update();
            yoloSettings.update();
            yoloSettings.updateHeartbeat();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            pointCloudVisualizer.destroy();
            yoloSettings.destroy();
            baseUI.dispose();

            destroy();
         }
      });

   }

   private void receivePose(PoseStamped message)
   {

   }

   private void consumeZEDImage()
   {
      try
      {
         zed.waitForGrab();

         RawImage color = zed.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
         RawImage depth = zed.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);

         pointCloudVisualizer.setColorImage(color);
         pointCloudVisualizer.setDepthImage(depth);

         color.release();
         depth.release();
      }
      catch (InterruptedException ignored) {}
   }

   private void publishRequest(List<InstantDetection> yoloDetections)
   {
      if (requestSent)
         return;

      for (InstantDetection detection : yoloDetections)
      {
         if (!detection.getDetectedObjectClass().equals(OBJECT_NAME))
            continue;

         if (detection instanceof YOLOv8InstantDetection yoloDetection)
         {
            System.out.println("Sending request");

            requestMessage.setMeshFile("mustard0.obj");
            PerceptionMessageTools.packImageMessage(yoloDetection.getColorImage(), "odom", requestMessage.getColor());
            PerceptionMessageTools.packImageMessage(yoloDetection.getDepthImage(), "odom", requestMessage.getDepth());
            PerceptionMessageTools.packImageMessage(yoloDetection.getObjectMask(), "odom", requestMessage.getObjectMask());
            requestPublisher.publish(requestMessage);

            requestSent = true;
         }
      }
   }

   private void destroy()
   {
      if (destroyed.getAndSet(true))
         return;

      imagePublishThread.blockingKill();
      yoloThread.blockingKill();
      zedImageConsumerThread.blockingKill();
      zed.close();

      robotClockOffsetEstimator.destroy();
      uiClockOffsetEstimator.destroy();
      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      new RDXFoundationPoseDemo();
   }
}
