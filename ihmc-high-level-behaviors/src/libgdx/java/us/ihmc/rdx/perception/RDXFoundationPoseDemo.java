package us.ihmc.rdx.perception;

import imgui.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.FoundationPoseRequest;
import perception_msgs.FoundationPoseResult;
import perception_msgs.ImageMessage;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionThread;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.graphics.ros2.yolo.RDXROS2YOLOv8Visualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import static us.ihmc.zed.global.zed.*;

/**
 * <h1>RDXFoundationPoseDemo</h1>
 *
 * <p>
 * This demo application showcases a real-time perception pipeline for object pose estimation
 * using a ZED stereo camera, YOLOv8 object detection, and ROS2-based communication within the
 * IHMC RDX (Robot Development eXperience) UI framework.
 * </p>
 *
 * <p>
 * The class integrates several perception and robotics components:
 * <ul>
 *   <li>Captures RGB and depth images from a ZED camera.</li>
 *   <li>Runs YOLOv8 object detection on the incoming images.</li>
 *   <li>Publishes and subscribes to ROS2 topics for image and pose data exchange.</li>
 *   <li>Allows user interaction through an ImGui-based UI for sending pose requests and controlling tracking.</li>
 *   <li>Visualizes point clouds, detected objects, and object poses in a 3D scene.</li>
 * </ul>
 * </p>
 *
 * <p>
 * The main purpose of this class is to demonstrate an end-to-end workflow for requesting and receiving
 * 6-DoF object poses (e.g., for a "mustard bottle") in a robotics context. It can be used as a template
 * for integrating perception, detection, and UI feedback in IHMC's RDX framework.
 * </p>
 *
 * <p>
 * Typical usage is as a standalone application: running the main method will launch the UI, start the
 * ZED camera, begin object detection, and allow the user to interactively request object pose estimation
 * results, which are visualized in real time.
 * </p>
 *
 * <p>
 * <b>Note:</b> This class is intended for demonstration and development purposes. It assumes a compatible
 * CUDA device for neural depth, a connected ZED camera, and an appropriate ROS2 environment.
 * </p>
 */
public class RDXFoundationPoseDemo
{
   private static final String OBJECT_ID = "mustard_bottle";
   private static final String OBJECT_NAME = "bottle";

   private static final BytePointer JPG = new BytePointer(".jpg");
   private static final BytePointer PNG = new BytePointer(".png");

   private static final ROS2Topic<?> RELIABLE_TOPIC = new ROS2Topic<>();
   private static final ROS2Topic<ImageMessage> COLOR_TOPIC = RELIABLE_TOPIC.appendedWith("foundation_pose/color_rgb8").withType(ImageMessage.class);
   private static final ROS2Topic<ImageMessage> DEPTH_TOPIC = RELIABLE_TOPIC.appendedWith("foundation_pose/depth_mono16").withType(ImageMessage.class);
   private static final ROS2Topic<FoundationPoseRequest> REQUEST_TOPIC = RELIABLE_TOPIC.appendedWith("foundation_pose/request").withType(FoundationPoseRequest.class);
   private static final ROS2Topic<std_msgs.String_> REMOVE_TOPIC = RELIABLE_TOPIC.appendedWith("foundation_pose/remove").withType(std_msgs.String_.class);
   private static final ROS2Topic<FoundationPoseResult> RESULT_TOPIC = new ROS2Topic<>().appendedWith("foundation_pose/result").withType(FoundationPoseResult.class);

   private final ROS2Node ros2Node;
   private final ROS2PeerClockOffsetEstimator robotClockOffsetEstimator;
   private final ROS2PeerClockOffsetEstimator uiClockOffsetEstimator;

   private final ROS2Publisher<FoundationPoseRequest> requestPublisher;
   private final FoundationPoseRequest requestMessage;
   private boolean sendRequest;

   private final ROS2Publisher<std_msgs.String_> removePublisher;
   private final std_msgs.String_ removeMessage;

   private final ROS2Publisher<ImageMessage> colorPublisher;
   private final ROS2Publisher<ImageMessage> depthPublisher;
   private final ImageMessage colorMessage;
   private final ImageMessage depthMessage;

   private final ZEDImageSensor zed;
   private final YOLOv8DetectionThread yoloThread;
   private final RepeatingTaskThread zedImageConsumerThread;


   // UI Stuff
   private final RDXBaseUI baseUI;
   private final RDXRawImagePointCloudVisualizer pointCloudVisualizer;
   private final RDXROS2YOLOv8Visualizer yoloSettings;
   private RDXPose3DGizmo zedPoseGizmo;
   private RDXReferenceFrameGraphic objectPoseGraphic;

   private final AtomicBoolean destroyed;

   private RDXFoundationPoseDemo()
   {
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy));

      ros2Node = new ROS2Node(getClass().getSimpleName());
      robotClockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);
      uiClockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);
      requestPublisher = ros2Node.createPublisher(REQUEST_TOPIC);
      requestMessage = new FoundationPoseRequest();
      sendRequest = false;
      removePublisher = ros2Node.createPublisher(REMOVE_TOPIC);
      removeMessage = new std_msgs.String_();

      ros2Node.createSubscription(RESULT_TOPIC, reader -> this.receivePose(reader.read()));

      boolean enableNeuralMode = CUDATools.hasCUDADeviceOfAtLeast(CUDATools.getDeviceName(0), "RTX 3080");
      zed = new ZEDImageSensor(0, ZEDModelData.ZED_2, SL_INPUT_TYPE_USB);
      zed.getInitParameters().depth_mode(enableNeuralMode ? SL_DEPTH_MODE_NEURAL : SL_DEPTH_MODE_PERFORMANCE);

      colorPublisher = ros2Node.createPublisher(COLOR_TOPIC);
      depthPublisher = ros2Node.createPublisher(DEPTH_TOPIC);
      colorMessage = new ImageMessage();
      depthMessage = new ImageMessage();

      yoloThread = new YOLOv8DetectionThread(ros2Node, robotClockOffsetEstimator, () -> true);
      yoloThread.setImageSensor(zed, ZEDImageSensor.LEFT_COLOR_IMAGE_KEY, ZEDImageSensor.DEPTH_IMAGE_KEY);
      yoloThread.addDetectionConsumerCallback(this::publishRequest);

      zedImageConsumerThread = new RepeatingTaskThread("ZEDImageConsumer", this::consumeZEDImage);

      baseUI = new RDXBaseUI();
      pointCloudVisualizer = new RDXRawImagePointCloudVisualizer("ZED Point Cloud");
      yoloSettings = new RDXROS2YOLOv8Visualizer("YOLO Results", ros2Node, uiClockOffsetEstimator, PerceptionAPI.EXPERIMENTAL_ZED_COLOR.get(RobotSide.LEFT));

      destroyed = new AtomicBoolean(false);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            yoloSettings.create();
            yoloSettings.setActive(true);
            zedPoseGizmo = new RDXPose3DGizmo();
            objectPoseGraphic = new RDXReferenceFrameGraphic(0.2);

            baseUI.getImGuiPanelManager().addPanel(yoloSettings.getPanel());
            baseUI.getImGuiPanelManager().addPanel("YOLO Settings", yoloSettings::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel("Options", this::renderOptions);
            baseUI.getPrimaryScene().addRenderableProvider(yoloSettings);
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudVisualizer);
            baseUI.getPrimaryScene().addRenderableProvider(zedPoseGizmo);
            baseUI.getPrimaryScene().addRenderableProvider(objectPoseGraphic);
            baseUI.create();

            zedPoseGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());

            zed.setSensorFrame(zedPoseGizmo.getGizmoFrame());
            zed.run(true);
            yoloThread.startRepeating();
            zedImageConsumerThread.startRepeating();
         }

         private void renderOptions()
         {
            if (ImGui.button("Send request"))
               sendRequest = true;

            if (ImGui.button("Stop tracking"))
            {
               removeMessage.setData(OBJECT_ID);
               removePublisher.publish(removeMessage);
            }
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

   private void receivePose(FoundationPoseResult result)
   {
      objectPoseGraphic.setPoseInWorldFrame(result.getObjectPose().getPose());
   }

   private void consumeZEDImage()
   {
      try
      {
         zed.waitForGrab();

         RawImage color = zed.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
         RawImage depth = zed.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);

         // Get color in RGB
         Mat rgbColor = new Mat();
         color.getPixelFormat().convertToPixelFormat(color.getCpuImageMat(), rgbColor, PixelFormat.RGB8);

         // Compress images
         BytePointer encodedColor = new BytePointer();
         BytePointer encodedDepth = new BytePointer();

         opencv_imgcodecs.imencode(JPG, rgbColor, encodedColor);
         opencv_imgcodecs.imencode(PNG, depth.getCpuImageMat(), encodedDepth);

         // Publish compressed images
         PerceptionMessageTools.packImageMessage(color, encodedColor, CompressionType.JPEG, colorMessage);
         PerceptionMessageTools.packImageMessage(depth, encodedDepth, CompressionType.PNG, depthMessage);

         colorPublisher.publish(colorMessage);
         depthPublisher.publish(depthMessage);

         // Update visualizer
         pointCloudVisualizer.setColorImage(color);
         pointCloudVisualizer.setDepthImage(depth);

         rgbColor.close();
         encodedColor.close();
         encodedDepth.close();
         color.release();
         depth.release();
      }
      catch (InterruptedException ignored) {}
   }

   private void publishRequest(List<InstantDetection> yoloDetections)
   {
      if (!sendRequest)
         return;

      for (InstantDetection detection : yoloDetections)
      {
         if (!detection.getDetectedObjectClass().equals(OBJECT_NAME))
            continue;

         if (detection instanceof YOLOv8InstantDetection yoloDetection)
         {
            System.out.println("Sending request");

            RawImage color = yoloDetection.getColorImage();
            RawImage depth = yoloDetection.getDepthImage();
            RawImage mask = yoloDetection.getObjectMask();

            // Get color in rgb
            Mat rgbColor = new Mat();
            color.getPixelFormat().convertToPixelFormat(color.getCpuImageMat(), rgbColor, PixelFormat.RGB8);

            // Compress images
            BytePointer encodedColor = new BytePointer();
            BytePointer encodedDepth = new BytePointer();
            BytePointer encodedMask = new BytePointer();

            opencv_imgcodecs.imencode(JPG, rgbColor, encodedColor);
            opencv_imgcodecs.imencode(PNG, depth.getCpuImageMat(), encodedDepth);
            opencv_imgcodecs.imencode(PNG, mask.getCpuImageMat(), encodedMask);

            // Pack and publish request
            requestMessage.setObjectId(OBJECT_ID);
            requestMessage.setMeshFile("mustard0.obj");
            PerceptionMessageTools.packImageMessage(color, encodedColor, CompressionType.JPEG, requestMessage.getColor());
            PerceptionMessageTools.packImageMessage(depth, encodedDepth, CompressionType.PNG, requestMessage.getDepth());
            PerceptionMessageTools.packImageMessage(mask, encodedMask, CompressionType.PNG, requestMessage.getObjectMask());
            requestPublisher.publish(requestMessage);

            // Release pointers
            rgbColor.close();
            encodedColor.close();
            encodedDepth.close();
            encodedMask.close();

            sendRequest = false;
         }
      }
   }

   private void destroy()
   {
      if (destroyed.getAndSet(true))
         return;

      yoloThread.blockingKill();
      zedImageConsumerThread.blockingKill();
      zed.close();

      robotClockOffsetEstimator.destroy();
      uiClockOffsetEstimator.destroy();
      ros2Node.close();
   }

   public static void main(String[] args)
   {
      new RDXFoundationPoseDemo();
   }
}
