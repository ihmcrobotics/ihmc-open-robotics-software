package us.ihmc.sensors;

import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.*;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.zed.SL_InitParameters;
import org.bytedeco.zed.SL_RuntimeParameters;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.cuda.CUDAImageEncoder;
import us.ihmc.perception.tools.ImageMessageDataPacker;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.PausablePeriodicThread;

import static org.bytedeco.zed.global.zed.*;

public class ZED2ColorStereoDepthPublisher
{
   private static final double UPDATE_PERIOD_SECONDS = 0.1;

   private int cameraID;
   private final SL_InitParameters zedInitializationParameters = new SL_InitParameters();
   private final SL_RuntimeParameters zedRuntimeParameters = new SL_RuntimeParameters();

   private int imageWidth;
   private int imageHeight;
   private Pointer colorImagePointer;
   private Pointer depthImagePointer;
   private Pointer pointCloudPointer;
   private GpuMat colorImageMatBGRA;
   private GpuMat colorImageMatBGR;
   private BytePointer jpegImagePointer = new BytePointer();

   private final ImageMessage colorImageMessage = new ImageMessage();
   private final ImageMessage depthImageMessage = new ImageMessage();
   private final IHMCROS2Publisher<ImageMessage> ros2ImagePublisher;
   private final ROS2Topic<ImageMessage> colorTopic;
   private final ROS2Topic<ImageMessage> depthTopic;
   private final ROS2Node ros2Node;
   private CUDAImageEncoder imageEncoder;

   private final PausablePeriodicThread zedPublisherThread = new PausablePeriodicThread("ZED publisher", UPDATE_PERIOD_SECONDS, this::update);

   public ZED2ColorStereoDepthPublisher(int cameraID,
                                        ROS2Topic<ImageMessage> colorTopic,
                                        ROS2Topic<ImageMessage> depthTopic)
   {
      this.cameraID = cameraID;

      sl_create_camera(cameraID);

      zedInitializationParameters.camera_fps(10);
      zedInitializationParameters.resolution(SL_RESOLUTION_HD1080);
      zedInitializationParameters.input_type(SL_INPUT_TYPE_USB);
      zedInitializationParameters.camera_device_id(cameraID);
      zedInitializationParameters.camera_image_flip(SL_FLIP_MODE_AUTO);
      zedInitializationParameters.camera_disable_self_calib(false);
      zedInitializationParameters.enable_image_enhancement(true);
      zedInitializationParameters.svo_real_time_mode(true);
      zedInitializationParameters.depth_mode(SL_DEPTH_MODE_PERFORMANCE);
      zedInitializationParameters.depth_stabilization(1);
      zedInitializationParameters.depth_maximum_distance(20);
      zedInitializationParameters.depth_minimum_distance(-1);
      zedInitializationParameters.coordinate_unit(SL_UNIT_METER);
      zedInitializationParameters.coordinate_system(SL_COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD);
      zedInitializationParameters.sdk_gpu_id(-1);
      zedInitializationParameters.sdk_verbose(0); // false
      zedInitializationParameters.sensors_required(false);
      zedInitializationParameters.enable_right_side_measure(false);
      zedInitializationParameters.open_timeout_sec(5.0f);
      zedInitializationParameters.async_grab_camera_recovery(false);

      checkError("sl_open_camera", sl_open_camera(cameraID, zedInitializationParameters, 0, "", "", 0, "", "", ""));

      zedRuntimeParameters.enable_depth(true);
      zedRuntimeParameters.confidence_threshold(95);
      zedRuntimeParameters.reference_frame(SL_REFERENCE_FRAME_CAMERA);
      zedRuntimeParameters.texture_confidence_threshold(100);
      zedRuntimeParameters.remove_saturated_areas(true);

      imageWidth = sl_get_width(cameraID);
      imageHeight = sl_get_height(cameraID);

      colorImagePointer = new Pointer(sl_mat_create_new(imageWidth, imageHeight, SL_MAT_TYPE_U8_C4, SL_MEM_GPU));
      depthImagePointer = new Pointer(sl_mat_create_new(imageWidth, imageHeight, SL_MAT_TYPE_F32_C4, SL_MEM_CPU));
      pointCloudPointer = new Pointer(sl_mat_create_new(imageWidth, imageHeight, SL_MAT_TYPE_F32_C4, SL_MEM_CPU));
      colorImageMatBGR = new GpuMat(imageWidth, imageHeight, opencv_core.CV_8UC3);

      this.colorTopic = colorTopic;
      this.depthTopic = depthTopic;
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "zed2_node");
      ros2ImagePublisher = ROS2Tools.createPublisher(ros2Node, colorTopic, ROS2QosProfile.BEST_EFFORT());

      imageEncoder = new CUDAImageEncoder();

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, getClass().getName() + "-Shutdown"));

      zedPublisherThread.start();
   }

   public void update()
   {
      checkError("sl_grab", sl_grab(cameraID, zedRuntimeParameters));

      retrieveAndPublishColorImage();
      checkError("sl_retrieve_measure", sl_retrieve_measure(cameraID, depthImagePointer, SL_MEASURE_DEPTH_RIGHT, SL_MEM_CPU, imageWidth, imageHeight));
      checkError("sl_retrieve_measure", sl_retrieve_measure(cameraID, pointCloudPointer, SL_MEASURE_XYZRGBA_RIGHT, SL_MEM_CPU, imageWidth, imageHeight));
   }

   private void retrieveAndPublishColorImage()
   {
      // Retrieve image
      checkError("sl_retrieve_image", sl_retrieve_image(cameraID, colorImagePointer, SL_VIEW_RIGHT, SL_MEM_GPU, imageWidth, imageHeight));

      // Convert to BGR and encode to jpeg
      colorImageMatBGRA = new GpuMat(colorImagePointer);
      opencv_cudaimgproc.cvtColor(colorImageMatBGRA, colorImageMatBGR, opencv_imgproc.COLOR_BGRA2BGR);
      imageEncoder.encodeBGR(colorImageMatBGR.data(), jpegImagePointer, imageWidth, imageHeight, colorImageMatBGR.step());

      // Publish image
      ImageMessageDataPacker imageMessageDataPacker = new ImageMessageDataPacker(jpegImagePointer.limit());
      imageMessageDataPacker.pack(colorImageMessage, jpegImagePointer);
      colorImageMessage.setImageWidth(imageWidth);
      colorImageMessage.setImageHeight(imageHeight);
      CameraModel.PINHOLE.packMessageFormat(colorImageMessage);
      ImageMessageFormat.COLOR_JPEG_BGR8.packMessageFormat(colorImageMessage);
      // TODO: Add position and orientation to image message
      ros2ImagePublisher.publish(colorImageMessage);
   }

   public void destroy()
   {
      zedPublisherThread.destroy();
      sl_close_camera(cameraID);
      ros2Node.destroy();
   }

   private void checkError(String functionName, int returnedState)
   {
      if (returnedState != SL_ERROR_CODE_SUCCESS)
      {
         LogTools.error(String.format("%s returned '%d'", functionName, returnedState));
      }
   }

   public static void main(String[] args)
   {
      new ZED2ColorStereoDepthPublisher(0, PerceptionAPI.ZED2_STEREO_COLOR, null);
   }
}
