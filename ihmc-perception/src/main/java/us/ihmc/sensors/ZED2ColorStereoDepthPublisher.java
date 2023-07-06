package us.ihmc.sensors;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.zed.SL_CalibrationParameters;
import org.bytedeco.zed.SL_InitParameters;
import org.bytedeco.zed.SL_RuntimeParameters;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.cuda.CUDAImageEncoder;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.ImageMessageDataPacker;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.time.Instant;
import java.util.concurrent.atomic.AtomicReference;

import static org.bytedeco.zed.global.zed.*;

public class ZED2ColorStereoDepthPublisher
{
   private static final double UPDATE_PERIOD_SECONDS = 0.1;
   private static final float DEPTH_DISCRETIZATION = 0.001f;

   private long depthImageSequenceNumber = 0L;
   private long colorImageSequenceNumber = 0L;

   private final int cameraID;
   private final SL_RuntimeParameters zedRuntimeParameters = new SL_RuntimeParameters();
   private final SL_CalibrationParameters zedCalibrationParameters;

   private final int imageWidth;
   private final int imageHeight;
   private final Pointer colorImagePointer;
   private final Pointer depthImagePointer;

   private final FramePose3D worldFramePose = new FramePose3D(ReferenceFrame.getWorldFrame());

   private final ImageMessage colorImageMessage = new ImageMessage();
   private final ImageMessage depthImageMessage = new ImageMessage();
   private final AtomicReference<Instant> colorImageAcquisitionTime = new AtomicReference<>();
   private final AtomicReference<Instant> depthImageAcquisitionTime = new AtomicReference<>();
   private final IHMCROS2Publisher<ImageMessage> ros2ColorImagePublisher;
   private final IHMCROS2Publisher<ImageMessage> ros2DepthImagePublisher;
   private final ROS2Node ros2Node;
   private CUDAImageEncoder imageEncoder;

   private final PausablePeriodicThread zedPublisherThread = new PausablePeriodicThread("ZED publisher", UPDATE_PERIOD_SECONDS, this::update);

   public ZED2ColorStereoDepthPublisher(int cameraID,
                                        ROS2Topic<ImageMessage> colorTopic,
                                        ROS2Topic<ImageMessage> depthTopic)
   {
      this.cameraID = cameraID;

      sl_create_camera(cameraID);

      SL_InitParameters zedInitializationParameters = new SL_InitParameters();
      zedInitializationParameters.camera_fps(10);
      zedInitializationParameters.resolution(SL_RESOLUTION_HD1080);
      zedInitializationParameters.input_type(SL_INPUT_TYPE_USB);
      zedInitializationParameters.camera_device_id(cameraID);
      zedInitializationParameters.camera_image_flip(SL_FLIP_MODE_OFF);
      zedInitializationParameters.camera_disable_self_calib(false);
      zedInitializationParameters.enable_image_enhancement(true);
      zedInitializationParameters.svo_real_time_mode(true);
      zedInitializationParameters.depth_mode(SL_DEPTH_MODE_ULTRA);
      zedInitializationParameters.depth_stabilization(1);
      zedInitializationParameters.depth_maximum_distance(20);
      zedInitializationParameters.depth_minimum_distance(-1);
      zedInitializationParameters.coordinate_unit(SL_UNIT_METER);
      zedInitializationParameters.coordinate_system(SL_COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD);
      zedInitializationParameters.sdk_gpu_id(-1);
      zedInitializationParameters.sdk_verbose(0); // false
      zedInitializationParameters.sensors_required(true);
      zedInitializationParameters.enable_right_side_measure(false);
      zedInitializationParameters.open_timeout_sec(5.0f);
      zedInitializationParameters.async_grab_camera_recovery(false);

      checkError("sl_open_camera", sl_open_camera(cameraID, zedInitializationParameters, 0, "", "", 0, "", "", ""));

      zedRuntimeParameters.enable_depth(true);
      zedRuntimeParameters.confidence_threshold(95);
      zedRuntimeParameters.reference_frame(SL_REFERENCE_FRAME_CAMERA);
      zedRuntimeParameters.texture_confidence_threshold(100);
      zedRuntimeParameters.remove_saturated_areas(true);
      zedRuntimeParameters.enable_fill_mode(false);

      zedCalibrationParameters = sl_get_calibration_parameters(cameraID, false);
      imageWidth = sl_get_width(cameraID);
      imageHeight = sl_get_height(cameraID);

      colorImagePointer = new Pointer(sl_mat_create_new(imageWidth, imageHeight, SL_MAT_TYPE_U8_C4, SL_MEM_CPU));
      depthImagePointer = new Pointer(sl_mat_create_new(imageWidth, imageHeight, SL_MAT_TYPE_U16_C1, SL_MEM_CPU));

      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "zed2_node");
      ros2ColorImagePublisher = ROS2Tools.createPublisher(ros2Node, colorTopic, ROS2QosProfile.BEST_EFFORT());
      ros2DepthImagePublisher = ROS2Tools.createPublisher(ros2Node, depthTopic, ROS2QosProfile.BEST_EFFORT());

      imageEncoder = new CUDAImageEncoder();

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, getClass().getName() + "-Shutdown"));

      zedPublisherThread.start();
   }

   public void update()
   {
      checkError("sl_grab", sl_grab(cameraID, zedRuntimeParameters));

      retrieveAndPublishColorImage();
      retrieveAndPublishDepthImage();
   }

   private void retrieveAndPublishColorImage()
   {
      // Retrieve image
      checkError("sl_retrieve_image", sl_retrieve_image(cameraID, colorImagePointer, SL_VIEW_LEFT, SL_MEM_CPU, imageWidth, imageHeight));

      // Convert to BGR and encode to jpeg
      // TODO: Make this work with GpuMats
      Mat colorImageBGRA = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, colorImagePointer, sl_mat_get_step_bytes(colorImagePointer, SL_MEM_CPU));
      Mat colorImageBGR = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC3);
      opencv_imgproc.cvtColor(colorImageBGRA, colorImageBGR, opencv_imgproc.COLOR_BGRA2BGR);

      GpuMat bgrImageGpuMat = new GpuMat(imageHeight, imageWidth, opencv_core.CV_8UC3);
      bgrImageGpuMat.upload(colorImageBGR);

      BytePointer colorJPEGPointer = new BytePointer((long) imageHeight * imageWidth);
      imageEncoder.encodeBGR(bgrImageGpuMat.data(), colorJPEGPointer, imageWidth, imageHeight, bgrImageGpuMat.step());

      colorImageAcquisitionTime.set(Instant.now());

      // Publish image
      ImageMessageDataPacker imageMessageDataPacker = new ImageMessageDataPacker(colorJPEGPointer.limit());
      imageMessageDataPacker.pack(colorImageMessage, colorJPEGPointer);
      MessageTools.toMessage(colorImageAcquisitionTime.get(), colorImageMessage.getAcquisitionTime());
      colorImageMessage.setFocalLengthXPixels(zedCalibrationParameters.left_cam().fx());
      colorImageMessage.setFocalLengthYPixels(zedCalibrationParameters.left_cam().fy());
      colorImageMessage.setPrincipalPointXPixels(zedCalibrationParameters.left_cam().cx());
      colorImageMessage.setPrincipalPointYPixels(zedCalibrationParameters.left_cam().cy());
      colorImageMessage.setImageWidth(imageWidth);
      colorImageMessage.setImageHeight(imageHeight);
      // TODO: Get ZED's FramePose3D on robot
      colorImageMessage.getPosition().set(worldFramePose.getPosition());
      colorImageMessage.getOrientation().set(worldFramePose.getOrientation());
      colorImageMessage.setSequenceNumber(colorImageSequenceNumber++);
      colorImageMessage.setDepthDiscretization(DEPTH_DISCRETIZATION);
      CameraModel.PINHOLE.packMessageFormat(colorImageMessage);
      ImageMessageFormat.COLOR_JPEG_BGR8.packMessageFormat(colorImageMessage);
      ros2ColorImagePublisher.publish(colorImageMessage);

      // Close stuff
      colorJPEGPointer.close();
      bgrImageGpuMat.release();
      bgrImageGpuMat.close();
      colorImageBGR.close();
      colorImageBGRA.close();
   }

   private void retrieveAndPublishDepthImage()
   {
      checkError("sl_retrieve_measure", sl_retrieve_measure(cameraID, depthImagePointer, SL_MEASURE_DEPTH_U16_MM, SL_MEM_CPU, imageWidth, imageHeight));

      Mat depthImage16UC1 = new Mat(imageHeight, imageWidth, opencv_core.CV_16UC1, depthImagePointer, sl_mat_get_step_bytes(depthImagePointer, SL_MEM_CPU));

      BytePointer depthPNGPointer = new BytePointer();
      OpenCVTools.compressImagePNG(depthImage16UC1, depthPNGPointer);

      depthImageAcquisitionTime.set(Instant.now());

      ImageMessageDataPacker imageMessageDataPacker = new ImageMessageDataPacker(depthPNGPointer.limit());
      imageMessageDataPacker.pack(depthImageMessage, depthPNGPointer);
      MessageTools.toMessage(depthImageAcquisitionTime.get(), depthImageMessage.getAcquisitionTime());
      depthImageMessage.setFocalLengthXPixels(zedCalibrationParameters.left_cam().fx());
      depthImageMessage.setFocalLengthYPixels(zedCalibrationParameters.left_cam().fy());
      depthImageMessage.setPrincipalPointXPixels(zedCalibrationParameters.left_cam().cx());
      depthImageMessage.setPrincipalPointYPixels(zedCalibrationParameters.left_cam().cy());
      depthImageMessage.setImageWidth(imageWidth);
      depthImageMessage.setImageHeight(imageHeight);
      depthImageMessage.getPosition().set(worldFramePose.getPosition());
      depthImageMessage.getOrientation().set(worldFramePose.getOrientation());
      depthImageMessage.setSequenceNumber(depthImageSequenceNumber++);
      depthImageMessage.setDepthDiscretization(DEPTH_DISCRETIZATION);
      CameraModel.PINHOLE.packMessageFormat(depthImageMessage);
      ImageMessageFormat.DEPTH_PNG_16UC1.packMessageFormat(depthImageMessage);
      ros2DepthImagePublisher.publish(depthImageMessage);

      // Close stuff
      depthPNGPointer.close();
      depthImage16UC1.close();
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
      new ZED2ColorStereoDepthPublisher(0, PerceptionAPI.ZED2_STEREO_COLOR, PerceptionAPI.ZED2_DEPTH);
   }
}
