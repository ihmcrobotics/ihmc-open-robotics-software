package us.ihmc.sensors;

import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.zed.SL_InitParameters;
import org.bytedeco.zed.SL_RuntimeParameters;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import static org.bytedeco.zed.global.zed.*;

public class ZED2ColorStereoDepthPublisher
{
   private int cameraID;
   private final SL_InitParameters zedInitializationParameters = new SL_InitParameters();
   private int cameraState;
   private final SL_RuntimeParameters zedRuntimeParameters = new SL_RuntimeParameters();

   private int imageWidth;
   private int imageHeight;
   private IntPointer colorImagePointer;
   private IntPointer depthImagePoitner;
   private IntPointer pointCloudPointer;

   private final ROS2Topic<ImageMessage> colorTopic;
   private final ROS2Topic<ImageMessage> depthTopic;
   private final ROS2Node ros2Node;

   public ZED2ColorStereoDepthPublisher(int cameraID,
                                        ROS2Topic<ImageMessage> colorTopic,
                                        ROS2Topic<ImageMessage> depthTopic)
   {
      this.cameraID = cameraID;

      sl_create_camera(cameraID);

      zedInitializationParameters.camera_fps(30);
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

      cameraState = sl_open_camera(cameraID, zedInitializationParameters, 0, "", "", 0, "", "", "")
      if (cameraState != 0)
      {
         throw new RuntimeException("Failed to open ZED 2 camera");
      }

      zedRuntimeParameters.enable_depth(true);
      zedRuntimeParameters.confidence_threshold(95);
      zedRuntimeParameters.reference_frame(SL_REFERENCE_FRAME_CAMERA);
      zedRuntimeParameters.texture_confidence_threshold(100);
      zedRuntimeParameters.remove_saturated_areas(true);

      imageWidth = sl_get_width(cameraID);
      imageHeight = sl_get_height(cameraID);

      colorImagePointer = new IntPointer(sl_mat_create_new(imageWidth, imageHeight, SL_MAT_TYPE_U8_C4, SL_MEM_GPU));

      this.colorTopic = colorTopic;
      this.depthTopic = depthTopic;
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "zed2_node");


      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, getClass().getName() + "-Shutdown"));
   }

   public void destroy()
   {

   }

}
