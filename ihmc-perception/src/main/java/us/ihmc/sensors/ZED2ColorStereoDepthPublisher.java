package us.ihmc.sensors;

import org.bytedeco.zed.SL_InitParameters;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import static org.bytedeco.zed.global.zed.sl_create_camera;

public class ZED2ColorStereoDepthPublisher
{
   private int cameraID;
   private SL_InitParameters

   private final ROS2Topic<ImageMessage> colorTopic;
   private final ROS2Topic<ImageMessage> depthTopic;
   private final ROS2Node ros2Node;

   public ZED2ColorStereoDepthPublisher(int cameraID,
                                        ROS2Topic<ImageMessage> colorTopic,
                                        ROS2Topic<ImageMessage> depthTopic)
   {
      this.cameraID = cameraID;

      sl_create_camera(cameraID);





      this.colorTopic = colorTopic;
      this.depthTopic = depthTopic;
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "zed2_node");


      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, getClass().getName() + "-Shutdown"));
   }

   public void destroy()
   {

   }

}
