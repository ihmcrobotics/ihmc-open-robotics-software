package us.ihmc.atlas.ros;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.ui.tools.JavaFXROS2PointCloudViewer;

import static us.ihmc.pubsub.DomainFactory.PubSubImplementation.FAST_RTPS;

public class AtlasMultiSensePointCloudViewer
{
   public static void main(String[] args)
   {
      new JavaFXROS2PointCloudViewer(ROS2Tools.createROS2Node(FAST_RTPS, "point_cloud_viewer"), ROS2Tools.MULTISENSE_LIDAR_POINT_CLOUD);
   }
}
