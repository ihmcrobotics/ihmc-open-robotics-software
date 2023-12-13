package us.ihmc.avatar.sensors.realsense;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;

import java.net.URI;

public class RealsenseROS1Bridge
{
   private final RealsenseVideoROS1Bridge d435VideoBridge;
   private final RealsenseVideoROS1Bridge l515VideoBridge;
   private final RealsensePointCloudROS1Bridge d435PointCloudBridge;
   private final RealsensePointCloudROS1Bridge l515PointCloudBridge;
   private final MapSensePlanarRegionROS1Bridge l515PlanarRegionBridge;

   public RealsenseROS1Bridge(DRCRobotModel robotModel)
   {
      URI masterURI = NetworkParameters.getROSURI();
      LogTools.info("Connecting to ROS 1 master URI: {}", masterURI);
      RosMainNode ros1Node = new RosMainNode(masterURI, "ImagePublisher", true);

      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "imagePublisherNode");

      d435VideoBridge = new RealsenseVideoROS1Bridge(ros1Node, ros2Node, RosTools.D435_VIDEO, PerceptionAPI.D435_VIDEO, 25.0);
      l515VideoBridge = new RealsenseVideoROS1Bridge(ros1Node, ros2Node, RosTools.L515_VIDEO, PerceptionAPI.L515_VIDEO, 25.0);
      d435PointCloudBridge = new RealsensePointCloudROS1Bridge(robotModel,
                                                               ros1Node,
                                                               ros2Node,
                                                               RosTools.D435_POINT_CLOUD,
                                                               PerceptionAPI.D435_POINT_CLOUD);
      l515PointCloudBridge = new RealsensePointCloudROS1Bridge(robotModel,
                                                               ros1Node,
                                                               ros2Node,
                                                               RosTools.L515_POINT_CLOUD,
                                                               PerceptionAPI.L515_POINT_CLOUD);
      l515PlanarRegionBridge = new MapSensePlanarRegionROS1Bridge(robotModel,
                                                                  ros1Node,
                                                                  ros2Node);

      ros1Node.execute();
   }
}
