package us.ihmc.avatar.sensors.realsense;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.euclid.transform.RigidBodyTransform;
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
   private final RealsenseD435PointCloudROS1Bridge pointCloudBridge;
   private final RealsensePlanarRegionROS1Bridge planarRegionBridge;

   public RealsenseROS1Bridge(DRCRobotModel robotModel, RigidBodyTransform pelvisToSensorTransform)
   {
      URI masterURI = NetworkParameters.getROSURI();
      LogTools.info("Connecting to ROS 1 master URI: {}", masterURI);
      RosMainNode ros1Node = new RosMainNode(masterURI, "ImagePublisher", true);

      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "imagePublisherNode");

      d435VideoBridge = new RealsenseVideoROS1Bridge(ros1Node, ros2Node, RosTools.D435_VIDEO, ROS2Tools.D435_VIDEO);
      l515VideoBridge = new RealsenseVideoROS1Bridge(ros1Node, ros2Node, RosTools.L515_VIDEO, ROS2Tools.L515_VIDEO);
      pointCloudBridge = new RealsenseD435PointCloudROS1Bridge(robotModel, ros1Node, ros2Node, pelvisToSensorTransform);
      planarRegionBridge = new RealsensePlanarRegionROS1Bridge(robotModel, ros1Node, ros2Node, RosTools.MAPSENSE_REGIONS, ROS2Tools.MAPSENSE_REGIONS);

      ros1Node.execute();
   }
}
