package us.ihmc.avatar.sensors.realsense;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.utilities.ros.RosMainNode;

import java.net.URI;

public class RealsenseD435ROS1Bridge
{
   private final RealsenseD435VideoROS1Bridge videoBridge;
   private final RealsenseD435PointCloudROS1Bridge pointCloudBridge;

   public RealsenseD435ROS1Bridge(DRCRobotModel robotModel, RigidBodyTransform pelvisToSensorTransform)
   {
      URI masterURI = NetworkParameters.getROSURI();
      LogTools.info("Connecting to ROS 1 master URI: {}", masterURI);
      RosMainNode ros1Node = new RosMainNode(masterURI, "ImagePublisher", true);

      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "imagePublisherNode");

      videoBridge = new RealsenseD435VideoROS1Bridge(ros1Node, ros2Node);
      pointCloudBridge = new RealsenseD435PointCloudROS1Bridge(robotModel, ros1Node, ros2Node, pelvisToSensorTransform);

      ros1Node.execute();
   }
}
