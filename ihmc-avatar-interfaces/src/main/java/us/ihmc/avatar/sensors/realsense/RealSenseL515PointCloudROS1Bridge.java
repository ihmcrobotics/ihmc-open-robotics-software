package us.ihmc.avatar.sensors.realsense;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import sensor_msgs.PointCloud2;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudMessageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.SingleThreadSizeOneQueueExecutor;
import us.ihmc.tools.Timer;
import us.ihmc.tools.UnitConversions;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RealSensePointCloudSubscriber;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;

public class RealSenseL515PointCloudROS1Bridge
{

   private Point3D32[] points;

   private static final int MAX_POINTS = 40000;
   private static final double MIN_PUBLISH_PERIOD = UnitConversions.hertzToSeconds(3.0);
   private static final String l515PointCloudTopic = "/camera/depth/color/points";

   private final Timer throttleTimer = new Timer();
   private final FramePose3D sensorPose = new FramePose3D();
   private final IHMCROS2Publisher<StereoVisionPointCloudMessage> stereoVisionPublisher;
   private final SingleThreadSizeOneQueueExecutor executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

   public RealSenseL515PointCloudROS1Bridge(RosMainNode rosMainNode, ROS2Node ros2Node)
   {
      // stereoVisionPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.IHMC_ROOT);
      ROS2Topic<StereoVisionPointCloudMessage> ros2Topic = ROS2Tools.MULTISENSE_STEREO_POINT_CLOUD;
      stereoVisionPublisher = ROS2Tools.createPublisher(ros2Node, ros2Topic, ROS2QosProfile.DEFAULT());

      new RealSensePointCloudSubscriber(rosMainNode, l515PointCloudTopic)
      {
         @Override
         protected void cloudReceived(PointCloud2 cloud)
         {
            executor.submitTask(() -> repackAndPublish(cloud));
         }
      };

      rosMainNode.execute();
   }

   private void repackAndPublish(PointCloud2 cloud)
   {
      throttleTimer.sleepUntilExpiration(MIN_PUBLISH_PERIOD);
      throttleTimer.reset();

      PointCloudData pointCloudData = new PointCloudData(cloud, MAX_POINTS, false);
//      LogTools.info("ROS1 PointCloud Received: {}", pointCloudData.getPointCloud().length);

      RigidBodyTransform transform = new RigidBodyTransform();
      ArrayList<Point3D> pointCloud = new ArrayList<>();
      for (int i = 0; i < pointCloudData.getNumberOfPoints(); i++)
      {
         Point3D point = pointCloudData.getPointCloud()[i];
         pointCloud.add(new Point3D(point.getZ(), -point.getX(), -point.getY()));
      }

      StereoVisionPointCloudMessage message = PointCloudMessageTools.toStereoVisionPointCloudMessage(pointCloud, sensorPose);
//      LogTools.info("ROS2 PointCloud Published: {}", message.getNumberOfPoints());
      stereoVisionPublisher.publish(message);
   }

   public static void main(String[] args) throws URISyntaxException
   {
      RosMainNode rosMainNode = new RosMainNode(new URI("http://localhost:11311"), "RealSenseL515DataPublisher", true);
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "video_viewer");
      new RealSenseL515PointCloudROS1Bridge(rosMainNode, ros2Node);
   }
}
