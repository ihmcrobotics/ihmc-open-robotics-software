package us.ihmc.avatar.sensors.realsense;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import sensor_msgs.PointCloud2;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
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
import us.ihmc.utilities.ros.subscriber.RealSenseL515PointCloudROS1Subscriber;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;

public class RealSenseL515PointCloudROS1Bridge
{

   private Point3D32[] points;

   private static final int MAX_POINTS = 5000;
   private static final double MIN_PUBLISH_PERIOD = UnitConversions.hertzToSeconds(3.0);
   private static final String l515PointCloudTopic = "/camera/depth/color/points";

   private final Timer throttleTimer = new Timer();
   private final FramePose3D sensorPose = new FramePose3D();
   private final IHMCROS2Publisher<StereoVisionPointCloudMessage> stereoVisionPublisher;
   private final SingleThreadSizeOneQueueExecutor executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

   public RealSenseL515PointCloudROS1Bridge(RosMainNode rosMainNode, ROS2Node ros2Node) throws URISyntaxException
   {
      // stereoVisionPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.IHMC_ROOT);
      ROS2Topic<StereoVisionPointCloudMessage> ros2Topic = ROS2Tools.D435_POINT_CLOUD;
      stereoVisionPublisher = ROS2Tools.createPublisher(ros2Node, ros2Topic, ROS2QosProfile.DEFAULT());

      new RealSenseL515PointCloudROS1Subscriber(rosMainNode, l515PointCloudTopic)
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

      LogTools.info("ROS1 PointCloud Received.");

      PointCloudData pointCloudData = new PointCloudData(cloud, MAX_POINTS, false);

      ArrayList<Point3D> pointCloud = new ArrayList<>();
      for (int i = 0; i < pointCloudData.getNumberOfPoints(); i++)
      {
         pointCloud.add(new Point3D(pointCloudData.getPointCloud()[i]));
      }

      StereoVisionPointCloudMessage message = PointCloudMessageTools.toStereoVisionPointCloudMessage(pointCloud, sensorPose);
      LogTools.info("Publishing point cloud of size {}", message.getNumberOfPoints());
      stereoVisionPublisher.publish(message);

      //        RosPointCloudSubscriber.UnpackedPointCloud unpackedPointCloud = RosPointCloudSubscriber.unpackPointsAndIntensities(cloud);
      //        points = new Point3D32[unpackedPointCloud.getPoints().length];
      //        int i = 0;
      //        for (Point3D point : unpackedPointCloud.getPoints()) {
      //            Point3D32 pointf = new Point3D32(point.getX32(), point.getY32(), point.getZ32());
      //            points[i] = pointf;
      //            i++;
      //        }
   }

   public static void main(String[] args) throws URISyntaxException
   {
      RosMainNode rosMainNode = new RosMainNode(new URI("http://localhost:11311"), "RealSenseL515DataPublisher", true);
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "video_viewer");
      new RealSenseL515PointCloudROS1Bridge(rosMainNode, ros2Node);
   }
}
