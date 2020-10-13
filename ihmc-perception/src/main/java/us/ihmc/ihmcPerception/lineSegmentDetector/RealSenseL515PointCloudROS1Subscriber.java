package us.ihmc.ihmcPerception.lineSegmentDetector;

import sensor_msgs.PointCloud2;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public abstract class RealSenseL515PointCloudROS1Subscriber extends AbstractRosTopicSubscriber<PointCloud2>
{
   private String l515PointCloudTopic = "/camera/depth/color/points";

   private PointCloud2 latestPointCloud2;
   private Point3D32[] points;

   public RealSenseL515PointCloudROS1Subscriber(RosMainNode rosMainNode)
   {
      super(PointCloud2._TYPE);
      rosMainNode.attachSubscriber(l515PointCloudTopic, this);
   }

   @Override
   public void onNewMessage(PointCloud2 pointCloud)
   {
      latestPointCloud2 = pointCloud;
      cloudReceived(pointCloud);
   }

   protected abstract void cloudReceived(PointCloud2 cloud);
}
