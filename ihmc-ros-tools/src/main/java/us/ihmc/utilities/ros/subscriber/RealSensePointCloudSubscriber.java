package us.ihmc.utilities.ros.subscriber;

import sensor_msgs.PointCloud2;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.utilities.ros.RosMainNode;

public abstract class RealSensePointCloudSubscriber extends AbstractRosTopicSubscriber<PointCloud2>
{

   private PointCloud2 latestPointCloud2;
   private Point3D32[] points;

   public RealSensePointCloudSubscriber(RosMainNode rosMainNode, String l515PointCloudTopic)
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
