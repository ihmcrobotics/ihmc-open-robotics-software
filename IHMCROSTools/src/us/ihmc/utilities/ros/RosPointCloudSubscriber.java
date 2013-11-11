package us.ihmc.utilities.ros;

import sensor_msgs.LaserScan;
import sensor_msgs.PointCloud2;

public abstract class RosPointCloudSubscriber extends AbstractRosTopicSubscriber<PointCloud2>
{
   private boolean DEBUG = false;

   public RosPointCloudSubscriber()
   {
      super(sensor_msgs.PointCloud2._TYPE);
   }

   public void onNewMessage(LaserScan message)
   {

   }

}
