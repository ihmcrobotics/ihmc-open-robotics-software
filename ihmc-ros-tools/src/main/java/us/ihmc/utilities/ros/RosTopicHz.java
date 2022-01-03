package us.ihmc.utilities.ros;

import org.apache.commons.lang3.mutable.MutableInt;
import sensor_msgs.PointCloud2;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.time.FrequencyStatisticPrinter;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class RosTopicHz
{
   public static void main(String[] args)
   {
      String topic = RosTools.OUSTER_POINT_CLOUD;
      MutableInt numberOfPointsInAScan = new MutableInt();
      FrequencyStatisticPrinter hz = new FrequencyStatisticPrinter(() ->
      {
         LogTools.info("Number of points in a scan: {}", numberOfPointsInAScan.getValue());
      });
      ROS1Helper helper = new ROS1Helper("hz");
      AbstractRosTopicSubscriber<PointCloud2> subscriber = new AbstractRosTopicSubscriber<PointCloud2>(PointCloud2._TYPE)
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud2)
         {
            hz.ping();
            numberOfPointsInAScan.setValue(pointCloud2.getHeight() * pointCloud2.getWidth());
         }
      };
      helper.attachSubscriber(topic, subscriber);

      ThreadTools.sleepForever();
   }
}
