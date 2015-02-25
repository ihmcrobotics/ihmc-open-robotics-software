package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import java.util.Arrays;

import sensor_msgs.PointCloud2;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.subscriber.RosTopicSubscriberInterface;

public class RosPointCloudReceiver
{
   private final ReferenceFrame scanFrame;
   private final PointCloudDataReceiver pointCloudDataReceiver;

   public RosPointCloudReceiver(DRCRobotLidarParameters lidarParams, RosMainNode rosMainNode, ReferenceFrame scanFrame,
         PointCloudDataReceiver pointCloudDataReceiver)
   {
      this.scanFrame = scanFrame;
      this.pointCloudDataReceiver = pointCloudDataReceiver;

      RosTopicSubscriberInterface<PointCloud2> rosPointCloudSubscriber = new AbstractRosTopicSubscriber<PointCloud2>(PointCloud2._TYPE)
      {

         @Override
         public void onNewMessage(PointCloud2 message)
         {
            RosPointCloud pointCloud = new RosPointCloud(message);
            long[] timestamps = new long[pointCloud.points.size()];
            long time = message.getHeader().getStamp().totalNsecs();
            Arrays.fill(timestamps, time);

            RosPointCloudReceiver.this.pointCloudDataReceiver.receivedPointCloudData(RosPointCloudReceiver.this.scanFrame, null, timestamps, pointCloud.points);
         }
      };
      rosMainNode.attachSubscriber(lidarParams.getRosTopic(), rosPointCloudSubscriber);

   }
}
