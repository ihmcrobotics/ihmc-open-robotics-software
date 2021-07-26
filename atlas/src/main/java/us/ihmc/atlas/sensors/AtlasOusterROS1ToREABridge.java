package us.ihmc.atlas.sensors;

import controller_msgs.msg.dds.LidarScanMessage;
import sensor_msgs.PointCloud2;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class AtlasOusterROS1ToREABridge
{
   public AtlasOusterROS1ToREABridge()
   {
      RosMainNode ros1Node = RosTools.createRosNode(NetworkParameters.getROSURI(), "ouster_to_rea");
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ouster_to_rea");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor("OusterToREABridge", true);

      AbstractRosTopicSubscriber<PointCloud2> ousterSubscriber = new AbstractRosTopicSubscriber<PointCloud2>(PointCloud2._TYPE)
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud2)
         {
            executor.submit(() ->
            {
               PointCloudData pointCloudData = new PointCloudData(pointCloud2, 1600000, false);
               LidarScanMessage lidarScanMessage = pointCloudData.toLidarScanMessage();
               ros2Helper.publish(ROS2Tools.MULTISENSE_LIDAR_SCAN, lidarScanMessage);
            });
         }
      };
      ros1Node.attachSubscriber(RosTools.OUSTER_POINT_CLOUD, ousterSubscriber);
      ros1Node.execute();

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         executor.destroy();
         ros1Node.shutdown();
      }, "IHMC-OusterROS1ToREABridgeShutdown"));

//      ThreadTools.join();
   }

   public static void main(String[] args)
   {
      new AtlasOusterROS1ToREABridge();
   }
}
