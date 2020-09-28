package us.ihmc.avatar.sensors.realsense;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessagePubSubType;
import sensor_msgs.msg.dds.CompressedImage;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2TopicHz;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class RealsenseD435PointCloudROS1Bridge extends AbstractRosTopicSubscriber<sensor_msgs.PointCloud2>
{
   private static final int MAX_POINTS = 50000;

   private final IHMCROS2Publisher<StereoVisionPointCloudMessage> publisher;
   private final ROS2TopicHz hz = new ROS2TopicHz();

   public RealsenseD435PointCloudROS1Bridge(RosMainNode ros1Node, ROS2Node ros2Node)
   {
      super(sensor_msgs.PointCloud2._TYPE);

      String ros1Topic = "/depthcam/depth/color/points";
      LogTools.info("Subscribing ROS 1: {}", ros1Topic);
      ros1Node.attachSubscriber(ros1Topic, this);

      ROS2Topic<StereoVisionPointCloudMessage> ros2Topic = ROS2Tools.D435_POINT_CLOUD;
      LogTools.info("Publishing ROS 2: {}", ros2Topic.getName());
      publisher = ROS2Tools.createPublisher(ros2Node, ros2Topic, ROS2QosProfile.DEFAULT());


   }

   @Override
   public void onNewMessage(sensor_msgs.PointCloud2 ros1PointCloud)
   {
      hz.ping();
      try
      {
         boolean hasColors = true;
         PointCloudData pointCloudData = new PointCloudData(ros1PointCloud, MAX_POINTS, hasColors);

         long robotTimestamp;

//         if (rosClockCalculator == null)
//         {
//            robotTimestamp = pointCloudData.getTimestamp();
//            robotConfigurationDataBuffer.updateFullRobotModelWithNewestData(fullRobotModel, null);
//         }
//         else
//         {
//            long rosTimestamp = pointCloudData.getTimestamp();
//            robotTimestamp = rosClockCalculator.computeRobotMonotonicTime(rosTimestamp);
//            boolean waitForTimestamp = true;
//            if (robotConfigurationDataBuffer.getNewestTimestamp() == -1)
//               return;
//
//            boolean success = robotConfigurationDataBuffer.updateFullRobotModel(waitForTimestamp, robotTimestamp, fullRobotModel, null) != -1;
//
//            if (!success)
//               return;
//         }
//
//         if (stereoVisionTransformer != null)
//         {
//            stereoVisionTransformer.computeTransformToWorld(fullRobotModel, transformToWorld, sensorPose);
//            pointCloudData.applyTransform(transformToWorld);
//         }
//         else
//         {
//            if (!stereoVisionPointsFrame.isWorldFrame())
//            {
//               stereoVisionPointsFrame.getTransformToDesiredFrame(transformToWorld, worldFrame);
//               pointCloudData.applyTransform(transformToWorld);
//            }
//
//            fullRobotModel.getHeadBaseFrame().getTransformToDesiredFrame(transformToWorld, worldFrame);
//            sensorPose.set(transformToWorld);
//         }
//
//         if (enableFilter.get())
//         {
//            double timeDiff = Conversions.nanosecondsToSeconds(robotTimestamp - previousTimeStamp);
//            double linearVelocity = sensorPose.getPosition().distance(previousSensorPosition) / timeDiff;
//            double angularVelocity = sensorPose.getOrientation().distance(previousSensorOrientation) / timeDiff;
//
//            previousTimeStamp = robotTimestamp;
//            previousSensorPosition.set(sensorPose.getPosition());
//            previousSensorOrientation.set(sensorPose.getOrientation());
//
//            if (linearVelocity > linearVelocityThreshold.get() || angularVelocity > angularVelocityThreshold.get())
//               return;
//         }
//
//         if (collisionFilter != null)
//            collisionFilter.update();
//         if (rangeFilter != null)
//            rangeFilter.setSensorPosition(sensorPose.getPosition());
//
//         long startTime = System.nanoTime();
//         StereoVisionPointCloudMessage ros2PointCloud = pointCloudData.toStereoVisionPointCloudMessage(minimumResolution, activeFilters);
//
//         if (message == null)
//            return; // TODO Sometimes the LZ4 compression fails. Need to figure it out, for now just giving up.
//
//         ros2PointCloud.getSensorPosition().set(sensorPose.getPosition());
//         ros2PointCloud.getSensorOrientation().set(sensorPose.getOrientation());
//         long endTime = System.nanoTime();
//
//         publisher.publish(ros2PointCloud);
      }
      catch (Exception e)
      {
         LogTools.error(e.getMessage());
         e.printStackTrace();
      }
   }
}
