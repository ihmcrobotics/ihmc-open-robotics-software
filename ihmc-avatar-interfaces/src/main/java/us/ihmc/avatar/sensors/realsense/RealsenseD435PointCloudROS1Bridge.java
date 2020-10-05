package us.ihmc.avatar.sensors.realsense;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudMessageTools;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RealsenseD435PointCloudROS1Bridge extends AbstractRosTopicSubscriber<sensor_msgs.PointCloud2>
{
   private static final int MAX_POINTS = 50000;

   private final IHMCROS2Publisher<StereoVisionPointCloudMessage> publisher;
   private final RemoteSyncedRobotModel syncedRobot;
   private final MovingReferenceFrame pelvisFrame;
   private final TransformReferenceFrame realsenseSensorFrame;
   private final FramePose3D tempSensorFramePose = new FramePose3D();
   private final RigidBodyTransform pelvisToSensorTransform;
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   public RealsenseD435PointCloudROS1Bridge(DRCRobotModel robotModel, RosMainNode ros1Node, ROS2Node ros2Node, RigidBodyTransform pelvisToSensorTransform)
   {
      super(sensor_msgs.PointCloud2._TYPE);

      this.pelvisToSensorTransform = pelvisToSensorTransform;

      syncedRobot = new RemoteSyncedRobotModel(robotModel, ros2Node);
      pelvisFrame = syncedRobot.getReferenceFrames().getPelvisFrame();
      realsenseSensorFrame = new TransformReferenceFrame("Realsense", pelvisFrame, pelvisToSensorTransform);

      String ros1Topic = "/depthcam/depth/color/points";
      LogTools.info("Subscribing ROS 1: {}", ros1Topic);
      ros1Node.attachSubscriber(ros1Topic, this);

      ROS2Topic<StereoVisionPointCloudMessage> ros2Topic = ROS2Tools.D435_POINT_CLOUD;
      LogTools.info("Publishing ROS 2: {}", ros2Topic.getName());
      publisher = ROS2Tools.createPublisher(ros2Node, ros2Topic, ROS2QosProfile.DEFAULT());


      // subscribe to /multisense/stamped_pps?
   }

//   private RobotROSClockCalculator getROSClockCalculator()
//   {
//      DRCROSPPSTimestampOffsetProvider timestampOffsetProvider = null;
//
//      if (target == RobotTarget.REAL_ROBOT)
//      {
//         timestampOffsetProvider = AtlasPPSTimestampOffsetProvider.getInstance(sensorInformation);
//      }
//
//      if (AtlasSensorInformation.SEND_ROBOT_DATA_TO_ROS)
//      {
//         if (target == RobotTarget.SCS)
//         {
//            timestampOffsetProvider = new SimulationRosClockPPSTimestampOffsetProvider();
//         }
//      }
//
//      if (timestampOffsetProvider == null)
//         timestampOffsetProvider = new DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider();
//
//      return new RobotROSClockCalculatorFromPPSOffset(timestampOffsetProvider);
//   }

   @Override
   public void onNewMessage(sensor_msgs.PointCloud2 ros1PointCloud)
   {
      try
      {
         if (!syncedRobot.getDataReceptionTimerSnapshot().isRunning(2.0))
            return;

         syncedRobot.update();

         boolean hasColors = true;
         PointCloudData pointCloudData = new PointCloudData(ros1PointCloud, MAX_POINTS, hasColors);


         pelvisFrame.getTransformToDesiredFrame(transformToWorld, ReferenceFrame.getWorldFrame());
         transformToWorld.multiply(pelvisToSensorTransform);
         tempSensorFramePose.set(transformToWorld);

         pointCloudData.applyTransform(transformToWorld);
         ArrayList<Point3D> pointCloud = new ArrayList<>();
         for (int i = 0; i < pointCloudData.getNumberOfPoints(); i++)
         {
            pointCloud.add(new Point3D(pointCloudData.getPointCloud()[i]));
         }

         //         tempSensorFramePose.setToZero(realsenseSensorFrame);
         //         tempSensorFramePose.changeFrame(ReferenceFrame.getWorldFrame());

         StereoVisionPointCloudMessage message = PointCloudMessageTools.toStereoVisionPointCloudMessage(pointCloud, tempSensorFramePose);
         LogTools.info("Publishing point cloud of size {}", message.getNumberOfPoints());
         publisher.publish(message);

//         long robotTimestamp;

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

//         publisher.publish(ros2PointCloud);
      }
      catch (Exception e)
      {
         LogTools.error(e.getMessage());
         e.printStackTrace();
      }
   }
}
