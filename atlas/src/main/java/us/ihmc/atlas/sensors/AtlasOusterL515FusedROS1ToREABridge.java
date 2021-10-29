package us.ihmc.atlas.sensors;

import controller_msgs.msg.dds.LidarScanMessage;
import sensor_msgs.PointCloud2;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.tools.thread.Throttler;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber.UnpackedPointCloud;

import java.util.concurrent.atomic.AtomicReference;

/**
 * This class is processing the
 *
 * L515:
 * ~180000 points @ 8 Hz (neither number is exact here and is approximate)
 * x, y, z, rgb - all 32-bit floats
 * 16 bytes per point
 * 23 MB/s bandwidth
 *
 * Ouster:
 * 131072 points @ 10 Hz (number of points per message seems pretty contant)
 * X, Y, Z, R, G, B, A, 0.01, 1.0, 0.0 - 32-bit floats
 * 48 bytes per point
 * 63 MB/s bandwidth
 *
 * Combined:
 * 311072 points @ 10 Hz
 * x, y, z - 32-bit floats
 * 12 bytes per point
 * 37 MB/s bandwidth
 *
 * For 144 Hz consumption:
 * 100 ms / 6.9 ms = ~14
 * Segmented by 14:
 * 22220 @ 140 Hz
 * x, y, z - 32-bit floats
 * 12 bytes per point
 * 37 MB/s bandwidth
 */
public class AtlasOusterL515FusedROS1ToREABridge
{
   private static final double REA_OUTPUT_FREQUENCY = UnitConversions.hertzToSeconds(10.0);

   public AtlasOusterL515FusedROS1ToREABridge()
   {
      RosMainNode ros1Node = RosTools.createRosNode(NetworkParameters.getROSURI(), "ousterl515_to_rea");
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ousterl515_to_rea");
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT);
      CommunicationHelper ros2Helper = new CommunicationHelper(robotModel, ros2Node);
      ROS2SyncedRobotModel syncedRobot = ros2Helper.newSyncedRobot();
      RigidBodyTransform l515ToWorldTransform = new RigidBodyTransform();
      RigidBodyTransform ousterToWorldTransform = new RigidBodyTransform();
      Throttler throttler = new Throttler();
      AtomicReference<PointCloud2> latestL515PointCloud = new AtomicReference<>();

      ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor("OusterL515ToREABridge", true);

      // TODO: Add remote interface to set:
      // - Wait for robot pose
      // - rates, bundling
      // - which sensors to fuse
      // - topics

      AbstractRosTopicSubscriber<PointCloud2> l515Subscriber = new AbstractRosTopicSubscriber<PointCloud2>(PointCloud2._TYPE)
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud2)
         {
            latestL515PointCloud.set(pointCloud2);
         }
      };
      ros1Node.attachSubscriber(RosTools.L515_POINT_CLOUD, l515Subscriber);
      AbstractRosTopicSubscriber<PointCloud2> ousterSubscriber = new AbstractRosTopicSubscriber<PointCloud2>(PointCloud2._TYPE)
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud2)
         {
            if (throttler.run(REA_OUTPUT_FREQUENCY))
            {
               executor.submit(() ->
               {
                  syncedRobot.update();
//                  if (syncedRobot.getDataReceptionTimerSnapshot().isRunning(3.0))
                  {
                     FramePose3DReadOnly l515Pose = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getSteppingCameraFrame);
                     l515Pose.get(l515ToWorldTransform);

                     FramePose3DReadOnly ousterPose = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getOusterLidarFrame);
                     ousterPose.get(ousterToWorldTransform);

                     PointCloud2 latestL515PointCloud2 = latestL515PointCloud.get();
                     Point3D[] l515Points = null;
                     if (latestL515PointCloud2 != null)
                     {
                        UnpackedPointCloud unpackPointsAndIntensities = RosPointCloudSubscriber.unpackPointsAndIntensities(latestL515PointCloud2);
                        l515Points = unpackPointsAndIntensities.getPoints();
                        for (int i = 0; i < l515Points.length; i++)
                        {
                           double x = l515Points[i].getX();
                           double y = l515Points[i].getY();
                           double z = l515Points[i].getZ();
                           l515Points[i].set(z, -x, -y);
                           l515Points[i].applyTransform(l515ToWorldTransform);
                        }
                     }

                     PointCloudData pointCloudData = new PointCloudData(pointCloud2, 1600000, false);
                     pointCloudData.applyTransform(ousterToWorldTransform);
                     LidarScanMessage lidarScanMessage = pointCloudData.toLidarScanMessage(null, l515Points);
                     lidarScanMessage.getLidarPosition().set(ousterPose.getPosition());
                     lidarScanMessage.getLidarOrientation().set(ousterPose.getOrientation());
                     lidarScanMessage.setSensorPoseConfidence(1.0);
                     ros2Helper.publish(ROS2Tools.MULTISENSE_LIDAR_SCAN, lidarScanMessage);
                  }
               });
            }
         }
      };
      ros1Node.attachSubscriber(RosTools.OUSTER_POINT_CLOUD, ousterSubscriber);
      ros1Node.execute();

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         executor.destroy();
         ros1Node.shutdown();
      }, "IHMC-OusterROS1ToREABridgeShutdown"));
   }

   public static void main(String[] args)
   {
      new AtlasOusterL515FusedROS1ToREABridge();
   }
}
