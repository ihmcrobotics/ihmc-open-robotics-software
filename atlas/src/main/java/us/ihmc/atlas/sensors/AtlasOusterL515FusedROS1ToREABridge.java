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
