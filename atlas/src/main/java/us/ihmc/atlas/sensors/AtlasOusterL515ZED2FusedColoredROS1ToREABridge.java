package us.ihmc.atlas.sensors;

import perception_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import org.bytedeco.javacpp.BytePointer;
import org.jboss.netty.buffer.ChannelBuffer;
import sensor_msgs.Image;
import sensor_msgs.PointCloud2;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.perception.depthData.PointCloudData;
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

import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicReference;

public class AtlasOusterL515ZED2FusedColoredROS1ToREABridge
{
   private static final double REA_OUTPUT_FREQUENCY = UnitConversions.hertzToSeconds(10.0);

   public AtlasOusterL515ZED2FusedColoredROS1ToREABridge()
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
      AtomicReference<Image> latestZED2Image = new AtomicReference<>();

      ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor("OusterL515ToREABridge", true);

      AbstractRosTopicSubscriber<Image> zed2LeftEyeSubscriber = new AbstractRosTopicSubscriber<Image>(Image._TYPE)
      {
         @Override
         public void onNewMessage(Image image)
         {
            latestZED2Image.set(image);
         }
      };
      ros1Node.attachSubscriber(RosTools.ZED2_LEFT_EYE_VIDEO, zed2LeftEyeSubscriber);
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
                     Image zed2Image = latestZED2Image.get();
                     Point3D[] l515Points = null;
                     if (latestL515PointCloud2 != null && zed2Image != null)
                     {
                        ChannelBuffer l515NettyImageData = latestL515PointCloud2.getData();
                        ByteBuffer l515DataByteBuffer = l515NettyImageData.toByteBuffer();
                        int arrayOffset = l515NettyImageData.arrayOffset();
                        l515DataByteBuffer.position(arrayOffset);
                        ByteBuffer offsetByteBuffer = l515DataByteBuffer.slice();
                        BytePointer imageDataPointer = new BytePointer(offsetByteBuffer);

                        // l515 is 4 float 32s, X,Y,Z,RGB
//                        Point4fVector l515Point3fVector = new Point4fVector(imageDataPointer);

                        UnpackedPointCloud unpackPointsAndIntensities = RosPointCloudSubscriber.unpackPointsAndIntensities(latestL515PointCloud2);
                        l515Points = unpackPointsAndIntensities.getPoints();
                        for (int i = 0; i < l515Points.length; i++)
                        {
                           double x = l515Points[i].getX();
                           double y = l515Points[i].getY();
                           double z = l515Points[i].getZ();
                           l515Points[i].set(z, -x, -y); // flip space z forward, x right, to z up, x forward
                           l515Points[i].applyTransform(l515ToWorldTransform);
                        }

                        StereoVisionPointCloudMessage fusedPointCloudMessage = new StereoVisionPointCloudMessage();
//                        fusedPointCloudMessage.

                        PointCloudData pointCloudData = new PointCloudData(pointCloud2, 1600000, true);
                        //                     pointCloudData.getColors()
                        pointCloudData.applyTransform(ousterToWorldTransform);
                        LidarScanMessage lidarScanMessage = pointCloudData.toLidarScanMessage(null, l515Points);
                        lidarScanMessage.getLidarPosition().set(ousterPose.getPosition());
                        lidarScanMessage.getLidarOrientation().set(ousterPose.getOrientation());
                        lidarScanMessage.setSensorPoseConfidence(1.0);
                        ros2Helper.publish(PerceptionAPI.MULTISENSE_LIDAR_SCAN, lidarScanMessage);
                     }

                     // TODO: Publish partial compositions if certain things are not available
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
      new AtlasOusterL515ZED2FusedColoredROS1ToREABridge();
   }
}
