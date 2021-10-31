package us.ihmc.atlas.sensors;

import controller_msgs.msg.dds.LidarScanMessage;
import sensor_msgs.PointCloud2;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.behaviors.tools.yo.YoVariableServerHelper;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.compression.LZ4CompressionImplementation;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.Timer;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.tools.thread.Throttler;
import us.ihmc.tools.time.DurationStatisticPrinter;
import us.ihmc.tools.time.FrequencyStatisticPrinter;
import us.ihmc.utilities.ros.ROS1Helper;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber.UnpackedPointCloud;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

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
 * 22220 points @ 140 Hz
 * x, y, z - 32-bit floats
 * 12 bytes per point
 * 37 MB/s bandwidth
 */
public class AtlasOusterL515FusedROS1ToREABridge
{
   public static final int YO_VARIABLE_SERVER_PORT = 6039;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoVariableServerHelper yoVariableServerHelper = new YoVariableServerHelper(getClass(), registry, YO_VARIABLE_SERVER_PORT, 5.0);
   private final YoDouble outputFrequency = new YoDouble("outputFrequency", registry);
   private final YoBoolean publishOnlyWhenRobotIsWalking = new YoBoolean("publishOnlyWhenRobotIsWalking", registry);
   private final YoInteger pointsPerSegment = new YoInteger("pointsPerSegment", registry);
   private final YoInteger numberOfSegments = new YoInteger("numberOfSegments", registry);
   private volatile boolean running = true;
   private final ROS1Helper ros1Helper = new ROS1Helper("ousterl515_to_rea");
   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ousterl515_to_rea");
   private final AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT);
   private final CommunicationHelper ros2Helper = new CommunicationHelper(robotModel, ros2Node);
   private final ROS2SyncedRobotModel syncedRobot = ros2Helper.newSyncedRobot();
   private final RigidBodyTransform l515ToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform ousterToWorldTransform = new RigidBodyTransform();
   private final Throttler throttler = new Throttler();
   private final AtomicReference<PointCloud2> latestL515PointCloud = new AtomicReference<>();
   private final AtomicReference<PointCloud2> latestOusterPointCloud = new AtomicReference<>();
   private final DurationStatisticPrinter durationStatisticPrinter = new DurationStatisticPrinter();
   private final FrequencyStatisticPrinter ousterInput = new FrequencyStatisticPrinter();
   private final FrequencyStatisticPrinter frequencyStatisticPrinter = new FrequencyStatisticPrinter();
   private final ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor("OusterL515ToREABridge", true, 1);
   private final LZ4CompressionImplementation lz4Compressor = new LZ4CompressionImplementation();

   public AtlasOusterL515FusedROS1ToREABridge()
   {
      outputFrequency.set(10.0);
      publishOnlyWhenRobotIsWalking.set(false);
      pointsPerSegment.set(22220);
      numberOfSegments.set(14);

      // TODO: Add remote interface to set:
      // - Wait for robot pose
      // - rates, bundling
      // - which sensors to fuse
      // - topics

      ros1Helper.subscribeToPointCloud2ViaCallback(RosTools.L515_POINT_CLOUD, latestL515PointCloud::set);
      ros1Helper.subscribeToPointCloud2ViaCallback(RosTools.OUSTER_POINT_CLOUD, newValue ->
      {
         ousterInput.ping();
         latestOusterPointCloud.set(newValue);
      });

      yoVariableServerHelper.start();

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         LogTools.info("Shutting down...");
         running = false;
         yoVariableServerHelper.destroy();
         executor.destroy();
         ros1Helper.destroy();
         ros2Node.destroy();
         durationStatisticPrinter.destroy();
         frequencyStatisticPrinter.destroy();
      }, "IHMC-OusterROS1ToREABridgeShutdown"));

      while (running)
      {
         double outputPeriod = UnitConversions.hertzToSeconds(outputFrequency.getValue());
         double segmentPeriod = outputPeriod / numberOfSegments.getValue();
         if (throttler.run(outputPeriod))
         {
            syncedRobot.update();
            if (publishOnlyWhenRobotIsWalking.getValue() && syncedRobot.getDataReceptionTimerSnapshot().isRunning(3.0))
            {
               durationStatisticPrinter.before();

               FramePose3DReadOnly l515Pose = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getSteppingCameraFrame);
               l515Pose.get(l515ToWorldTransform);

               FramePose3DReadOnly ousterPose = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getOusterLidarFrame);
               ousterPose.get(ousterToWorldTransform);

               // These are uncompressed messages coming in
               PointCloud2 latestL515PointCloud2 = latestL515PointCloud.get();
               PointCloud2 latestOusterPointCloud2 = latestOusterPointCloud.get();

               // We are only publishing X, Y, Z points in world to start with

               Timer timer = new Timer();
               for (int i = 0; i < numberOfSegments.getValue(); i++)
               {
                  frequencyStatisticPrinter.ping();
                  timer.reset();

                  // Send one segment
                  LidarScanMessage lidarScanMessage = new LidarScanMessage();
                  lidarScanMessage.setRobotTimestamp(latestOusterPointCloud2.getHeader().getStamp().totalNsecs());
                  lidarScanMessage.setSensorPoseConfidence(1.0); // TODO: ??
                  lidarScanMessage.setNumberOfPoints(pointsPerSegment.getValue());



                  ros2Helper.publish(ROS2Tools.MULTISENSE_LIDAR_SCAN, lidarScanMessage);

                  timer.sleepUntilExpiration(segmentPeriod);
               }


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

               PointCloudData pointCloudData = new PointCloudData(latestOusterPointCloud2, 1600000, false);
               pointCloudData.applyTransform(ousterToWorldTransform);
               LidarScanMessage lidarScanMessage2 = pointCloudData.toLidarScanMessage(null, l515Points);
               lidarScanMessage2.getLidarPosition().set(ousterPose.getPosition());
               lidarScanMessage2.getLidarOrientation().set(ousterPose.getOrientation());
               lidarScanMessage2.setSensorPoseConfidence(1.0);
               ros2Helper.publish(ROS2Tools.MULTISENSE_LIDAR_SCAN, lidarScanMessage2);

               durationStatisticPrinter.after();
            }
         }
      }
   }

   private void addPointsToMessage(LidarScanMessage lidarScanMessage, PointCloud2 pointCloud2, int segmentIndex, int pointsPerSegment)
   {
      int numberOfPoints = pointCloud2.getWidth() * pointCloud2.getHeight();

      int startIndex = segmentIndex * pointsPerSegment;
      int endIndex = startIndex + pointsPerSegment;
      for (int i = startIndex; i < endIndex; i++)
      {
         if (i < numberOfPoints)
         {

         }
         else
         {
            // NaN point

         }
      }



   }

   public static void main(String[] args)
   {
      new AtlasOusterL515FusedROS1ToREABridge();
   }
}
