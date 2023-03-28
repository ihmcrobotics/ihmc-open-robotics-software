package us.ihmc.robotEnvironmentAwareness.communication.converters;

import perception_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.packets.LidarPointCloudCompression;
import us.ihmc.communication.packets.StereoPointCloudCompression;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

import java.util.List;

public class PointCloudMessageTools
{
   public static StereoVisionPointCloudMessage toStereoVisionPointCloudMessage(List<? extends Point3DReadOnly> pointCloud, Pose3DReadOnly sensorPose)
   {
      Point3D[] pointArray = new Point3D[pointCloud.size()];
      int[] colors = new int[pointCloud.size()];
      for (int i = 0; i < pointCloud.size(); i++)
      {
         Point3DReadOnly point3DReadOnly = pointCloud.get(i);
         pointArray[i] = new Point3D(point3DReadOnly);
         colors[i] = 0;
      }

      // TODO: make parameters to method
      StereoVisionPointCloudMessage message = StereoPointCloudCompression.compressPointCloud(System.nanoTime(), pointArray, colors, pointArray.length, 0.005, null);
      message.getSensorPosition().set(sensorPose.getPosition());
      message.getSensorOrientation().set(sensorPose.getOrientation());
      return message;
   }

   public static LidarScanMessage toLidarScanMessage(List<? extends Point3DReadOnly> scan, Pose3DReadOnly scanSensorPose)
   {
      return toLidarScanMessage(System.nanoTime(), scan, scanSensorPose);
   }

   public static LidarScanMessage toLidarScanMessage(long timestamp, List<? extends Point3DReadOnly> scan, Pose3DReadOnly scanSensorPose)
   {
      LidarScanMessage message = new LidarScanMessage();

      // TODO: apply filters
      LidarPointCloudCompression.compressPointCloud(scan.size(), message, (i, j) -> (float) scan.get(i).getElement(j));
      message.setRobotTimestamp(timestamp);
      message.setSensorPoseConfidence(1.0);
      message.setPointCloudConfidence(1.0);
      message.getLidarPosition().set(scanSensorPose.getPosition());
      message.getLidarOrientation().set(scanSensorPose.getOrientation());
      return message;
   }
}
