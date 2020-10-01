package us.ihmc.humanoidBehaviors.tools.perception;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudCompression;

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

      PointCloudCompression.compressPointCloud(System.nanoTime(), pointArray, colors, pointArray.length, 0.005, null); // TODO: make parameters to method

      StereoVisionPointCloudMessage message = new StereoVisionPointCloudMessage();
      message.getSensorPosition().set(sensorPose.getPosition());
      message.getSensorOrientation().set(sensorPose.getOrientation());
      return message;
   }
}
