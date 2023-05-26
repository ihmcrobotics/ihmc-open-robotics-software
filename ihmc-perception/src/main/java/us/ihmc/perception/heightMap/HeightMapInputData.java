package us.ihmc.perception.heightMap;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.depthData.PointCloudData;
import us.ihmc.sensorProcessing.heightMap.HeightMapManager;

public class HeightMapInputData
{
   public PointCloudData pointCloud;
   public FramePose3D sensorPose;
   public Point3D gridCenter;
   public double verticalMeasurementVariance = HeightMapManager.defaultVariance;
}
