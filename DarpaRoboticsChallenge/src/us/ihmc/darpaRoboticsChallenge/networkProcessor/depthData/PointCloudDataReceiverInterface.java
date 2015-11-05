package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import java.util.ArrayList;

import javax.vecmath.Point3d;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface PointCloudDataReceiverInterface
{
   void receivedPointCloudData(ReferenceFrame scanFrame, ReferenceFrame lidarFrame, long[] timestamps, ArrayList<Point3d> points, PointCloudSource... sources);

   ReferenceFrame getLidarFrame(String sensorNameInSdf);
}
