package us.ihmc.ihmcPerception.depthData;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;

public interface PointCloudDataReceiverInterface
{
   void receivedPointCloudData(ReferenceFrame scanFrame, ReferenceFrame lidarFrame, long[] timestamps, ArrayList<Point3D> points, PointCloudSource... sources);

}
