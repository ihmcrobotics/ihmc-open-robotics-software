package us.ihmc.atlas.multisenseBlobExperiments;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.depthData.PointCloudDataReceiverInterface;
import us.ihmc.perception.depthData.PointCloudSource;

public class BlobDetectionPointCloudReceiver implements PointCloudDataReceiverInterface
{
   @Override
   public void receivedPointCloudData(ReferenceFrame scanFrame, ReferenceFrame lidarFrame, long[] timestamps, ArrayList<Point3D> points,
         PointCloudSource... sources)
   {
      System.out.println(getClass().getSimpleName() + " - receiving point cloud");
   }
}
