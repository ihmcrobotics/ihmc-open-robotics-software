package us.ihmc.atlas.multisenseBlobExperiments;

import us.ihmc.ihmcPerception.depthData.PointCloudDataReceiverInterface;
import us.ihmc.ihmcPerception.depthData.PointCloudSource;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Point3d;
import java.util.ArrayList;

public class BlobDetectionPointCloudReceiver implements PointCloudDataReceiverInterface
{
   @Override
   public void receivedPointCloudData(ReferenceFrame scanFrame, ReferenceFrame lidarFrame, long[] timestamps, ArrayList<Point3d> points,
         PointCloudSource... sources)
   {
      System.out.println(getClass().getSimpleName() + " - receiving point cloud");
   }
}
