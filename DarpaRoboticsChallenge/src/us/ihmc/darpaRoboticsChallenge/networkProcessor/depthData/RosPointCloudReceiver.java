package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import java.util.ArrayList;
import java.util.Arrays;

import javax.vecmath.Point3d;

import sensor_msgs.PointCloud2;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

public class RosPointCloudReceiver extends RosPointCloudSubscriber
{
   private final ReferenceFrame scanFrame;
   private final PointCloudDataReceiver pointCloudDataReceiver;

   public RosPointCloudReceiver(String rosTopic, RosMainNode rosMainNode, ReferenceFrame scanFrame,
         PointCloudDataReceiver pointCloudDataReceiver)
   {
      this.scanFrame = scanFrame;
      this.pointCloudDataReceiver = pointCloudDataReceiver;

      rosMainNode.attachSubscriber(rosTopic, this);

   }

	@Override
	public void onNewMessage(PointCloud2 pointCloud) 
	{
		UnpackedPointCloud pointCloudData = unpackPointsAndIntensities(pointCloud);
		Point3d[] points = pointCloudData.getPoints();
		ArrayList<Point3d> pointsAsArrayList = new ArrayList<Point3d>(Arrays.asList(points));
	    long[] timestamps = new long[points.length];
	    long time = pointCloud.getHeader().getStamp().totalNsecs();
	    Arrays.fill(timestamps, time);
	
		RosPointCloudReceiver.this.pointCloudDataReceiver.receivedPointCloudData(RosPointCloudReceiver.this.scanFrame, null, timestamps, pointsAsArrayList);
	}
}
