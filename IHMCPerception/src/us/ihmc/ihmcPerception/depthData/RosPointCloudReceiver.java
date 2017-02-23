package us.ihmc.ihmcPerception.depthData;

import java.util.ArrayList;
import java.util.Arrays;

import sensor_msgs.PointCloud2;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.time.Timer;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

public class RosPointCloudReceiver extends RosPointCloudSubscriber
{
   private final boolean DEBUG = false;
   private Timer timer;
   {
      if(DEBUG)
         timer = new Timer().start();
   }

   private final String rosTopic;
   private final ReferenceFrame cloudFrame;
   private final PointCloudDataReceiverInterface pointCloudDataReceiver;
   private final ReferenceFrame sensorframe;
   private final PointCloudSource[] pointCloudSource;

   public RosPointCloudReceiver(String rosTopic, RosMainNode rosMainNode, ReferenceFrame cloudFrame, ReferenceFrame sensorframe,
         PointCloudDataReceiverInterface pointCloudDataReceiver, PointCloudSource... pointCloudSource)
   {
      this.rosTopic = rosTopic;
      this.cloudFrame = cloudFrame;
      this.pointCloudDataReceiver = pointCloudDataReceiver;
      this.sensorframe = sensorframe;
      this.pointCloudSource = pointCloudSource;

      rosMainNode.attachSubscriber(rosTopic, this);
   }

   @Override
	public void onNewMessage(PointCloud2 pointCloud) 
	{
      if(DEBUG)
      {
         System.out.println(getClass().getSimpleName() + ": Received point cloud from " + rosTopic + " at rate " + 1.0 / timer.averageLap() + " FPS");
         timer.lap();
      }

      UnpackedPointCloud pointCloudData = unpackPointsAndIntensities(pointCloud);
		Point3D[] points = pointCloudData.getPoints();
		ArrayList<Point3D> pointsAsArrayList = new ArrayList<Point3D>(Arrays.asList(points));
	   long[] timestamps = new long[points.length];
	   long time = pointCloud.getHeader().getStamp().totalNsecs();
	   Arrays.fill(timestamps, time);
		pointCloudDataReceiver.receivedPointCloudData(cloudFrame, sensorframe, timestamps, pointsAsArrayList, pointCloudSource);
	}
}
