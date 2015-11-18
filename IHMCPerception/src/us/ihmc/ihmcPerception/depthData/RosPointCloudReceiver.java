package us.ihmc.ihmcPerception.depthData;

import sensor_msgs.PointCloud2;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.parameters.DRCRobotPointCloudParameters;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

import javax.vecmath.Point3d;
import java.util.ArrayList;
import java.util.Arrays;

public class RosPointCloudReceiver extends RosPointCloudSubscriber
{
   private final ReferenceFrame cloudFrame;
   private final PointCloudDataReceiverInterface pointCloudDataReceiver;
   private final ReferenceFrame sensorframe;
   private final PointCloudSource[] pointCloudSource;

   public RosPointCloudReceiver(String sensorNameInSdf, String rosTopic, RosMainNode rosMainNode, ReferenceFrame cloudFrame,
         PointCloudDataReceiverInterface pointCloudDataReceiver, PointCloudSource... pointCloudSource)
   {
      this(rosTopic, rosMainNode, cloudFrame, pointCloudDataReceiver.getLidarFrame(sensorNameInSdf), pointCloudDataReceiver, pointCloudSource);
   }

   public RosPointCloudReceiver(DRCRobotPointCloudParameters pointCloudParameters, RosMainNode rosMainNode, ReferenceFrame cloudFrame,
         PointCloudDataReceiverInterface pointCloudDataReceiver, PointCloudSource... pointCloudSource)
   {
      this(pointCloudParameters.getRosTopic(), rosMainNode, cloudFrame, pointCloudDataReceiver.getLidarFrame(pointCloudParameters.getSensorNameInSdf()), pointCloudDataReceiver, pointCloudSource);
   }

   public RosPointCloudReceiver(String rosTopic, RosMainNode rosMainNode, ReferenceFrame cloudFrame, ReferenceFrame sensorframe,
         PointCloudDataReceiverInterface pointCloudDataReceiver, PointCloudSource... pointCloudSource)
   {
      this.cloudFrame = cloudFrame;
      this.pointCloudDataReceiver = pointCloudDataReceiver;
      this.sensorframe = sensorframe;
      this.pointCloudSource = pointCloudSource;

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
		pointCloudDataReceiver.receivedPointCloudData(cloudFrame, sensorframe, timestamps, pointsAsArrayList, pointCloudSource);
	}
}
