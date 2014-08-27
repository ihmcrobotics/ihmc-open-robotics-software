package us.ihmc.darpaRoboticsChallenge.ros;

import javax.media.j3d.Transform3D;
import javax.vecmath.AxisAngle4d;

import org.ros.message.Time;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotLidarParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.SpindleAnglePacket;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.SpindleAngleReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.utilities.lidar.polarLidar.LidarScan;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.net.LocalObjectCommunicator;
import us.ihmc.utilities.net.ObjectConsumer;
import us.ihmc.utilities.ros.RosLidarPublisher;
import us.ihmc.utilities.ros.RosMainNode;

public class RosSCSLidarPublisher implements ObjectConsumer<LidarScan>
{
   private final RosLidarPublisher[] lidarPublisher;
   private final RosMainNode rosMainNode;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final SDFFullRobotModel fullRobotModel;
   private final DRCRobotLidarParameters[] lidarParameters;
   private final SpindleAngleReceiver spindleAngleReceiver;
   private final RosTfPublisher tfPublisher;
   private final int nSensors;

   public RosSCSLidarPublisher(LocalObjectCommunicator scsCommunicator,
         RosMainNode rosMainNode,
         PPSTimestampOffsetProvider ppsTimestampOffsetProvider,
         SDFFullRobotModel fullRobotModel,
         DRCRobotLidarParameters[] lidarParameters,
         RosTfPublisher tfPublisher)
   {
      nSensors = lidarParameters.length;
      this.rosMainNode = rosMainNode;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.fullRobotModel = fullRobotModel;
      this.lidarParameters = lidarParameters;
      this.tfPublisher = tfPublisher;
      
      spindleAngleReceiver = new SpindleAngleReceiver(this.ppsTimestampOffsetProvider);
      
      lidarPublisher = new RosLidarPublisher[nSensors];
      for (int sensorId = 0; sensorId < nSensors; sensorId++)
      {
         lidarPublisher[sensorId] = new RosLidarPublisher(false);
         String rosTopic = lidarParameters[sensorId].getRosTopic();
         rosMainNode.attachPublisher(rosTopic, lidarPublisher[sensorId]);
      }

      scsCommunicator.attachListener(LidarScan.class, this);
      scsCommunicator.attachListener(SpindleAnglePacket.class, spindleAngleReceiver);
   }
   
   @Override
   public void consumeObject(LidarScan object)
   {
      if(rosMainNode.isStarted()){
         int sensorId = object.getSensorId();
         long timestamp = ppsTimestampOffsetProvider.adjustRobotTimeStampToRosClock(object.getScanParameters().getTimestamp());
         Time time = Time.fromNano(timestamp);
         String frameId = lidarParameters[sensorId].getEndFrameForRosTransform();
         publishLidarScanFrame(sensorId, timestamp);
         lidarPublisher[sensorId].publish(object, frameId, time);
      }
   }
   
   private void publishLidarScanFrame(int sensorId, long timestamp)
   {
      Transform3D spindleRotationTransform = getTransformFromJointAngle(sensorId, timestamp);
      if (spindleRotationTransform == null)
      {
         return;
      }
      
      String targetFrame = lidarParameters[sensorId].getPoseFrameForSdf();
      String sourceFrame = lidarParameters[sensorId].getBaseFrameForRosTransform();
      tfPublisher.publish(spindleRotationTransform, timestamp, sourceFrame, targetFrame);
   }
   
   private Transform3D getTransformFromJointAngle(int sensorId, long timestamp)
   {
      String sensorNameInSdf = lidarParameters[sensorId].getSensorNameInSdf();
      FrameVector spindleAxis = fullRobotModel.getLidarJointAxis(sensorNameInSdf);
      Transform3D lidarBaseFrameTransform = fullRobotModel.getLidarBaseFrame(sensorNameInSdf).getTransformToParent();
      double angle = spindleAngleReceiver.interpolate(timestamp);
      if (angle == Double.NaN)
      {
         return null;
      }
      AxisAngle4d spindleRotation = new AxisAngle4d(spindleAxis.getVector(), angle);
      Transform3D spindleRotationTransform = new Transform3D();
      spindleRotationTransform.set(spindleRotation);
      Transform3D transform = lidarBaseFrameTransform;
      transform.mul(spindleRotationTransform);
      return transform;
   }
}
