package us.ihmc.darpaRoboticsChallenge.ros;

import javax.vecmath.AxisAngle4d;

import org.ros.message.Time;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.sensing.LidarScanPacket;
import us.ihmc.communication.packets.sensing.SpindleAnglePacket;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.SpindleAngleReceiver;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosLidarPublisher;

public class RosSCSLidarPublisher implements PacketConsumer<LidarScanPacket>
{
   private final RosLidarPublisher[] lidarPublisher;
   private final RosMainNode rosMainNode;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final SDFFullRobotModel fullRobotModel;
   private final DRCRobotLidarParameters[] lidarParameters;
   private final SpindleAngleReceiver spindleAngleReceiver;
   private final RosTfPublisher tfPublisher;
   private final int nSensors;

   public RosSCSLidarPublisher(PacketCommunicator scsCommunicator,
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

      scsCommunicator.attachListener(LidarScanPacket.class, this);
      scsCommunicator.attachListener(SpindleAnglePacket.class, spindleAngleReceiver);
   }
   
   @Override
   public void receivedPacket(LidarScanPacket object)
   {
      if(rosMainNode.isStarted()){
         int sensorId = object.getSensorId();
         long timestamp = ppsTimestampOffsetProvider.adjustRobotTimeStampToRosClock(object.getLidarScanParameters().getTimestamp());
         Time time = Time.fromNano(timestamp);
         String frameId = lidarParameters[sensorId].getEndFrameForRosTransform();
         publishLidarScanFrame(sensorId, timestamp);
         lidarPublisher[sensorId].publish(object, frameId, time);
      }
   }
   
   private void publishLidarScanFrame(int sensorId, long timestamp)
   {
      RigidBodyTransform spindleRotationTransform = getTransformFromJointAngle(sensorId, timestamp);
      if (spindleRotationTransform == null)
      {
         return;
      }
      
      String targetFrame = lidarParameters[sensorId].getPoseFrameForSdf();
      String sourceFrame = lidarParameters[sensorId].getBaseFrameForRosTransform();
      tfPublisher.publish(spindleRotationTransform, timestamp, sourceFrame, targetFrame);
   }
   
   private RigidBodyTransform getTransformFromJointAngle(int sensorId, long timestamp)
   {
      String sensorNameInSdf = lidarParameters[sensorId].getSensorNameInSdf();
      FrameVector spindleAxis = fullRobotModel.getLidarJointAxis(sensorNameInSdf);
      RigidBodyTransform lidarBaseFrameTransform = fullRobotModel.getLidarBaseFrame(sensorNameInSdf).getTransformToParent();
      double angle = spindleAngleReceiver.interpolate(timestamp);
      if (angle == Double.NaN)
      {
         return null;
      }
      AxisAngle4d spindleRotation = new AxisAngle4d(spindleAxis.getVector(), angle);
      RigidBodyTransform spindleRotationTransform = new RigidBodyTransform();
      spindleRotationTransform.setRotationAndZeroTranslation(spindleRotation);
      RigidBodyTransform transform = lidarBaseFrameTransform;
      transform.multiply(spindleRotationTransform);
      return transform;
   }
}
