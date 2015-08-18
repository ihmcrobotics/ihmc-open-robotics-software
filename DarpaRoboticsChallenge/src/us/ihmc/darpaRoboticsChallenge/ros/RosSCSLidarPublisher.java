package us.ihmc.darpaRoboticsChallenge.ros;

import org.ros.message.Time;

import us.ihmc.SdfLoader.SDFBaseFullRobotModel;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.packets.sensing.SimulatedLidarScanPacket;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosLidarPublisher;

public class RosSCSLidarPublisher implements ObjectConsumer<SimulatedLidarScanPacket>
{
   private final RosLidarPublisher[] lidarPublisher;
   private final RosMainNode rosMainNode;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final DRCRobotLidarParameters[] lidarParameters;
   private final int nSensors;

   public RosSCSLidarPublisher(ObjectCommunicator localObjectCommunicator,
         RosMainNode rosMainNode,
         PPSTimestampOffsetProvider ppsTimestampOffsetProvider,
         SDFBaseFullRobotModel fullRobotModel,
         DRCRobotLidarParameters[] lidarParameters)
   {
      nSensors = lidarParameters.length;
      this.rosMainNode = rosMainNode;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.lidarParameters = lidarParameters;
      
      lidarPublisher = new RosLidarPublisher[nSensors];
      for (int sensorId = 0; sensorId < nSensors; sensorId++)
      {
         lidarPublisher[sensorId] = new RosLidarPublisher(false);
         String rosTopic = lidarParameters[sensorId].getRosTopic();
         rosMainNode.attachPublisher(rosTopic, lidarPublisher[sensorId]);
      }

      localObjectCommunicator.attachListener(SimulatedLidarScanPacket.class, this);
   }
   
   @Override
   public void consumeObject(SimulatedLidarScanPacket simLidarScan)
   {
      if(rosMainNode.isStarted()){
         int sensorId = simLidarScan.getSensorId();
         long timestamp = ppsTimestampOffsetProvider.adjustRobotTimeStampToRosClock(simLidarScan.getLidarScanParameters().getTimestamp());
         Time time = Time.fromNano(timestamp);
         String frameId = lidarParameters[sensorId].getEndFrameForRosTransform();
         lidarPublisher[sensorId].publish(simLidarScan, frameId, time);
      }
   }
   
}
