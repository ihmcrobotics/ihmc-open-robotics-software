package us.ihmc.avatar.ros;

import org.ros.message.Time;

import perception_msgs.msg.dds.SimulatedLidarScanPacket;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosLidarPublisher;

public class RosSCSLidarPublisher implements ObjectConsumer<SimulatedLidarScanPacket>
{
   private final RosLidarPublisher[] lidarPublisher;
   private final RosMainNode rosMainNode;
   private final RobotROSClockCalculator rosClockCalculator;
   private final AvatarRobotLidarParameters[] lidarParameters;
   private final int nSensors;

   public RosSCSLidarPublisher(ObjectCommunicator localObjectCommunicator, RosMainNode rosMainNode, RobotROSClockCalculator rosClockCalculator,
                               FullRobotModel fullRobotModel, AvatarRobotLidarParameters[] lidarParameters)
   {
      nSensors = lidarParameters.length;
      this.rosMainNode = rosMainNode;
      this.rosClockCalculator = rosClockCalculator;
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
      if (rosMainNode.isStarted())
      {
         int sensorId = simLidarScan.getSensorId();
         long timestamp = rosClockCalculator.computeROSTime(simLidarScan.getLidarScanParameters().getTimestamp(), simLidarScan.getLidarScanParameters().getTimestamp());
         Time time = Time.fromNano(timestamp);
         String frameId = lidarParameters[sensorId].getEndFrameForRosTransform();
         lidarPublisher[sensorId].publish(simLidarScan, frameId, time);
      }
   }

}
