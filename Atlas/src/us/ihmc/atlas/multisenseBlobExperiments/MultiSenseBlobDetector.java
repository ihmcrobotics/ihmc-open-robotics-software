package us.ihmc.atlas.multisenseBlobExperiments;

import java.io.IOException;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.multisenseMocapExperiments.MultisenseHeadOnAStickManualTestNetworkProcessor;
import us.ihmc.avatar.drcRobot.DRCRobotModel;

public class MultiSenseBlobDetector
{
   public MultiSenseBlobDetector()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.HEAD_ON_A_STICK, false);

      try
      {
         new MultisenseHeadOnAStickManualTestNetworkProcessor(robotModel, "");
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public static void main(String[] args)
   {
      new MultiSenseBlobDetector();
   }
}











// RANDOM STUFF

//      AtlasSensorInformation sensorInformation = new AtlasSensorInformation(DRCRobotModel.RobotTarget.HEAD_ON_A_STICK);
//      DRCRobotLidarParameters lidarParameters = sensorInformation.getLidarParameters()[0];
//
//      PointCloudDataReceiverInterface pointCloudDataReceiver = new BlobDetectionPointCloudReceiver();
//
//      URI rosUri = NetworkParameters.getROSURI();
//      RosMainNode rosMainNode = new RosMainNode(rosUri, "atlas/sensorSuiteManager", true);
//
//      System.out.println("lidar sensor name: " + lidarParameters.getSensorNameInSdf());
//      System.out.println("lidar ros topic:   " + lidarParameters.getRosTopic());
//      System.out.println("ros main node:     " + rosMainNode);
//
//      new RosPointCloudReceiver(lidarParameters.getSensorNameInSdf(), lidarParameters.getRosTopic(), rosMainNode, ReferenceFrame.getWorldFrame(), pointCloudDataReceiver, PointCloudSource.NEARSCAN);
//      new RosPointCloudReceiver(lidarParameters.getSensorNameInSdf(), lidarParameters.getGroundCloudTopic(), rosMainNode, ReferenceFrame.getWorldFrame(), pointCloudDataReceiver, PointCloudSource.QUADTREE);





//      CollisionBoxProvider collisionBoxProvider = robotModel.getCollisionBoxProvider();
//      PPSTimestampOffsetProvider ppsTimestampOffsetProvider = new DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider();
//      DRCRobotJointMap jointMap = robotModel.getJointMap();
//      RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
//      PacketCommunicator sensorSuitePacketCommunicatorServer = PacketCommunicator
//            .createIntraprocessPacketCommunicator(NetworkPorts.SENSOR_MANAGER, new IHMCCommunicationKryoNetClassList());
//




//
//      PointCloudDataReceiver pointCloudDataReceiver = new PointCloudDataReceiver(robotModel, collisionBoxProvider, ppsTimestampOffsetProvider, jointMap,
//            robotConfigurationDataBuffer, sensorSuitePacketCommunicatorServer);
//
//      pointCloudDataReceiver.run();
